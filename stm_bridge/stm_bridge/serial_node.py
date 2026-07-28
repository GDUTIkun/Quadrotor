"""ROS 2 node for the STM32 serial bridge."""

from __future__ import annotations

from math import cos, pi, sin
from typing import Optional

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster

try:
    import serial
    from serial import SerialException
except ImportError:  # pragma: no cover - reported at runtime in ROS.
    serial = None
    SerialException = Exception

from stm_bridge.protocol import (
    MSG_CMD_VEL,
    MSG_IMU,
    MSG_STATUS,
    MSG_WHEEL_ODOM,
    Frame,
    FrameParser,
    build_frame,
    decode_imu,
    decode_status,
    decode_wheel_odom,
    encode_cmd_vel,
)


GRAVITY = 9.80665


def as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


def yaw_to_quaternion(yaw: float):
    q = type('QuaternionTuple', (), {})()
    q.x = 0.0
    q.y = 0.0
    q.z = sin(yaw * 0.5)
    q.w = cos(yaw * 0.5)
    return q


def euler_to_quaternion(roll: float, pitch: float, yaw: float):
    cr = cos(roll * 0.5)
    sr = sin(roll * 0.5)
    cp = cos(pitch * 0.5)
    sp = sin(pitch * 0.5)
    cy = cos(yaw * 0.5)
    sy = sin(yaw * 0.5)
    q = type('QuaternionTuple', (), {})()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class StmBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__('stm_bridge_node')

        self.declare_parameter('port', '/dev/ttyS0')
        self.declare_parameter('baudrate', 576000)
        self.declare_parameter('cmd_rate_hz', 50.0)
        self.declare_parameter('cmd_timeout_s', 0.3)
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('publish_wheel_odom', False)
        self.declare_parameter('publish_odom_tf', False)

        self.port = self.get_parameter('port').value
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.cmd_rate_hz = float(self.get_parameter('cmd_rate_hz').value)
        self.cmd_timeout_s = float(self.get_parameter('cmd_timeout_s').value)
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.publish_wheel_odom_enabled = as_bool(self.get_parameter('publish_wheel_odom').value)
        self.publish_odom_tf = as_bool(self.get_parameter('publish_odom_tf').value)

        self.parser = FrameParser()
        self.serial_port = None
        self.seq = 0
        self.tx_frames = 0
        self.rx_frames = 0
        self.last_cmd = Twist()
        self.has_cmd = False
        self.last_cmd_time = self.get_clock().now()
        self.last_status_stamp_ms = 0

        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)
        self.imu_pub = self.create_publisher(Imu, '/track2vision/imu/data_valid', 20)
        self.odom_pub = (
            self.create_publisher(Odometry, '/odom/wheel', 20)
            if self.publish_wheel_odom_enabled
            else None
        )
        self.status_pub = self.create_publisher(DiagnosticArray, '/stm/status', 10)
        self.tf_broadcaster: Optional[TransformBroadcaster] = (
            TransformBroadcaster(self)
            if self.publish_wheel_odom_enabled and self.publish_odom_tf
            else None
        )

        self.open_serial()
        self.create_timer(max(0.001, 1.0 / self.cmd_rate_hz), self.send_cmd_vel)
        self.create_timer(0.002, self.read_serial)
        self.create_timer(1.0, self.reconnect_if_needed)
        self.get_logger().info(
            'STM protocol v1.0: 0x01 CMD_VEL, 0x81 IMU, 0x82 WHEEL_ODOM, 0x83 STATUS'
        )

    def open_serial(self) -> None:
        if serial is None:
            self.get_logger().error('python3-serial is not installed; install python3-serial')
            return
        if self.serial_port and self.serial_port.is_open:
            return
        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0,
                write_timeout=0,
            )
            self.get_logger().info(f'Opened STM serial port {self.port} at {self.baudrate}')
        except SerialException as exc:
            self.serial_port = None
            self.get_logger().warn(f'Cannot open STM serial port {self.port}: {exc}')

    def reconnect_if_needed(self) -> None:
        if self.serial_port is None or not self.serial_port.is_open:
            self.open_serial()

    def on_cmd_vel(self, msg: Twist) -> None:
        self.last_cmd = msg
        self.has_cmd = True
        self.last_cmd_time = self.get_clock().now()

    def send_cmd_vel(self) -> None:
        if self.serial_port is None or not self.serial_port.is_open:
            return

        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        enabled = self.has_cmd and elapsed <= self.cmd_timeout_s
        v = self.last_cmd.linear.x if enabled else 0.0
        w = self.last_cmd.angular.z if enabled else 0.0
        payload = encode_cmd_vel(v, w, enabled)
        packet = build_frame(MSG_CMD_VEL, self.seq, payload)
        self.seq = (self.seq + 1) & 0xFF

        try:
            self.serial_port.write(packet)
            self.tx_frames += 1
        except SerialException as exc:
            self.get_logger().warn(f'STM serial write failed: {exc}')
            self.close_serial()

    def read_serial(self) -> None:
        if self.serial_port is None or not self.serial_port.is_open:
            return
        try:
            waiting = self.serial_port.in_waiting
            if waiting <= 0:
                return
            data = self.serial_port.read(min(waiting, 512))
        except SerialException as exc:
            self.get_logger().warn(f'STM serial read failed: {exc}')
            self.close_serial()
            return

        for frame in self.parser.feed(data):
            self.rx_frames += 1
            self.handle_frame(frame)

    def close_serial(self) -> None:
        if self.serial_port is None:
            return
        try:
            self.serial_port.close()
        except SerialException:
            pass
        self.serial_port = None

    def handle_frame(self, frame: Frame) -> None:
        try:
            if frame.msg_id == MSG_IMU:
                self.publish_imu(frame.payload)
            elif frame.msg_id == MSG_WHEEL_ODOM:
                self.publish_wheel_odom(frame.payload)
            elif frame.msg_id == MSG_STATUS:
                self.publish_status(frame.payload)
            else:
                self.get_logger().debug(f'Ignored unknown STM msg_id=0x{frame.msg_id:02X}')
        except ValueError as exc:
            self.get_logger().warn(f'Dropped malformed STM frame 0x{frame.msg_id:02X}: {exc}')

    def publish_imu(self, payload: bytes) -> None:
        data = decode_imu(payload)
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame_id

        roll = data.roll_cdeg / 100.0 * pi / 180.0
        pitch = data.pitch_cdeg / 100.0 * pi / 180.0
        yaw = data.yaw_cdeg / 100.0 * pi / 180.0
        q = euler_to_quaternion(roll, pitch, yaw)
        msg.orientation.x = q.x
        msg.orientation.y = q.y
        msg.orientation.z = q.z
        msg.orientation.w = q.w

        msg.angular_velocity.x = data.gx_mdps * pi / (180.0 * 1000.0)
        msg.angular_velocity.y = data.gy_mdps * pi / (180.0 * 1000.0)
        msg.angular_velocity.z = data.gz_mdps * pi / (180.0 * 1000.0)
        msg.linear_acceleration.x = data.ax_mg * GRAVITY / 1000.0
        msg.linear_acceleration.y = data.ay_mg * GRAVITY / 1000.0
        msg.linear_acceleration.z = data.az_mg * GRAVITY / 1000.0

        msg.orientation_covariance = [0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.01]
        msg.angular_velocity_covariance = [0.0004, 0.0, 0.0, 0.0, 0.0004, 0.0, 0.0, 0.0, 0.0004]
        msg.linear_acceleration_covariance = [0.04, 0.0, 0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 0.04]
        self.imu_pub.publish(msg)

    def publish_wheel_odom(self, payload: bytes) -> None:
        if not self.publish_wheel_odom_enabled or self.odom_pub is None:
            return

        data = decode_wheel_odom(payload)
        yaw = data.yaw_mrad / 1000.0
        q = yaw_to_quaternion(yaw)

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.odom_frame_id
        msg.child_frame_id = self.base_frame_id
        msg.pose.pose.position.x = data.x_mm / 1000.0
        msg.pose.pose.position.y = data.y_mm / 1000.0
        msg.pose.pose.orientation.x = q.x
        msg.pose.pose.orientation.y = q.y
        msg.pose.pose.orientation.z = q.z
        msg.pose.pose.orientation.w = q.w
        msg.twist.twist.linear.x = data.vx_mm_s / 1000.0
        msg.twist.twist.angular.z = data.wz_mrad_s / 1000.0
        msg.pose.covariance[0] = 0.01
        msg.pose.covariance[7] = 0.01
        msg.pose.covariance[35] = 0.05
        msg.twist.covariance[0] = 0.02
        msg.twist.covariance[35] = 0.05
        self.odom_pub.publish(msg)

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header = msg.header
            tf.child_frame_id = msg.child_frame_id
            tf.transform.translation.x = msg.pose.pose.position.x
            tf.transform.translation.y = msg.pose.pose.position.y
            tf.transform.rotation = msg.pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf)

    def publish_status(self, payload: bytes) -> None:
        data = decode_status(payload)
        self.last_status_stamp_ms = data.stamp_ms
        status = DiagnosticStatus()
        status.name = 'stm_bridge'
        status.hardware_id = self.port
        status.level = DiagnosticStatus.ERROR if data.error_flags else DiagnosticStatus.OK
        status.message = 'error flags set' if data.error_flags else 'ok'
        status.values = [
            KeyValue(key='voltage_mv', value=str(data.voltage_mv)),
            KeyValue(key='current_ma', value=str(data.current_ma)),
            KeyValue(key='state', value=str(data.state)),
            KeyValue(key='error_flags', value=f'0x{data.error_flags:04X}'),
            KeyValue(key='stamp_ms', value=str(data.stamp_ms)),
            KeyValue(key='rx_frames', value=str(self.rx_frames)),
            KeyValue(key='tx_frames', value=str(self.tx_frames)),
            KeyValue(key='crc_errors', value=str(self.parser.crc_errors)),
            KeyValue(key='dropped_bytes', value=str(self.parser.dropped_bytes)),
        ]

        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status.append(status)
        self.status_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StmBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.close_serial()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
