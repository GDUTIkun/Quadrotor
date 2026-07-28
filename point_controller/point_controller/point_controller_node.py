"""ROS 2 node that drives the robot toward a single map-frame goal pose."""

from __future__ import annotations

from math import atan2

from geometry_msgs.msg import PoseStamped, Twist
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from point_controller.control import ControllerConfig, GoalPose, RobotPose, compute_command


def quaternion_to_yaw(q) -> float:
    return atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class PointControllerNode(Node):
    def __init__(self) -> None:
        super().__init__('point_controller_node')

        self.declare_parameter('global_frame_id', 'map')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('xy_tolerance_m', 0.05)
        self.declare_parameter('yaw_tolerance_rad', 0.0872665)
        self.declare_parameter('heading_tolerance_rad', 0.15)
        self.declare_parameter('k_v', 0.8)
        self.declare_parameter('k_w', 1.8)
        self.declare_parameter('v_max_m_s', 0.25)
        self.declare_parameter('w_max_rad_s', 0.8)
        self.declare_parameter('v_min_m_s', 0.03)
        self.declare_parameter('w_min_rad_s', 0.08)

        self.global_frame_id = self.get_parameter('global_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.config = ControllerConfig(
            xy_tolerance_m=float(self.get_parameter('xy_tolerance_m').value),
            yaw_tolerance_rad=float(self.get_parameter('yaw_tolerance_rad').value),
            heading_tolerance_rad=float(self.get_parameter('heading_tolerance_rad').value),
            k_v=float(self.get_parameter('k_v').value),
            k_w=float(self.get_parameter('k_w').value),
            v_max_m_s=float(self.get_parameter('v_max_m_s').value),
            w_max_rad_s=float(self.get_parameter('w_max_rad_s').value),
            v_min_m_s=float(self.get_parameter('v_min_m_s').value),
            w_min_rad_s=float(self.get_parameter('w_min_rad_s').value),
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.goal = None
        self.last_phase = ''

        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.on_goal_pose, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/point_controller/status', 10)

        control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.create_timer(max(0.01, 1.0 / control_rate_hz), self.control_step)

    def on_goal_pose(self, msg: PoseStamped) -> None:
        frame_id = msg.header.frame_id or self.global_frame_id
        if frame_id != self.global_frame_id:
            self.get_logger().warn(
                f'Ignoring goal in frame "{frame_id}"; expected "{self.global_frame_id}"'
            )
            return

        self.goal = GoalPose(
            x=msg.pose.position.x,
            y=msg.pose.position.y,
            yaw=quaternion_to_yaw(msg.pose.orientation),
        )
        self.last_phase = ''
        self.get_logger().info(
            f'Accepted goal x={self.goal.x:.3f}, y={self.goal.y:.3f}, yaw={self.goal.yaw:.3f}'
        )

    def control_step(self) -> None:
        if self.goal is None:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame_id,
                self.base_frame_id,
                Time(),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f'Cannot lookup {self.global_frame_id}->{self.base_frame_id}: {exc}',
                throttle_duration_sec=1.0,
            )
            self.publish_stop()
            return

        robot = RobotPose(
            x=transform.transform.translation.x,
            y=transform.transform.translation.y,
            yaw=quaternion_to_yaw(transform.transform.rotation),
        )
        result = compute_command(robot, self.goal, self.config)

        if result.phase != self.last_phase:
            self.last_phase = result.phase
            self.get_logger().info(
                f'Point controller phase={result.phase}, '
                f'distance={result.distance_error:.3f}, yaw_error={result.yaw_error:.3f}'
            )

        twist = Twist()
        twist.linear.x = result.v
        twist.angular.z = result.w
        self.cmd_pub.publish(twist)

        status = String()
        status.data = (
            f'phase={result.phase} distance={result.distance_error:.3f} '
            f'yaw_error={result.yaw_error:.3f} v={result.v:.3f} w={result.w:.3f}'
        )
        self.status_pub.publish(status)

        if result.reached:
            self.goal = None

    def publish_stop(self) -> None:
        self.cmd_pub.publish(Twist())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PointControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
