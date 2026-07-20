#!/usr/bin/env python3

import argparse
import csv
import math
import os
import signal
from datetime import datetime
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import PositionTarget
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


class SimpleSetpointRecorder(Node):
    def __init__(self, args):
        super().__init__("simple_setpoint_recorder")
        self.args = args
        self.finished = False
        self.clock_start = self.get_clock().now()
        self.time_zero = None

        self.position_setpoint_samples = []
        self.actual_position_samples = []
        self.velocity_setpoint_samples = []
        self.actual_velocity_samples = []

        self.last_position_setpoint = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=30,
        )

        self.create_subscription(
            PositionTarget,
            args.position_setpoint_topic,
            self.position_setpoint_callback,
            qos,
        )
        self.create_subscription(
            PoseStamped,
            args.actual_position_topic,
            self.actual_position_callback,
            qos,
        )
        self.create_subscription(
            TwistStamped,
            args.actual_velocity_topic,
            self.actual_velocity_callback,
            qos,
        )

        self.timer = self.create_timer(0.2, self.timer_callback)
        self.get_logger().info(
            f"Recording {args.duration:.1f}s to {args.output_dir}: "
            f"pos_sp={args.position_setpoint_topic}, "
            f"pos_actual={args.actual_position_topic}, "
            f"vel_actual={args.actual_velocity_topic}"
        )

    def msg_time(self, msg):
        stamp = msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            t = self.get_clock().now().nanoseconds * 1e-9
        else:
            t = stamp.sec + stamp.nanosec * 1e-9

        if self.time_zero is None:
            self.time_zero = t

        return t - self.time_zero

    def position_setpoint_callback(self, msg):
        t = self.msg_time(msg)
        x = msg.position.x
        y = msg.position.y
        z = msg.position.z
        self.position_setpoint_samples.append((t, x, y, z))

        if self.last_position_setpoint is not None:
            last_t, last_x, last_y, last_z = self.last_position_setpoint
            dt = t - last_t
            if dt > 1e-4:
                vx = (x - last_x) / dt
                vy = (y - last_y) / dt
                vz = (z - last_z) / dt
                speed = math.sqrt(vx * vx + vy * vy + vz * vz)
                if speed <= self.args.max_derived_speed:
                    self.velocity_setpoint_samples.append((t, vx, vy, vz))

        self.last_position_setpoint = (t, x, y, z)

    def actual_position_callback(self, msg):
        t = self.msg_time(msg)
        p = msg.pose.position
        self.actual_position_samples.append((t, p.x, p.y, p.z))

    def actual_velocity_callback(self, msg):
        t = self.msg_time(msg)
        v = msg.twist.linear
        self.actual_velocity_samples.append((t, v.x, v.y, v.z))

    def timer_callback(self):
        elapsed = (self.get_clock().now() - self.clock_start).nanoseconds * 1e-9
        if elapsed >= self.args.duration:
            self.finished = True
            self.timer.cancel()

        if self.time_zero is None:
            self.get_logger().warn(
                "Waiting for first sample...",
                throttle_duration_sec=2.0,
            )

    def save(self):
        output_dir = Path(os.path.expanduser(self.args.output_dir))
        output_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = output_dir / f"simple_setpoint_actual_{stamp}.csv"
        png_path = output_dir / f"simple_setpoint_actual_{stamp}.png"

        with csv_path.open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["source", "time_s", "x_or_vx", "y_or_vy", "z_or_vz"])
            for sample in self.position_setpoint_samples:
                writer.writerow(["position_setpoint", *sample])
            for sample in self.actual_position_samples:
                writer.writerow(["actual_position", *sample])
            for sample in self.velocity_setpoint_samples:
                writer.writerow(["derived_velocity_setpoint", *sample])
            for sample in self.actual_velocity_samples:
                writer.writerow(["actual_velocity", *sample])

        self.plot(png_path)
        self.get_logger().info(f"Saved CSV: {csv_path}")
        self.get_logger().info(f"Saved plot: {png_path}")

    def plot(self, png_path):
        fig, axes = plt.subplots(3, 2, figsize=(14, 10), sharex="col")
        names = ["X", "Y", "Z"]

        for axis_index in range(3):
            value_index = axis_index + 1
            pos_axis = axes[axis_index][0]
            vel_axis = axes[axis_index][1]

            if self.position_setpoint_samples:
                pos_axis.plot(
                    [s[0] for s in self.position_setpoint_samples],
                    [s[value_index] for s in self.position_setpoint_samples],
                    label="Position setpoint",
                    linewidth=1.6,
                )
            if self.actual_position_samples:
                pos_axis.plot(
                    [s[0] for s in self.actual_position_samples],
                    [s[value_index] for s in self.actual_position_samples],
                    label="Actual position",
                    linewidth=1.2,
                )

            if self.velocity_setpoint_samples:
                vel_axis.plot(
                    [s[0] for s in self.velocity_setpoint_samples],
                    [s[value_index] for s in self.velocity_setpoint_samples],
                    label="Derived velocity setpoint",
                    linewidth=1.4,
                )
            if self.actual_velocity_samples:
                vel_axis.plot(
                    [s[0] for s in self.actual_velocity_samples],
                    [s[value_index] for s in self.actual_velocity_samples],
                    label="Actual velocity",
                    linewidth=1.2,
                )

            pos_axis.set_ylabel(f"{names[axis_index]} [m]")
            vel_axis.set_ylabel(f"V{names[axis_index]} [m/s]")
            pos_axis.grid(True, alpha=0.35)
            vel_axis.grid(True, alpha=0.35)
            pos_axis.legend(loc="best")
            vel_axis.legend(loc="best")

        axes[-1][0].set_xlabel("Time [s]")
        axes[-1][1].set_xlabel("Time [s]")
        fig.suptitle(
            "simple_offboard_node setpoint vs actual\n"
            "velocity setpoint is derived from the published position setpoint"
        )
        fig.tight_layout()
        fig.savefig(png_path, dpi=160)
        plt.close(fig)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Record and plot simple_offboard_node position/velocity setpoint and actual response."
    )
    parser.add_argument(
        "-d",
        "--duration",
        type=float,
        default=50.0,
        help="Recording duration in seconds.",
    )
    parser.add_argument(
        "--position-setpoint-topic",
        default="/mavros/setpoint_raw/local",
        help="PositionTarget topic published by simple_offboard_node.",
    )
    parser.add_argument(
        "--actual-position-topic",
        default="/mavros/local_position/pose",
        help="Actual local position PoseStamped topic.",
    )
    parser.add_argument(
        "--actual-velocity-topic",
        default="/mavros/local_position/velocity_local",
        help="Actual local velocity TwistStamped topic.",
    )
    parser.add_argument(
        "--max-derived-speed",
        type=float,
        default=20.0,
        help="Drop derived setpoint velocity samples above this speed to ignore timestamp glitches.",
    )
    parser.add_argument(
        "-o",
        "--output-dir",
        default="/home/t/flight_test/bags",
        help="Directory for CSV and PNG output.",
    )
    return parser.parse_known_args()


def main():
    args, ros_args = parse_args()
    rclpy.init(args=ros_args)
    node = SimpleSetpointRecorder(args)

    interrupted = False

    def handle_sigint(signum, frame):
        nonlocal interrupted
        interrupted = True
        node.finished = True

    signal.signal(signal.SIGINT, handle_sigint)

    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if (
            node.position_setpoint_samples
            or node.actual_position_samples
            or node.velocity_setpoint_samples
            or node.actual_velocity_samples
        ):
            node.save()
        else:
            node.get_logger().warn("No samples received; nothing saved.")

        if interrupted:
            node.get_logger().info("Stopped by Ctrl-C.")

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
