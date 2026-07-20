#!/usr/bin/env python3

import argparse
import csv
import os
import signal
from datetime import datetime
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class VelocityRecorder(Node):
    def __init__(self, args):
        super().__init__("pd_velocity_recorder")
        self.args = args
        self.start_time = None
        self.finished = False
        self.setpoint_samples = []
        self.actual_samples = []

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )

        self.create_subscription(
            TwistStamped,
            args.setpoint_topic,
            self.setpoint_callback,
            qos,
        )
        self.create_subscription(
            TwistStamped,
            args.actual_topic,
            self.actual_callback,
            qos,
        )

        self.timer = self.create_timer(0.2, self.timer_callback)
        self.get_logger().info(
            f"Recording {args.duration:.1f}s: setpoint={args.setpoint_topic}, "
            f"actual={args.actual_topic}"
        )

    def sample_time(self, msg):
        stamp = msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            now = self.get_clock().now().nanoseconds * 1e-9
        else:
            now = stamp.sec + stamp.nanosec * 1e-9

        if self.start_time is None:
            self.start_time = now

        return now - self.start_time

    def setpoint_callback(self, msg):
        t = self.sample_time(msg)
        self.setpoint_samples.append((
            t,
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z,
        ))

    def actual_callback(self, msg):
        t = self.sample_time(msg)
        self.actual_samples.append((
            t,
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z,
        ))

    def timer_callback(self):
        if self.start_time is None:
            self.get_logger().warn(
                "Waiting for first velocity sample...",
                throttle_duration_sec=2.0,
            )
            return

        elapsed = self.get_clock().now().nanoseconds * 1e-9 - self.start_time
        if elapsed >= self.args.duration:
            self.finished = True
            self.timer.cancel()

    def save(self):
        output_dir = Path(os.path.expanduser(self.args.output_dir))
        output_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = output_dir / f"pd_velocity_{stamp}.csv"
        png_path = output_dir / f"pd_velocity_{stamp}.png"

        with csv_path.open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["source", "time_s", "vx", "vy", "vz"])
            for sample in self.setpoint_samples:
                writer.writerow(["pd_setpoint", *sample])
            for sample in self.actual_samples:
                writer.writerow(["actual_velocity", *sample])

        self.plot(png_path)
        self.get_logger().info(f"Saved CSV: {csv_path}")
        self.get_logger().info(f"Saved plot: {png_path}")

    def plot(self, png_path):
        fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
        names = ["X", "Y", "Z"]

        for axis_index, axis in enumerate(axes):
            value_index = axis_index + 1
            if self.setpoint_samples:
                axis.plot(
                    [s[0] for s in self.setpoint_samples],
                    [s[value_index] for s in self.setpoint_samples],
                    label="PD velocity setpoint",
                    linewidth=1.6,
                )
            if self.actual_samples:
                axis.plot(
                    [s[0] for s in self.actual_samples],
                    [s[value_index] for s in self.actual_samples],
                    label="Actual velocity",
                    linewidth=1.2,
                )

            axis.set_ylabel(f"V{names[axis_index]} [m/s]")
            axis.grid(True, alpha=0.35)
            axis.legend(loc="best")

        axes[-1].set_xlabel("Time [s]")
        fig.suptitle(
            f"PD velocity setpoint vs actual velocity\n"
            f"setpoint: {self.args.setpoint_topic} | actual: {self.args.actual_topic}"
        )
        fig.tight_layout()
        fig.savefig(png_path, dpi=160)
        plt.close(fig)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Record PD velocity setpoint and MAVROS actual velocity, then plot them."
    )
    parser.add_argument(
        "-d",
        "--duration",
        type=float,
        default=30.0,
        help="Recording duration in seconds.",
    )
    parser.add_argument(
        "--setpoint-topic",
        default="/pd_shadow/velocity_setpoint",
        help="TwistStamped topic produced by pd_hover_node shadow output.",
    )
    parser.add_argument(
        "--actual-topic",
        default="/mavros/local_position/velocity_local",
        help="Actual TwistStamped velocity topic in local/map frame. Use /mavros/local_position/velocity_body only for body-frame comparison.",
    )
    parser.add_argument(
        "-o",
        "--output-dir",
        default="~/flight_test_logs",
        help="Directory for CSV and PNG output.",
    )
    return parser.parse_known_args()


def main():
    args, ros_args = parse_args()
    rclpy.init(args=ros_args)
    node = VelocityRecorder(args)

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
        if node.setpoint_samples or node.actual_samples:
            node.save()
        else:
            node.get_logger().warn("No samples received; nothing saved.")

        if interrupted:
            node.get_logger().info("Stopped by Ctrl-C.")

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
