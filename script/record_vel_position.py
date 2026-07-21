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


class VelocityPositionRecorder(Node):
    def __init__(self, args):
        super().__init__("velocity_position_recorder")
        self.args = args
        self.finished = False
        self.start_time = self.get_clock().now().nanoseconds * 1e-9
        self.rows = []

        self.latest_setpoint = None
        self.latest_actual_velocity = None
        self.latest_actual_position = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )

        self.create_subscription(
            PositionTarget,
            args.setpoint_topic,
            self.setpoint_callback,
            qos,
        )
        self.create_subscription(
            TwistStamped,
            args.actual_velocity_topic,
            self.actual_velocity_callback,
            qos,
        )
        self.create_subscription(
            PoseStamped,
            args.actual_position_topic,
            self.actual_position_callback,
            qos,
        )

        self.sample_timer = self.create_timer(1.0 / args.sample_rate, self.sample_callback)
        self.stop_timer = self.create_timer(0.2, self.stop_callback)

        self.get_logger().info(
            f"Recording {args.duration:.1f}s at {args.sample_rate:.1f}Hz to {args.output_dir}"
        )
        self.get_logger().info(
            f"Topics: setpoint={args.setpoint_topic}, "
            f"actual_velocity={args.actual_velocity_topic}, "
            f"actual_position={args.actual_position_topic}"
        )

    def elapsed(self):
        return self.get_clock().now().nanoseconds * 1e-9 - self.start_time

    def setpoint_callback(self, msg):
        t = self.elapsed()
        self.latest_setpoint = {
            "t": t,
            "vx": msg.velocity.x,
            "vy": msg.velocity.y,
            "vz": msg.velocity.z,
            "x": msg.position.x,
            "y": msg.position.y,
            "z": msg.position.z,
            "type_mask": msg.type_mask,
        }

    def actual_velocity_callback(self, msg):
        t = self.elapsed()
        v = msg.twist.linear
        self.latest_actual_velocity = {
            "t": t,
            "vx": v.x,
            "vy": v.y,
            "vz": v.z,
        }

    def actual_position_callback(self, msg):
        t = self.elapsed()
        p = msg.pose.position
        self.latest_actual_position = {
            "t": t,
            "x": p.x,
            "y": p.y,
            "z": p.z,
        }

    def sample_callback(self):
        t = self.elapsed()
        sp = self.latest_setpoint
        av = self.latest_actual_velocity
        ap = self.latest_actual_position

        row = {
            "time_s": t,
            "setpoint_age_s": self.age(t, sp),
            "actual_velocity_age_s": self.age(t, av),
            "actual_position_age_s": self.age(t, ap),
            "setpoint_vx": self.value(sp, "vx"),
            "setpoint_vy": self.value(sp, "vy"),
            "setpoint_vz": self.value(sp, "vz"),
            "setpoint_speed_xy": self.speed_xy(sp),
            "setpoint_speed_xyz": self.speed_xyz(sp),
            "actual_vx": self.value(av, "vx"),
            "actual_vy": self.value(av, "vy"),
            "actual_vz": self.value(av, "vz"),
            "actual_speed_xy": self.speed_xy(av),
            "actual_speed_xyz": self.speed_xyz(av),
            "actual_x": self.value(ap, "x"),
            "actual_y": self.value(ap, "y"),
            "actual_z": self.value(ap, "z"),
            "setpoint_position_x": self.value(sp, "x"),
            "setpoint_position_y": self.value(sp, "y"),
            "setpoint_position_z": self.value(sp, "z"),
            "setpoint_type_mask": self.value(sp, "type_mask"),
        }
        self.rows.append(row)

    def stop_callback(self):
        if self.elapsed() >= self.args.duration:
            self.finished = True
            self.sample_timer.cancel()
            self.stop_timer.cancel()

        if not any([self.latest_setpoint, self.latest_actual_velocity, self.latest_actual_position]):
            self.get_logger().warn(
                "Waiting for first sample...",
                throttle_duration_sec=2.0,
            )

    @staticmethod
    def value(sample, key):
        if sample is None:
            return ""
        return sample[key]

    @staticmethod
    def age(t, sample):
        if sample is None:
            return ""
        return max(0.0, t - sample["t"])

    @staticmethod
    def speed_xy(sample):
        if sample is None:
            return ""
        return math.hypot(sample["vx"], sample["vy"])

    @staticmethod
    def speed_xyz(sample):
        if sample is None:
            return ""
        return math.sqrt(sample["vx"] ** 2 + sample["vy"] ** 2 + sample["vz"] ** 2)

    def save(self):
        output_dir = Path(os.path.expanduser(self.args.output_dir))
        output_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = output_dir / f"vel_position_{stamp}.csv"
        png_path = output_dir / f"vel_position_{stamp}.png"

        fieldnames = [
            "time_s",
            "setpoint_age_s",
            "actual_velocity_age_s",
            "actual_position_age_s",
            "setpoint_vx",
            "setpoint_vy",
            "setpoint_vz",
            "setpoint_speed_xy",
            "setpoint_speed_xyz",
            "actual_vx",
            "actual_vy",
            "actual_vz",
            "actual_speed_xy",
            "actual_speed_xyz",
            "actual_x",
            "actual_y",
            "actual_z",
            "setpoint_position_x",
            "setpoint_position_y",
            "setpoint_position_z",
            "setpoint_type_mask",
        ]

        with csv_path.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.rows)

        self.get_logger().info(f"Saved CSV: {csv_path}")

        if not self.args.no_plot:
            self.plot(png_path)
            self.get_logger().info(f"Saved plot: {png_path}")

    def plot(self, png_path):
        t = self.column("time_s")
        fig, axes = plt.subplots(4, 1, figsize=(13, 11), sharex=True)

        self.plot_pair(axes[0], t, "setpoint_vx", "actual_vx", "Vx [m/s]")
        self.plot_pair(axes[1], t, "setpoint_vy", "actual_vy", "Vy [m/s]")
        self.plot_pair(axes[2], t, "setpoint_speed_xy", "actual_speed_xy", "XY speed [m/s]")

        for key, label in [("actual_x", "x"), ("actual_y", "y"), ("actual_z", "z")]:
            axes[3].plot(t, self.column(key), label=label, linewidth=1.2)
        axes[3].set_ylabel("Position [m]")
        axes[3].grid(True, alpha=0.35)
        axes[3].legend(loc="best")
        axes[3].set_xlabel("Time [s]")

        fig.suptitle("Velocity setpoint, actual velocity, and local position")
        fig.tight_layout()
        fig.savefig(png_path, dpi=160)
        plt.close(fig)

    def plot_pair(self, axis, t, setpoint_key, actual_key, ylabel):
        axis.plot(t, self.column(setpoint_key), label="setpoint", linewidth=1.5)
        axis.plot(t, self.column(actual_key), label="actual", linewidth=1.2)
        axis.set_ylabel(ylabel)
        axis.grid(True, alpha=0.35)
        axis.legend(loc="best")

    def column(self, key):
        values = []
        for row in self.rows:
            value = row[key]
            values.append(float("nan") if value == "" else value)
        return values


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Record MAVROS raw setpoint velocity, actual local velocity, "
            "and actual local position into an aligned CSV."
        )
    )
    parser.add_argument(
        "-d",
        "--duration",
        type=float,
        default=60.0,
        help="Recording duration in seconds.",
    )
    parser.add_argument(
        "--sample-rate",
        type=float,
        default=50.0,
        help="CSV sampling rate in Hz.",
    )
    parser.add_argument(
        "--setpoint-topic",
        default="/mavros/setpoint_raw/local",
        help="PositionTarget setpoint topic from the offboard node.",
    )
    parser.add_argument(
        "--actual-velocity-topic",
        default="/mavros/local_position/velocity_local",
        help="Actual local velocity TwistStamped topic.",
    )
    parser.add_argument(
        "--actual-position-topic",
        default="/mavros/local_position/pose",
        help="Actual local position PoseStamped topic.",
    )
    parser.add_argument(
        "-o",
        "--output-dir",
        default="analysis",
        help="Directory for CSV and PNG output.",
    )
    parser.add_argument(
        "--no-plot",
        action="store_true",
        help="Only save CSV; skip PNG plot generation.",
    )
    return parser.parse_known_args()


def main():
    args, ros_args = parse_args()
    if args.sample_rate <= 0.0:
        raise SystemExit("--sample-rate must be positive")
    if args.duration <= 0.0:
        raise SystemExit("--duration must be positive")

    rclpy.init(args=ros_args)
    node = VelocityPositionRecorder(args)

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
        if node.rows:
            node.save()
        else:
            node.get_logger().warn("No rows recorded; nothing saved.")

        if interrupted:
            node.get_logger().info("Stopped by Ctrl-C.")

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
