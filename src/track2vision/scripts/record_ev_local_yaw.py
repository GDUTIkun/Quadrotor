#!/usr/bin/env python3

import argparse
import csv
import math
import os
from bisect import bisect_left
from datetime import datetime

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def unwrap_angles(angles):
    if not angles:
        return []
    unwrapped = [angles[0]]
    for angle in angles[1:]:
        delta = normalize_angle(angle - unwrapped[-1])
        unwrapped.append(unwrapped[-1] + delta)
    return unwrapped


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def msg_stamp_s(msg, fallback_time):
    stamp = msg.header.stamp
    t = stamp.sec + stamp.nanosec * 1e-9
    return t if t > 0.0 else fallback_time


def median(values):
    if not values:
        return 0.0
    ordered = sorted(values)
    mid = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[mid]
    return 0.5 * (ordered[mid - 1] + ordered[mid])


def nearest_value(times, values, query_time, max_dt):
    if not times:
        return None
    index = bisect_left(times, query_time)
    candidates = []
    if index < len(times):
        candidates.append(index)
    if index > 0:
        candidates.append(index - 1)
    best_index = min(candidates, key=lambda i: abs(times[i] - query_time))
    if abs(times[best_index] - query_time) > max_dt:
        return None
    return values[best_index]


class EvLocalYawRecorder(Node):
    def __init__(self, args):
        super().__init__("record_ev_local_yaw")
        self.args = args
        self.start_ros_time = None

        self.ev_samples = []
        self.local_samples = []
        self.vision_samples = []
        self.imu_samples = []

        os.makedirs(args.out_dir, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(args.out_dir, f"ev_local_yaw_{stamp}.csv")
        self.png_path = os.path.join(args.out_dir, f"ev_local_yaw_{stamp}.png")

        qos = QoSProfile(depth=200)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.ev_sub = self.create_subscription(
            Odometry, args.ev_topic, self.ev_callback, qos
        )
        self.local_sub = self.create_subscription(
            PoseStamped, args.local_topic, self.local_callback, qos
        )
        self.vision_sub = self.create_subscription(
            PoseStamped, args.vision_topic, self.vision_callback, qos
        )
        self.imu_sub = self.create_subscription(
            Imu, args.imu_topic, self.imu_callback, qos
        )
        self.status_timer = self.create_timer(2.0, self.print_status)

        self.get_logger().info(f"Recording EV yaw: {args.ev_topic}")
        self.get_logger().info(f"Recording local yaw: {args.local_topic}")
        self.get_logger().info(f"Optional Carto/vision yaw: {args.vision_topic}")
        self.get_logger().info(f"Optional IMU yawrate: {args.imu_topic}")
        self.get_logger().info("Press Ctrl+C to stop and generate CSV/PNG.")

    def relative_time(self, stamp_s):
        if self.start_ros_time is None:
            self.start_ros_time = stamp_s
        return stamp_s - self.start_ros_time

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def ev_callback(self, msg):
        stamp_s = msg_stamp_s(msg, self.now_s())
        t = self.relative_time(stamp_s)
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.ev_samples.append((t, yaw))

    def local_callback(self, msg):
        stamp_s = msg_stamp_s(msg, self.now_s())
        t = self.relative_time(stamp_s)
        yaw = quaternion_to_yaw(msg.pose.orientation)
        self.local_samples.append((t, yaw))

    def vision_callback(self, msg):
        stamp_s = msg_stamp_s(msg, self.now_s())
        t = self.relative_time(stamp_s)
        yaw = quaternion_to_yaw(msg.pose.orientation)
        self.vision_samples.append((t, yaw))

    def imu_callback(self, msg):
        stamp_s = msg_stamp_s(msg, self.now_s())
        t = self.relative_time(stamp_s)
        self.imu_samples.append((t, msg.angular_velocity.z))

    def print_status(self):
        self.get_logger().info(
            "samples: "
            f"ev={len(self.ev_samples)} local={len(self.local_samples)} "
            f"vision={len(self.vision_samples)} imu={len(self.imu_samples)}"
        )

    def finish(self):
        if not self.ev_samples or not self.local_samples:
            self.get_logger().error(
                "Not enough data. Need both EV odometry and local pose samples."
            )
            return

        data = self.build_aligned_data()
        self.write_csv(data)
        self.plot(data)
        self.get_logger().info(f"Saved CSV: {self.csv_path}")
        self.get_logger().info(f"Saved plot: {self.png_path}")

    def build_aligned_data(self):
        ev_t, ev_yaw = zip(*self.ev_samples)
        local_t, local_yaw = zip(*self.local_samples)

        ev_yaw_unwrapped = unwrap_angles(list(ev_yaw))
        local_yaw_unwrapped = unwrap_angles(list(local_yaw))

        align_end = self.args.align_start_s + self.args.align_window_s
        ev_align = [
            yaw
            for t, yaw in zip(ev_t, ev_yaw_unwrapped)
            if self.args.align_start_s <= t <= align_end
        ]
        local_align = [
            yaw
            for t, yaw in zip(local_t, local_yaw_unwrapped)
            if self.args.align_start_s <= t <= align_end
        ]
        yaw_offset = median(local_align) - median(ev_align)
        ev_aligned = [yaw + yaw_offset for yaw in ev_yaw_unwrapped]

        local_diff_t = []
        local_minus_ev = []
        for t, local_yaw_value in zip(local_t, local_yaw_unwrapped):
            ev_value = nearest_value(ev_t, ev_aligned, t, self.args.match_tolerance_s)
            if ev_value is None:
                continue
            local_diff_t.append(t)
            local_minus_ev.append(normalize_angle(local_yaw_value - ev_value))

        vision_data = None
        if self.vision_samples:
            vision_t, vision_yaw = zip(*self.vision_samples)
            vision_data = (list(vision_t), unwrap_angles(list(vision_yaw)))

        imu_data = None
        if self.imu_samples:
            imu_t, imu_yawrate = zip(*self.imu_samples)
            imu_data = (list(imu_t), list(imu_yawrate))

        return {
            "ev_t": list(ev_t),
            "ev_yaw_raw": ev_yaw_unwrapped,
            "ev_yaw_aligned": ev_aligned,
            "local_t": list(local_t),
            "local_yaw": local_yaw_unwrapped,
            "vision": vision_data,
            "imu": imu_data,
            "diff_t": local_diff_t,
            "local_minus_ev": local_minus_ev,
            "yaw_offset": yaw_offset,
        }

    def write_csv(self, data):
        ev_t = data["ev_t"]
        local_t = data["local_t"]
        local_yaw = data["local_yaw"]
        diff_t = data["diff_t"]
        diff = data["local_minus_ev"]

        with open(self.csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    "ev_time_s",
                    "ev_yaw_raw_deg",
                    "ev_yaw_aligned_deg",
                    "nearest_local_yaw_deg",
                    "local_minus_ev_deg",
                ]
            )
            for t, raw, aligned in zip(
                ev_t, data["ev_yaw_raw"], data["ev_yaw_aligned"]
            ):
                nearest_local = nearest_value(
                    local_t, local_yaw, t, self.args.match_tolerance_s
                )
                nearest_diff = nearest_value(diff_t, diff, t, self.args.match_tolerance_s)
                writer.writerow(
                    [
                        f"{t:.6f}",
                        f"{math.degrees(raw):.6f}",
                        f"{math.degrees(aligned):.6f}",
                        "" if nearest_local is None else f"{math.degrees(nearest_local):.6f}",
                        "" if nearest_diff is None else f"{math.degrees(nearest_diff):.6f}",
                    ]
                )

    def plot(self, data):
        fig, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=True)

        axes[0].plot(
            data["ev_t"],
            [math.degrees(v) for v in data["ev_yaw_aligned"]],
            label="EV yaw aligned (/mavros/odometry/out)",
            linewidth=1.4,
            linestyle="-",
            zorder=3,
        )
        axes[0].plot(
            data["local_t"],
            [math.degrees(v) for v in data["local_yaw"]],
            label="Local EKF yaw (/mavros/local_position/pose)",
            linewidth=1.2,
            linestyle="--",
            zorder=2,
        )
        if data["vision"] is not None:
            vision_t, vision_yaw = data["vision"]
            axes[0].plot(
                vision_t,
                [math.degrees(v) for v in vision_yaw],
                label="Vision pose yaw (/track2vision/vision_pose)",
                linewidth=1.0,
                linestyle=":",
                alpha=0.8,
                zorder=1,
            )
        axes[0].set_ylabel("yaw deg")
        axes[0].grid(True)
        axes[0].legend(loc="best")
        axes[0].set_title(
            "EV/local yaw comparison "
            f"(EV offset applied: {math.degrees(data['yaw_offset']):.2f} deg)"
        )

        axes[1].plot(
            data["diff_t"],
            [math.degrees(v) for v in data["local_minus_ev"]],
            label="local yaw - aligned EV yaw",
            color="tab:red",
            linewidth=1.1,
        )
        axes[1].axhline(0.0, color="black", linewidth=0.8)
        axes[1].set_ylabel("yaw error deg")
        axes[1].grid(True)
        axes[1].legend(loc="best")

        if data["imu"] is not None:
            imu_t, imu_yawrate = data["imu"]
            axes[2].plot(
                imu_t,
                [math.degrees(v) for v in imu_yawrate],
                label="IMU angular_velocity.z",
                color="tab:green",
                linewidth=1.0,
            )
        axes[2].set_ylabel("yawrate deg/s")
        axes[2].set_xlabel("time s")
        axes[2].grid(True)
        axes[2].legend(loc="best")

        fig.tight_layout()
        fig.savefig(self.png_path, dpi=150)
        plt.close(fig)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Record EV odometry yaw and MAVROS local yaw, align initial yaw, then plot."
    )
    parser.add_argument("--ev-topic", default="/mavros/odometry/out")
    parser.add_argument("--local-topic", default="/mavros/local_position/pose")
    parser.add_argument("--vision-topic", default="/track2vision/vision_pose")
    parser.add_argument("--imu-topic", default="/mavros/imu/data")
    parser.add_argument("--out-dir", default="bags")
    parser.add_argument(
        "--align-start-s",
        type=float,
        default=0.5,
        help="Start time of initial yaw alignment window.",
    )
    parser.add_argument(
        "--align-window-s",
        type=float,
        default=2.0,
        help="Window length used to estimate constant EV yaw offset.",
    )
    parser.add_argument(
        "--match-tolerance-s",
        type=float,
        default=0.05,
        help="Max time difference for nearest-sample yaw error calculation.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = EvLocalYawRecorder(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopping recorder, generating plot...")
    finally:
        node.finish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
