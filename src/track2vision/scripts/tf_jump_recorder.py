#!/usr/bin/env python3

import argparse
import csv
import math
import os
from datetime import datetime

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


TF_PAIRS = (
    ("map_odom", "map", "odom"),
    ("odom_base_link", "odom", "base_link"),
    ("map_base_link", "map", "base_link"),
    ("map_laser", "map", "laser"),
    ("base_link_laser", "base_link", "laser"),
)


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_rpy(q):
    x = q.x
    y = q.y
    z = q.z
    w = q.w

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class TfJumpRecorder(Node):
    def __init__(self, args):
        super().__init__("tf_jump_recorder")
        self.args = args
        self.tf_buffer = Buffer(cache_time=Duration(seconds=args.tf_cache_s))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_scan = None
        self.scan_count = 0
        self.last_scan_wall = None
        self.scan_hz = 0.0
        self.prev_latest = {}
        self.start_wall = self.get_clock().now()

        os.makedirs(args.out_dir, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(args.out_dir, f"tf_jump_{stamp}.csv")
        self.events_path = os.path.join(args.out_dir, f"tf_jump_events_{stamp}.txt")

        self.csv_file = open(self.csv_path, "w", newline="")
        self.events_file = open(self.events_path, "w")
        self.writer = csv.DictWriter(self.csv_file, fieldnames=self.fieldnames())
        self.writer.writeheader()

        self.scan_sub = self.create_subscription(
            LaserScan,
            args.scan_topic,
            self.scan_callback,
            50,
        )
        self.timer = self.create_timer(1.0 / args.rate_hz, self.record_sample)

        self.get_logger().info(f"Recording TF jumps to: {self.csv_path}")
        self.get_logger().info(f"Recording jump events to: {self.events_path}")
        self.get_logger().info(
            "Watching: "
            + ", ".join(f"{target}->{source}" for _, target, source in TF_PAIRS)
        )

    def destroy_node(self):
        try:
            self.csv_file.flush()
            self.csv_file.close()
            self.events_file.flush()
            self.events_file.close()
        finally:
            super().destroy_node()

    def fieldnames(self):
        fields = [
            "wall_time_s",
            "ros_time_s",
            "scan_count",
            "scan_frame",
            "scan_stamp_s",
            "scan_age_s",
            "scan_points",
            "scan_hz",
            "scan_time_tf_ok",
            "scan_time_tf_error",
        ]
        for prefix, _, _ in TF_PAIRS:
            fields.extend(
                [
                    f"{prefix}_ok",
                    f"{prefix}_error",
                    f"{prefix}_tf_stamp_s",
                    f"{prefix}_tf_age_s",
                    f"{prefix}_x",
                    f"{prefix}_y",
                    f"{prefix}_z",
                    f"{prefix}_roll_deg",
                    f"{prefix}_pitch_deg",
                    f"{prefix}_yaw_deg",
                    f"{prefix}_delta_xy_m",
                    f"{prefix}_delta_yaw_deg",
                    f"{prefix}_jump",
                ]
            )
        return fields

    def scan_callback(self, msg):
        now = self.get_clock().now()
        if self.last_scan_wall is not None:
            dt = (now - self.last_scan_wall).nanoseconds / 1e9
            if dt > 0.0:
                hz = 1.0 / dt
                self.scan_hz = hz if self.scan_hz <= 0.0 else 0.85 * self.scan_hz + 0.15 * hz
        self.last_scan_wall = now
        self.latest_scan = msg
        self.scan_count += 1

    def record_sample(self):
        now = self.get_clock().now()
        wall_elapsed = (now - self.start_wall).nanoseconds / 1e9
        ros_time_s = now.nanoseconds / 1e9

        row = {
            "wall_time_s": f"{wall_elapsed:.6f}",
            "ros_time_s": f"{ros_time_s:.9f}",
            "scan_count": self.scan_count,
            "scan_frame": "",
            "scan_stamp_s": "",
            "scan_age_s": "",
            "scan_points": "",
            "scan_hz": f"{self.scan_hz:.3f}",
            "scan_time_tf_ok": "",
            "scan_time_tf_error": "",
        }

        if self.latest_scan is not None:
            scan_stamp = Time.from_msg(self.latest_scan.header.stamp)
            row["scan_frame"] = self.latest_scan.header.frame_id
            row["scan_stamp_s"] = f"{scan_stamp.nanoseconds / 1e9:.9f}"
            row["scan_age_s"] = f"{(now - scan_stamp).nanoseconds / 1e9:.6f}"
            row["scan_points"] = len(self.latest_scan.ranges)
            self.record_scan_time_tf(scan_stamp, row)

        events = []
        for prefix, target, source in TF_PAIRS:
            event = self.record_latest_tf(prefix, target, source, now, row)
            if event:
                events.append(event)

        self.writer.writerow(row)
        self.csv_file.flush()

        if events:
            for event in events:
                self.events_file.write(event + "\n")
                self.get_logger().warn(event)
            self.events_file.flush()

        if self.args.duration_s > 0.0 and wall_elapsed >= self.args.duration_s:
            self.get_logger().info("Duration reached, stopping recorder.")
            raise KeyboardInterrupt

    def record_scan_time_tf(self, scan_stamp, row):
        try:
            self.tf_buffer.lookup_transform("map", "laser", scan_stamp)
            row["scan_time_tf_ok"] = "1"
            row["scan_time_tf_error"] = ""
        except TransformException as exc:
            row["scan_time_tf_ok"] = "0"
            row["scan_time_tf_error"] = str(exc).replace("\n", " ")

    def record_latest_tf(self, prefix, target, source, now, row):
        for key in (
            "ok",
            "error",
            "tf_stamp_s",
            "tf_age_s",
            "x",
            "y",
            "z",
            "roll_deg",
            "pitch_deg",
            "yaw_deg",
            "delta_xy_m",
            "delta_yaw_deg",
            "jump",
        ):
            row[f"{prefix}_{key}"] = ""

        try:
            transform = self.tf_buffer.lookup_transform(target, source, Time())
        except TransformException as exc:
            row[f"{prefix}_ok"] = "0"
            row[f"{prefix}_error"] = str(exc).replace("\n", " ")
            return None

        t = transform.transform.translation
        r = transform.transform.rotation
        roll, pitch, yaw = quaternion_to_rpy(r)
        tf_stamp = Time.from_msg(transform.header.stamp)
        tf_age = (now - tf_stamp).nanoseconds / 1e9

        row[f"{prefix}_ok"] = "1"
        row[f"{prefix}_tf_stamp_s"] = f"{tf_stamp.nanoseconds / 1e9:.9f}"
        row[f"{prefix}_tf_age_s"] = f"{tf_age:.6f}"
        row[f"{prefix}_x"] = f"{t.x:.6f}"
        row[f"{prefix}_y"] = f"{t.y:.6f}"
        row[f"{prefix}_z"] = f"{t.z:.6f}"
        row[f"{prefix}_roll_deg"] = f"{math.degrees(roll):.6f}"
        row[f"{prefix}_pitch_deg"] = f"{math.degrees(pitch):.6f}"
        row[f"{prefix}_yaw_deg"] = f"{math.degrees(yaw):.6f}"

        previous = self.prev_latest.get(prefix)
        self.prev_latest[prefix] = (t.x, t.y, yaw)
        if previous is None:
            row[f"{prefix}_delta_xy_m"] = "0.000000"
            row[f"{prefix}_delta_yaw_deg"] = "0.000000"
            row[f"{prefix}_jump"] = "0"
            return None

        dx = t.x - previous[0]
        dy = t.y - previous[1]
        delta_xy = math.hypot(dx, dy)
        delta_yaw_deg = math.degrees(normalize_angle(yaw - previous[2]))
        jump = (
            abs(delta_yaw_deg) >= self.args.yaw_jump_deg
            or delta_xy >= self.args.xy_jump_m
        )

        row[f"{prefix}_delta_xy_m"] = f"{delta_xy:.6f}"
        row[f"{prefix}_delta_yaw_deg"] = f"{delta_yaw_deg:.6f}"
        row[f"{prefix}_jump"] = "1" if jump else "0"

        if not jump:
            return None

        return (
            f"t={row['wall_time_s']}s {target}->{source} jump: "
            f"dxy={delta_xy:.3f}m dyaw={delta_yaw_deg:.2f}deg "
            f"pose=({t.x:.3f},{t.y:.3f},{math.degrees(yaw):.2f}deg) "
            f"scan_age={row['scan_age_s']}"
        )


def parse_args():
    parser = argparse.ArgumentParser(
        description="Record TF and LaserScan timing to diagnose RViz scan jumps."
    )
    parser.add_argument("--out-dir", default=os.path.expanduser("~/flight_tf_logs"))
    parser.add_argument("--scan-topic", default="/scan")
    parser.add_argument("--rate-hz", type=float, default=30.0)
    parser.add_argument("--duration-s", type=float, default=0.0)
    parser.add_argument("--yaw-jump-deg", type=float, default=2.0)
    parser.add_argument("--xy-jump-m", type=float, default=0.03)
    parser.add_argument("--tf-cache-s", type=float, default=20.0)
    return parser.parse_known_args()


def main():
    args, ros_args = parse_args()
    rclpy.init(args=ros_args)
    node = TfJumpRecorder(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f"CSV saved: {node.csv_path}")
        node.get_logger().info(f"Events saved: {node.events_path}")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
