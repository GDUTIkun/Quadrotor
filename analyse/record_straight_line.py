#!/usr/bin/env python3
"""Record the first straight segment of the track_runner closed-loop test."""

import argparse
import csv
import math
import re
import signal
from datetime import datetime
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from std_msgs.msg import String


STATUS_RE = {
    "state": re.compile(r"\bstate=([^\s]+)"),
    "lap": re.compile(r"\blap=([^\s]+)"),
    "index": re.compile(r"\bindex=([^\s]+)"),
    "progress": re.compile(r"\bprogress=([-+0-9.eE]+)"),
    "speed": re.compile(r"\bspeed=([-+0-9.eE]+)"),
    "cmd_w": re.compile(r"\bcmd_w=([-+0-9.eE]+)"),
    "yaw_error": re.compile(r"\byaw_error=([-+0-9.eE]+)"),
    "target": re.compile(r"\btarget=\(([-+0-9.eE]+),([-+0-9.eE]+)\)"),
    "status_pose": re.compile(r"\bpose=\(([-+0-9.eE]+),([-+0-9.eE]+),([-+0-9.eE]+)\)"),
    "remaining": re.compile(r"\bremaining=([-+0-9.eE]+)"),
    "reason": re.compile(r"\breason=([^\s]+)"),
}


def quaternion_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def float_or_blank(value):
    if value is None:
        return ""
    return f"{value:.9g}"


def parse_status(text):
    parsed = {
        "status_raw": text,
        "state": "",
        "lap": "",
        "index": "",
        "progress_m": None,
        "status_speed_m_s": None,
        "status_cmd_w_rad_s": None,
        "yaw_error_rad": None,
        "target_x": None,
        "target_y": None,
        "status_pose_x": None,
        "status_pose_y": None,
        "status_pose_yaw": None,
        "remaining_m": None,
        "reason": "",
    }

    for key in ("state", "lap", "index", "reason"):
        match = STATUS_RE[key].search(text)
        if match:
            parsed[key] = match.group(1)

    numeric_keys = {
        "progress": "progress_m",
        "speed": "status_speed_m_s",
        "cmd_w": "status_cmd_w_rad_s",
        "yaw_error": "yaw_error_rad",
        "remaining": "remaining_m",
    }
    for source_key, target_key in numeric_keys.items():
        match = STATUS_RE[source_key].search(text)
        if match:
            parsed[target_key] = float(match.group(1))

    match = STATUS_RE["target"].search(text)
    if match:
        parsed["target_x"] = float(match.group(1))
        parsed["target_y"] = float(match.group(2))

    match = STATUS_RE["status_pose"].search(text)
    if match:
        parsed["status_pose_x"] = float(match.group(1))
        parsed["status_pose_y"] = float(match.group(2))
        parsed["status_pose_yaw"] = float(match.group(3))

    return parsed


class StraightLineRecorder(Node):
    def __init__(self, args):
        super().__init__("straight_line_recorder")
        self.args = args
        self.start_time = self.get_clock().now()
        self.stop_requested = False
        self.auto_stop_sent = False

        self.latest_pose = None
        self.latest_cmd = None
        self.latest_status = parse_status("")
        self.pose_count = 0
        self.cmd_count = 0
        self.status_count = 0
        self.rows_written = 0
        self.first_pose = None
        self.last_pose = None
        self.max_abs_yaw_error = 0.0
        self.max_abs_cmd_w = 0.0

        output_path = Path(args.output).expanduser()
        if output_path.is_dir() or str(args.output).endswith("/"):
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output_path = output_path / f"straight_line_{stamp}.csv"
        output_path.parent.mkdir(parents=True, exist_ok=True)
        self.output_path = output_path
        self.csv_file = output_path.open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.csv_file, fieldnames=self.fieldnames())
        self.writer.writeheader()

        self.pose_sub = self.create_subscription(
            PoseStamped, args.pose_topic, self.on_pose, 50
        )
        self.cmd_sub = self.create_subscription(Twist, args.cmd_vel_topic, self.on_cmd, 50)
        self.status_sub = self.create_subscription(
            String, args.status_topic, self.on_status, 50
        )
        self.command_pub = self.create_publisher(String, args.command_topic, 10)

        period = 1.0 / max(args.sample_hz, 0.1)
        self.sample_timer = self.create_timer(period, self.write_sample)
        self.info_timer = self.create_timer(1.0, self.print_runtime_info)

        if args.send_start:
            self.create_timer(0.5, self.send_start_once)

    @staticmethod
    def fieldnames():
        return [
            "time_s",
            "pose_age_s",
            "cmd_age_s",
            "status_age_s",
            "cmd_vel_publishers",
            "cmd_vel_subscribers",
            "pose_x",
            "pose_y",
            "pose_yaw",
            "linear_x_m_s",
            "angular_z_rad_s",
            "state",
            "lap",
            "index",
            "progress_m",
            "status_speed_m_s",
            "status_cmd_w_rad_s",
            "yaw_error_rad",
            "target_x",
            "target_y",
            "target_dx",
            "target_dy",
            "remaining_m",
            "reason",
            "status_raw",
        ]

    def on_pose(self, msg):
        stamp = self.get_clock().now()
        yaw = quaternion_to_yaw(msg.pose.orientation)
        pose = {
            "stamp": stamp,
            "x": msg.pose.position.x,
            "y": msg.pose.position.y,
            "yaw": yaw,
        }
        self.latest_pose = pose
        self.last_pose = pose
        self.pose_count += 1
        if self.first_pose is None:
            self.first_pose = pose

    def on_cmd(self, msg):
        self.latest_cmd = {
            "stamp": self.get_clock().now(),
            "linear_x": msg.linear.x,
            "angular_z": msg.angular.z,
        }
        self.cmd_count += 1
        self.max_abs_cmd_w = max(self.max_abs_cmd_w, abs(msg.angular.z))

    def on_status(self, msg):
        self.latest_status = parse_status(msg.data)
        self.latest_status["stamp"] = self.get_clock().now()
        self.status_count += 1
        yaw_error = self.latest_status.get("yaw_error_rad")
        if yaw_error is not None:
            self.max_abs_yaw_error = max(self.max_abs_yaw_error, abs(yaw_error))

    def send_start_once(self):
        if getattr(self, "_start_sent", False):
            return
        self._start_sent = True
        self.publish_command("start")
        self.get_logger().info("sent start command")

    def publish_command(self, command):
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)

    def age_seconds(self, item, now):
        if not item or "stamp" not in item:
            return None
        return (now - item["stamp"]).nanoseconds / 1e9

    def elapsed_seconds(self, now):
        return (now - self.start_time).nanoseconds / 1e9

    def write_sample(self):
        now = self.get_clock().now()
        elapsed = self.elapsed_seconds(now)
        if self.args.duration_s > 0.0 and elapsed >= self.args.duration_s:
            self.stop_requested = True

        pose = self.latest_pose or {}
        cmd = self.latest_cmd or {}
        status = self.latest_status or {}

        target_dx = None
        target_dy = None
        if pose and status.get("target_x") is not None and status.get("target_y") is not None:
            target_dx = status["target_x"] - pose["x"]
            target_dy = status["target_y"] - pose["y"]

        row = {
            "time_s": f"{elapsed:.3f}",
            "pose_age_s": float_or_blank(self.age_seconds(self.latest_pose, now)),
            "cmd_age_s": float_or_blank(self.age_seconds(self.latest_cmd, now)),
            "status_age_s": float_or_blank(self.age_seconds(self.latest_status, now)),
            "cmd_vel_publishers": self.count_publishers(self.args.cmd_vel_topic),
            "cmd_vel_subscribers": self.count_subscribers(self.args.cmd_vel_topic),
            "pose_x": float_or_blank(pose.get("x")),
            "pose_y": float_or_blank(pose.get("y")),
            "pose_yaw": float_or_blank(pose.get("yaw")),
            "linear_x_m_s": float_or_blank(cmd.get("linear_x")),
            "angular_z_rad_s": float_or_blank(cmd.get("angular_z")),
            "state": status.get("state", ""),
            "lap": status.get("lap", ""),
            "index": status.get("index", ""),
            "progress_m": float_or_blank(status.get("progress_m")),
            "status_speed_m_s": float_or_blank(status.get("status_speed_m_s")),
            "status_cmd_w_rad_s": float_or_blank(status.get("status_cmd_w_rad_s")),
            "yaw_error_rad": float_or_blank(status.get("yaw_error_rad")),
            "target_x": float_or_blank(status.get("target_x")),
            "target_y": float_or_blank(status.get("target_y")),
            "target_dx": float_or_blank(target_dx),
            "target_dy": float_or_blank(target_dy),
            "remaining_m": float_or_blank(status.get("remaining_m")),
            "reason": status.get("reason", ""),
            "status_raw": status.get("status_raw", ""),
        }
        self.writer.writerow(row)
        self.rows_written += 1

        progress = status.get("progress_m")
        if (
            self.args.auto_stop_at_straight_end
            and not self.auto_stop_sent
            and progress is not None
            and progress >= self.args.straight_length_m
        ):
            self.auto_stop_sent = True
            self.publish_command("stop")
            self.get_logger().info(
                f"sent stop command at progress={progress:.3f} m"
            )

    def print_runtime_info(self):
        cmd_publishers = self.count_publishers(self.args.cmd_vel_topic)
        cmd_subscribers = self.count_subscribers(self.args.cmd_vel_topic)
        state = self.latest_status.get("state", "")
        progress = self.latest_status.get("progress_m")
        yaw_error = self.latest_status.get("yaw_error_rad")
        cmd_w = self.latest_cmd["angular_z"] if self.latest_cmd else None
        self.get_logger().info(
            "recording %s | /cmd_vel pub=%d sub=%d | state=%s progress=%s yaw_error=%s cmd_w=%s",
            self.output_path,
            cmd_publishers,
            cmd_subscribers,
            state or "-",
            "-" if progress is None else f"{progress:.3f}",
            "-" if yaw_error is None else f"{yaw_error:.3f}",
            "-" if cmd_w is None else f"{cmd_w:.3f}",
        )

    def close(self):
        if self.args.stop_on_exit:
            self.publish_command("stop")
        self.csv_file.flush()
        self.csv_file.close()

    def summary(self):
        dx = dy = distance = None
        if self.first_pose and self.last_pose:
            dx = self.last_pose["x"] - self.first_pose["x"]
            dy = self.last_pose["y"] - self.first_pose["y"]
            distance = math.hypot(dx, dy)
        return {
            "output_path": str(self.output_path),
            "rows": self.rows_written,
            "pose_msgs": self.pose_count,
            "cmd_msgs": self.cmd_count,
            "status_msgs": self.status_count,
            "delta_x_m": dx,
            "delta_y_m": dy,
            "distance_m": distance,
            "max_abs_yaw_error_rad": self.max_abs_yaw_error,
            "max_abs_cmd_w_rad_s": self.max_abs_cmd_w,
        }


def build_arg_parser():
    parser = argparse.ArgumentParser(
        description=(
            "Record /car/pose, /cmd_vel, and /car/track_runner/status during "
            "the first A->B straight-line closed-loop track_runner test."
        )
    )
    default_output = (
        Path(__file__).resolve().parent
        / "straight_line_logs"
        / f"straight_line_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    )
    parser.add_argument("--output", default=str(default_output), help="CSV file or directory")
    parser.add_argument("--duration-s", type=float, default=0.0, help="0 means record until Ctrl-C")
    parser.add_argument("--sample-hz", type=float, default=20.0)
    parser.add_argument("--straight-length-m", type=float, default=1.5)
    parser.add_argument("--pose-topic", default="/car/pose")
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel")
    parser.add_argument("--status-topic", default="/car/track_runner/status")
    parser.add_argument("--command-topic", default="/car/track_runner/command")
    parser.add_argument("--send-start", action="store_true", help="publish start after recorder is ready")
    parser.add_argument("--stop-on-exit", action="store_true", help="publish stop when the recorder exits")
    parser.add_argument(
        "--auto-stop-at-straight-end",
        action="store_true",
        help="publish stop once status progress reaches --straight-length-m",
    )
    return parser


def main():
    args = build_arg_parser().parse_args()
    rclpy.init()
    recorder = StraightLineRecorder(args)

    def handle_signal(_signum, _frame):
        recorder.stop_requested = True

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        while rclpy.ok() and not recorder.stop_requested:
            rclpy.spin_once(recorder, timeout_sec=0.1)
    finally:
        summary = recorder.summary()
        recorder.close()
        recorder.destroy_node()
        rclpy.shutdown()

    print("\nStraight-line record summary")
    print(f"  csv: {summary['output_path']}")
    print(
        "  messages: "
        f"pose={summary['pose_msgs']} cmd_vel={summary['cmd_msgs']} "
        f"status={summary['status_msgs']} rows={summary['rows']}"
    )
    if summary["distance_m"] is not None:
        print(
            "  odom delta: "
            f"dx={summary['delta_x_m']:.3f} m dy={summary['delta_y_m']:.3f} m "
            f"distance={summary['distance_m']:.3f} m"
        )
    print(
        "  max abs: "
        f"yaw_error={summary['max_abs_yaw_error_rad']:.3f} rad "
        f"cmd_w={summary['max_abs_cmd_w_rad_s']:.3f} rad/s"
    )


if __name__ == "__main__":
    main()
