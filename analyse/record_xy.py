#!/usr/bin/env python3
"""Record live pose/target topics and plot x-y tracks on exit."""

import argparse
import csv
import math
import re
import signal
import sys
import time
from datetime import datetime
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from plot_xy import plot_xy


STATUS_RE = {
    "state": re.compile(r"\bstate=([^\s]+)"),
    "running": re.compile(r"\brunning=([^\s]+)"),
    "reason": re.compile(r"\breason=([^\s]+)"),
    "target": re.compile(r"\btarget=\(([-+0-9.eE]+),([-+0-9.eE]+)\)"),
    "status_pose": re.compile(r"\bpose=\(([-+0-9.eE]+),([-+0-9.eE]+),([-+0-9.eE]+)\)"),
    "k_w": re.compile(r"\bk_w=([-+0-9.eE]+)"),
    "k_w_ff": re.compile(r"\bk_w_ff=([-+0-9.eE]+)"),
    "k_w_rate": re.compile(r"\bk_w_rate=([-+0-9.eE]+)"),
    "k_i_rate": re.compile(r"\bk_i_rate=([-+0-9.eE]+)"),
    "k_d_rate": re.compile(r"\bk_d_rate=([-+0-9.eE]+)"),
    "target_w": re.compile(r"\btarget_w=([-+0-9.eE]+)"),
    "yaw_rate_ff": re.compile(r"\byaw_rate_ff=([-+0-9.eE]+)"),
    "measured_w": re.compile(r"\bmeasured_w=([-+0-9.eE]+)"),
    "w_error": re.compile(r"\bw_error=([-+0-9.eE]+)"),
    "w_error_integral": re.compile(r"\bw_error_integral=([-+0-9.eE]+)"),
    "w_error_derivative": re.compile(r"\bw_error_derivative=([-+0-9.eE]+)"),
    "cmd_w": re.compile(r"\bcmd_w=([-+0-9.eE]+)"),
    "w_max": re.compile(r"\bw_max=([-+0-9.eE]+)"),
    "yaw_error": re.compile(r"\byaw_error=([-+0-9.eE]+)"),
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


def to_float(value):
    if value in (None, ""):
        return None
    try:
        return float(value)
    except ValueError:
        return None


def parse_status(text):
    parsed = {"status_raw": text}
    for key in ("state", "running", "reason"):
        match = STATUS_RE[key].search(text)
        if match:
            parsed[key] = match.group(1)

    match = STATUS_RE["target"].search(text)
    if match:
        parsed["target_x"] = float(match.group(1))
        parsed["target_y"] = float(match.group(2))

    match = STATUS_RE["status_pose"].search(text)
    if match:
        parsed["status_pose_x"] = float(match.group(1))
        parsed["status_pose_y"] = float(match.group(2))
        parsed["status_pose_yaw"] = float(match.group(3))

    for key in (
        "k_w",
        "k_w_ff",
        "k_w_rate",
        "k_i_rate",
        "k_d_rate",
        "target_w",
        "yaw_rate_ff",
        "measured_w",
        "w_error",
        "w_error_integral",
        "w_error_derivative",
        "cmd_w",
        "w_max",
        "yaw_error",
    ):
        match = STATUS_RE[key].search(text)
        if match:
            parsed[key] = float(match.group(1))
    return parsed


def resolve_rate_plot_output(csv_path, output):
    csv_path = Path(csv_path).expanduser()
    if not output:
        return csv_path.with_name(f"{csv_path.stem}_rate.png")
    output_path = Path(output).expanduser()
    if output_path.is_dir() or str(output).endswith("/"):
        return output_path / f"{csv_path.stem}_rate.png"
    return output_path.with_name(f"{output_path.stem}_rate{output_path.suffix or '.png'}")


def plot_angular_rate_tracking(csv_path, output=""):
    csv_path = Path(csv_path).expanduser()
    output_path = resolve_rate_plot_output(csv_path, output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with csv_path.open(newline="", encoding="utf-8") as csv_file:
        rows = list(csv.DictReader(csv_file))

    samples = []
    for row in rows:
        time_s = to_float(row.get("time_s"))
        target_w = to_float(row.get("target_w_rad_s"))
        measured_w = to_float(row.get("measured_w_rad_s"))
        if time_s is None or target_w is None or measured_w is None:
            continue
        w_error = to_float(row.get("w_error_rad_s"))
        if w_error is None:
            w_error = target_w - measured_w
        samples.append({
            "time_s": time_s,
            "target_w": target_w,
            "yaw_rate_ff": to_float(row.get("yaw_rate_ff_rad_s")),
            "measured_w": measured_w,
            "cmd_w": to_float(row.get("cmd_w_rad_s")),
            "w_error": w_error,
            "w_error_integral": to_float(row.get("w_error_integral_rad")),
            "w_error_derivative": to_float(row.get("w_error_derivative_rad_s2")),
            "yaw_error": to_float(row.get("yaw_error_rad")),
            "k_w": to_float(row.get("k_w")),
            "k_w_ff": to_float(row.get("k_w_ff")),
            "k_w_rate": to_float(row.get("k_w_rate")),
            "k_i_rate": to_float(row.get("k_i_rate")),
            "k_d_rate": to_float(row.get("k_d_rate")),
        })

    if not samples:
        raise RuntimeError(f"No plottable angular-rate samples in {csv_path}")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    times = [sample["time_s"] for sample in samples]
    target_w = [sample["target_w"] for sample in samples]
    yaw_rate_ff = [sample["yaw_rate_ff"] for sample in samples]
    measured_w = [sample["measured_w"] for sample in samples]
    cmd_w = [sample["cmd_w"] for sample in samples]
    w_error = [sample["w_error"] for sample in samples]
    w_error_integral = [sample["w_error_integral"] for sample in samples]
    w_error_derivative = [sample["w_error_derivative"] for sample in samples]
    yaw_error = [sample["yaw_error"] for sample in samples]
    k_w = [sample["k_w"] for sample in samples]
    k_w_ff = [sample["k_w_ff"] for sample in samples]
    k_w_rate = [sample["k_w_rate"] for sample in samples]
    k_i_rate = [sample["k_i_rate"] for sample in samples]
    k_d_rate = [sample["k_d_rate"] for sample in samples]

    fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
    fig.suptitle(f"Angular Rate Tracking: {csv_path.name}")

    axes[0].plot(times, target_w, label="target_w", linewidth=1.7)
    if any(value is not None for value in yaw_rate_ff):
        axes[0].plot(times, yaw_rate_ff, label="yaw_rate_ff", linewidth=1.1)
    axes[0].plot(times, measured_w, label="measured_w", linewidth=1.5)
    if any(value is not None for value in cmd_w):
        axes[0].plot(times, cmd_w, label="cmd_w", linewidth=1.1)
    axes[0].set_ylabel("w (rad/s)")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="best")

    axes[1].plot(times, w_error, label="w_error", color="tab:red", linewidth=1.4)
    axes[1].axhline(0.0, color="black", linewidth=0.8, alpha=0.5)
    axes[1].set_ylabel("w error")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc="best")

    has_pid_trace = False
    if any(value is not None for value in w_error_integral):
        axes[2].plot(times, w_error_integral, label="w_error_integral", linewidth=1.2)
        has_pid_trace = True
    if any(value is not None for value in w_error_derivative):
        axes[2].plot(times, w_error_derivative, label="w_error_derivative", linewidth=1.0)
        has_pid_trace = True
    if any(value is not None for value in yaw_error):
        axes[2].plot(times, yaw_error, label="yaw_error", linewidth=1.2)
        has_pid_trace = True
    axes[2].axhline(0.0, color="black", linewidth=0.8, alpha=0.5)
    axes[2].set_ylabel("pid/yaw")
    axes[2].grid(True, alpha=0.3)
    if has_pid_trace:
        axes[2].legend(loc="best")

    has_gain_trace = False
    for values, label in (
        (k_w, "k_w"),
        (k_w_ff, "k_w_ff"),
        (k_w_rate, "k_w_rate"),
        (k_i_rate, "k_i_rate"),
        (k_d_rate, "k_d_rate"),
    ):
        if any(value is not None for value in values):
            axes[3].plot(times, values, label=label, linewidth=1.2)
            has_gain_trace = True
    axes[3].set_ylabel("gain")
    axes[3].set_xlabel("time (s)")
    axes[3].grid(True, alpha=0.3)
    if has_gain_trace:
        axes[3].legend(loc="best")

    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)
    return output_path, len(samples)


class XYRecorder(Node):
    def __init__(self, args):
        super().__init__("xy_recorder")
        self.args = args
        self.start_time = self.get_clock().now()
        self.stop_requested = False
        self.start_attempts = 0
        self.pose_count = 0
        self.status_count = 0
        self.rows_written = 0
        self.latest_pose = {}
        self.latest_status = parse_status("")

        output_path = Path(args.output).expanduser()
        if output_path.is_dir() or str(args.output).endswith("/"):
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output_path = output_path / f"xy_track_{stamp}.csv"
        output_path.parent.mkdir(parents=True, exist_ok=True)
        self.output_path = output_path
        self.csv_file = output_path.open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.csv_file, fieldnames=self.fieldnames())
        self.writer.writeheader()

        self.pose_sub = self.create_subscription(PoseStamped, args.pose_topic, self.on_pose, 50)
        self.status_sub = self.create_subscription(String, args.status_topic, self.on_status, 50)
        self.command_pub = self.create_publisher(String, args.command_topic, 10)

        self.sample_timer = self.create_timer(1.0 / max(args.sample_hz, 0.1), self.write_sample)
        self.info_timer = self.create_timer(1.0, self.print_runtime_info)
        if args.send_start:
            self.start_timer = self.create_timer(0.2, self.send_start_until_active)

    @staticmethod
    def fieldnames():
        return [
            "time_s",
            "pose_age_s",
            "status_age_s",
            "pose_x",
            "pose_y",
            "pose_yaw",
            "target_x",
            "target_y",
            "status_pose_x",
            "status_pose_y",
            "status_pose_yaw",
            "state",
            "running",
            "reason",
            "k_w",
            "k_w_ff",
            "k_w_rate",
            "k_i_rate",
            "k_d_rate",
            "yaw_error_rad",
            "target_w_rad_s",
            "yaw_rate_ff_rad_s",
            "measured_w_rad_s",
            "w_error_rad_s",
            "w_error_integral_rad",
            "w_error_derivative_rad_s2",
            "cmd_w_rad_s",
            "w_max_rad_s",
            "status_raw",
        ]

    def on_pose(self, msg):
        self.latest_pose = {
            "stamp": self.get_clock().now(),
            "x": msg.pose.position.x,
            "y": msg.pose.position.y,
            "yaw": quaternion_to_yaw(msg.pose.orientation),
        }
        self.pose_count += 1

    def on_status(self, msg):
        self.latest_status = parse_status(msg.data)
        self.latest_status["stamp"] = self.get_clock().now()
        self.status_count += 1

    def publish_command(self, command):
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)

    def send_start_until_active(self):
        status = self.latest_status
        if status.get("state") == "running" or status.get("running") == "true":
            if hasattr(self, "start_timer"):
                self.start_timer.cancel()
            return
        if self.start_attempts >= self.args.start_repeats:
            if hasattr(self, "start_timer"):
                self.start_timer.cancel()
            self.get_logger().warning(
                f"start command was sent {self.start_attempts} times but status is not active"
            )
            return
        self.start_attempts += 1
        self.publish_command("start")
        self.get_logger().info(f"sent start command attempt {self.start_attempts}")

    def elapsed_seconds(self, now):
        return (now - self.start_time).nanoseconds / 1e9

    def age_seconds(self, item, now):
        if not item or "stamp" not in item:
            return None
        return (now - item["stamp"]).nanoseconds / 1e9

    def write_sample(self):
        now = self.get_clock().now()
        elapsed = self.elapsed_seconds(now)
        if self.args.duration_s > 0.0 and elapsed >= self.args.duration_s:
            self.stop_requested = True

        pose = self.latest_pose or {}
        status = self.latest_status or {}
        row = {
            "time_s": f"{elapsed:.3f}",
            "pose_age_s": float_or_blank(self.age_seconds(pose, now)),
            "status_age_s": float_or_blank(self.age_seconds(status, now)),
            "pose_x": float_or_blank(pose.get("x")),
            "pose_y": float_or_blank(pose.get("y")),
            "pose_yaw": float_or_blank(pose.get("yaw")),
            "target_x": float_or_blank(status.get("target_x")),
            "target_y": float_or_blank(status.get("target_y")),
            "status_pose_x": float_or_blank(status.get("status_pose_x")),
            "status_pose_y": float_or_blank(status.get("status_pose_y")),
            "status_pose_yaw": float_or_blank(status.get("status_pose_yaw")),
            "state": status.get("state", ""),
            "running": status.get("running", ""),
            "reason": status.get("reason", ""),
            "k_w": float_or_blank(status.get("k_w")),
            "k_w_ff": float_or_blank(status.get("k_w_ff")),
            "k_w_rate": float_or_blank(status.get("k_w_rate")),
            "k_i_rate": float_or_blank(status.get("k_i_rate")),
            "k_d_rate": float_or_blank(status.get("k_d_rate")),
            "yaw_error_rad": float_or_blank(status.get("yaw_error")),
            "target_w_rad_s": float_or_blank(status.get("target_w")),
            "yaw_rate_ff_rad_s": float_or_blank(status.get("yaw_rate_ff")),
            "measured_w_rad_s": float_or_blank(status.get("measured_w")),
            "w_error_rad_s": float_or_blank(status.get("w_error")),
            "w_error_integral_rad": float_or_blank(status.get("w_error_integral")),
            "w_error_derivative_rad_s2": float_or_blank(status.get("w_error_derivative")),
            "cmd_w_rad_s": float_or_blank(status.get("cmd_w")),
            "w_max_rad_s": float_or_blank(status.get("w_max")),
            "status_raw": status.get("status_raw", ""),
        }
        self.writer.writerow(row)
        self.rows_written += 1

    def print_runtime_info(self):
        pose = self.latest_pose or {}
        status = self.latest_status or {}
        self.get_logger().info(
            f"recording {self.output_path} | "
            f"pose=({float_or_blank(pose.get('x'))},{float_or_blank(pose.get('y'))}) "
            f"target=({float_or_blank(status.get('target_x'))},{float_or_blank(status.get('target_y'))}) "
            f"state={status.get('state') or status.get('running') or '-'} "
            f"reason={status.get('reason') or '-'}"
        )

    def close(self):
        if self.args.stop_on_exit:
            for _ in range(self.args.stop_repeats):
                self.publish_command("stop")
                rclpy.spin_once(self, timeout_sec=0.02)
                time.sleep(0.02)
        self.csv_file.flush()
        self.csv_file.close()

    def summary(self):
        return {
            "output_path": str(self.output_path),
            "rows": self.rows_written,
            "pose_msgs": self.pose_count,
            "status_msgs": self.status_count,
        }


def build_arg_parser():
    parser = argparse.ArgumentParser(
        description="Subscribe to pose/status topics, record x-y data, and plot on exit."
    )
    default_output = (
        Path(__file__).resolve().parent
        / "log"
        / f"xy_track_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    )
    parser.add_argument("--output", default=str(default_output), help="CSV file or directory")
    parser.add_argument("--duration-s", type=float, default=0.0, help="0 means record until Ctrl-C")
    parser.add_argument("--sample-hz", type=float, default=20.0)
    parser.add_argument("--pose-topic", default="/car/pose")
    parser.add_argument("--status-topic", default="/car/track_runner/status")
    parser.add_argument("--command-topic", default="/car/track_runner/command")
    parser.add_argument("--send-start", action="store_true", help="publish start after recorder is ready")
    parser.add_argument("--stop-on-exit", action="store_true", help="publish stop when the recorder exits")
    parser.add_argument("--start-repeats", type=int, default=20)
    parser.add_argument("--stop-repeats", type=int, default=10)
    parser.add_argument("--no-plot", action="store_false", dest="plot", help="do not write a PNG plot")
    parser.add_argument("--plot-output", default="", help="PNG file or directory")
    parser.add_argument("--target-point-size", type=float, default=10.0)
    parser.add_argument("--pose-point-size", type=float, default=0.0)
    parser.set_defaults(plot=True)
    return parser


def main():
    args = build_arg_parser().parse_args()
    rclpy.init()
    recorder = XYRecorder(args)

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

    print("\nX-Y record summary")
    print(f"  csv: {summary['output_path']}")
    print(
        "  messages: "
        f"pose={summary['pose_msgs']} status={summary['status_msgs']} rows={summary['rows']}"
    )
    if args.plot:
        output_path, pose_count, target_count = plot_xy(
            summary["output_path"],
            output=args.plot_output,
            target_point_size=args.target_point_size,
            pose_point_size=args.pose_point_size,
        )
        print(f"  xy plot: {output_path} pose_points={pose_count} target_points={target_count}")
        try:
            rate_output_path, rate_count = plot_angular_rate_tracking(
                summary["output_path"],
                output=args.plot_output,
            )
            print(f"  rate plot: {rate_output_path} samples={rate_count}")
        except RuntimeError as exc:
            print(f"  rate plot skipped: {exc}")


if __name__ == "__main__":
    main()
