#!/usr/bin/env python3
"""Record circle angle-loop target/current yaw and angular velocity."""

import argparse
import csv
import re
import signal
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


STATUS_RE = {
    "running": re.compile(r"\brunning=([^\s]+)"),
    "radius": re.compile(r"\bradius=([-+0-9.eE]+)"),
    "clockwise": re.compile(r"\bclockwise=([^\s]+)"),
    "speed": re.compile(r"\bspeed=([-+0-9.eE]+)"),
    "lookahead": re.compile(r"\blookahead=([-+0-9.eE]+)"),
    "k_w": re.compile(r"\bk_w=([-+0-9.eE]+)"),
    "k_w_rate": re.compile(r"\bk_w_rate=([-+0-9.eE]+)"),
    "k_i_rate": re.compile(r"\bk_i_rate=([-+0-9.eE]+)"),
    "k_d_rate": re.compile(r"\bk_d_rate=([-+0-9.eE]+)"),
    "pose": re.compile(r"\bpose=\(([-+0-9.eE]+),([-+0-9.eE]+),([-+0-9.eE]+)\)"),
    "center": re.compile(r"\bcenter=\(([-+0-9.eE]+),([-+0-9.eE]+)\)"),
    "target": re.compile(r"\btarget=\(([-+0-9.eE]+),([-+0-9.eE]+)\)"),
    "target_yaw": re.compile(r"\btarget_yaw=([-+0-9.eE]+)"),
    "yaw_error": re.compile(r"\byaw_error=([-+0-9.eE]+)"),
    "target_w": re.compile(r"\btarget_w=([-+0-9.eE]+)"),
    "measured_w": re.compile(r"\bmeasured_w=([-+0-9.eE]+)"),
    "w_error": re.compile(r"\bw_error=([-+0-9.eE]+)"),
    "w_error_integral": re.compile(r"\bw_error_integral=([-+0-9.eE]+)"),
    "w_error_derivative": re.compile(r"\bw_error_derivative=([-+0-9.eE]+)"),
    "cmd_w": re.compile(r"\bcmd_w=([-+0-9.eE]+)"),
    "w_max": re.compile(r"\bw_max=([-+0-9.eE]+)"),
    "pose_fresh": re.compile(r"\bpose_fresh=([^\s]+)"),
    "odom_fresh": re.compile(r"\bodom_fresh=([^\s]+)"),
    "reason": re.compile(r"\breason=([^\s]+)"),
}


def to_float(value):
    if value in (None, ""):
        return None
    try:
        return float(value)
    except ValueError:
        return None


def float_or_blank(value):
    if value is None:
        return ""
    return f"{value:.9g}"


def parse_status(text):
    parsed = {"status_raw": text}
    text_keys = {"running", "clockwise", "pose_fresh", "odom_fresh", "reason"}
    for key, pattern in STATUS_RE.items():
        match = pattern.search(text)
        if not match:
            continue
        if key in text_keys:
            parsed[key] = match.group(1)
        elif key == "pose":
            parsed["pose_x"] = float(match.group(1))
            parsed["pose_y"] = float(match.group(2))
            parsed["pose_yaw"] = float(match.group(3))
        elif key == "center":
            parsed["center_x"] = float(match.group(1))
            parsed["center_y"] = float(match.group(2))
        elif key == "target":
            parsed["target_x"] = float(match.group(1))
            parsed["target_y"] = float(match.group(2))
        else:
            parsed[key] = float(match.group(1))
    return parsed


def resolve_plot_output(csv_path, plot_output):
    if not plot_output:
        return csv_path.with_suffix(".png")
    output_path = Path(plot_output).expanduser()
    if output_path.is_dir() or str(plot_output).endswith("/"):
        return output_path / f"{csv_path.stem}.png"
    return output_path


def plot_circle_angle_csv(csv_path, plot_output=""):
    csv_path = Path(csv_path).expanduser()
    output_path = resolve_plot_output(csv_path, plot_output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with csv_path.open(newline="", encoding="utf-8") as csv_file:
        rows = list(csv.DictReader(csv_file))

    samples = []
    for row in rows:
        time_s = to_float(row.get("time_s"))
        yaw_error = to_float(row.get("yaw_error_rad"))
        target_w = to_float(row.get("target_w_rad_s"))
        measured_w = to_float(row.get("measured_w_rad_s"))
        if time_s is None or yaw_error is None or target_w is None or measured_w is None:
            continue
        samples.append({
            "time_s": time_s,
            "pose_yaw": to_float(row.get("pose_yaw_rad")),
            "target_yaw": to_float(row.get("target_yaw_rad")),
            "yaw_error": yaw_error,
            "target_w": target_w,
            "measured_w": measured_w,
            "cmd_w": to_float(row.get("cmd_w_rad_s")),
            "w_error": to_float(row.get("w_error_rad_s")),
            "k_w": to_float(row.get("k_w")),
        })

    if not samples:
        raise RuntimeError(f"No plottable circle-angle samples in {csv_path}")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    times = [sample["time_s"] for sample in samples]
    pose_yaw = [sample["pose_yaw"] for sample in samples]
    target_yaw = [sample["target_yaw"] for sample in samples]
    yaw_error = [sample["yaw_error"] for sample in samples]
    target_w = [sample["target_w"] for sample in samples]
    measured_w = [sample["measured_w"] for sample in samples]
    cmd_w = [sample["cmd_w"] for sample in samples]
    w_error = [sample["w_error"] for sample in samples]
    k_w = [sample["k_w"] for sample in samples]

    fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
    fig.suptitle(f"Circle Angle Tracking: {csv_path.name}")

    if any(value is not None for value in target_yaw):
        axes[0].plot(times, target_yaw, label="target_yaw", linewidth=1.5)
    if any(value is not None for value in pose_yaw):
        axes[0].plot(times, pose_yaw, label="pose_yaw", linewidth=1.5)
    axes[0].set_ylabel("yaw (rad)")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="best")

    axes[1].plot(times, yaw_error, label="yaw_error", color="tab:red", linewidth=1.5)
    axes[1].axhline(0.0, color="black", linewidth=0.8, alpha=0.5)
    axes[1].set_ylabel("yaw error (rad)")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc="best")

    axes[2].plot(times, target_w, label="target_w", linewidth=1.5)
    axes[2].plot(times, measured_w, label="measured_w", linewidth=1.5)
    if any(value is not None for value in cmd_w):
        axes[2].plot(times, cmd_w, label="cmd_w", linewidth=1.1)
    axes[2].set_ylabel("w (rad/s)")
    axes[2].grid(True, alpha=0.3)
    axes[2].legend(loc="best")

    if any(value is not None for value in w_error):
        axes[3].plot(times, w_error, label="w_error", color="tab:orange", linewidth=1.4)
    if any(value is not None for value in k_w):
        axes[3].plot(times, k_w, label="k_w", color="tab:purple", linewidth=1.2)
    axes[3].axhline(0.0, color="black", linewidth=0.8, alpha=0.5)
    axes[3].set_ylabel("error / gain")
    axes[3].set_xlabel("time (s)")
    axes[3].grid(True, alpha=0.3)
    axes[3].legend(loc="best")

    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)
    return output_path, len(samples)


class CircleAngleRecorder(Node):
    def __init__(self, args):
        super().__init__("circle_angle_recorder")
        self.args = args
        self.start_time = self.get_clock().now()
        self.stop_requested = False
        self.start_attempts = 0
        self.latest_status = parse_status("")
        self.status_count = 0
        self.rows_written = 0

        output_path = Path(args.output).expanduser()
        if output_path.is_dir() or str(args.output).endswith("/"):
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output_path = output_path / f"circle_angle_{stamp}.csv"
        output_path.parent.mkdir(parents=True, exist_ok=True)
        self.output_path = output_path
        self.csv_file = output_path.open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.csv_file, fieldnames=self.fieldnames())
        self.writer.writeheader()

        self.status_sub = self.create_subscription(
            String, args.status_topic, self.on_status, 50
        )
        self.command_pub = self.create_publisher(String, args.command_topic, 10)

        self.sample_timer = self.create_timer(1.0 / max(args.sample_hz, 0.1), self.write_sample)
        self.info_timer = self.create_timer(1.0, self.print_runtime_info)
        if args.send_start:
            self.start_timer = self.create_timer(0.2, self.send_start_until_running)

    @staticmethod
    def fieldnames():
        return [
            "time_s",
            "status_age_s",
            "running",
            "reason",
            "radius_m",
            "clockwise",
            "linear_speed_m_s",
            "lookahead_m",
            "k_w",
            "k_w_rate",
            "k_i_rate",
            "k_d_rate",
            "pose_x",
            "pose_y",
            "pose_yaw_rad",
            "center_x",
            "center_y",
            "target_x",
            "target_y",
            "target_yaw_rad",
            "yaw_error_rad",
            "target_w_rad_s",
            "measured_w_rad_s",
            "w_error_rad_s",
            "w_error_integral_rad",
            "w_error_derivative_rad_s2",
            "cmd_w_rad_s",
            "w_max_rad_s",
            "pose_fresh",
            "odom_fresh",
            "status_raw",
        ]

    def on_status(self, msg):
        self.latest_status = parse_status(msg.data)
        self.latest_status["stamp"] = self.get_clock().now()
        self.status_count += 1

    def send_start_until_running(self):
        if self.latest_status.get("running") == "true":
            if hasattr(self, "start_timer"):
                self.start_timer.cancel()
            return
        if self.start_attempts >= self.args.start_repeats:
            if hasattr(self, "start_timer"):
                self.start_timer.cancel()
            self.get_logger().warning(
                f"start command was sent {self.start_attempts} times but status is not running"
            )
            return
        self.start_attempts += 1
        self.publish_command("start")
        self.get_logger().info(f"sent start command attempt {self.start_attempts}")

    def publish_command(self, command):
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)

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

        s = self.latest_status or {}
        row = {
            "time_s": f"{elapsed:.3f}",
            "status_age_s": float_or_blank(self.age_seconds(s, now)),
            "running": s.get("running") or "",
            "reason": s.get("reason") or "",
            "radius_m": float_or_blank(s.get("radius")),
            "clockwise": s.get("clockwise") or "",
            "linear_speed_m_s": float_or_blank(s.get("speed")),
            "lookahead_m": float_or_blank(s.get("lookahead")),
            "k_w": float_or_blank(s.get("k_w")),
            "k_w_rate": float_or_blank(s.get("k_w_rate")),
            "k_i_rate": float_or_blank(s.get("k_i_rate")),
            "k_d_rate": float_or_blank(s.get("k_d_rate")),
            "pose_x": float_or_blank(s.get("pose_x")),
            "pose_y": float_or_blank(s.get("pose_y")),
            "pose_yaw_rad": float_or_blank(s.get("pose_yaw")),
            "center_x": float_or_blank(s.get("center_x")),
            "center_y": float_or_blank(s.get("center_y")),
            "target_x": float_or_blank(s.get("target_x")),
            "target_y": float_or_blank(s.get("target_y")),
            "target_yaw_rad": float_or_blank(s.get("target_yaw")),
            "yaw_error_rad": float_or_blank(s.get("yaw_error")),
            "target_w_rad_s": float_or_blank(s.get("target_w")),
            "measured_w_rad_s": float_or_blank(s.get("measured_w")),
            "w_error_rad_s": float_or_blank(s.get("w_error")),
            "w_error_integral_rad": float_or_blank(s.get("w_error_integral")),
            "w_error_derivative_rad_s2": float_or_blank(s.get("w_error_derivative")),
            "cmd_w_rad_s": float_or_blank(s.get("cmd_w")),
            "w_max_rad_s": float_or_blank(s.get("w_max")),
            "pose_fresh": s.get("pose_fresh") or "",
            "odom_fresh": s.get("odom_fresh") or "",
            "status_raw": s.get("status_raw") or "",
        }
        self.writer.writerow(row)
        self.rows_written += 1

    def print_runtime_info(self):
        s = self.latest_status or {}
        yaw_error = s.get("yaw_error")
        target_w = s.get("target_w")
        measured_w = s.get("measured_w")
        self.get_logger().info(
            "recording "
            f"{self.output_path} | "
            f"yaw_error={'-' if yaw_error is None else f'{yaw_error:.3f}'} "
            f"target_w={'-' if target_w is None else f'{target_w:.3f}'} "
            f"measured_w={'-' if measured_w is None else f'{measured_w:.3f}'} "
            f"k_w={'-' if s.get('k_w') is None else f'{s['k_w']:.3f}'} "
            f"reason={s.get('reason') or '-'}"
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
            "status_msgs": self.status_count,
        }


def build_arg_parser():
    parser = argparse.ArgumentParser(
        description="Record circle angle-loop yaw and angular-rate tracking."
    )
    default_output = (
        Path(__file__).resolve().parent
        / "log"
        / f"circle_angle_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    )
    parser.add_argument("--output", default=str(default_output), help="CSV file or directory")
    parser.add_argument("--duration-s", type=float, default=0.0, help="0 means record until Ctrl-C")
    parser.add_argument("--sample-hz", type=float, default=20.0)
    parser.add_argument("--status-topic", default="/car/circle_angle_tuner/status")
    parser.add_argument("--command-topic", default="/car/circle_angle_tuner/command")
    parser.add_argument("--send-start", action="store_true", help="publish start after recorder is ready")
    parser.add_argument("--stop-on-exit", action="store_true", help="publish stop when the recorder exits")
    parser.add_argument("--start-repeats", type=int, default=20)
    parser.add_argument("--stop-repeats", type=int, default=10)
    parser.add_argument("--no-plot", action="store_false", dest="plot", help="do not write a PNG plot")
    parser.add_argument("--plot-output", default="", help="PNG file or directory")
    parser.add_argument("--plot-only", default="", help="plot an existing CSV and exit")
    parser.set_defaults(plot=True)
    return parser


def main():
    args = build_arg_parser().parse_args()
    if args.plot_only:
        output_path, sample_count = plot_circle_angle_csv(args.plot_only, args.plot_output)
        print(f"plotted {sample_count} samples: {output_path}")
        return

    rclpy.init()
    recorder = CircleAngleRecorder(args)

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

    print("\nCircle-angle record summary")
    print(f"  csv: {summary['output_path']}")
    print(f"  messages: status={summary['status_msgs']} rows={summary['rows']}")
    if args.plot:
        try:
            output_path, sample_count = plot_circle_angle_csv(
                summary["output_path"], args.plot_output
            )
            print(f"  plot: {output_path} ({sample_count} samples)")
        except RuntimeError as exc:
            print(f"  plot skipped: {exc}")


if __name__ == "__main__":
    main()
