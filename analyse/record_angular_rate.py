#!/usr/bin/env python3
"""Record target and current angular velocity during angular_rate_tuner tests."""

import argparse
import csv
import re
import signal
import time
from datetime import datetime
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


STATUS_RE = {
    "running": re.compile(r"\brunning=([^\s]+)"),
    "k_w_rate": re.compile(r"\bk_w_rate=([-+0-9.eE]+)"),
    "k_i_rate": re.compile(r"\bk_i_rate=([-+0-9.eE]+)"),
    "speed": re.compile(r"\bspeed=([-+0-9.eE]+)"),
    "target_w": re.compile(r"\btarget_w=([-+0-9.eE]+)"),
    "measured_w": re.compile(r"\bmeasured_w=([-+0-9.eE]+)"),
    "w_error": re.compile(r"\bw_error=([-+0-9.eE]+)"),
    "w_error_integral": re.compile(r"\bw_error_integral=([-+0-9.eE]+)"),
    "cmd_w": re.compile(r"\bcmd_w=([-+0-9.eE]+)"),
    "w_max": re.compile(r"\bw_max=([-+0-9.eE]+)"),
    "odom_fresh": re.compile(r"\bodom_fresh=([^\s]+)"),
    "reason": re.compile(r"\breason=([^\s]+)"),
}


def float_or_blank(value):
    if value is None:
        return ""
    return f"{value:.9g}"


def parse_status(text):
    parsed = {"status_raw": text}
    for key, pattern in STATUS_RE.items():
        match = pattern.search(text)
        if not match:
            parsed[key] = None
            continue
        if key in ("running", "odom_fresh", "reason"):
            parsed[key] = match.group(1)
        else:
            parsed[key] = float(match.group(1))
    return parsed


def to_float(value):
    if value in (None, ""):
        return None
    try:
        return float(value)
    except ValueError:
        return None


def resolve_plot_output(csv_path, plot_output):
    if plot_output is None:
        return csv_path.with_suffix(".png")
    output_path = Path(plot_output).expanduser()
    if output_path.is_dir() or str(plot_output).endswith("/"):
        return output_path / f"{csv_path.stem}.png"
    return output_path


def plot_angular_rate_csv(csv_path, plot_output=None):
    csv_path = Path(csv_path).expanduser()
    output_path = resolve_plot_output(csv_path, plot_output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with csv_path.open(newline="", encoding="utf-8") as csv_file:
        rows = list(csv.DictReader(csv_file))

    samples = []
    for row in rows:
        time_s = to_float(row.get("time_s"))
        target_w = to_float(row.get("target_w_rad_s"))
        current_w = to_float(row.get("current_w_rad_s"))
        if current_w is None:
            current_w = to_float(row.get("odom_current_w_rad_s"))
        if time_s is None or target_w is None or current_w is None:
            continue
        samples.append({
            "time_s": time_s,
            "target_w": target_w,
            "current_w": current_w,
            "w_error": to_float(row.get("w_error_rad_s")),
            "cmd_w": to_float(row.get("cmd_w_rad_s")),
            "k_w_rate": to_float(row.get("k_w_rate")),
            "k_i_rate": to_float(row.get("k_i_rate")),
            "w_error_integral": to_float(row.get("w_error_integral_rad")),
            "reason": row.get("reason", ""),
        })

    if not samples:
        raise RuntimeError(f"No plottable angular-rate samples in {csv_path}")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    times = [sample["time_s"] for sample in samples]
    target = [sample["target_w"] for sample in samples]
    current = [sample["current_w"] for sample in samples]
    error = [
        sample["w_error"] if sample["w_error"] is not None else
        sample["target_w"] - sample["current_w"]
        for sample in samples
    ]
    cmd = [sample["cmd_w"] for sample in samples]
    k_w_rate = [sample["k_w_rate"] for sample in samples]
    k_i_rate = [sample["k_i_rate"] for sample in samples]
    error_integral = [sample["w_error_integral"] for sample in samples]

    fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
    fig.suptitle(f"Angular Rate Tracking: {csv_path.name}")

    axes[0].plot(times, target, label="target_w", linewidth=1.8)
    axes[0].plot(times, current, label="current_w", linewidth=1.5)
    axes[0].set_ylabel("w (rad/s)")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="best")

    axes[1].plot(times, error, label="w_error", color="tab:red", linewidth=1.5)
    if any(value is not None for value in cmd):
        axes[1].plot(times, cmd, label="cmd_w", color="tab:green", linewidth=1.2)
    axes[1].axhline(0.0, color="black", linewidth=0.8, alpha=0.5)
    axes[1].set_ylabel("rad/s")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc="best")

    if any(value is not None for value in error_integral):
        axes[2].plot(
            times, error_integral, label="error_integral", color="tab:orange", linewidth=1.5
        )
        axes[2].legend(loc="best")
    axes[2].axhline(0.0, color="black", linewidth=0.8, alpha=0.5)
    axes[2].set_ylabel("integral (rad)")
    axes[2].grid(True, alpha=0.3)

    if any(value is not None for value in k_w_rate):
        axes[3].plot(times, k_w_rate, label="k_w_rate", color="tab:purple", linewidth=1.5)
    if any(value is not None for value in k_i_rate):
        axes[3].plot(times, k_i_rate, label="k_i_rate", color="tab:brown", linewidth=1.5)
    axes[3].set_ylabel("gain")
    axes[3].set_xlabel("time (s)")
    axes[3].grid(True, alpha=0.3)
    axes[3].legend(loc="best")

    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)
    return output_path, len(samples)


class AngularRateRecorder(Node):
    def __init__(self, args):
        super().__init__("angular_rate_recorder")
        self.args = args
        self.start_time = self.get_clock().now()
        self.stop_requested = False

        self.latest_status = parse_status("")
        self.latest_odom = {}
        self.status_count = 0
        self.odom_count = 0
        self.rows_written = 0
        self.start_attempts = 0

        output_path = Path(args.output).expanduser()
        if output_path.is_dir() or str(args.output).endswith("/"):
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output_path = output_path / f"angular_rate_{stamp}.csv"
        output_path.parent.mkdir(parents=True, exist_ok=True)
        self.output_path = output_path
        self.csv_file = output_path.open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.csv_file, fieldnames=self.fieldnames())
        self.writer.writeheader()

        self.status_sub = self.create_subscription(
            String, args.status_topic, self.on_status, 50
        )
        self.odom_sub = self.create_subscription(Odometry, args.odom_topic, self.on_odom, 50)
        self.command_pub = self.create_publisher(String, args.command_topic, 10)

        period = 1.0 / max(args.sample_hz, 0.1)
        self.sample_timer = self.create_timer(period, self.write_sample)
        self.info_timer = self.create_timer(1.0, self.print_runtime_info)

        if args.send_start:
            self.start_timer = self.create_timer(0.2, self.send_start_until_running)

    @staticmethod
    def fieldnames():
        return [
            "time_s",
            "status_age_s",
            "odom_age_s",
            "running",
            "k_w_rate",
            "k_i_rate",
            "linear_speed_m_s",
            "target_w_rad_s",
            "current_w_rad_s",
            "odom_current_w_rad_s",
            "w_error_rad_s",
            "w_error_integral_rad",
            "cmd_w_rad_s",
            "w_max_rad_s",
            "odom_fresh",
            "reason",
            "status_raw",
        ]

    def on_status(self, msg):
        self.latest_status = parse_status(msg.data)
        self.latest_status["stamp"] = self.get_clock().now()
        self.status_count += 1

    def on_odom(self, msg):
        self.latest_odom = {
            "stamp": self.get_clock().now(),
            "current_w": msg.twist.twist.angular.z,
        }
        self.odom_count += 1

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

        status = self.latest_status or {}
        odom = self.latest_odom or {}
        current_w = status.get("measured_w")
        if current_w is None:
            current_w = odom.get("current_w")

        row = {
            "time_s": f"{elapsed:.3f}",
            "status_age_s": float_or_blank(self.age_seconds(status, now)),
            "odom_age_s": float_or_blank(self.age_seconds(odom, now)),
            "running": status.get("running") or "",
            "k_w_rate": float_or_blank(status.get("k_w_rate")),
            "k_i_rate": float_or_blank(status.get("k_i_rate")),
            "linear_speed_m_s": float_or_blank(status.get("speed")),
            "target_w_rad_s": float_or_blank(status.get("target_w")),
            "current_w_rad_s": float_or_blank(current_w),
            "odom_current_w_rad_s": float_or_blank(odom.get("current_w")),
            "w_error_rad_s": float_or_blank(status.get("w_error")),
            "w_error_integral_rad": float_or_blank(status.get("w_error_integral")),
            "cmd_w_rad_s": float_or_blank(status.get("cmd_w")),
            "w_max_rad_s": float_or_blank(status.get("w_max")),
            "odom_fresh": status.get("odom_fresh") or "",
            "reason": status.get("reason") or "",
            "status_raw": status.get("status_raw") or "",
        }
        self.writer.writerow(row)
        self.rows_written += 1

    def print_runtime_info(self):
        status = self.latest_status or {}
        target_w = status.get("target_w")
        current_w = status.get("measured_w")
        if current_w is None:
            current_w = self.latest_odom.get("current_w")
        target_text = "-" if target_w is None else f"{target_w:.3f}"
        current_text = "-" if current_w is None else f"{current_w:.3f}"
        cmd_w = status.get("cmd_w")
        cmd_text = "-" if cmd_w is None else f"{cmd_w:.3f}"
        k_w_rate = status.get("k_w_rate")
        kp_text = "-" if k_w_rate is None else f"{k_w_rate:.3f}"
        k_i_rate = status.get("k_i_rate")
        ki_text = "-" if k_i_rate is None else f"{k_i_rate:.3f}"
        self.get_logger().info(
            "recording "
            f"{self.output_path} | "
            f"target_w={target_text} "
            f"current_w={current_text} "
            f"cmd_w={cmd_text} "
            f"k_w_rate={kp_text} "
            f"k_i_rate={ki_text} "
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
            "status_msgs": self.status_count,
            "odom_msgs": self.odom_count,
        }


def build_arg_parser():
    parser = argparse.ArgumentParser(
        description=(
            "Record target and current angular velocity from angular_rate_tuner "
            "status and /car/odom/carto."
        )
    )
    default_output = (
        Path(__file__).resolve().parent
        / "log"
        / f"angular_rate_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    )
    parser.add_argument("--output", default=str(default_output), help="CSV file or directory")
    parser.add_argument("--duration-s", type=float, default=0.0, help="0 means record until Ctrl-C")
    parser.add_argument("--sample-hz", type=float, default=20.0)
    parser.add_argument("--status-topic", default="/car/angular_rate_tuner/status")
    parser.add_argument("--odom-topic", default="/car/odom/carto")
    parser.add_argument("--command-topic", default="/car/angular_rate_tuner/command")
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
    plot_output = args.plot_output or None
    if args.plot_only:
        output_path, sample_count = plot_angular_rate_csv(args.plot_only, plot_output)
        print(f"plotted {sample_count} samples: {output_path}")
        return

    rclpy.init()
    recorder = AngularRateRecorder(args)

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

    print("\nAngular-rate record summary")
    print(f"  csv: {summary['output_path']}")
    print(
        "  messages: "
        f"status={summary['status_msgs']} odom={summary['odom_msgs']} rows={summary['rows']}"
    )
    if args.plot:
        try:
            output_path, sample_count = plot_angular_rate_csv(summary["output_path"], plot_output)
            print(f"  plot: {output_path} ({sample_count} samples)")
        except RuntimeError as exc:
            print(f"  plot skipped: {exc}")


if __name__ == "__main__":
    main()
