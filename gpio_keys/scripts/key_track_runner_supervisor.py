#!/usr/bin/env python3
"""Start or stop track_runner_offboard.launch.py according to two GPIO key states."""

from __future__ import annotations

import os
import shlex
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class KeyTrackRunnerSupervisor(Node):
    def __init__(self) -> None:
        super().__init__("key_track_runner_supervisor")
        self.key1_topic = self.declare_parameter("key1_topic", "/gpio_keys/key1").value
        self.key2_topic = self.declare_parameter("key2_topic", "/gpio_keys/key2").value
        self.target_package = self.declare_parameter("target_package", "car_bringup").value
        self.target_launch = self.declare_parameter(
            "target_launch", "track_runner_offboard.launch.py"
        ).value
        self.target_args = self.declare_parameter("target_args", "").value
        self.start_delay_s = float(self.declare_parameter("start_delay_s", 3.0).value)
        self.shutdown_timeout_s = float(
            self.declare_parameter("shutdown_timeout_s", 5.0).value
        )
        self.stop_command_topic = self.declare_parameter(
            "stop_command_topic", "/car/track_runner/command"
        ).value
        self.publish_stop_before_kill = self.to_bool(
            self.declare_parameter("publish_stop_before_kill", True).value
        )

        if self.start_delay_s < 0.0:
            raise ValueError("start_delay_s must be >= 0")
        if self.shutdown_timeout_s <= 0.0:
            raise ValueError("shutdown_timeout_s must be > 0")

        self.key1: bool | None = None
        self.key2: bool | None = None
        self.pending_start_at: float | None = None
        self.process: subprocess.Popen | None = None

        self.stop_pub = self.create_publisher(String, self.stop_command_topic, 10)
        self.create_subscription(Bool, self.key1_topic, self.on_key1, 10)
        self.create_subscription(Bool, self.key2_topic, self.on_key2, 10)
        self.timer = self.create_timer(0.1, self.tick)

        self.get_logger().info(
            f"waiting for {self.key1_topic} and {self.key2_topic}; "
            f"xor=true for {self.start_delay_s:.1f}s starts "
            f"{self.target_package} {self.target_launch}, equal states stop it"
        )

    def to_bool(self, value: object) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            normalized = value.strip().lower()
            if normalized in ("1", "true", "yes", "on"):
                return True
            if normalized in ("0", "false", "no", "off"):
                return False
        return bool(value)

    def on_key1(self, message: Bool) -> None:
        self.key1 = bool(message.data)
        self.evaluate_keys()

    def on_key2(self, message: Bool) -> None:
        self.key2 = bool(message.data)
        self.evaluate_keys()

    def have_both_keys(self) -> bool:
        return self.key1 is not None and self.key2 is not None

    def keys_are_exclusive(self) -> bool:
        return bool(self.key1) != bool(self.key2)

    def target_is_running(self) -> bool:
        return self.process is not None and self.process.poll() is None

    def evaluate_keys(self) -> None:
        if not self.have_both_keys():
            return

        if self.keys_are_exclusive():
            if self.target_is_running() or self.pending_start_at is not None:
                return
            self.pending_start_at = time.monotonic()
            self.get_logger().info(
                f"detected key1={self.key1} key2={self.key2}; "
                f"starting target launch after {self.start_delay_s:.1f}s"
            )
            return

        if self.pending_start_at is not None:
            self.pending_start_at = None
            self.get_logger().info(f"cancelled pending start: key1={self.key1} key2={self.key2}")
        if self.target_is_running():
            self.stop_target(f"key1={self.key1} key2={self.key2}")

    def tick(self) -> None:
        if self.process is not None and self.process.poll() is not None:
            return_code = self.process.returncode
            self.process = None
            self.get_logger().warn(f"target launch exited with code {return_code}")
            if self.have_both_keys() and self.keys_are_exclusive():
                self.pending_start_at = time.monotonic()
                self.get_logger().info(
                    f"keys are still exclusive; retrying after {self.start_delay_s:.1f}s"
                )

        if self.pending_start_at is None:
            return
        if not self.have_both_keys() or not self.keys_are_exclusive():
            self.pending_start_at = None
            return
        if time.monotonic() - self.pending_start_at >= self.start_delay_s:
            self.pending_start_at = None
            self.start_target()

    def start_target(self) -> None:
        if self.target_is_running():
            return
        command = ["ros2", "launch", str(self.target_package), str(self.target_launch)]
        if str(self.target_args).strip():
            command.extend(shlex.split(str(self.target_args)))

        self.get_logger().info(f"starting target launch: {' '.join(command)}")
        self.process = subprocess.Popen(
            command,
            env=os.environ.copy(),
            preexec_fn=os.setsid,
        )

    def publish_stop(self) -> None:
        message = String()
        message.data = "stop"
        for _ in range(3):
            self.stop_pub.publish(message)
            time.sleep(0.05)

    def stop_target(self, reason: str) -> None:
        process = self.process
        if process is None:
            return

        if rclpy.ok():
            self.get_logger().info(f"stopping target launch because {reason}")
        if self.publish_stop_before_kill and rclpy.ok():
            self.publish_stop()

        try:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=self.shutdown_timeout_s)
        except ProcessLookupError:
            pass
        except subprocess.TimeoutExpired:
            if rclpy.ok():
                self.get_logger().warn("target launch did not stop after SIGINT; sending SIGTERM")
            try:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=self.shutdown_timeout_s)
            except ProcessLookupError:
                pass
        finally:
            self.process = None

    def destroy_node(self) -> bool:
        if self.target_is_running():
            self.stop_target("supervisor is shutting down")
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = KeyTrackRunnerSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
