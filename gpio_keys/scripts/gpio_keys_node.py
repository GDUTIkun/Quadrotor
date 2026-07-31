#!/usr/bin/env python3
"""Read two Orange Pi GPIO keys and publish ROS 2 key events."""

from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


GPIO_ROOT = Path("/sys/class/gpio")


@dataclass
class KeyState:
    name: str
    gpio: int
    value_path: Path
    active_high: bool
    stable_pressed: bool
    raw_pressed: bool
    raw_changed_at: float
    last_published_at: float
    publisher: object


class GpioKeysNode(Node):
    def __init__(self) -> None:
        super().__init__("gpio_keys_node")
        self.key1_gpio = self.declare_parameter("key1_gpio", 46).value
        self.key2_gpio = self.declare_parameter("key2_gpio", 47).value
        self.active_high = self.to_bool(self.declare_parameter("active_high", True).value)
        self.debounce_ms = self.declare_parameter("debounce_ms", 30).value
        self.poll_hz = self.declare_parameter("poll_hz", 100.0).value
        self.state_publish_ms = self.declare_parameter("state_publish_ms", 200).value
        self.auto_export = self.declare_parameter("auto_export", True).value
        self.event_topic = self.declare_parameter("event_topic", "/gpio_keys/event").value
        self.key1_topic = self.declare_parameter("key1_topic", "/gpio_keys/key1").value
        self.key2_topic = self.declare_parameter("key2_topic", "/gpio_keys/key2").value

        if self.poll_hz <= 0.0:
            raise ValueError("poll_hz must be > 0")
        if self.debounce_ms < 0:
            raise ValueError("debounce_ms must be >= 0")
        if self.state_publish_ms <= 0:
            raise ValueError("state_publish_ms must be > 0")

        self.event_pub = self.create_publisher(String, self.event_topic, 10)
        self.keys = [
            self.open_key("key1", int(self.key1_gpio), self.key1_topic),
            self.open_key("key2", int(self.key2_gpio), self.key2_topic),
        ]

        period = 1.0 / float(self.poll_hz)
        self.timer = self.create_timer(period, self.poll_keys)
        self.get_logger().info(
            f"GPIO keys ready: key1=gpio{int(self.key1_gpio)} "
            f"key2=gpio{int(self.key2_gpio)} "
            f"active_{'high' if self.active_high else 'low'} "
            f"debounce={int(self.debounce_ms)}ms "
            f"state_publish={int(self.state_publish_ms)}ms"
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

    def open_key(self, name: str, gpio: int, topic: str) -> KeyState:
        gpio_dir = GPIO_ROOT / f"gpio{gpio}"
        if not gpio_dir.exists() and self.auto_export:
            try:
                (GPIO_ROOT / "export").write_text(f"{gpio}\n", encoding="ascii")
            except OSError as error:
                self.get_logger().error(
                    f"failed to export gpio{gpio}: {error}. "
                    "Run: sudo ros2 run gpio_keys register_gpio_keys.sh"
                )

        direction_path = gpio_dir / "direction"
        if direction_path.exists():
            try:
                direction_path.write_text("in\n", encoding="ascii")
            except OSError as error:
                self.get_logger().warn(f"failed to set gpio{gpio} direction=input: {error}")

        value_path = gpio_dir / "value"
        if not value_path.exists():
            raise RuntimeError(f"gpio{gpio} value path is not available")

        pressed = self.read_pressed(value_path)
        publisher = self.create_publisher(Bool, topic, 10)
        message = Bool()
        message.data = pressed
        publisher.publish(message)
        return KeyState(
            name=name,
            gpio=gpio,
            value_path=value_path,
            active_high=bool(self.active_high),
            stable_pressed=pressed,
            raw_pressed=pressed,
            raw_changed_at=time.monotonic(),
            last_published_at=time.monotonic(),
            publisher=publisher,
        )

    def read_pressed(self, value_path: Path) -> bool:
        value = value_path.read_text(encoding="ascii").strip()
        high = value == "1"
        return high if self.active_high else not high

    def poll_keys(self) -> None:
        now = time.monotonic()
        debounce_sec = float(self.debounce_ms) / 1000.0
        state_publish_sec = float(self.state_publish_ms) / 1000.0
        for key in self.keys:
            try:
                raw_pressed = self.read_pressed(key.value_path)
            except OSError as error:
                self.get_logger().error(f"failed to read gpio{key.gpio}: {error}")
                continue

            changed = False
            if raw_pressed != key.raw_pressed:
                key.raw_pressed = raw_pressed
                key.raw_changed_at = now
                continue

            if raw_pressed != key.stable_pressed and now - key.raw_changed_at >= debounce_sec:
                key.stable_pressed = raw_pressed
                changed = True

                event_msg = String()
                event_msg.data = f"{key.name}_{'pressed' if raw_pressed else 'released'}"
                self.event_pub.publish(event_msg)
                self.get_logger().info(
                    f"{key.name} gpio{key.gpio} {'pressed' if raw_pressed else 'released'}"
                )

            should_publish_state = changed or now - key.last_published_at >= state_publish_sec
            if should_publish_state:
                state_msg = Bool()
                state_msg.data = key.stable_pressed
                key.publisher.publish(state_msg)
                key.last_published_at = now


def main() -> None:
    rclpy.init()
    node = GpioKeysNode()
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
