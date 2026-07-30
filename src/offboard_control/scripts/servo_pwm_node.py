#!/usr/bin/env python3
"""ROS 2 node controlling a servo on Pi 5 GPIO18 via RP1 hardware PWM."""

from pathlib import Path
from time import sleep

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class Rp1ServoPwm:
    PWM_DEVICE = "pwm@98000"
    PWM_CHANNEL = 2
    PERIOD_NS = 20_000_000

    def __init__(self, min_pulse_ns: int, max_pulse_ns: int) -> None:
        self.min_pulse_ns = min_pulse_ns
        self.max_pulse_ns = max_pulse_ns
        self.pwm = self._export_channel(self._find_pwm_chip())
        self._configure()

    @staticmethod
    def _read_int(path: Path) -> int:
        return int(path.read_text(encoding="ascii").strip())

    @staticmethod
    def _write_int(path: Path, value: int) -> None:
        path.write_text(str(value), encoding="ascii")

    def _find_pwm_chip(self) -> Path:
        expected = f"OF_FULLNAME=/axi/pcie@120000/rp1/{self.PWM_DEVICE}"
        for chip in sorted(Path("/sys/class/pwm").glob("pwmchip*")):
            uevent = chip / "device/uevent"
            if uevent.exists() and expected in uevent.read_text(encoding="ascii"):
                return chip
        raise RuntimeError(f"RP1 {self.PWM_DEVICE} PWM controller not found")

    def _export_channel(self, chip: Path) -> Path:
        pwm = chip / f"pwm{self.PWM_CHANNEL}"
        if not pwm.exists():
            self._write_int(chip / "export", self.PWM_CHANNEL)
            for _ in range(100):
                if pwm.exists():
                    break
                sleep(0.01)
            else:
                raise RuntimeError(f"{pwm} did not appear after export")
        return pwm

    def _configure(self) -> None:
        if self._read_int(self.pwm / "enable") != 0:
            self._write_int(self.pwm / "enable", 0)
        if self._read_int(self.pwm / "duty_cycle") != 0:
            self._write_int(self.pwm / "duty_cycle", 0)
        self._write_int(self.pwm / "period", self.PERIOD_NS)
        if (self.pwm / "polarity").exists():
            (self.pwm / "polarity").write_text("normal", encoding="ascii")

    def set_angle(self, angle_deg: float) -> None:
        duty_ns = round(
            self.min_pulse_ns
            + angle_deg / 180.0 * (self.max_pulse_ns - self.min_pulse_ns)
        )
        self._write_int(self.pwm / "duty_cycle", duty_ns)
        if self._read_int(self.pwm / "enable") == 0:
            self._write_int(self.pwm / "enable", 1)

    def close(self) -> None:
        if self._read_int(self.pwm / "enable") != 0:
            self._write_int(self.pwm / "enable", 0)


class ServoPwmNode(Node):
    def __init__(self) -> None:
        super().__init__("servo_pwm_node")
        self.declare_parameter("initial_angle", 90.0)
        self.declare_parameter("min_pulse_us", 1000)
        self.declare_parameter("max_pulse_us", 2000)
        initial_angle = float(self.get_parameter("initial_angle").value)
        min_pulse_ns = int(self.get_parameter("min_pulse_us").value) * 1000
        max_pulse_ns = int(self.get_parameter("max_pulse_us").value) * 1000

        if not 0.0 <= initial_angle <= 180.0:
            raise ValueError("initial_angle must be between 0 and 180")
        if not 0 < min_pulse_ns < max_pulse_ns < Rp1ServoPwm.PERIOD_NS:
            raise ValueError("invalid servo pulse width parameters")

        self.servo = Rp1ServoPwm(min_pulse_ns, max_pulse_ns)
        self.servo.set_angle(initial_angle)
        self.subscription = self.create_subscription(
            Float64, "/servo/angle_deg", self._angle_callback, 10
        )
        self.get_logger().info(
            f"GPIO18 RP1 PWM ready; initial angle {initial_angle:.1f} deg, "
            "listening on /servo/angle_deg"
        )

    def _angle_callback(self, message: Float64) -> None:
        angle = float(message.data)
        if not 0.0 <= angle <= 180.0:
            self.get_logger().warning(
                f"Ignoring angle {angle:.1f}; valid range is 0..180 degrees"
            )
            return
        try:
            self.servo.set_angle(angle)
            self.get_logger().info(f"Servo angle set to {angle:.1f} deg")
        except OSError as error:
            self.get_logger().error(f"Failed to update PWM: {error}")

    def destroy_node(self) -> bool:
        self.servo.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ServoPwmNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
