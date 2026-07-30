#!/usr/bin/env python3
"""Control a positional servo on GPIO18 using Raspberry Pi hardware PWM."""

from pathlib import Path
from time import sleep

PWM_DRIVER = "pwm-brcmstb"
PWM_CHANNEL = 0
PERIOD_NS = 20_000_000
MIN_PULSE_NS = 1_000_000
MAX_PULSE_NS = 2_000_000


def write_value(path: Path, value: int) -> None:
    path.write_text(str(value), encoding="ascii")


def find_pwm_chip() -> Path:
    for chip in sorted(Path("/sys/class/pwm").glob("pwmchip*")):
        uevent = chip / "device/uevent"
        if uevent.exists() and f"DRIVER={PWM_DRIVER}" in uevent.read_text():
            return chip
    raise RuntimeError(
        "Hardware PWM controller not found; enable the GPIO18 PWM overlay and reboot"
    )


def export_channel(chip: Path) -> Path:
    pwm = chip / f"pwm{PWM_CHANNEL}"
    if not pwm.exists():
        write_value(chip / "export", PWM_CHANNEL)
        for _ in range(100):
            if pwm.exists():
                return pwm
            sleep(0.01)
        raise RuntimeError(f"{pwm} did not appear after export")
    return pwm


def angle_to_duty_ns(angle: float) -> int:
    return round(
        MIN_PULSE_NS + angle / 180.0 * (MAX_PULSE_NS - MIN_PULSE_NS)
    )


def main() -> None:
    chip = find_pwm_chip()
    pwm = export_channel(chip)

    if int((pwm / "enable").read_text().strip()) != 0:
        write_value(pwm / "enable", 0)
    if int((pwm / "duty_cycle").read_text().strip()) != 0:
        write_value(pwm / "duty_cycle", 0)
    write_value(pwm / "period", PERIOD_NS)
    if (pwm / "polarity").exists():
        (pwm / "polarity").write_text("normal", encoding="ascii")
    write_value(pwm / "duty_cycle", angle_to_duty_ns(90))
    write_value(pwm / "enable", 1)

    print(f"GPIO18 hardware PWM: {chip.name}, channel {PWM_CHANNEL}")
    print("Enter an angle from 0 to 180, or q to quit.")

    try:
        while True:
            command = input("angle> ").strip()
            if command.lower() in {"q", "quit", "exit"}:
                break
            try:
                angle = float(command)
            except ValueError:
                print("Please enter a number from 0 to 180.")
                continue
            if not 0.0 <= angle <= 180.0:
                print("Angle must be between 0 and 180 degrees.")
                continue
            write_value(pwm / "duty_cycle", angle_to_duty_ns(angle))
            print(f"Servo commanded to {angle:.1f} degrees.")
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        if int((pwm / "enable").read_text().strip()) != 0:
            write_value(pwm / "enable", 0)


if __name__ == "__main__":
    main()
