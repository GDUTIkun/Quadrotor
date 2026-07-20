#!/usr/bin/env python3

import argparse
import os
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def sign(value):
    if value > 0.0:
        return 1.0
    if value < 0.0:
        return -1.0
    return 0.0


def simulate(args):
    times = []
    raw = []
    lowpass = []
    rate_limited = []

    lp_value = args.initial
    rate_value = args.initial
    tau = args.lowpass_tau
    alpha = args.dt / (tau + args.dt) if tau > 0.0 else 1.0

    steps = int(args.duration / args.dt) + 1
    for i in range(steps):
        t = i * args.dt
        target = args.target if t >= args.step_time else args.initial

        lp_value += alpha * (target - lp_value)

        delta = target - rate_value
        max_delta = args.max_rate * args.dt
        if abs(delta) <= max_delta:
            rate_value = target
        else:
            rate_value += sign(delta) * max_delta

        times.append(t)
        raw.append(target)
        lowpass.append(lp_value)
        rate_limited.append(rate_value)

    return times, raw, lowpass, rate_limited


def derivative(values, dt):
    out = [0.0]
    for i in range(1, len(values)):
        out.append((values[i] - values[i - 1]) / dt)
    return out


def main():
    parser = argparse.ArgumentParser(
        description="Plot raw step input, first-order low-pass input, and rate-limited input."
    )
    parser.add_argument("--target", type=float, default=1.5, help="Step target position.")
    parser.add_argument("--initial", type=float, default=0.0, help="Initial position input.")
    parser.add_argument("--step-time", type=float, default=1.0, help="Step time in seconds.")
    parser.add_argument("--duration", type=float, default=8.0, help="Simulation duration in seconds.")
    parser.add_argument("--dt", type=float, default=0.02, help="Simulation time step in seconds.")
    parser.add_argument("--lowpass-tau", type=float, default=0.8, help="Low-pass time constant in seconds.")
    parser.add_argument("--max-rate", type=float, default=0.6, help="Position input slew rate limit in m/s.")
    parser.add_argument(
        "-o",
        "--output",
        default="/home/t/flight_test/bags/input_shaping_compare.png",
        help="Output PNG path.",
    )
    args = parser.parse_args()

    times, raw, lowpass, rate_limited = simulate(args)
    raw_vel = derivative(raw, args.dt)
    lowpass_vel = derivative(lowpass, args.dt)
    rate_vel = derivative(rate_limited, args.dt)

    output = Path(os.path.expanduser(args.output))
    output.parent.mkdir(parents=True, exist_ok=True)

    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)

    axes[0].plot(times, raw, label="Raw step input", linewidth=1.8)
    axes[0].plot(times, lowpass, label=f"Low-pass input, tau={args.lowpass_tau:.2f}s", linewidth=1.8)
    axes[0].plot(times, rate_limited, label=f"Rate-limited input, max_rate={args.max_rate:.2f}m/s", linewidth=1.8)
    axes[0].set_ylabel("Position input x [m]")
    axes[0].grid(True, alpha=0.35)
    axes[0].legend(loc="best")

    axes[1].plot(times, raw_vel, label="Raw input velocity", linewidth=1.3)
    axes[1].plot(times, lowpass_vel, label="Low-pass equivalent velocity", linewidth=1.5)
    axes[1].plot(times, rate_vel, label="Rate-limited equivalent velocity", linewidth=1.5)
    axes[1].set_ylabel("Input slope dx/dt [m/s]")
    axes[1].set_xlabel("Time [s]")
    axes[1].grid(True, alpha=0.35)
    axes[1].legend(loc="best")

    fig.suptitle(f"Input shaping comparison: x {args.initial:.1f} -> {args.target:.1f} m")
    fig.tight_layout()
    fig.savefig(output, dpi=160)
    plt.close(fig)

    print(output)


if __name__ == "__main__":
    main()
