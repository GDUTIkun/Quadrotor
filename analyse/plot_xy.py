#!/usr/bin/env python3
"""Plot pose and target x-y tracks from recorder CSV files."""

import argparse
import csv
from pathlib import Path


def to_float(value):
    if value in (None, ""):
        return None
    try:
        return float(value)
    except ValueError:
        return None


def collect_xy(rows, x_field, y_field):
    points = []
    for row in rows:
        x = to_float(row.get(x_field))
        y = to_float(row.get(y_field))
        if x is not None and y is not None:
            points.append((x, y))
    return points


def decimate(points, max_points):
    if max_points <= 0 or len(points) <= max_points:
        return points
    step = max(1, len(points) // max_points)
    return points[::step]


def resolve_output(csv_path, output):
    if not output:
        return csv_path.with_name(f"{csv_path.stem}_xy.png")
    output_path = Path(output).expanduser()
    if output_path.is_dir() or str(output).endswith("/"):
        return output_path / f"{csv_path.stem}_xy.png"
    return output_path


def plot_xy(csv_path, output="", max_points=0, target_point_size=10.0, pose_point_size=0.0):
    csv_path = Path(csv_path).expanduser()
    output_path = resolve_output(csv_path, output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with csv_path.open(newline="", encoding="utf-8") as csv_file:
        rows = list(csv.DictReader(csv_file))

    pose = decimate(collect_xy(rows, "pose_x", "pose_y"), max_points)
    target = decimate(collect_xy(rows, "target_x", "target_y"), max_points)
    center = collect_xy(rows, "center_x", "center_y")

    if not pose and not target:
        raise RuntimeError(f"No pose_x/pose_y or target_x/target_y samples in {csv_path}")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(8, 8))
    fig.suptitle(f"X-Y Track: {csv_path.name}")

    if target:
        tx, ty = zip(*target)
        ax.plot(tx, ty, label="target path", linewidth=1.2, color="tab:blue", alpha=0.75)
        if target_point_size > 0.0:
            ax.scatter(
                tx, ty, label="target points", s=target_point_size,
                color="tab:blue", alpha=0.7
            )
        ax.scatter([tx[0]], [ty[0]], label="target start", s=36, color="tab:cyan")
    if pose:
        px, py = zip(*pose)
        ax.plot(px, py, label="pose path", linewidth=1.6, color="tab:orange")
        if pose_point_size > 0.0:
            ax.scatter(
                px, py, label="pose points", s=pose_point_size,
                color="tab:orange", alpha=0.55
            )
        ax.scatter([px[0]], [py[0]], label="pose start", s=36, color="tab:green")
        ax.scatter([px[-1]], [py[-1]], label="pose end", s=36, color="tab:red")
    if center:
        cx, cy = center[-1]
        ax.scatter([cx], [cy], label="circle center", marker="x", s=70, color="black")

    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)
    return output_path, len(pose), len(target)


def build_arg_parser():
    parser = argparse.ArgumentParser(description="Plot x-y tracks from recorder CSV files.")
    parser.add_argument("csv", help="Input CSV path")
    parser.add_argument("--output", default="", help="PNG file or output directory")
    parser.add_argument(
        "--max-points",
        type=int,
        default=0,
        help="Optional point decimation for very large CSVs; 0 keeps all points",
    )
    parser.add_argument("--target-point-size", type=float, default=10.0)
    parser.add_argument("--pose-point-size", type=float, default=0.0)
    return parser


def main():
    args = build_arg_parser().parse_args()
    output_path, pose_count, target_count = plot_xy(
        args.csv,
        output=args.output,
        max_points=args.max_points,
        target_point_size=args.target_point_size,
        pose_point_size=args.pose_point_size,
    )
    print(f"wrote {output_path} pose_points={pose_count} target_points={target_count}")


if __name__ == "__main__":
    main()
