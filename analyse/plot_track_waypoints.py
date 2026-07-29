#!/usr/bin/env python3
"""Draw the current track_runner waypoint path without ROS dependencies."""

import argparse
import csv
import math
from pathlib import Path


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def push_point(path, point):
    if not path or distance(path[-1], point) > 1e-6:
        path.append(point)


def append_line(path, start, end, spacing):
    length = distance(start, end)
    steps = max(1, math.ceil(length / spacing))
    for i in range(steps + 1):
        t = i / steps
        push_point(
            path,
            (
                start[0] + (end[0] - start[0]) * t,
                start[1] + (end[1] - start[1]) * t,
            ),
        )


def append_arc(path, center, radius, start_angle, delta_angle, spacing):
    length = abs(delta_angle) * radius
    steps = max(1, math.ceil(length / spacing))
    for i in range(1, steps + 1):
        theta = start_angle + delta_angle * i / steps
        push_point(
            path,
            (
                center[0] + radius * math.cos(theta),
                center[1] + radius * math.sin(theta),
            ),
        )


def build_path(straight_length_m, radius_m, path_spacing_m):
    path = []
    append_line(path, (0.0, 0.0), (0.0, straight_length_m), path_spacing_m)
    append_arc(path, (radius_m, straight_length_m), radius_m, math.pi, -math.pi, path_spacing_m)
    append_line(
        path,
        (2.0 * radius_m, straight_length_m),
        (2.0 * radius_m, 0.0),
        path_spacing_m,
    )
    append_arc(path, (radius_m, 0.0), radius_m, 0.0, -math.pi, path_spacing_m)
    return path


def cumulative_distances(path):
    distances = [0.0]
    for previous, current in zip(path, path[1:]):
        distances.append(distances[-1] + distance(previous, current))
    return distances


def write_csv(path, cumulative, csv_path):
    with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(["index", "x_m", "y_m", "s_m"])
        for index, ((x, y), s_m) in enumerate(zip(path, cumulative)):
            writer.writerow([index, f"{x:.9f}", f"{y:.9f}", f"{s_m:.9f}"])


def svg_point(point, min_x, max_y, scale, margin):
    x, y = point
    return margin + (x - min_x) * scale, margin + (max_y - y) * scale


def write_svg(path, cumulative, svg_path, straight_length_m, radius_m):
    min_x = min(x for x, _ in path)
    max_x = max(x for x, _ in path)
    min_y = min(y for _, y in path)
    max_y = max(y for _, y in path)
    margin = 56
    scale = 260
    width = int((max_x - min_x) * scale + 2 * margin)
    height = int((max_y - min_y) * scale + 2 * margin)

    points = [svg_point(point, min_x, max_y, scale, margin) for point in path]
    polyline = " ".join(f"{x:.1f},{y:.1f}" for x, y in points)
    dots = []
    label_every = max(1, len(path) // 40)
    for index, (x, y) in enumerate(points):
        if index % label_every == 0 or index in (0, len(points) - 1):
            dots.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="2.2" fill="#2563eb" />')

    labels = {
        "A start": (0.0, 0.0),
        "B": (0.0, straight_length_m),
        "C": (2.0 * radius_m, straight_length_m),
        "D": (2.0 * radius_m, 0.0),
    }
    label_nodes = []
    for text, point in labels.items():
        x, y = svg_point(point, min_x, max_y, scale, margin)
        label_nodes.append(
            f'<circle cx="{x:.1f}" cy="{y:.1f}" r="5" fill="#dc2626" />'
            f'<text x="{x + 8:.1f}" y="{y - 8:.1f}" '
            'font-family="sans-serif" font-size="14" fill="#111827">'
            f"{text}</text>"
        )

    arrow_start = svg_point(path[4], min_x, max_y, scale, margin)
    arrow_end = svg_point(path[12], min_x, max_y, scale, margin)
    lap_length = cumulative[-1]

    svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
  <rect width="100%" height="100%" fill="#f8fafc"/>
  <text x="16" y="28" font-family="sans-serif" font-size="18" font-weight="700" fill="#0f172a">track_runner current waypoints</text>
  <text x="16" y="50" font-family="sans-serif" font-size="13" fill="#475569">straight={straight_length_m:.2f} m, radius={radius_m:.2f} m, points={len(path)}, lap={lap_length:.3f} m, first segment is +y</text>
  <g stroke="#cbd5e1" stroke-width="1">
    <line x1="{margin}" y1="{height - margin}" x2="{width - margin}" y2="{height - margin}" />
    <line x1="{margin}" y1="{height - margin}" x2="{margin}" y2="{margin}" />
  </g>
  <text x="{width - margin - 18}" y="{height - margin + 24}" font-family="sans-serif" font-size="13" fill="#475569">+x</text>
  <text x="{margin - 34}" y="{margin + 4}" font-family="sans-serif" font-size="13" fill="#475569">+y</text>
  <polyline points="{polyline}" fill="none" stroke="#0f172a" stroke-width="4" stroke-linecap="round" stroke-linejoin="round"/>
  <g>{''.join(dots)}</g>
  <line x1="{arrow_start[0]:.1f}" y1="{arrow_start[1]:.1f}" x2="{arrow_end[0]:.1f}" y2="{arrow_end[1]:.1f}" stroke="#16a34a" stroke-width="4" marker-end="url(#arrow)"/>
  <defs>
    <marker id="arrow" markerWidth="10" markerHeight="10" refX="8" refY="3" orient="auto" markerUnits="strokeWidth">
      <path d="M0,0 L0,6 L9,3 z" fill="#16a34a" />
    </marker>
  </defs>
  <g>{''.join(label_nodes)}</g>
</svg>
'''
    svg_path.write_text(svg, encoding="utf-8")


def build_arg_parser():
    parser = argparse.ArgumentParser(description="Plot the track_runner default waypoint path.")
    parser.add_argument("--straight-length-m", type=float, default=1.5)
    parser.add_argument("--radius-m", type=float, default=0.75)
    parser.add_argument("--path-spacing-m", type=float, default=0.03)
    parser.add_argument(
        "--output-prefix",
        default=str(Path(__file__).resolve().parent / "current_track_waypoints"),
    )
    return parser


def main():
    args = build_arg_parser().parse_args()
    path = build_path(
        max(0.01, args.straight_length_m),
        max(0.01, args.radius_m),
        max(0.005, args.path_spacing_m),
    )
    cumulative = cumulative_distances(path)
    output_prefix = Path(args.output_prefix).expanduser()
    output_prefix.parent.mkdir(parents=True, exist_ok=True)
    svg_path = output_prefix.with_suffix(".svg")
    csv_path = output_prefix.with_suffix(".csv")
    write_svg(path, cumulative, svg_path, args.straight_length_m, args.radius_m)
    write_csv(path, cumulative, csv_path)
    print(f"wrote {svg_path}")
    print(f"wrote {csv_path}")
    print(f"points={len(path)} lap_length={cumulative[-1]:.3f} m")


if __name__ == "__main__":
    main()
