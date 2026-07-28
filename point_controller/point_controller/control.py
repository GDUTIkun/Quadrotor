"""Pure control helpers for the map-frame point controller."""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, pi, sqrt


@dataclass(frozen=True)
class ControllerConfig:
    xy_tolerance_m: float = 0.05
    yaw_tolerance_rad: float = 0.0872665
    heading_tolerance_rad: float = 0.15
    k_v: float = 0.8
    k_w: float = 1.8
    v_max_m_s: float = 0.25
    w_max_rad_s: float = 0.8
    v_min_m_s: float = 0.03
    w_min_rad_s: float = 0.08


@dataclass(frozen=True)
class RobotPose:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class GoalPose:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class CommandResult:
    v: float
    w: float
    reached: bool
    phase: str
    distance_error: float
    yaw_error: float


def normalize_angle(angle: float) -> float:
    while angle > pi:
        angle -= 2.0 * pi
    while angle < -pi:
        angle += 2.0 * pi
    return angle


def limit_with_min(value: float, max_abs: float, min_abs: float) -> float:
    if value == 0.0:
        return 0.0
    sign = 1.0 if value > 0.0 else -1.0
    limited = min(abs(value), max_abs)
    if limited < min_abs:
        limited = min_abs
    return sign * limited


def compute_command(robot: RobotPose, goal: GoalPose, config: ControllerConfig) -> CommandResult:
    dx = goal.x - robot.x
    dy = goal.y - robot.y
    distance = sqrt(dx * dx + dy * dy)

    if distance > config.xy_tolerance_m:
        desired_yaw = atan2(dy, dx)
        heading_error = normalize_angle(desired_yaw - robot.yaw)
        w = limit_with_min(config.k_w * heading_error, config.w_max_rad_s, config.w_min_rad_s)
        if abs(heading_error) > config.heading_tolerance_rad:
            return CommandResult(0.0, w, False, 'align_heading', distance, heading_error)

        v = limit_with_min(config.k_v * distance, config.v_max_m_s, config.v_min_m_s)
        return CommandResult(v, w, False, 'drive', distance, heading_error)

    yaw_error = normalize_angle(goal.yaw - robot.yaw)
    if abs(yaw_error) > config.yaw_tolerance_rad:
        w = limit_with_min(config.k_w * yaw_error, config.w_max_rad_s, config.w_min_rad_s)
        return CommandResult(0.0, w, False, 'align_final_yaw', distance, yaw_error)

    return CommandResult(0.0, 0.0, True, 'reached', distance, yaw_error)
