from math import pi

from point_controller.control import (
    ControllerConfig,
    GoalPose,
    RobotPose,
    compute_command,
    normalize_angle,
)


def test_normalize_angle_wraps_to_pi_range():
    assert normalize_angle(3.5) < pi
    assert normalize_angle(-3.5) > -pi


def test_controller_aligns_before_driving():
    result = compute_command(
        RobotPose(0.0, 0.0, pi / 2.0),
        GoalPose(1.0, 0.0, 0.0),
        ControllerConfig(),
    )

    assert result.phase == 'align_heading'
    assert result.v == 0.0
    assert result.w < 0.0


def test_controller_drives_when_heading_is_close():
    result = compute_command(
        RobotPose(0.0, 0.0, 0.01),
        GoalPose(1.0, 0.0, 0.0),
        ControllerConfig(),
    )

    assert result.phase == 'drive'
    assert result.v > 0.0


def test_controller_reaches_inside_tolerances():
    result = compute_command(
        RobotPose(0.0, 0.0, 0.0),
        GoalPose(0.01, 0.0, 0.01),
        ControllerConfig(),
    )

    assert result.reached
    assert result.v == 0.0
    assert result.w == 0.0
