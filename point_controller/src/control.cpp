#include "point_controller/control.hpp"

#include <algorithm>
#include <cmath>

namespace point_controller
{

double normalize_angle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double limit_with_min(double value, double max_abs, double min_abs)
{
  if (value == 0.0) {
    return 0.0;
  }
  const double sign = value > 0.0 ? 1.0 : -1.0;
  double limited = std::min(std::abs(value), max_abs);
  if (limited < min_abs) {
    limited = min_abs;
  }
  return sign * limited;
}

CommandResult compute_command(
  const RobotPose & robot,
  const GoalPose & goal,
  const ControllerConfig & config)
{
  const double dx = goal.x - robot.x;
  const double dy = goal.y - robot.y;
  const double distance = std::hypot(dx, dy);

  if (distance > config.xy_tolerance_m) {
    const double desired_yaw = std::atan2(dy, dx);
    const double heading_error = normalize_angle(desired_yaw - robot.yaw);
    const double w = limit_with_min(
      config.k_w * heading_error, config.w_max_rad_s, config.w_min_rad_s);
    if (std::abs(heading_error) > config.heading_tolerance_rad) {
      return CommandResult{0.0, w, false, "align_heading", distance, heading_error};
    }

    const double v = limit_with_min(
      config.k_v * distance, config.v_max_m_s, config.v_min_m_s);
    return CommandResult{v, w, false, "drive", distance, heading_error};
  }

  const double yaw_error = normalize_angle(goal.yaw - robot.yaw);
  if (std::abs(yaw_error) > config.yaw_tolerance_rad) {
    const double w = limit_with_min(
      config.k_w * yaw_error, config.w_max_rad_s, config.w_min_rad_s);
    return CommandResult{0.0, w, false, "align_final_yaw", distance, yaw_error};
  }

  return CommandResult{0.0, 0.0, true, "reached", distance, yaw_error};
}

}  // namespace point_controller

