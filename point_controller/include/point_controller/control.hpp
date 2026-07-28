#pragma once

#include <string>

namespace point_controller
{

struct ControllerConfig
{
  double xy_tolerance_m{0.05};
  double yaw_tolerance_rad{0.0872665};
  double heading_tolerance_rad{0.15};
  double k_v{0.8};
  double k_w{1.8};
  double v_max_m_s{0.25};
  double w_max_rad_s{0.8};
  double v_min_m_s{0.03};
  double w_min_rad_s{0.08};
};

struct RobotPose
{
  double x{};
  double y{};
  double yaw{};
};

struct GoalPose
{
  double x{};
  double y{};
  double yaw{};
};

struct CommandResult
{
  double v{};
  double w{};
  bool reached{};
  std::string phase;
  double distance_error{};
  double yaw_error{};
};

double normalize_angle(double angle);

double limit_with_min(double value, double max_abs, double min_abs);

CommandResult compute_command(
  const RobotPose & robot,
  const GoalPose & goal,
  const ControllerConfig & config);

}  // namespace point_controller

