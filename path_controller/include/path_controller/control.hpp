#pragma once

#include <string>
#include <utility>
#include <vector>

namespace path_controller
{

struct ControllerConfig
{
  double xy_tolerance_m{0.05};
  double yaw_tolerance_rad{0.0872665};
  double heading_tolerance_rad{0.15};
  double lookahead_distance_m{0.35};
  double goal_slowdown_radius_m{0.40};
  double k_w{1.8};
  double target_speed_m_s{0.05};
  double v_max_m_s{0.05};
  double w_max_rad_s{0.5};
  double v_min_m_s{0.0};
  double w_min_rad_s{0.08};
};

struct PathPoint
{
  double x{};
  double y{};
  double yaw{};

  double distance_to(const PathPoint & other) const;
};

struct RobotPose
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
  double lookahead_x{};
  double lookahead_y{};
};

double normalize_angle(double angle);

double limit(double value, double max_abs);

double limit_with_min(double value, double max_abs, double min_abs);

CommandResult compute_command(
  const RobotPose & robot,
  const std::vector<PathPoint> & path,
  const ControllerConfig & config,
  bool allow_goal_completion = true);

PathPoint find_lookahead_point(
  const RobotPose & robot,
  const std::vector<PathPoint> & path,
  double lookahead_distance);

std::pair<std::size_t, double> closest_path_position(
  const RobotPose & robot,
  const std::vector<PathPoint> & path);

PathPoint interpolate(const PathPoint & a, const PathPoint & b, double t);

std::pair<double, double> transform_to_base(
  const RobotPose & robot, const PathPoint & point);

}  // namespace path_controller
