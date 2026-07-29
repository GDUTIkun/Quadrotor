#include "path_controller/control.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace path_controller
{

double PathPoint::distance_to(const PathPoint & other) const
{
  return std::hypot(x - other.x, y - other.y);
}

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

double limit(double value, double max_abs)
{
  return std::max(-max_abs, std::min(max_abs, value));
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
  const std::vector<PathPoint> & path,
  const ControllerConfig & config,
  bool allow_goal_completion)
{
  if (path.empty()) {
    return CommandResult{0.0, 0.0, false, "no_path", 0.0, 0.0, 0.0, 0.0};
  }

  const auto & goal = path.back();
  const double distance_to_goal = std::hypot(goal.x - robot.x, goal.y - robot.y);
  if (allow_goal_completion && distance_to_goal <= config.xy_tolerance_m) {
    const double yaw_error = normalize_angle(goal.yaw - robot.yaw);
    if (std::abs(yaw_error) <= config.yaw_tolerance_rad) {
      return CommandResult{
        0.0, 0.0, true, "reached", distance_to_goal, yaw_error, 0.0, 0.0};
    }
    const double w = limit_with_min(
      config.k_w * yaw_error, config.w_max_rad_s, config.w_min_rad_s);
    return CommandResult{
      0.0, w, false, "align_final_yaw", distance_to_goal, yaw_error, 0.0, 0.0};
  }

  const auto target = find_lookahead_point(robot, path, config.lookahead_distance_m);
  const auto [base_x, base_y] = transform_to_base(robot, target);
  const double heading_error = std::atan2(base_y, base_x);
  const double w_align = limit_with_min(
    config.k_w * heading_error, config.w_max_rad_s, config.w_min_rad_s);

  if (base_x < 0.0 || std::abs(heading_error) > config.heading_tolerance_rad) {
    return CommandResult{
      0.0, w_align, false, "align_path", distance_to_goal, heading_error,
      target.x, target.y};
  }

  double v = std::min(std::abs(config.target_speed_m_s), config.v_max_m_s);
  if (v > 0.0 && v < config.v_min_m_s) {
    v = config.v_min_m_s;
  }

  const double lookahead = std::max(
    {config.lookahead_distance_m, std::hypot(base_x, base_y), 1e-6});
  const double curvature = 2.0 * base_y / (lookahead * lookahead);
  const double w = limit(v * curvature, config.w_max_rad_s);

  return CommandResult{
    v, w, false, "track_path", distance_to_goal, heading_error, target.x, target.y};
}

PathPoint find_lookahead_point(
  const RobotPose & robot,
  const std::vector<PathPoint> & path,
  double lookahead_distance)
{
  if (path.size() == 1) {
    return path.front();
  }

  const auto [closest_segment_index, along_segment] = closest_path_position(robot, path);
  double remaining = std::max(lookahead_distance, 0.0);

  auto a = path[closest_segment_index];
  auto b = path[closest_segment_index + 1];
  double segment_length = a.distance_to(b);
  const double distance_to_segment_end =
    std::max(0.0, segment_length * (1.0 - along_segment));
  if (remaining <= distance_to_segment_end && segment_length > 0.0) {
    const double t = along_segment + remaining / segment_length;
    return interpolate(a, b, t);
  }

  remaining -= distance_to_segment_end;
  for (std::size_t index = closest_segment_index + 1; index + 1 < path.size(); ++index) {
    a = path[index];
    b = path[index + 1];
    segment_length = a.distance_to(b);
    if (segment_length <= 0.0) {
      continue;
    }
    if (remaining <= segment_length) {
      return interpolate(a, b, remaining / segment_length);
    }
    remaining -= segment_length;
  }

  return path.back();
}

std::pair<std::size_t, double> closest_path_position(
  const RobotPose & robot,
  const std::vector<PathPoint> & path)
{
  std::size_t best_index = 0;
  double best_t = 0.0;
  double best_distance = std::numeric_limits<double>::infinity();
  const PathPoint robot_point{robot.x, robot.y, 0.0};

  for (std::size_t index = 0; index + 1 < path.size(); ++index) {
    const auto & a = path[index];
    const auto & b = path[index + 1];
    const double ab_x = b.x - a.x;
    const double ab_y = b.y - a.y;
    const double ab_len_sq = ab_x * ab_x + ab_y * ab_y;
    if (ab_len_sq <= 0.0) {
      continue;
    }
    double t = ((robot.x - a.x) * ab_x + (robot.y - a.y) * ab_y) / ab_len_sq;
    t = std::max(0.0, std::min(1.0, t));
    const auto projected = interpolate(a, b, t);
    const double distance = robot_point.distance_to(projected);
    if (distance < best_distance) {
      best_distance = distance;
      best_index = index;
      best_t = t;
    }
  }

  return {best_index, best_t};
}

PathPoint interpolate(const PathPoint & a, const PathPoint & b, double t)
{
  t = std::max(0.0, std::min(1.0, t));
  const double yaw = std::atan2(b.y - a.y, b.x - a.x);
  return PathPoint{
    a.x + (b.x - a.x) * t,
    a.y + (b.y - a.y) * t,
    yaw};
}

std::pair<double, double> transform_to_base(
  const RobotPose & robot, const PathPoint & point)
{
  const double dx = point.x - robot.x;
  const double dy = point.y - robot.y;
  const double c = std::cos(robot.yaw);
  const double s = std::sin(robot.yaw);
  return {c * dx + s * dy, -s * dx + c * dy};
}

}  // namespace path_controller
