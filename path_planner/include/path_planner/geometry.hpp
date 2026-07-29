#pragma once

#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace path_planner
{

struct Point
{
  double x{};
  double y{};

  double distance_to(const Point & other) const;
};

struct KeepoutZone
{
  std::string name;
  std::vector<Point> points;
};

class PlanningError : public std::runtime_error
{
public:
  explicit PlanningError(const std::string & message);
};

KeepoutZone rectangle_zone(
  const std::string & name, const std::vector<double> & center,
  const std::vector<double> & size);

KeepoutZone polygon_zone(
  const std::string & name, const std::vector<std::vector<double>> & points);

KeepoutZone inflate_zone(const KeepoutZone & zone, double radius);

std::vector<KeepoutZone> inflate_zones(
  const std::vector<KeepoutZone> & zones, double radius);

std::vector<Point> plan_path(
  const Point & start, const Point & goal,
  const std::vector<KeepoutZone> & keepout_zones,
  double min_waypoint_spacing_m = 0.10,
  double max_plan_length_m = 20.0);

std::vector<Point> simplify_path(
  const std::vector<Point> & points, double min_spacing);

double path_length(const std::vector<Point> & points);

std::vector<Point> make_straight_path(
  const Point & start, double yaw, double length_m, double spacing_m);

std::vector<Point> make_arc_path(
  const Point & start, double start_yaw, double radius_m,
  double angle_rad, double spacing_m);

std::vector<Point> make_racetrack_path(
  const Point & start, double start_yaw, double straight_length_m,
  double radius_m, double spacing_m, bool turn_right = true);

bool segment_is_visible(
  const Point & a, const Point & b,
  const std::vector<KeepoutZone> & zones);

bool segment_crosses_polygon_interior(
  const Point & a, const Point & b,
  const std::vector<Point> & polygon);

bool point_in_polygon(
  const Point & point, const std::vector<Point> & polygon,
  bool include_boundary);

bool point_on_polygon_boundary(
  const Point & point, const std::vector<Point> & polygon);

bool point_on_segment(const Point & point, const Point & a, const Point & b);

double orientation(const Point & a, const Point & b, const Point & c);

bool same_point(const Point & a, const Point & b);

}  // namespace path_planner
