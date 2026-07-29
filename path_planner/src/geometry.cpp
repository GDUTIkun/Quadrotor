#include "path_planner/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace path_planner
{
namespace
{

constexpr double kEps = 1e-9;

std::vector<std::pair<Point, Point>> polygon_edges(const std::vector<Point> & polygon)
{
  std::vector<std::pair<Point, Point>> edges;
  for (std::size_t i = 0; i < polygon.size(); ++i) {
    edges.emplace_back(polygon[i], polygon[(i + 1) % polygon.size()]);
  }
  return edges;
}

bool is_axis_aligned_rectangle(const std::vector<Point> & points)
{
  if (points.size() != 4) {
    return false;
  }

  std::vector<double> xs;
  std::vector<double> ys;
  for (const auto & point : points) {
    xs.push_back(point.x);
    ys.push_back(point.y);
  }
  std::sort(xs.begin(), xs.end());
  std::sort(ys.begin(), ys.end());
  xs.erase(std::unique(xs.begin(), xs.end(), [](double a, double b) {
    return std::abs(a - b) < 1e-9;
  }), xs.end());
  ys.erase(std::unique(ys.begin(), ys.end(), [](double a, double b) {
    return std::abs(a - b) < 1e-9;
  }), ys.end());
  return xs.size() == 2 && ys.size() == 2;
}

Point midpoint(const Point & a, const Point & b)
{
  return Point{(a.x + b.x) * 0.5, (a.y + b.y) * 0.5};
}

bool segments_properly_intersect(
  const Point & a, const Point & b, const Point & c, const Point & d)
{
  const double o1 = orientation(a, b, c);
  const double o2 = orientation(a, b, d);
  const double o3 = orientation(c, d, a);
  const double o4 = orientation(c, d, b);

  if (o1 * o2 < -kEps && o3 * o4 < -kEps) {
    return true;
  }

  if (std::abs(o1) <= kEps && point_on_segment(c, a, b)) {
    return !(same_point(c, a) || same_point(c, b));
  }
  if (std::abs(o2) <= kEps && point_on_segment(d, a, b)) {
    return !(same_point(d, a) || same_point(d, b));
  }
  if (std::abs(o3) <= kEps && point_on_segment(a, c, d)) {
    return !(same_point(a, c) || same_point(a, d));
  }
  if (std::abs(o4) <= kEps && point_on_segment(b, c, d)) {
    return !(same_point(b, c) || same_point(b, d));
  }
  return false;
}

std::vector<std::size_t> dijkstra(
  const std::vector<std::vector<std::pair<std::size_t, double>>> & edges,
  std::size_t start, std::size_t goal)
{
  using QueueItem = std::pair<double, std::size_t>;
  std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> queue;
  std::vector<double> distances(edges.size(), std::numeric_limits<double>::infinity());
  std::vector<std::size_t> previous(edges.size(), edges.size());

  distances[start] = 0.0;
  queue.emplace(0.0, start);

  while (!queue.empty()) {
    const auto [distance, node] = queue.top();
    queue.pop();
    if (node == goal) {
      break;
    }
    if (distance > distances[node]) {
      continue;
    }
    for (const auto & [neighbor, weight] : edges[node]) {
      const double candidate = distance + weight;
      if (candidate < distances[neighbor]) {
        distances[neighbor] = candidate;
        previous[neighbor] = node;
        queue.emplace(candidate, neighbor);
      }
    }
  }

  if (!std::isfinite(distances[goal])) {
    return {};
  }

  std::vector<std::size_t> route{goal};
  while (route.back() != start) {
    route.push_back(previous[route.back()]);
  }
  std::reverse(route.begin(), route.end());
  return route;
}

}  // namespace

double Point::distance_to(const Point & other) const
{
  return std::hypot(x - other.x, y - other.y);
}

PlanningError::PlanningError(const std::string & message)
: std::runtime_error(message)
{
}

KeepoutZone rectangle_zone(
  const std::string & name, const std::vector<double> & center,
  const std::vector<double> & size)
{
  if (center.size() != 2 || size.size() != 2) {
    throw PlanningError("Rectangle \"" + name + "\" must have 2D center and size");
  }
  const double sx = size[0];
  const double sy = size[1];
  if (sx <= 0.0 || sy <= 0.0) {
    throw PlanningError("Rectangle \"" + name + "\" size must be positive");
  }

  const double cx = center[0];
  const double cy = center[1];
  const double hx = sx * 0.5;
  const double hy = sy * 0.5;
  return KeepoutZone{
    name,
    {
      Point{cx - hx, cy - hy},
      Point{cx + hx, cy - hy},
      Point{cx + hx, cy + hy},
      Point{cx - hx, cy + hy},
    }};
}

KeepoutZone polygon_zone(
  const std::string & name, const std::vector<std::vector<double>> & points)
{
  if (points.size() < 3) {
    throw PlanningError("Polygon \"" + name + "\" must have at least 3 points");
  }

  KeepoutZone zone{name, {}};
  for (const auto & point : points) {
    if (point.size() != 2) {
      throw PlanningError("Polygon \"" + name + "\" contains a non-2D point");
    }
    zone.points.push_back(Point{point[0], point[1]});
  }
  return zone;
}

KeepoutZone inflate_zone(const KeepoutZone & zone, double radius)
{
  if (radius <= 0.0) {
    return zone;
  }

  if (is_axis_aligned_rectangle(zone.points)) {
    double min_x = zone.points.front().x;
    double max_x = zone.points.front().x;
    double min_y = zone.points.front().y;
    double max_y = zone.points.front().y;
    for (const auto & point : zone.points) {
      min_x = std::min(min_x, point.x);
      max_x = std::max(max_x, point.x);
      min_y = std::min(min_y, point.y);
      max_y = std::max(max_y, point.y);
    }
    return KeepoutZone{
      zone.name,
      {
        Point{min_x - radius, min_y - radius},
        Point{max_x + radius, min_y - radius},
        Point{max_x + radius, max_y + radius},
        Point{min_x - radius, max_y + radius},
      }};
  }

  double cx = 0.0;
  double cy = 0.0;
  for (const auto & point : zone.points) {
    cx += point.x;
    cy += point.y;
  }
  cx /= static_cast<double>(zone.points.size());
  cy /= static_cast<double>(zone.points.size());

  KeepoutZone inflated{zone.name, {}};
  for (const auto & point : zone.points) {
    const double dx = point.x - cx;
    const double dy = point.y - cy;
    const double length = std::hypot(dx, dy);
    if (length <= kEps) {
      inflated.points.push_back(point);
    } else {
      inflated.points.push_back(Point{
        point.x + radius * dx / length,
        point.y + radius * dy / length});
    }
  }
  return inflated;
}

std::vector<KeepoutZone> inflate_zones(
  const std::vector<KeepoutZone> & zones, double radius)
{
  std::vector<KeepoutZone> inflated;
  inflated.reserve(zones.size());
  for (const auto & zone : zones) {
    inflated.push_back(inflate_zone(zone, radius));
  }
  return inflated;
}

std::vector<Point> plan_path(
  const Point & start, const Point & goal,
  const std::vector<KeepoutZone> & keepout_zones,
  double min_waypoint_spacing_m,
  double max_plan_length_m)
{
  for (const auto & zone : keepout_zones) {
    if (point_in_polygon(start, zone.points, true)) {
      throw PlanningError("Start is inside keepout zone \"" + zone.name + "\"");
    }
    if (point_in_polygon(goal, zone.points, true)) {
      throw PlanningError("Goal is inside keepout zone \"" + zone.name + "\"");
    }
  }

  if (segment_is_visible(start, goal, keepout_zones)) {
    return {start, goal};
  }

  std::vector<Point> nodes{start, goal};
  for (const auto & zone : keepout_zones) {
    nodes.insert(nodes.end(), zone.points.begin(), zone.points.end());
  }

  std::vector<std::vector<std::pair<std::size_t, double>>> edges(nodes.size());
  for (std::size_t i = 0; i < nodes.size(); ++i) {
    for (std::size_t j = i + 1; j < nodes.size(); ++j) {
      if (segment_is_visible(nodes[i], nodes[j], keepout_zones)) {
        const double distance = nodes[i].distance_to(nodes[j]);
        edges[i].emplace_back(j, distance);
        edges[j].emplace_back(i, distance);
      }
    }
  }

  const auto route_indices = dijkstra(edges, 0, 1);
  if (route_indices.empty()) {
    throw PlanningError("No visible path to goal");
  }

  std::vector<Point> route;
  route.reserve(route_indices.size());
  for (const auto index : route_indices) {
    route.push_back(nodes[index]);
  }

  route = simplify_path(route, min_waypoint_spacing_m);
  const double length = path_length(route);
  if (length > max_plan_length_m) {
    throw PlanningError("Planned path is too long: " + std::to_string(length) + " m");
  }
  return route;
}

std::vector<Point> simplify_path(const std::vector<Point> & points, double min_spacing)
{
  if (points.size() <= 2 || min_spacing <= 0.0) {
    return points;
  }

  std::vector<Point> result{points.front()};
  for (std::size_t i = 1; i + 1 < points.size(); ++i) {
    if (points[i].distance_to(result.back()) >= min_spacing) {
      result.push_back(points[i]);
    }
  }
  if (!same_point(points.back(), result.back())) {
    result.push_back(points.back());
  }
  return result;
}

double path_length(const std::vector<Point> & points)
{
  double length = 0.0;
  for (std::size_t i = 0; i + 1 < points.size(); ++i) {
    length += points[i].distance_to(points[i + 1]);
  }
  return length;
}

std::vector<Point> make_straight_path(
  const Point & start, double yaw, double length_m, double spacing_m)
{
  if (length_m <= 0.0) {
    throw PlanningError("Straight path length must be positive");
  }
  if (spacing_m <= 0.0) {
    throw PlanningError("Path spacing must be positive");
  }

  const int steps = std::max(1, static_cast<int>(std::ceil(length_m / spacing_m)));
  std::vector<Point> points;
  points.reserve(static_cast<std::size_t>(steps) + 1);
  for (int i = 0; i <= steps; ++i) {
    const double s = length_m * static_cast<double>(i) / static_cast<double>(steps);
    points.push_back(Point{
      start.x + s * std::cos(yaw),
      start.y + s * std::sin(yaw)});
  }
  return points;
}

std::vector<Point> make_arc_path(
  const Point & start, double start_yaw, double radius_m,
  double angle_rad, double spacing_m)
{
  if (radius_m <= 0.0) {
    throw PlanningError("Arc radius must be positive");
  }
  if (std::abs(angle_rad) <= kEps) {
    throw PlanningError("Arc angle must be non-zero");
  }
  if (spacing_m <= 0.0) {
    throw PlanningError("Path spacing must be positive");
  }

  const double arc_length = radius_m * std::abs(angle_rad);
  const int steps = std::max(1, static_cast<int>(std::ceil(arc_length / spacing_m)));
  const double direction = angle_rad > 0.0 ? 1.0 : -1.0;
  const Point center{
    start.x - direction * radius_m * std::sin(start_yaw),
    start.y + direction * radius_m * std::cos(start_yaw)};
  const double radial_start = start_yaw - direction * M_PI * 0.5;

  std::vector<Point> points;
  points.reserve(static_cast<std::size_t>(steps) + 1);
  for (int i = 0; i <= steps; ++i) {
    const double theta =
      angle_rad * static_cast<double>(i) / static_cast<double>(steps);
    const double radial = radial_start + theta;
    points.push_back(Point{
      center.x + radius_m * std::cos(radial),
      center.y + radius_m * std::sin(radial)});
  }
  return points;
}

std::vector<Point> make_racetrack_path(
  const Point & start, double start_yaw, double straight_length_m,
  double radius_m, double spacing_m, bool turn_right)
{
  const double arc_angle = turn_right ? -M_PI : M_PI;
  std::vector<Point> path;

  const auto append_segment = [&path](const std::vector<Point> & segment) {
      if (segment.empty()) {
        return;
      }
      const auto first = segment.begin() + (path.empty() ? 0 : 1);
      path.insert(path.end(), first, segment.end());
    };

  const auto first_straight =
    make_straight_path(start, start_yaw, straight_length_m, spacing_m);
  append_segment(first_straight);

  const auto first_arc =
    make_arc_path(path.back(), start_yaw, radius_m, arc_angle, spacing_m);
  append_segment(first_arc);

  const double return_yaw = start_yaw + arc_angle;
  const auto second_straight =
    make_straight_path(path.back(), return_yaw, straight_length_m, spacing_m);
  append_segment(second_straight);

  const auto second_arc =
    make_arc_path(path.back(), return_yaw, radius_m, arc_angle, spacing_m);
  append_segment(second_arc);

  // Make the closed-loop endpoint exact instead of leaving trigonometric drift.
  path.back() = start;
  return path;
}

bool segment_is_visible(
  const Point & a, const Point & b,
  const std::vector<KeepoutZone> & zones)
{
  if (same_point(a, b)) {
    return false;
  }
  for (const auto & zone : zones) {
    if (segment_crosses_polygon_interior(a, b, zone.points)) {
      return false;
    }
  }
  return true;
}

bool segment_crosses_polygon_interior(
  const Point & a, const Point & b,
  const std::vector<Point> & polygon)
{
  if (point_in_polygon(midpoint(a, b), polygon, false)) {
    return true;
  }

  for (const auto & [edge_start, edge_end] : polygon_edges(polygon)) {
    if (segments_properly_intersect(a, b, edge_start, edge_end)) {
      return true;
    }
  }

  constexpr int samples = 8;
  for (int i = 1; i < samples; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(samples);
    const Point sample{
      a.x + (b.x - a.x) * t,
      a.y + (b.y - a.y) * t};
    if (point_in_polygon(sample, polygon, false)) {
      return true;
    }
  }
  return false;
}

bool point_in_polygon(
  const Point & point, const std::vector<Point> & polygon,
  bool include_boundary)
{
  if (include_boundary && point_on_polygon_boundary(point, polygon)) {
    return true;
  }

  bool inside = false;
  std::size_t j = polygon.size() - 1;
  for (std::size_t i = 0; i < polygon.size(); ++i) {
    const auto & pi = polygon[i];
    const auto & pj = polygon[j];
    if ((pi.y > point.y) != (pj.y > point.y)) {
      const double x_cross =
        (pj.x - pi.x) * (point.y - pi.y) / (pj.y - pi.y) + pi.x;
      if (point.x < x_cross) {
        inside = !inside;
      }
    }
    j = i;
  }
  return inside;
}

bool point_on_polygon_boundary(
  const Point & point, const std::vector<Point> & polygon)
{
  for (const auto & [a, b] : polygon_edges(polygon)) {
    if (point_on_segment(point, a, b)) {
      return true;
    }
  }
  return false;
}

bool point_on_segment(const Point & point, const Point & a, const Point & b)
{
  if (std::abs(orientation(a, b, point)) > kEps) {
    return false;
  }
  return std::min(a.x, b.x) - kEps <= point.x &&
         point.x <= std::max(a.x, b.x) + kEps &&
         std::min(a.y, b.y) - kEps <= point.y &&
         point.y <= std::max(a.y, b.y) + kEps;
}

double orientation(const Point & a, const Point & b, const Point & c)
{
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

bool same_point(const Point & a, const Point & b)
{
  return std::abs(a.x - b.x) <= 1e-7 && std::abs(a.y - b.y) <= 1e-7;
}

}  // namespace path_planner
