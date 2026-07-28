#include "path_planner/geometry.hpp"

#include <gtest/gtest.h>

using path_planner::PlanningError;
using path_planner::Point;
using path_planner::inflate_zone;
using path_planner::plan_path;
using path_planner::rectangle_zone;
using path_planner::segment_is_visible;

TEST(PathPlannerGeometry, DirectPathWithoutObstacles)
{
  const auto path = plan_path(Point{0.0, 0.0}, Point{1.0, 0.0}, {});

  ASSERT_EQ(path.size(), 2U);
  EXPECT_DOUBLE_EQ(path[0].x, 0.0);
  EXPECT_DOUBLE_EQ(path[1].x, 1.0);
}

TEST(PathPlannerGeometry, DirectPathWhenObstacleIsNotCrossed)
{
  const auto zone = rectangle_zone("box", {0.5, 1.0}, {0.2, 0.2});

  const auto path = plan_path(Point{0.0, 0.0}, Point{1.0, 0.0}, {zone});

  ASSERT_EQ(path.size(), 2U);
}

TEST(PathPlannerGeometry, PlansAroundRectangleWhenDirectPathIsBlocked)
{
  const auto zone = rectangle_zone("box", {0.5, 0.0}, {0.2, 0.4});

  const auto path = plan_path(Point{0.0, 0.0}, Point{1.0, 0.0}, {zone});

  ASSERT_GT(path.size(), 2U);
  EXPECT_DOUBLE_EQ(path.front().x, 0.0);
  EXPECT_DOUBLE_EQ(path.back().x, 1.0);
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    EXPECT_TRUE(segment_is_visible(path[i], path[i + 1], {zone}));
  }
}

TEST(PathPlannerGeometry, RejectsGoalInsideKeepoutZone)
{
  const auto zone = rectangle_zone("box", {0.5, 0.0}, {0.2, 0.4});

  EXPECT_THROW(
    plan_path(Point{0.0, 0.0}, Point{0.5, 0.0}, {zone}),
    PlanningError);
}

TEST(PathPlannerGeometry, InflatesAxisAlignedRectangle)
{
  const auto zone = rectangle_zone("box", {0.0, 0.0}, {1.0, 1.0});

  const auto inflated = inflate_zone(zone, 0.2);

  double min_x = inflated.points.front().x;
  double max_x = inflated.points.front().x;
  double min_y = inflated.points.front().y;
  double max_y = inflated.points.front().y;
  for (const auto & point : inflated.points) {
    min_x = std::min(min_x, point.x);
    max_x = std::max(max_x, point.x);
    min_y = std::min(min_y, point.y);
    max_y = std::max(max_y, point.y);
  }
  EXPECT_DOUBLE_EQ(min_x, -0.7);
  EXPECT_DOUBLE_EQ(max_x, 0.7);
  EXPECT_DOUBLE_EQ(min_y, -0.7);
  EXPECT_DOUBLE_EQ(max_y, 0.7);
}

