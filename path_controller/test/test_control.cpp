#include "path_controller/control.hpp"

#include <gtest/gtest.h>

#include <cmath>

using path_controller::ControllerConfig;
using path_controller::PathPoint;
using path_controller::RobotPose;
using path_controller::compute_command;
using path_controller::find_lookahead_point;
using path_controller::normalize_angle;
using path_controller::transform_to_base;

TEST(PathControllerControl, NormalizeAngleWrapsToPiRange)
{
  EXPECT_LT(normalize_angle(3.5), M_PI);
  EXPECT_GT(normalize_angle(-3.5), -M_PI);
}

TEST(PathControllerControl, LookaheadOnStraightPath)
{
  const auto target = find_lookahead_point(
    RobotPose{0.0, 0.0, 0.0},
    {PathPoint{0.0, 0.0, 0.0}, PathPoint{1.0, 0.0, 0.0}},
    0.35);

  EXPECT_DOUBLE_EQ(target.x, 0.35);
  EXPECT_DOUBLE_EQ(target.y, 0.0);
}

TEST(PathControllerControl, TransformToBaseRespectsRobotYaw)
{
  const auto [x, y] = transform_to_base(
    RobotPose{0.0, 0.0, M_PI / 2.0}, PathPoint{0.0, 1.0, 0.0});

  EXPECT_NEAR(x, 1.0, 1e-6);
  EXPECT_NEAR(y, 0.0, 1e-6);
}

TEST(PathControllerControl, ControllerTracksStraightPath)
{
  const auto result = compute_command(
    RobotPose{0.0, 0.0, 0.0},
    {PathPoint{0.0, 0.0, 0.0}, PathPoint{1.0, 0.0, 0.0}},
    ControllerConfig{});

  EXPECT_EQ(result.phase, "track_path");
  EXPECT_GT(result.v, 0.0);
  EXPECT_NEAR(result.w, 0.0, 1e-6);
}

TEST(PathControllerControl, ControllerAlignsBeforeTracking)
{
  const auto result = compute_command(
    RobotPose{0.0, 0.0, M_PI / 2.0},
    {PathPoint{0.0, 0.0, 0.0}, PathPoint{1.0, 0.0, 0.0}},
    ControllerConfig{});

  EXPECT_EQ(result.phase, "align_path");
  EXPECT_DOUBLE_EQ(result.v, 0.0);
  EXPECT_LT(result.w, 0.0);
}

TEST(PathControllerControl, ControllerReachesInsideTolerances)
{
  const auto result = compute_command(
    RobotPose{1.0, 0.0, 0.0},
    {PathPoint{0.0, 0.0, 0.0}, PathPoint{1.0, 0.0, 0.01}},
    ControllerConfig{});

  EXPECT_TRUE(result.reached);
  EXPECT_DOUBLE_EQ(result.v, 0.0);
  EXPECT_DOUBLE_EQ(result.w, 0.0);
}

