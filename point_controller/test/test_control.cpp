#include "point_controller/control.hpp"

#include <gtest/gtest.h>

#include <cmath>

using point_controller::ControllerConfig;
using point_controller::GoalPose;
using point_controller::RobotPose;
using point_controller::compute_command;
using point_controller::normalize_angle;

TEST(PointControllerControl, NormalizeAngleWrapsToPiRange)
{
  EXPECT_LT(normalize_angle(3.5), M_PI);
  EXPECT_GT(normalize_angle(-3.5), -M_PI);
}

TEST(PointControllerControl, ControllerAlignsBeforeDriving)
{
  const auto result = compute_command(
    RobotPose{0.0, 0.0, M_PI / 2.0},
    GoalPose{1.0, 0.0, 0.0},
    ControllerConfig{});

  EXPECT_EQ(result.phase, "align_heading");
  EXPECT_DOUBLE_EQ(result.v, 0.0);
  EXPECT_LT(result.w, 0.0);
}

TEST(PointControllerControl, ControllerDrivesWhenHeadingIsClose)
{
  const auto result = compute_command(
    RobotPose{0.0, 0.0, 0.01},
    GoalPose{1.0, 0.0, 0.0},
    ControllerConfig{});

  EXPECT_EQ(result.phase, "drive");
  EXPECT_GT(result.v, 0.0);
}

TEST(PointControllerControl, ControllerReachesInsideTolerances)
{
  const auto result = compute_command(
    RobotPose{0.0, 0.0, 0.0},
    GoalPose{0.01, 0.0, 0.01},
    ControllerConfig{});

  EXPECT_TRUE(result.reached);
  EXPECT_DOUBLE_EQ(result.v, 0.0);
  EXPECT_DOUBLE_EQ(result.w, 0.0);
}

