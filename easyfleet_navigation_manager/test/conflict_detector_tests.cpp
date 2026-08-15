// Copyright 2026 Intelligent Robotics Lab
//
// This file is part of the project EasyFleet
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cmath>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "easyfleet_navigation_manager/conflict_detector.hpp"

#include "gtest/gtest.h"

namespace
{

// A straight-line path from (x0, y0) to (x1, y1), sampled every ~step_m,
// mirroring the fine, near-uniform sampling real planner plugins produce.
nav_msgs::msg::Path make_straight_path(
  double x0, double y0, double x1, double y1, double step_m = 0.2)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";

  const double dx = x1 - x0;
  const double dy = y1 - y0;
  const double length = std::hypot(dx, dy);
  const int n = std::max(1, static_cast<int>(length / step_m));

  for (int i = 0; i <= n; ++i) {
    const double t = static_cast<double>(i) / n;
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = x0 + t * dx;
    pose.pose.position.y = y0 + t * dy;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  return path;
}

}  // namespace

TEST(ConflictDetectorTest, ConvergingPathsPauseTheLongerOne)
{
  // robot_a: short remaining path, heads from (0,0) towards (1,0) -- close to its goal.
  // robot_b: long remaining path, heads from (0,1) towards (0,-4) -- far from its goal.
  // Both start close to each other and their lookahead segments cross.
  std::map<std::string, nav_msgs::msg::Path> paths;
  paths["robot_a"] = make_straight_path(0.0, 0.0, 1.0, 0.0);
  paths["robot_b"] = make_straight_path(0.0, 1.0, 0.0, -4.0);

  const auto to_pause = easyfleet::compute_robots_to_pause(paths);

  ASSERT_EQ(to_pause.size(), 1u);
  EXPECT_EQ(*to_pause.begin(), "robot_b");
}

TEST(ConflictDetectorTest, ParallelPathsNeverConverge)
{
  std::map<std::string, nav_msgs::msg::Path> paths;
  paths["robot_a"] = make_straight_path(0.0, 0.0, 5.0, 0.0);
  paths["robot_b"] = make_straight_path(0.0, 2.0, 5.0, 2.0);

  const auto to_pause = easyfleet::compute_robots_to_pause(paths);

  EXPECT_TRUE(to_pause.empty());
}

TEST(ConflictDetectorTest, FarApartCurrentPositionsAreNotAConflictEvenIfLookaheadEndsConverge)
{
  // With the default params (lookahead 2.0 m, proximity 3.0 m), these two
  // robots' *current* positions are ~4.0 m apart -- beyond
  // proximity_radius_m -- even though the far ends of their 2 m lookahead
  // windows come within path_conflict_distance_m of each other. The
  // proximity gate must reject this pair.
  std::map<std::string, nav_msgs::msg::Path> paths;
  paths["robot_a"] = make_straight_path(0.0, 0.0, 4.0, 0.0);
  paths["robot_b"] = make_straight_path(4.0, 0.3, 0.0, 0.3);

  const auto to_pause = easyfleet::compute_robots_to_pause(paths);

  EXPECT_TRUE(to_pause.empty());
}

TEST(ConflictDetectorTest, RobotWithEmptyPathIsIgnored)
{
  std::map<std::string, nav_msgs::msg::Path> paths;
  paths["robot_a"] = make_straight_path(0.0, 0.0, 1.0, 0.0);
  paths["robot_b"] = nav_msgs::msg::Path();  // no poses at all

  const auto to_pause = easyfleet::compute_robots_to_pause(paths);

  EXPECT_TRUE(to_pause.empty());
}

TEST(ConflictDetectorTest, SingleRobotHasNoConflict)
{
  std::map<std::string, nav_msgs::msg::Path> paths;
  paths["robot_a"] = make_straight_path(0.0, 0.0, 1.0, 0.0);

  const auto to_pause = easyfleet::compute_robots_to_pause(paths);

  EXPECT_TRUE(to_pause.empty());
}

TEST(ConflictDetectorTest, ThreeRobotsOnlyPausesTheOnesInvolvedInAConflict)
{
  // robot_a and robot_b converge (as in ConvergingPathsPauseTheLongerOne);
  // robot_c is far away from both and shares no conflict.
  std::map<std::string, nav_msgs::msg::Path> paths;
  paths["robot_a"] = make_straight_path(0.0, 0.0, 1.0, 0.0);
  paths["robot_b"] = make_straight_path(0.0, 1.0, 0.0, -4.0);
  paths["robot_c"] = make_straight_path(50.0, 50.0, 51.0, 50.0);

  const auto to_pause = easyfleet::compute_robots_to_pause(paths);

  ASSERT_EQ(to_pause.size(), 1u);
  EXPECT_EQ(*to_pause.begin(), "robot_b");
}

TEST(ConflictDetectorTest, CustomParamsAreRespected)
{
  // Same geometry as ParallelPathsNeverConverge, but with a much larger
  // path_conflict_distance_m the 2 m separation now counts as a conflict.
  std::map<std::string, nav_msgs::msg::Path> paths;
  paths["robot_a"] = make_straight_path(0.0, 0.0, 5.0, 0.0);
  paths["robot_b"] = make_straight_path(0.0, 2.0, 5.0, -2.0);  // ends closer to robot_a

  easyfleet::ConflictParams params;
  params.path_conflict_distance_m = 2.5;
  params.proximity_radius_m = 10.0;

  const auto to_pause = easyfleet::compute_robots_to_pause(paths, params);

  EXPECT_FALSE(to_pause.empty());
}

TEST(ConflictDetectorTest, AlreadyPausedRobotKeepsYieldingRegardlessOfPathLength)
{
  // Same geometry as ConvergingPathsPauseTheLongerOne, where the
  // path-length heuristic alone would pick robot_b (the longer path).
  // But robot_a is already paused (stationary, cmd_vel zero): it must
  // keep yielding, and robot_b -- still moving -- must be free to
  // continue rather than being newly paused because of it.
  std::map<std::string, nav_msgs::msg::Path> paths;
  paths["robot_a"] = make_straight_path(0.0, 0.0, 1.0, 0.0);
  paths["robot_b"] = make_straight_path(0.0, 1.0, 0.0, -4.0);

  const std::set<std::string> already_paused = {"robot_a"};

  const auto to_pause = easyfleet::compute_robots_to_pause(
    paths, easyfleet::ConflictParams(), already_paused);

  ASSERT_EQ(to_pause.size(), 1u);
  EXPECT_EQ(*to_pause.begin(), "robot_a");
}

TEST(ConflictDetectorTest, BothAlreadyPausedStayPaused)
{
  std::map<std::string, nav_msgs::msg::Path> paths;
  paths["robot_a"] = make_straight_path(0.0, 0.0, 1.0, 0.0);
  paths["robot_b"] = make_straight_path(0.0, 1.0, 0.0, -4.0);

  const std::set<std::string> already_paused = {"robot_a", "robot_b"};

  const auto to_pause = easyfleet::compute_robots_to_pause(
    paths, easyfleet::ConflictParams(), already_paused);

  EXPECT_EQ(to_pause, already_paused);
}

TEST(ConflictDetectorTest, DecisionsAreReportedOnlyForNearbyPairs)
{
  std::map<std::string, nav_msgs::msg::Path> paths;
  paths["robot_a"] = make_straight_path(0.0, 0.0, 1.0, 0.0);
  paths["robot_b"] = make_straight_path(0.0, 1.0, 0.0, -4.0);
  paths["robot_c"] = make_straight_path(50.0, 50.0, 51.0, 50.0);  // far from both

  std::vector<easyfleet::ConflictDecision> decisions;
  easyfleet::compute_robots_to_pause(paths, easyfleet::ConflictParams(), {}, &decisions);

  // Only the (robot_a, robot_b) pair is within proximity_radius_m;
  // robot_c never appears in any reported decision.
  ASSERT_EQ(decisions.size(), 1u);
  EXPECT_TRUE(decisions[0].in_conflict);
  EXPECT_EQ(decisions[0].yielding_robot, "robot_b");
  EXPECT_FALSE(decisions[0].reason.empty());
}

TEST(ConflictDetectorTest, DecisionsReportNoRiskForNearButNonConvergingPaths)
{
  std::map<std::string, nav_msgs::msg::Path> paths;
  paths["robot_a"] = make_straight_path(0.0, 0.0, 5.0, 0.0);
  paths["robot_b"] = make_straight_path(0.0, 2.0, 5.0, 2.0);

  std::vector<easyfleet::ConflictDecision> decisions;
  const auto to_pause = easyfleet::compute_robots_to_pause(
    paths, easyfleet::ConflictParams(), {}, &decisions);

  EXPECT_TRUE(to_pause.empty());
  ASSERT_EQ(decisions.size(), 1u);
  EXPECT_FALSE(decisions[0].in_conflict);
  EXPECT_TRUE(decisions[0].yielding_robot.empty());
  EXPECT_FALSE(decisions[0].reason.empty());
}
