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

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "easyfleet_navigation_manager/fleet_obstacles.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace
{

using easyfleet::FleetObstacleParams;
using easyfleet::RobotPose;
using easyfleet::RobotPoseTracker;

geometry_msgs::msg::TransformStamped transform(
  const std::string & parent, const std::string & child, double x, double y, double yaw,
  int32_t sec = 1)
{
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = parent;
  t.header.stamp.sec = sec;
  t.child_frame_id = child;
  t.transform.translation.x = x;
  t.transform.translation.y = y;
  t.transform.rotation.z = std::sin(yaw / 2.0);
  t.transform.rotation.w = std::cos(yaw / 2.0);
  return t;
}

tf2_msgs::msg::TFMessage message(std::vector<geometry_msgs::msg::TransformStamped> transforms)
{
  tf2_msgs::msg::TFMessage msg;
  msg.transforms = std::move(transforms);
  return msg;
}

std::vector<std::pair<float, float>> points(const sensor_msgs::msg::PointCloud2 & cloud)
{
  std::vector<std::pair<float, float>> out;
  if (cloud.width == 0) {
    return out;
  }
  sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y(cloud, "y");
  for (; x != x.end(); ++x, ++y) {
    out.emplace_back(*x, *y);
  }
  return out;
}

const auto kNow = std::chrono::steady_clock::now();

TEST(RobotPoseTracker, ChainsMapOdomAndBase)
{
  RobotPoseTracker tracker("robot_1", "base_footprint");
  EXPECT_FALSE(tracker.pose());
  // map -> odom: 10 m ahead and turned 90 deg; odom -> base: 2 m ahead of odom.
  tracker.update(message({transform("robot_1/map", "robot_1/odom", 10.0, 0.0, M_PI / 2)}), kNow);
  EXPECT_FALSE(tracker.pose()) << "no base frame yet";
  tracker.update(
    message({transform("robot_1/odom", "robot_1/base_footprint", 2.0, 0.0, 0.0, 3)}), kNow);
  const auto pose = tracker.pose();
  ASSERT_TRUE(pose);
  EXPECT_NEAR(pose->x, 10.0, 1e-9);
  EXPECT_NEAR(pose->y, 2.0, 1e-9);
  EXPECT_EQ(pose->stamp.sec, 3) << "the newest stamp of the chain";
}

TEST(RobotPoseTracker, FollowsTheNewestTransformAndIgnoresALateOlderOne)
{
  RobotPoseTracker tracker("r", "base_footprint");
  tracker.update(
    message({transform("r/map", "r/odom", 0.0, 0.0, 0.0),
      transform("r/odom", "r/base_footprint", 1.0, 0.0, 0.0, 5)}), kNow);
  tracker.update(message({transform("r/odom", "r/base_footprint", 3.0, 0.0, 0.0, 6)}), kNow);
  EXPECT_NEAR(tracker.pose()->x, 3.0, 1e-9);
  tracker.update(message({transform("r/odom", "r/base_footprint", 9.0, 0.0, 0.0, 4)}), kNow);
  EXPECT_NEAR(tracker.pose()->x, 3.0, 1e-9) << "an older transform arriving late";
}

TEST(RobotPoseTracker, LongerChainsAndOtherBaseFrames)
{
  RobotPoseTracker tracker("r", "base_link");
  tracker.update(
    message({transform("r/map", "r/odom", 1.0, 1.0, 0.0),
      transform("r/odom", "r/base_footprint", 1.0, 0.0, 0.0),
      transform("r/base_footprint", "r/base_link", 0.0, 0.0, 0.0)}), kNow);
  ASSERT_TRUE(tracker.pose());
  EXPECT_NEAR(tracker.pose()->x, 2.0, 1e-9);
  EXPECT_NEAR(tracker.pose()->y, 1.0, 1e-9);
}

TEST(RobotPoseTracker, NeedsTheChainToReachItsOwnMap)
{
  RobotPoseTracker tracker("r", "base_footprint");
  // Another robot's frames, and a chain that ends in a frame that is not r/map.
  tracker.update(
    message({transform("q/map", "q/odom", 0.0, 0.0, 0.0),
      transform("q/odom", "q/base_footprint", 1.0, 0.0, 0.0),
      transform("world", "r/odom", 0.0, 0.0, 0.0),
      transform("r/odom", "r/base_footprint", 1.0, 0.0, 0.0)}), kNow);
  EXPECT_FALSE(tracker.pose());
  tracker.update(message({transform("r/map", "r/odom", 0.5, 0.0, 0.0)}), kNow);
  ASSERT_TRUE(tracker.pose());
  EXPECT_NEAR(tracker.pose()->x, 1.5, 1e-9);
}

TEST(RobotPoseTracker, ALoopInTheTreeDoesNotHang)
{
  RobotPoseTracker tracker("r", "base_footprint");
  tracker.update(
    message({transform("r/b", "r/a", 0.0, 0.0, 0.0), transform("r/a", "r/b", 0.0, 0.0, 0.0),
      transform("r/a", "r/base_footprint", 0.0, 0.0, 0.0)}), kNow);
  EXPECT_FALSE(tracker.pose());
}

RobotPose at(double x, double y, int32_t sec = 7)
{
  RobotPose pose;
  pose.x = x;
  pose.y = y;
  pose.stamp.sec = sec;
  pose.received = kNow;
  return pose;
}

TEST(FleetObstacleCloud, HoldsADiscPerOtherRobotInTheReceiversMap)
{
  FleetObstacleParams params;
  params.radius_m = 0.3;
  params.height_m = 0.2;
  const std::map<std::string, RobotPose> poses{
    {"robot_1", at(0.0, 0.0, 11)}, {"robot_2", at(5.0, 0.0)}, {"robot_3", at(-4.0, 3.0)}};
  const auto cloud = easyfleet::make_fleet_obstacle_cloud("robot_1", poses, params);
  EXPECT_EQ(cloud.header.frame_id, "robot_1/map");
  EXPECT_EQ(cloud.header.stamp.sec, 11) << "the receiver's own stamp";
  const auto pts = points(cloud);
  ASSERT_FALSE(pts.empty());
  int near_2 = 0, near_3 = 0;
  for (const auto & [x, y] : pts) {
    const double d2 = std::hypot(x - 5.0, y - 0.0);
    const double d3 = std::hypot(x + 4.0, y - 3.0);
    EXPECT_TRUE(d2 <= 0.3 + 1e-4 || d3 <= 0.3 + 1e-4) << x << ", " << y;
    EXPECT_GT(std::hypot(x, y), 1.0) << "never the receiver itself";
    near_2 += d2 <= 0.3 + 1e-4;
    near_3 += d3 <= 0.3 + 1e-4;
  }
  EXPECT_EQ(near_2, near_3);
  EXPECT_GT(near_2, 20) << "a filled disc, not a few points";
  sensor_msgs::PointCloud2ConstIterator<float> z(cloud, "z");
  EXPECT_FLOAT_EQ(*z, 0.2f);
}

TEST(FleetObstacleCloud, DiscReachesItsEdgeAndItsCenter)
{
  FleetObstacleParams params;
  params.radius_m = 0.25;
  const auto cloud = easyfleet::make_fleet_obstacle_cloud(
    "a", {{"a", at(0.0, 0.0)}, {"b", at(2.0, 2.0)}}, params);
  double min_d = 1e9, max_d = 0.0;
  for (const auto & [x, y] : points(cloud)) {
    const double d = std::hypot(x - 2.0, y - 2.0);
    min_d = std::min(min_d, d);
    max_d = std::max(max_d, d);
  }
  EXPECT_NEAR(min_d, 0.0, 1e-4);
  EXPECT_NEAR(max_d, 0.25, 1e-4);
}

TEST(FleetObstacleCloud, EmptyWithoutOtherRobotsOrWithoutTheReceiversPose)
{
  FleetObstacleParams params;
  const auto alone = easyfleet::make_fleet_obstacle_cloud("a", {{"a", at(0.0, 0.0)}}, params);
  EXPECT_EQ(alone.header.frame_id, "a/map");
  EXPECT_TRUE(points(alone).empty());
  const auto unknown = easyfleet::make_fleet_obstacle_cloud("a", {{"b", at(1.0, 0.0)}}, params);
  EXPECT_TRUE(points(unknown).empty()) << "no stamp to place the cloud with";
}

}  // namespace
