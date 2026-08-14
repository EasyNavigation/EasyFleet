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

#include <string>
#include <vector>

#include "easyfleet_mission_manager/mission_helpers.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "gtest/gtest.h"

using easyfleet_mission_manager::CapabilityInfo;
using easyfleet_mission_manager::find_robot_capability;
using easyfleet_mission_manager::make_manipulation_goal;
using easyfleet_mission_manager::make_navigation_goal;
using easyfleet_mission_manager::make_perception_goal;
using easyfleet_mission_manager::Manipulation;
using easyfleet_mission_manager::Navigation;

namespace
{

CapabilityInfo make_info(
  const std::string & robot, const std::string & capability, bool active = true)
{
  CapabilityInfo info;
  info.robot = robot;
  info.capability = capability;
  info.action_name = "/" + robot + "/" + capability;
  info.active = active;
  return info;
}

}  // namespace

TEST(FindRobotCapabilityTest, EmptyListReturnsNullopt)
{
  std::vector<CapabilityInfo> capabilities;
  EXPECT_FALSE(find_robot_capability(capabilities, "robot_1", "navigation"));
}

TEST(FindRobotCapabilityTest, FindsExactMatch)
{
  std::vector<CapabilityInfo> capabilities{make_info("robot_1", "navigation")};
  auto found = find_robot_capability(capabilities, "robot_1", "navigation");
  ASSERT_TRUE(found);
  EXPECT_EQ(found->get().action_name, "/robot_1/navigation");
}

TEST(FindRobotCapabilityTest, RobotNameMismatchReturnsNullopt)
{
  std::vector<CapabilityInfo> capabilities{make_info("robot_1", "navigation")};
  EXPECT_FALSE(find_robot_capability(capabilities, "robot_2", "navigation"));
}

TEST(FindRobotCapabilityTest, CapabilityTypeMismatchReturnsNullopt)
{
  std::vector<CapabilityInfo> capabilities{make_info("robot_1", "navigation")};
  EXPECT_FALSE(find_robot_capability(capabilities, "robot_1", "manipulation"));
}

TEST(FindRobotCapabilityTest, InactiveCapabilityIsNotFound)
{
  std::vector<CapabilityInfo> capabilities{make_info("robot_1", "navigation", /*active=*/ false)};
  EXPECT_FALSE(find_robot_capability(capabilities, "robot_1", "navigation"));
}

TEST(FindRobotCapabilityTest, DistinguishesSameCapabilityOnDifferentRobots)
{
  std::vector<CapabilityInfo> capabilities{
    make_info("robot_1", "navigation"),
    make_info("robot_2", "navigation"),
  };
  auto found_1 = find_robot_capability(capabilities, "robot_1", "navigation");
  auto found_2 = find_robot_capability(capabilities, "robot_2", "navigation");
  ASSERT_TRUE(found_1);
  ASSERT_TRUE(found_2);
  EXPECT_EQ(found_1->get().action_name, "/robot_1/navigation");
  EXPECT_EQ(found_2->get().action_name, "/robot_2/navigation");
}

TEST(FindRobotCapabilityTest, ReturnsFirstMatchWhenDuplicated)
{
  std::vector<CapabilityInfo> capabilities{
    make_info("robot_1", "navigation"),
    make_info("robot_1", "navigation"),
  };
  capabilities[1].action_name = "/robot_1/navigation_2";
  auto found = find_robot_capability(capabilities, "robot_1", "navigation");
  ASSERT_TRUE(found);
  EXPECT_EQ(found->get().action_name, "/robot_1/navigation");
}

TEST(FindRobotCapabilityTest, ReturnedReferenceAliasesTheOriginalVector)
{
  std::vector<CapabilityInfo> capabilities{make_info("robot_1", "navigation")};
  auto found = find_robot_capability(capabilities, "robot_1", "navigation");
  ASSERT_TRUE(found);
  // Mutating through the returned reference should be visible in the
  // original vector -- find_robot_capability returns an alias, not a copy.
  const_cast<CapabilityInfo &>(found->get()).action_name = "/robot_1/renamed";
  EXPECT_EQ(capabilities[0].action_name, "/robot_1/renamed");
}

TEST(MissionGoalBuildersTest, NavigationGoalHasExpectedFields)
{
  const Navigation::Goal goal = make_navigation_goal();
  EXPECT_EQ(goal.target_pose.header.frame_id, "map");
  EXPECT_DOUBLE_EQ(goal.target_pose.pose.position.x, 2.0);
  EXPECT_DOUBLE_EQ(goal.target_pose.pose.position.y, 1.0);
  EXPECT_DOUBLE_EQ(goal.target_pose.pose.orientation.w, 1.0);
}

TEST(MissionGoalBuildersTest, ManipulationGoalHasExpectedFields)
{
  const Manipulation::Goal goal = make_manipulation_goal();
  EXPECT_EQ(goal.mode, Manipulation::Goal::MODE_JOINT_TARGET);
  ASSERT_EQ(goal.joint_target.name.size(), 1u);
  EXPECT_EQ(goal.joint_target.name[0], "joint1");
  ASSERT_EQ(goal.joint_target.position.size(), 1u);
  EXPECT_DOUBLE_EQ(goal.joint_target.position[0], 1.0);
}

TEST(MissionGoalBuildersTest, PerceptionGoalHasExpectedFields)
{
  const auto goal = make_perception_goal();
  ASSERT_EQ(goal.object_classes.size(), 1u);
  EXPECT_EQ(goal.object_classes[0], "gato");
}

TEST(MissionGoalBuildersTest, NavigationGoalFromWaypointIdSetsParametersJson)
{
  const Navigation::Goal goal = make_navigation_goal("kitchen");
  EXPECT_EQ(goal.parameters_json, R"({"goal_id": "kitchen"})");
}

TEST(MissionGoalBuildersTest, ManipulationGoalFromPoseSetsPoseTargetMode)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = 1.5;
  pose.pose.orientation.w = 1.0;

  const Manipulation::Goal goal = make_manipulation_goal(pose);
  EXPECT_EQ(goal.mode, Manipulation::Goal::MODE_POSE_TARGET);
  EXPECT_EQ(goal.pose_target.header.frame_id, "map");
  EXPECT_DOUBLE_EQ(goal.pose_target.pose.position.x, 1.5);
  EXPECT_DOUBLE_EQ(goal.pose_target.pose.orientation.w, 1.0);
}

TEST(MissionGoalBuildersTest, PerceptionGoalFromObjectClassesSetsThemVerbatim)
{
  const std::vector<std::string> classes{"cat", "dog"};
  const auto goal = make_perception_goal(classes);
  ASSERT_EQ(goal.object_classes.size(), 2u);
  EXPECT_EQ(goal.object_classes[0], "cat");
  EXPECT_EQ(goal.object_classes[1], "dog");
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
