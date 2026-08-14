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

// Deployment::shutdown() calls rclcpp::shutdown(), invalidating the ROS
// context for the rest of the process -- so it gets its own test binary,
// with exactly this one test, rather than sharing a process (and
// therefore an rclcpp context) with test_deployment.cpp's other tests.

#include <string>

#include "easyfleet_core/deployment.hpp"

#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"

TEST(DeploymentShutdownTest, ShutdownReachesFinalizedFromActive)
{
  easyfleet::Deployment deployment("robot_1");
  deployment.add_capability("test_capability/fibonacci");
  ASSERT_EQ(deployment.capabilities().size(), 1u);

  // Deliberately no capabilities_file override: start() would exit(1) on
  // configure/activate failure, and shutdown() from PRIMARY_STATE_ACTIVE
  // isn't the only reachable-states case worth covering -- shutdown_node()
  // must also work cleanly from UNCONFIGURED (the state right after
  // add_capability(), before start() is ever called), which is exactly
  // what's exercised here.
  EXPECT_EQ(
    deployment.capabilities()[0]->get_current_state_id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

  deployment.shutdown();

  EXPECT_EQ(
    deployment.capabilities()[0]->get_current_state_id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED);
  EXPECT_FALSE(rclcpp::ok());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
