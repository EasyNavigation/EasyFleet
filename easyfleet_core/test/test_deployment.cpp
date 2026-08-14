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

#include "easyfleet_core/deployment.hpp"

#include "gtest/gtest.h"
#include "pluginlib/exceptions.hpp"
#include "rclcpp/rclcpp.hpp"

// easyfleet_core itself has no concrete CapabilityFactory plugin of its
// own to load (the fake/real backends registering one all live in
// downstream deployment packages), so these tests only cover what doesn't
// need one: identity and the failure path for an unknown plugin. The
// success path (add_capability() -> start() -> run() -> shutdown()) is
// exercised end to end by whichever deployment package registers a real
// plugin -- see easyfleet_easynav_collaboration_deployment.

TEST(DeploymentTest, NameReturnsWhatWasConstructedWith)
{
  easyfleet::Deployment deployment("robot_1");
  EXPECT_EQ(deployment.name(), "robot_1");
}

TEST(DeploymentTest, StartsWithNoCapabilities)
{
  easyfleet::Deployment deployment("robot_1");
  EXPECT_TRUE(deployment.capabilities().empty());
}

TEST(DeploymentTest, AddCapabilityWithUnknownPluginThrows)
{
  easyfleet::Deployment deployment("robot_1");
  EXPECT_THROW(
    deployment.add_capability("nonexistent_type/nonexistent_plugin"),
    pluginlib::PluginlibException);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
