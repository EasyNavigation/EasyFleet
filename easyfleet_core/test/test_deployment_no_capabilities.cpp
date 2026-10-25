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

// Own process: test_deployment.cpp sets a global "capabilities" parameter
// override for its whole process, and this test needs it unset.

#include "easyfleet_core/deployment.hpp"

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

TEST(DeploymentNoCapabilitiesTest, TryAddCapabilitiesFromParametersFailsWhenNoneAreConfigured)
{
  easyfleet::Deployment deployment("robot_1");
  EXPECT_FALSE(deployment.try_add_capabilities_from_parameters("easyfleet_core"));
  EXPECT_TRUE(deployment.capabilities().empty());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
