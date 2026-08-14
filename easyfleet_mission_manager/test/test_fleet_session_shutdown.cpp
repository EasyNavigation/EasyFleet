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

// FleetSession::shutdown() calls rclcpp::shutdown(), invalidating the ROS
// context for the rest of the process -- so, like
// easyfleet_core/test/test_deployment_shutdown.cpp, it gets its own test
// binary with exactly this one test, rather than sharing a process (and
// therefore an rclcpp context) with every other FleetSession/RobotHandle
// test.

#include "easyfleet_core/init.hpp"
#include "easyfleet_mission_manager/fleet_session.hpp"

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

TEST(FleetSessionShutdownTest, ShutdownStopsTheContext)
{
  easyfleet::FleetSession session;
  EXPECT_TRUE(rclcpp::ok());

  session.shutdown();

  EXPECT_FALSE(rclcpp::ok());
}

int main(int argc, char ** argv)
{
  easyfleet::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
