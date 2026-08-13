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

#include "behaviortree_cpp/bt_factory.h"

#include "easyfleet_easynav_deployment/bt_nodes/finish.hpp"
#include "easyfleet_easynav_deployment/bt_nodes/start_off.hpp"

using namespace std::chrono_literals;

namespace
{

/// Ticks `tree` at 20 Hz until it leaves RUNNING or `timeout` elapses.
BT::NodeStatus tick_until_done(BT::Tree & tree, std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  while (status == BT::NodeStatus::RUNNING && std::chrono::steady_clock::now() < deadline) {
    status = tree.tickOnce();
    std::this_thread::sleep_for(50ms);
  }
  return status;
}

}  // namespace

TEST(StartOffTest, TakesAboutTwoSecondsAndSucceeds)
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<easyfleet_easynav_deployment::StartOff>("StartOff");
  auto tree = factory.createTreeFromText(
    R"(<root BTCPP_format="4"><BehaviorTree ID="Test"><StartOff/></BehaviorTree></root>)");

  const auto start = std::chrono::steady_clock::now();
  const auto status = tick_until_done(tree, 5s);
  const auto elapsed = std::chrono::steady_clock::now() - start;

  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
  EXPECT_GE(elapsed, 1900ms);
  EXPECT_LT(elapsed, 4s);
}

TEST(FinishTest, TakesAboutTwoSecondsAndSucceeds)
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<easyfleet_easynav_deployment::Finish>("Finish");
  auto tree = factory.createTreeFromText(
    R"(<root BTCPP_format="4"><BehaviorTree ID="Test"><Finish/></BehaviorTree></root>)");

  const auto start = std::chrono::steady_clock::now();
  const auto status = tick_until_done(tree, 5s);
  const auto elapsed = std::chrono::steady_clock::now() - start;

  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
  EXPECT_GE(elapsed, 1900ms);
  EXPECT_LT(elapsed, 4s);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
