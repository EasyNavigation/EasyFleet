// Copyright 2026 Intelligent Robotics Lab
//
// This file is part of the projects Arquimea-URJC and AURORAS
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

#ifndef ARCH_MOCKUP__TEST__TEST_CAPABILITY_SERVER_HPP_
#define ARCH_MOCKUP__TEST__TEST_CAPABILITY_SERVER_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "example_interfaces/action/fibonacci.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "arch_mockup/action_server_base.hpp"

namespace arch_mockup_test
{

using Fibonacci = example_interfaces::action::Fibonacci;

/// Minimal ActionServerBase<Fibonacci> subclass usable as the ActionServerT
/// of arch_mockup::Capability<T>: constructible from a LifecycleNode pointer
/// plus an action name, as Capability<T> requires.
///
/// Settles goals instantly by default (`<action_name>.mock_delay_seconds`
/// defaults to 0), which is what most tests want. Tests that need to
/// observe a goal while it is still executing (e.g. to check
/// `CapabilityStatus.busy`) can override that parameter to hold the goal
/// open for a bit before it succeeds.
class TestCapabilityActionServer : public arch_mockup::ActionServerBase<Fibonacci>
{
public:
  TestCapabilityActionServer(
    rclcpp_lifecycle::LifecycleNode * node,
    const std::string & action_name)
  : arch_mockup::ActionServerBase<Fibonacci>(node, action_name)
  {
    mock_delay_s_ = node->declare_parameter(action_name + ".mock_delay_seconds", 0.0);
  }

protected:
  rclcpp_action::GoalResponse on_goal_received(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const Fibonacci::Goal> goal) override
  {
    if (goal->order < 0) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  void on_execute(const GoalHandleSharedPtr goal_handle) override
  {
    if (mock_delay_s_ > 0.0) {
      std::this_thread::sleep_for(std::chrono::duration<double>(mock_delay_s_));
    }
    auto result = std::make_shared<Fibonacci::Result>();
    result->sequence = {0, 1};
    goal_handle->succeed(result);
  }

private:
  double mock_delay_s_{0.0};
};

}  // namespace arch_mockup_test

#endif  // ARCH_MOCKUP__TEST__TEST_CAPABILITY_SERVER_HPP_
