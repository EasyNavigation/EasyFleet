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

#ifndef EASYFLEET_MISSION_MANAGER__TEST__TEST_NAV_FAKE_CAPABILITY_HPP_
#define EASYFLEET_MISSION_MANAGER__TEST__TEST_NAV_FAKE_CAPABILITY_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easyfleet_core/capability.hpp"
#include "easyfleet_core/navigation_action_server_base.hpp"

namespace easyfleet_mission_manager_test
{

/// Magic waypoint id (parameters_json's goal_id) that makes
/// TestNavActionServer reject the goal outright -- the only way this fake
/// ever produces a genuine FAILED (as opposed to CANCELED) outcome.
inline const std::string kRejectWaypointId = "__reject__";

/// Minimal fake navigation action server for exercising RobotHandle end to
/// end: ignores the goal content entirely (RobotHandle::run_capability()'s
/// "navigation" goals only ever carry parameters_json, which this doesn't
/// need to actually parse, except to check for kRejectWaypointId), succeeds
/// after `<action_name>.mock_duration_seconds` (0 by default -- fast
/// tests), and honors cancellation immediately.
class TestNavActionServer : public easyfleet_core::NavigationActionServerBase
{
public:
  TestNavActionServer(
    rclcpp_lifecycle::LifecycleNode & node,
    const std::string & action_name)
  : easyfleet_core::NavigationActionServerBase(node, action_name)
  {
    mock_duration_s_ = node.declare_parameter(action_name + ".mock_duration_seconds", 0.0);
  }

protected:
  rclcpp_action::GoalResponse on_goal_received(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const Goal> goal) override
  {
    if (goal->parameters_json.find(kRejectWaypointId) != std::string::npos) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  void on_execute(const GoalHandleSharedPtr goal_handle) override
  {
    const auto step = std::chrono::milliseconds(20);
    auto remaining = std::chrono::duration<double>(mock_duration_s_);
    while (remaining.count() > 0.0) {
      if (goal_handle->is_canceling()) {
        auto result = std::make_shared<Result>();
        goal_handle->canceled(result);
        return;
      }
      if (is_preempt_requested() || is_shutdown_requested()) {
        auto result = std::make_shared<Result>();
        goal_handle->abort(result);
        return;
      }
      std::this_thread::sleep_for(step);
      remaining -= step;
    }
    auto result = std::make_shared<Result>();
    goal_handle->succeed(result);
  }

private:
  double mock_duration_s_{0.0};
};

class TestNavCapability : public easyfleet_core::Capability<TestNavActionServer>
{
public:
  explicit TestNavCapability(
    const std::string & capability_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : easyfleet_core::Capability<TestNavActionServer>(capability_name, options)
  {
  }
};

}  // namespace easyfleet_mission_manager_test

#endif  // EASYFLEET_MISSION_MANAGER__TEST__TEST_NAV_FAKE_CAPABILITY_HPP_
