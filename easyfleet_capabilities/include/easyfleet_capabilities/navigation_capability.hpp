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

#ifndef EASYFLEET_CAPABILITIES__NAVIGATION_CAPABILITY_HPP_
#define EASYFLEET_CAPABILITIES__NAVIGATION_CAPABILITY_HPP_

#include <string>

#include "easyfleet_interfaces/action/navigation.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easyfleet_core/action_server_base.hpp"
#include "easyfleet_core/capability.hpp"

namespace easyfleet_capabilities
{

/// Mock implementation of the easyfleet_interfaces/Navigation action.
/**
 * Part of the EasyFleet architecture: it does not perform real path
 * planning, localization or obstacle avoidance. It simulates navigation
 * progress over a configurable duration, publishing feedback at a
 * configurable rate, so that the rest of a multi-capability system can be
 * integrated and tested before a real navigation stack is wired in.
 */
class NavigationActionServer
  : public easyfleet_core::ActionServerBase<easyfleet_interfaces::action::Navigation>
{
public:
  NavigationActionServer(
    rclcpp_lifecycle::LifecycleNode * node,
    const std::string & action_name);

protected:
  rclcpp_action::GoalResponse on_goal_received(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Goal> goal) override;

  void on_execute(const GoalHandleSharedPtr goal_handle) override;

private:
  double mock_navigation_duration_s_;
  double mock_feedback_period_s_;
  double mock_initial_distance_m_;
};

/// The "navigation" capability: a lifecycle node advertising a (mock)
/// easyfleet_interfaces/Navigation action, described by config/navigation.json.
class NavigationCapability : public easyfleet_core::Capability<NavigationActionServer>
{
public:
  explicit NavigationCapability(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
};

}  // namespace easyfleet_capabilities

#endif  // EASYFLEET_CAPABILITIES__NAVIGATION_CAPABILITY_HPP_
