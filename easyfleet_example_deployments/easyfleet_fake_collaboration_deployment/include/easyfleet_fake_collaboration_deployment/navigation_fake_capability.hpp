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

#ifndef EASYFLEET_FAKE_COLLABORATION_DEPLOYMENT__NAVIGATION_FAKE_CAPABILITY_HPP_
#define EASYFLEET_FAKE_COLLABORATION_DEPLOYMENT__NAVIGATION_FAKE_CAPABILITY_HPP_

#include <string>

#include "easyfleet_interfaces/action/navigation.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easyfleet_core/capability.hpp"
#include "easyfleet_core/navigation_action_server_base.hpp"

namespace easyfleet_fake_collaboration_deployment
{

/// @brief Fake/mock implementation of the easyfleet_interfaces/Navigation action.
/**
 * A reference implementation of `easyfleet_core::NavigationActionServerBase`:
 * it does not perform real path planning, localization or obstacle
 * avoidance. It simulates navigation progress over a configurable duration,
 * publishing feedback at a configurable rate, so that the rest of a
 * multi-capability system can be integrated and tested before a real
 * navigation stack (Nav2, EasyNav, ...) is wired in through its own
 * `NavigationActionServerBase` subclass.
 */
class NavigationFakeActionServer : public easyfleet_core::NavigationActionServerBase
{
public:
  /// @brief Constructs the action server.
  /// @param node Lifecycle node that will host this action server.
  /// @param action_name Name under which the action is advertised.
  NavigationFakeActionServer(
    rclcpp_lifecycle::LifecycleNode & node,
    const std::string & action_name);

protected:
  /// @brief Always accepts (this mock never rejects a goal).
  /// @param uuid Id of the incoming goal.
  /// @param goal Goal content (unused).
  /// @return Always `ACCEPT_AND_EXECUTE`.
  rclcpp_action::GoalResponse on_goal_received(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Goal> goal) override;

  /// @brief Simulates navigation progress over `mock_navigation_duration_s_`,
  /// publishing feedback every `mock_feedback_period_s_`.
  /// @param goal_handle Handle of the accepted goal to run to completion.
  void on_execute(const GoalHandleSharedPtr goal_handle) override;

private:
  double mock_navigation_duration_s_;
  double mock_feedback_period_s_;
  double mock_initial_distance_m_;
};

/// @brief The "navigation" capability, backed by the fake/mock action server: a
/// lifecycle node advertising a (mock) easyfleet_interfaces/Navigation
/// action, described by config/navigation.json.
class NavigationFakeCapability : public easyfleet_core::Capability<NavigationFakeActionServer>
{
public:
  /// @brief Constructs the capability node.
  /// @param options Forwarded to the underlying `LifecycleNode`.
  explicit NavigationFakeCapability(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
};

}  // namespace easyfleet_fake_collaboration_deployment

#endif  // EASYFLEET_FAKE_COLLABORATION_DEPLOYMENT__NAVIGATION_FAKE_CAPABILITY_HPP_
