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

#ifndef NAVIGATION_CAPABILITY__NAVIGATE_TO_POSE_ACTION_SERVER_HPP_
#define NAVIGATION_CAPABILITY__NAVIGATE_TO_POSE_ACTION_SERVER_HPP_

#include <string>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "arch_mockup/action_server_base.hpp"

namespace navigation_capability
{

/// Mock implementation of the nav2_msgs/NavigateToPose action.
/**
 * Part of the arch_mockup architecture: it does not perform real path
 * planning, localization or obstacle avoidance. It simulates navigation
 * progress over a configurable duration, publishing feedback at a
 * configurable rate, so that the rest of a multi-capability system can be
 * integrated and tested before a real navigation stack is wired in.
 */
class NavigateToPoseActionServer
  : public arch_mockup::ActionServerBase<nav2_msgs::action::NavigateToPose>
{
public:
  NavigateToPoseActionServer(
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

}  // namespace navigation_capability

#endif  // NAVIGATION_CAPABILITY__NAVIGATE_TO_POSE_ACTION_SERVER_HPP_
