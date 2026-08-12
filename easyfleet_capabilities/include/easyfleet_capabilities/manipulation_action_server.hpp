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

#ifndef MANIPULATION_CAPABILITY__EXECUTE_TRAJECTORY_ACTION_SERVER_HPP_
#define MANIPULATION_CAPABILITY__EXECUTE_TRAJECTORY_ACTION_SERVER_HPP_

#include <string>

#include "moveit_msgs/action/execute_trajectory.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "arch_mockup/action_server_base.hpp"

namespace manipulation_capability
{

/// Mock implementation of the moveit_msgs/ExecuteTrajectory action.
/**
 * Part of the arch_mockup architecture: it does not drive a real
 * manipulator or interact with real controllers. It simulates trajectory
 * execution progress over a configurable duration, publishing feedback at a
 * configurable rate, so that the rest of a multi-capability system can be
 * integrated and tested before a real MoveIt-driven manipulator is wired in.
 */
class ExecuteTrajectoryActionServer
  : public arch_mockup::ActionServerBase<moveit_msgs::action::ExecuteTrajectory>
{
public:
  ExecuteTrajectoryActionServer(
    rclcpp_lifecycle::LifecycleNode * node,
    const std::string & action_name);

protected:
  rclcpp_action::GoalResponse on_goal_received(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Goal> goal) override;

  void on_execute(const GoalHandleSharedPtr goal_handle) override;

private:
  double mock_execution_duration_s_;
  double mock_feedback_period_s_;
};

}  // namespace manipulation_capability

#endif  // MANIPULATION_CAPABILITY__EXECUTE_TRAJECTORY_ACTION_SERVER_HPP_
