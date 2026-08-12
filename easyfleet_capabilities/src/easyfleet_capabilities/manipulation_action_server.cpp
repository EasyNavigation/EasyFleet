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

#include "manipulation_capability/execute_trajectory_action_server.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

namespace manipulation_capability
{

ExecuteTrajectoryActionServer::ExecuteTrajectoryActionServer(
  rclcpp_lifecycle::LifecycleNode * node,
  const std::string & action_name)
: arch_mockup::ActionServerBase<moveit_msgs::action::ExecuteTrajectory>(node, action_name)
{
  mock_execution_duration_s_ =
    node->declare_parameter(action_name + ".mock_execution_duration", 3.0);
  mock_feedback_period_s_ =
    node->declare_parameter(action_name + ".mock_feedback_period", 0.3);
}

rclcpp_action::GoalResponse ExecuteTrajectoryActionServer::on_goal_received(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const Goal> goal)
{
  const auto & trajectory = goal->trajectory;
  if (trajectory.joint_trajectory.points.empty() &&
    trajectory.multi_dof_joint_trajectory.points.empty())
  {
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void ExecuteTrajectoryActionServer::on_execute(const GoalHandleSharedPtr goal_handle)
{
  auto feedback = std::make_shared<Feedback>();
  feedback->state = "EXECUTING";

  const int num_steps = std::max(
    1, static_cast<int>(mock_execution_duration_s_ / mock_feedback_period_s_));

  for (int i = 0; i < num_steps; ++i) {
    if (goal_handle->is_canceling()) {
      auto result = std::make_shared<Result>();
      result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::PREEMPTED;
      goal_handle->canceled(result);
      return;
    }
    if (is_preempt_requested()) {
      auto result = std::make_shared<Result>();
      result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::PREEMPTED;
      goal_handle->abort(result);
      return;
    }
    if (is_shutdown_requested()) {
      auto result = std::make_shared<Result>();
      result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
      goal_handle->abort(result);
      return;
    }

    std::this_thread::sleep_for(std::chrono::duration<double>(mock_feedback_period_s_));
    goal_handle->publish_feedback(feedback);
  }

  auto result = std::make_shared<Result>();
  result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  goal_handle->succeed(result);
}

}  // namespace manipulation_capability
