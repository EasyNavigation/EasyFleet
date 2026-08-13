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

#include "easyfleet_easynav_collaboration_deployment/manipulation_fake_capability.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

namespace easyfleet_easynav_collaboration_deployment
{

namespace
{
bool goal_target_is_empty(const ManipulationFakeActionServer::Goal & goal)
{
  switch (goal.mode) {
    case easyfleet_interfaces::action::Manipulation::Goal::MODE_JOINT_TARGET:
      return goal.joint_target.name.empty();
    case easyfleet_interfaces::action::Manipulation::Goal::MODE_POSE_TARGET:
      return goal.pose_target.header.frame_id.empty();
    case easyfleet_interfaces::action::Manipulation::Goal::MODE_NAMED_TASK:
      return goal.named_task.empty();
    default:
      return true;
  }
}
}  // namespace

ManipulationFakeActionServer::ManipulationFakeActionServer(
  rclcpp_lifecycle::LifecycleNode * node,
  const std::string & action_name)
: easyfleet_core::ManipulationActionServerBase(node, action_name)
{
  mock_execution_duration_s_ =
    node->declare_parameter(action_name + ".mock_execution_duration", 3.0);
  mock_feedback_period_s_ =
    node->declare_parameter(action_name + ".mock_feedback_period", 0.3);
}

rclcpp_action::GoalResponse ManipulationFakeActionServer::on_goal_received(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const Goal> goal)
{
  if (goal_target_is_empty(*goal)) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void ManipulationFakeActionServer::on_execute(const GoalHandleSharedPtr goal_handle)
{
  const auto goal = goal_handle->get_goal();

  auto feedback = std::make_shared<Feedback>();
  feedback->state = "EXECUTING";
  feedback->current_joint_state = goal->joint_target;

  const int num_steps = std::max(
    1, static_cast<int>(mock_execution_duration_s_ / mock_feedback_period_s_));

  for (int i = 0; i < num_steps; ++i) {
    if (goal_handle->is_canceling()) {
      auto result = std::make_shared<Result>();
      result->error_code = Result::CANCELED;
      result->error_msg = "Manipulation canceled by client.";
      goal_handle->canceled(result);
      return;
    }
    if (is_preempt_requested()) {
      auto result = std::make_shared<Result>();
      result->error_code = Result::ABORTED;
      result->error_msg = "Manipulation preempted by a newer goal.";
      goal_handle->abort(result);
      return;
    }
    if (is_shutdown_requested()) {
      auto result = std::make_shared<Result>();
      result->error_code = Result::ABORTED;
      result->error_msg = "Manipulation capability is shutting down.";
      goal_handle->abort(result);
      return;
    }

    std::this_thread::sleep_for(std::chrono::duration<double>(mock_feedback_period_s_));
    feedback->progress = static_cast<float>((i + 1) / static_cast<double>(num_steps));
    goal_handle->publish_feedback(feedback);
  }

  auto result = std::make_shared<Result>();
  result->error_code = Result::SUCCESS;
  result->final_joint_state = goal->joint_target;
  goal_handle->succeed(result);
}

ManipulationFakeCapability::ManipulationFakeCapability(const rclcpp::NodeOptions & options)
: easyfleet_core::Capability<ManipulationFakeActionServer>("manipulation", options)
{
}

}  // namespace easyfleet_easynav_collaboration_deployment
