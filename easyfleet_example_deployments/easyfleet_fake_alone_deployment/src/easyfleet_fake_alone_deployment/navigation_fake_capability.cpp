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

#include "easyfleet_fake_alone_deployment/navigation_fake_capability.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "builtin_interfaces/msg/duration.hpp"

namespace easyfleet_fake_alone_deployment
{

namespace
{
builtin_interfaces::msg::Duration to_duration_msg(double seconds)
{
  builtin_interfaces::msg::Duration duration;
  duration.sec = static_cast<int32_t>(seconds);
  duration.nanosec = static_cast<uint32_t>((seconds - duration.sec) * 1e9);
  return duration;
}
}  // namespace

NavigationFakeActionServer::NavigationFakeActionServer(
  rclcpp_lifecycle::LifecycleNode * node,
  const std::string & action_name)
: easyfleet_core::NavigationActionServerBase(node, action_name)
{
  mock_navigation_duration_s_ =
    node->declare_parameter(action_name + ".mock_navigation_duration", 5.0);
  mock_feedback_period_s_ =
    node->declare_parameter(action_name + ".mock_feedback_period", 0.5);
  mock_initial_distance_m_ =
    node->declare_parameter(action_name + ".mock_initial_distance", 5.0);
}

rclcpp_action::GoalResponse NavigationFakeActionServer::on_goal_received(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const Goal> goal)
{
  if (goal->target_pose.header.frame_id.empty()) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void NavigationFakeActionServer::on_execute(const GoalHandleSharedPtr goal_handle)
{
  const auto goal = goal_handle->get_goal();

  auto feedback = std::make_shared<Feedback>();
  // This mock has no real localization, so it simply reports the goal pose
  // as the (already reached) current pose while progress is simulated.
  feedback->current_pose = goal->target_pose;
  feedback->current_waypoint_index = 0;

  const int num_steps = std::max(
    1, static_cast<int>(mock_navigation_duration_s_ / mock_feedback_period_s_));

  for (int i = 0; i < num_steps; ++i) {
    if (goal_handle->is_canceling()) {
      auto result = std::make_shared<Result>();
      result->error_code = Result::CANCELED;
      result->error_msg = "Navigation canceled by client.";
      goal_handle->canceled(result);
      return;
    }
    if (is_preempt_requested()) {
      auto result = std::make_shared<Result>();
      result->error_code = Result::ABORTED;
      result->error_msg = "Navigation preempted by a newer goal.";
      goal_handle->abort(result);
      return;
    }
    if (is_shutdown_requested()) {
      auto result = std::make_shared<Result>();
      result->error_code = Result::ABORTED;
      result->error_msg = "Navigation capability is shutting down.";
      goal_handle->abort(result);
      return;
    }

    std::this_thread::sleep_for(std::chrono::duration<double>(mock_feedback_period_s_));

    const double elapsed_s = (i + 1) * mock_feedback_period_s_;
    const double ratio = static_cast<double>(i + 1) / num_steps;

    feedback->navigation_time = to_duration_msg(elapsed_s);
    feedback->estimated_time_remaining =
      to_duration_msg(std::max(0.0, mock_navigation_duration_s_ - elapsed_s));
    feedback->distance_remaining =
      static_cast<float>((1.0 - ratio) * mock_initial_distance_m_);
    feedback->number_of_recoveries = 0;
    goal_handle->publish_feedback(feedback);
  }

  auto result = std::make_shared<Result>();
  result->error_code = Result::SUCCESS;
  result->final_pose = goal->target_pose;
  result->total_time = to_duration_msg(mock_navigation_duration_s_);
  goal_handle->succeed(result);
}

NavigationFakeCapability::NavigationFakeCapability(const rclcpp::NodeOptions & options)
: easyfleet_core::Capability<NavigationFakeActionServer>("navigation", options)
{
}

}  // namespace easyfleet_fake_alone_deployment
