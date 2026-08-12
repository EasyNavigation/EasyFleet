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

#ifndef EASYFLEET_CORE__DETAIL__ACTION_CLIENT_IMPL_HPP_
#define EASYFLEET_CORE__DETAIL__ACTION_CLIENT_IMPL_HPP_

// Out-of-line member definitions for easyfleet_core::ActionClient<ActionT>.
// Included from the bottom of easyfleet_core/action_client.hpp. Not meant to be
// included directly: templates cannot be compiled into easyfleet_core's
// .cpp/.so, so the implementation lives here to keep the class declaration
// in action_client.hpp free of member bodies.

#include <atomic>
#include <future>
#include <stdexcept>
#include <string>
#include <utility>

#include "easyfleet_core/detail/param_utils.hpp"

namespace easyfleet_core
{

template<typename ActionT>
typename ActionClient<ActionT>::SharedPtr ActionClient<ActionT>::create(
  rclcpp::Node * parent_node,
  const std::string & action_name,
  std::chrono::milliseconds default_server_timeout)
{
  return SharedPtr(new ActionClient(parent_node, action_name, default_server_timeout));
}

template<typename ActionT>
ActionClient<ActionT>::ActionClient(
  rclcpp::Node * parent_node,
  const std::string & action_name,
  std::chrono::milliseconds default_server_timeout)
: action_name_(action_name),
  default_server_timeout_(default_server_timeout)
{
  if (parent_node == nullptr) {
    throw std::invalid_argument("ActionClient: parent_node must not be null");
  }

  static std::atomic<uint64_t> instance_counter{0};
  const std::string node_name =
    detail::sanitize_identifier(
    std::string(parent_node->get_name()) + "_ac_" +
    detail::sanitize_parameter_name(action_name)) +
    "_" + std::to_string(instance_counter.fetch_add(1));

  rclcpp::NodeOptions options;
  options.start_parameter_services(false);
  options.start_parameter_event_publisher(false);
  options.use_global_arguments(false);

  internal_node_ = std::make_shared<rclcpp::Node>(
    node_name, parent_node->get_namespace(), options);

  client_ = rclcpp_action::create_client<ActionT>(internal_node_, action_name_);

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(internal_node_);
  spin_thread_ = std::thread([this] {executor_->spin();});
  // executor_->cancel() only reliably interrupts a spin() that has already
  // started; calling it any earlier (e.g. if this object is destroyed right
  // after construction) can race with the thread above and block forever.
  while (!executor_->is_spinning()) {
    std::this_thread::yield();
  }
}

template<typename ActionT>
ActionClient<ActionT>::~ActionClient()
{
  if (executor_) {
    executor_->cancel();
  }
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
  if (executor_ && internal_node_) {
    executor_->remove_node(internal_node_);
  }
}

template<typename ActionT>
const std::string & ActionClient<ActionT>::get_action_name() const noexcept
{
  return action_name_;
}

template<typename ActionT>
bool ActionClient<ActionT>::is_server_ready() const
{
  return client_->action_server_is_ready();
}

template<typename ActionT>
bool ActionClient<ActionT>::wait_for_server()
{
  return client_->wait_for_action_server(default_server_timeout_);
}

template<typename ActionT>
bool ActionClient<ActionT>::wait_for_server(std::chrono::milliseconds timeout)
{
  return client_->wait_for_action_server(timeout);
}

template<typename ActionT>
bool ActionClient<ActionT>::send_goal(
  const Goal & goal,
  ResultCallback result_callback,
  FeedbackCallback feedback_callback,
  GoalResponseCallback goal_response_callback)
{
  if (!client_->action_server_is_ready()) {
    if (result_callback) {
      result_callback(GoalResult{GoalOutcome::SERVER_UNAVAILABLE, {}, nullptr});
    }
    return false;
  }

  typename rclcpp_action::Client<ActionT>::SendGoalOptions send_options;

  if (feedback_callback) {
    send_options.feedback_callback =
      [feedback_callback](
      typename ClientGoalHandle::SharedPtr /*handle*/,
      const std::shared_ptr<const Feedback> feedback)
      {
        feedback_callback(feedback);
      };
  }

  send_options.goal_response_callback =
    [this, result_callback, goal_response_callback](typename ClientGoalHandle::SharedPtr handle)
    {
      if (!handle) {
        if (goal_response_callback) {
          goal_response_callback(false, rclcpp_action::GoalUUID{});
        }
        if (result_callback) {
          result_callback(GoalResult{GoalOutcome::REJECTED, {}, nullptr});
        }
        return;
      }
      {
        std::lock_guard<std::mutex> lock(goals_mutex_);
        active_goals_[rclcpp_action::to_string(handle->get_goal_id())] = handle;
      }
      if (goal_response_callback) {
        goal_response_callback(true, handle->get_goal_id());
      }
    };

  send_options.result_callback =
    [this, result_callback](const typename ClientGoalHandle::WrappedResult & wrapped)
    {
      {
        std::lock_guard<std::mutex> lock(goals_mutex_);
        active_goals_.erase(rclcpp_action::to_string(wrapped.goal_id));
      }
      if (result_callback) {
        result_callback(to_goal_result(wrapped));
      }
    };

  client_->async_send_goal(goal, send_options);
  return true;
}

template<typename ActionT>
typename ActionClient<ActionT>::GoalResult ActionClient<ActionT>::send_goal_and_wait(
  const Goal & goal,
  FeedbackCallback feedback_callback,
  std::chrono::milliseconds timeout)
{
  auto promise = std::make_shared<std::promise<GoalResult>>();
  auto future = promise->get_future();

  send_goal(
    goal,
    [promise](const GoalResult & result) {
      promise->set_value(result);
    },
    feedback_callback);

  if (timeout.count() <= 0) {
    return future.get();
  }
  if (future.wait_for(timeout) == std::future_status::timeout) {
    return GoalResult{GoalOutcome::TIMEOUT, {}, nullptr};
  }
  return future.get();
}

template<typename ActionT>
bool ActionClient<ActionT>::cancel_goal(const rclcpp_action::GoalUUID & goal_id)
{
  typename ClientGoalHandle::SharedPtr handle;
  {
    std::lock_guard<std::mutex> lock(goals_mutex_);
    auto it = active_goals_.find(rclcpp_action::to_string(goal_id));
    if (it == active_goals_.end()) {
      return false;
    }
    handle = it->second;
  }
  client_->async_cancel_goal(handle);
  return true;
}

template<typename ActionT>
void ActionClient<ActionT>::cancel_all_goals()
{
  client_->async_cancel_all_goals();
}

template<typename ActionT>
std::size_t ActionClient<ActionT>::active_goal_count() const
{
  std::lock_guard<std::mutex> lock(goals_mutex_);
  return active_goals_.size();
}

template<typename ActionT>
typename ActionClient<ActionT>::GoalResult ActionClient<ActionT>::to_goal_result(
  const typename ClientGoalHandle::WrappedResult & wrapped)
{
  GoalOutcome outcome;
  switch (wrapped.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      outcome = GoalOutcome::SUCCEEDED;
      break;
    case rclcpp_action::ResultCode::CANCELED:
      outcome = GoalOutcome::CANCELED;
      break;
    case rclcpp_action::ResultCode::ABORTED:
    default:
      outcome = GoalOutcome::ABORTED;
      break;
  }
  return GoalResult{outcome, wrapped.goal_id, wrapped.result};
}

}  // namespace easyfleet_core

#endif  // EASYFLEET_CORE__DETAIL__ACTION_CLIENT_IMPL_HPP_
