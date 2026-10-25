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

#ifndef EASYFLEET_CORE__DETAIL__ACTION_SERVER_BASE_IMPL_HPP_
#define EASYFLEET_CORE__DETAIL__ACTION_SERVER_BASE_IMPL_HPP_

// Out-of-line member definitions for easyfleet_core::ActionServerBase<ActionT>.
// Included from the bottom of easyfleet_core/action_server_base.hpp. Not meant
// to be included directly: templates cannot be compiled into easyfleet_core's
// .cpp/.so, so the implementation lives here to keep the class declaration
// in action_server_base.hpp free of member bodies.

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "easyfleet_core/detail/namespace_utils.hpp"
#include "easyfleet_core/detail/param_utils.hpp"

namespace easyfleet_core
{

template<typename ActionT>
ActionServerBase<ActionT>::ActionServerBase(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock,
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters,
  rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables,
  const std::string & action_name,
  bool default_allow_preemption)
: node_base_(std::move(node_base)),
  node_clock_(std::move(node_clock)),
  node_logging_(std::move(node_logging)),
  node_parameters_(std::move(node_parameters)),
  node_waitables_(std::move(node_waitables)),
  action_name_(action_name),
  param_name_(detail::sanitize_parameter_name(action_name) + ".allow_preemption")
{
  if (!node_base_ || !node_clock_ || !node_logging_ || !node_parameters_ || !node_waitables_) {
    throw std::invalid_argument("ActionServerBase: node must not be null");
  }
  robot_name_ = detail::robot_label(node_base_->get_namespace());

  if (!node_parameters_->has_parameter(param_name_)) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description =
      "Whether the '" + action_name_ +
      "' action server may preempt an active goal when a new one is accepted.";
    node_parameters_->declare_parameter(
      param_name_, rclcpp::ParameterValue(default_allow_preemption), descriptor);
  }
  allow_preemption_.store(node_parameters_->get_parameter(param_name_).as_bool());

  param_cb_handle_ = node_parameters_->add_post_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params) {
      this->on_parameters_set(params);
    });

  server_ = rclcpp_action::create_server<ActionT>(
    node_base_,
    node_clock_,
    node_logging_,
    node_waitables_,
    action_name_,
    [this](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Goal> goal) {
      return this->handle_goal(uuid, goal);
    },
    [this](const GoalHandleSharedPtr goal_handle) {
      return this->handle_cancel(goal_handle);
    },
    [this](const GoalHandleSharedPtr goal_handle) {
      this->handle_accepted(goal_handle);
    });

  worker_ = std::thread(&ActionServerBase::worker_loop, this);
}

template<typename ActionT>
ActionServerBase<ActionT>::~ActionServerBase()
{
  // Last resort: the derived part is already gone here, so a goal still
  // running would be calling into a destroyed object. Derived classes must
  // call stop() from their own destructor.
  stop();
}

template<typename ActionT>
void ActionServerBase<ActionT>::enable_goals()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!shutting_down_.load()) {
    goals_enabled_.store(true);
  }
}

template<typename ActionT>
void ActionServerBase<ActionT>::disable_goals()
{
  GoalHandleSharedPtr queued;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    goals_enabled_.store(false);
    queued = std::move(pending_handle_);
    pending_handle_.reset();
    if (active_handle_) {
      halt_requested_.store(true);
    }
  }
  if (queued) {
    settle_unstarted(queued);
  }

  if (std::this_thread::get_id() == worker_.get_id()) {
    // Called from on_execute(): waiting here would deadlock.
    return;
  }
  std::unique_lock<std::mutex> lock(mutex_);
  idle_cv_.wait(lock, [this] {return active_handle_ == nullptr;});
  halt_requested_.store(false);
}

template<typename ActionT>
bool ActionServerBase<ActionT>::goals_enabled() const noexcept
{
  return goals_enabled_.load();
}

template<typename ActionT>
void ActionServerBase<ActionT>::stop()
{
  disable_goals();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    shutting_down_.store(true);
  }
  cv_.notify_all();
  if (worker_.joinable() && std::this_thread::get_id() != worker_.get_id()) {
    worker_.join();
  }
  if (param_cb_handle_) {
    node_parameters_->remove_post_set_parameters_callback(param_cb_handle_.get());
    param_cb_handle_.reset();
  }
}

template<typename ActionT>
void ActionServerBase<ActionT>::settle_unstarted(const GoalHandleSharedPtr & goal_handle)
{
  if (!goal_handle->is_active()) {
    return;
  }
  if (goal_handle->is_canceling()) {
    goal_handle->canceled(std::make_shared<Result>());
  } else {
    goal_handle->abort(std::make_shared<Result>());
  }
}

template<typename ActionT>
const std::string & ActionServerBase<ActionT>::get_action_name() const noexcept
{
  return action_name_;
}

template<typename ActionT>
bool ActionServerBase<ActionT>::is_active() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return active_handle_ != nullptr;
}

template<typename ActionT>
bool ActionServerBase<ActionT>::is_preemptable() const noexcept
{
  return allow_preemption_.load();
}

template<typename ActionT>
rclcpp_action::CancelResponse ActionServerBase<ActionT>::on_cancel_requested(
  const GoalHandleSharedPtr /*goal_handle*/)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

template<typename ActionT>
void ActionServerBase<ActionT>::on_preempted(const GoalHandleSharedPtr & /*preempted_goal_handle*/)
{
}

template<typename ActionT>
bool ActionServerBase<ActionT>::is_preempt_requested() const noexcept
{
  return preempt_requested_.load();
}

template<typename ActionT>
bool ActionServerBase<ActionT>::is_shutdown_requested() const noexcept
{
  return shutting_down_.load() || halt_requested_.load();
}

template<typename ActionT>
rclcpp::Logger ActionServerBase<ActionT>::logger() const
{
  return node_logging_->get_logger();
}

template<typename ActionT>
rclcpp_action::GoalResponse ActionServerBase<ActionT>::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const Goal> goal)
{
  if (!goals_enabled_.load()) {
    RCLCPP_WARN(
      node_logging_->get_logger(),
      "Action '%s': goal rejected, goals are disabled (capability not active).",
      action_name_.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  rclcpp_action::GoalResponse response;
  try {
    response = this->on_goal_received(uuid, goal);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      node_logging_->get_logger(), "Action '%s': exception thrown from on_goal_received(): %s",
      action_name_.c_str(), e.what());
    return rclcpp_action::GoalResponse::REJECT;
  } catch (...) {
    RCLCPP_ERROR(
      node_logging_->get_logger(), "Action '%s': unknown exception from on_goal_received()",
      action_name_.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (response != rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE) {
    return response;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (!goals_enabled_.load()) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  // A queued goal counts as busy too: two goals arriving back to back must
  // not both be accepted when preemption is off.
  if ((active_handle_ || pending_handle_) && !allow_preemption_.load()) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

template<typename ActionT>
rclcpp_action::CancelResponse ActionServerBase<ActionT>::handle_cancel(
  const GoalHandleSharedPtr goal_handle)
{
  try {
    return this->on_cancel_requested(goal_handle);
  } catch (...) {
    RCLCPP_ERROR(
      node_logging_->get_logger(), "Action '%s': exception thrown from on_cancel_requested()",
      action_name_.c_str());
    return rclcpp_action::CancelResponse::REJECT;
  }
}

template<typename ActionT>
void ActionServerBase<ActionT>::handle_accepted(GoalHandleSharedPtr goal_handle)
{
  GoalHandleSharedPtr bumped;
  GoalHandleSharedPtr preempted_active;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!goals_enabled_.load()) {
      // Accepted just before goals were disabled: never run it.
      bumped = goal_handle;
    }
  }
  if (bumped) {
    settle_unstarted(bumped);
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (pending_handle_) {
      // A goal was already queued to run next but never started: it is
      // superseded by this newer one.
      bumped = pending_handle_;
    }
    pending_handle_ = goal_handle;
    if (active_handle_) {
      preempt_requested_.store(true);
      preempted_active = active_handle_;
    }
  }

  if (bumped) {
    RCLCPP_WARN(
      node_logging_->get_logger(),
      "Action '%s': a queued goal was superseded before it could start executing.",
      action_name_.c_str());
    settle_unstarted(bumped);
  }
  if (preempted_active) {
    this->on_preempted(preempted_active);
  }
  cv_.notify_one();
}

template<typename ActionT>
void ActionServerBase<ActionT>::on_parameters_set(const std::vector<rclcpp::Parameter> & params)
{
  for (const auto & param : params) {
    if (param.get_name() == param_name_ &&
      param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
    {
      allow_preemption_.store(param.as_bool());
    }
  }
}

template<typename ActionT>
void ActionServerBase<ActionT>::worker_loop()
{
  while (true) {
    GoalHandleSharedPtr goal;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      cv_.wait(
        lock, [this] {
          return shutting_down_.load() || pending_handle_ != nullptr;
        });
      if (shutting_down_.load()) {
        auto queued = std::move(pending_handle_);
        pending_handle_.reset();
        lock.unlock();
        if (queued) {
          settle_unstarted(queued);
        }
        return;
      }
      goal = pending_handle_;
      pending_handle_.reset();
      active_handle_ = goal;
      preempt_requested_.store(false);
    }

    if (goal->is_canceling()) {
      // Canceled by the client before it ever got to run.
      goal->canceled(std::make_shared<Result>());
    } else {
      RCLCPP_INFO(
        node_logging_->get_logger(),
        "Capability '%s' on robot '%s': goal execution started.",
        action_name_.c_str(), robot_name_.c_str());

      // Goals are always accepted with ACCEPT_AND_EXECUTE, so rcl_action has
      // already transitioned this goal straight to EXECUTING; calling
      // goal->execute() again here would be an invalid double transition.
      try {
        this->on_execute(goal);
      } catch (const std::exception & e) {
        RCLCPP_ERROR(
          node_logging_->get_logger(),
          "Action '%s': exception thrown from on_execute(): %s",
          action_name_.c_str(), e.what());
      } catch (...) {
        RCLCPP_ERROR(
          node_logging_->get_logger(),
          "Action '%s': unknown exception thrown from on_execute()",
          action_name_.c_str());
      }
      if (goal->is_active()) {
        RCLCPP_WARN(
          node_logging_->get_logger(),
          "Action '%s': on_execute() returned without settling the goal; aborting it.",
          action_name_.c_str());
        settle_unstarted(goal);
      }

      RCLCPP_INFO(
        node_logging_->get_logger(),
        "Capability '%s' on robot '%s': goal execution finished.",
        action_name_.c_str(), robot_name_.c_str());
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      active_handle_.reset();
    }
    idle_cv_.notify_all();
  }
}

}  // namespace easyfleet_core

#endif  // EASYFLEET_CORE__DETAIL__ACTION_SERVER_BASE_IMPL_HPP_
