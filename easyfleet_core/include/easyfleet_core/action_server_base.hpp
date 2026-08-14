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

#ifndef EASYFLEET_CORE__ACTION_SERVER_BASE_HPP_
#define EASYFLEET_CORE__ACTION_SERVER_BASE_HPP_

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace easyfleet_core
{

/// Base class that hides the boilerplate of running a ROS 2 action server.
/**
 * A node exposes an action by inheriting from `ActionServerBase<ActionT>` and
 * implementing three hooks: `on_goal_received()`, `on_execute()` and,
 * optionally, `on_cancel_requested()`. Everything else -- wiring the
 * `rclcpp_action::Server`, running goals on a dedicated worker thread,
 * transitioning the goal state machine, and deciding whether an incoming
 * goal is allowed to preempt the one currently running -- is handled here.
 *
 * Preemption is governed by a boolean ROS 2 parameter named
 * `"<action_name>.allow_preemption"` (with '/' in the action name mapped to
 * '.'), so it can be set at startup and changed at runtime. Only one goal is
 * ever executed at a time:
 *  - If preemption is disallowed and a goal is active, new goals are
 *    rejected outright.
 *  - If preemption is allowed, a new goal is accepted immediately and
 *    `is_preempt_requested()` starts returning `true` for the running goal.
 *    A long-running `on_execute()` implementation is expected to poll it
 *    (and `is_shutdown_requested()`) and return promptly, having settled the
 *    goal handle (succeeded/aborted/canceled).
 *
 * This class is not copyable and does not itself derive from `rclcpp::Node`,
 * so a node can inherit from several `ActionServerBase<ActionT>` (one per
 * action) without any diamond-inheritance issues.
 *
 * It attaches to any node type exposing the standard `rclcpp` node
 * interfaces (`get_node_base_interface()`, `get_node_clock_interface()`,
 * `get_node_logging_interface()`, `get_node_parameters_interface()`,
 * `get_node_waitables_interface()`), which covers both `rclcpp::Node` and
 * `rclcpp_lifecycle::LifecycleNode`.
 */
template<typename ActionT>
class ActionServerBase
{
public:
  using ActionType = ActionT;
  using Goal = typename ActionT::Goal;
  using Feedback = typename ActionT::Feedback;
  using Result = typename ActionT::Result;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;
  using GoalHandleSharedPtr = std::shared_ptr<GoalHandle>;
  using SharedPtr = std::shared_ptr<ActionServerBase<ActionT>>;

  ActionServerBase(const ActionServerBase &) = delete;
  ActionServerBase & operator=(const ActionServerBase &) = delete;

  virtual ~ActionServerBase();

  /// Fully-qualified name of the action served by this instance.
  const std::string & get_action_name() const noexcept;

  /// Whether a goal is currently executing.
  bool is_active() const;

  /// Current value of the "allow_preemption" parameter for this action.
  bool is_preemptable() const noexcept;

protected:
  /// @param node Node (or lifecycle node) that will host the action server.
  ///   Only used here, to extract its interfaces below (each kept as its
  ///   own `SharedPtr` member) -- not stored itself, so a reference (never
  ///   null, unlike a pointer) is all this needs; must outlive this object.
  /// @param action_name Name under which the action is advertised.
  /// @param default_allow_preemption Initial value of the preemption parameter,
  ///   used only if the parameter has not already been declared/set (e.g. from
  ///   a YAML params file or the command line).
  template<typename NodeT>
  explicit ActionServerBase(
    NodeT & node,
    const std::string & action_name,
    bool default_allow_preemption = true)
  : ActionServerBase(
      node.get_node_base_interface(),
      node.get_node_clock_interface(),
      node.get_node_logging_interface(),
      node.get_node_parameters_interface(),
      node.get_node_waitables_interface(),
      action_name,
      default_allow_preemption)
  {
  }

  /// Validate an incoming goal. Return `ACCEPT_AND_EXECUTE` or `REJECT`.
  /// Preemption bookkeeping is applied automatically after this returns
  /// `ACCEPT_AND_EXECUTE`, so implementations only need to worry about
  /// whether the *content* of the goal is valid.
  virtual rclcpp_action::GoalResponse on_goal_received(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Goal> goal) = 0;

  /// Run an accepted goal to completion. Must leave the goal handle in a
  /// terminal state (`succeed()`, `abort()` or `canceled()`) before
  /// returning. Long-running implementations should periodically check
  /// `is_preempt_requested()`, `goal_handle->is_canceling()` and
  /// `is_shutdown_requested()` and return promptly when any of them is true.
  /// Only settle the goal with `canceled()` when `goal_handle->is_canceling()`
  /// is true (a client asked for it, so rcl_action has already transitioned
  /// the goal to CANCELING); for preemption or shutdown, which happen while
  /// the goal is still EXECUTING, use `abort()` instead -- `canceled()` from
  /// EXECUTING is an invalid state transition.
  virtual void on_execute(const GoalHandleSharedPtr goal_handle) = 0;

  /// Decide whether a cancel request (via the action's cancel service) should
  /// be accepted. Unrelated to preemption. Defaults to always accepting.
  virtual rclcpp_action::CancelResponse on_cancel_requested(const GoalHandleSharedPtr goal_handle);

  /// Optional hook invoked (from the accepting thread, not the worker thread)
  /// when a new goal is about to preempt the currently running one.
  virtual void on_preempted(const GoalHandleSharedPtr & preempted_goal_handle);

  /// True once a newer goal has been accepted and is waiting to replace the
  /// one currently executing. Only meaningful from within `on_execute()`.
  bool is_preempt_requested() const noexcept;

  /// True once this object is being destroyed. Long-running `on_execute()`
  /// implementations should treat this the same as a preemption request.
  bool is_shutdown_requested() const noexcept;

  /// Logger of the node hosting this action server. Named without the usual
  /// `get_` prefix to avoid an ambiguous lookup in classes that also inherit
  /// from `rclcpp::Node` or `rclcpp_lifecycle::LifecycleNode`, both of which
  /// already declare a `get_logger()` of their own.
  rclcpp::Logger logger() const;

private:
  ActionServerBase(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters,
    rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables,
    const std::string & action_name,
    bool default_allow_preemption);

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const GoalHandleSharedPtr goal_handle);
  void handle_accepted(GoalHandleSharedPtr goal_handle);
  void on_parameters_set(const std::vector<rclcpp::Parameter> & params);
  void worker_loop();

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_;
  rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_;

  std::string action_name_;
  std::string robot_name_;
  std::string param_name_;
  std::atomic_bool allow_preemption_{true};

  typename rclcpp_action::Server<ActionT>::SharedPtr server_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::atomic_bool shutting_down_{false};
  std::atomic_bool preempt_requested_{false};
  GoalHandleSharedPtr active_handle_;
  GoalHandleSharedPtr pending_handle_;
  std::thread worker_;
};

}  // namespace easyfleet_core

#include "easyfleet_core/detail/action_server_base_impl.hpp"

#endif  // EASYFLEET_CORE__ACTION_SERVER_BASE_HPP_
