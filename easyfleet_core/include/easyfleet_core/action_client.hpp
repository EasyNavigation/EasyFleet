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

#ifndef EASYFLEET_CORE__ACTION_CLIENT_HPP_
#define EASYFLEET_CORE__ACTION_CLIENT_HPP_

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace easyfleet_core
{

/// Comfortable wrapper around `rclcpp_action::Client<ActionT>`.
/**
 * Each `ActionClient<ActionT>` owns a small internal node dedicated to a
 * single action and spins it on a background thread for its entire
 * lifetime. This means callers never have to worry about whether their own
 * node's executor is spinning: goals can be sent, results awaited (even
 * synchronously, via `send_goal_and_wait()`), and feedback/cancel handled,
 * from any thread, independently of how the owning node is executed.
 *
 * A node typically owns one `ActionClient<ActionT>::SharedPtr` per action it
 * talks to, created with `create()`.
 */
template<typename ActionT>
class ActionClient
{
public:
  using SharedPtr = std::shared_ptr<ActionClient<ActionT>>;
  using Goal = typename ActionT::Goal;
  using Feedback = typename ActionT::Feedback;
  using Result = typename ActionT::Result;
  using ClientGoalHandle = typename rclcpp_action::Client<ActionT>::GoalHandle;

  using FeedbackCallback = std::function<void (std::shared_ptr<const Feedback>)>;

  /// Outcome of a goal, unifying rejection/timeout/unavailability with the
  /// normal terminal states of the action state machine.
  enum class GoalOutcome : uint8_t
  {
    SUCCEEDED,
    ABORTED,
    CANCELED,
    REJECTED,
    TIMEOUT,
    SERVER_UNAVAILABLE,
  };

  struct GoalResult
  {
    GoalOutcome outcome{GoalOutcome::ABORTED};
    rclcpp_action::GoalUUID goal_id{};
    typename Result::SharedPtr result;
  };

  using ResultCallback = std::function<void (const GoalResult &)>;
  using GoalResponseCallback =
    std::function<void (bool accepted, const rclcpp_action::GoalUUID & goal_id)>;

  /// @param parent_node Node on whose behalf the action is called; only used
  ///   to inherit its namespace and to derive a name for the internal node.
  /// @param action_name Name of the action to call.
  /// @param default_server_timeout Timeout used by the no-argument overload
  ///   of `wait_for_server()`.
  static SharedPtr create(
    rclcpp::Node * parent_node,
    const std::string & action_name,
    std::chrono::milliseconds default_server_timeout = std::chrono::seconds(5));

  ActionClient(const ActionClient &) = delete;
  ActionClient & operator=(const ActionClient &) = delete;

  ~ActionClient();

  const std::string & get_action_name() const noexcept;

  bool is_server_ready() const;

  bool wait_for_server();
  bool wait_for_server(std::chrono::milliseconds timeout);

  /// Send a goal without blocking. Returns `false` immediately (without
  /// contacting the server) if it is not currently reachable; in that case,
  /// `result_callback`, if any, is invoked synchronously with
  /// `GoalOutcome::SERVER_UNAVAILABLE`.
  bool send_goal(
    const Goal & goal,
    ResultCallback result_callback = nullptr,
    FeedbackCallback feedback_callback = nullptr,
    GoalResponseCallback goal_response_callback = nullptr);

  /// Send a goal and block the calling thread until it reaches a terminal
  /// state (or `timeout` elapses, if positive). Safe to call from any
  /// thread, including one that also spins the owning node: this object
  /// spins its own dedicated internal node to drive the exchange.
  GoalResult send_goal_and_wait(
    const Goal & goal,
    FeedbackCallback feedback_callback = nullptr,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(0));

  /// Request cancellation of a specific in-flight goal. Returns `false` if
  /// the goal is unknown (already terminal, or never accepted).
  bool cancel_goal(const rclcpp_action::GoalUUID & goal_id);

  /// Request cancellation of every goal currently tracked by this client.
  void cancel_all_goals();

  /// Number of goals accepted by the server and not yet in a terminal state.
  std::size_t active_goal_count() const;

private:
  ActionClient(
    rclcpp::Node * parent_node,
    const std::string & action_name,
    std::chrono::milliseconds default_server_timeout);

  static GoalResult to_goal_result(const typename ClientGoalHandle::WrappedResult & wrapped);

  std::string action_name_;
  std::chrono::milliseconds default_server_timeout_;

  rclcpp::Node::SharedPtr internal_node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread spin_thread_;
  typename rclcpp_action::Client<ActionT>::SharedPtr client_;

  mutable std::mutex goals_mutex_;
  std::unordered_map<std::string, typename ClientGoalHandle::SharedPtr> active_goals_;
};

}  // namespace easyfleet_core

#include "easyfleet_core/detail/action_client_impl.hpp"

#endif  // EASYFLEET_CORE__ACTION_CLIENT_HPP_
