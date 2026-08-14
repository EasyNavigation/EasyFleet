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

/// @brief Comfortable wrapper around `rclcpp_action::Client<ActionT>`.
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
  /// @brief Shared pointer to an `ActionClient<ActionT>`.
  using SharedPtr = std::shared_ptr<ActionClient<ActionT>>;
  /// @brief Goal type of the wrapped action.
  using Goal = typename ActionT::Goal;
  /// @brief Feedback type of the wrapped action.
  using Feedback = typename ActionT::Feedback;
  /// @brief Result type of the wrapped action.
  using Result = typename ActionT::Result;
  /// @brief `rclcpp_action` goal handle type of the wrapped action.
  using ClientGoalHandle = typename rclcpp_action::Client<ActionT>::GoalHandle;

  /// @brief Callback invoked with feedback as it arrives for an in-flight goal.
  using FeedbackCallback = std::function<void (std::shared_ptr<const Feedback>)>;

  /// @brief Outcome of a goal, unifying rejection/timeout/unavailability with the
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

  /// @brief Terminal outcome of a goal, as delivered to a `ResultCallback` or
  /// returned by `send_goal_and_wait()`.
  struct GoalResult
  {
    /// @brief How the goal ended.
    GoalOutcome outcome{GoalOutcome::ABORTED};
    /// @brief Id of the goal this result belongs to.
    rclcpp_action::GoalUUID goal_id{};
    /// @brief Action-specific result, if `outcome` is `GoalOutcome::SUCCEEDED`.
    typename Result::SharedPtr result;
  };

  /// @brief Callback invoked once a goal reaches a terminal state.
  using ResultCallback = std::function<void (const GoalResult &)>;
  /// @brief Callback invoked once the server has accepted or rejected a goal.
  using GoalResponseCallback =
    std::function<void (bool accepted, const rclcpp_action::GoalUUID & goal_id)>;

  /// @brief Constructs a new client for `action_name`, spinning its own internal node in the background.
  /// @param parent_node Node on whose behalf the action is called; only used
  ///   to inherit its namespace and to derive a name for the internal node
  ///   (never stored), so a reference -- never null, unlike a pointer -- is
  ///   all this needs.
  /// @param action_name Name of the action to call.
  /// @param default_server_timeout Timeout used by the no-argument overload
  ///   of `wait_for_server()`.
  /// @return A new `ActionClient<ActionT>`, already spinning its internal
  ///   node in the background.
  static SharedPtr create(
    rclcpp::Node & parent_node,
    const std::string & action_name,
    std::chrono::milliseconds default_server_timeout = std::chrono::seconds(5));

  ActionClient(const ActionClient &) = delete;
  ActionClient & operator=(const ActionClient &) = delete;

  ~ActionClient();

  /// @brief The action name this client was created for.
  /// @return The action name this client was created for.
  const std::string & get_action_name() const noexcept;

  /// @brief Whether the action server is currently reachable.
  /// @return Whether the action server is currently reachable.
  bool is_server_ready() const;

  /// @brief Block until the server is reachable, using the constructor's
  /// `default_server_timeout`.
  /// @return Whether the server became reachable before the timeout.
  bool wait_for_server();
  /// @brief Block until the server is reachable or `timeout` elapses.
  /// @param timeout Maximum time to wait.
  /// @return Whether the server became reachable before the timeout.
  bool wait_for_server(std::chrono::milliseconds timeout);

  /// @brief Send a goal without blocking. Returns `false` immediately (without
  /// contacting the server) if it is not currently reachable; in that case,
  /// `result_callback`, if any, is invoked synchronously with
  /// `GoalOutcome::SERVER_UNAVAILABLE`.
  /// @param goal Goal to send.
  /// @param result_callback Invoked once the goal reaches a terminal state.
  /// @param feedback_callback Invoked as feedback arrives for this goal.
  /// @param goal_response_callback Invoked once the server accepts/rejects
  ///   the goal.
  /// @return Whether the goal was sent (not whether it was accepted).
  bool send_goal(
    const Goal & goal,
    ResultCallback result_callback = nullptr,
    FeedbackCallback feedback_callback = nullptr,
    GoalResponseCallback goal_response_callback = nullptr);

  /// @brief Send a goal and block the calling thread until it reaches a terminal
  /// state (or `timeout` elapses, if positive). Safe to call from any
  /// thread, including one that also spins the owning node: this object
  /// spins its own dedicated internal node to drive the exchange.
  /// @param goal Goal to send.
  /// @param feedback_callback Invoked as feedback arrives for this goal.
  /// @param timeout Maximum time to wait for a terminal state; `0` (the
  ///   default) waits indefinitely.
  /// @return The goal's terminal outcome (`GoalOutcome::TIMEOUT` if
  ///   `timeout` elapsed first).
  GoalResult send_goal_and_wait(
    const Goal & goal,
    FeedbackCallback feedback_callback = nullptr,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(0));

  /// @brief Request cancellation of a specific in-flight goal. Returns `false` if
  /// the goal is unknown (already terminal, or never accepted).
  /// @param goal_id Id of the goal to cancel.
  /// @return Whether a cancellation request was sent.
  bool cancel_goal(const rclcpp_action::GoalUUID & goal_id);

  /// @brief Request cancellation of every goal currently tracked by this client.
  void cancel_all_goals();

  /// @brief Number of goals accepted by the server and not yet in a terminal state.
  /// @return Number of goals accepted by the server and not yet in a
  ///   terminal state.
  std::size_t active_goal_count() const;

private:
  ActionClient(
    rclcpp::Node & parent_node,
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
