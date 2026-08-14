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

#ifndef EASYFLEET_CORE__CAPABILITY_CLIENT_HPP_
#define EASYFLEET_CORE__CAPABILITY_CLIENT_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "easyfleet_core/action_client.hpp"

namespace easyfleet_core
{

/// @brief The simplest possible way to ask a capability to do something.
/**
 * Where `ActionClient<ActionT>` exposes the full action protocol (goal
 * acceptance, per-goal cancellation by id, active goal bookkeeping...),
 * `CapabilityClient<ActionT>` reduces that to the three things a caller
 * of a capability actually cares about: send a request, optionally observe
 * feedback while it runs, and get the final response. It talks to a
 * capability by name, exactly as advertised by `easyfleet_core::Capability`
 * (the capability name is the action name).
 *
 * Usage:
 * \code
 * auto nav = easyfleet_core::CapabilityClient<nav2_msgs::action::NavigateToPose>::create(
 *   this, "navigation");
 * if (!nav->wait_for_capability()) { ... not available ... }
 *
 * using NavClient = easyfleet_core::CapabilityClient<nav2_msgs::action::NavigateToPose>;
 * nav2_msgs::action::NavigateToPose::Goal goal;
 * goal.pose = ...;
 * auto response = nav->request_and_wait(goal);
 * if (response.outcome == NavClient::Outcome::SUCCEEDED) { ... }
 * \endcode
 */
template<typename ActionT>
class CapabilityClient
{
public:
  /// @brief Shared pointer to a `CapabilityClient<ActionT>`.
  using SharedPtr = std::shared_ptr<CapabilityClient<ActionT>>;
  /// @brief Goal type of the wrapped action.
  using Goal = typename ActionT::Goal;
  /// @brief Feedback type of the wrapped action.
  using Feedback = typename ActionT::Feedback;
  /// @brief Result type of the wrapped action.
  using Result = typename ActionT::Result;
  /// @brief Callback invoked with feedback as it arrives for the request.
  using FeedbackCallback = std::function<void (std::shared_ptr<const Feedback>)>;

  /// @brief Same vocabulary as `ActionClient::GoalOutcome`.
  using Outcome = typename ActionClient<ActionT>::GoalOutcome;

  /// @brief Final outcome of a request, as delivered to a `ResponseCallback` or
  /// returned by `request_and_wait()`.
  struct Response
  {
    /// @brief How the request ended.
    Outcome outcome{Outcome::ABORTED};
    /// @brief Action-specific result, if `outcome` is `Outcome::SUCCEEDED`.
    typename Result::SharedPtr result;
  };

  /// @brief Callback invoked once a request reaches a final outcome.
  using ResponseCallback = std::function<void (const Response &)>;

  /// @brief Constructs a new client for the capability named `capability_name`.
  /// @param node Node on whose behalf the capability is called. Only used
  ///   to construct the underlying `ActionClient` (never stored), so a
  ///   reference -- never null, unlike a pointer -- is all this needs.
  /// @param capability_name Name of the capability to call (its action name).
  /// @param default_wait_timeout Timeout used by the no-argument overload of
  ///   `wait_for_capability()`.
  /// @return A new `CapabilityClient<ActionT>`.
  static SharedPtr create(
    rclcpp::Node & node,
    const std::string & capability_name,
    std::chrono::milliseconds default_wait_timeout = std::chrono::seconds(5));

  CapabilityClient(const CapabilityClient &) = delete;
  CapabilityClient & operator=(const CapabilityClient &) = delete;

  /// @brief Blocks until the capability is reachable, using the constructor's
  /// `default_wait_timeout`.
  /// @return Whether the capability became reachable before the timeout.
  bool wait_for_capability();
  /// @brief Blocks until the capability is reachable, or `timeout` elapses.
  /// @param timeout Maximum time to wait.
  /// @return Whether the capability became reachable before the timeout.
  bool wait_for_capability(std::chrono::milliseconds timeout);

  /// @brief Asks the capability to do something. Returns immediately; `on_response`
  /// is called exactly once with the final outcome, and `on_feedback` (if
  /// given) once per feedback message while the request is running.
  /// @param goal Request content to send.
  /// @param on_response Invoked once with the final outcome.
  /// @param on_feedback Invoked as feedback arrives, if given.
  void request(
    const Goal & goal,
    ResponseCallback on_response,
    FeedbackCallback on_feedback = nullptr);

  /// @brief Asks the capability to do something and blocks the calling thread
  /// until the response is ready (or `timeout` elapses, if positive). Safe
  /// to call from any thread.
  /// @param goal Request content to send.
  /// @param on_feedback Invoked as feedback arrives, if given.
  /// @param timeout Maximum time to wait for a response; `0` (the default)
  ///   waits indefinitely.
  /// @return The request's final outcome (`Outcome::TIMEOUT` if `timeout`
  ///   elapsed first).
  Response request_and_wait(
    const Goal & goal,
    FeedbackCallback on_feedback = nullptr,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(0));

  /// @brief Asks the capability to stop whatever it is currently doing (e.g. a
  /// navigating robot stops moving, a perception stream stops reporting
  /// detections). Returns immediately; the pending request's `on_response`
  /// (from `request()`) or the return of `request_and_wait()` will resolve
  /// with `Outcome::CANCELED` once the capability has stopped.
  void cancel();

private:
  explicit CapabilityClient(typename ActionClient<ActionT>::SharedPtr action_client);

  typename ActionClient<ActionT>::SharedPtr action_client_;
};

}  // namespace easyfleet_core

#include "easyfleet_core/detail/capability_client_impl.hpp"

#endif  // EASYFLEET_CORE__CAPABILITY_CLIENT_HPP_
