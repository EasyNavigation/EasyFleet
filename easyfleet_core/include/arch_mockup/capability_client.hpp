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

#ifndef ARCH_MOCKUP__CAPABILITY_CLIENT_HPP_
#define ARCH_MOCKUP__CAPABILITY_CLIENT_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "arch_mockup/action_client.hpp"

namespace arch_mockup
{

/// The simplest possible way to ask a capability to do something.
/**
 * Where `ActionClient<ActionT>` exposes the full action protocol (goal
 * acceptance, per-goal cancellation by id, active goal bookkeeping...),
 * `CapabilityClient<ActionT>` reduces that to the three things a caller
 * of a capability actually cares about: send a request, optionally observe
 * feedback while it runs, and get the final response. It talks to a
 * capability by name, exactly as advertised by `arch_mockup::Capability`
 * (the capability name is the action name).
 *
 * Usage:
 * \code
 * auto nav = arch_mockup::CapabilityClient<nav2_msgs::action::NavigateToPose>::create(
 *   this, "navigation");
 * if (!nav->wait_for_capability()) { ... not available ... }
 *
 * using NavClient = arch_mockup::CapabilityClient<nav2_msgs::action::NavigateToPose>;
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
  using SharedPtr = std::shared_ptr<CapabilityClient<ActionT>>;
  using Goal = typename ActionT::Goal;
  using Feedback = typename ActionT::Feedback;
  using Result = typename ActionT::Result;
  using FeedbackCallback = std::function<void (std::shared_ptr<const Feedback>)>;

  /// Same vocabulary as `ActionClient::GoalOutcome`.
  using Outcome = typename ActionClient<ActionT>::GoalOutcome;

  struct Response
  {
    Outcome outcome{Outcome::ABORTED};
    typename Result::SharedPtr result;
  };

  using ResponseCallback = std::function<void (const Response &)>;

  /// @param node Node on whose behalf the capability is called.
  /// @param capability_name Name of the capability to call (its action name).
  /// @param default_wait_timeout Timeout used by the no-argument overload of
  ///   `wait_for_capability()`.
  static SharedPtr create(
    rclcpp::Node * node,
    const std::string & capability_name,
    std::chrono::milliseconds default_wait_timeout = std::chrono::seconds(5));

  CapabilityClient(const CapabilityClient &) = delete;
  CapabilityClient & operator=(const CapabilityClient &) = delete;

  /// Blocks until the capability is reachable, or the timeout elapses.
  bool wait_for_capability();
  bool wait_for_capability(std::chrono::milliseconds timeout);

  /// Asks the capability to do something. Returns immediately; `on_response`
  /// is called exactly once with the final outcome, and `on_feedback` (if
  /// given) once per feedback message while the request is running.
  void request(
    const Goal & goal,
    ResponseCallback on_response,
    FeedbackCallback on_feedback = nullptr);

  /// Asks the capability to do something and blocks the calling thread
  /// until the response is ready (or `timeout` elapses, if positive). Safe
  /// to call from any thread.
  Response request_and_wait(
    const Goal & goal,
    FeedbackCallback on_feedback = nullptr,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(0));

  /// Asks the capability to stop whatever it is currently doing (e.g. a
  /// navigating robot stops moving, a perception stream stops reporting
  /// detections). Returns immediately; the pending request's `on_response`
  /// (from `request()`) or the return of `request_and_wait()` will resolve
  /// with `Outcome::CANCELED` once the capability has stopped.
  void cancel();

private:
  explicit CapabilityClient(typename ActionClient<ActionT>::SharedPtr action_client);

  typename ActionClient<ActionT>::SharedPtr action_client_;
};

}  // namespace arch_mockup

#include "arch_mockup/detail/capability_client_impl.hpp"

#endif  // ARCH_MOCKUP__CAPABILITY_CLIENT_HPP_
