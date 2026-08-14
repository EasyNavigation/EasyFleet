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

#ifndef EASYFLEET_MISSION_MANAGER__DETAIL__RUNNING_CAPABILITY_HPP_
#define EASYFLEET_MISSION_MANAGER__DETAIL__RUNNING_CAPABILITY_HPP_

#include <atomic>
#include <chrono>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "easyfleet_core/capability_client.hpp"

#include "easyfleet_mission_manager/ansi.hpp"
#include "easyfleet_mission_manager/capability_state.hpp"
#include "easyfleet_mission_manager/output.hpp"
#include "easyfleet_mission_manager/status_markers.hpp"

// RobotHandle::run_capability<ActionT>() already knows ActionT at the call
// site (it's the template parameter), so RobotHandle only needs a
// *non-template* handle to hold in its single per-capability-type map for
// the ActionT-independent parts (current state, stop(), timeout
// enforcement) -- RunningCapabilityBase. run() itself, which needs
// ActionT::Goal, is not part of that virtual interface: RobotHandle
// dynamic_casts down to the concrete RunningCapability<ActionT> (verifying
// this capability_type hasn't previously been run() with a *different*
// ActionT -- see robot_handle_impl.hpp) and calls it directly.
namespace easyfleet_mission_manager::detail
{

/// Maps a CapabilityClient<ActionT>::Outcome onto the matching
/// easyfleet::CapabilityState one to one -- the real, specific outcome is
/// what a RobotHandle reports, never collapsed into a generic
/// success/failure bit.
inline easyfleet::CapabilityState outcome_to_state(uint8_t outcome_value)
{
  // Mirrors easyfleet_core::ActionClient<ActionT>::GoalOutcome's values
  // (CapabilityClient::Outcome is a direct alias of it) member for member.
  switch (outcome_value) {
    case 0: return easyfleet::CapabilityState::SUCCEEDED;
    case 1: return easyfleet::CapabilityState::ABORTED;
    case 2: return easyfleet::CapabilityState::CANCELED;
    case 3: return easyfleet::CapabilityState::REJECTED;
    case 4: return easyfleet::CapabilityState::TIMEOUT;
    default: return easyfleet::CapabilityState::UNREACHABLE;  // SERVER_UNAVAILABLE
  }
}

/// Non-template interface RobotHandle holds one of per capability type it
/// has ever run, so it doesn't need to know ActionT itself for anything
/// but sending a fresh goal (see run(), on the concrete subclass only).
class RunningCapabilityBase
{
public:
  virtual ~RunningCapabilityBase() = default;

  /// Asks the in-flight goal, if any, to stop.
  virtual void stop() = 0;

  /// @return Current state, per the last run()/its outcome.
  virtual easyfleet::CapabilityState state() const = 0;

  /// Called periodically (from FleetSession::spin_some()) so a `timeout`
  /// passed to run() actually gets enforced without blocking the caller.
  virtual void check_timeout() = 0;
};

/// The one concrete RunningCapabilityBase implementation, parameterized by
/// the actual action type a given capability type is run with.
template<typename ActionT>
class RunningCapability : public RunningCapabilityBase
{
public:
  /// The CapabilityClient<ActionT> specialization this instance wraps.
  using Client = easyfleet_core::CapabilityClient<ActionT>;

  /// @param node Node the underlying CapabilityClient is created on.
  /// @param action_name Resolved action name to call (e.g. "/robot_1/navigation").
  /// @param robot_name Robot identity the status marker is shown above.
  /// @param capability_type Capability type, for status/log lines (e.g. "navigation").
  /// @param status Status marker publisher run() publishes through.
  RunningCapability(
    rclcpp::Node & node,
    const std::string & action_name,
    std::string robot_name,
    std::string capability_type,
    easyfleet_mission_manager::StatusMarkerPublisher & status)
  : client_(Client::create(node, action_name)),
    robot_name_(std::move(robot_name)),
    capability_type_(std::move(capability_type)),
    status_(status)
  {
  }

  /// Sends `goal` and starts tracking it.
  /// @param goal Goal to send, fully typed -- built by hand or via one of
  ///   mission_helpers.hpp's make_*_goal() convenience overloads.
  /// @param timeout Stop the goal if it hasn't finished on its own within
  ///   this long, enforced by check_timeout().
  void run(const typename ActionT::Goal & goal, std::chrono::seconds timeout)
  {
    // A new run() call means a new goal supersedes whatever was in flight
    // (server-side preemption -- see RobotHandle::run_capability()'s own
    // doc comment). The *client*-side outcome of that superseded goal can
    // still arrive after this point, though: its own request() callback
    // below is still registered and will fire (typically with an
    // ABORTED-flavored outcome) once the server settles it. Without this
    // generation guard, that late callback would stomp state_ with the
    // *old* goal's outcome after the new goal has already started (or
    // even after it has already finished) -- exactly the race that
    // produced spurious FAILED results for a robot whose goal was
    // preempted mid-mission. Only the request() call that is still the
    // *current* one when its callback fires is allowed to update state_.
    const auto generation = generation_.fetch_add(1) + 1;
    state_.store(easyfleet::CapabilityState::RUNNING);
    deadline_ = std::chrono::steady_clock::now() + timeout;
    status_.set_status(robot_name_, "Running " + capability_type_);

    client_->request(
      goal,
      [this, generation](const typename Client::Response & response) {
        if (generation_.load() != generation) {
          return;
        }
        const auto new_state = outcome_to_state(static_cast<uint8_t>(response.outcome));
        state_.store(new_state);
        status_.set_status(
          robot_name_,
          capability_type_ + " -> " + easyfleet::to_string(new_state));
        std::ostringstream out;
        out << "  " << easyfleet_mission_manager::ansi::dim << "[" << robot_name_ << "/" <<
          capability_type_ << "] " << easyfleet_mission_manager::ansi::reset <<
          "finished with outcome " << easyfleet_mission_manager::ansi::magenta <<
          easyfleet::to_string(new_state) << easyfleet_mission_manager::ansi::reset;
        easyfleet_mission_manager::safe_print(out.str());
      });
  }

  void stop() override
  {
    client_->cancel();
  }

  easyfleet::CapabilityState state() const override
  {
    return state_.load();
  }

  void check_timeout() override
  {
    if (state_.load() == easyfleet::CapabilityState::RUNNING &&
      std::chrono::steady_clock::now() >= deadline_)
    {
      client_->cancel();
    }
  }

private:
  typename Client::SharedPtr client_;
  std::string robot_name_;
  std::string capability_type_;
  easyfleet_mission_manager::StatusMarkerPublisher & status_;
  std::atomic<easyfleet::CapabilityState> state_{easyfleet::CapabilityState::IDLE};
  std::chrono::steady_clock::time_point deadline_;
  /// Incremented on every run() call; see run()'s own comment for why.
  std::atomic<uint64_t> generation_{0};
};

}  // namespace easyfleet_mission_manager::detail

#endif  // EASYFLEET_MISSION_MANAGER__DETAIL__RUNNING_CAPABILITY_HPP_
