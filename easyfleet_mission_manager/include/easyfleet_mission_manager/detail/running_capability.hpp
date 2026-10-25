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
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <utility>

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
template<typename OutcomeT>
easyfleet::CapabilityState outcome_to_state(OutcomeT outcome)
{
  switch (outcome) {
    case OutcomeT::SUCCEEDED: return easyfleet::CapabilityState::SUCCEEDED;
    case OutcomeT::ABORTED: return easyfleet::CapabilityState::ABORTED;
    case OutcomeT::CANCELED: return easyfleet::CapabilityState::CANCELED;
    case OutcomeT::REJECTED: return easyfleet::CapabilityState::REJECTED;
    case OutcomeT::TIMEOUT: return easyfleet::CapabilityState::TIMEOUT;
    case OutcomeT::SERVER_UNAVAILABLE: return easyfleet::CapabilityState::UNREACHABLE;
  }
  return easyfleet::CapabilityState::UNREACHABLE;
}

/// How long a canceled goal may take to report back before it is given up
/// on and settled locally as TIMEOUT.
constexpr std::chrono::seconds kCancelGrace(5);

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
  /// @param alive Whether the capability's heartbeat is still being seen;
  ///   a running goal on a dead capability is settled as UNREACHABLE.
  virtual void check_timeout(bool alive) = 0;
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
  : robot_name_(std::move(robot_name)),
    capability_type_(std::move(capability_type)),
    status_(status),
    client_(Client::create(node, action_name))
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
    uint64_t generation;
    {
      std::lock_guard<std::mutex> lock(settle_mutex_);
      generation = ++generation_;
      state_.store(easyfleet::CapabilityState::RUNNING);
    }
    deadline_ = std::chrono::steady_clock::now() + timeout;
    cancel_deadline_.reset();
    status_.set_status(robot_name_, "Running " + capability_type_);

    client_->request(
      goal,
      [this, generation](const typename Client::Response & response) {
        settle(generation, outcome_to_state(response.outcome));
      });
  }

  void stop() override
  {
    if (state_.load() != easyfleet::CapabilityState::RUNNING) {
      return;
    }
    request_cancel();
  }

  easyfleet::CapabilityState state() const override
  {
    return state_.load();
  }

  void check_timeout(bool alive) override
  {
    if (state_.load() != easyfleet::CapabilityState::RUNNING) {
      return;
    }
    const auto now = std::chrono::steady_clock::now();
    if (!alive) {
      // No heartbeat: the result may never come. Give up on this goal.
      settle(std::nullopt, easyfleet::CapabilityState::UNREACHABLE);
    } else if (cancel_deadline_ && now >= *cancel_deadline_) {
      // Canceled, but the capability never reported back.
      settle(std::nullopt, easyfleet::CapabilityState::TIMEOUT);
    } else if (!cancel_deadline_ && now >= deadline_) {
      request_cancel();
    }
  }

private:
  /// Cancels the in-flight goal once; check_timeout() settles it as TIMEOUT
  /// if no result arrives within kCancelGrace.
  void request_cancel()
  {
    if (cancel_deadline_) {
      return;
    }
    cancel_deadline_ = std::chrono::steady_clock::now() + kCancelGrace;
    client_->cancel();
  }

  /// Records the final state of the current goal. With a `generation`, only
  /// if that goal is still the current one (a result from the network);
  /// without one, unconditionally, and later results are ignored (a local
  /// decision to give up on the goal).
  void settle(std::optional<uint64_t> generation, easyfleet::CapabilityState new_state)
  {
    {
      std::lock_guard<std::mutex> lock(settle_mutex_);
      if (generation) {
        if (*generation != generation_) {
          return;
        }
      } else {
        ++generation_;
      }
      state_.store(new_state);
    }
    status_.set_status(robot_name_, capability_type_ + " -> " + easyfleet::to_string(new_state));
    std::ostringstream out;
    out << "  " << easyfleet_mission_manager::ansi::dim << "[" << robot_name_ << "/" <<
      capability_type_ << "] " << easyfleet_mission_manager::ansi::reset <<
      "finished with outcome " << easyfleet_mission_manager::ansi::magenta <<
      easyfleet::to_string(new_state) << easyfleet_mission_manager::ansi::reset;
    easyfleet_mission_manager::safe_print(out.str());
  }

  std::string robot_name_;
  std::string capability_type_;
  easyfleet_mission_manager::StatusMarkerPublisher & status_;
  std::atomic<easyfleet::CapabilityState> state_{easyfleet::CapabilityState::IDLE};
  std::chrono::steady_clock::time_point deadline_;
  /// Set once the in-flight goal has been asked to cancel.
  std::optional<std::chrono::steady_clock::time_point> cancel_deadline_;
  /// Incremented on every run() call and on every local settle; see run()'s
  /// own comment for why. Guarded, with state_ updates, by settle_mutex_.
  std::mutex settle_mutex_;
  uint64_t generation_{0};
  /// Declared last so it is destroyed first: its destructor joins the thread
  /// that runs the result callback above, which uses every other member.
  typename Client::SharedPtr client_;
};

}  // namespace easyfleet_mission_manager::detail

#endif  // EASYFLEET_MISSION_MANAGER__DETAIL__RUNNING_CAPABILITY_HPP_
