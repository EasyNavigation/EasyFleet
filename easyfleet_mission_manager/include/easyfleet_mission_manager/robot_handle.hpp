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

#ifndef EASYFLEET_MISSION_MANAGER__ROBOT_HANDLE_HPP_
#define EASYFLEET_MISSION_MANAGER__ROBOT_HANDLE_HPP_

#include <chrono>
#include <string>
#include <vector>

#include "easyfleet_mission_manager/capability_info.hpp"
#include "easyfleet_mission_manager/capability_state.hpp"
#include "easyfleet_mission_manager/mission_helpers.hpp"

namespace easyfleet
{

class FleetSession;

/// A remote robot's capabilities, as seen and commanded from a mission
/// script.
/**
 * The client-side counterpart to `easyfleet_core::Robot` (which *hosts*
 * real capability nodes) -- deliberately a different type, since a
 * `RobotHandle` owns no nodes at all: it's a thin proxy over whatever this
 * robot announced on `/capabilities`, discovered by the `FleetSession`
 * it's added to (directly, or through whichever `Controller`-flavored
 * class owns that session -- see `FleetSession`'s own doc comment for why
 * `RobotHandle` depends on the shared session, not on any one controller
 * type: it's meant to work the same under `SimpleController`, an
 * `LLMController`, a `PlanSys2Controller`, or any other controller a user
 * writes).
 *
 * Everything here is non-blocking and observable, replacing the pattern
 * every current mission script hand-rolls: raw `std::thread` objects for
 * parallel robots, manual `CapabilityClient` null-checks at every call
 * site, and manual `StatusMarkerPublisher::set_status()` calls at every
 * transition (easy to forget -- see `refactor.md`). `run_capability()`
 * publishes that status text itself, automatically, for the life of the
 * goal; a mission script using `RobotHandle` never touches
 * `StatusMarkerPublisher` directly.
 *
 * Usage:
 * \code
 * easyfleet::SimpleController controller;
 * easyfleet::RobotHandle robot_1("robot_1");
 * controller.add_robot(robot_1);
 * controller.discover_capabilities();
 *
 * if (!robot_1.has_capability("navigation")) { ... }
 *
 * robot_1.run_capability("navigation", "kitchen");
 * while (robot_1.is_capability_running("navigation")) {
 *   controller.spin_some();
 * }
 * if (robot_1.capability_state("navigation") == easyfleet::CapabilityState::FAILED) { ... }
 * \endcode
 */
class RobotHandle
{
public:
  explicit RobotHandle(std::string name);

  const std::string & name() const noexcept;

  /// True if this robot announced an *active* capability of this type
  /// (per the `SimpleController`'s last `discover_capabilities()` pass).
  bool has_capability(const std::string & capability_type) const;

  /// Sends a goal to `capability_type` and returns immediately -- the
  /// counterpart to today's blocking `easyfleet_mission_manager::run_capability<ActionT>()`.
  /// Also publishes an automatic status marker above this robot
  /// ("Navigating -> kitchen", etc.), kept up to date until the goal
  /// settles or is stopped.
  /**
   * @param target Meaning is capability-specific by convention -- for
   *   "navigation" this is a waypoint id (matching the existing
   *   `parameters_json: {"goal_id": ...}` convention the real
   *   EasyNav-backed navigation capability already uses). What it should
   *   mean for other capability types (manipulation, perception, and any
   *   future one) is **not yet decided** -- see the open question in
   *   `refactor.md`. Left as a plain string for now, matching the
   *   sketch this design started from; likely needs to become either a
   *   generic JSON-string escape hatch, typed per-capability overloads
   *   (`navigate_to`, `manipulate`, ...), or both.
   * @param timeout Same semantics as today's `run_capability()`: stop the
   *   goal if it hasn't finished on its own within this long. Does not
   *   block -- check `is_capability_running()`/`capability_state()` (or
   *   `SimpleController::spin_for()`) to observe the outcome.
   */
  void run_capability(
    const std::string & capability_type,
    const std::string & target = "",
    std::chrono::seconds timeout = easyfleet_mission_manager::kRunTimeout);

  /// Asks `capability_type` to stop whatever it's doing, if anything.
  void stop_capability(const std::string & capability_type);

  /// Shorthand for `capability_state(capability_type) == CapabilityState::RUNNING`.
  bool is_capability_running(const std::string & capability_type) const;

  /// Current state of the last `run_capability()` call for this type (or
  /// `IDLE` if none has been made). See `CapabilityState` for what each
  /// value means and how they differ from a plain running/not-running bool.
  CapabilityState capability_state(const std::string & capability_type) const;

  /// Whether a heartbeat has been seen recently on `/capabilities_status`
  /// for this capability -- continuously tracked for the whole mission
  /// (not a one-off discovery-time snapshot the way today's
  /// `CapabilityInfo::active` is), so it reflects the *current* liveness
  /// of the capability's process, including one that dies mid-mission.
  bool is_alive(const std::string & capability_type) const;

  /// Prints every discovered capability of this robot, like today's
  /// `print_capability_info()`/`print_capability_summary_line()` free
  /// functions.
  void print_capabilities() const;

  /// Every capability this robot announced, as discovered -- the full
  /// `CapabilityInfo` (type, action name, description JSON, active/busy
  /// state), not just a yes/no check like `has_capability()`. Meant for
  /// code that needs to *reason about* what a robot can do, not just
  /// command a specific known capability by name -- e.g. a controller
  /// that builds an LLM prompt describing every robot's capabilities, or
  /// generates a PDDL problem instance from them.
  const std::vector<easyfleet_mission_manager::CapabilityInfo> & capabilities() const noexcept;

private:
  friend class FleetSession;

  /// Called by the owning `FleetSession` once, when this handle is added,
  /// and again after every `discover_capabilities()` pass -- supplies the
  /// node/executor context this handle needs to build `CapabilityClient`s
  /// and this robot's slice of the session's one shared discovery scan
  /// (never a redundant per-robot re-scan).
  void attach(FleetSession & session);
  void set_capabilities(std::vector<easyfleet_mission_manager::CapabilityInfo> capabilities);

  std::string name_;
  std::vector<easyfleet_mission_manager::CapabilityInfo> capabilities_;
  // Deliberately a raw pointer, not a reference: unlike every other
  // non-owning back-reference in this codebase, this one is genuinely
  // nullable (a RobotHandle can exist before attach() ever runs -- e.g.
  // right after `RobotHandle robot_1("robot_1");`, before it's added to
  // any FleetSession) *and* set exactly once, later, by attach() -- a
  // reference member could do neither (never null, never reseatable).
  FleetSession * session_{nullptr};
};

}  // namespace easyfleet

#endif  // EASYFLEET_MISSION_MANAGER__ROBOT_HANDLE_HPP_
