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

#ifndef EASYFLEET_MISSION_MANAGER__FLEET_SESSION_HPP_
#define EASYFLEET_MISSION_MANAGER__FLEET_SESSION_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "easyfleet_mission_manager/robot_handle.hpp"
#include "easyfleet_mission_manager/status_markers.hpp"

namespace easyfleet
{

/// Everything talking to a fleet from a mission-control process actually
/// needs, regardless of what decides *when* to command which robot: the
/// ROS node and background-spinning executor every `RobotHandle` rides on,
/// one shared capability discovery pass, the registry of added robots, and
/// the automatic status markers `RobotHandle::run_capability()` publishes.
/**
 * `SimpleController` -- a plain mission script driving robots by hand --
 * is the simplest possible thing built on top of a `FleetSession`, but
 * it's deliberately not the *only* thing that can be: a `FleetSession` has
 * no notion of "how a mission decides what to do next", only "how to talk
 * to the robots once something has decided". That's what makes it the
 * right thing for other controller flavors to build on too -- e.g. an
 * `LLMController` that, from inside its own `spin_some()`, asks a model
 * what to do next and calls `run_capability()` accordingly; or a
 * `PlanSys2Controller` whose PDDL action implementations reach into the
 * fleet through `robots()`/`find_robot()` to actually move things. Each
 * such controller **owns a `FleetSession` by composition** (a plain
 * member, not a base class to inherit from and override) and adds
 * whatever domain-specific decision logic it needs around it -- matching
 * how the rest of this codebase favors composition/mixins
 * (`ActionServerBase`, `Capability<T>`) over runtime-polymorphic base
 * classes. A user is free to write their own controller the same way,
 * with no support needed from this class beyond what's public here.
 *
 * Usage (this is genuinely all `SimpleController` itself does):
 * \code
 * easyfleet::init(argc, argv);
 * easyfleet::FleetSession session;
 *
 * easyfleet::RobotHandle robot_1("robot_1");
 * session.add_robot(robot_1);
 * session.discover_capabilities();
 *
 * robot_1.run_capability("navigation", "kitchen");
 * while (robot_1.is_capability_running("navigation")) {
 *   session.spin_some();
 * }
 *
 * session.shutdown();
 * \endcode
 */
class FleetSession
{
public:
  FleetSession();
  ~FleetSession();

  /// @param robot Must outlive this `FleetSession`.
  void add_robot(RobotHandle & robot);

  /// Every robot added so far, in `add_robot()` order -- for code that
  /// needs to enumerate the fleet rather than command one robot it
  /// already knows the name of (e.g. building an LLM prompt describing
  /// every robot, or a PDDL problem instance).
  const std::vector<std::reference_wrapper<RobotHandle>> & robots() const noexcept;

  /// Looks up a robot added so far by `RobotHandle::name()`.
  /// @return The matching robot, or `std::nullopt` if none was added under
  ///   that name -- a nullable *reference*, without resorting to a raw
  ///   pointer to express "might not exist".
  std::optional<std::reference_wrapper<RobotHandle>> find_robot(
    const std::string & name) const noexcept;

  /// Runs exactly one discovery scan (see today's
  /// `easyfleet_mission_manager::discover_capabilities()`, which this
  /// wraps) and fans the results out to every added `RobotHandle`'s own
  /// `has_capability()`/`run_capability()`/etc. -- never a redundant
  /// per-robot re-scan, unlike calling `RobotHandle::discover_capabilities()`
  /// independently on each robot would be.
  /// @param window How long to listen before returning, same meaning as
  ///   today's `discover_capabilities()` window parameter.
  void discover_capabilities(std::chrono::milliseconds window = std::chrono::milliseconds(2500));

  /// One non-blocking spin of the underlying executor's queued work --
  /// call this in a `while (robot.is_capability_running(...))` loop the
  /// way the sketch this design started from does, instead of blocking.
  void spin_some();

  /// Blocks for exactly `duration`, still spinning underneath (equivalent
  /// to a bounded `spin_some()` loop) -- for phases with a deliberate,
  /// fixed-length window rather than a "wait for completion" one (e.g. "10
  /// seconds into this goal, redirect it elsewhere regardless of
  /// progress").
  void spin_for(std::chrono::milliseconds duration);

  /// Underlying node, for anything not yet covered by `RobotHandle`
  /// (matches today's escape hatch of reaching for `node.get()` directly).
  rclcpp::Node::SharedPtr node() const noexcept;

  /// Cancels the executor, joins its background thread, and calls
  /// `rclcpp::shutdown()` -- the mission-side mirror of
  /// `easyfleet_core::Deployment::run()`'s teardown.
  void shutdown();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_thread_;
  std::vector<std::reference_wrapper<RobotHandle>> robots_;
  std::unique_ptr<easyfleet_mission_manager::StatusMarkerPublisher> status_;
};

}  // namespace easyfleet

#endif  // EASYFLEET_MISSION_MANAGER__FLEET_SESSION_HPP_
