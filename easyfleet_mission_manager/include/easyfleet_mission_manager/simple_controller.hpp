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

#ifndef EASYFLEET_MISSION_MANAGER__SIMPLE_CONTROLLER_HPP_
#define EASYFLEET_MISSION_MANAGER__SIMPLE_CONTROLLER_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "easyfleet_mission_manager/fleet_session.hpp"
#include "easyfleet_mission_manager/robot_handle.hpp"

namespace easyfleet
{

/// @brief The plain, manual controller: a mission script decides everything by
/// hand (which robot runs which capability, when), `SimpleController` just
/// hosts the `FleetSession` that makes talking to those robots possible.
/**
 * Owns a `FleetSession` by composition and forwards every call straight to
 * it -- there is no behavior here beyond the session's own. This is
 * deliberate: `SimpleController` is meant to be the reference example of
 * "a controller with zero decision-making logic of its own", so it also
 * doubles as the template for writing a *different* one (an
 * `LLMController`, a `PlanSys2Controller`, or one of your own) -- see
 * `FleetSession`'s own doc comment for why those should be built the same
 * way (own a `FleetSession`, add logic around it) rather than by
 * subclassing this class.
 *
 * Usage:
 * \code
 * easyfleet::init(argc, argv);
 * easyfleet::SimpleController controller;
 *
 * easyfleet::RobotHandle robot_1("robot_1");
 * easyfleet::RobotHandle robot_2("robot_2");
 * controller.add_robot(robot_1);
 * controller.add_robot(robot_2);
 *
 * controller.discover_capabilities();
 * // ... robot_1.run_capability(...), controller.spin_some()/spin_for(...) ...
 *
 * controller.shutdown();
 * \endcode
 */
class SimpleController
{
public:
  SimpleController() = default;

  /// @brief Adds `robot` to this controller's session.
  /// @param robot Must outlive this `SimpleController`.
  void add_robot(RobotHandle & robot) {session_.add_robot(robot);}

  /// @brief See `FleetSession::robots()`.
  /// @return Every robot added so far.
  const std::vector<std::reference_wrapper<RobotHandle>> & robots() const noexcept
  {
    return session_.robots();
  }

  /// @brief See `FleetSession::find_robot()`.
  /// @param name Name to look up, as passed to `RobotHandle`'s constructor.
  /// @return The matching robot, or `std::nullopt` if none was added under
  ///   that name.
  std::optional<std::reference_wrapper<RobotHandle>> find_robot(
    const std::string & name) const noexcept
  {
    return session_.find_robot(name);
  }

  /// @brief See `FleetSession::discover_capabilities()`.
  /// @param window How long to listen before returning.
  void discover_capabilities(std::chrono::milliseconds window = std::chrono::milliseconds(2500))
  {
    session_.discover_capabilities(window);
  }

  /// @brief See `FleetSession::spin_some()`.
  void spin_some() {session_.spin_some();}

  /// @brief See `FleetSession::spin_for()`.
  /// @param duration How long to block for.
  void spin_for(std::chrono::milliseconds duration) {session_.spin_for(duration);}

  /// @brief See `FleetSession::node()`.
  /// @return The underlying ROS node.
  rclcpp::Node::SharedPtr node() const noexcept {return session_.node();}

  /// @brief See `FleetSession::shutdown()`.
  void shutdown() {session_.shutdown();}

private:
  FleetSession session_;
};

}  // namespace easyfleet

#endif  // EASYFLEET_MISSION_MANAGER__SIMPLE_CONTROLLER_HPP_
