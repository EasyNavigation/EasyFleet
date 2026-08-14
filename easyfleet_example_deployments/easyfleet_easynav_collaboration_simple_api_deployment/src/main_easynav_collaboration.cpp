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

// TEMPORARY, NOT YET BUILT (see CMakeLists.txt): the simple-API mission
// script for this scenario, written against easyfleet_mission_manager's
// RobotHandle/SimpleController -- which are still header-only (no .cpp
// bodies yet, see refactor.md). Also still references the pre-rename
// client-side "Robot" type from the original sketch; that type is meant to
// become "RobotHandle" once implemented (see refactor.md decision 4) --
// this file hasn't been updated to match that rename yet either. Kept as
// the authoritative starting point for that implementation work, not
// deleted, even though it can't compile today.
//
// Mission control demo for this deployment scenario (see
// src/EasyFleet/easyfleet_example_deployments/easyfleet_easynav_collaboration_simple_api_deployment/launch):
// two robots, both real-EasyNav-navigating on the same shared map --
//   - robot_1: navigation (real EasyNav), perception (mock)
//   - robot_2: navigation (real EasyNav), manipulation (mock)
//
// A longer, five-phase choreography exercising real navigation end to end
// (goals are left to actually SUCCEED, not stopped early -- see
// kLongTimeout below -- except where phase 4 deliberately cancels one):
//   1. robot_1 -> "kitchen" (perceiving throughout) while robot_2 -> "dock",
//      simultaneously. Waits for both navigations to actually finish (not
//      a timeout).
//   2. robot_1 -> "kitchen_standby" (1m short of "kitchen", still facing
//      it, perception still running from phase 1) to clear space, while
//      robot_2 -> "kitchen", simultaneously.
//   3. Once robot_2 is at "kitchen" (and robot_1's perception is stopped,
//      its job here done), robot_2 runs manipulation for up to 10s.
//   4. robot_1 -> "dock" while robot_2 -> "dock" too, simultaneously; 10s
//      into robot_1's navigation it is explicitly stopped and redirected to
//      "charging_station" instead.
//   5. Once both navigations from phase 4 finish, both robots return to
//      their own starting pose ("robot_1_home"/"robot_2_home") and any
//      still-running capability goal is stopped.
//
// Also publishes a short status line above each robot in RViz
// (easyfleet_mission_manager::StatusMarkerPublisher, a floating
// TEXT_VIEW_FACING marker at each robot's own base_link) at every
// meaningful transition above, so someone watching only RViz -- not this
// program's terminal output -- can follow which capability each robot is
// using and what it's doing/its result, without reading code or logs.

#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "easyfleet_core/capability_client.hpp"
#include "easyfleet_mission_manager/capability_discovery.hpp"
#include "easyfleet_mission_manager/capability_info.hpp"
#include "easyfleet_mission_manager/mission_helpers.hpp"
#include "easyfleet_mission_manager/output.hpp"
#include "easyfleet_mission_manager/run_capability.hpp"
#include "easyfleet_mission_manager/status_markers.hpp"

using namespace easyfleet_mission_manager;
using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  easyfleet::init(argc, argv);

  easyfleet::SimpleController controller;

  easyfleet::Robot robot_1("robot_1");
  easyfleet::Robot robot_2("robot_2");

  controller.add_robot(robot_1);
  controller.add_robot(robot_2);

  robot_1.discover_capabilities();
  robot_2.discover_capabilities();

  robot_1.print_capabilities();
  robot_2.print_capabilities();

  if (!robot_1.has_capability("navigation") || !robot_2.has_capability("navigation")) {
    safe_print(
      std::string(ansi::bold) + ansi::red +
      "Both robots must have a navigation capability for this demo to work." + ansi::reset);
    return 1;
  }

  if (!robot_1.has_capability("perception")) {
    safe_print(
      std::string(ansi::bold) + ansi::red +
      "Robot 1 must have a perception capability for this demo to work." + ansi::reset);
    return 1;
  }

  if (!robot_2.has_capability("manipulation")) {
    safe_print(
      std::string(ansi::bold) + ansi::red +
      "Robot 2 must have a manipulation capability for this demo to work." + ansi::reset);
    return 1;
  }

  // Phase 1: robot_1 -> "kitchen" (perceiving throughout) while robot_2 ->
  // "dock", simultaneously. robot_1's perception is started here and kept
  // running (untouched) across phase 2 too -- it's only stopped once
  // robot_1 actually reaches "kitchen_standby" at the end of phase 2, see
  // there.

  print_section(
    "Phase 1: " + kRobot1 + " -> \"" + kKitchen + "\" (perceiving), " + kRobot2 + " -> \"" +
    kDock + "\" (simultaneously)");

  robot_1.run_capability("navigation", "kitchen");
  robot_1.run_capability("perception");
  robot_2.run_capability("navigation", "dock");

  while (robot_1.is_capability_running("navigation") ||
    robot_2.is_capability_running("navigation"))
  {
    controller.spin_some();
  }

  print_step("Phase 1 done: both robots reached their waypoint.");

  // Phase 2: robot_1 -> "kitchen_standby" (still perceiving) while robot_2
  // -> "kitchen", simultaneously.
  print_section(
    "Phase 2: " + kRobot1 + " -> \"" + kKitchenStandby + "\" (clearing space, still perceiving), " +
      kRobot2 + " -> \"" + kKitchen + "\" (simultaneously)");

  robot_1.run_capability("navigation", "kitchen_standby");
  robot_1.run_capability("perception");

  robot_2.run_capability("navigation", "kitchen");

  while (robot_1.is_capability_running("navigation") ||
    robot_2.is_capability_running("navigation"))
  {
    controller.spin_some();
  }

  robot_1.stop_capability("perception");

  print_step("Phase 2 done: " + kRobot1 + " clear of \"" + kKitchen + "\", " + kRobot2 + " there.");

  // Phase 3: robot_2, now at "kitchen", runs manipulation for up to 10s
  // (the mock's own configured duration finishes well within that).
  print_section("Phase 3: " + kRobot2 + " manipulation at \"" + kKitchen + "\"");

  robot_2.run_capability("manipulation", "kitchen", 10);
  controller.spin_for(std::chrono::seconds(10));

  robot_2.stop_capability("manipulation");

  print_step("Phase 3 done.");

  // Phase 4: robot_1 -> "dock" while robot_2 -> "dock" too (robot_2 was
  // still at "kitchen" from phase 3, so this is a real move, not a no-op),
  // simultaneously; 10s into robot_1's navigation it is explicitly stopped
  // (via run_capability's own timeout-then-cancel behavior) and redirected
  // to "charging_station" instead.
  print_section(
    "Phase 4: " + kRobot1 + " -> \"" + kDock + "\" (canceled after 10s) -> \"" +
    kChargingStation + "\", " + kRobot2 + " -> \"" + kDock + "\" (simultaneously)");

  robot_1.run_capability("navigation", "dock");
  robot_2.run_capability("navigation", "dock");

  controller.spin_for(std::chrono::seconds(10));

  robot_1.run_capability("navigation", "charging_station");

  while (robot_1.is_capability_running("navigation") ||
    robot_2.is_capability_running("navigation"))
  {
    controller.spin_some();
  }
  print_step(
    "Phase 4 done: " + kRobot1 + " at \"" + kChargingStation + "\", " + kRobot2 + " at \"" +
    kDock + "\".");

  // Phase 5: both robots return to their own starting pose, simultaneously.
  print_section(
    "Phase 5: " + kRobot1 + " -> \"" + robot_1_home + "\", " + kRobot2 + " -> \"" + robot_2_home +
    "\" (simultaneously)");

  robot_1.run_capability("navigation", "robot_1_home");
  robot_2.run_capability("navigation", "robot_2_home");

  while (robot_1.is_capability_running("navigation") ||
    robot_2.is_capability_running("navigation"))
  {
    controller.spin_some();
  }

  print_section("Mission complete");

  controller.shutdown();

  return 0;
}
