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

// The simple-API mission script for this scenario, written against
// easyfleet_mission_manager's RobotHandle/SimpleController -- see
// easyfleet_easynav_collaboration_deployment/src/main_easynav_collaboration.cpp
// for the low-level-API sibling this is meant to behave identically to.
// Status markers above each robot in RViz are not set here explicitly the
// way the low-level-API sibling does: RobotHandle::run_capability()
// publishes them automatically for the life of each goal, which is exactly
// the boilerplate this simpler API exists to remove.
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

#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "easyfleet_core/init.hpp"
#include "easyfleet_mission_manager/mission_helpers.hpp"
#include "easyfleet_mission_manager/output.hpp"
#include "easyfleet_mission_manager/robot_handle.hpp"
#include "easyfleet_mission_manager/simple_controller.hpp"

using namespace easyfleet_mission_manager;
using namespace std::chrono_literals;

namespace
{

// This demo is specific to this deployment scenario, which defines exactly
// these two robots (see .../easyfleet_easynav_collaboration_simple_api_deployment/launch/).
const std::string kRobot1 = "robot_1";
const std::string kRobot2 = "robot_2";

// Named waypoints configured on both robots' navigation capability, see
// config/navigation_params.yaml -- real, reachable points on the shared
// home2 map.
const std::string kDock = "dock";
const std::string kKitchen = "kitchen";
const std::string kKitchenStandby = "kitchen_standby";
const std::string kChargingStation = "charging_station";
const std::string kRobot1Home = "robot_1_home";
const std::string kRobot2Home = "robot_2_home";

// Real navigation goals are meant to run to actual completion in this
// mission (see the file's own header comment) -- this is a generous safety
// net, not the expected way a goal ends, unlike easyfleet_mission_manager's
// own kRunTimeout (10s), which is both too short for real navigation and
// -- for every navigation goal below except one -- not what's wanted here.
constexpr std::chrono::seconds kLongTimeout(180);

}  // namespace

int main(int argc, char ** argv)
{
  easyfleet::init(argc, argv);

  easyfleet::SimpleController controller;

  easyfleet::RobotHandle robot_1(kRobot1);
  easyfleet::RobotHandle robot_2(kRobot2);

  controller.add_robot(robot_1);
  controller.add_robot(robot_2);

  safe_print(
    std::string(ansi::bold) + ansi::cyan +
    "Control Center -- two-robot \"easynav\" mission (" + kRobot1 + ", " + kRobot2 + ")" +
    ansi::reset);

  print_section("Phase 0: Discovering capabilities");
  print_step("listening on /capabilities and /capabilities_status ...");
  controller.discover_capabilities();

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

  robot_1.run_capability<Navigation>("navigation", make_navigation_goal(kKitchen), kLongTimeout);
  robot_1.run_capability<Perception>("perception", make_perception_goal(), kLongTimeout);
  robot_2.run_capability<Navigation>("navigation", make_navigation_goal(kDock), kLongTimeout);

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

  // robot_1's perception keeps running, untouched, from phase 1 -- no need
  // to (re-)issue it here; doing so would just preempt the still-running
  // goal with an identical one.
  robot_1.run_capability<Navigation>(
    "navigation", make_navigation_goal(kKitchenStandby), kLongTimeout);

  robot_2.run_capability<Navigation>("navigation", make_navigation_goal(kKitchen), kLongTimeout);

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

  // Default timeout (kRunTimeout, 10s) already matches what this phase
  // wants.
  robot_2.run_capability<Manipulation>("manipulation", make_manipulation_goal());
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

  // Default timeout (kRunTimeout, 10s) here is deliberate: this is the
  // goal meant to be capped and redirected below, unlike every other
  // navigation goal in this mission.
  robot_1.run_capability<Navigation>("navigation", make_navigation_goal(kDock));
  robot_2.run_capability<Navigation>("navigation", make_navigation_goal(kDock), kLongTimeout);

  controller.spin_for(std::chrono::seconds(10));

  // A new goal on the same capability preempts the one still in flight
  // (ActionServerBase's own preemption handling) -- no explicit
  // stop_capability() needed first.
  robot_1.run_capability<Navigation>(
    "navigation", make_navigation_goal(kChargingStation), kLongTimeout);

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
    "Phase 5: " + kRobot1 + " -> \"" + kRobot1Home + "\", " + kRobot2 + " -> \"" + kRobot2Home +
    "\" (simultaneously)");

  robot_1.run_capability<Navigation>("navigation", make_navigation_goal(kRobot1Home), kLongTimeout);
  robot_2.run_capability<Navigation>("navigation", make_navigation_goal(kRobot2Home), kLongTimeout);

  while (robot_1.is_capability_running("navigation") ||
    robot_2.is_capability_running("navigation"))
  {
    controller.spin_some();
  }

  // Safety net: nothing should still be running by this point (perception
  // was stopped after phase 2, manipulation after phase 3, and every
  // navigation goal above already ran to completion or was explicitly
  // redirected), but stop anything left active regardless before
  // declaring the mission over.
  robot_1.stop_capability("perception");
  robot_2.stop_capability("manipulation");

  print_step("Phase 5 done: both robots back at their starting pose.");
  print_section("Mission complete");

  controller.shutdown();

  return 0;
}
