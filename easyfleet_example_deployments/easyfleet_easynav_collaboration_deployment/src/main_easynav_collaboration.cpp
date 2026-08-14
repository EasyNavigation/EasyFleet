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

// Mission control demo for this deployment scenario (see
// src/EasyFleet/easyfleet_example_deployments/easyfleet_easynav_collaboration_deployment/launch):
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

namespace
{

// This demo is specific to this deployment scenario, which defines exactly
// these two robots (see .../easyfleet_easynav_collaboration_deployment/launch/).
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

// Unlike the mock navigation capability, this backend ignores target_pose
// and instead reads the requested waypoint id from parameters_json (see
// easyfleet_easynav_deployment/README.md's "The EasyNav-backed navigation
// capability" section).
Navigation::Goal make_easynav_goal(const std::string & waypoint_id)
{
  Navigation::Goal goal;
  goal.parameters_json = R"({"goal_id": ")" + waypoint_id + R"("})";
  return goal;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("easynav_collaboration_mission");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spin_thread = spin_in_background(executor);

  StatusMarkerPublisher status(*node);
  status.set_status(kRobot1, "Idle");
  status.set_status(kRobot2, "Idle");

  safe_print(
    std::string(ansi::bold) + ansi::cyan +
    "Control Center -- two-robot \"easynav\" mission (" + kRobot1 + ", " + kRobot2 + ")" +
    ansi::reset);

  // Discover active capabilities and show their descriptions.
  print_section("Phase 0: Discovering capabilities");
  print_step("listening on /capabilities and /capabilities_status ...");
  auto capabilities = discover_capabilities(*node);

  if (capabilities.empty()) {
    print_step("no capabilities detected. Is anything published on /capabilities?");
  } else {
    safe_print("");
    for (const auto & info : capabilities) {
      print_capability_summary_line(info);
    }
    for (const auto & info : capabilities) {
      safe_print("");
      print_capability_info(info);
    }
  }

  const auto robot1_navigation = find_robot_capability(capabilities, kRobot1, "navigation");
  const auto robot1_perception = find_robot_capability(capabilities, kRobot1, "perception");
  const auto robot2_navigation = find_robot_capability(capabilities, kRobot2, "navigation");
  const auto robot2_manipulation = find_robot_capability(capabilities, kRobot2, "manipulation");

  // Each CapabilityClient is created against the *resolved* action name
  // (e.g. "/robot_1/navigation"), which is what each robot actually
  // announced itself under.
  easyfleet_core::CapabilityClient<Navigation>::SharedPtr robot1_navigation_client;
  if (robot1_navigation) {
    robot1_navigation_client = easyfleet_core::CapabilityClient<Navigation>::create(
      *node, robot1_navigation->get().action_name);
  }
  easyfleet_core::CapabilityClient<Perception>::SharedPtr robot1_perception_client;
  if (robot1_perception) {
    robot1_perception_client = easyfleet_core::CapabilityClient<Perception>::create(
      *node, robot1_perception->get().action_name);
  }
  easyfleet_core::CapabilityClient<Navigation>::SharedPtr robot2_navigation_client;
  if (robot2_navigation) {
    robot2_navigation_client = easyfleet_core::CapabilityClient<Navigation>::create(
      *node, robot2_navigation->get().action_name);
  }
  easyfleet_core::CapabilityClient<Manipulation>::SharedPtr robot2_manipulation_client;
  if (robot2_manipulation) {
    robot2_manipulation_client = easyfleet_core::CapabilityClient<Manipulation>::create(
      *node, robot2_manipulation->get().action_name);
  }

  // Phase 1: robot_1 -> "kitchen" (perceiving throughout) while robot_2 ->
  // "dock", simultaneously. robot_1's perception is started here and kept
  // running (untouched) across phase 2 too -- it's only stopped once
  // robot_1 actually reaches "kitchen_standby" at the end of phase 2, see
  // there.
  print_section(
    "Phase 1: " + kRobot1 + " -> \"" + kKitchen + "\" (perceiving), " + kRobot2 + " -> \"" +
    kDock + "\" (simultaneously)");

  std::thread robot1_perception_thread;
  if (robot1_perception_client) {
    status.set_status(kRobot1, "Navigating -> " + kKitchen + "\nPerceiving");
    robot1_perception_thread = std::thread(
      [&] {
        run_capability<Perception>(
          robot1_perception_client, robot1_perception->get().action_name, make_perception_goal(),
          kLongTimeout, make_perception_feedback_printer(robot1_perception->get().action_name));
      });
  } else {
    status.set_status(kRobot1, "Navigating -> " + kKitchen);
    print_step("[" + kRobot1 + "/perception] not active, skipping.");
  }

  {
    std::thread robot1_thread;
    if (robot1_navigation_client) {
      robot1_thread = std::thread(
        [&] {
          run_capability<Navigation>(
            robot1_navigation_client, robot1_navigation->get().action_name,
            make_easynav_goal(kKitchen), kLongTimeout,
            make_navigation_feedback_printer(robot1_navigation->get().action_name));
        });
    } else {
      print_step("[" + kRobot1 + "/navigation] not active, skipping.");
    }
    if (robot2_navigation_client) {
      status.set_status(kRobot2, "Navigating -> " + kDock);
      run_capability<Navigation>(
        robot2_navigation_client, robot2_navigation->get().action_name, make_easynav_goal(kDock),
        kLongTimeout, make_navigation_feedback_printer(robot2_navigation->get().action_name));
    } else {
      print_step("[" + kRobot2 + "/navigation] not active, skipping.");
    }
    if (robot1_thread.joinable()) {
      robot1_thread.join();
    }
  }
  print_step("Phase 1 done: both robots reached their waypoint.");

  // Phase 2: robot_1 -> "kitchen_standby" (still perceiving) while robot_2
  // -> "kitchen", simultaneously.
  print_section(
    "Phase 2: " + kRobot1 + " -> \"" + kKitchenStandby + "\" (clearing space, still perceiving), " +
      kRobot2 + " -> \"" + kKitchen + "\" (simultaneously)");
  {
    std::thread robot1_thread;
    if (robot1_navigation_client) {
      status.set_status(
        kRobot1,
        "Navigating -> " + kKitchenStandby +
        (robot1_perception_thread.joinable() ? "\nPerceiving" : ""));
      robot1_thread = std::thread(
        [&] {
          run_capability<Navigation>(
            robot1_navigation_client, robot1_navigation->get().action_name,
            make_easynav_goal(kKitchenStandby), kLongTimeout,
            make_navigation_feedback_printer(robot1_navigation->get().action_name));
        });
    } else {
      print_step("[" + kRobot1 + "/navigation] not active, skipping.");
    }
    if (robot2_navigation_client) {
      status.set_status(kRobot2, "Navigating -> " + kKitchen);
      run_capability<Navigation>(
        robot2_navigation_client, robot2_navigation->get().action_name, make_easynav_goal(kKitchen),
        kLongTimeout, make_navigation_feedback_printer(robot2_navigation->get().action_name));
    } else {
      print_step("[" + kRobot2 + "/navigation] not active, skipping.");
    }
    if (robot1_thread.joinable()) {
      robot1_thread.join();
    }
  }
  print_step("Phase 2 done: " + kRobot1 + " clear of \"" + kKitchen + "\", " + kRobot2 + " there.");

  // robot_1's perception has done its job (giving robot_2 room to work at
  // "kitchen") -- stop it now.
  if (robot1_perception_thread.joinable()) {
    print_step("stopping " + kRobot1 + "'s perception now that it's clear of \"" + kKitchen +
      "\".");
    robot1_perception_client->cancel();
    robot1_perception_thread.join();
  }
  status.set_status(kRobot1, "At " + kKitchenStandby + "\nIdle");

  // Phase 3: robot_2, now at "kitchen", runs manipulation for up to 10s
  // (the mock's own configured duration finishes well within that).
  print_section("Phase 3: " + kRobot2 + " manipulation at \"" + kKitchen + "\"");
  if (robot2_manipulation_client) {
    status.set_status(kRobot2, "At " + kKitchen + "\nManipulating");
    run_capability<Manipulation>(
      robot2_manipulation_client, robot2_manipulation->get().action_name, make_manipulation_goal(),
      kRunTimeout, make_manipulation_feedback_printer(robot2_manipulation->get().action_name));
    status.set_status(kRobot2, "At " + kKitchen + "\nManipulation done");
  } else {
    print_step("[" + kRobot2 + "/manipulation] not active, skipping.");
  }
  print_step("Phase 3 done.");

  // Phase 4: robot_1 -> "dock" while robot_2 -> "dock" too (robot_2 was
  // still at "kitchen" from phase 3, so this is a real move, not a no-op),
  // simultaneously; 10s into robot_1's navigation it is explicitly stopped
  // (via run_capability's own timeout-then-cancel behavior) and redirected
  // to "charging_station" instead.
  print_section(
    "Phase 4: " + kRobot1 + " -> \"" + kDock + "\" (canceled after 10s) -> \"" +
    kChargingStation + "\", " + kRobot2 + " -> \"" + kDock + "\" (simultaneously)");
  {
    std::thread robot1_thread;
    if (robot1_navigation_client) {
      status.set_status(kRobot1, "Navigating -> " + kDock + "\n(will redirect in 10s)");
      robot1_thread = std::thread(
        [&] {
          run_capability<Navigation>(
            robot1_navigation_client, robot1_navigation->get().action_name + " (dock, 10s cap)",
            make_easynav_goal(kDock), kRunTimeout,
            make_navigation_feedback_printer(robot1_navigation->get().action_name));
          print_step(kRobot1 + " redirected to \"" + kChargingStation + "\".");
          status.set_status(kRobot1, "Navigating -> " + kChargingStation + "\n(redirected)");
          run_capability<Navigation>(
            robot1_navigation_client, robot1_navigation->get().action_name + " (charging_station)",
            make_easynav_goal(kChargingStation), kLongTimeout,
            make_navigation_feedback_printer(robot1_navigation->get().action_name));
        });
    } else {
      print_step("[" + kRobot1 + "/navigation] not active, skipping.");
    }
    if (robot2_navigation_client) {
      status.set_status(kRobot2, "Navigating -> " + kDock);
      run_capability<Navigation>(
        robot2_navigation_client, robot2_navigation->get().action_name, make_easynav_goal(kDock),
        kLongTimeout, make_navigation_feedback_printer(robot2_navigation->get().action_name));
    } else {
      print_step("[" + kRobot2 + "/navigation] not active, skipping.");
    }
    if (robot1_thread.joinable()) {
      robot1_thread.join();
    }
  }
  print_step(
    "Phase 4 done: " + kRobot1 + " at \"" + kChargingStation + "\", " + kRobot2 + " at \"" +
    kDock + "\".");
  status.set_status(kRobot1, "At " + kChargingStation + "\nIdle");
  status.set_status(kRobot2, "At " + kDock + "\nIdle");

  // Phase 5: both robots return to their own starting pose, simultaneously.
  print_section(
    "Phase 5: " + kRobot1 + " -> \"" + kRobot1Home + "\", " + kRobot2 + " -> \"" + kRobot2Home +
    "\" (simultaneously)");
  {
    std::thread robot1_thread;
    if (robot1_navigation_client) {
      status.set_status(kRobot1, "Navigating -> home");
      robot1_thread = std::thread(
        [&] {
          run_capability<Navigation>(
            robot1_navigation_client, robot1_navigation->get().action_name,
            make_easynav_goal(kRobot1Home), kLongTimeout,
            make_navigation_feedback_printer(robot1_navigation->get().action_name));
        });
    } else {
      print_step("[" + kRobot1 + "/navigation] not active, skipping.");
    }
    if (robot2_navigation_client) {
      status.set_status(kRobot2, "Navigating -> home");
      run_capability<Navigation>(
        robot2_navigation_client, robot2_navigation->get().action_name,
        make_easynav_goal(kRobot2Home),
        kLongTimeout, make_navigation_feedback_printer(robot2_navigation->get().action_name));
    } else {
      print_step("[" + kRobot2 + "/navigation] not active, skipping.");
    }
    if (robot1_thread.joinable()) {
      robot1_thread.join();
    }
  }
  // Safety net: nothing should still be running by this point (perception
  // was stopped after phase 2, manipulation and every navigation goal
  // above already ran to completion or was explicitly redirected), but
  // stop anything left active regardless before declaring the mission over.
  if (robot1_perception_client) {
    robot1_perception_client->cancel();
  }
  if (robot2_manipulation_client) {
    robot2_manipulation_client->cancel();
  }
  print_step("Phase 5 done: both robots back at their starting pose.");
  status.set_status(kRobot1, "Mission complete");
  status.set_status(kRobot2, "Mission complete");

  print_section("Mission complete");

  executor.cancel();
  spin_thread.join();
  executor.remove_node(node);
  rclcpp::shutdown();
  return 0;
}
