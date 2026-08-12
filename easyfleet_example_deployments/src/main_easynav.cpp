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

// Mission control demo for the "easynav" deployment scenario (see
// src/EasyFleet/easyfleet_example_deployments/launch/easynav): whichever
// robot's navigation capability is the real EasyNav-backed one. Discovers
// it, prints its description, sends it to a named waypoint, then
// demonstrates EasyNav-level preemption by sending a second goal to a
// different waypoint while the first is still running.
//
// Deliberately does *not* hardcode a robot name: this scenario has two
// launch variants that both advertise a "navigation" capability but under
// different identities -- easynav_robot_launch.yaml (Dummy* plugins) names
// its robot "easynav_robot", while easynav_robot_gazebo_launch.yaml (real
// navigation) runs unnamespaced (robot ""). Whichever one is actually up
// is the one this mission talks to.

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "easyfleet_core/capability_client.hpp"
#include "easyfleet_mission_manager/capability_discovery.hpp"
#include "easyfleet_mission_manager/capability_info.hpp"
#include "easyfleet_mission_manager/mission_helpers.hpp"
#include "easyfleet_mission_manager/output.hpp"
#include "easyfleet_mission_manager/run_capability.hpp"

using namespace easyfleet_mission_manager;
using namespace std::chrono_literals;

namespace
{

// Named waypoints configured on the navigation capability, see
// config/easynav/easynav_robot/navigation_params.yaml -- shared by both
// launch variants.
const std::string kFirstWaypoint = "dock";
const std::string kSecondWaypoint = "kitchen";

// Unlike the mock navigation capability, this backend ignores target_pose
// and instead reads the requested waypoint id from parameters_json (see
// README.md's "The EasyNav-backed navigation capability" section).
Navigation::Goal make_easynav_goal(const std::string & waypoint_id)
{
  Navigation::Goal goal;
  goal.parameters_json = R"({"goal_id": ")" + waypoint_id + R"("})";
  return goal;
}

/// Finds the single active "navigation" capability, regardless of which
/// robot (or lack thereof) it's announced under. If more than one is
/// active at once -- e.g. both easynav launch variants running
/// simultaneously -- warns and picks the first, since only one is meant to
/// be up at a time for this scenario.
const CapabilityInfo * find_navigation_capability(
  const std::vector<CapabilityInfo> & capabilities)
{
  const CapabilityInfo * found = nullptr;
  for (const auto & info : capabilities) {
    if (info.capability != "navigation" || !info.active) {
      continue;
    }
    if (found) {
      print_step(
        "warning: more than one active 'navigation' capability found (also '" +
        info.action_name + "') -- using '" + found->action_name + "'.");
      continue;
    }
    found = &info;
  }
  return found;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("easynav_mission");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spin_thread = spin_in_background(executor);

  safe_print(
    std::string(ansi::bold) + ansi::cyan +
    "Control Center -- \"easynav\" mission" + ansi::reset);

  // 1 & 2: discover active capabilities and show their descriptions.
  print_section("Phase 1: Discovering capabilities");
  print_step("listening on /capabilities and /capabilities_status ...");
  auto capabilities = discover_capabilities(node.get());

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

  const auto * navigation_info = find_navigation_capability(capabilities);
  if (navigation_info) {
    print_step(
      "using '" + navigation_info->action_name + "' (robot '" + navigation_info->robot + "').");
  }

  // The CapabilityClient is created against the *resolved* action name
  // (e.g. "/easynav_robot/navigation" for the Dummy-plugin launch, or
  // "/navigation" for the real-navigation one), whichever this capability
  // actually announced itself under.
  easyfleet_core::CapabilityClient<Navigation>::SharedPtr navigation_client;
  if (navigation_info) {
    navigation_client = easyfleet_core::CapabilityClient<Navigation>::create(
      node.get(), navigation_info->action_name);
  }

  // 3: navigate to a waypoint. Against the Dummy-plugin launch (see
  // config/easynav/easynav_robot/easynav_system.dummy.params.yaml),
  // DummyLocalizer never reports a robot pose and the goal never finishes
  // on its own -- run_capability() stops it once kRunTimeout elapses, same
  // "runs until stopped" pattern as perception_fake_capability. Against
  // the real-navigation launch, the goal actually completes once the
  // robot reaches the waypoint.
  print_section("Phase 2: Navigate to \"" + kFirstWaypoint + "\"");
  if (navigation_client) {
    run_capability<Navigation>(
      navigation_client, navigation_info->action_name, make_easynav_goal(kFirstWaypoint),
      kRunTimeout, make_navigation_feedback_printer(navigation_info->action_name));
  } else {
    print_step(
      "no active 'navigation' capability found -- is easynav_launch.yaml or "
      "easynav_gazebo_launch.yaml running? Skipping.");
  }

  // 4: send a second goal to a different waypoint while the capability is
  // still executing the first one, to show EasyNav-level preemption: the
  // capability's persistent GoalManagerClient redirects EasyNav to the new
  // target instead of stopping first (see "The EasyNav-backed navigation
  // capability" in README.md).
  print_section("Phase 3: Preempt with a goal to \"" + kSecondWaypoint + "\"");
  if (navigation_client) {
    print_step(
      "sending a goal to \"" + kFirstWaypoint + "\", then, partway through, a second goal to \"" +
      kSecondWaypoint + "\" -- the first is aborted at the ROS level, but EasyNav is redirected, "
      "not stopped.");
    auto first_client = easyfleet_core::CapabilityClient<Navigation>::create(
      node.get(), navigation_info->action_name);
    std::thread first_goal(
      [&] {
        run_capability<Navigation>(
          first_client, navigation_info->action_name + " (1st goal)",
          make_easynav_goal(kFirstWaypoint), kRunTimeout,
          make_navigation_feedback_printer(navigation_info->action_name + " (1st goal)"));
      });
    std::this_thread::sleep_for(3s);
    run_capability<Navigation>(
      navigation_client, navigation_info->action_name + " (2nd goal)",
      make_easynav_goal(kSecondWaypoint), kRunTimeout,
      make_navigation_feedback_printer(navigation_info->action_name + " (2nd goal)"));
    first_goal.join();
  } else {
    print_step(
      "no active 'navigation' capability found -- is easynav_launch.yaml or "
      "easynav_gazebo_launch.yaml running? Skipping.");
  }
  print_step("Phase 3 done.");

  print_section("Mission complete");

  executor.cancel();
  spin_thread.join();
  executor.remove_node(node);
  rclcpp::shutdown();
  return 0;
}
