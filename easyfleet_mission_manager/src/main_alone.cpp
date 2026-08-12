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

// Mission control demo for the "alone" deployment scenario (see
// src/EasyFleet/easyfleet_example_deployments/launch/alone): a single robot, robot_1,
// carrying all three mock capabilities. Discovers them, prints their
// description, and exercises them sequentially and in parallel.

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

// This demo is specific to the "alone" deployment scenario, which defines
// exactly one robot: robot_1 (see
// src/EasyFleet/easyfleet_example_deployments/launch/alone/robot_1_launch.yaml).
const std::string kRobot = "robot_1";

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("easyfleet_mission_manager_alone");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spin_thread = spin_in_background(executor);

  safe_print(
    std::string(ansi::bold) + ansi::cyan +
    "Control Center -- \"alone\" mission (single robot: " + kRobot + ")" + ansi::reset);

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

  const auto * navigation_info = find_robot_capability(capabilities, kRobot, "navigation");
  const auto * manipulation_info = find_robot_capability(capabilities, kRobot, "manipulation");
  const auto * perception_info = find_robot_capability(capabilities, kRobot, "perception");

  // Each CapabilityClient is created against the *resolved* action name
  // (e.g. "/robot_1/navigation"), which is what robot_1 actually announced
  // itself under.
  easyfleet_core::CapabilityClient<Navigation>::SharedPtr navigation_client;
  if (navigation_info) {
    navigation_client = easyfleet_core::CapabilityClient<Navigation>::create(
      node.get(), navigation_info->action_name);
  }
  easyfleet_core::CapabilityClient<Manipulation>::SharedPtr manipulation_client;
  if (manipulation_info) {
    manipulation_client = easyfleet_core::CapabilityClient<Manipulation>::create(
      node.get(), manipulation_info->action_name);
  }
  easyfleet_core::CapabilityClient<Perception>::SharedPtr perception_client;
  if (perception_info) {
    perception_client = easyfleet_core::CapabilityClient<Perception>::create(
      node.get(), perception_info->action_name);
  }

  // 3: run each active capability sequentially, one at a time.
  print_section("Phase 2: Sequential execution");
  print_step(
    "running " + kRobot + "'s navigation, manipulation and perception one at a time, "
    "each for up to " + std::to_string(kRunTimeout.count()) +
    "s or until it finishes on its own.");
  if (navigation_client) {
    run_capability<Navigation>(
      navigation_client, navigation_info->action_name, make_navigation_goal(), kRunTimeout,
      make_navigation_feedback_printer(navigation_info->action_name));
  } else {
    print_step("[navigation] not active on '" + kRobot + "', skipping.");
  }

  if (manipulation_client) {
    run_capability<Manipulation>(
      manipulation_client, manipulation_info->action_name, make_manipulation_goal(), kRunTimeout,
      make_manipulation_feedback_printer(manipulation_info->action_name));
  } else {
    print_step("[manipulation] not active on '" + kRobot + "', skipping.");
  }

  if (perception_client) {
    run_capability<Perception>(
      perception_client, perception_info->action_name, make_perception_goal(), kRunTimeout,
      make_perception_feedback_printer(perception_info->action_name));
  } else {
    print_step("[perception] not active on '" + kRobot + "', skipping.");
  }
  print_step("Phase 2 done.");

  // 4: run every active capability in parallel.
  print_section("Phase 3: Parallel execution");
  print_step(
    "running every one of " + kRobot + "'s active capabilities together, same " +
    std::to_string(kRunTimeout.count()) + "s cap.");
  std::vector<std::thread> parallel_threads;
  if (navigation_client) {
    parallel_threads.emplace_back(
      [&navigation_client, &navigation_info] {
        run_capability<Navigation>(
          navigation_client, navigation_info->action_name, make_navigation_goal(), kRunTimeout,
          make_navigation_feedback_printer(navigation_info->action_name));
      });
  }
  if (manipulation_client) {
    parallel_threads.emplace_back(
      [&manipulation_client, &manipulation_info] {
        run_capability<Manipulation>(
          manipulation_client, manipulation_info->action_name, make_manipulation_goal(),
          kRunTimeout, make_manipulation_feedback_printer(manipulation_info->action_name));
      });
  }
  if (perception_client) {
    parallel_threads.emplace_back(
      [&perception_client, &perception_info] {
        run_capability<Perception>(
          perception_client, perception_info->action_name, make_perception_goal(), kRunTimeout,
          make_perception_feedback_printer(perception_info->action_name));
      });
  }
  if (parallel_threads.empty()) {
    print_step("no active capabilities to run in parallel.");
  }
  for (auto & thread : parallel_threads) {
    thread.join();
  }
  print_step("Phase 3 done.");

  print_section("Mission complete");

  executor.cancel();
  spin_thread.join();
  executor.remove_node(node);
  rclcpp::shutdown();
  return 0;
}
