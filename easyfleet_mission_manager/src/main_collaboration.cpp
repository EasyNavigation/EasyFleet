// Copyright 2026 Intelligent Robotics Lab
//
// This file is part of the projects Arquimea-URJC and AURORAS
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

// Mission control demo for the "collaboration" deployment scenario (see
// src/arquimea_project/deployments/launch/collaboration): three robots --
//   - robot_1: perception, navigation
//   - robot_2: perception, navigation
//   - robot_3: navigation, manipulation
// The mission has two phases:
//   1. robot_1 and robot_2 run navigation and perception together, in
//      parallel, for up to 10s.
//   2. robot_3 runs navigation to completion, then manipulation.

#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "arch_mockup/capability_client.hpp"
#include "control_center/capability_discovery.hpp"
#include "control_center/capability_info.hpp"
#include "control_center/mission_helpers.hpp"
#include "control_center/output.hpp"
#include "control_center/run_capability.hpp"

using namespace control_center;
using namespace std::chrono_literals;

namespace
{

// This demo is specific to the "collaboration" deployment scenario, which
// defines exactly these three robots (see
// src/arquimea_project/deployments/launch/collaboration/).
const std::string kRobot1 = "robot_1";
const std::string kRobot2 = "robot_2";
const std::string kRobot3 = "robot_3";

/// Sends `goal` to `client` and blocks until it finishes (naturally, or
/// because `kRunTimeout` elapsed and it got stopped), returning the
/// outcome as text -- used to narrate sequential phases of the mission.
template<typename ActionT>
std::string run_and_describe(
  typename arch_mockup::CapabilityClient<ActionT>::SharedPtr client,
  const std::string & label,
  const typename ActionT::Goal & goal,
  std::function<void(const typename ActionT::Feedback &)> on_feedback)
{
  auto response = run_capability<ActionT>(client, label, goal, kRunTimeout, on_feedback);
  return outcome_to_string(static_cast<uint8_t>(response.outcome));
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("control_center_collaboration");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spin_thread = spin_in_background(executor);

  safe_print(
    std::string(ansi::bold) + ansi::cyan +
    "Control Center -- \"collaboration\" mission (" + kRobot1 + ", " + kRobot2 + ", " +
    kRobot3 + ")" + ansi::reset);

  // Discover active capabilities and show their descriptions.
  print_section("Phase 0: Discovering capabilities");
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

  const auto * robot1_navigation = find_robot_capability(capabilities, kRobot1, "navigation");
  const auto * robot1_perception = find_robot_capability(capabilities, kRobot1, "perception");
  const auto * robot2_navigation = find_robot_capability(capabilities, kRobot2, "navigation");
  const auto * robot2_perception = find_robot_capability(capabilities, kRobot2, "perception");
  const auto * robot3_navigation = find_robot_capability(capabilities, kRobot3, "navigation");
  const auto * robot3_manipulation = find_robot_capability(capabilities, kRobot3, "manipulation");

  // Each CapabilityClient is created against the *resolved* action name
  // (e.g. "/robot_1/navigation"), which is what each robot actually
  // announced itself under.
  arch_mockup::CapabilityClient<NavigateToPose>::SharedPtr robot1_navigation_client;
  if (robot1_navigation) {
    robot1_navigation_client = arch_mockup::CapabilityClient<NavigateToPose>::create(
      node.get(), robot1_navigation->action_name);
  }
  arch_mockup::CapabilityClient<Perception>::SharedPtr robot1_perception_client;
  if (robot1_perception) {
    robot1_perception_client = arch_mockup::CapabilityClient<Perception>::create(
      node.get(), robot1_perception->action_name);
  }
  arch_mockup::CapabilityClient<NavigateToPose>::SharedPtr robot2_navigation_client;
  if (robot2_navigation) {
    robot2_navigation_client = arch_mockup::CapabilityClient<NavigateToPose>::create(
      node.get(), robot2_navigation->action_name);
  }
  arch_mockup::CapabilityClient<Perception>::SharedPtr robot2_perception_client;
  if (robot2_perception) {
    robot2_perception_client = arch_mockup::CapabilityClient<Perception>::create(
      node.get(), robot2_perception->action_name);
  }
  arch_mockup::CapabilityClient<NavigateToPose>::SharedPtr robot3_navigation_client;
  if (robot3_navigation) {
    robot3_navigation_client = arch_mockup::CapabilityClient<NavigateToPose>::create(
      node.get(), robot3_navigation->action_name);
  }
  arch_mockup::CapabilityClient<ExecuteTrajectory>::SharedPtr robot3_manipulation_client;
  if (robot3_manipulation) {
    robot3_manipulation_client = arch_mockup::CapabilityClient<ExecuteTrajectory>::create(
      node.get(), robot3_manipulation->action_name);
  }

  // Phase 1: robot_1 and robot_2 run navigation + perception together, in
  // parallel, for up to kRunTimeout seconds.
  print_section("Phase 1: " + kRobot1 + " & " + kRobot2 + " -- navigation + perception");
  print_step(
    "running " + kRobot1 + "'s and " + kRobot2 + "'s navigation and perception all at "
    "once, for up to " + std::to_string(kRunTimeout.count()) + "s.");
  std::vector<std::thread> phase1_threads;
  if (robot1_navigation_client) {
    phase1_threads.emplace_back(
      [&robot1_navigation_client, &robot1_navigation] {
        run_capability<NavigateToPose>(
          robot1_navigation_client, robot1_navigation->action_name, make_navigation_goal(),
          kRunTimeout, make_navigation_feedback_printer(robot1_navigation->action_name));
      });
  } else {
    print_step("[" + kRobot1 + "/navigation] not active, skipping.");
  }
  if (robot1_perception_client) {
    phase1_threads.emplace_back(
      [&robot1_perception_client, &robot1_perception] {
        run_capability<Perception>(
          robot1_perception_client, robot1_perception->action_name, make_perception_goal(),
          kRunTimeout, make_perception_feedback_printer(robot1_perception->action_name));
      });
  } else {
    print_step("[" + kRobot1 + "/perception] not active, skipping.");
  }
  if (robot2_navigation_client) {
    phase1_threads.emplace_back(
      [&robot2_navigation_client, &robot2_navigation] {
        run_capability<NavigateToPose>(
          robot2_navigation_client, robot2_navigation->action_name, make_navigation_goal(),
          kRunTimeout, make_navigation_feedback_printer(robot2_navigation->action_name));
      });
  } else {
    print_step("[" + kRobot2 + "/navigation] not active, skipping.");
  }
  if (robot2_perception_client) {
    phase1_threads.emplace_back(
      [&robot2_perception_client, &robot2_perception] {
        run_capability<Perception>(
          robot2_perception_client, robot2_perception->action_name, make_perception_goal(),
          kRunTimeout, make_perception_feedback_printer(robot2_perception->action_name));
      });
  } else {
    print_step("[" + kRobot2 + "/perception] not active, skipping.");
  }
  if (phase1_threads.empty()) {
    print_step("nothing active for " + kRobot1 + " or " + kRobot2 + ", skipping phase 1.");
  }
  for (auto & thread : phase1_threads) {
    thread.join();
  }
  print_step("Phase 1 done: " + kRobot1 + " and " + kRobot2 + " have stopped.");

  // Phase 2: robot_3 runs navigation to completion, then manipulation.
  print_section("Phase 2: " + kRobot3 + " -- navigation, then manipulation");
  if (robot3_navigation_client) {
    print_step("starting " + kRobot3 + "'s navigation; manipulation will follow once it ends.");
    const auto outcome = run_and_describe<NavigateToPose>(
      robot3_navigation_client, robot3_navigation->action_name, make_navigation_goal(),
      make_navigation_feedback_printer(robot3_navigation->action_name));
    print_step(
      kRobot3 + "'s navigation finished with outcome " + outcome + "; starting manipulation.");
  } else {
    print_step("[" + kRobot3 + "/navigation] not active, skipping straight to manipulation.");
  }

  if (robot3_manipulation_client) {
    run_capability<ExecuteTrajectory>(
      robot3_manipulation_client, robot3_manipulation->action_name, make_manipulation_goal(),
      kRunTimeout, make_manipulation_feedback_printer(robot3_manipulation->action_name));
  } else {
    print_step("[" + kRobot3 + "/manipulation] not active, skipping.");
  }
  print_step("Phase 2 done.");

  print_section("Mission complete");

  executor.cancel();
  spin_thread.join();
  executor.remove_node(node);
  rclcpp::shutdown();
  return 0;
}
