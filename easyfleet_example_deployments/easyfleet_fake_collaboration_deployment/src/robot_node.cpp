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

// The robot: a single process hosting whichever fake capabilities this
// robot is configured to carry (see the "capabilities"/"config_subdir"
// parameters, e.g. config/robot_1/robot_params.yaml), each its own
// easyfleet_core::Capability lifecycle node. Reused unchanged across
// robot_1/robot_2/robot_3 (each carries a different capability subset --
// see their own robot_params.yaml). Kept deliberately separate from the
// mission control program (main_collaboration.cpp / collaboration_mission_node),
// which only ever talks to capabilities over their ROS actions, never
// links against their implementation -- this process is "the robot", that
// one is "the operator".

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_path.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easyfleet_core/spin_utils.hpp"

#include "easyfleet_fake_collaboration_deployment/manipulation_fake_capability.hpp"
#include "easyfleet_fake_collaboration_deployment/navigation_fake_capability.hpp"
#include "easyfleet_fake_collaboration_deployment/perception_fake_capability.hpp"

namespace
{

using LifecycleNodePtr = std::shared_ptr<rclcpp_lifecycle::LifecycleNode>;

/// Constructs the capability named `name`, pre-seeded with its
/// `capabilities_file` parameter (resolved here, in code, via
/// ament_index_cpp, rather than a launch-time `$(find-pkg-share ...)`
/// substitution -- this process hosts several distinctly-named lifecycle
/// nodes, and launch_yaml's inline parameter overrides only have one
/// unambiguous target node per `node:` action, this process's exec has
/// none). Returns nullptr (and logs) if `name` isn't one this package
/// knows how to host.
LifecycleNodePtr make_capability(
  const rclcpp::Logger & logger, const std::string & name, const std::string & capabilities_file)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("capabilities_file", capabilities_file)});

  if (name == "navigation") {
    return std::make_shared<easyfleet_fake_collaboration_deployment::NavigationFakeCapability>(
        options);
  }
  if (name == "manipulation") {
    return std::make_shared<easyfleet_fake_collaboration_deployment::ManipulationFakeCapability>(
        options);
  }
  if (name == "perception") {
    return std::make_shared<easyfleet_fake_collaboration_deployment::PerceptionFakeCapability>(
        options);
  }
  RCLCPP_FATAL(logger, "Unknown capability '%s' requested.", name.c_str());
  return nullptr;
}

}  // namespace

int main(int argc, char ** argv)
{
  // SignalHandlerOptions::None: see easyfleet_core::spin_until_shutdown()'s
  // doc comment for why rclcpp's own default SIGINT/SIGTERM handling would
  // break this executable's graceful lifecycle shutdown below.
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);

  // A throwaway node, existing only to read which capabilities this robot
  // process should host and where their JSON descriptions live -- not
  // spun, not added to the executor below. Deliberately not remapped by
  // the launch file (which would apply globally to every LifecycleNode
  // this process creates): each capability below already carries its own
  // explicit name ("navigation", "manipulation", "perception", set by its
  // own constructor), so this node just needs one distinct from theirs.
  auto config_node = std::make_shared<rclcpp::Node>("robot_config");
  const auto capability_names = config_node->declare_parameter(
    "capabilities", std::vector<std::string>());
  const auto config_subdir = config_node->declare_parameter(
    "config_subdir", std::string());

  if (capability_names.empty()) {
    RCLCPP_FATAL(
      config_node->get_logger(),
      "No 'capabilities' configured for this robot -- nothing to host, exiting.");
    rclcpp::shutdown();
    return 1;
  }

  const std::filesystem::path share_dir = ament_index_cpp::get_package_share_path(
    "easyfleet_fake_collaboration_deployment");

  std::vector<LifecycleNodePtr> nodes;
  for (const auto & name : capability_names) {
    const std::string capabilities_file =
      (share_dir / "config" / config_subdir / (name + ".json")).string();
    auto node = make_capability(config_node->get_logger(), name, capabilities_file);
    if (!node) {
      rclcpp::shutdown();
      return 1;
    }
    nodes.push_back(node);
  }

  for (const auto & node : nodes) {
    node->configure();
    if (node->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      RCLCPP_FATAL(node->get_logger(), "Failed to configure, exiting.");
      rclcpp::shutdown();
      return 1;
    }
  }
  for (const auto & node : nodes) {
    node->activate();
    if (node->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      RCLCPP_FATAL(node->get_logger(), "Failed to activate, exiting.");
      rclcpp::shutdown();
      return 1;
    }
  }

  // One executor for every capability this robot hosts: each capability's
  // actual goal execution runs on its own worker thread (see
  // easyfleet_core::ActionServerBase), so sharing a single-threaded
  // executor here for the lightweight ROS callback dispatch (goal accept/
  // reject, feedback publish, heartbeats) doesn't starve the others.
  rclcpp::executors::SingleThreadedExecutor executor;
  for (const auto & node : nodes) {
    executor.add_node(node->get_node_base_interface());
  }
  easyfleet_core::spin_until_shutdown(executor);
  for (const auto & node : nodes) {
    executor.remove_node(node->get_node_base_interface());
  }

  for (const auto & node : nodes) {
    if (node->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      node->deactivate();
    }
    if (node->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      node->cleanup();
    }
    // Reaches PRIMARY_STATE_FINALIZED from whichever state the node is in
    // now -- without this, the node's destructor logs a (harmless, but
    // noisy) "not shut down" warning.
    node->shutdown();
  }

  rclcpp::shutdown();
  return 0;
}
