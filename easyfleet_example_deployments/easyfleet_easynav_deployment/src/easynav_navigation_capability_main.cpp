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

#include <memory>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"

#include "easyfleet_core/spin_utils.hpp"
#include "easyfleet_easynav_deployment/easynav_navigation_capability.hpp"

int main(int argc, char ** argv)
{
  // SignalHandlerOptions::None: see easyfleet_core::spin_until_shutdown()'s
  // doc comment for why rclcpp's own default SIGINT/SIGTERM handling would
  // break this executable's graceful lifecycle shutdown below.
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);

  auto node = std::make_shared<easyfleet_easynav_deployment::EasynavNavigationCapability>();

  node->configure();
  if (node->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    RCLCPP_FATAL(node->get_logger(), "easynav navigation capability failed to configure, exiting.");
    rclcpp::shutdown();
    return 1;
  }

  node->activate();
  if (node->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_FATAL(node->get_logger(), "easynav navigation capability failed to activate, exiting.");
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  easyfleet_core::spin_until_shutdown(executor);
  executor.remove_node(node->get_node_base_interface());

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

  rclcpp::shutdown();
  return 0;
}
