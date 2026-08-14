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

#include "easyfleet_core/deployment.hpp"

#include <cstdlib>
#include <filesystem>
#include <utility>

#include "ament_index_cpp/get_package_share_path.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"

#include "easyfleet_core/detail/namespace_utils.hpp"
#include "easyfleet_core/spin_utils.hpp"

namespace easyfleet
{

namespace
{
using CallbackReturn = easyfleet_core::CapabilityNodeBase::CallbackReturn;

// A throwaway node, discarded immediately, solely to read this process's
// own namespace. Deliberately a free function touching no member of
// Deployment: a delegating constructor's target-argument expression runs
// before any member (including config_node_, which add_capabilities_from_parameters()
// lazily creates instead) has begun construction, so evaluating it via a
// member function here would be undefined behavior.
std::string infer_robot_name()
{
  auto node = std::make_shared<rclcpp::Node>("deployment_identity");
  return easyfleet_core::detail::robot_label(node->get_namespace());
}
}  // namespace

Deployment::Deployment()
: Deployment(infer_robot_name())
{
}

Deployment::Deployment(std::string name)
: name_(std::move(name))
{
}

const std::string & Deployment::name() const noexcept
{
  return name_;
}

void Deployment::add_capability(
  const std::string & plugin_lookup_name,
  const rclcpp::NodeOptions & options)
{
  if (!loader_) {
    loader_ = std::make_unique<pluginlib::ClassLoader<easyfleet_core::CapabilityFactory>>(
      "easyfleet_core", "easyfleet_core::CapabilityFactory");
  }

  const auto slash_pos = plugin_lookup_name.find('/');
  const std::string capability_name = slash_pos == std::string::npos ?
    plugin_lookup_name : plugin_lookup_name.substr(0, slash_pos);

  const auto factory = loader_->createSharedInstance(plugin_lookup_name);
  capabilities_.push_back(factory->create(capability_name, options));
}

const std::vector<easyfleet_core::CapabilityNodeBase::SharedPtr> &
Deployment::capabilities() const noexcept
{
  return capabilities_;
}

rclcpp::Node::SharedPtr Deployment::config_node()
{
  if (!config_node_) {
    config_node_ = std::make_shared<rclcpp::Node>("deployment_config");
  }
  return config_node_;
}

void Deployment::add_capabilities_from_parameters(const std::string & package_name)
{
  const auto node = config_node();
  const auto capability_names = node->declare_parameter(
    "capabilities", std::vector<std::string>());
  const auto config_subdir = node->declare_parameter(
    "config_subdir", std::string());

  if (capability_names.empty()) {
    RCLCPP_FATAL(
      node->get_logger(),
      "No 'capabilities' configured for this robot -- nothing to host, exiting.");
    rclcpp::shutdown();
    std::exit(1);
  }

  const std::filesystem::path share_dir = ament_index_cpp::get_package_share_path(package_name);
  for (const auto & plugin_lookup_name : capability_names) {
    const auto slash_pos = plugin_lookup_name.find('/');
    const std::string capability_name = slash_pos == std::string::npos ?
      plugin_lookup_name : plugin_lookup_name.substr(0, slash_pos);
    const std::string capabilities_file =
      (share_dir / "config" / config_subdir / (capability_name + ".json")).string();

    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("capabilities_file", capabilities_file)});
    add_capability(plugin_lookup_name, options);
  }
}

void Deployment::start()
{
  for (const auto & capability : capabilities_) {
    if (capability->configure_node() != CallbackReturn::SUCCESS) {
      RCLCPP_FATAL(
        rclcpp::get_logger(name_), "Capability '%s' failed to configure, exiting.",
        capability->get_capability_name().c_str());
      rclcpp::shutdown();
      std::exit(1);
    }
  }
  for (const auto & capability : capabilities_) {
    if (capability->activate_node() != CallbackReturn::SUCCESS) {
      RCLCPP_FATAL(
        rclcpp::get_logger(name_), "Capability '%s' failed to activate, exiting.",
        capability->get_capability_name().c_str());
      rclcpp::shutdown();
      std::exit(1);
    }
  }
}

void Deployment::run()
{
  // One executor for every capability this robot hosts: each capability's
  // actual goal execution runs on its own worker thread (see
  // easyfleet_core::ActionServerBase), so sharing a single-threaded
  // executor here for the lightweight ROS callback dispatch (goal accept/
  // reject, feedback publish, heartbeats) doesn't starve the others.
  for (const auto & capability : capabilities_) {
    executor_.add_node(capability->get_node_base_interface());
  }
  easyfleet_core::spin_until_shutdown(executor_);
  for (const auto & capability : capabilities_) {
    executor_.remove_node(capability->get_node_base_interface());
  }

  for (auto it = capabilities_.rbegin(); it != capabilities_.rend(); ++it) {
    if ((*it)->get_current_state_id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      (*it)->deactivate_node();
    }
    if ((*it)->get_current_state_id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      (*it)->cleanup_node();
    }
  }
}

void Deployment::shutdown()
{
  // Reaches PRIMARY_STATE_FINALIZED from whichever state each capability is
  // in now -- without this, its destructor logs a (harmless, but noisy)
  // "not shut down" warning.
  for (auto it = capabilities_.rbegin(); it != capabilities_.rend(); ++it) {
    (*it)->shutdown_node();
  }
  rclcpp::shutdown();
}

}  // namespace easyfleet
