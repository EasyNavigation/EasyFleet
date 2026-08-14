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

#ifndef EASYFLEET_CORE__DETAIL__CAPABILITY_IMPL_HPP_
#define EASYFLEET_CORE__DETAIL__CAPABILITY_IMPL_HPP_

// Out-of-line member definitions for easyfleet_core::Capability<ActionServerT>.
// Included from the bottom of easyfleet_core/capability.hpp. Not meant to be
// included directly: templates cannot be compiled into easyfleet_core's
// .cpp/.so, so the implementation lives here to keep the class declaration
// in capability.hpp free of member bodies.

#include <algorithm>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>

#include "easyfleet_core/detail/namespace_utils.hpp"

namespace easyfleet_core
{

namespace detail
{
constexpr char kCapabilitiesFileParam[] = "capabilities_file";
}  // namespace detail

template<typename ActionServerT>
Capability<ActionServerT>::Capability(
  const std::string & capability_name,
  const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode(capability_name, options),
  capability_name_(capability_name)
{
  this->declare_parameter(detail::kCapabilitiesFileParam, std::string());
  action_server_ = std::make_shared<ActionServerT>(*this, capability_name_);

  robot_name_ = detail::strip_leading_slash(this->get_namespace());
  resolved_action_name_ = this->get_node_base_interface()->resolve_topic_or_service_name(
    action_server_->get_action_name(), /*is_service=*/ false);

  RCLCPP_INFO(
    this->get_logger(),
    "Starting capability '%s' on robot '%s' (action: %s).",
    capability_name_.c_str(),
    detail::robot_label(this->get_namespace()).c_str(),
    resolved_action_name_.c_str());
}

template<typename ActionServerT>
Capability<ActionServerT>::~Capability()
{
  stop_heartbeat();
}

template<typename ActionServerT>
typename Capability<ActionServerT>::ActionServerBaseT::SharedPtr
Capability<ActionServerT>::get_action_server() const noexcept
{
  return action_server_;
}

template<typename ActionServerT>
const std::string & Capability<ActionServerT>::get_capability_name() const noexcept
{
  return capability_name_;
}

template<typename ActionServerT>
const std::string & Capability<ActionServerT>::get_robot_name() const noexcept
{
  return robot_name_;
}

template<typename ActionServerT>
const std::string & Capability<ActionServerT>::get_resolved_action_name() const noexcept
{
  return resolved_action_name_;
}

template<typename ActionServerT>
typename Capability<ActionServerT>::CallbackReturn
Capability<ActionServerT>::on_configure(const rclcpp_lifecycle::State &)
{
  capabilities_pub_ = this->create_publisher<easyfleet_interfaces::msg::CapabilityDescription>(
    "/capabilities", rclcpp::QoS(1).reliable().transient_local());
  status_pub_ = this->create_publisher<easyfleet_interfaces::msg::CapabilityStatus>(
    "/capabilities_status", rclcpp::QoS(1).reliable());
  return CallbackReturn::SUCCESS;
}

template<typename ActionServerT>
typename Capability<ActionServerT>::CallbackReturn
Capability<ActionServerT>::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  // Activates capabilities_pub_/status_pub_ so publish() below actually
  // sends; a subclass overriding on_activate must call this or the
  // lifecycle-managed publishers stay muted.
  rclcpp_lifecycle::LifecycleNode::on_activate(previous_state);

  const auto file_path = this->get_parameter(detail::kCapabilitiesFileParam).as_string();
  if (file_path.empty()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Capability '%s': the '%s' parameter must be set to a capabilities JSON file path.",
      capability_name_.c_str(), detail::kCapabilitiesFileParam);
    return CallbackReturn::FAILURE;
  }

  std::ifstream file(file_path);
  if (!file.is_open()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Capability '%s': could not open capabilities file '%s'.",
      capability_name_.c_str(), file_path.c_str());
    return CallbackReturn::FAILURE;
  }
  std::ostringstream contents;
  contents << file.rdbuf();

  easyfleet_interfaces::msg::CapabilityDescription description_msg;
  description_msg.robot = robot_name_;
  description_msg.capability = capability_name_;
  description_msg.action_name = resolved_action_name_;
  description_msg.description_json = contents.str();
  capabilities_pub_->publish(description_msg);

  heartbeat_timer_ = this->create_wall_timer(
    std::chrono::seconds(1), std::bind(&Capability::publish_heartbeat, this));

  return CallbackReturn::SUCCESS;
}

template<typename ActionServerT>
typename Capability<ActionServerT>::CallbackReturn
Capability<ActionServerT>::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  stop_heartbeat();
  rclcpp_lifecycle::LifecycleNode::on_deactivate(previous_state);
  return CallbackReturn::SUCCESS;
}

template<typename ActionServerT>
typename Capability<ActionServerT>::CallbackReturn
Capability<ActionServerT>::on_cleanup(const rclcpp_lifecycle::State &)
{
  stop_heartbeat();
  capabilities_pub_.reset();
  status_pub_.reset();
  return CallbackReturn::SUCCESS;
}

template<typename ActionServerT>
typename Capability<ActionServerT>::CallbackReturn
Capability<ActionServerT>::on_shutdown(const rclcpp_lifecycle::State &)
{
  stop_heartbeat();
  capabilities_pub_.reset();
  status_pub_.reset();
  return CallbackReturn::SUCCESS;
}

template<typename ActionServerT>
void Capability<ActionServerT>::publish_heartbeat()
{
  easyfleet_interfaces::msg::CapabilityStatus msg;
  msg.robot = robot_name_;
  msg.capability = capability_name_;
  msg.action_name = resolved_action_name_;
  msg.busy = action_server_->is_active();
  status_pub_->publish(msg);
}

template<typename ActionServerT>
void Capability<ActionServerT>::stop_heartbeat()
{
  if (heartbeat_timer_) {
    heartbeat_timer_->cancel();
    heartbeat_timer_.reset();
  }
}

// -- CapabilityNodeBase -- each of these calls the corresponding
// LifecycleNode transition (unambiguously -- see capability.hpp for why
// these are named *_node() instead of colliding with LifecycleNode's own
// configure()/activate()/etc.) and surfaces just the CallbackReturn, which
// is all a Robot/Deployment driving this instance through CapabilityNodeBase
// alone needs to know.

template<typename ActionServerT>
typename Capability<ActionServerT>::CallbackReturn
Capability<ActionServerT>::configure_node()
{
  CallbackReturn cb_return_code;
  this->configure(cb_return_code);
  return cb_return_code;
}

template<typename ActionServerT>
typename Capability<ActionServerT>::CallbackReturn
Capability<ActionServerT>::activate_node()
{
  CallbackReturn cb_return_code;
  this->activate(cb_return_code);
  return cb_return_code;
}

template<typename ActionServerT>
typename Capability<ActionServerT>::CallbackReturn
Capability<ActionServerT>::deactivate_node()
{
  CallbackReturn cb_return_code;
  this->deactivate(cb_return_code);
  return cb_return_code;
}

template<typename ActionServerT>
typename Capability<ActionServerT>::CallbackReturn
Capability<ActionServerT>::cleanup_node()
{
  CallbackReturn cb_return_code;
  this->cleanup(cb_return_code);
  return cb_return_code;
}

template<typename ActionServerT>
typename Capability<ActionServerT>::CallbackReturn
Capability<ActionServerT>::shutdown_node()
{
  CallbackReturn cb_return_code;
  this->shutdown(cb_return_code);
  return cb_return_code;
}

template<typename ActionServerT>
uint8_t Capability<ActionServerT>::get_current_state_id() const
{
  return rclcpp_lifecycle::LifecycleNode::get_current_state().id();
}

template<typename ActionServerT>
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
Capability<ActionServerT>::get_node_base_interface()
{
  return rclcpp_lifecycle::LifecycleNode::get_node_base_interface();
}

}  // namespace easyfleet_core

#endif  // EASYFLEET_CORE__DETAIL__CAPABILITY_IMPL_HPP_
