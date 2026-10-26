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
#include "easyfleet_core/detail/param_utils.hpp"

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
  const auto prefix = detail::sanitize_parameter_name(capability_name_);
  order_history_param_ = prefix + ".order_history";
  link_timeout_param_ = prefix + ".link_timeout";
  on_link_lost_param_ = prefix + ".on_link_lost";
  this->declare_parameter(order_history_param_, 20);
  this->declare_parameter(link_timeout_param_, 0.0);
  this->declare_parameter(on_link_lost_param_, std::string("continue"));
  order_history_ = std::make_unique<OrderHistory>(
    static_cast<std::size_t>(std::max<int64_t>(1,
      this->get_parameter(order_history_param_).as_int())));

  action_server_ = std::make_shared<ActionServerT>(*this, capability_name_);
  // Goals are only accepted while the capability is ACTIVE.
  action_server_->disable_goals();

  robot_name_ = detail::strip_leading_slash(this->get_namespace());
  resolved_action_name_ = this->get_node_base_interface()->resolve_topic_or_service_name(
    action_server_->get_action_name(), /*is_service=*/ false);

  RCLCPP_INFO(
    this->get_logger(),
    "Starting capability '%s' on robot '%s' (action: %s).",
    capability_name_.c_str(),
    detail::robot_label(this->get_namespace()).c_str(),
    resolved_action_name_.c_str());

  action_server_->set_goal_accepted_callback(
    [this](const rclcpp_action::GoalUUID & uuid, const std::string & order_id) {
      const auto id = order_history_->accepted(
        uuid, order_id, resolved_action_name_, this->now());
      RCLCPP_INFO(
        this->get_logger(), "Capability '%s': order '%s' accepted.",
        capability_name_.c_str(), id.c_str());
    });
  // The action server's own status topic: the states of every goal it holds.
  goal_status_sub_ = this->create_subscription<action_msgs::msg::GoalStatusArray>(
    resolved_action_name_ + "/_action/status",
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
    [this](const action_msgs::msg::GoalStatusArray & msg) {
      order_history_->update(msg, this->now());
    });
}

template<typename ActionServerT>
Capability<ActionServerT>::~Capability()
{
  action_server_->set_goal_accepted_callback(nullptr);
  stop_heartbeat();
  // Join the worker while the derived action server still exists.
  action_server_->stop();
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
const OrderHistory & Capability<ActionServerT>::get_order_history() const noexcept
{
  return *order_history_;
}

template<typename ActionServerT>
easyfleet_interfaces::msg::CapabilityStatus Capability<ActionServerT>::get_status() const
{
  easyfleet_interfaces::msg::CapabilityStatus msg;
  msg.robot = robot_name_;
  msg.capability = capability_name_;
  msg.action_name = resolved_action_name_;
  msg.busy = action_server_->is_active();
  if (const auto order = order_history_->latest()) {
    msg.order_id = order->order_id;
    msg.order_state = order->state;
  }
  msg.link_state = link_watchdog_.state();
  return msg;
}

template<typename ActionServerT>
typename Capability<ActionServerT>::CallbackReturn
Capability<ActionServerT>::on_configure(const rclcpp_lifecycle::State &)
{
  const auto on_link_lost = this->get_parameter(on_link_lost_param_).as_string();
  if (on_link_lost != "continue" && on_link_lost != "cancel") {
    RCLCPP_ERROR(
      this->get_logger(), "Capability '%s': '%s' must be \"continue\" or \"cancel\", not \"%s\".",
      capability_name_.c_str(), on_link_lost_param_.c_str(), on_link_lost.c_str());
    return CallbackReturn::FAILURE;
  }
  cancel_on_link_lost_ = on_link_lost == "cancel";

  capabilities_pub_ = this->create_publisher<easyfleet_interfaces::msg::CapabilityDescription>(
    "/capabilities", rclcpp::QoS(1).reliable().transient_local());
  status_pub_ = this->create_publisher<easyfleet_interfaces::msg::CapabilityStatus>(
    "/capabilities_status", rclcpp::QoS(1).reliable());
  get_order_srv_ = this->create_service<easyfleet_interfaces::srv::GetOrder>(
    "~/get_order",
    [this](
      const std::shared_ptr<easyfleet_interfaces::srv::GetOrder::Request> request,
      std::shared_ptr<easyfleet_interfaces::srv::GetOrder::Response> response) {
      on_get_order(request, response);
    });
  return CallbackReturn::SUCCESS;
}

template<typename ActionServerT>
void Capability<ActionServerT>::on_get_order(
  const std::shared_ptr<easyfleet_interfaces::srv::GetOrder::Request> request,
  std::shared_ptr<easyfleet_interfaces::srv::GetOrder::Response> response)
{
  if (const auto order = order_history_->find(request->order_id)) {
    response->found = true;
    response->order = *order;
  }
}

template<typename ActionServerT>
typename Capability<ActionServerT>::CallbackReturn
Capability<ActionServerT>::on_activate(const rclcpp_lifecycle::State & previous_state)
{
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

  // Activates capabilities_pub_/status_pub_ so publish() below actually
  // sends, only once the file is known to be valid; a subclass overriding
  // on_activate must call this or the lifecycle-managed publishers stay muted.
  rclcpp_lifecycle::LifecycleNode::on_activate(previous_state);

  easyfleet_interfaces::msg::CapabilityDescription description_msg;
  description_msg.robot = robot_name_;
  description_msg.capability = capability_name_;
  description_msg.action_name = resolved_action_name_;
  description_msg.description_json = contents.str();
  capabilities_pub_->publish(description_msg);

  heartbeat_timer_ = this->create_wall_timer(
    std::chrono::seconds(1), std::bind(&Capability::publish_heartbeat, this));

  const double link_timeout = this->get_parameter(link_timeout_param_).as_double();
  link_watchdog_.reset(link_timeout);
  if (link_watchdog_.monitored()) {
    fleet_heartbeat_sub_ = this->create_subscription<easyfleet_interfaces::msg::FleetHeartbeat>(
      "/fleet_heartbeat", rclcpp::QoS(1).best_effort(),
      [this](const easyfleet_interfaces::msg::FleetHeartbeat &) {
        link_watchdog_.heard(LinkWatchdog::Clock::now());
      });
    // A few checks per timeout, at most one a second.
    const auto period = std::chrono::duration<double>(std::clamp(link_timeout / 4.0, 0.01, 1.0));
    link_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&Capability::check_link, this));
    RCLCPP_INFO(
      this->get_logger(),
        "Capability '%s': watching /fleet_heartbeat (timeout %.2f s, on loss: %s).",
      capability_name_.c_str(), link_timeout, cancel_on_link_lost_ ? "cancel" : "continue");
  }
  action_server_->enable_goals();

  return CallbackReturn::SUCCESS;
}

template<typename ActionServerT>
typename Capability<ActionServerT>::CallbackReturn
Capability<ActionServerT>::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  // Rejects new goals and waits for the running one to stop.
  action_server_->disable_goals();
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
  get_order_srv_.reset();
  return CallbackReturn::SUCCESS;
}

template<typename ActionServerT>
typename Capability<ActionServerT>::CallbackReturn
Capability<ActionServerT>::on_shutdown(const rclcpp_lifecycle::State &)
{
  action_server_->disable_goals();
  stop_heartbeat();
  capabilities_pub_.reset();
  status_pub_.reset();
  get_order_srv_.reset();
  return CallbackReturn::SUCCESS;
}

template<typename ActionServerT>
void Capability<ActionServerT>::publish_heartbeat()
{
  status_pub_->publish(get_status());
}

template<typename ActionServerT>
void Capability<ActionServerT>::check_link()
{
  const auto before = link_watchdog_.state();
  const auto now = link_watchdog_.update(LinkWatchdog::Clock::now());
  if (now == before) {
    return;
  }
  using easyfleet_interfaces::msg::CapabilityStatus;
  if (now == CapabilityStatus::LINK_LOST) {
    const bool busy = action_server_->is_active();
    RCLCPP_WARN(
      this->get_logger(), "Capability '%s': link to the fleet lost (no /fleet_heartbeat)%s.",
      capability_name_.c_str(),
      cancel_on_link_lost_ && busy ? ", stopping the running goal" : "");
    if (cancel_on_link_lost_) {
      action_server_->halt_active_goal();
    }
  } else if (now == CapabilityStatus::LINK_OK && before == CapabilityStatus::LINK_LOST) {
    RCLCPP_INFO(
      this->get_logger(), "Capability '%s': link to the fleet recovered.",
        capability_name_.c_str());
  }
}

template<typename ActionServerT>
void Capability<ActionServerT>::stop_heartbeat()
{
  if (heartbeat_timer_) {
    heartbeat_timer_->cancel();
    heartbeat_timer_.reset();
  }
  if (link_timer_) {
    link_timer_->cancel();
    link_timer_.reset();
  }
  fleet_heartbeat_sub_.reset();
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
