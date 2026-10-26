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


#include "easyfleet_mission_manager/fleet_session.hpp"

#include <algorithm>
#include <chrono>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "easyfleet_mission_manager/mission_helpers.hpp"

namespace easyfleet
{

namespace
{

/// How long without a heartbeat before a capability counts as dead -- a
/// few times the 1 Hz heartbeat period, so one or two dropped messages
/// don't flip it.
constexpr std::chrono::seconds kAliveTimeout(3);

}  // namespace

FleetSession::FleetSession()
: node_(std::make_shared<rclcpp::Node>("mission_control"))
{
  status_ = std::make_unique<easyfleet_mission_manager::StatusMarkerPublisher>(*node_);

  using easyfleet_interfaces::msg::CapabilityDescription;
  using easyfleet_interfaces::msg::CapabilityStatus;

  // Both subscriptions live as long as the session: the registry tracks
  // capabilities that appear, restart or die at any point of the mission.
  capability_description_sub_ = node_->create_subscription<CapabilityDescription>(
    "/capabilities", rclcpp::QoS(10).reliable().transient_local(),
    [this](const CapabilityDescription::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(registry_mutex_);
      auto & info = registry_[msg->action_name].info;
      info.robot = msg->robot;
      info.capability = msg->capability;
      info.action_name = msg->action_name;
      info.description_json_raw = msg->description_json;
      try {
        info.description_json = nlohmann::json::parse(msg->description_json);
        // Only a JSON object is a valid description (same rule as Python).
        info.description_json_valid = info.description_json.is_object();
      } catch (const nlohmann::json::parse_error &) {
        info.description_json = nlohmann::json();
        info.description_json_valid = false;
      }
    });

  capability_status_sub_ = node_->create_subscription<CapabilityStatus>(
    "/capabilities_status", rclcpp::QoS(10).reliable(),
    [this](const CapabilityStatus::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(registry_mutex_);
      auto & entry = registry_[msg->action_name];
      entry.info.robot = msg->robot;
      entry.info.capability = msg->capability;
      entry.info.action_name = msg->action_name;
      entry.info.busy = msg->busy;
      entry.info.order_id = msg->order_id;
      entry.info.order_state = msg->order_state;
      entry.info.link_state = msg->link_state;
      entry.last_heartbeat = std::chrono::steady_clock::now();
    });

  robot_status_sub_ = node_->create_subscription<easyfleet_interfaces::msg::RobotStatus>(
    "/robot_status", rclcpp::QoS(10).reliable(),
    [this](const easyfleet_interfaces::msg::RobotStatus & msg) {on_robot_status(msg);});

  // The link every capability with a link_timeout watches.
  fleet_heartbeat_pub_ = node_->create_publisher<easyfleet_interfaces::msg::FleetHeartbeat>(
    "/fleet_heartbeat", rclcpp::QoS(1).best_effort());
  fleet_heartbeat_timer_ = node_->create_wall_timer(
    std::chrono::seconds(1), [this]() {
      easyfleet_interfaces::msg::FleetHeartbeat msg;
      msg.source = node_->get_fully_qualified_name();
      fleet_heartbeat_pub_->publish(msg);
    });

  executor_.add_node(node_);
  spin_thread_ = easyfleet_mission_manager::spin_in_background(executor_);
}

FleetSession::~FleetSession()
{
  if (spin_thread_.joinable()) {
    executor_.cancel();
    spin_thread_.join();
  }
  // Robots that outlive the session must not reach back into it.
  for (RobotHandle & robot : robots_) {
    robot.detach();
  }
}

void FleetSession::add_robot(RobotHandle & robot)
{
  if (robot.session_) {
    throw std::logic_error(
            "FleetSession::add_robot(\"" + robot.name() + "\"): this robot is already "
            "attached to a session.");
  }
  robots_.emplace_back(robot);
  robot.attach(*this);
}

void FleetSession::remove_robot(const RobotHandle & robot) noexcept
{
  robots_.erase(
    std::remove_if(
      robots_.begin(), robots_.end(),
      [&](const RobotHandle & other) {return &other == &robot;}),
    robots_.end());
}

const std::vector<std::reference_wrapper<RobotHandle>> & FleetSession::robots() const noexcept
{
  return robots_;
}

std::optional<std::reference_wrapper<RobotHandle>> FleetSession::find_robot(
  const std::string & name) const noexcept
{
  const auto it = std::find_if(
    robots_.begin(), robots_.end(),
    [&](const RobotHandle & robot) {return robot.name() == name;});
  if (it == robots_.end()) {
    return std::nullopt;
  }
  return *it;
}

void FleetSession::discover_capabilities(std::chrono::milliseconds window)
{
  spin_for(window);
}

bool FleetSession::alive(
  const RegistryEntry & entry, std::chrono::steady_clock::time_point now) const
{
  return entry.last_heartbeat && now - *entry.last_heartbeat < kAliveTimeout;
}

std::vector<easyfleet_mission_manager::CapabilityInfo> FleetSession::capabilities_of(
  const std::string & robot_name) const
{
  const auto now = std::chrono::steady_clock::now();
  std::vector<easyfleet_mission_manager::CapabilityInfo> result;
  std::lock_guard<std::mutex> lock(registry_mutex_);
  for (const auto & [action_name, entry] : registry_) {
    (void)action_name;
    if (entry.info.robot == robot_name) {
      result.push_back(entry.info);
      result.back().active = alive(entry, now);
    }
  }
  return result;
}

bool FleetSession::is_alive(
  const std::string & robot_name, const std::string & capability_type) const
{
  const auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(registry_mutex_);
  return std::any_of(
    registry_.begin(), registry_.end(),
    [&](const auto & item) {
      return item.second.info.robot == robot_name &&
             item.second.info.capability == capability_type && alive(item.second, now);
    });
}

void FleetSession::on_robot_status(const easyfleet_interfaces::msg::RobotStatus & msg)
{
  const auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(robot_status_mutex_);
  auto & entry = robot_status_[msg.robot];
  for (const auto & capability : msg.capabilities) {
    entry.capabilities[capability.action_name] = {capability, now};
  }
  if (msg.has_pose) {
    entry.pose = msg.pose;
  }
  if (msg.has_battery) {
    entry.battery = msg.battery;
  }
  if (rclcpp::Time(msg.stamp) > rclcpp::Time(entry.stamp)) {
    entry.stamp = msg.stamp;
  }
}

std::optional<easyfleet_interfaces::msg::RobotStatus> FleetSession::robot_status(
  const std::string & robot_name) const
{
  const auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(robot_status_mutex_);
  const auto it = robot_status_.find(robot_name);
  if (it == robot_status_.end()) {
    return std::nullopt;
  }
  easyfleet_interfaces::msg::RobotStatus msg;
  msg.robot = robot_name;
  msg.stamp = it->second.stamp;
  for (const auto & [action_name, report] : it->second.capabilities) {
    (void)action_name;
    if (now - report.second < kAliveTimeout) {
      msg.capabilities.push_back(report.first);
    }
  }
  if (it->second.pose) {
    msg.has_pose = true;
    msg.pose = *it->second.pose;
  }
  if (it->second.battery) {
    msg.has_battery = true;
    msg.battery = *it->second.battery;
  }
  return msg;
}

std::optional<easyfleet_interfaces::msg::OrderStatus> FleetSession::get_order(
  const std::string & robot_name, const std::string & order_id,
  std::chrono::milliseconds timeout)
{
  using easyfleet_interfaces::srv::GetOrder;
  std::vector<std::pair<rclcpp::Client<GetOrder>::SharedPtr,
    rclcpp::Client<GetOrder>::FutureAndRequestId>> answers;
  {
    std::lock_guard<std::mutex> lock(get_order_mutex_);
    for (const auto & info : capabilities_of(robot_name)) {
      auto & client = get_order_clients_[info.action_name];
      if (!client) {
        client = node_->create_client<GetOrder>(info.action_name + "/get_order");
      }
      if (!client->service_is_ready()) {
        // Just created, or the capability is gone: give discovery a moment.
        client->wait_for_service(std::chrono::milliseconds(200));
      }
      if (client->service_is_ready()) {
        auto request = std::make_shared<GetOrder::Request>();
        request->order_id = order_id;
        answers.emplace_back(client, client->async_send_request(request));
      }
    }
  }

  const auto deadline = std::chrono::steady_clock::now() + timeout;
  std::optional<easyfleet_interfaces::msg::OrderStatus> newest;
  for (auto & [client, answer] : answers) {
    if (answer.wait_until(deadline) != std::future_status::ready) {
      client->remove_pending_request(answer.request_id);
      continue;
    }
    const auto response = answer.get();
    if (response->found && (!newest || rclcpp::Time(response->order.accepted_stamp) >
      rclcpp::Time(newest->accepted_stamp)))
    {
      newest = response->order;
    }
  }
  return newest;
}

void FleetSession::spin_some()
{
  // The actual spinning happens continuously on spin_thread_ (started in
  // the constructor) -- calling executor_.spin_some() here too, from this
  // (the caller's) thread, would spin the same executor concurrently from
  // two threads at once, which rclcpp does not support. This just gives
  // that background thread a slice of time to make progress and lets
  // every added robot's in-flight run_capability() timeouts get checked.
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  for (RobotHandle & robot : robots_) {
    robot.check_timeouts();
  }
}

void FleetSession::spin_for(std::chrono::milliseconds duration)
{
  const auto deadline = std::chrono::steady_clock::now() + duration;
  while (std::chrono::steady_clock::now() < deadline) {
    spin_some();
  }
}

rclcpp::Node::SharedPtr FleetSession::node() const noexcept
{
  return node_;
}

easyfleet_mission_manager::StatusMarkerPublisher & FleetSession::status_marker_publisher()
{
  return *status_;
}

void FleetSession::shutdown()
{
  if (spin_thread_.joinable()) {
    executor_.cancel();
    spin_thread_.join();
  }
  rclcpp::shutdown();
}

}  // namespace easyfleet
