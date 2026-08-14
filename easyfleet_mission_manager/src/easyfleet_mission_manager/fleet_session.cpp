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
#include <thread>
#include <vector>

#include "easyfleet_mission_manager/capability_discovery.hpp"
#include "easyfleet_mission_manager/mission_helpers.hpp"

namespace easyfleet
{

FleetSession::FleetSession()
: node_(std::make_shared<rclcpp::Node>("mission_control"))
{
  executor_.add_node(node_);
  spin_thread_ = easyfleet_mission_manager::spin_in_background(executor_);
  status_ = std::make_unique<easyfleet_mission_manager::StatusMarkerPublisher>(*node_);

  // Persistent (unlike discover_capabilities()'s own temporary
  // subscription) so is_alive() reflects the *current* liveness of every
  // robot's capabilities for the whole mission, not just a discovery-time
  // snapshot -- dispatched to whichever attached RobotHandle matches the
  // message's robot field.
  capability_status_sub_ = node_->create_subscription<easyfleet_interfaces::msg::CapabilityStatus>(
    "/capabilities_status", rclcpp::QoS(10).reliable(),
    [this](const easyfleet_interfaces::msg::CapabilityStatus::SharedPtr msg) {
      const auto now = std::chrono::steady_clock::now();
      auto robot = find_robot(msg->robot);
      if (robot) {
        robot->get().note_heartbeat(msg->capability, now);
      }
    });
}

FleetSession::~FleetSession()
{
  if (spin_thread_.joinable()) {
    executor_.cancel();
    spin_thread_.join();
  }
}

void FleetSession::add_robot(RobotHandle & robot)
{
  robots_.emplace_back(robot);
  robot.attach(*this);
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
  const auto all_capabilities = easyfleet_mission_manager::discover_capabilities(*node_, window);
  for (RobotHandle & robot : robots_) {
    std::vector<easyfleet_mission_manager::CapabilityInfo> this_robot;
    for (const auto & info : all_capabilities) {
      if (info.robot == robot.name()) {
        this_robot.push_back(info);
      }
    }
    robot.set_capabilities(std::move(this_robot));
  }
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
