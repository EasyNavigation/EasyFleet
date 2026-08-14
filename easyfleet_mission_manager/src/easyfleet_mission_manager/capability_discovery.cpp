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

#include "easyfleet_mission_manager/capability_discovery.hpp"

#include <map>
#include <mutex>
#include <string>
#include <thread>

#include "easyfleet_interfaces/msg/capability_description.hpp"
#include "easyfleet_interfaces/msg/capability_status.hpp"

namespace easyfleet_mission_manager
{

std::vector<CapabilityInfo> discover_capabilities(
  rclcpp::Node & node,
  std::chrono::milliseconds window)
{
  using easyfleet_interfaces::msg::CapabilityDescription;
  using easyfleet_interfaces::msg::CapabilityStatus;

  std::mutex mutex;
  std::map<std::string, CapabilityInfo> by_action_name;

  auto capabilities_sub = node.create_subscription<CapabilityDescription>(
    "/capabilities", rclcpp::QoS(10).reliable().transient_local(),
    [&mutex, &by_action_name](const CapabilityDescription::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex);
      auto & info = by_action_name[msg->action_name];
      info.robot = msg->robot;
      info.capability = msg->capability;
      info.action_name = msg->action_name;
      info.description_json_raw = msg->description_json;
      try {
        info.description_json = nlohmann::json::parse(msg->description_json);
        info.description_json_valid = true;
      } catch (const nlohmann::json::parse_error &) {
        info.description_json_valid = false;
      }
    });

  auto status_sub = node.create_subscription<CapabilityStatus>(
    "/capabilities_status", rclcpp::QoS(10).reliable(),
    [&mutex, &by_action_name](const CapabilityStatus::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex);
      auto & info = by_action_name[msg->action_name];
      info.robot = msg->robot;
      info.capability = msg->capability;
      info.action_name = msg->action_name;
      info.active = true;
      info.busy = msg->busy;
    });

  std::this_thread::sleep_for(window);

  std::vector<CapabilityInfo> result;
  {
    std::lock_guard<std::mutex> lock(mutex);
    result.reserve(by_action_name.size());
    for (auto & [action_name, info] : by_action_name) {
      (void)action_name;
      result.push_back(std::move(info));
    }
  }
  return result;
}

}  // namespace easyfleet_mission_manager
