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

#ifndef EASYFLEET_MISSION_MANAGER__CAPABILITY_DISCOVERY_HPP_
#define EASYFLEET_MISSION_MANAGER__CAPABILITY_DISCOVERY_HPP_

#include <chrono>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "easyfleet_mission_manager/capability_info.hpp"

namespace easyfleet_mission_manager
{

/// @brief Listens on `/capabilities` (retained CapabilityDescription messages) and
/// `/capabilities_status` (1 Hz CapabilityStatus heartbeats) for `window`,
/// and returns one `CapabilityInfo` per distinct `action_name` seen on
/// either topic -- this is the right key because two robots can offer the
/// same `capability` (e.g. "navigation") under different namespaces.
/// `node` must already be part of an executor that is being spun on another
/// thread: this function only subscribes and waits, it does not spin.
std::vector<CapabilityInfo> discover_capabilities(
  rclcpp::Node & node,
  std::chrono::milliseconds window = std::chrono::milliseconds(2500));

}  // namespace easyfleet_mission_manager

#endif  // EASYFLEET_MISSION_MANAGER__CAPABILITY_DISCOVERY_HPP_
