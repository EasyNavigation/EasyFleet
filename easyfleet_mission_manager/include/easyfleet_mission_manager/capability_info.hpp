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

#ifndef EASYFLEET_MISSION_MANAGER__CAPABILITY_INFO_HPP_
#define EASYFLEET_MISSION_MANAGER__CAPABILITY_INFO_HPP_

#include <string>

#include "nlohmann/json.hpp"

namespace easyfleet_mission_manager
{

/// @brief Everything known about one running capability instance, gathered from
/// its `/capabilities` (easyfleet_interfaces/CapabilityDescription) and
/// `/capabilities_status` (easyfleet_interfaces/CapabilityStatus)
/// messages. `action_name` is the unique identity: two robots can both
/// offer a `capability` named "navigation", but each has its own
/// `action_name` (e.g. "/robot1/navigation" vs "/robot2/navigation").
struct CapabilityInfo
{
  /// @brief Robot identity this capability was announced under (e.g. "robot1").
  std::string robot;
  /// @brief Capability type (e.g. "navigation").
  std::string capability;
  /// @brief Fully-qualified action name (e.g. "/robot1/navigation").
  std::string action_name;
  /// @brief Raw JSON text of the capability description, as published.
  std::string description_json_raw;
  /// @brief `description_json_raw`, parsed -- empty if parsing failed.
  nlohmann::json description_json;
  /// @brief Whether `description_json_raw` parsed successfully as JSON.
  bool description_json_valid{false};
  /// @brief Whether this capability was announced on /capabilities at all, at
  /// discovery time.
  bool active{false};
  /// @brief True while the capability has a goal currently executing (from the
  /// latest CapabilityStatus heartbeat seen). Meaningless if `active` is
  /// false: a capability that isn't publishing heartbeats at all can't be
  /// "busy" or "idle", it's just gone.
  bool busy{false};
};

/// @brief Pretty-prints the full capability description (requirements, effects,
/// parameters, ...) to the terminal.
void print_capability_info(const CapabilityInfo & info);

/// @brief Prints a single summary line, e.g. for a "known capabilities" list.
void print_capability_summary_line(const CapabilityInfo & info);

}  // namespace easyfleet_mission_manager

#endif  // EASYFLEET_MISSION_MANAGER__CAPABILITY_INFO_HPP_
