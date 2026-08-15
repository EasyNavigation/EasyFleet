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

#ifndef EASYFLEET_NAVIGATION_MANAGER__MAP_PUBLISHER_BASE_HPP_
#define EASYFLEET_NAVIGATION_MANAGER__MAP_PUBLISHER_BASE_HPP_

#include "rclcpp/rclcpp.hpp"

namespace easyfleet
{

/// @brief One "map type" NavigationManagerNode can publish on /global_map.
///
/// Costmap is the only implementation today (CostmapMapPublisher);
/// NavMap/Simple are meant to be added later as new implementations of
/// this same interface (registered in NavigationManagerNode's factory),
/// not by restructuring the manager itself. Not a pluginlib plugin --
/// this extensibility point is internal-only, no dynamic `.so` loading
/// needed for it.
class MapPublisherBase
{
public:
  virtual ~MapPublisherBase() = default;

  /// @brief Load the configured map and publish it once on /global_map.
  /// @param node Node to declare parameters on and to create the
  ///   publisher from. Must outlive this MapPublisherBase.
  virtual void publish(rclcpp::Node & node) = 0;
};

}  // namespace easyfleet

#endif  // EASYFLEET_NAVIGATION_MANAGER__MAP_PUBLISHER_BASE_HPP_
