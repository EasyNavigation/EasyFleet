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

#ifndef EASYFLEET_NAVIGATION_MANAGER__COSTMAP_MAP_PUBLISHER_HPP_
#define EASYFLEET_NAVIGATION_MANAGER__COSTMAP_MAP_PUBLISHER_HPP_

#include "easyfleet_navigation_manager/map_publisher_base.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace easyfleet
{

/// @brief Publishes an OccupancyGrid map on /global_map.
///
/// Loaded the same way easynav_costmap_maps_manager's own
/// CostmapMapsManager loads its local `map_path_file`, via
/// `easynav::loadMapFromYaml()` -- every robot's own "costmap"
/// maps_manager_node need only remap its "incoming_map" topic to
/// /global_map to receive this centrally instead of loading its own
/// copy (see easyfleet_navigation_manager's README).
class CostmapMapPublisher : public MapPublisherBase
{
public:
  /// @brief Read "map.package"/"map.map_path_file" from `node`, load the
  /// map they name, and publish it once on /global_map (QoS: depth 1,
  /// transient_local, reliable -- matching what "incoming_map"
  /// subscribers expect).
  /// @param node Node to declare parameters on and to create the
  ///   publisher from. Must outlive this CostmapMapPublisher.
  /// @throw std::runtime_error if the parameters are unset, the named
  ///   package doesn't exist, or the map YAML/image fails to load.
  void publish(rclcpp::Node & node) override;

private:
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
};

}  // namespace easyfleet

#endif  // EASYFLEET_NAVIGATION_MANAGER__COSTMAP_MAP_PUBLISHER_HPP_
