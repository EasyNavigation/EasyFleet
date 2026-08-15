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

#include "easyfleet_navigation_manager/costmap_map_publisher.hpp"

#include <stdexcept>
#include <string>

#include "ament_index_cpp/get_package_share_path.hpp"
#include "easynav_costmap_maps_manager/map_io.hpp"

namespace easyfleet
{

void CostmapMapPublisher::publish(rclcpp::Node & node)
{
  const std::string package_name = node.declare_parameter("map.package", std::string());
  const std::string map_path_file = node.declare_parameter("map.map_path_file", std::string());

  if (package_name.empty() || map_path_file.empty()) {
    throw std::runtime_error(
            "Parameters 'map.package' and 'map.map_path_file' must both be set for "
            "map_type 'costmap'.");
  }

  const auto pkgpath = ament_index_cpp::get_package_share_path(package_name);
  const std::string map_path = (pkgpath / map_path_file).string();

  nav_msgs::msg::OccupancyGrid grid;
  if (easynav::loadMapFromYaml(map_path, grid) != easynav::LOAD_MAP_SUCCESS) {
    throw std::runtime_error("Could not load map YAML file: " + map_path);
  }
  grid.header.frame_id = "map";
  grid.header.stamp = node.now();

  if (!pub_) {
    pub_ = node.create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/global_map", rclcpp::QoS(1).transient_local().reliable());
  }
  pub_->publish(grid);

  RCLCPP_INFO(
    node.get_logger(),
    "Publishing global map from '%s' on /global_map (%u x %u @ %.3f m/cell).",
    map_path.c_str(), grid.info.width, grid.info.height,
    static_cast<double>(grid.info.resolution));
}

}  // namespace easyfleet
