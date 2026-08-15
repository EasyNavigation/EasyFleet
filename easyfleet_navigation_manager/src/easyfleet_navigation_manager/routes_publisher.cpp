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

#include "easyfleet_navigation_manager/routes_publisher.hpp"

#include <stdexcept>
#include <string>

#include "ament_index_cpp/get_package_share_path.hpp"
#include "easynav_routes_maps_manager/route_io.hpp"

namespace easyfleet
{

void RoutesPublisher::publish(rclcpp::Node & node)
{
  const std::string package_name = node.declare_parameter("routes.package", std::string());
  const std::string map_path_file = node.declare_parameter("routes.map_path_file", std::string());

  if (package_name.empty() || map_path_file.empty()) {
    throw std::runtime_error(
            "Parameters 'routes.package' and 'routes.map_path_file' must both be set.");
  }

  const auto pkgpath = ament_index_cpp::get_package_share_path(package_name);
  const std::string routes_path = (pkgpath / map_path_file).string();

  const auto routes = easynav::load_routes_from_yaml(routes_path);
  const auto msg = easynav::to_msg(routes);

  if (!pub_) {
    pub_ = node.create_publisher<easynav_routes_maps_manager::msg::RoutesMap>(
      "/global_routes", rclcpp::QoS(1).transient_local().reliable());
  }
  pub_->publish(msg);

  RCLCPP_INFO(
    node.get_logger(),
    "Publishing %zu route(s) from '%s' on /global_routes.",
    routes.size(), routes_path.c_str());
}

}  // namespace easyfleet
