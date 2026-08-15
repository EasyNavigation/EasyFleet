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

#ifndef EASYFLEET_NAVIGATION_MANAGER__ROUTES_PUBLISHER_HPP_
#define EASYFLEET_NAVIGATION_MANAGER__ROUTES_PUBLISHER_HPP_

#include "easynav_routes_maps_manager/msg/routes_map.hpp"
#include "rclcpp/rclcpp.hpp"

namespace easyfleet
{

/// @brief Publishes a RoutesMap on /global_routes.
///
/// Loaded the same way easynav_routes_maps_manager's own
/// RoutesMapsManager loads its local `map_path_file`, via
/// `easynav::load_routes_from_yaml()` -- every robot's own "routes"
/// maps_manager_node need only remap its "incoming_routes" topic to
/// /global_routes to receive this centrally instead of loading its own
/// copy (see easyfleet_navigation_manager's README). Unlike maps, only
/// one route representation exists, so this is a single concrete class
/// rather than a MapPublisherBase-style extensibility point.
class RoutesPublisher
{
public:
  /// @brief Read "routes.package"/"routes.map_path_file" from `node`,
  /// load the routes they name, and publish them once on /global_routes
  /// (QoS: depth 1, transient_local, reliable, per the user's own spec
  /// for this topic).
  /// @param node Node to declare parameters on and to create the
  ///   publisher from. Must outlive this RoutesPublisher.
  /// @throw std::runtime_error if the parameters are unset or the named
  ///   package doesn't exist.
  void publish(rclcpp::Node & node);

private:
  rclcpp::Publisher<easynav_routes_maps_manager::msg::RoutesMap>::SharedPtr pub_;
};

}  // namespace easyfleet

#endif  // EASYFLEET_NAVIGATION_MANAGER__ROUTES_PUBLISHER_HPP_
