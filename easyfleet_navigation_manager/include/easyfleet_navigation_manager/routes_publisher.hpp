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

#include <memory>
#include <string>

#include "easynav_routes_maps_manager/msg/routes_map.hpp"
#include "easynav_routes_maps_manager/routes_map.hpp"
#include "interactive_markers/interactive_marker_server.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace easyfleet
{

/// @brief Publishes a RoutesMap on /global_routes, and keeps it live-editable.
///
/// Loaded the same way easynav_routes_maps_manager's own
/// RoutesMapsManager loads its local `map_path_file`, via
/// `easynav::load_routes_from_yaml()` -- every robot's own "routes"
/// maps_manager_node need only remap its "incoming_routes" topic to
/// /global_routes to receive this centrally instead of loading its own
/// copy (see easyfleet_navigation_manager's README). Unlike maps, only
/// one route representation exists, so this is a single concrete class
/// rather than a MapPublisherBase-style extensibility point.
///
/// Reproduces RoutesMapsManager's own interactive-marker editing
/// mechanism exactly (same marker layout, same per-segment edit-mode
/// toggle, same move/rotate/add/remove controls -- see
/// RoutesMapsManager::publish_interactive_markers()/handle_interactive_feedback()
/// in easynav_routes_maps_manager), with one difference in what a route
/// *edit* (as opposed to just toggling edit mode) does afterwards: here
/// it also republishes the updated RoutesMap on /global_routes -- so
/// every robot picks up the edit immediately -- and re-saves it to the
/// same YAML file it was loaded from (via
/// `easynav::save_routes_to_yaml()`), so edits made live in RViz survive
/// a restart. RoutesMapsManager itself only does the equivalent of the
/// second part on demand, via its own `save_routes` service.
///
/// The visualization MarkerArray on /global_routes_markers is a
/// deliberate exception to the "publish once, rely on transient_local"
/// approach used everywhere else in this class: a late-joining
/// subscriber reliably getting the retained
/// visualization_msgs/MarkerArray sample via durability-service replay
/// turned out NOT to be guaranteed in practice here (verified
/// independently of this class, with a minimal throwaway
/// publisher/subscriber pair outside this codebase -- a plain
/// std_msgs/String with the exact same QoS delivered its retained
/// sample to a late joiner every time, MarkerArray did not). A topic is
/// one or the other, not both: something that needs periodic
/// republishing to reach late joiners isn't really transient_local, so
/// /global_routes_markers is volatile and kept alive by a low-rate
/// timer (`routes.markers_republish_rate_hz`) instead -- matching the
/// rate nav2's own costmap/marker publishers already default to for the
/// same class of topic.
/// interactive_markers::InteractiveMarkerServer sidesteps the whole
/// question for its own topics: it doesn't rely on durability-service
/// replay at all, it actively resends the full marker set whenever a
/// new subscriber connects. /global_routes itself (a plain custom
/// message, no embedded image/mesh types, and genuinely only
/// republished in response to an edit -- never periodically) was
/// confirmed to not have this problem, so it's left as transient_local,
/// publish-on-change only, with no timer.
class RoutesPublisher
{
public:
  /// @brief Read "routes.package"/"routes.map_path_file" from `node`,
  /// load the routes they name, publish them once on /global_routes
  /// (QoS: depth 1, transient_local, reliable, per the user's own spec
  /// for this topic), and set up the visualization + interactive-marker
  /// publishers so the routes can be edited live from RViz. Also starts
  /// a low-rate timer that keeps re-publishing the visualization
  /// MarkerArray (see the class doc comment for why).
  /// @param node Node to declare parameters on and to create the
  ///   publishers/interactive marker server from. Must outlive this
  ///   RoutesPublisher.
  /// @throw std::runtime_error if the parameters are unset or the named
  ///   package doesn't exist.
  void publish(rclcpp::Node & node);

private:
  /// @brief Publish the current routes_ on /global_routes.
  void publish_routes_msg();

  /// @brief Publish the current routes_ as a MarkerArray for visualization.
  void publish_routes_markers();

  /// @brief (Re)build the interactive markers (per-segment edit-mode
  /// toggle, and while in edit mode, draggable start/end endpoints with
  /// add/remove-segment controls) from the current routes_.
  void publish_interactive_markers();

  /// @brief Handle interactive marker feedback: toggling edit mode,
  /// dragging an endpoint, or the add/remove-segment button controls.
  /// A geometry-changing edit (drag/add/remove, as opposed to just
  /// toggling edit mode) also republishes on /global_routes and
  /// re-saves to map_path_.
  void handle_interactive_feedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback);

  /// @brief Recompute next_route_id_ from whatever is currently in
  /// routes_, so newly created routes (the interactive marker's
  /// "add_segment" control) get unique IDs that don't clash with
  /// existing ones.
  void recompute_next_route_id();

  /// @brief Persist routes_ to map_path_ (easynav::save_routes_to_yaml());
  /// logs a warning on failure rather than throwing, since this runs
  /// from an interactive-marker feedback callback.
  void save_routes();

  rclcpp::Node * node_ {nullptr};
  std::string map_path_;
  easynav::RoutesMap routes_;
  int next_route_id_ {0};

  rclcpp::Publisher<easynav_routes_maps_manager::msg::RoutesMap>::SharedPtr pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> imarker_server_;
  rclcpp::TimerBase::SharedPtr markers_republish_timer_;
};

}  // namespace easyfleet

#endif  // EASYFLEET_NAVIGATION_MANAGER__ROUTES_PUBLISHER_HPP_
