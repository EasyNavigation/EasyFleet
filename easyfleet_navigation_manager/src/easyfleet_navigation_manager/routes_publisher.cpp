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

#include <chrono>
#include <stdexcept>
#include <string>

#include "ament_index_cpp/get_package_share_path.hpp"
#include "easynav_routes_maps_manager/route_io.hpp"

namespace easyfleet
{

namespace
{
// The fleet-wide global map/routes are unprefixed, matching
// CostmapMapPublisher's own hardcoded "map" frame_id in this same
// package -- unlike a per-robot RoutesMapsManager plugin, there is no
// per-robot tf_prefix to apply here.
constexpr char kFrameId[] = "map";
}  // namespace

void RoutesPublisher::publish(rclcpp::Node & node)
{
  node_ = &node;

  const std::string package_name = node.declare_parameter("routes.package", std::string());
  const std::string map_path_file = node.declare_parameter("routes.map_path_file", std::string());

  if (package_name.empty() || map_path_file.empty()) {
    throw std::runtime_error(
            "Parameters 'routes.package' and 'routes.map_path_file' must both be set.");
  }

  const auto pkgpath = ament_index_cpp::get_package_share_path(package_name);
  map_path_ = (pkgpath / map_path_file).string();

  routes_ = easynav::load_routes_from_yaml(map_path_);
  recompute_next_route_id();

  if (!pub_) {
    pub_ = node.create_publisher<easynav_routes_maps_manager::msg::RoutesMap>(
      "/global_routes", rclcpp::QoS(1).transient_local().reliable());
  }
  if (!markers_pub_) {
    // Deliberately volatile, not transient_local: this topic is kept
    // alive by the periodic republish timer set up below, not by
    // durability-service replay -- a topic is one or the other, not
    // both (see the class doc comment for why transient_local alone
    // isn't used here).
    markers_pub_ = node.create_publisher<visualization_msgs::msg::MarkerArray>(
      "/global_routes_markers", rclcpp::QoS(10).durability_volatile().reliable());
  }
  if (!imarker_server_) {
    // NodePtr is an unconstrained template parameter (any type exposing
    // get_node_*_interface()), so a raw pointer works fine here -- no
    // need for a real shared_ptr to `node`, which publish() only ever
    // receives as a reference.
    imarker_server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
      "global_routes_imarkers", &node);
  }

  publish_routes_msg();
  publish_routes_markers();
  publish_interactive_markers();

  if (!markers_republish_timer_) {
    const double republish_rate_hz =
      node.declare_parameter("routes.markers_republish_rate_hz", 1.0);
    if (republish_rate_hz <= 0.0) {
      throw std::runtime_error("Parameter 'routes.markers_republish_rate_hz' must be > 0");
    }
    markers_republish_timer_ = node.create_wall_timer(
      std::chrono::duration<double>(1.0 / republish_rate_hz),
      [this] {
        publish_routes_markers();
        // Not just applyChanges(): it only ever publishes when there are
        // pending insert()/erase()/setPose() calls since the last time
        // it ran (confirmed empirically -- calling it alone, with no
        // actual change, produces zero traffic on
        // .../imarkers/update). RViz's InteractiveMarkerClient expects
        // to keep hearing from the server even when nothing has
        // changed, and resets its connection ("Server not available
        // while running, resetting") if it doesn't -- so this
        // re-inserts every marker (a real, if redundant, change from
        // the library's point of view) to force genuine keep-alive
        // traffic. Already happens today on every actual edit with no
        // ill effect on in-progress drags (the endpoint-drag feedback
        // handler below rebuilds on every POSE_UPDATE tick, not just on
        // mouse-up), so doing it periodically at rest is no riskier.
        publish_interactive_markers();
      });
  }

  RCLCPP_INFO(
    node.get_logger(),
    "Publishing %zu route(s) from '%s' on /global_routes (live-editable via interactive markers).",
    routes_.size(), map_path_.c_str());
}

void RoutesPublisher::publish_routes_msg()
{
  pub_->publish(easynav::to_msg(routes_));
}

void RoutesPublisher::recompute_next_route_id()
{
  next_route_id_ = 0;
  for (const auto & seg : routes_) {
    if (seg.id.rfind("route", 0) == 0 && seg.id.size() > 5) {
      try {
        const int n = std::stoi(seg.id.substr(5));
        if (n >= next_route_id_) {
          next_route_id_ = n + 1;
        }
      } catch (...) {
        // Non-numeric suffixes are ignored.
      }
    }
  }
}

void RoutesPublisher::save_routes()
{
  std::string error_message;
  if (!easynav::save_routes_to_yaml(map_path_, routes_, error_message)) {
    RCLCPP_WARN(
      node_->get_logger(), "Failed to save routes to '%s': %s",
      map_path_.c_str(), error_message.c_str());
  }
}

void RoutesPublisher::publish_routes_markers()
{
  if (!markers_pub_) {
    return;
  }

  visualization_msgs::msg::MarkerArray array;

  // First, delete all previous markers in our namespaces so that
  // removed segments do not leave orphaned markers behind. id is
  // irrelevant to DELETEALL itself, but RViz's "Duplicate Marker Check"
  // flags any (ns, id) pair that appears twice in the same MarkerArray
  // message regardless of action type -- the default-constructed id (0)
  // would collide with the first ADD marker in the same ns (whose id
  // counter below also starts at 0), so give these an id no ADD marker
  // ever uses.
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = kFrameId;
    m.action = visualization_msgs::msg::Marker::DELETEALL;
    m.id = -1;

    m.ns = "routes_line";
    array.markers.push_back(m);

    m.ns = "routes_arrow";
    array.markers.push_back(m);
  }

  int id = 0;
  for (const auto & seg : routes_) {
    // Line between start and end
    visualization_msgs::msg::Marker line;
    line.header.frame_id = kFrameId;
    line.ns = "routes_line";
    line.id = id++;
    line.type = visualization_msgs::msg::Marker::LINE_LIST;
    line.action = visualization_msgs::msg::Marker::ADD;

    line.scale.x = 0.05;  // line width

    line.color.r = 0.0f;
    line.color.g = 1.0f;
    line.color.b = 0.0f;
    line.color.a = 1.0f;

    line.points.resize(2);
    line.points[0].x = seg.start.position.x;
    line.points[0].y = seg.start.position.y;
    line.points[0].z = seg.start.position.z;

    line.points[1].x = seg.end.position.x;
    line.points[1].y = seg.end.position.y;
    line.points[1].z = seg.end.position.z;

    array.markers.push_back(line);

    // Arrow for start orientation (same style as end)
    visualization_msgs::msg::Marker start_arrow;
    start_arrow.header.frame_id = kFrameId;
    start_arrow.ns = "routes_arrow";
    start_arrow.id = id++;
    start_arrow.type = visualization_msgs::msg::Marker::ARROW;
    start_arrow.action = visualization_msgs::msg::Marker::ADD;
    start_arrow.pose = seg.start;
    start_arrow.scale.x = 0.25;   // shaft length
    start_arrow.scale.y = 0.05;   // shaft diameter
    start_arrow.scale.z = 0.1;   // head diameter
    start_arrow.color.r = 1.0f;
    start_arrow.color.g = 1.0f;
    start_arrow.color.b = 0.0f;
    start_arrow.color.a = 0.9f;
    array.markers.push_back(start_arrow);

    // Arrow for end orientation (same style)
    visualization_msgs::msg::Marker end_arrow;
    end_arrow.header.frame_id = kFrameId;
    end_arrow.ns = "routes_arrow";
    end_arrow.id = id++;
    end_arrow.type = visualization_msgs::msg::Marker::ARROW;
    end_arrow.action = visualization_msgs::msg::Marker::ADD;
    end_arrow.pose = seg.end;
    end_arrow.scale.x = 0.25;
    end_arrow.scale.y = 0.05;
    end_arrow.scale.z = 0.1;
    end_arrow.color.r = 1.0f;
    end_arrow.color.g = 1.0f;
    end_arrow.color.b = 0.0f;
    end_arrow.color.a = 0.9f;
    array.markers.push_back(end_arrow);
  }

  markers_pub_->publish(array);
}

void RoutesPublisher::publish_interactive_markers()
{
  if (!imarker_server_) {
    return;
  }
  imarker_server_->clear();

  for (const auto & seg : routes_) {
    // Per-segment toggle cube (red in normal mode, green in edit mode).
    visualization_msgs::msg::InteractiveMarker mode_marker;
    mode_marker.header.frame_id = kFrameId;
    mode_marker.name = seg.id + "_mode";
    mode_marker.scale = 1.0;

    // Mid-point between start and end (no vertical offset)
    mode_marker.pose.position.x = 0.5 * (seg.start.position.x + seg.end.position.x);
    mode_marker.pose.position.y = 0.5 * (seg.start.position.y + seg.end.position.y);
    mode_marker.pose.position.z = 0.5 * (seg.start.position.z + seg.end.position.z);

    visualization_msgs::msg::InteractiveMarkerControl mode_ctrl;
    mode_ctrl.name = "toggle_edit";
    mode_ctrl.interaction_mode =
      visualization_msgs::msg::InteractiveMarkerControl::BUTTON;
    mode_ctrl.always_visible = true;

    visualization_msgs::msg::Marker cube;
    cube.type = visualization_msgs::msg::Marker::CUBE;
    cube.scale.x = 0.15;
    cube.scale.y = 0.15;
    cube.scale.z = 0.15;
    if (!seg.edit_mode) {
      // Red in normal mode
      cube.color.r = 1.0f;
      cube.color.g = 0.0f;
      cube.color.b = 0.0f;
    } else {
      // Green in edit mode
      cube.color.r = 0.0f;
      cube.color.g = 1.0f;
      cube.color.b = 0.0f;
    }
    cube.color.a = 0.9f;

    // Text label for the toggle control
    visualization_msgs::msg::Marker toggle_text;
    toggle_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    toggle_text.text = "toggle edit";
    toggle_text.scale.z = 0.1;  // font height
    toggle_text.color.r = 1.0f;
    toggle_text.color.g = 1.0f;
    toggle_text.color.b = 1.0f;
    toggle_text.color.a = 1.0f;
    toggle_text.pose.position.z = 0.3;  // slightly above the cube

    mode_ctrl.markers.push_back(cube);
    mode_ctrl.markers.push_back(toggle_text);

    mode_marker.controls.push_back(mode_ctrl);

    imarker_server_->insert(
      mode_marker,
      std::bind(
        &RoutesPublisher::handle_interactive_feedback,
        this,
        std::placeholders::_1));

    if (!seg.edit_mode) {
      // In normal mode we only show the cube and skip endpoint controls.
      continue;
    }

    visualization_msgs::msg::InteractiveMarker start_marker;
    start_marker.header.frame_id = kFrameId;
    start_marker.name = seg.id + "_start";
    start_marker.description = "Route " + seg.id + " start";
    start_marker.pose = seg.start;
    start_marker.scale = 1.0;

    visualization_msgs::msg::InteractiveMarker end_marker;
    end_marker.header.frame_id = kFrameId;
    end_marker.name = seg.id + "_end";
    end_marker.description = "Route " + seg.id + " end";
    end_marker.pose = seg.end;
    end_marker.scale = 1.0;

    auto add_controls = [](visualization_msgs::msg::InteractiveMarker & marker) {
        visualization_msgs::msg::InteractiveMarkerControl control;

        // Move along X
        control.orientation.w = 1.0;
        control.orientation.x = 1.0;
        control.orientation.y = 0.0;
        control.orientation.z = 0.0;
        control.name = "move_x";
        control.interaction_mode =
          visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
        marker.controls.push_back(control);

        // Move along Y
        control.orientation.x = 0.0;
        control.orientation.y = 1.0;
        control.name = "move_y";
        marker.controls.push_back(control);

        // Move along Z
        control.orientation.y = 0.0;
        control.orientation.z = 1.0;
        control.name = "move_z";
        marker.controls.push_back(control);

        // Rotate around Z (yaw), orientation as in interactive_markers examples
        control.interaction_mode =
          visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
        control.orientation.w = 1.0;
        control.orientation.x = 0.0;
        control.orientation.y = 1.0;
        control.orientation.z = 0.0;
        control.name = "rotate_z";
        marker.controls.push_back(control);

        // Button control to add a new segment starting from this endpoint
        visualization_msgs::msg::InteractiveMarkerControl add_ctrl;
        add_ctrl.name = "add_segment";
        add_ctrl.interaction_mode =
          visualization_msgs::msg::InteractiveMarkerControl::BUTTON;
        add_ctrl.always_visible = true;

        visualization_msgs::msg::Marker add_marker;
        add_marker.type = visualization_msgs::msg::Marker::SPHERE;
        add_marker.scale.x = 0.2;
        add_marker.scale.y = 0.2;
        add_marker.scale.z = 0.2;
        add_marker.color.r = 1.0f;
        add_marker.color.g = 0.5f;
        add_marker.color.b = 0.0f;
        add_marker.color.a = 0.9f;

        visualization_msgs::msg::Marker add_text;
        add_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        add_text.text = "add";
        add_text.scale.z = 0.1;
        add_text.color.r = 1.0f;
        add_text.color.g = 1.0f;
        add_text.color.b = 1.0f;
        add_text.color.a = 1.0f;
        add_text.pose.position.z = 0.4;

        add_ctrl.markers.push_back(add_marker);
        add_ctrl.markers.push_back(add_text);

        marker.controls.push_back(add_ctrl);

        // Button control to remove the segment this endpoint belongs to
        visualization_msgs::msg::InteractiveMarkerControl remove_ctrl;
        remove_ctrl.name = "remove_segment";
        remove_ctrl.interaction_mode =
          visualization_msgs::msg::InteractiveMarkerControl::BUTTON;
        remove_ctrl.always_visible = true;

        visualization_msgs::msg::Marker remove_marker;
        remove_marker.type = visualization_msgs::msg::Marker::SPHERE;
        // Place the red sphere 1 m above the endpoint so that it
        // does not overlap with the orange "add" sphere.
        remove_marker.pose.position.z = 1.0;
        remove_marker.scale.x = 0.15;
        remove_marker.scale.y = 0.15;
        remove_marker.scale.z = 0.15;
        remove_marker.color.r = 1.0f;
        remove_marker.color.g = 0.0f;
        remove_marker.color.b = 0.0f;
        remove_marker.color.a = 0.9f;

        visualization_msgs::msg::Marker remove_text;
        remove_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        remove_text.text = "remove";
        remove_text.scale.z = 0.1;
        remove_text.color.r = 1.0f;
        remove_text.color.g = 1.0f;
        remove_text.color.b = 1.0f;
        remove_text.color.a = 1.0f;
        remove_text.pose.position.z = 1.3;

        remove_ctrl.markers.push_back(remove_marker);
        remove_ctrl.markers.push_back(remove_text);

        marker.controls.push_back(remove_ctrl);
      };

    add_controls(start_marker);
    add_controls(end_marker);

    imarker_server_->insert(
      start_marker,
      std::bind(
        &RoutesPublisher::handle_interactive_feedback,
        this,
        std::placeholders::_1));
    imarker_server_->insert(
      end_marker,
      std::bind(
        &RoutesPublisher::handle_interactive_feedback,
        this,
        std::placeholders::_1));
  }

  imarker_server_->applyChanges();
}

void RoutesPublisher::handle_interactive_feedback(
  const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
{
  if (!feedback) {
    return;
  }

  const auto & name = feedback->marker_name;

  // Toggle per-segment edit mode when clicking the central cube. Purely
  // a UI-editor-state change (RouteSegment::edit_mode is never carried
  // over the wire or saved to YAML), so this does not republish on
  // /global_routes or save to file.
  if (feedback->control_name == "toggle_edit" &&
    (feedback->event_type ==
    visualization_msgs::msg::InteractiveMarkerFeedback::BUTTON_CLICK ||
    feedback->event_type ==
    visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP))
  {
    // name is <id>_mode
    const auto underscore_pos = name.rfind("_");
    if (underscore_pos != std::string::npos) {
      const auto base_id = name.substr(0, underscore_pos);
      for (auto & seg : routes_) {
        if (seg.id == base_id) {
          seg.edit_mode = !seg.edit_mode;
          break;
        }
      }
      publish_interactive_markers();
    }
    return;
  }

  // Creation of a new segment from this endpoint
  if (feedback->control_name == "add_segment" &&
    (feedback->event_type ==
    visualization_msgs::msg::InteractiveMarkerFeedback::BUTTON_CLICK ||
    feedback->event_type ==
    visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP))
  {
    // Compute forward direction from marker orientation (assume x-forward).
    const auto & p = feedback->pose.position;
    const auto & q = feedback->pose.orientation;

    const double qx = q.x;
    const double qy = q.y;
    const double qz = q.z;
    const double qw = q.w;

    // Forward vector in world coordinates: q * (1,0,0) * q^{-1}
    const double fx = 2.0 * (qx * qx + qw * qw) - 1.0;
    const double fy = 2.0 * (qx * qy + qw * qz);
    const double fz = 2.0 * (qx * qz - qw * qy);

    const double length = 2.0;  // meters

    easynav::RouteSegment new_seg;
    // New unique segment id based on a monotonic counter
    new_seg.id = "route" + std::to_string(next_route_id_++);

    new_seg.start = feedback->pose;
    new_seg.end = feedback->pose;
    new_seg.end.position.x = p.x + fx * length;
    new_seg.end.position.y = p.y + fy * length;
    new_seg.end.position.z = p.z + fz * length;

    routes_.push_back(new_seg);

    publish_routes_msg();
    publish_routes_markers();
    publish_interactive_markers();
    save_routes();
    return;
  }

  // Removal of the segment this endpoint belongs to
  if (feedback->control_name == "remove_segment" &&
    (feedback->event_type ==
    visualization_msgs::msg::InteractiveMarkerFeedback::BUTTON_CLICK ||
    feedback->event_type ==
    visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP))
  {
    // marker_name is either <id>_start or <id>_end
    const auto underscore_pos = name.rfind("_");
    if (underscore_pos != std::string::npos) {
      const auto base_id = name.substr(0, underscore_pos);
      for (auto it = routes_.begin(); it != routes_.end(); ++it) {
        if (it->id == base_id) {
          routes_.erase(it);
          break;
        }
      }
      publish_routes_msg();
      publish_routes_markers();
      publish_interactive_markers();
      save_routes();
    }
    return;
  }

  // Endpoint dragged/rotated: update that segment's start or end pose.
  for (auto & seg : routes_) {
    if (name == seg.id + "_start") {
      seg.start = feedback->pose;
      publish_routes_msg();
      publish_routes_markers();
      publish_interactive_markers();
      save_routes();
      return;
    } else if (name == seg.id + "_end") {
      seg.end = feedback->pose;
      publish_routes_msg();
      publish_routes_markers();
      publish_interactive_markers();
      save_routes();
      return;
    }
  }
}

}  // namespace easyfleet
