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

#ifndef EASYFLEET_MISSION_MANAGER__STATUS_MARKERS_HPP_
#define EASYFLEET_MISSION_MANAGER__STATUS_MARKERS_HPP_

#include <mutex>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace easyfleet_mission_manager
{

/// @brief Publishes a floating `TEXT_VIEW_FACING` marker above each robot's own
/// `base_link`, showing a short, human-readable line of what that robot is
/// currently doing (which capability, what state/result) -- so watching
/// the mission in RViz alone (no terminal) is enough for a non-expert to
/// follow along. Meant to be driven from mission control, not from a
/// capability itself: it has no notion of *why* a robot is doing
/// something, only whatever text the mission script hands it at each
/// meaningful transition.
///
/// One marker per robot, keyed by name (e.g. "robot_1"): each set_status()
/// call *replaces* that robot's previous text (same marker id), it never
/// accumulates markers. The marker's `frame_id` is `"<robot>/base_link"`
/// (a relative TF frame id, exactly matching how `easyfleet_core::Capability`
/// itself derives a robot's identity from its own namespace elsewhere in
/// this project) with the text positioned at that frame's origin plus a
/// fixed height, so it rides along with the robot as it moves and turns.
class StatusMarkerPublisher
{
public:
  /// @brief Constructs the marker publisher.
  /// @param node Node the marker publisher is created on. Kept as a
  ///   reference for this object's whole life (used later by the refresh
  ///   timer callback), so `node` must outlive this `StatusMarkerPublisher`
  ///   -- never null, unlike a pointer.
  /// @param topic Topic the MarkerArray is published on.
  /// @param height Height (m) above each robot's base_link the text floats at.
  /// @param text_size Marker `scale.z`: character height, **in world-space
  ///   meters**, like every other marker dimension -- not a screen-space
  ///   font size. How large it looks on screen is therefore also a
  ///   function of camera distance in RViz's 3D view, same as the robot
  ///   model itself: viewed close up (as this project's own Kobuki-sized
  ///   robots tend to be, being small), even a modest value here can fill
  ///   much of the view. Tune to taste for your own camera distance/robot
  ///   size; the default here favors staying legibly sized next to a
  ///   Kobuki at a typical close-in RViz view over matching some absolute
  ///   "normal" text size.
  explicit StatusMarkerPublisher(
    rclcpp::Node & node,
    const std::string & topic = "mission_status_markers",
    double height = 0.75,
    double text_size = 0.25)
  : node_(node),
    height_(height),
    text_size_(text_size)
  {
    // Transient-local: a subscriber (RViz) that joins after the mission has
    // already started still needs to see every robot's last known status
    // immediately, not wait for its next transition.
    pub_ = node_.create_publisher<visualization_msgs::msg::MarkerArray>(
      topic, rclcpp::QoS(10).reliable().transient_local());
    // set_status() is only called at mission phase transitions -- sometimes
    // minutes apart (kLongTimeout in this scenario's own mission script) --
    // but a marker's `header.stamp` must stay recent for RViz to keep
    // resolving "<robot>/base_link" and moving the text with the robot: a
    // stamp left over from the last set_status() call ages out of RViz's TF
    // buffer between updates, at which point RViz can no longer transform
    // the marker and it freezes/falls back to the fixed frame's origin.
    // Re-publishing every robot's current marker on a short timer instead
    // (rather than a message meant to be interpreted once) keeps every
    // stamp within a couple hundred ms of "now" at all times, independent
    // of how often the *text* itself actually changes.
    refresh_timer_ = node_.create_wall_timer(
      std::chrono::milliseconds(200), [this] {publish_all();});
  }

  /// @brief Sets `robot`'s current status text and publishes it immediately (the
  /// background refresh timer then keeps republishing it, and every other
  /// robot's last known text, with a fresh timestamp). Safe to call from
  /// multiple threads (e.g. one per robot, as a mission script's own
  /// parallel phases do).
  /// @param robot Robot identity the marker is shown above.
  /// @param text Status text to display.
  void set_status(const std::string & robot, const std::string & text)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      texts_[robot] = text;
      marker_id(robot);
    }
    publish_all();
  }

private:
  void publish_all()
  {
    visualization_msgs::msg::MarkerArray array;
    std::lock_guard<std::mutex> lock(mutex_);
    const rclcpp::Time stamp = node_.get_clock()->now();
    for (const auto & [robot, text] : texts_) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = robot + "/base_link";
      marker.header.stamp = stamp;
      marker.ns = "mission_status";
      marker.id = marker_ids_.at(robot);
      marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      marker.action = visualization_msgs::msg::Marker::ADD;
      // Tells RViz to re-transform this marker against the *latest*
      // available "<robot>/base_link" transform on every render frame,
      // instead of looking up the transform at this message's exact
      // header.stamp -- an exact-time lookup is racy under sim time (the
      // publishing node's clock sample can be a tick ahead of the latest
      // transform actually broadcast yet), which is what was producing
      // RViz's "Lookup would require extrapolation into the future" error
      // and the marker falling back to the fixed frame's origin.
      marker.frame_locked = true;
      marker.pose.position.z = height_;
      marker.pose.orientation.w = 1.0;
      marker.scale.z = text_size_;
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 1.0;
      marker.color.a = 1.0;
      marker.text = text;
      // Slightly longer than the refresh period, so a marker never visibly
      // blinks out between two refresh ticks.
      marker.lifetime = rclcpp::Duration(std::chrono::milliseconds(500));
      array.markers.push_back(marker);
    }
    if (!array.markers.empty()) {
      pub_->publish(array);
    }
  }

  // Requires mutex_ to already be held by the caller.
  int32_t marker_id(const std::string & robot)
  {
    auto it = marker_ids_.find(robot);
    if (it != marker_ids_.end()) {
      return it->second;
    }
    const int32_t id = next_id_++;
    marker_ids_.emplace(robot, id);
    return id;
  }

  rclcpp::Node & node_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr refresh_timer_;
  double height_;
  double text_size_;

  std::mutex mutex_;
  std::unordered_map<std::string, std::string> texts_;
  std::unordered_map<std::string, int32_t> marker_ids_;
  int32_t next_id_{0};
};

}  // namespace easyfleet_mission_manager

#endif  // EASYFLEET_MISSION_MANAGER__STATUS_MARKERS_HPP_
