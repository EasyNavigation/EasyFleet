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

#include "easyfleet_navigation_manager/navigation_manager_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <functional>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "easyfleet_navigation_manager/costmap_map_publisher.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace easyfleet
{

namespace
{

/// Registry of map_type name -> MapPublisherBase factory. Adding a new
/// map type (NavMap, Simple, ...) is one new MapPublisherBase
/// implementation plus one new entry here -- not a restructuring of
/// NavigationManagerNode itself.
const std::map<std::string, std::function<std::unique_ptr<MapPublisherBase>()>> &
map_publisher_factories()
{
  static const std::map<std::string, std::function<std::unique_ptr<MapPublisherBase>()>>
  factories{
    {"costmap", [] {return std::make_unique<CostmapMapPublisher>();}},
  };
  return factories;
}

}  // namespace

NavigationManagerNode::NavigationManagerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("navigation_manager_node", options)
{
  const std::string map_type = this->declare_parameter("map_type", std::string("costmap"));

  const auto & factories = map_publisher_factories();
  const auto it = factories.find(map_type);
  if (it == factories.end()) {
    RCLCPP_FATAL(
      get_logger(),
      "Unknown map_type '%s' -- only 'costmap' is supported today, exiting.", map_type.c_str());
    rclcpp::shutdown();
    std::exit(1);
  }

  map_publisher_ = it->second();
  map_publisher_->publish(*this);
  routes_publisher_.publish(*this);

  map_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  planner_plugin_key_ = this->declare_parameter("robots.planner_plugin_key", std::string("simple"));
  const double check_rate_hz = this->declare_parameter("conflict.check_rate_hz", 2.0);
  conflict_params_.lookahead_distance_m =
    this->declare_parameter("conflict.lookahead_distance_m", conflict_params_.lookahead_distance_m);
  conflict_params_.path_conflict_distance_m = this->declare_parameter(
    "conflict.path_conflict_distance_m", conflict_params_.path_conflict_distance_m);
  conflict_params_.proximity_radius_m =
    this->declare_parameter("conflict.proximity_radius_m", conflict_params_.proximity_radius_m);

  const auto static_list =
    this->declare_parameter("robots.static_list", std::vector<std::string>());

  if (!static_list.empty()) {
    // Fixed robot set: no capability tracking needed at all.
    for (const auto & robot_id : static_list) {
      RCLCPP_INFO(get_logger(), "Conflict monitor: watching robot '%s'", robot_id.c_str());
      watchers_.push_back(
        std::make_unique<RobotNavigationWatcher>(*this, robot_id, planner_plugin_key_));
      publish_robot_map_tf(robot_id);
    }
  } else {
    // Dynamic robot set: track it via plain subscription callbacks (no
    // extra rclcpp::Node/executor/thread -- see the class doc comment),
    // reconciled on reconcile_timer_.
    robot_staleness_sec_ = this->declare_parameter("robots.staleness_sec", robot_staleness_sec_);
    const double rescan_rate_hz = this->declare_parameter("robots.rescan_rate_hz", 1.0);

    capabilities_status_sub_ = this->create_subscription<easyfleet_interfaces::msg::CapabilityStatus>(
      "/capabilities_status", rclcpp::QoS(10).reliable(),
      [this](easyfleet_interfaces::msg::CapabilityStatus::UniquePtr msg) {
        if (!msg->robot.empty()) {
          last_heartbeat_by_robot_[msg->robot] = this->now();
        }
      });

    reconcile_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / rescan_rate_hz),
      std::bind(&NavigationManagerNode::reconcile_watchers, this));
  }

  conflict_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / check_rate_hz),
    std::bind(&NavigationManagerNode::check_conflicts, this));
}

std::vector<std::string>
NavigationManagerNode::get_watched_robots() const
{
  std::vector<std::string> robots;
  robots.reserve(watchers_.size());
  for (const auto & watcher : watchers_) {
    robots.push_back(watcher->robot_id());
  }
  return robots;
}

void
NavigationManagerNode::publish_robot_map_tf(const std::string & robot_id)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = this->now();
  transform.header.frame_id = "map";
  transform.child_frame_id = robot_id + "/map";
  transform.transform.rotation.w = 1.0;

  map_tf_broadcaster_->sendTransform(transform);
}

void
NavigationManagerNode::reconcile_watchers()
{
  const auto now = this->now();

  std::set<std::string> alive_robots;
  for (const auto & [robot_id, last_heartbeat] : last_heartbeat_by_robot_) {
    if ((now - last_heartbeat).seconds() <= robot_staleness_sec_) {
      alive_robots.insert(robot_id);
    }
  }

  for (const auto & robot_id : alive_robots) {
    const bool already_watched = std::any_of(
      watchers_.begin(), watchers_.end(),
      [&robot_id](const auto & watcher) {return watcher->robot_id() == robot_id;});
    if (!already_watched) {
      RCLCPP_INFO(get_logger(), "Conflict monitor: watching robot '%s'", robot_id.c_str());
      watchers_.push_back(
        std::make_unique<RobotNavigationWatcher>(*this, robot_id, planner_plugin_key_));
      publish_robot_map_tf(robot_id);
    }
  }

  for (auto watcher_it = watchers_.begin(); watcher_it != watchers_.end(); ) {
    if (alive_robots.count((*watcher_it)->robot_id()) == 0) {
      RCLCPP_INFO(
        get_logger(), "Conflict monitor: '%s' stopped sending /capabilities_status; "
        "no longer watching it", (*watcher_it)->robot_id().c_str());
      currently_paused_.erase((*watcher_it)->robot_id());
      watcher_it = watchers_.erase(watcher_it);
    } else {
      ++watcher_it;
    }
  }
}

void
NavigationManagerNode::check_conflicts()
{
  std::map<std::string, nav_msgs::msg::Path> paths;
  for (const auto & watcher : watchers_) {
    paths[watcher->robot_id()] = watcher->get_latest_path();
  }

  // One entry per pair close enough to be worth explaining (both the
  // "at risk" and the "close but no risk" cases) -- run with
  // `--ros-args --log-level navigation_manager_node:=debug` to see them
  // live, including which robot is already stationary and yielding to
  // the other.
  std::vector<ConflictDecision> decisions;
  const auto to_pause = compute_robots_to_pause(
    paths, conflict_params_, currently_paused_, &decisions);

  for (const auto & decision : decisions) {
    if (decision.in_conflict) {
      RCLCPP_DEBUG(
        get_logger(), "[risk] %s <-> %s: %s",
        decision.robot_a.c_str(), decision.robot_b.c_str(), decision.reason.c_str());
    } else {
      RCLCPP_DEBUG(
        get_logger(), "[no risk] %s <-> %s: %s",
        decision.robot_a.c_str(), decision.robot_b.c_str(), decision.reason.c_str());
    }
  }

  for (const auto & watcher : watchers_) {
    const bool should_pause = to_pause.count(watcher->robot_id()) > 0;
    const bool already_paused = currently_paused_.count(watcher->robot_id()) > 0;

    if (should_pause && !already_paused) {
      RCLCPP_INFO(
        get_logger(), "Pausing '%s' to avoid an imminent navigation conflict",
        watcher->robot_id().c_str());
      watcher->pause();
      currently_paused_.insert(watcher->robot_id());
    } else if (!should_pause && already_paused) {
      RCLCPP_INFO(
        get_logger(), "Resuming '%s': navigation conflict cleared",
        watcher->robot_id().c_str());
      watcher->resume();
      currently_paused_.erase(watcher->robot_id());
    }
  }
}

}  // namespace easyfleet
