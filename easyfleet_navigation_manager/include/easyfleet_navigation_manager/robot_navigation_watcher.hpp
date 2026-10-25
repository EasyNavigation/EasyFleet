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

#ifndef EASYFLEET_NAVIGATION_MANAGER__ROBOT_NAVIGATION_WATCHER_HPP_
#define EASYFLEET_NAVIGATION_MANAGER__ROBOT_NAVIGATION_WATCHER_HPP_

#include <chrono>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>

#include "easynav_interfaces/msg/navigation_control.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "rclcpp/rclcpp.hpp"

namespace easyfleet
{

/// @brief Watches one robot's planned path and can pause/resume its
/// navigation.
///
/// Deliberately does *not* own a separate rclcpp::Node or
/// easynav::GoalManagerClient: those are designed for "one process
/// talks to one robot" (a fixed node namespace resolves the relative
/// "easynav_control"/"path" topic names), which doesn't fit
/// NavigationManagerNode watching *many* robots from a single process.
/// Instead, this class creates its publisher/subscriptions directly on
/// the given `node` -- once, in the constructor -- using each robot's
/// fully-qualified topic names (e.g. "/robot_1/easynav_control"), and
/// reimplements just the PAUSE/RESUME subset of GoalManager's control
/// protocol (see GoalManager.cpp's PAUSE/RESUME handling in
/// easynav_system): the SEND_GOAL/CANCEL machinery isn't needed here,
/// this class never sends a goal.
///
/// `node` ends up being serviced by whichever single executor the
/// caller spins it with -- no dedicated executor or thread of its own,
/// and (since every callback then runs on that one thread) no locking
/// either.
class RobotNavigationWatcher
{
public:
  /// @param node Node to create the publisher/subscriptions on. Must
  ///   outlive this watcher.
  /// @param robot_id Robot namespace, without a leading slash (e.g. "robot_1").
  /// @param planner_plugin_key The `planner_types` entry name configured
  ///   on that robot's PlannerNode (e.g. "simple") -- determines the path
  ///   topic to subscribe to.
  /// @param relay_tf If set, called with every message on the robot's own
  ///   /<robot_id>/tf topic (and `relay_tf_static` with /<robot_id>/tf_static),
  ///   so a fleet-wide view can merge them; unset, those topics are not
  ///   subscribed.
  /// @param relay_tf_static See `relay_tf`.
  RobotNavigationWatcher(
    rclcpp::Node & node, const std::string & robot_id, const std::string & planner_plugin_key,
    std::function<void(const tf2_msgs::msg::TFMessage &)> relay_tf = nullptr,
    std::function<void(const tf2_msgs::msg::TFMessage &)> relay_tf_static = nullptr);

  /// @brief Robot namespace this watcher was constructed for.
  [[nodiscard]] const std::string & robot_id() const {return robot_id_;}

  /// @brief Most recently received planned path. Empty (default-constructed
  /// nav_msgs::msg::Path) if none has arrived yet.
  [[nodiscard]] const nav_msgs::msg::Path & get_latest_path() const {return latest_path_;}

  /// @brief Age of the latest path, or `std::nullopt` if none has arrived.
  /// A robot whose process died keeps its last path forever otherwise.
  [[nodiscard]] std::optional<std::chrono::steady_clock::duration> path_age() const;

  /// @brief Latest pose reported in the robot's navigation FEEDBACK, if one
  /// arrived within `max_age`. More current than the front of the path,
  /// which is only as fresh as the last replanning.
  [[nodiscard]] std::optional<geometry_msgs::msg::PoseStamped> current_pose(
    std::chrono::steady_clock::duration max_age) const;

  /// @brief Ask this robot's GoalManager to pause its navigation.
  void pause();

  /// @brief Ask this robot's GoalManager to resume its navigation.
  void resume();

  /// @brief Send PAUSE unless the robot is already paused, idle, or a
  /// PAUSE is still waiting for its answer (re-sent after a second).
  void ensure_paused();

  /// @brief Send RESUME unless the robot is not paused or a RESUME is
  /// still waiting for its answer (re-sent after a second).
  void ensure_resumed();

  /// @brief Whether this robot's navigation is currently paused, by
  /// anyone, as last reported by its GoalManager. A new or preempting goal
  /// clears it.
  [[nodiscard]] bool is_paused() const {return paused_;}

  /// @brief Whether the robot has an active navigation goal, as last
  /// reported by its GoalManager. Assumed true until it says otherwise.
  [[nodiscard]] bool is_navigating() const {return navigating_;}

private:
  void send(uint8_t control_type);
  void on_control(easynav_interfaces::msg::NavigationControl::UniquePtr msg);
  /// Whether a request sent at `sent` may still be answered.
  static bool in_flight(const std::optional<std::chrono::steady_clock::time_point> & sent);

  rclcpp::Node & node_;
  std::string robot_id_;
  std::string id_;
  int64_t seq_ {0};

  rclcpp::Publisher<easynav_interfaces::msg::NavigationControl>::SharedPtr control_pub_;
  rclcpp::Subscription<easynav_interfaces::msg::NavigationControl>::SharedPtr control_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub_;

  nav_msgs::msg::Path latest_path_;
  std::optional<std::chrono::steady_clock::time_point> path_received_;
  std::optional<geometry_msgs::msg::PoseStamped> current_pose_;
  std::chrono::steady_clock::time_point pose_received_;
  bool paused_ {false};
  bool navigating_ {true};
  std::optional<std::chrono::steady_clock::time_point> pause_sent_;
  std::optional<std::chrono::steady_clock::time_point> resume_sent_;
};

}  // namespace easyfleet

#endif  // EASYFLEET_NAVIGATION_MANAGER__ROBOT_NAVIGATION_WATCHER_HPP_
