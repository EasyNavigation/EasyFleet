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

#ifndef EASYFLEET_NAVIGATION_MANAGER__NAVIGATION_MANAGER_NODE_HPP_
#define EASYFLEET_NAVIGATION_MANAGER__NAVIGATION_MANAGER_NODE_HPP_

#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "easyfleet_interfaces/msg/capability_status.hpp"
#include "easyfleet_navigation_manager/conflict_detector.hpp"
#include "easyfleet_navigation_manager/map_publisher_base.hpp"
#include "easyfleet_navigation_manager/robot_navigation_watcher.hpp"
#include "easyfleet_navigation_manager/routes_publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"

namespace easyfleet
{

/**
 * @class NavigationManagerNode
 * @brief Fleet-wide navigation manager.
 *
 * Publishes one shared map on /global_map and one shared route set on
 * /global_routes at construction, so every robot's own EasyNav instance
 * can be pointed at the same data (by remapping its own
 * "incoming_map"/"incoming_routes" topics) instead of each robot
 * loading its own local copy. Both publishers use transient_local
 * durability, so a robot that starts (and subscribes) after this node
 * already published still receives the retained last message -- no
 * periodic re-publish timer is needed, but the process must keep
 * running (spinning) for that durability guarantee to hold, since it's
 * tied to the publisher's lifetime, not persisted to disk.
 *
 * Also continuously monitors every robot's planned navigation path (one
 * RobotNavigationWatcher per robot, all sharing this node -- see that
 * class'es own doc comment for why no extra rclcpp::Node/executor/thread
 * is used) and pauses/resumes robots when compute_robots_to_pause() (see
 * conflict_detector.hpp) detects an imminent conflict.
 *
 * The robot set being watched can also change over time: unless
 * `robots.static_list` is set, this node subscribes to
 * `/capabilities_status` (see easyfleet_interfaces/CapabilityStatus --
 * "its presence, not its content, is the alive signal") and, on
 * reconcile_watchers()'s own timer, adds a RobotNavigationWatcher for
 * any newly-heard-from robot and drops the watcher for any robot whose
 * heartbeat has gone stale for longer than `robots.staleness_sec`. This
 * -- like the conflict-check below -- is plain subscription callbacks
 * plus a periodic timer on this one node; no extra rclcpp::Node,
 * executor, or thread is created anywhere in this class.
 *
 * Every robot's own EasyNav instance applies its `tf_prefix` to its
 * local "map" frame (see RTTFBuffer::set_tf_info() in easynav_common),
 * so e.g. robot_1's own map frame is literally "robot_1/map" -- while
 * /global_map and /global_routes (published above) are both stamped
 * with the plain, unprefixed "map" frame. For the two to actually line
 * up in one TF tree, this node also broadcasts one static, identity
 * transform "map" -> "<robot_id>/map" per watched robot (via
 * publish_robot_map_tf(), on /tf_static through a single
 * tf2_ros::StaticTransformBroadcaster) as soon as that robot starts
 * being watched -- every robot in this fleet is assumed to share the
 * exact same map, only their pose *within* it differs, so identity is
 * the correct transform (see e.g. GpsLocalizer's own identical
 * map->odom identity broadcast for the same kind of "these two frames
 * are just aliases of each other" case).
 *
 * ROS parameters:
 * - `map_type` (string, default `"costmap"`): which MapPublisherBase
 *   implementation to use. Only `"costmap"` exists today; any other
 *   value is a fatal configuration error.
 * - `map.package` / `map.map_path_file`: passed through to the chosen
 *   MapPublisherBase.
 * - `routes.package` / `routes.map_path_file`: passed through to
 *   RoutesPublisher.
 * - `robots.static_list` (string array, default empty): explicit, fixed
 *   list of robot namespaces to monitor (e.g. ["robot_1", "robot_2"]).
 *   When empty, the robot set is instead tracked dynamically via
 *   `/capabilities_status`, as described above.
 * - `robots.staleness_sec` (double, default 5.0): how long without a
 *   `/capabilities_status` heartbeat before a robot's watcher is
 *   dropped. Only used when `robots.static_list` is empty.
 * - `robots.rescan_rate_hz` (double, default 1.0): how often
 *   reconcile_watchers() runs. Only used when `robots.static_list` is
 *   empty.
 * - `robots.planner_plugin_key` (string, default "simple"): the
 *   `planner_types` entry name each robot's PlannerNode is configured
 *   with -- determines the per-robot path topic to subscribe to
 *   (`planner_node/<key>/path`).
 * - `conflict.check_rate_hz` (double, default 2.0): how often the
 *   conflict-detection timer runs.
 * - `conflict.lookahead_distance_m`, `conflict.path_conflict_distance_m`,
 *   `conflict.proximity_radius_m`: see ConflictParams (conflict_detector.hpp).
 */
class NavigationManagerNode : public rclcpp::Node
{
public:
  /// @brief Declares parameters, builds the configured MapPublisherBase,
  /// publishes both the map and the routes once, sets up the robot set
  /// (static list, or dynamic capability tracking), and starts the
  /// conflict-monitoring timer.
  /// @param options Standard rclcpp node options.
  explicit NavigationManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// @brief Robots currently being watched (for testing and tools).
  [[nodiscard]] std::vector<std::string> get_watched_robots() const;

private:
  /// @brief Add or remove RobotNavigationWatchers so `watchers_` matches
  /// the robots that have sent a `/capabilities_status` heartbeat within
  /// `robots.staleness_sec`. Called from a timer; a no-op (never
  /// scheduled) when `robots.static_list` is set.
  void reconcile_watchers();

  /// @brief Periodic conflict-check, called from conflict_timer_: reads
  /// every watcher's latest path, calls compute_robots_to_pause(), and
  /// calls pause()/resume() on whichever watchers newly entered/left
  /// that set.
  void check_conflicts();

  /// @brief Broadcast a static, identity transform from the fleet-wide
  /// "map" frame to `robot_id`'s own "<robot_id>/map" frame, on
  /// /tf_static. Safe to call more than once for the same robot_id
  /// (StaticTransformBroadcaster keys its retained set by
  /// child_frame_id, so a repeat call just re-sends the same transform).
  void publish_robot_map_tf(const std::string & robot_id);

  std::unique_ptr<MapPublisherBase> map_publisher_;
  RoutesPublisher routes_publisher_;

  std::string planner_plugin_key_;
  ConflictParams conflict_params_;
  std::vector<std::unique_ptr<RobotNavigationWatcher>> watchers_;
  std::set<std::string> currently_paused_;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> map_tf_broadcaster_;

  /// @brief Only used when `robots.static_list` is empty.
  rclcpp::Subscription<easyfleet_interfaces::msg::CapabilityStatus>::SharedPtr
    capabilities_status_sub_;
  /// @brief robot -> time of the last /capabilities_status heartbeat
  /// naming it, across all of that robot's capabilities.
  std::map<std::string, rclcpp::Time> last_heartbeat_by_robot_;
  double robot_staleness_sec_ {5.0};

  rclcpp::TimerBase::SharedPtr reconcile_timer_;
  rclcpp::TimerBase::SharedPtr conflict_timer_;
};

}  // namespace easyfleet

#endif  // EASYFLEET_NAVIGATION_MANAGER__NAVIGATION_MANAGER_NODE_HPP_
