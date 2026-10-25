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

  map_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);

  relay_tf_ = this->declare_parameter("robots.relay_tf", false);
  for (const auto & frame :
    this->declare_parameter("robots.relay_tf_frames", std::vector<std::string>()))
  {
    relay_tf_frames_.insert(frame);
  }
  const double relay_rate_hz = this->declare_parameter("robots.relay_tf_rate_hz", 0.0);
  relay_tf_period_ = relay_rate_hz > 0.0 ?
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(1.0 / relay_rate_hz)) :
    std::chrono::steady_clock::duration::zero();
  if (relay_tf_) {
    tf_pub_ = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(100));
  }

  share_obstacles_ = this->declare_parameter("robots.share_obstacles", false);
  obstacles_topic_ = this->declare_parameter("robots.obstacles_topic", obstacles_topic_);
  obstacles_base_frame_ =
    this->declare_parameter("robots.obstacles_base_frame", obstacles_base_frame_);
  obstacle_params_.radius_m =
    this->declare_parameter("robots.obstacle_radius_m", obstacle_params_.radius_m);
  obstacle_params_.height_m =
    this->declare_parameter("robots.obstacle_height_m", obstacle_params_.height_m);
  obstacles_pose_timeout_sec_ = this->declare_parameter(
    "robots.obstacles_pose_timeout_sec", obstacles_pose_timeout_sec_);
  const double obstacles_rate_hz = this->declare_parameter("robots.obstacles_rate_hz", 10.0);
  if (share_obstacles_) {
    if (obstacles_rate_hz <= 0.0) {
      RCLCPP_FATAL(
        get_logger(),
        "Parameter 'robots.obstacles_rate_hz' must be > 0 (got %.3f), exiting.",
        obstacles_rate_hz);
      rclcpp::shutdown();
      std::exit(1);
    }
    obstacles_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / obstacles_rate_hz),
      std::bind(&NavigationManagerNode::publish_fleet_obstacles, this));
  }

  planner_plugin_key_ = this->declare_parameter("robots.planner_plugin_key", std::string("simple"));
  const double check_rate_hz = this->declare_parameter("conflict.check_rate_hz", 2.0);
  if (check_rate_hz <= 0.0) {
    RCLCPP_FATAL(
      get_logger(),
      "Parameter 'conflict.check_rate_hz' must be > 0 (got %.3f), exiting.", check_rate_hz);
    rclcpp::shutdown();
    std::exit(1);
  }
  conflict_params_.lookahead_distance_m =
    this->declare_parameter("conflict.lookahead_distance_m", conflict_params_.lookahead_distance_m);
  conflict_params_.path_conflict_distance_m = this->declare_parameter(
    "conflict.path_conflict_distance_m", conflict_params_.path_conflict_distance_m);
  conflict_params_.proximity_radius_m =
    this->declare_parameter("conflict.proximity_radius_m", conflict_params_.proximity_radius_m);
  conflict_params_.resume_hysteresis_factor = this->declare_parameter(
    "conflict.resume_hysteresis_factor", conflict_params_.resume_hysteresis_factor);
  path_timeout_sec_ = this->declare_parameter("conflict.path_timeout_sec", path_timeout_sec_);
  pose_timeout_sec_ = this->declare_parameter("conflict.pose_timeout_sec", pose_timeout_sec_);
  max_pause_sec_ = this->declare_parameter("conflict.max_pause_sec", max_pause_sec_);

  const auto static_list =
    this->declare_parameter("robots.static_list", std::vector<std::string>());

  if (!static_list.empty()) {
    // Fixed robot set: no capability tracking needed at all.
    for (const auto & robot_id : static_list) {
      RCLCPP_INFO(get_logger(), "Conflict monitor: watching robot '%s'", robot_id.c_str());
      watchers_.push_back(
        make_watcher(robot_id));
      publish_robot_map_tf(robot_id);
    }
  } else {
    // Dynamic robot set: track it via plain subscription callbacks (no
    // extra rclcpp::Node/executor/thread -- see the class doc comment),
    // reconciled on reconcile_timer_.
    robot_staleness_sec_ = this->declare_parameter("robots.staleness_sec", robot_staleness_sec_);
    const double rescan_rate_hz = this->declare_parameter("robots.rescan_rate_hz", 1.0);
    if (rescan_rate_hz <= 0.0) {
      RCLCPP_FATAL(
        get_logger(),
        "Parameter 'robots.rescan_rate_hz' must be > 0 (got %.3f), exiting.", rescan_rate_hz);
      rclcpp::shutdown();
      std::exit(1);
    }

    capabilities_status_sub_ =
      this->create_subscription<easyfleet_interfaces::msg::CapabilityStatus>(
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

std::unique_ptr<RobotNavigationWatcher>
NavigationManagerNode::make_watcher(const std::string & robot_id)
{
  if (!relay_tf_ && !share_obstacles_) {
    return std::make_unique<RobotNavigationWatcher>(*this, robot_id, planner_plugin_key_);
  }
  if (share_obstacles_) {
    pose_trackers_.insert_or_assign(robot_id, RobotPoseTracker(robot_id, obstacles_base_frame_));
    obstacle_pubs_[robot_id] = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/" + robot_id + "/" + obstacles_topic_, rclcpp::QoS(1));
  }
  // Robots with their own /<robot_id>/tf: the selected frames are merged
  // into /tf and /tf_static so a fleet-wide RViz sees every robot (their
  // frames are already prefixed), without flooding /tf with each robot's
  // whole tree. The static broadcaster accumulates transforms by child
  // frame, so a late subscriber still gets every robot's static frames.
  return std::make_unique<RobotNavigationWatcher>(
    *this, robot_id, planner_plugin_key_,
    [this, robot_id](const tf2_msgs::msg::TFMessage & msg) {
      const auto now = std::chrono::steady_clock::now();
      if (share_obstacles_) {
        pose_trackers_.at(robot_id).update(msg, now);
      }
      if (!relay_tf_) {
        return;
      }
      tf2_msgs::msg::TFMessage out;
      for (const auto & transform : msg.transforms) {
        if (!relays_frame(robot_id, transform.child_frame_id)) {
          continue;
        }
        auto & last = last_relayed_[transform.child_frame_id];
        if (now - last >= relay_tf_period_) {
          last = now;
          out.transforms.push_back(transform);
        }
      }
      if (!out.transforms.empty()) {
        tf_pub_->publish(out);
      }
    },
    [this, robot_id](const tf2_msgs::msg::TFMessage & msg) {
      if (share_obstacles_) {
        pose_trackers_.at(robot_id).update(msg, std::chrono::steady_clock::now());
      }
      if (!relay_tf_) {
        return;
      }
      std::vector<geometry_msgs::msg::TransformStamped> selected;
      for (const auto & transform : msg.transforms) {
        if (relays_frame(robot_id, transform.child_frame_id)) {
          selected.push_back(transform);
        }
      }
      if (!selected.empty()) {
        map_tf_broadcaster_->sendTransform(selected);
      }
    });
}

bool
NavigationManagerNode::relays_frame(
  const std::string & robot_id, const std::string & child_frame_id) const
{
  if (relay_tf_frames_.empty()) {
    return true;
  }
  const std::string prefix = robot_id + "/";
  const bool prefixed = child_frame_id.compare(0, prefix.size(), prefix) == 0;
  return relay_tf_frames_.count(prefixed ? child_frame_id.substr(prefix.size()) : child_frame_id);
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
        make_watcher(robot_id));
      publish_robot_map_tf(robot_id);
    }
  }

  for (auto watcher_it = watchers_.begin(); watcher_it != watchers_.end(); ) {
    if (alive_robots.count((*watcher_it)->robot_id()) == 0) {
      RCLCPP_INFO(
        get_logger(), "Conflict monitor: '%s' stopped sending /capabilities_status; "
        "no longer watching it", (*watcher_it)->robot_id().c_str());
      if (currently_paused_.erase((*watcher_it)->robot_id()) > 0) {
        // Best effort: if it comes back, it must not stay stopped by us.
        (*watcher_it)->resume();
      }
      pose_trackers_.erase((*watcher_it)->robot_id());
      obstacle_pubs_.erase((*watcher_it)->robot_id());
      watcher_it = watchers_.erase(watcher_it);
    } else {
      ++watcher_it;
    }
  }
}

void
NavigationManagerNode::publish_fleet_obstacles()
{
  const auto now = std::chrono::steady_clock::now();
  std::map<std::string, RobotPose> poses;
  for (const auto & [robot_id, tracker] : pose_trackers_) {
    const auto pose = tracker.pose();
    if (pose && std::chrono::duration<double>(now - pose->received).count() <=
      obstacles_pose_timeout_sec_)
    {
      poses[robot_id] = *pose;
    }
  }
  if (!rclcpp::ok()) {
    return;  // shutting down: publishing would throw
  }
  for (const auto & [robot_id, pub] : obstacle_pubs_) {
    // Nothing to place the cloud with until the robot's own pose is known.
    if (poses.count(robot_id) == 0) {
      continue;
    }
    auto cloud = make_fleet_obstacle_cloud(robot_id, poses, obstacle_params_);
    // No other robot: nothing to send (an empty cloud crashes pcl::fromROSMsg in EasyNav's
    // sensors, and the robot forgets the last points after its forget_time anyway).
    if (cloud.width > 0) {
      try {
        pub->publish(cloud);
      } catch (const rclcpp::exceptions::RCLError & e) {
        // The middleware may already be gone while shutting down.
        RCLCPP_DEBUG(
          get_logger(), "Could not publish %s's obstacles: %s", robot_id.c_str(), e.what());
      }
    }
  }
}

void
NavigationManagerNode::check_conflicts()
{
  const auto now = std::chrono::steady_clock::now();
  const auto path_timeout = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(path_timeout_sec_));
  const auto pose_timeout = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(pose_timeout_sec_));

  // Only robots that are navigating, with a recent path, as seen from where
  // they actually are now.
  std::map<std::string, nav_msgs::msg::Path> paths;
  for (const auto & watcher : watchers_) {
    const auto age = watcher->path_age();
    if (!watcher->is_navigating() || !age || *age > path_timeout) {
      continue;
    }
    const auto & path = watcher->get_latest_path();
    const auto pose = watcher->current_pose(pose_timeout);
    if (pose && (pose->header.frame_id.empty() || path.header.frame_id.empty() ||
      pose->header.frame_id == path.header.frame_id))
    {
      paths[watcher->robot_id()] = trim_path_to_position(path, pose->pose.position);
    } else {
      paths[watcher->robot_id()] = path;
    }
  }

  std::set<std::string> paused;
  std::set<std::string> priority;
  for (const auto & [robot_id, since] : currently_paused_) {
    paused.insert(robot_id);
    if (std::chrono::duration<double>(now - since).count() > max_pause_sec_) {
      priority.insert(robot_id);
    }
  }

  // One entry per pair close enough to be worth explaining (both the
  // "at risk" and the "close but no risk" cases) -- run with
  // `--ros-args --log-level navigation_manager_node:=debug` to see them
  // live, including which robot is already stationary and yielding to
  // the other.
  std::vector<ConflictDecision> decisions;
  const auto to_pause = compute_robots_to_pause(
    paths, conflict_params_, paused, &decisions, priority);

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
    const auto & robot_id = watcher->robot_id();
    const bool should_pause = to_pause.count(robot_id) > 0;
    const bool paused_by_us = currently_paused_.count(robot_id) > 0;

    if (should_pause) {
      if (!paused_by_us) {
        RCLCPP_INFO(
          get_logger(), "Pausing '%s' to avoid an imminent navigation conflict", robot_id.c_str());
        currently_paused_[robot_id] = now;
      }
      // Also re-sends PAUSE if a new goal cleared it meanwhile.
      watcher->ensure_paused();
    } else if (paused_by_us) {
      if (priority.count(robot_id) > 0) {
        RCLCPP_WARN(
          get_logger(), "Resuming '%s' after %.0f s paused: likely a deadlock, giving it "
          "priority", robot_id.c_str(), max_pause_sec_);
      } else {
        RCLCPP_INFO(
          get_logger(), "Resuming '%s': navigation conflict cleared", robot_id.c_str());
      }
      watcher->resume();
      currently_paused_.erase(robot_id);
    }
  }
}

}  // namespace easyfleet
