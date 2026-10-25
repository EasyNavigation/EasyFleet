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

#include "easyfleet_navigation_manager/robot_navigation_watcher.hpp"

namespace easyfleet
{

RobotNavigationWatcher::RobotNavigationWatcher(
  rclcpp::Node & node, const std::string & robot_id, const std::string & planner_plugin_key,
  std::function<void(const tf2_msgs::msg::TFMessage &)> relay_tf,
  std::function<void(const tf2_msgs::msg::TFMessage &)> relay_tf_static)
: node_(node), robot_id_(robot_id)
{
  id_ = std::string(node_.get_name()) + "_conflict_monitor";

  const std::string control_topic = "/" + robot_id_ + "/easynav_control";
  const std::string path_topic =
    "/" + robot_id_ + "/planner_node/" + planner_plugin_key + "/path";

  // Created once, here, and reused for every pause()/resume() call --
  // not recreated per call.
  control_pub_ = node_.create_publisher<easynav_interfaces::msg::NavigationControl>(
    control_topic, 100);
  control_sub_ = node_.create_subscription<easynav_interfaces::msg::NavigationControl>(
    control_topic, 100,
    [this](easynav_interfaces::msg::NavigationControl::UniquePtr msg) {
      on_control(std::move(msg));
    });
  path_sub_ = node_.create_subscription<nav_msgs::msg::Path>(
    path_topic, 10,
    [this](nav_msgs::msg::Path::UniquePtr msg) {
      latest_path_ = *msg;
      path_received_ = std::chrono::steady_clock::now();
    });

  if (relay_tf) {
    tf_sub_ = node_.create_subscription<tf2_msgs::msg::TFMessage>(
      "/" + robot_id_ + "/tf", rclcpp::QoS(100),
      [relay_tf](tf2_msgs::msg::TFMessage::UniquePtr msg) {relay_tf(*msg);});
  }
  if (relay_tf_static) {
    tf_static_sub_ = node_.create_subscription<tf2_msgs::msg::TFMessage>(
      "/" + robot_id_ + "/tf_static", rclcpp::QoS(100).transient_local(),
      [relay_tf_static](tf2_msgs::msg::TFMessage::UniquePtr msg) {relay_tf_static(*msg);});
  }
}

std::optional<std::chrono::steady_clock::duration>
RobotNavigationWatcher::path_age() const
{
  if (!path_received_) {
    return std::nullopt;
  }
  return std::chrono::steady_clock::now() - *path_received_;
}

std::optional<geometry_msgs::msg::PoseStamped>
RobotNavigationWatcher::current_pose(std::chrono::steady_clock::duration max_age) const
{
  if (!current_pose_ || std::chrono::steady_clock::now() - pose_received_ > max_age) {
    return std::nullopt;
  }
  return current_pose_;
}

bool
RobotNavigationWatcher::in_flight(
  const std::optional<std::chrono::steady_clock::time_point> & sent)
{
  return sent && std::chrono::steady_clock::now() - *sent < std::chrono::seconds(1);
}

void
RobotNavigationWatcher::ensure_paused()
{
  if (paused_ || !navigating_ || in_flight(pause_sent_)) {
    return;
  }
  pause();
}

void
RobotNavigationWatcher::ensure_resumed()
{
  if (!paused_ || in_flight(resume_sent_)) {
    return;
  }
  resume();
}

void
RobotNavigationWatcher::send(uint8_t control_type)
{
  easynav_interfaces::msg::NavigationControl msg;
  msg.header.stamp = node_.now();
  msg.type = control_type;
  msg.user_id = id_;
  msg.seq = seq_++;

  control_pub_->publish(msg);
}

void
RobotNavigationWatcher::pause()
{
  pause_sent_ = std::chrono::steady_clock::now();
  resume_sent_.reset();
  send(easynav_interfaces::msg::NavigationControl::PAUSE);
}

void
RobotNavigationWatcher::resume()
{
  resume_sent_ = std::chrono::steady_clock::now();
  pause_sent_.reset();
  send(easynav_interfaces::msg::NavigationControl::RESUME);
}

void
RobotNavigationWatcher::on_control(easynav_interfaces::msg::NavigationControl::UniquePtr msg)
{
  using easynav_interfaces::msg::NavigationControl;
  if (msg->user_id == id_) {return;}  // Avoid self messages

  // Only the GoalManager's answers and reports describe the robot's state;
  // other clients' requests (REQUEST, CANCEL, PAUSE, RESUME) don't.
  switch (msg->type) {
    case NavigationControl::ACCEPT:
      // A new or preempting goal: GoalManager clears its pause flag.
      navigating_ = true;
      paused_ = false;
      pause_sent_.reset();
      break;
    case NavigationControl::FEEDBACK:
      navigating_ = true;
      current_pose_ = msg->current_pose;
      pose_received_ = std::chrono::steady_clock::now();
      break;
    case NavigationControl::FINISHED:
    case NavigationControl::FAILED:
    case NavigationControl::CANCELLED:
      navigating_ = false;
      paused_ = false;
      pause_sent_.reset();
      resume_sent_.reset();
      break;
    case NavigationControl::PAUSED:
      // The pause flag is shared: whoever paused it, the robot is stopped.
      paused_ = true;
      pause_sent_.reset();
      break;
    case NavigationControl::RESUMED:
      paused_ = false;
      resume_sent_.reset();
      break;
    case NavigationControl::REJECT:
      if (msg->nav_current_user_id == id_) {
        // Our PAUSE/RESUME was refused: the robot is idle.
        navigating_ = false;
        paused_ = false;
        pause_sent_.reset();
        resume_sent_.reset();
      }
      break;
    default:
      break;
  }
}

}  // namespace easyfleet
