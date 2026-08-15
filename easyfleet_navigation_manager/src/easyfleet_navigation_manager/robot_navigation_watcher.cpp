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
  rclcpp::Node & node, const std::string & robot_id, const std::string & planner_plugin_key)
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
    });
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
  send(easynav_interfaces::msg::NavigationControl::PAUSE);
}

void
RobotNavigationWatcher::resume()
{
  send(easynav_interfaces::msg::NavigationControl::RESUME);
}

void
RobotNavigationWatcher::on_control(easynav_interfaces::msg::NavigationControl::UniquePtr msg)
{
  if (msg->user_id == id_) {return;}  // Avoid self messages
  if (msg->nav_current_user_id != id_) {return;}  // Avoid messages addressed to others

  if (msg->type == easynav_interfaces::msg::NavigationControl::PAUSED) {
    paused_ = true;
  } else if (msg->type == easynav_interfaces::msg::NavigationControl::RESUMED) {
    paused_ = false;
  }
}

}  // namespace easyfleet
