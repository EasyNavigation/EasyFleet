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


#include "easyfleet_core/robot_status_publisher.hpp"

#include <chrono>
#include <memory>
#include <utility>

#include "easyfleet_core/detail/namespace_utils.hpp"

namespace easyfleet_core
{

RobotStatusPublisher::RobotStatusPublisher(rclcpp::Node::SharedPtr node, CollectFn collect)
: node_(std::move(node)), collect_(std::move(collect))
{
  robot_ = detail::strip_leading_slash(node_->get_namespace());
  const auto rate_hz = node_->declare_parameter("robot_status.rate_hz", 1.0);
  const auto pose_topic = node_->declare_parameter("robot_status.pose_topic", std::string());
  const auto battery_topic = node_->declare_parameter("robot_status.battery_topic", std::string());

  pub_ = node_->create_publisher<easyfleet_interfaces::msg::RobotStatus>(
    "/robot_status", rclcpp::QoS(10).reliable());
  if (!pose_topic.empty()) {
    pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      pose_topic, rclcpp::QoS(1).reliable(),
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped & msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        pose_ = msg;
      });
  }
  if (!battery_topic.empty()) {
    battery_sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
      battery_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::BatteryState & msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        battery_ = msg;
      });
  }
  if (rate_hz > 0.0) {
    timer_ = node_->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / rate_hz)),
      [this]() {publish();});
  }
}

easyfleet_interfaces::msg::RobotStatus RobotStatusPublisher::make_status() const
{
  easyfleet_interfaces::msg::RobotStatus msg;
  msg.robot = robot_;
  msg.stamp = node_->now();
  if (collect_) {
    msg.capabilities = collect_();
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (pose_) {
    msg.has_pose = true;
    msg.pose = *pose_;
  }
  if (battery_) {
    msg.has_battery = true;
    msg.battery = *battery_;
  }
  return msg;
}

void RobotStatusPublisher::publish()
{
  pub_->publish(make_status());
}

}  // namespace easyfleet_core
