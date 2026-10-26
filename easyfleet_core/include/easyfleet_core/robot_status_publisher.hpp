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


#ifndef EASYFLEET_CORE__ROBOT_STATUS_PUBLISHER_HPP_
#define EASYFLEET_CORE__ROBOT_STATUS_PUBLISHER_HPP_

#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

#include "easyfleet_interfaces/msg/capability_status.hpp"
#include "easyfleet_interfaces/msg/robot_status.hpp"

namespace easyfleet_core
{

/// @brief Publishes the aggregated status of a robot process on /robot_status.
/**
 * Parameters, declared on `node`:
 *  - `robot_status.rate_hz` (double, 1.0): publishing rate; 0 disables it.
 *  - `robot_status.pose_topic` (string, empty): a PoseWithCovarianceStamped
 *    topic (e.g. "amcl_pose") whose last pose is reported.
 *  - `robot_status.battery_topic` (string, empty): a BatteryState topic whose
 *    last message is reported.
 *
 * The robot is `node`'s namespace, and the capabilities come from `collect`.
 */
class RobotStatusPublisher
{
public:
  using CollectFn = std::function<std::vector<easyfleet_interfaces::msg::CapabilityStatus>()>;

  /// @param node The node to publish, subscribe and declare parameters with.
  /// @param collect Returns the status of each hosted capability.
  RobotStatusPublisher(rclcpp::Node::SharedPtr node, CollectFn collect);

  /// @brief The message that would be published now.
  easyfleet_interfaces::msg::RobotStatus make_status() const;

  /// @brief Publishes one message now.
  void publish();

private:
  rclcpp::Node::SharedPtr node_;
  CollectFn collect_;
  std::string robot_;

  mutable std::mutex mutex_;
  std::optional<geometry_msgs::msg::PoseWithCovarianceStamped> pose_;
  std::optional<sensor_msgs::msg::BatteryState> battery_;

  rclcpp::Publisher<easyfleet_interfaces::msg::RobotStatus>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace easyfleet_core

#endif  // EASYFLEET_CORE__ROBOT_STATUS_PUBLISHER_HPP_
