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

#ifndef EASYFLEET_EASYNAV_NAVIGATION__BT_NODES__NAVIGATE_HPP_
#define EASYFLEET_EASYNAV_NAVIGATION__BT_NODES__NAVIGATE_HPP_

#include <map>
#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "easynav_system/GoalManagerClient.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace easyfleet_easynav_navigation
{

/// @brief Drives EasyNav (via easynav::GoalManagerClient) to the waypoint named by
/// the "goal_id" input port, resolved against a fixed id->pose registry
/// (EasyNav itself has no named-waypoint concept -- this node is where
/// that resolution happens).
class Navigate : public BT::StatefulActionNode
{
public:
  /// @brief Constructs the BT node.
  /// @param name Name of this node instance, as given in the BT XML.
  /// @param config BT.CPP node configuration (ports, blackboard); the
  ///   waypoint registry and `GoalManagerClient` are read off the
  ///   blackboard here.
  Navigate(const std::string & name, const BT::NodeConfig & config);

  /// @brief This node's ports: the input port `goal_id`.
  /// @return This node's ports: the input port `goal_id`.
  static BT::PortsList providedPorts();

  /// @brief Resolves the `goal_id` input port against the waypoint registry and
  /// sends the corresponding pose through `gm_client_`.
  /// @return `RUNNING`, or `FAILURE` if `goal_id` is missing/unknown.
  BT::NodeStatus onStart() override;
  /// @brief Polls `gm_client_` for the in-flight goal's progress.
  /// @return `RUNNING` while EasyNav is still navigating, `SUCCESS`/
  ///   `FAILURE` once `gm_client_` reports a terminal state.
  BT::NodeStatus onRunning() override;
  /// @brief No-op: an in-flight EasyNav goal is left alone on halt, deliberately
  /// (see the class-level design note on preemption in the owning
  /// capability), not canceled here.
  void onHalted() override;

private:
  std::shared_ptr<const std::map<std::string, geometry_msgs::msg::PoseStamped>> waypoints_;
  easynav::GoalManagerClient::SharedPtr gm_client_;
  std::string last_goal_id_;
};

}  // namespace easyfleet_easynav_navigation

#endif  // EASYFLEET_EASYNAV_NAVIGATION__BT_NODES__NAVIGATE_HPP_
