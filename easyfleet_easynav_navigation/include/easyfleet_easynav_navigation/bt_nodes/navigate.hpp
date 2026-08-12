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

/// Drives EasyNav (via easynav::GoalManagerClient) to the waypoint named by
/// the "goal_id" input port, resolved against a fixed id->pose registry
/// (EasyNav itself has no named-waypoint concept -- this node is where
/// that resolution happens).
/**
 * The GoalManagerClient is injected (owned by the capability, one instance
 * for its whole lifetime, shared across every tree/goal this node runs
 * in) rather than created here: this is what lets a *new* ROS navigation
 * goal preempt an in-flight one at the EasyNav level even though each ROS
 * goal ticks a brand-new tree from StartOff -- the new tree's Navigate
 * still calls send_goal() on the same client, and EasyNav treats that as
 * a preemption of the goal it already has from that client. Because of
 * this, onHalted() intentionally does nothing: only the capability itself
 * decides when to actually cancel() the shared client (real cancellation
 * or shutdown), never a plain preemption/halt of this node.
 */
class Navigate : public BT::StatefulActionNode
{
public:
  Navigate(
    const std::string & name,
    const BT::NodeConfig & config,
    std::shared_ptr<const std::map<std::string, geometry_msgs::msg::PoseStamped>> waypoints,
    easynav::GoalManagerClient::SharedPtr gm_client);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::shared_ptr<const std::map<std::string, geometry_msgs::msg::PoseStamped>> waypoints_;
  easynav::GoalManagerClient::SharedPtr gm_client_;
  std::string last_goal_id_;
};

}  // namespace easyfleet_easynav_navigation

#endif  // EASYFLEET_EASYNAV_NAVIGATION__BT_NODES__NAVIGATE_HPP_
