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

#ifndef EASYFLEET_EASYNAV_NAVIGATION__EASYNAV_NAVIGATION_CAPABILITY_HPP_
#define EASYFLEET_EASYNAV_NAVIGATION__EASYNAV_NAVIGATION_CAPABILITY_HPP_

#include <map>
#include <memory>
#include <string>
#include <thread>

#include "behaviortree_cpp/bt_factory.h"
#include "easynav_system/GoalManagerClient.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easyfleet_core/capability.hpp"
#include "easyfleet_core/navigation_action_server_base.hpp"

namespace easyfleet_easynav_navigation
{

/// Real navigation capability backed by EasyNav (via easynav::GoalManagerClient)
/// and structured internally as a BehaviorTree.CPP tree
/// (StartOff -> Navigate -> Finish). Named waypoints are configured as ROS
/// parameters and resolved to poses by the Navigate node -- EasyNav itself
/// has no named-waypoint concept. The incoming ROS goal selects one by id
/// via `parameters_json: {"goal_id": "<id>"}` (target_pose/waypoints on the
/// goal itself go unused by this backend).
class EasynavNavigationActionServer : public easyfleet_core::NavigationActionServerBase
{
public:
  EasynavNavigationActionServer(
    rclcpp_lifecycle::LifecycleNode * node,
    const std::string & action_name);
  ~EasynavNavigationActionServer() override;

protected:
  rclcpp_action::GoalResponse on_goal_received(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Goal> goal) override;

  void on_execute(const GoalHandleSharedPtr goal_handle) override;

private:
  std::shared_ptr<std::map<std::string, geometry_msgs::msg::PoseStamped>> waypoints_;
  std::string behavior_tree_xml_;
  double tick_rate_hz_;

  // GoalManagerClient needs a plain rclcpp::Node::SharedPtr, but this
  // capability is an rclcpp_lifecycle::LifecycleNode -- so, same pattern as
  // easyfleet_core::ActionClient, it owns a small internal node dedicated
  // to talking to EasyNav, spun on its own background thread for the
  // capability's whole lifetime.
  rclcpp::Node::SharedPtr internal_node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread spin_thread_;

  // One instance for the capability's entire lifetime (not per ROS goal):
  // this is what lets a new goal preempt an in-flight one at the EasyNav
  // level (see bt_nodes/navigate.hpp for why).
  easynav::GoalManagerClient::SharedPtr gm_client_;

  BT::BehaviorTreeFactory factory_;
};

/// The "navigation" capability, backed by EasyNav: a lifecycle node
/// advertising a real easyfleet_interfaces/Navigation action, described by
/// config/easynav/navigation.json.
class EasynavNavigationCapability
  : public easyfleet_core::Capability<EasynavNavigationActionServer>
{
public:
  explicit EasynavNavigationCapability(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
};

}  // namespace easyfleet_easynav_navigation

#endif  // EASYFLEET_EASYNAV_NAVIGATION__EASYNAV_NAVIGATION_CAPABILITY_HPP_
