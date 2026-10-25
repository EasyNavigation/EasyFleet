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

#ifndef EASYFLEET_BT_NODES__CAPABILITY_NODES_HPP_
#define EASYFLEET_BT_NODES__CAPABILITY_NODES_HPP_

#include <chrono>
#include <optional>
#include <string>

#include "easyfleet_bt_nodes/call_capability.hpp"
#include "easyfleet_interfaces/action/manipulation.hpp"
#include "easyfleet_interfaces/action/navigation.hpp"
#include "easyfleet_interfaces/action/perception.hpp"

namespace easyfleet_bt_nodes
{

/// @brief Sends a robot's navigation capability to the waypoint named by
/// "goal_id" (parameters_json {"goal_id": ...}, as the EasyNav-backed
/// capability expects). Its completion is the fraction of the longest
/// distance_remaining seen that has been covered.
class CallNavigation : public CallCapability<easyfleet_interfaces::action::Navigation>
{
public:
  /// @brief Constructs the BT node.
  CallNavigation(const std::string & name, const BT::NodeConfig & config);

  /// @brief Ports: the base ones plus "goal_id".
  /// @return The ports.
  static BT::PortsList providedPorts();

protected:
  bool make_goal(Goal & goal) override;
  BT::NodeStatus on_feedback(const Feedback & feedback) override;

private:
  float max_distance_{0.0f};
};

/// @brief Runs a robot's perception capability until "min_detections" objects
/// of "object_classes" (separated by ';') are detected, then stops it.
/// Fails after "timeout" seconds (0: no limit). Writes how many were
/// detected to the "detected" output port.
class CallPerception : public CallCapability<easyfleet_interfaces::action::Perception>
{
public:
  /// @brief Constructs the BT node.
  CallPerception(const std::string & name, const BT::NodeConfig & config);

  /// @brief Ports: the base ones plus "object_classes", "min_detections",
  /// "timeout" and the "detected" output.
  /// @return The ports.
  static BT::PortsList providedPorts();

protected:
  bool make_goal(Goal & goal) override;
  BT::NodeStatus on_feedback(const Feedback & feedback) override;
  BT::NodeStatus on_result(const GoalResult & result) override;
  BT::NodeStatus onRunning() override;

private:
  /// Detections in a feedback or result.
  BT::NodeStatus check_detections(std::size_t detections);

  int min_detections_{1};
  std::optional<std::chrono::steady_clock::time_point> deadline_;
};

/// @brief Runs the named task "named_task" (MODE_NAMED_TASK) on a robot's
/// manipulation capability. Its completion is the capability's progress.
class CallManipulation : public CallCapability<easyfleet_interfaces::action::Manipulation>
{
public:
  /// @brief Constructs the BT node.
  CallManipulation(const std::string & name, const BT::NodeConfig & config);

  /// @brief Ports: the base ones plus "named_task".
  /// @return The ports.
  static BT::PortsList providedPorts();

protected:
  bool make_goal(Goal & goal) override;
  BT::NodeStatus on_feedback(const Feedback & feedback) override;
};

}  // namespace easyfleet_bt_nodes

#endif  // EASYFLEET_BT_NODES__CAPABILITY_NODES_HPP_
