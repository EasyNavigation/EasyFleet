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

#include "easyfleet_bt_nodes/capability_nodes.hpp"

#include <algorithm>
#include <sstream>
#include <string>
#include <vector>

#include "nlohmann/json.hpp"

namespace easyfleet_bt_nodes
{

namespace
{

template<typename PortsT>
BT::PortsList with_base(PortsT extra, BT::PortsList base)
{
  for (auto & port : extra) {
    base.insert(std::move(port));
  }
  return base;
}

std::vector<std::string> split_classes(const std::string & classes)
{
  std::vector<std::string> out;
  std::stringstream stream(classes);
  std::string item;
  while (std::getline(stream, item, ';')) {
    item.erase(0, item.find_first_not_of(" \t"));
    item.erase(item.find_last_not_of(" \t") + 1);
    if (!item.empty()) {
      out.push_back(item);
    }
  }
  return out;
}

}  // namespace

CallNavigation::CallNavigation(const std::string & name, const BT::NodeConfig & config)
: CallCapability(name, config, "navigation")
{
}

BT::PortsList CallNavigation::providedPorts()
{
  return with_base(
    BT::PortsList{BT::InputPort<std::string>("goal_id", "waypoint to navigate to")},
    base_ports());
}

bool CallNavigation::make_goal(Goal & goal)
{
  const auto goal_id = getInput<std::string>("goal_id");
  if (!goal_id || goal_id.value().empty()) {
    error("needs the 'goal_id' input port");
    return false;
  }
  goal.parameters_json = nlohmann::json{{"goal_id", goal_id.value()}}.dump();
  max_distance_ = 0.0f;
  set_completion(0.0f);
  return true;
}

BT::NodeStatus CallNavigation::on_feedback(const Feedback & feedback)
{
  max_distance_ = std::max(max_distance_, feedback.distance_remaining);
  if (max_distance_ > 0.0f) {
    set_completion(1.0f - feedback.distance_remaining / max_distance_);
  }
  return BT::NodeStatus::RUNNING;
}

CallPerception::CallPerception(const std::string & name, const BT::NodeConfig & config)
: CallCapability(name, config, "perception")
{
}

BT::PortsList CallPerception::providedPorts()
{
  return with_base(
    BT::PortsList{
      BT::InputPort<std::string>("object_classes", "classes to detect, separated by ';'"),
      BT::InputPort<int>("min_detections", 1, "detections needed to succeed"),
      BT::InputPort<double>("timeout", 0.0, "seconds before failing; 0 for no limit"),
      BT::OutputPort<int>("detected", "detections that made the node succeed"),
    },
    base_ports());
}

bool CallPerception::make_goal(Goal & goal)
{
  goal.object_classes = split_classes(getInput<std::string>("object_classes").value_or(""));
  if (goal.object_classes.empty()) {
    error("needs the 'object_classes' input port");
    return false;
  }
  min_detections_ = std::max(1, getInput<int>("min_detections").value_or(1));
  goal.continuous = false;
  const double timeout = getInput<double>("timeout").value_or(0.0);
  deadline_.reset();
  if (timeout > 0.0) {
    deadline_ = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(timeout));
  }
  return true;
}

BT::NodeStatus CallPerception::check_detections(std::size_t detections)
{
  if (detections >= static_cast<std::size_t>(min_detections_)) {
    setOutput("detected", static_cast<int>(detections));
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus CallPerception::on_feedback(const Feedback & feedback)
{
  return check_detections(
    std::max(feedback.detections_3d.detections.size(), feedback.detections_2d.detections.size()));
}

BT::NodeStatus CallPerception::on_result(const GoalResult & result)
{
  if (result.outcome == Client::GoalOutcome::SUCCEEDED && result.result &&
    result.result->error_code == easyfleet_interfaces::action::Perception::Result::SUCCESS &&
    check_detections(
      std::max(
        result.result->detections_3d.detections.size(),
        result.result->detections_2d.detections.size())) == BT::NodeStatus::SUCCESS)
  {
    return BT::NodeStatus::SUCCESS;
  }
  if (result.outcome == Client::GoalOutcome::SUCCEEDED) {
    error("perception finished without enough detections");
    return BT::NodeStatus::FAILURE;
  }
  return CallCapability::on_result(result);
}

BT::NodeStatus CallPerception::onRunning()
{
  const BT::NodeStatus status = CallCapability::onRunning();
  if (status == BT::NodeStatus::RUNNING && deadline_ &&
    std::chrono::steady_clock::now() > *deadline_)
  {
    error("nothing detected before the timeout");
    finish();
    return BT::NodeStatus::FAILURE;
  }
  return status;
}

CallManipulation::CallManipulation(const std::string & name, const BT::NodeConfig & config)
: CallCapability(name, config, "manipulation")
{
}

BT::PortsList CallManipulation::providedPorts()
{
  return with_base(
    BT::PortsList{BT::InputPort<std::string>("named_task", "task to run")},
    base_ports());
}

bool CallManipulation::make_goal(Goal & goal)
{
  const auto task = getInput<std::string>("named_task");
  if (!task || task.value().empty()) {
    error("needs the 'named_task' input port");
    return false;
  }
  goal.mode = Goal::MODE_NAMED_TASK;
  goal.named_task = task.value();
  set_completion(0.0f);
  return true;
}

BT::NodeStatus CallManipulation::on_feedback(const Feedback & feedback)
{
  set_completion(feedback.progress);
  return BT::NodeStatus::RUNNING;
}

}  // namespace easyfleet_bt_nodes

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<easyfleet_bt_nodes::CallNavigation>("CallNavigation");
  factory.registerNodeType<easyfleet_bt_nodes::CallPerception>("CallPerception");
  factory.registerNodeType<easyfleet_bt_nodes::CallManipulation>("CallManipulation");
}
