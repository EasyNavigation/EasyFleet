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

#include "easyfleet_easynav_navigation/bt_nodes/navigate.hpp"

#include <iostream>
#include <map>
#include <memory>
#include <string>

namespace easyfleet_easynav_navigation
{

namespace
{

/// GoalManagerClient::send_goal()/send_goals() only accept a new goal from
/// IDLE or ACCEPTED_AND_NAVIGATING -- any terminal state left over from a
/// previous ROS goal (NAVIGATION_FINISHED/REJECTED/FAILED/CANCELLED/ERROR)
/// otherwise makes it silently ignore every subsequent send_goal() call
/// ("Trying to send new goals in state N. Ignoring"), since this client is
/// kept alive across ROS goals (see easynav_navigation_capability.hpp).
void reset_if_terminal(const easynav::GoalManagerClient::SharedPtr & gm_client)
{
  using State = easynav::GoalManagerClient::State;
  switch (gm_client->get_state()) {
    case State::NAVIGATION_FINISHED:
    case State::NAVIGATION_REJECTED:
    case State::NAVIGATION_FAILED:
    case State::NAVIGATION_CANCELLED:
    case State::ERROR:
      gm_client->reset();
      break;
    default:
      break;
  }
}

}  // namespace

Navigate::Navigate(
  const std::string & name,
  const BT::NodeConfig & config,
  std::shared_ptr<const std::map<std::string, geometry_msgs::msg::PoseStamped>> waypoints,
  easynav::GoalManagerClient::SharedPtr gm_client)
: BT::StatefulActionNode(name, config),
  waypoints_(std::move(waypoints)),
  gm_client_(std::move(gm_client))
{
}

BT::PortsList Navigate::providedPorts()
{
  return {BT::InputPort<std::string>("goal_id", "id of the waypoint to navigate to")};
}

BT::NodeStatus Navigate::onStart()
{
  const BT::Expected<std::string> goal_id = getInput<std::string>("goal_id");
  if (!goal_id) {
    std::cerr << "[Navigate] missing required input port 'goal_id': " <<
      goal_id.error() << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  const auto it = waypoints_->find(goal_id.value());
  if (it == waypoints_->end()) {
    std::cerr << "[Navigate] unknown waypoint id '" << goal_id.value() << "'" << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  last_goal_id_ = goal_id.value();
  reset_if_terminal(gm_client_);
  gm_client_->send_goal(it->second);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Navigate::onRunning()
{
  // If the blackboard's goal_id changed while this node is still RUNNING,
  // re-send: EasyNav treats a new send_goal() from an already-navigating
  // client as a preemption of its current target.
  const BT::Expected<std::string> goal_id = getInput<std::string>("goal_id");
  if (goal_id && goal_id.value() != last_goal_id_) {
    const auto it = waypoints_->find(goal_id.value());
    if (it != waypoints_->end()) {
      last_goal_id_ = goal_id.value();
      reset_if_terminal(gm_client_);
      gm_client_->send_goal(it->second);
    }
  }

  using State = easynav::GoalManagerClient::State;
  switch (gm_client_->get_state()) {
    case State::IDLE:
    case State::SENT_GOAL:
    case State::SENT_PREEMPT:
    case State::ACCEPTED_AND_NAVIGATING:
      return BT::NodeStatus::RUNNING;
    case State::NAVIGATION_FINISHED:
      return BT::NodeStatus::SUCCESS;
    case State::NAVIGATION_FAILED:
    case State::NAVIGATION_REJECTED:
    case State::NAVIGATION_CANCELLED:
    case State::ERROR:
    default:
      return BT::NodeStatus::FAILURE;
  }
}

void Navigate::onHalted()
{
  // Intentionally empty: cancelling the shared GoalManagerClient is the
  // capability's decision (real cancellation/shutdown), not a side effect
  // of this node being halted (which also happens on ordinary preemption
  // by a new ROS goal that will keep driving EasyNav via the same client).
}

}  // namespace easyfleet_easynav_navigation
