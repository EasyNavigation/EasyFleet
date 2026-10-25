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

#include <chrono>
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

Navigate::Navigate(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
  if (!config.blackboard->get("waypoints", waypoints_)) {
    throw BT::RuntimeError("Navigate: missing 'waypoints' entry on the blackboard");
  }
  if (!config.blackboard->get("gm_client", gm_client_)) {
    throw BT::RuntimeError("Navigate: missing 'gm_client' entry on the blackboard");
  }
}

BT::PortsList Navigate::providedPorts()
{
  return {
    BT::InputPort<std::string>("goal_id", "id of the waypoint to navigate to"),
    BT::InputPort<double>(
      "response_timeout", 5.0, "seconds to wait for EasyNav to accept a goal before failing"),
  };
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

  last_send_time_ = std::chrono::steady_clock::now();
  using State = easynav::GoalManagerClient::State;
  const auto state = gm_client_->get_state();
  if (state == State::SENT_GOAL || state == State::SENT_PREEMPT) {
    // Still waiting for the answer to a previous send, which would make this
    // one be ignored: onRunning() sends it as soon as the client can take it.
    last_goal_id_.clear();
    return BT::NodeStatus::RUNNING;
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
  using State = easynav::GoalManagerClient::State;
  const BT::Expected<std::string> goal_id = getInput<std::string>("goal_id");
  const auto state_before = gm_client_->get_state();
  // GoalManagerClient ignores a send while it still waits for the answer to
  // the previous one: keep the change pending and retry on the next tick.
  if (goal_id && goal_id.value() != last_goal_id_ &&
    state_before != State::SENT_GOAL && state_before != State::SENT_PREEMPT)
  {
    const auto it = waypoints_->find(goal_id.value());
    if (it != waypoints_->end()) {
      last_goal_id_ = goal_id.value();
      reset_if_terminal(gm_client_);
      gm_client_->send_goal(it->second);
      last_send_time_ = std::chrono::steady_clock::now();
    }
  }

  const auto state = gm_client_->get_state();
  switch (state) {
    case State::IDLE:
    case State::SENT_GOAL:
    case State::SENT_PREEMPT:
      {
        // EasyNav never answered (not running, or the goal was lost).
        const double timeout = getInput<double>("response_timeout").value_or(5.0);
        if (std::chrono::duration<double>(
            std::chrono::steady_clock::now() - last_send_time_).count() > timeout)
        {
          std::cerr << "[Navigate] no answer from EasyNav after " << timeout << " s" << std::endl;
          return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
      }
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

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<easyfleet_easynav_navigation::Navigate>("Navigate");
}
