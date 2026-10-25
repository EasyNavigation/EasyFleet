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

#ifndef EASYFLEET_BT_NODES__CALL_CAPABILITY_HPP_
#define EASYFLEET_BT_NODES__CALL_CAPABILITY_HPP_

#include <algorithm>
#include <atomic>
#include <chrono>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"

#include "easyfleet_core/action_client.hpp"

namespace easyfleet_bt_nodes
{

/// @brief Base of the BT nodes that run one goal of a robot's capability.
/**
 * The action is `/<robot>/<capability_type>` (what an EasyFleet capability
 * named `<capability_type>` of robot `<robot>` serves), or the `action_name`
 * input port when given. RUNNING while the goal runs; SUCCESS or FAILURE
 * from its outcome (see `on_result()`), or earlier if `on_feedback()`
 * decides, in which case the goal is canceled. Halting the node cancels it.
 *
 * Needs no ROS node from the blackboard: the action client spins its own.
 */
template<typename ActionT>
class CallCapability : public BT::StatefulActionNode
{
public:
  using Client = easyfleet_core::ActionClient<ActionT>;
  using Goal = typename ActionT::Goal;
  using Feedback = typename ActionT::Feedback;
  using GoalResult = typename Client::GoalResult;

  /// @brief Constructs the BT node.
  /// @param name Name of this node instance, as given in the BT XML.
  /// @param config BT.CPP node configuration.
  /// @param capability_type Capability type, last part of the default action name.
  CallCapability(
    const std::string & name, const BT::NodeConfig & config, std::string capability_type)
  : BT::StatefulActionNode(name, config), capability_type_(std::move(capability_type))
  {
  }

  ~CallCapability() override
  {
    if (client_) {
      client_->cancel_all_goals();
      // Joins the client's spin thread before the members its callbacks use go away.
      client_.reset();
    }
  }

  /// @brief Ports shared by every capability call.
  /// @return The ports.
  static BT::PortsList base_ports()
  {
    return {
      BT::InputPort<std::string>("robot", "robot whose capability is called"),
      BT::InputPort<std::string>(
        "action_name", "", "action to call; empty for /<robot>/<capability type>"),
      BT::InputPort<double>("server_timeout", 5.0, "seconds to wait for the action server"),
    };
  }

protected:
  /// @brief Fills the goal from the input ports.
  /// @param goal Goal to fill.
  /// @return False (the node fails) if the ports are not valid.
  virtual bool make_goal(Goal & goal) = 0;

  /// @brief Called on each feedback; a status other than RUNNING ends the call
  /// (and cancels the goal). Runs on the BT thread.
  /// @param feedback Last feedback received.
  /// @return RUNNING to keep waiting, SUCCESS or FAILURE to end now.
  virtual BT::NodeStatus on_feedback(const Feedback & feedback)
  {
    (void)feedback;
    return BT::NodeStatus::RUNNING;
  }

  /// @brief Status from the goal's terminal outcome; SUCCESS only if it succeeded
  /// with `error_code` SUCCESS.
  /// @param result Terminal outcome.
  /// @return SUCCESS or FAILURE.
  virtual BT::NodeStatus on_result(const GoalResult & result)
  {
    if (result.outcome == Client::GoalOutcome::SUCCEEDED && result.result &&
      result.result->error_code == ActionT::Result::SUCCESS)
    {
      return BT::NodeStatus::SUCCESS;
    }
    error("goal ended with outcome " + std::to_string(static_cast<int>(result.outcome)) +
      (result.result && !result.result->error_msg.empty() ?
      ": " + result.result->error_msg : std::string()));
    return BT::NodeStatus::FAILURE;
  }

  /// @brief Writes the progress of the current goal (clamped to 0..1) to the
  /// blackboard's "completion" entry, which PlanSys2's BTAction reports as
  /// the progress of the action.
  /// @param completion Progress of the goal.
  void set_completion(float completion)
  {
    config().blackboard->set<float>("completion", std::clamp(completion, 0.0f, 1.0f));
  }

  /// @brief Prints an error tagged with this node's name.
  void error(const std::string & msg) const
  {
    std::cerr << "[" << registrationName() << " " << name() << "] " << msg << std::endl;
  }

  BT::NodeStatus onStart() override
  {
    const std::string action = action_name();
    if (action.empty()) {
      error("needs the 'robot' or the 'action_name' input port");
      return BT::NodeStatus::FAILURE;
    }
    Goal goal;
    if (!make_goal(goal)) {
      return BT::NodeStatus::FAILURE;
    }
    if (!client_ || client_->get_action_name() != action) {
      client_ = Client::create(*parent_node(), action);
    }
    const double timeout = getInput<double>("server_timeout").value_or(5.0);
    if (!client_->wait_for_server(
        std::chrono::milliseconds(static_cast<int64_t>(timeout * 1000.0))))
    {
      error("action server '" + action + "' not available");
      return BT::NodeStatus::FAILURE;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      result_.reset();
      feedback_.reset();
    }
    const uint64_t call = ++call_;
    const bool sent = client_->send_goal(
      goal,
      [this, call](const GoalResult & result) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (call == call_) {
          result_ = result;
        }
      },
      [this, call](std::shared_ptr<const Feedback> feedback) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (call == call_) {
          feedback_ = feedback;
        }
      });
    if (!sent) {
      error("could not send the goal to '" + action + "'");
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    std::optional<GoalResult> result;
    std::shared_ptr<const Feedback> feedback;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      result = result_;
      feedback = feedback_;
      feedback_.reset();
    }
    if (result) {
      return on_result(*result);
    }
    if (feedback) {
      const BT::NodeStatus status = on_feedback(*feedback);
      if (status != BT::NodeStatus::RUNNING) {
        finish();
        return status;
      }
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    finish();
  }

  /// @brief Ignores whatever arrives for the current goal and cancels it.
  void finish()
  {
    ++call_;
    if (client_) {
      client_->cancel_all_goals();
    }
  }

private:
  std::string action_name()
  {
    const std::string explicit_name = getInput<std::string>("action_name").value_or("");
    if (!explicit_name.empty()) {
      return explicit_name;
    }
    const auto robot = getInput<std::string>("robot");
    if (!robot || robot.value().empty()) {
      return "";
    }
    return "/" + robot.value() + "/" + capability_type_;
  }

  /// Only names the action client's own node; never spun.
  rclcpp::Node::SharedPtr parent_node()
  {
    if (!parent_node_) {
      static std::atomic<uint64_t> counter{0};
      rclcpp::NodeOptions options;
      options.use_global_arguments(false);
      options.start_parameter_services(false);
      options.start_parameter_event_publisher(false);
      parent_node_ = std::make_shared<rclcpp::Node>(
        "easyfleet_bt_" + capability_type_ + "_" + std::to_string(counter.fetch_add(1)), options);
    }
    return parent_node_;
  }

  std::string capability_type_;
  rclcpp::Node::SharedPtr parent_node_;
  typename Client::SharedPtr client_;
  std::mutex mutex_;
  std::atomic<uint64_t> call_{0};
  std::optional<GoalResult> result_;
  std::shared_ptr<const Feedback> feedback_;
};

}  // namespace easyfleet_bt_nodes

#endif  // EASYFLEET_BT_NODES__CALL_CAPABILITY_HPP_
