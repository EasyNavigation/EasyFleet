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

#ifndef EASYFLEET_FAKE_COLLABORATION_DEPLOYMENT__MANIPULATION_FAKE_CAPABILITY_HPP_
#define EASYFLEET_FAKE_COLLABORATION_DEPLOYMENT__MANIPULATION_FAKE_CAPABILITY_HPP_

#include <string>

#include "easyfleet_interfaces/action/manipulation.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easyfleet_core/capability.hpp"
#include "easyfleet_core/manipulation_action_server_base.hpp"

namespace easyfleet_fake_collaboration_deployment
{

/// Fake/mock implementation of the easyfleet_interfaces/Manipulation action.
/**
 * A reference implementation of `easyfleet_core::ManipulationActionServerBase`:
 * it does not drive a real manipulator or interact with real controllers. It
 * simulates trajectory execution progress over a configurable duration,
 * publishing feedback at a configurable rate, so that the rest of a
 * multi-capability system can be integrated and tested before a real
 * manipulator stack is wired in through its own
 * `ManipulationActionServerBase` subclass.
 */
class ManipulationFakeActionServer : public easyfleet_core::ManipulationActionServerBase
{
public:
  ManipulationFakeActionServer(
    rclcpp_lifecycle::LifecycleNode & node,
    const std::string & action_name);

protected:
  rclcpp_action::GoalResponse on_goal_received(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Goal> goal) override;

  void on_execute(const GoalHandleSharedPtr goal_handle) override;

private:
  double mock_execution_duration_s_;
  double mock_feedback_period_s_;
};

/// The "manipulation" capability, backed by the fake/mock action server: a
/// lifecycle node advertising a (mock) easyfleet_interfaces/Manipulation
/// action, described by config/manipulation.json.
class ManipulationFakeCapability
  : public easyfleet_core::Capability<ManipulationFakeActionServer>
{
public:
  explicit ManipulationFakeCapability(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
};

}  // namespace easyfleet_fake_collaboration_deployment

#endif  // EASYFLEET_FAKE_COLLABORATION_DEPLOYMENT__MANIPULATION_FAKE_CAPABILITY_HPP_
