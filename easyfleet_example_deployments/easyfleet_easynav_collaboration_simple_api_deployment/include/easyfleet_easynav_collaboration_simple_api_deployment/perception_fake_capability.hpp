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

#ifndef EASYFLEET_EASYNAV_COLLABORATION_DEPLOYMENT__PERCEPTION_FAKE_CAPABILITY_HPP_
#define EASYFLEET_EASYNAV_COLLABORATION_DEPLOYMENT__PERCEPTION_FAKE_CAPABILITY_HPP_

#include <random>
#include <string>

#include "easyfleet_interfaces/action/perception.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easyfleet_core/capability.hpp"
#include "easyfleet_core/perception_action_server_base.hpp"

namespace easyfleet_easynav_collaboration_simple_api_deployment
{

/// Fake/mock implementation of the easyfleet_interfaces/Perception action.
/**
 * A reference implementation of `easyfleet_core::PerceptionActionServerBase`:
 * it does not run a real object detector. When asked for the configured
 * target class (a cat, "gato", by default) it publishes feedback at a
 * configurable rate (20 Hz by default) with 1 to 3 randomly placed
 * detections, in both 2D and 3D, until the goal is canceled, preempted or
 * the node shuts down -- there is no other terminal condition, mirroring
 * how a continuous perception stream behaves. Goals asking for any other
 * class are rejected. `continuous`/`max_rate_hz` are accepted but not
 * honored by this mock: every goal streams forever at the fixed
 * `detection_rate_hz` parameter, since a mock has no notion of "detect once
 * and stop".
 *
 * Duplicated (not shared) from easyfleet_fake_collaboration_deployment's
 * own copy of this same reference implementation, matching this project's
 * established convention of keeping each deployment package self-contained
 * rather than introducing a cross-deployment dependency for a mock.
 */
class PerceptionFakeActionServer : public easyfleet_core::PerceptionActionServerBase
{
public:
  PerceptionFakeActionServer(
    rclcpp_lifecycle::LifecycleNode & node,
    const std::string & action_name);

protected:
  rclcpp_action::GoalResponse on_goal_received(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Goal> goal) override;

  void on_execute(const GoalHandleSharedPtr goal_handle) override;

private:
  Feedback make_random_detections();

  std::string target_class_;
  std::string frame_id_;
  double detection_rate_hz_;

  std::mt19937 random_engine_;
};

/// The "perception" capability, backed by the fake/mock action server: a
/// lifecycle node advertising a (mock) easyfleet_interfaces/Perception
/// action, described by config/robot_1/perception.json.
class PerceptionFakeCapability : public easyfleet_core::Capability<PerceptionFakeActionServer>
{
public:
  explicit PerceptionFakeCapability(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
};

}  // namespace easyfleet_easynav_collaboration_simple_api_deployment

#endif  // EASYFLEET_EASYNAV_COLLABORATION_DEPLOYMENT__PERCEPTION_FAKE_CAPABILITY_HPP_
