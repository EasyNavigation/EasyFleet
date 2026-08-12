// Copyright 2026 Intelligent Robotics Lab
//
// This file is part of the projects Arquimea-URJC and AURORAS
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

#ifndef PERCEPTION_CAPABILITY__PERCEPTION_ACTION_SERVER_HPP_
#define PERCEPTION_CAPABILITY__PERCEPTION_ACTION_SERVER_HPP_

#include <random>
#include <string>

#include "perception_interfaces/action/perception.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "arch_mockup/action_server_base.hpp"

namespace perception_capability
{

/// Mock implementation of the perception_interfaces/Perception action.
/**
 * Part of the arch_mockup architecture: it does not run a real object
 * detector. When asked for the configured target class (a cat, "gato", by
 * default) it publishes feedback at a configurable rate (20 Hz by default)
 * with 1 to 3 randomly placed detections, in both 2D and 3D, until the goal
 * is canceled, preempted or the node shuts down -- there is no other
 * terminal condition, mirroring how a continuous perception stream behaves.
 * Goals asking for any other class are rejected.
 */
class PerceptionActionServer
  : public arch_mockup::ActionServerBase<perception_interfaces::action::Perception>
{
public:
  PerceptionActionServer(
    rclcpp_lifecycle::LifecycleNode * node,
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

}  // namespace perception_capability

#endif  // PERCEPTION_CAPABILITY__PERCEPTION_ACTION_SERVER_HPP_
