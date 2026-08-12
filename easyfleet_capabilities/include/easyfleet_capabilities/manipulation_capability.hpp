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

#ifndef MANIPULATION_CAPABILITY__MANIPULATION_CAPABILITY_HPP_
#define MANIPULATION_CAPABILITY__MANIPULATION_CAPABILITY_HPP_

#include "rclcpp/rclcpp.hpp"

#include "arch_mockup/capability.hpp"
#include "manipulation_capability/execute_trajectory_action_server.hpp"

namespace manipulation_capability
{

/// The "manipulation" capability: a lifecycle node advertising a (mock)
/// moveit_msgs/ExecuteTrajectory action, described by
/// config/manipulation_capability.json.
class ManipulationCapability : public arch_mockup::Capability<ExecuteTrajectoryActionServer>
{
public:
  explicit ManipulationCapability(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
};

}  // namespace manipulation_capability

#endif  // MANIPULATION_CAPABILITY__MANIPULATION_CAPABILITY_HPP_
