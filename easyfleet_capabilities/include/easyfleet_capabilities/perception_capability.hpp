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

#ifndef PERCEPTION_CAPABILITY__PERCEPTION_CAPABILITY_HPP_
#define PERCEPTION_CAPABILITY__PERCEPTION_CAPABILITY_HPP_

#include "rclcpp/rclcpp.hpp"

#include "arch_mockup/capability.hpp"
#include "perception_capability/perception_action_server.hpp"

namespace perception_capability
{

/// The "perception" capability: a lifecycle node advertising a (mock)
/// perception_interfaces/Perception action, described by
/// config/perception_capability.json.
class PerceptionCapability : public arch_mockup::Capability<PerceptionActionServer>
{
public:
  explicit PerceptionCapability(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
};

}  // namespace perception_capability

#endif  // PERCEPTION_CAPABILITY__PERCEPTION_CAPABILITY_HPP_
