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

#ifndef ARCH_MOCKUP__DETAIL__PARAM_UTILS_HPP_
#define ARCH_MOCKUP__DETAIL__PARAM_UTILS_HPP_

#include <string>

namespace arch_mockup
{
namespace detail
{

/// Turns an action name (e.g. "/robot/follow_path") into a valid, readable
/// ROS 2 parameter name segment (e.g. "robot.follow_path").
std::string sanitize_parameter_name(const std::string & action_name);

/// Turns an arbitrary string into a valid ROS 2 base node name segment
/// (alphanumeric and underscores only, not starting with a digit).
std::string sanitize_identifier(const std::string & text);

}  // namespace detail
}  // namespace arch_mockup

#endif  // ARCH_MOCKUP__DETAIL__PARAM_UTILS_HPP_
