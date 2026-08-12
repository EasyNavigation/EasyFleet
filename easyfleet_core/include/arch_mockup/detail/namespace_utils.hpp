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

#ifndef ARCH_MOCKUP__DETAIL__NAMESPACE_UTILS_HPP_
#define ARCH_MOCKUP__DETAIL__NAMESPACE_UTILS_HPP_

#include <string>

namespace arch_mockup
{
namespace detail
{

/// Strips the leading '/' from a ROS 2 namespace, so the default (no)
/// namespace "/" becomes "" and "/robot1" becomes "robot1".
inline std::string strip_leading_slash(const std::string & ns)
{
  return (!ns.empty() && ns.front() == '/') ? ns.substr(1) : ns;
}

/// `strip_leading_slash(ns)`, or `"-"` if the result would be empty. Used
/// when logging the robot identity, so unnamespaced nodes (e.g. in tests)
/// print something more legible than an empty pair of quotes.
inline std::string robot_label(const std::string & ns)
{
  const auto stripped = strip_leading_slash(ns);
  return stripped.empty() ? std::string("-") : stripped;
}

}  // namespace detail
}  // namespace arch_mockup

#endif  // ARCH_MOCKUP__DETAIL__NAMESPACE_UTILS_HPP_
