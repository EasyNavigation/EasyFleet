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

#ifndef EASYFLEET_CORE__DETAIL__NAMESPACE_UTILS_HPP_
#define EASYFLEET_CORE__DETAIL__NAMESPACE_UTILS_HPP_

#include <string>

namespace easyfleet_core
{
namespace detail
{

/// @brief Strips the leading '/' from a ROS 2 namespace, so the default (no)
/// namespace "/" becomes "" and "/robot1" becomes "robot1".
inline std::string strip_leading_slash(const std::string & ns)
{
  return (!ns.empty() && ns.front() == '/') ? ns.substr(1) : ns;
}

/// @brief `strip_leading_slash(ns)`, or `"-"` if the result would be empty. Used
/// when logging the robot identity, so unnamespaced nodes (e.g. in tests)
/// print something more legible than an empty pair of quotes.
inline std::string robot_label(const std::string & ns)
{
  const auto stripped = strip_leading_slash(ns);
  return stripped.empty() ? std::string("-") : stripped;
}

}  // namespace detail
}  // namespace easyfleet_core

#endif  // EASYFLEET_CORE__DETAIL__NAMESPACE_UTILS_HPP_
