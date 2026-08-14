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

#ifndef EASYFLEET_CORE__PERCEPTION_ACTION_SERVER_BASE_HPP_
#define EASYFLEET_CORE__PERCEPTION_ACTION_SERVER_BASE_HPP_

#include "easyfleet_core/action_server_base.hpp"
#include "easyfleet_interfaces/action/perception.hpp"

namespace easyfleet_core
{

/// @brief Fixes `ActionServerBase`'s action type to `easyfleet_interfaces/Perception`,
/// so a concrete perception backend (a classic detector, a VLM-based one, a
/// fake mock, ...) only has to subclass this and implement
/// `on_goal_received()`/`on_execute()` -- it never needs to spell out the
/// action type itself.
class PerceptionActionServerBase
  : public ActionServerBase<easyfleet_interfaces::action::Perception>
{
public:
  using ActionServerBase<easyfleet_interfaces::action::Perception>::ActionServerBase;
};

}  // namespace easyfleet_core

#endif  // EASYFLEET_CORE__PERCEPTION_ACTION_SERVER_BASE_HPP_
