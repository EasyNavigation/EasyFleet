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

#ifndef EASYFLEET_MISSION_MANAGER__DETAIL__ROBOT_HANDLE_IMPL_HPP_
#define EASYFLEET_MISSION_MANAGER__DETAIL__ROBOT_HANDLE_IMPL_HPP_

// Out-of-line definition of RobotHandle::run_capability<ActionT>(),
// deliberately *not* included from robot_handle.hpp itself: the template
// body needs FleetSession as a complete type (session_->node()/
// status_marker_publisher()), and robot_handle.hpp only forward-declares
// FleetSession to break the circular include the other way round
// (fleet_session.hpp already includes robot_handle.hpp in full). This
// header is instead included from the bottom of fleet_session.hpp, the
// one place both classes are simultaneously complete -- so any
// translation unit that wants to call run_capability<ActionT>() needs
// fleet_session.hpp (or simple_controller.hpp, which already pulls it in)
// in scope, same as it already needs a real FleetSession/SimpleController
// to make the call meaningful in the first place.

#include <algorithm>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

#include "easyfleet_mission_manager/fleet_session.hpp"
#include "easyfleet_mission_manager/robot_handle.hpp"

namespace easyfleet
{

template<typename ActionT>
void RobotHandle::run_capability(
  const std::string & capability_type,
  const typename ActionT::Goal & goal,
  std::chrono::seconds timeout)
{
  if (!session_) {
    // Never add_robot()-ed to a FleetSession -- capabilities_ is
    // unconditionally empty in that case too (only FleetSession::add_robot()
    // can populate it, and it always attach()es first), so this would
    // otherwise be silently swallowed by the "not active" branch below,
    // indistinguishable from a real, already-attached robot that simply
    // hasn't announced this capability yet. A caller bug, not a runtime
    // condition to tolerate quietly.
    throw std::logic_error(
            "RobotHandle::run_capability(\"" + capability_type + "\", ...): this handle was "
            "never added to a FleetSession (call FleetSession::add_robot()/"
            "SimpleController::add_robot() first).");
  }

  const auto it = std::find_if(
    capabilities_.begin(), capabilities_.end(),
    [&](const easyfleet_mission_manager::CapabilityInfo & info) {
      return info.capability == capability_type && info.active;
    });
  if (it == capabilities_.end()) {
    easyfleet_mission_manager::safe_print(
      std::string(easyfleet_mission_manager::ansi::yellow) + "[" + name_ + "/" +
      capability_type + "] not active, skipping." + easyfleet_mission_manager::ansi::reset);
    return;
  }

  using Running = easyfleet_mission_manager::detail::RunningCapability<ActionT>;

  auto running_it = running_.find(capability_type);
  if (running_it == running_.end()) {
    auto running = std::make_unique<Running>(
      *session_->node(), it->action_name, name_, capability_type,
      session_->status_marker_publisher());
    running_it = running_.emplace(capability_type, std::move(running)).first;
  }

  auto * running = dynamic_cast<Running *>(running_it->second.get());
  if (!running) {
    throw std::logic_error(
            "RobotHandle::run_capability(\"" + capability_type + "\", ...): this capability "
            "type was already run with a different action type on this handle -- a "
            "capability_type must mean the same ActionT for the life of a RobotHandle.");
  }

  running->run(goal, timeout);
}

}  // namespace easyfleet

#endif  // EASYFLEET_MISSION_MANAGER__DETAIL__ROBOT_HANDLE_IMPL_HPP_
