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

#ifndef EASYFLEET_MISSION_MANAGER__CAPABILITY_STATE_HPP_
#define EASYFLEET_MISSION_MANAGER__CAPABILITY_STATE_HPP_

#include <string>

namespace easyfleet
{

/// What a `RobotHandle`-driven capability is doing right now, from the
/// mission script's point of view. Deliberately richer than a plain
/// running/not-running bool: `is_capability_running()` alone can't tell a
/// beginner's mission script "it finished, and here's whether it actually
/// worked" apart from "nothing is happening because it crashed" -- three
/// very different situations that all look like "not running" if that's
/// all you can ask.
enum class CapabilityState
{
  /// Never asked to do anything (or done, superseded, and not re-run).
  IDLE,
  /// A goal is in flight.
  RUNNING,
  /// The last goal finished successfully.
  SUCCEEDED,
  /// The last goal finished unsuccessfully (aborted, canceled, rejected,
  /// or client-side timeout) -- see `RobotHandle::last_outcome()` for the
  /// specific reason.
  FAILED,
  /// No heartbeat seen recently on `/capabilities_status` for this
  /// capability -- the process is gone, was never there, or has stalled.
  /// Distinct from FAILED: a FAILED goal still means the capability itself
  /// is alive and responded; UNREACHABLE means it may not even know a goal
  /// was sent.
  UNREACHABLE,
};

std::string to_string(CapabilityState state);

}  // namespace easyfleet

#endif  // EASYFLEET_MISSION_MANAGER__CAPABILITY_STATE_HPP_
