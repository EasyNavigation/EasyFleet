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

/// @brief What a `RobotHandle`-driven capability is doing right now, from the
/// mission script's point of view. Deliberately richer than a plain
/// running/not-running bool: `is_capability_running()` alone can't tell a
/// beginner's mission script "it finished, and here's what actually
/// happened" apart from "nothing is happening because it crashed" -- very
/// different situations that all look like "not running" if that's all you
/// can ask. Every terminal value below mirrors
/// `easyfleet_core::CapabilityClient<ActionT>::Outcome` one to one
/// (`RunningCapability`'s own doc comment maps them) -- the real, specific
/// outcome is what gets reported, never collapsed into a generic
/// success/failure bit.
enum class CapabilityState
{
  /// @brief Never asked to do anything (or done, superseded, and not re-run).
  IDLE,
  /// @brief A goal is in flight.
  RUNNING,
  /// @brief The last goal finished successfully.
  SUCCEEDED,
  /// @brief The last goal was aborted -- it started, then failed on its own.
  ABORTED,
  /// @brief The last goal was stopped -- either explicitly, via
  /// `RobotHandle::stop_capability()`, or automatically, by its own
  /// `run_capability()` timeout elapsing. Distinct from `ABORTED`/`REJECTED`:
  /// this is an expected, deliberate outcome (a mission script asked for
  /// it, directly or via a timeout it chose), not the capability itself
  /// going wrong.
  CANCELED,
  /// @brief The last goal was rejected outright -- the capability never
  /// even started it (invalid/unsupported goal content, or busy and not
  /// preemptable).
  REJECTED,
  /// @brief The underlying `ActionClient`'s own send/response mechanics
  /// timed out -- distinct from a `run_capability()` timeout, which
  /// produces `CANCELED` instead (that path issues an explicit cancel
  /// rather than the client giving up on its own).
  TIMEOUT,
  /// @brief No heartbeat seen recently on `/capabilities_status` for this
  /// capability -- the process is gone, was never there, or has stalled.
  /// Distinct from every other terminal value above: those all mean the
  /// capability itself is alive and responded; UNREACHABLE means it may
  /// not even know a goal was sent.
  UNREACHABLE,
};

std::string to_string(CapabilityState state);

}  // namespace easyfleet

#endif  // EASYFLEET_MISSION_MANAGER__CAPABILITY_STATE_HPP_
