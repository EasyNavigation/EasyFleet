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

#include "easyfleet_mission_manager/capability_state.hpp"

namespace easyfleet
{

std::string to_string(CapabilityState state)
{
  switch (state) {
    case CapabilityState::IDLE: return "IDLE";
    case CapabilityState::RUNNING: return "RUNNING";
    case CapabilityState::SUCCEEDED: return "SUCCEEDED";
    case CapabilityState::ABORTED: return "ABORTED";
    case CapabilityState::CANCELED: return "CANCELED";
    case CapabilityState::REJECTED: return "REJECTED";
    case CapabilityState::TIMEOUT: return "TIMEOUT";
    case CapabilityState::UNREACHABLE: return "UNREACHABLE";
  }
  return "UNKNOWN";
}

}  // namespace easyfleet
