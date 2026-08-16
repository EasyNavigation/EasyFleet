# Copyright 2026 Intelligent Robotics Lab
#
# This file is part of the project EasyFleet
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""CapabilityState, mirroring easyfleet_mission_manager's capability_state.hpp."""

import enum


class CapabilityState(enum.Enum):
    """
    What a RobotHandle-driven capability is doing right now.

    Deliberately richer than a plain running/not-running bool -- see the
    C++ CapabilityState's own doc comment for the full rationale. Every
    terminal value mirrors easyfleet_core::CapabilityClient<ActionT>::Outcome
    (capability_client.Outcome here) one to one, via
    detail.running_capability's own outcome-to-state mapping.
    """

    IDLE = 'IDLE'
    RUNNING = 'RUNNING'
    SUCCEEDED = 'SUCCEEDED'
    ABORTED = 'ABORTED'
    CANCELED = 'CANCELED'
    REJECTED = 'REJECTED'
    TIMEOUT = 'TIMEOUT'
    UNREACHABLE = 'UNREACHABLE'


def to_string(state: CapabilityState) -> str:
    return state.value
