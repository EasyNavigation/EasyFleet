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

"""RunningCapability, mirroring easyfleet_mission_manager's detail/running_capability.hpp."""

import threading
import time

from .. import ansi
from ..capability_client import CapabilityClient, Outcome
from ..capability_state import CapabilityState, to_string
from ..output import safe_print

_OUTCOME_TO_STATE = {
    Outcome.SUCCEEDED: CapabilityState.SUCCEEDED,
    Outcome.ABORTED: CapabilityState.ABORTED,
    Outcome.CANCELED: CapabilityState.CANCELED,
    Outcome.REJECTED: CapabilityState.REJECTED,
    Outcome.SERVER_UNAVAILABLE: CapabilityState.UNREACHABLE,
}


class RunningCapability:
    """
    Tracks one capability_type's in-flight/last-finished goal on a RobotHandle.

    Owns the one CapabilityClient a given (robot, capability_type) pair
    is ever run through -- a later run() call reuses it, sending a fresh
    goal (server-side preemption).
    """

    def __init__(self, node, action_type, action_name, robot_name, capability_type, status):
        self._client = CapabilityClient(node, action_type, action_name)
        self._robot_name = robot_name
        self._capability_type = capability_type
        self._status = status
        self._lock = threading.Lock()
        self._state = CapabilityState.IDLE
        self._deadline = None
        self._generation = 0

    def run(self, goal, timeout_sec: float) -> None:
        """Send `goal` and start tracking it, stopping it after `timeout_sec` if still running."""
        with self._lock:
            self._generation += 1
            generation = self._generation
            self._state = CapabilityState.RUNNING
            self._deadline = time.monotonic() + timeout_sec
        self._status.set_status(self._robot_name, f'Running {self._capability_type}')

        def on_response(response):
            # A new run() call means a new goal supersedes whatever was
            # in flight (server-side preemption). The *client*-side
            # outcome of that superseded goal can still arrive after
            # this point -- only the response whose generation still
            # matches the current one is allowed to update state, so a
            # late-arriving stale outcome can never stomp a newer goal's
            # already-settled state.
            with self._lock:
                if generation != self._generation:
                    return
                new_state = _OUTCOME_TO_STATE.get(response.outcome, CapabilityState.ABORTED)
                self._state = new_state
            self._status.set_status(
                self._robot_name, f'{self._capability_type} -> {to_string(new_state)}')
            safe_print(
                f'  {ansi.DIM}[{self._robot_name}/{self._capability_type}] {ansi.RESET}'
                f'finished with outcome {ansi.MAGENTA}{to_string(new_state)}{ansi.RESET}')

        self._client.request(goal, on_response)

    def stop(self) -> None:
        self._client.cancel()

    @property
    def state(self) -> CapabilityState:
        with self._lock:
            return self._state

    def check_timeout(self) -> None:
        """Cancel the in-flight goal if `timeout_sec` (from the last run()) has elapsed."""
        with self._lock:
            state = self._state
            deadline = self._deadline
        timed_out = deadline is not None and time.monotonic() >= deadline
        if state == CapabilityState.RUNNING and timed_out:
            self._client.cancel()
