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
    Outcome.TIMEOUT: CapabilityState.TIMEOUT,
    Outcome.SERVER_UNAVAILABLE: CapabilityState.UNREACHABLE,
}

# How long a canceled goal may take to report back before it is given up on
# and settled locally as TIMEOUT.
CANCEL_GRACE_SEC = 5.0


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
        self._cancel_deadline = None
        # Incremented on every run() and every local settle: a late result
        # of a superseded or given-up goal must not overwrite the state.
        self._generation = 0

    def run(self, goal, timeout_sec: float) -> None:
        """Send `goal` and start tracking it, stopping it after `timeout_sec` if still running."""
        with self._lock:
            self._generation += 1
            generation = self._generation
            self._state = CapabilityState.RUNNING
            self._deadline = time.monotonic() + timeout_sec
            self._cancel_deadline = None
        self._status.set_status(self._robot_name, f'Running {self._capability_type}')
        self._client.request(
            goal,
            lambda response: self._settle(generation, _OUTCOME_TO_STATE.get(
                response.outcome, CapabilityState.ABORTED)))

    def stop(self) -> None:
        """Ask the running goal, if any, to finish early; the cancel is sent once."""
        if self.state == CapabilityState.RUNNING:
            self._request_cancel()

    @property
    def state(self) -> CapabilityState:
        with self._lock:
            return self._state

    def check_timeout(self, alive: bool) -> None:
        """
        Enforce the run() timeout and the capability's liveness.

        Cancels the goal once when its timeout elapses; settles it as
        TIMEOUT if the cancel isn't answered within CANCEL_GRACE_SEC, and
        as UNREACHABLE as soon as the capability stops sending heartbeats.
        """
        with self._lock:
            if self._state != CapabilityState.RUNNING:
                return
            deadline = self._deadline
            cancel_deadline = self._cancel_deadline
        now = time.monotonic()
        if not alive:
            self._settle(None, CapabilityState.UNREACHABLE)
        elif cancel_deadline is not None and now >= cancel_deadline:
            self._settle(None, CapabilityState.TIMEOUT)
        elif cancel_deadline is None and deadline is not None and now >= deadline:
            self._request_cancel()

    def _request_cancel(self) -> None:
        with self._lock:
            if self._cancel_deadline is not None:
                return
            self._cancel_deadline = time.monotonic() + CANCEL_GRACE_SEC
        self._client.cancel()

    def _settle(self, generation, new_state: CapabilityState) -> None:
        """
        Record the final state of the current goal.

        With a `generation` (a result from the network), only if that goal
        is still the current one; without one (a local decision to give up),
        unconditionally, and later results are ignored.
        """
        with self._lock:
            if generation is not None:
                if generation != self._generation:
                    return
            else:
                self._generation += 1
            self._state = new_state
        self._status.set_status(
            self._robot_name, f'{self._capability_type} -> {to_string(new_state)}')
        safe_print(
            f'  {ansi.DIM}[{self._robot_name}/{self._capability_type}] {ansi.RESET}'
            f'finished with outcome {ansi.MAGENTA}{to_string(new_state)}{ansi.RESET}')
