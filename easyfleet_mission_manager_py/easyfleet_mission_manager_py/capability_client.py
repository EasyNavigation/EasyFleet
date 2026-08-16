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

"""
CapabilityClient, mirroring easyfleet_core's CapabilityClient<ActionT>/ActionClient<ActionT>.

The C++ original splits these into two layers (a generic ActionClient
that owns its own internal node/executor/thread, and a thin
CapabilityClient wrapper over it) because rclcpp_action's own client
needs *some* node spinning it, and a caller's own node might not be
spinning yet. In Python, a FleetSession already spins one node's
executor in the background for its whole life (see fleet_session.py),
so every CapabilityClient here is created on -- and serviced by -- that
same node/executor: no per-client internal node/thread is needed, which
is why this is a single class instead of two.
"""

import enum
import threading

from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient


class Outcome(enum.Enum):
    """The action's own terminal states, plus rejection and server-unavailability."""

    SUCCEEDED = 'SUCCEEDED'
    ABORTED = 'ABORTED'
    CANCELED = 'CANCELED'
    REJECTED = 'REJECTED'
    SERVER_UNAVAILABLE = 'SERVER_UNAVAILABLE'


_STATUS_TO_OUTCOME = {
    GoalStatus.STATUS_SUCCEEDED: Outcome.SUCCEEDED,
    GoalStatus.STATUS_CANCELED: Outcome.CANCELED,
}


class Response:
    """Final outcome of a request, as delivered to a `request()` response callback."""

    __slots__ = ('outcome', 'result')

    def __init__(self, outcome: Outcome, result=None):
        self.outcome = outcome
        self.result = result


class CapabilityClient:
    """
    The simplest possible way to ask a capability to do something.

    Talks to a capability by name, exactly as advertised by
    easyfleet_core::Capability (the capability name is the action name).
    """

    def __init__(self, node, action_type, capability_name: str):
        self._action_name = capability_name
        self._client = ActionClient(node, action_type, capability_name)
        self._lock = threading.Lock()
        self._active_goal_handle = None
        # Unlike rclcpp_action::Client's async_cancel_all_goals() (which
        # rclpy has no equivalent of), rclpy can only cancel a specific,
        # already-accepted GoalHandle -- so cancel() arriving before the
        # goal-response callback has run (a real race: is_capability_running()
        # already reads RUNNING synchronously from run(), before the goal
        # is even accepted) has nothing to cancel yet. This flag makes
        # that cancel() request stick: _on_goal_response() checks it and
        # cancels immediately once a handle actually exists.
        self._cancel_requested = False

    @property
    def action_name(self) -> str:
        return self._action_name

    def request(self, goal, on_response=None, on_feedback=None) -> None:
        """
        Ask the capability to do something. Returns immediately.

        `on_response` is called exactly once with the final outcome;
        `on_feedback` (if given) once per feedback message while the
        request is running.
        """
        if not self._client.server_is_ready():
            if on_response:
                on_response(Response(Outcome.SERVER_UNAVAILABLE))
            return

        with self._lock:
            self._cancel_requested = False

        def unwrap_feedback(feedback_msg):
            on_feedback(feedback_msg.feedback)

        send_future = self._client.send_goal_async(
            goal, feedback_callback=unwrap_feedback if on_feedback else None)
        send_future.add_done_callback(
            lambda future: self._on_goal_response(future, on_response))

    def _on_goal_response(self, future, on_response) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            if on_response:
                on_response(Response(Outcome.REJECTED))
            return

        cancel_now = False
        with self._lock:
            self._active_goal_handle = goal_handle
            cancel_now = self._cancel_requested
        if cancel_now:
            goal_handle.cancel_goal_async()

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future: self._on_result(future, goal_handle, on_response))

    def _on_result(self, future, goal_handle, on_response) -> None:
        with self._lock:
            if self._active_goal_handle is goal_handle:
                self._active_goal_handle = None
        response = future.result()
        outcome = _STATUS_TO_OUTCOME.get(response.status, Outcome.ABORTED)
        if on_response:
            on_response(Response(outcome, response.result))

    def cancel(self) -> None:
        """Ask the capability to stop whatever it is currently doing, if anything."""
        with self._lock:
            self._cancel_requested = True
            goal_handle = self._active_goal_handle
        if goal_handle is not None:
            goal_handle.cancel_goal_async()
