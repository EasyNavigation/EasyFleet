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
Integration tests for ActionServerBase, against a real rclpy ActionClient.

Uses the easyfleet_interfaces/Navigation action purely as a convenient,
already-available action type -- these tests exercise ActionServerBase's
own goal/preemption/cancel/shutdown machinery, not navigation semantics.
Mirrors this package's own test_nav_fake_capability.py pattern: the
server-side node is spun on its own background executor/thread, like a
real robot process, while the test talks to it through a normal
ActionClient.
"""

import itertools
import threading
import time

from action_msgs.msg import GoalStatus
from easyfleet_interfaces.action import Navigation
from easyfleet_mission_manager_py.action_server_base import ActionServerBase
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

_name_counter = itertools.count()


def _unique_name(base: str) -> str:
    return f'{base}_{next(_name_counter)}'


def _wait_until(condition, timeout_sec: float) -> bool:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if condition():
            return True
        time.sleep(0.01)
    return condition()


REJECT_MARKER = '__reject__'


class _RecordingActionServer(ActionServerBase):
    """Records every hook call so tests can assert on them; settles quickly by default."""

    def __init__(self, node, action_name, *, step_sec=0.02, **kwargs):
        super().__init__(node, Navigation, action_name, **kwargs)
        self.step_sec = step_sec
        self.preempted_goals = []
        self.settle_goal = True  # if False, on_execute deliberately never settles.
        self.raise_in_execute = False
        self.raise_in_goal_received = False
        self._running_lock = threading.Lock()
        self.running = 0
        self.max_running = 0
        self.started = 0

    def on_goal_received(self, goal) -> bool:
        if self.raise_in_goal_received:
            raise RuntimeError('bad goal')
        return REJECT_MARKER not in goal.parameters_json

    def on_execute(self, goal_handle):
        with self._running_lock:
            self.started += 1
            self.running += 1
            self.max_running = max(self.max_running, self.running)
        try:
            return self._execute(goal_handle)
        finally:
            with self._running_lock:
                self.running -= 1

    def _execute(self, goal_handle):
        if self.raise_in_execute:
            raise RuntimeError('boom')
        poll_sec = 0.02
        remaining = self.step_sec
        while remaining > 0.0:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return Navigation.Result()
            if self.is_preempt_requested() or self.is_shutdown_requested():
                goal_handle.abort()
                return Navigation.Result()
            if not self.settle_goal:
                return None  # Deliberately misbehave: never settle the goal.
            time.sleep(poll_sec)
            remaining -= poll_sec
        goal_handle.succeed()
        return Navigation.Result()

    def on_preempted(self, preempted_goal_handle) -> None:
        self.preempted_goals.append(preempted_goal_handle)


class _Harness:
    """Hosts one _RecordingActionServer on its own executor/thread, plus a client to talk to it."""

    def __init__(self, **server_kwargs):
        self.action_name = f'/{_unique_name("test_action")}'
        self.server_node = rclpy.create_node(
            _unique_name('server'), start_parameter_services=False)
        self.server = _RecordingActionServer(self.server_node, self.action_name, **server_kwargs)

        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.server_node)
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()
        while not self.executor.is_spinning:
            time.sleep(0)

        self.client_node = rclpy.create_node(
            _unique_name('client'), start_parameter_services=False)
        self.client = ActionClient(self.client_node, Navigation, self.action_name)
        assert self.client.wait_for_server(timeout_sec=5.0)

    def send_goal(self, parameters_json: str = ''):
        """Send a goal; returns the accepted (or rejected) client-side goal handle."""
        goal = Navigation.Goal()
        goal.parameters_json = parameters_json
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.client_node, future, timeout_sec=5.0)
        assert future.done()
        return future.result()

    def get_result(self, goal_handle):
        future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.client_node, future, timeout_sec=5.0)
        assert future.done()
        return future.result()

    def shutdown(self) -> None:
        self.server.destroy()
        self.executor.shutdown()
        if self.spin_thread.is_alive():
            self.spin_thread.join(timeout=5.0)
        self.server_node.destroy_node()
        self.client.destroy()
        self.client_node.destroy_node()


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def harness():
    h = _Harness()
    yield h
    h.shutdown()


def test_accepts_and_succeeds_a_normal_goal(harness):
    handle = harness.send_goal()
    assert handle.accepted
    result = harness.get_result(handle)
    assert result.status == GoalStatus.STATUS_SUCCEEDED


def test_rejects_goal_when_on_goal_received_returns_false(harness):
    handle = harness.send_goal(REJECT_MARKER)
    assert not handle.accepted


def test_cancel_settles_as_canceled(harness):
    harness.server.step_sec = 5.0  # Long enough to reliably cancel before it settles itself.
    handle = harness.send_goal()
    assert handle.accepted
    assert _wait_until(harness.server.is_active, timeout_sec=2.0)

    cancel_future = handle.cancel_goal_async()
    rclpy.spin_until_future_complete(harness.client_node, cancel_future, timeout_sec=5.0)

    result = harness.get_result(handle)
    assert result.status == GoalStatus.STATUS_CANCELED


def test_rejects_new_goal_when_busy_and_preemption_disabled():
    h = _Harness(default_allow_preemption=False)
    try:
        h.server.step_sec = 2.0
        first = h.send_goal()
        assert first.accepted
        assert _wait_until(h.server.is_active, timeout_sec=2.0)

        second = h.send_goal()
        assert not second.accepted

        result = h.get_result(first)
        assert result.status == GoalStatus.STATUS_SUCCEEDED
    finally:
        h.shutdown()


def test_preempts_active_goal_when_preemption_enabled(harness):
    harness.server.step_sec = 2.0
    first = harness.send_goal()
    assert first.accepted
    assert _wait_until(harness.server.is_active, timeout_sec=2.0)

    harness.server.step_sec = 0.02
    second = harness.send_goal()
    assert second.accepted

    first_result = harness.get_result(first)
    assert first_result.status == GoalStatus.STATUS_ABORTED
    assert len(harness.server.preempted_goals) == 1

    second_result = harness.get_result(second)
    assert second_result.status == GoalStatus.STATUS_SUCCEEDED


def test_is_active_false_when_idle_true_while_executing(harness):
    assert harness.server.is_active() is False
    harness.server.step_sec = 1.0
    handle = harness.send_goal()
    assert _wait_until(harness.server.is_active, timeout_sec=2.0)
    harness.get_result(handle)
    assert _wait_until(lambda: not harness.server.is_active(), timeout_sec=2.0)


def test_exception_in_on_execute_aborts_the_goal(harness):
    harness.server.raise_in_execute = True
    handle = harness.send_goal()
    assert handle.accepted
    result = harness.get_result(handle)
    assert result.status == GoalStatus.STATUS_ABORTED


def test_on_execute_returning_without_settling_aborts_the_goal(harness):
    harness.server.settle_goal = False
    handle = harness.send_goal()
    assert handle.accepted
    result = harness.get_result(handle)
    assert result.status == GoalStatus.STATUS_ABORTED


def test_destroy_is_idempotent_and_marks_shutdown_requested(harness):
    harness.server.destroy()
    assert harness.server.is_shutdown_requested()
    harness.server.destroy()  # must not raise the second time


def test_preempted_and_new_goal_never_run_at_the_same_time(harness):
    harness.server.step_sec = 1.0
    first = harness.send_goal()
    assert _wait_until(harness.server.is_active, timeout_sec=2.0)
    second = harness.send_goal()
    assert second.accepted
    assert harness.get_result(first).status == GoalStatus.STATUS_ABORTED
    harness.get_result(second)
    assert harness.server.max_running == 1


def test_burst_of_goals_with_preemption_disabled_accepts_only_one():
    h = _Harness(default_allow_preemption=False)
    try:
        h.server.step_sec = 1.0
        goal = Navigation.Goal()
        futures = [h.client.send_goal_async(goal) for _ in range(5)]
        for future in futures:
            rclpy.spin_until_future_complete(h.client_node, future, timeout_sec=5.0)
        assert sum(1 for future in futures if future.result().accepted) == 1
    finally:
        h.shutdown()


def test_goal_queued_behind_a_preempted_one_is_superseded_by_a_newer_one(harness):
    harness.server.step_sec = 1.0
    first = harness.send_goal()
    assert _wait_until(harness.server.is_active, timeout_sec=2.0)
    second = harness.send_goal()
    third = harness.send_goal()
    assert second.accepted and third.accepted
    assert harness.get_result(first).status == GoalStatus.STATUS_ABORTED
    harness.server.step_sec = 0.02
    assert harness.get_result(third).status == GoalStatus.STATUS_SUCCEEDED
    # The second never ran, or was aborted as soon as it started.
    assert harness.get_result(second).status == GoalStatus.STATUS_ABORTED
    assert harness.server.max_running == 1


def test_exception_in_on_goal_received_rejects_and_server_keeps_working(harness):
    harness.server.raise_in_goal_received = True
    assert not harness.send_goal().accepted
    harness.server.raise_in_goal_received = False
    handle = harness.send_goal()
    assert handle.accepted
    assert harness.get_result(handle).status == GoalStatus.STATUS_SUCCEEDED


def test_disable_goals_stops_the_running_goal_and_rejects_new_ones(harness):
    harness.server.step_sec = 30.0
    running = harness.send_goal()
    assert _wait_until(harness.server.is_active, timeout_sec=2.0)

    start = time.monotonic()
    harness.server.disable_goals()
    assert time.monotonic() - start < 2.0
    assert not harness.server.is_active()
    assert not harness.server.goals_enabled()
    assert harness.get_result(running).status == GoalStatus.STATUS_ABORTED
    assert not harness.send_goal().accepted

    harness.server.enable_goals()
    harness.server.step_sec = 0.02
    handle = harness.send_goal()
    assert handle.accepted
    assert harness.get_result(handle).status == GoalStatus.STATUS_SUCCEEDED


def test_disable_goals_while_idle_returns_at_once(harness):
    start = time.monotonic()
    harness.server.disable_goals()
    harness.server.disable_goals()
    assert time.monotonic() - start < 0.2
    assert not harness.server.is_shutdown_requested()


def test_destroyed_server_cannot_be_re_enabled(harness):
    harness.server.destroy()
    harness.server.enable_goals()
    assert not harness.server.goals_enabled()
