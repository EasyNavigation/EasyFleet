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
Integration tests for Capability, against real /capabilities(_status) topics.

The action server it hosts is a trivial ActionServerBase subclass (using
Navigation purely as a convenient, already-available action type) --
these tests exercise Capability's own lifecycle/publishing behavior, not
action semantics (see test_action_server_base.py for that).
"""

import itertools
import json
import threading
import time

from action_msgs.msg import GoalStatus
from easyfleet_interfaces.action import Navigation
from easyfleet_interfaces.msg import CapabilityDescription, CapabilityStatus
from easyfleet_mission_manager_py.action_server_base import ActionServerBase
from easyfleet_mission_manager_py.capability import Capability, spin_capability
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

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


class _ControllableActionServer(ActionServerBase):
    """Accepts every goal and blocks in on_execute() until `release` is set."""

    def __init__(self, node, action_name):
        super().__init__(node, Navigation, action_name)
        self.release = threading.Event()

    def on_goal_received(self, goal) -> bool:
        return True

    def on_execute(self, goal_handle):
        deadline = time.monotonic() + 5.0
        while not self.release.wait(timeout=0.02) and time.monotonic() < deadline:
            if self.is_shutdown_requested():
                goal_handle.abort()
                return Navigation.Result()
        goal_handle.succeed()
        return Navigation.Result()


class _Harness:
    """Hosts one Capability(namespaced) on its own executor/thread, plus a plain listener node."""

    def __init__(self, capability_name='test_cap', robot_name=None):
        self.robot_name = robot_name or _unique_name('robot')
        self.node = Capability(
            capability_name, _ControllableActionServer,
            namespace=f'/{self.robot_name}')

        self.listener = rclpy.create_node(
            _unique_name('listener'), start_parameter_services=False)
        self.descriptions = []
        self.statuses = []
        self.listener.create_subscription(
            CapabilityDescription, '/capabilities', self.descriptions.append,
            QoSProfile(
                depth=10, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.listener.create_subscription(
            CapabilityStatus, '/capabilities_status', self.statuses.append,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))

        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.node)
        self.executor.add_node(self.listener)
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()
        while not self.executor.is_spinning:
            time.sleep(0)

        # Deliberately NOT added to `self.executor`: a node already spun by
        # a background executor must not also be driven directly via
        # rclpy.spin_until_future_complete() from the test thread (they
        # fight over the same node's callbacks) -- this one is only ever
        # driven that way, so it stays off the shared executor.
        self.client_node = rclpy.create_node(
            _unique_name('client'), start_parameter_services=False)

    def set_capabilities_file(self, path: str) -> None:
        self.node.set_parameters([rclpy.parameter.Parameter('capabilities_file', value=path)])

    def shutdown(self) -> None:
        self.executor.shutdown()
        if self.spin_thread.is_alive():
            self.spin_thread.join(timeout=5.0)
        self.node.destroy_node()
        self.listener.destroy_node()
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


@pytest.fixture
def capabilities_file(tmp_path):
    path = tmp_path / 'test_cap.json'
    path.write_text(json.dumps({'display_name': 'Test Capability'}))
    return str(path)


def test_identity_fields_resolved_from_namespace(harness):
    assert harness.node.get_capability_name() == 'test_cap'
    assert harness.node.get_robot_name() == harness.robot_name
    assert harness.node.get_resolved_action_name() == f'/{harness.robot_name}/test_cap'


def test_activate_fails_without_capabilities_file_param(harness):
    assert harness.node.trigger_configure() == TransitionCallbackReturn.SUCCESS
    assert harness.node.trigger_activate() == TransitionCallbackReturn.FAILURE
    assert harness.descriptions == []


def test_activate_fails_when_capabilities_file_does_not_exist(harness):
    harness.set_capabilities_file('/nonexistent/path/to/some.json')
    assert harness.node.trigger_configure() == TransitionCallbackReturn.SUCCESS
    assert harness.node.trigger_activate() == TransitionCallbackReturn.FAILURE
    assert harness.descriptions == []


def test_activate_publishes_capability_description(harness, capabilities_file):
    harness.set_capabilities_file(capabilities_file)
    assert harness.node.trigger_configure() == TransitionCallbackReturn.SUCCESS
    assert harness.node.trigger_activate() == TransitionCallbackReturn.SUCCESS

    assert _wait_until(lambda: len(harness.descriptions) >= 1, timeout_sec=5.0)
    description = harness.descriptions[0]
    assert description.robot == harness.robot_name
    assert description.capability == 'test_cap'
    assert description.action_name == f'/{harness.robot_name}/test_cap'
    assert json.loads(description.description_json) == {'display_name': 'Test Capability'}


def test_heartbeat_reports_busy_while_a_goal_executes(harness, capabilities_file):
    harness.set_capabilities_file(capabilities_file)
    harness.node.trigger_configure()
    harness.node.trigger_activate()

    assert _wait_until(lambda: len(harness.statuses) >= 1, timeout_sec=5.0)
    assert harness.statuses[-1].busy is False

    action_server = harness.node.get_action_server()
    client = ActionClient(
        harness.client_node, Navigation, harness.node.get_resolved_action_name())
    assert client.wait_for_server(timeout_sec=5.0)
    goal_future = client.send_goal_async(Navigation.Goal())
    rclpy.spin_until_future_complete(harness.client_node, goal_future, timeout_sec=5.0)
    assert goal_future.result().accepted

    assert _wait_until(lambda: harness.statuses[-1].busy is True, timeout_sec=5.0)

    action_server.release.set()
    assert _wait_until(lambda: harness.statuses[-1].busy is False, timeout_sec=5.0)
    client.destroy()


def test_deactivate_stops_the_heartbeat(harness, capabilities_file):
    harness.set_capabilities_file(capabilities_file)
    harness.node.trigger_configure()
    harness.node.trigger_activate()
    assert _wait_until(lambda: len(harness.statuses) >= 1, timeout_sec=5.0)

    assert harness.node.trigger_deactivate() == TransitionCallbackReturn.SUCCESS
    count_after_deactivate = len(harness.statuses)
    time.sleep(1.5)
    assert len(harness.statuses) == count_after_deactivate


def _send_goal(harness):
    client = ActionClient(
        harness.client_node, Navigation, harness.node.get_resolved_action_name())
    assert client.wait_for_server(timeout_sec=5.0)
    future = client.send_goal_async(Navigation.Goal())
    rclpy.spin_until_future_complete(harness.client_node, future, timeout_sec=5.0)
    return client, future.result()


def test_goals_are_rejected_until_the_capability_is_active(harness, capabilities_file):
    client, handle = _send_goal(harness)
    assert not handle.accepted, 'unconfigured'

    harness.set_capabilities_file(capabilities_file)
    harness.node.trigger_configure()
    handle = client.send_goal_async(Navigation.Goal())
    rclpy.spin_until_future_complete(harness.client_node, handle, timeout_sec=5.0)
    assert not handle.result().accepted, 'inactive'

    harness.node.trigger_activate()
    harness.node.get_action_server().release.set()
    handle = client.send_goal_async(Navigation.Goal())
    rclpy.spin_until_future_complete(harness.client_node, handle, timeout_sec=5.0)
    assert handle.result().accepted
    client.destroy()


def test_failed_activation_keeps_goals_rejected(harness):
    harness.set_capabilities_file('/no/such/file.json')
    harness.node.trigger_configure()
    assert harness.node.trigger_activate() != TransitionCallbackReturn.SUCCESS
    client, handle = _send_goal(harness)
    assert not handle.accepted
    time.sleep(0.5)
    assert [d for d in harness.descriptions if d.robot == harness.robot_name] == []
    client.destroy()


def test_deactivate_stops_the_running_goal_and_rejects_new_ones(harness, capabilities_file):
    harness.set_capabilities_file(capabilities_file)
    harness.node.trigger_configure()
    harness.node.trigger_activate()
    client, handle = _send_goal(harness)
    assert handle.accepted
    action_server = harness.node.get_action_server()
    assert _wait_until(action_server.is_active, timeout_sec=2.0)

    start = time.monotonic()
    assert harness.node.trigger_deactivate() == TransitionCallbackReturn.SUCCESS
    assert time.monotonic() - start < 2.0
    assert not action_server.is_active()

    result_future = handle.get_result_async()
    rclpy.spin_until_future_complete(harness.client_node, result_future, timeout_sec=5.0)
    assert result_future.result().status == GoalStatus.STATUS_ABORTED

    again = client.send_goal_async(Navigation.Goal())
    rclpy.spin_until_future_complete(harness.client_node, again, timeout_sec=5.0)
    assert not again.result().accepted
    client.destroy()


def test_heartbeat_keeps_ticking_with_sim_time_and_no_clock(capabilities_file):
    # A paused simulation (no /clock) must not make a capability look dead.
    h = _Harness()
    try:
        h.node.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        h.set_capabilities_file(capabilities_file)
        h.node.trigger_configure()
        h.node.trigger_activate()
        assert _wait_until(lambda: len(h.statuses) >= 2, timeout_sec=4.0)
    finally:
        h.shutdown()


def test_spin_capability_reports_the_real_failure(tmp_path):
    node = Capability(_unique_name('spin_cap'), _ControllableActionServer)
    node.set_parameters(
        [rclpy.parameter.Parameter('capabilities_file', value=str(tmp_path / 'missing.json'))])
    with pytest.raises(RuntimeError, match='activate failed'):
        spin_capability(node)
