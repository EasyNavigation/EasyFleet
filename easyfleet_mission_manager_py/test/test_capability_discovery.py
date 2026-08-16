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

"""Integration tests for discover_capabilities(), mirroring test_capability_discovery.cpp."""

import itertools
import threading

from easyfleet_interfaces.msg import CapabilityDescription, CapabilityStatus
from easyfleet_mission_manager_py.capability_discovery import discover_capabilities
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

_name_counter = itertools.count()


def _unique_name(base: str) -> str:
    return f'{base}_{next(_name_counter)}'


def _spin_in_background(executor) -> threading.Thread:
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    while not executor.is_spinning:
        pass
    return thread


def _make_description(capability: str, action_name: str, description_json: str, robot: str = ''):
    msg = CapabilityDescription()
    msg.robot = robot
    msg.capability = capability
    msg.action_name = action_name
    msg.description_json = description_json
    return msg


class _Fixture:

    def __init__(self):
        self.consumer_node = rclpy.create_node(_unique_name('test_discovery_consumer'))
        self.publisher_node = rclpy.create_node(_unique_name('test_discovery_publisher'))
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.consumer_node)
        self.executor.add_node(self.publisher_node)
        self.spin_thread = _spin_in_background(self.executor)

        self.capabilities_pub = self.publisher_node.create_publisher(
            CapabilityDescription, '/capabilities',
            QoSProfile(
                depth=10, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.status_pub = self.publisher_node.create_publisher(
            CapabilityStatus, '/capabilities_status',
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self._timers = []

    def start_heartbeat(self, capability: str, action_name: str, robot: str = '', busy=False):
        def publish():
            msg = CapabilityStatus()
            msg.robot = robot
            msg.capability = capability
            msg.action_name = action_name
            msg.busy = busy
            self.status_pub.publish(msg)

        timer = self.publisher_node.create_timer(0.05, publish)
        self._timers.append(timer)
        return timer

    def teardown(self):
        self.executor.shutdown()
        if self.spin_thread.is_alive():
            self.spin_thread.join()
        self.consumer_node.destroy_node()
        self.publisher_node.destroy_node()


@pytest.fixture
def fixture():
    rclpy.init()
    f = _Fixture()
    yield f
    f.teardown()
    rclpy.shutdown()


def test_returns_empty_when_nothing_is_published(fixture):
    result = discover_capabilities(fixture.consumer_node, window_sec=0.3)
    assert result == []


def test_parses_json_and_marks_active_when_heartbeat_seen(fixture):
    fixture.capabilities_pub.publish(
        _make_description(
            'fake_cap', '/fake_cap', '{"name":"fake_cap","display_name":"Fake Capability"}'))
    fixture.start_heartbeat('fake_cap', '/fake_cap')

    result = discover_capabilities(fixture.consumer_node, window_sec=0.8)

    assert len(result) == 1
    info = result[0]
    assert info.capability == 'fake_cap'
    assert info.action_name == '/fake_cap'
    assert info.description_json_valid is True
    assert info.active is True
    assert info.busy is False
    assert info.description_json['display_name'] == 'Fake Capability'


def test_tracks_busy_state_from_the_latest_heartbeat(fixture):
    fixture.capabilities_pub.publish(_make_description('fake_cap', '/fake_cap', '{}'))
    fixture.start_heartbeat('fake_cap', '/fake_cap', busy=True)

    result = discover_capabilities(fixture.consumer_node, window_sec=0.8)

    assert len(result) == 1
    assert result[0].active is True
    assert result[0].busy is True


def test_capability_without_heartbeat_is_not_active(fixture):
    fixture.capabilities_pub.publish(_make_description('silent_cap', '/silent_cap', '{}'))

    result = discover_capabilities(fixture.consumer_node, window_sec=0.5)

    assert len(result) == 1
    assert result[0].capability == 'silent_cap'
    assert result[0].description_json_valid is True
    assert result[0].active is False


def test_malformed_json_still_registers_the_capability_but_flags_it_invalid(fixture):
    fixture.capabilities_pub.publish(
        _make_description('broken_cap', '/broken_cap', 'not valid json {{{'))

    result = discover_capabilities(fixture.consumer_node, window_sec=0.3)

    assert len(result) == 1
    assert result[0].capability == 'broken_cap'
    assert result[0].description_json_valid is False


def test_discovers_multiple_distinct_capabilities(fixture):
    fixture.capabilities_pub.publish(_make_description('cap_a', '/cap_a', '{}'))
    fixture.capabilities_pub.publish(_make_description('cap_b', '/cap_b', '{}'))
    fixture.start_heartbeat('cap_a', '/cap_a')

    result = discover_capabilities(fixture.consumer_node, window_sec=0.8)

    assert len(result) == 2
    by_action_name = {info.action_name: info for info in result}
    assert by_action_name['/cap_a'].active is True
    assert by_action_name['/cap_b'].active is False


def test_same_capability_from_two_robots_are_kept_distinct_by_action_name(fixture):
    fixture.capabilities_pub.publish(
        _make_description('navigation', '/robot1/navigation', '{}', robot='robot1'))
    fixture.capabilities_pub.publish(
        _make_description('navigation', '/robot2/navigation', '{}', robot='robot2'))
    fixture.start_heartbeat('navigation', '/robot1/navigation', robot='robot1')
    fixture.start_heartbeat('navigation', '/robot2/navigation', robot='robot2')

    result = discover_capabilities(fixture.consumer_node, window_sec=0.8)

    assert len(result) == 2
    for info in result:
        assert info.capability == 'navigation'
        assert info.active is True
        assert info.robot in ('robot1', 'robot2')
    assert result[0].action_name != result[1].action_name
