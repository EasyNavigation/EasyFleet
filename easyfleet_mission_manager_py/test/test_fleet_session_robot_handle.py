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

"""Integration tests for FleetSession/RobotHandle/SimpleController, mirroring the C++ suite."""

import itertools
import time

from easyfleet_mission_manager_py.capability_state import CapabilityState
from easyfleet_mission_manager_py.fleet_session import FleetSession
from easyfleet_mission_manager_py.mission_helpers import (
    make_manipulation_goal,
    make_navigation_goal,
    Manipulation,
    Navigation,
)
from easyfleet_mission_manager_py.robot_handle import MissionManagerError, RobotHandle
from easyfleet_mission_manager_py.simple_controller import SimpleController
import pytest
import rclpy

from .test_nav_fake_capability import FakeNavCapabilityProcess, REJECT_WAYPOINT_ID

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


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def session():
    s = FleetSession()
    yield s
    s._teardown()


# --- RobotHandle, standalone (no FleetSession needed) ---

def test_name_returns_what_was_constructed_with():
    robot = RobotHandle(_unique_name('robot'))
    assert robot.name != ''


def test_has_no_capabilities_before_any_discovery():
    robot = RobotHandle(_unique_name('robot'))
    assert robot.has_capability('navigation') is False
    assert robot.capabilities == []


def test_capability_state_is_idle_before_ever_running():
    robot = RobotHandle(_unique_name('robot'))
    assert robot.capability_state('navigation') == CapabilityState.IDLE
    assert robot.is_capability_running('navigation') is False


def test_is_alive_is_false_without_any_heartbeat_seen():
    robot = RobotHandle(_unique_name('robot'))
    assert robot.is_alive('navigation') is False


def test_run_capability_on_a_handle_never_added_to_a_fleet_session_raises():
    # Never add_robot()-ed to a FleetSession -- must fail loudly, not
    # silently no-op, since that's otherwise indistinguishable from a
    # properly attached robot that simply hasn't announced this
    # capability yet.
    robot = RobotHandle(_unique_name('robot'))
    with pytest.raises(MissionManagerError):
        robot.run_capability('navigation', Navigation, make_navigation_goal('kitchen'))


# --- FleetSession, without a real robot process ---

def test_find_robot_returns_none_before_adding(session):
    assert session.find_robot('robot_1') is None
    assert session.robots == []


def test_add_robot_makes_it_findable_and_attached(session):
    robot = RobotHandle(_unique_name('robot'))
    session.add_robot(robot)

    assert len(session.robots) == 1
    found = session.find_robot(robot.name)
    assert found is robot


# --- FleetSession + RobotHandle + a real (fake) robot process ---

def test_discover_run_and_succeed(session):
    robot_name = _unique_name('robot')
    fake_robot = FakeNavCapabilityProcess(robot_name, mock_duration_sec=0.05)
    try:
        robot = RobotHandle(robot_name)
        session.add_robot(robot)

        session.discover_capabilities()
        assert robot.has_capability('navigation') is True

        robot.run_capability(
            'navigation', Navigation, make_navigation_goal('kitchen'), timeout_sec=5.0)
        assert robot.is_capability_running('navigation') is True

        assert _wait_until(
            lambda: not robot.is_capability_running('navigation'), timeout_sec=3.0)
        assert robot.capability_state('navigation') == CapabilityState.SUCCEEDED
    finally:
        fake_robot.shutdown()


def test_run_capability_with_a_mismatched_action_type_raises(session):
    robot_name = _unique_name('robot')
    fake_robot = FakeNavCapabilityProcess(robot_name, mock_duration_sec=30.0)
    try:
        robot = RobotHandle(robot_name)
        session.add_robot(robot)
        session.discover_capabilities()
        assert robot.has_capability('navigation') is True

        robot.run_capability(
            'navigation', Navigation, make_navigation_goal('kitchen'), timeout_sec=30.0)
        assert _wait_until(
            lambda: robot.is_capability_running('navigation'), timeout_sec=0.5)

        with pytest.raises(MissionManagerError):
            robot.run_capability(
                'navigation', Manipulation, make_manipulation_goal(), timeout_sec=5.0)

        robot.stop_capability('navigation')
    finally:
        fake_robot.shutdown()


def test_stop_capability_cancels_a_long_running_goal(session):
    robot_name = _unique_name('robot')
    fake_robot = FakeNavCapabilityProcess(robot_name, mock_duration_sec=30.0)
    try:
        robot = RobotHandle(robot_name)
        session.add_robot(robot)
        session.discover_capabilities()
        assert robot.has_capability('navigation') is True

        robot.run_capability(
            'navigation', Navigation, make_navigation_goal('kitchen'), timeout_sec=60.0)
        assert _wait_until(
            lambda: robot.is_capability_running('navigation'), timeout_sec=0.5)

        robot.stop_capability('navigation')

        assert _wait_until(
            lambda: not robot.is_capability_running('navigation'), timeout_sec=3.0)
        assert robot.capability_state('navigation') == CapabilityState.CANCELED
    finally:
        fake_robot.shutdown()


def test_run_capability_timeout_stops_it_automatically(session):
    robot_name = _unique_name('robot')
    fake_robot = FakeNavCapabilityProcess(robot_name, mock_duration_sec=30.0)
    try:
        robot = RobotHandle(robot_name)
        session.add_robot(robot)
        session.discover_capabilities()
        assert robot.has_capability('navigation') is True

        # A short client-side timeout, well under the goal's own (much
        # longer) mock duration: only spin_some()'s periodic
        # check_timeout() should be what stops it.
        robot.run_capability(
            'navigation', Navigation, make_navigation_goal('kitchen'), timeout_sec=1.0)

        deadline = time.monotonic() + 3.0
        while robot.is_capability_running('navigation') and time.monotonic() < deadline:
            session.spin_some()

        assert robot.capability_state('navigation') == CapabilityState.CANCELED
    finally:
        fake_robot.shutdown()


def test_rejected_goal_reports_rejected_specifically(session):
    # CapabilityState reports the real, specific outcome -- a goal the
    # server itself rejects must read as REJECTED, not CANCELED (which
    # is reserved for a mission script/timeout deliberately stopping an
    # in-flight goal).
    robot_name = _unique_name('robot')
    fake_robot = FakeNavCapabilityProcess(robot_name)
    try:
        robot = RobotHandle(robot_name)
        session.add_robot(robot)
        session.discover_capabilities()
        assert robot.has_capability('navigation') is True

        robot.run_capability(
            'navigation', Navigation, make_navigation_goal(REJECT_WAYPOINT_ID), timeout_sec=5.0)

        assert _wait_until(
            lambda: not robot.is_capability_running('navigation'), timeout_sec=3.0)
        assert robot.capability_state('navigation') == CapabilityState.REJECTED
    finally:
        fake_robot.shutdown()


def test_preempting_goal_wins_over_a_late_arriving_stale_outcome(session):
    # Regression test: a second run_capability() call while the first
    # goal is still in flight preempts it server-side. The *first*
    # goal's own client-side outcome callback can still arrive after
    # the second goal has already settled -- without the generation
    # guard in RunningCapability, that late callback would stomp
    # capability_state() with the superseded goal's outcome.
    robot_name = _unique_name('robot')
    fake_robot = FakeNavCapabilityProcess(robot_name, mock_duration_sec=1.0)
    try:
        robot = RobotHandle(robot_name)
        session.add_robot(robot)
        session.discover_capabilities()
        assert robot.has_capability('navigation') is True

        robot.run_capability(
            'navigation', Navigation, make_navigation_goal('kitchen'), timeout_sec=30.0)
        assert _wait_until(
            lambda: robot.is_capability_running('navigation'), timeout_sec=0.5)

        # Preempt partway through the first goal's ~1s mock duration.
        robot.run_capability(
            'navigation', Navigation, make_navigation_goal('charging_station'), timeout_sec=30.0)

        assert _wait_until(
            lambda: not robot.is_capability_running('navigation'), timeout_sec=4.0)
        assert robot.capability_state('navigation') == CapabilityState.SUCCEEDED

        # The first goal's own late-arriving outcome, if not filtered
        # out, would land some time after the preempting goal already
        # succeeded -- give it a window to (not) do so.
        time.sleep(0.5)
        assert robot.capability_state('navigation') == CapabilityState.SUCCEEDED
    finally:
        fake_robot.shutdown()


def test_is_alive_true_after_discovery_of_an_active_capability(session):
    robot_name = _unique_name('robot')
    fake_robot = FakeNavCapabilityProcess(robot_name)
    try:
        robot = RobotHandle(robot_name)
        session.add_robot(robot)
        session.discover_capabilities()

        assert _wait_until(lambda: robot.is_alive('navigation'), timeout_sec=2.0)
    finally:
        fake_robot.shutdown()


def test_simple_controller_forwards_to_a_working_fleet_session():
    robot_name = _unique_name('robot')
    fake_robot = FakeNavCapabilityProcess(robot_name, mock_duration_sec=0.05)
    controller = SimpleController()
    try:
        robot = RobotHandle(robot_name)
        controller.add_robot(robot)

        assert len(controller.robots) == 1
        assert controller.find_robot(robot_name) is robot

        controller.discover_capabilities()
        assert robot.has_capability('navigation') is True

        robot.run_capability(
            'navigation', Navigation, make_navigation_goal('kitchen'), timeout_sec=5.0)
        controller.spin_for(0.5)

        assert _wait_until(
            lambda: not robot.is_capability_running('navigation'), timeout_sec=3.0)
        assert robot.capability_state('navigation') == CapabilityState.SUCCEEDED
        assert controller.node is not None
    finally:
        fake_robot.shutdown()
        # SimpleController wraps a FleetSession the test fixture doesn't
        # own -- tear it down the same test-only way.
        controller._session._teardown()
