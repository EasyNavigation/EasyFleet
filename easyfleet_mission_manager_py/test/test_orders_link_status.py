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

"""Orders, get_order, link watchdog and robot status, end to end over ROS."""

import itertools
import threading
import time

from action_msgs.msg import GoalStatus
from easyfleet_interfaces.action import Navigation
from easyfleet_interfaces.msg import (
    CapabilityStatus, FleetHeartbeat, OrderStatus, RobotStatus)
from easyfleet_interfaces.srv import GetOrder
from easyfleet_mission_manager_py.action_server_base import ActionServerBase
from easyfleet_mission_manager_py.capability import Capability
from easyfleet_mission_manager_py.fleet_session import FleetSession
from easyfleet_mission_manager_py.order_history import to_hex
from easyfleet_mission_manager_py.robot_handle import RobotHandle
from easyfleet_mission_manager_py.robot_status_publisher import RobotStatusPublisher
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy

_names = itertools.count()


def _unique(base: str) -> str:
    return f'{base}_{next(_names)}'


def _wait_until(condition, timeout_sec: float) -> bool:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if condition():
            return True
        time.sleep(0.02)
    return condition()


def _wait_future(future, timeout_sec: float = 5.0):
    assert _wait_until(future.done, timeout_sec), 'no answer in time'
    return future.result()


class _TimedServer(ActionServerBase):
    """Holds each goal for `delay` seconds, stopping early when asked."""

    delay = 0.0

    def __init__(self, node, action_name):
        super().__init__(node, Navigation, action_name)

    def on_goal_received(self, goal) -> bool:
        return goal.parameters_json != 'reject'

    def on_execute(self, goal_handle):
        deadline = time.monotonic() + self.delay
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return Navigation.Result()
            if self.is_shutdown_requested() or self.is_preempt_requested():
                goal_handle.abort()
                return Navigation.Result()
            time.sleep(0.01)
        goal_handle.succeed()
        return Navigation.Result()


class _Robot:
    """A Python navigation Capability under /<robot>, plus a client node, each spinning."""

    def __init__(self, tmp_path, delay=0.0, params=None, activate=True):
        self.robot = _unique('pyrobot')
        overrides = [Parameter(f'navigation.{k}', value=v) for k, v in (params or {}).items()]
        caps_file = tmp_path / f'{self.robot}.json'
        caps_file.write_text('{"name": "navigation"}')
        overrides.append(Parameter('capabilities_file', value=str(caps_file)))

        def factory(node, name):
            server = _TimedServer(node, name)
            server.delay = delay
            return server
        self.cap = Capability(
            'navigation', factory, namespace=f'/{self.robot}', parameter_overrides=overrides)
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.cap)
        self.client_node = rclpy.create_node(_unique('client'), start_parameter_services=False)
        self.client_executor = SingleThreadedExecutor()
        self.client_executor.add_node(self.client_node)
        self.threads = [
            threading.Thread(target=self.executor.spin, daemon=True),
            threading.Thread(target=self.client_executor.spin, daemon=True)]
        for thread in self.threads:
            thread.start()
        self.action = ActionClient(self.client_node, Navigation, f'/{self.robot}/navigation')
        self.get_order_client = self.client_node.create_client(
            GetOrder, f'/{self.robot}/navigation/get_order')
        self.heartbeat_pub = self.client_node.create_publisher(
            FleetHeartbeat, '/fleet_heartbeat', QoSProfile(depth=1))
        self.heartbeat_timer = None
        if activate:
            assert self.cap.trigger_configure() == TransitionCallbackReturn.SUCCESS
            assert self.cap.trigger_activate() == TransitionCallbackReturn.SUCCESS
            assert self.action.wait_for_server(timeout_sec=5.0)
            assert self.get_order_client.wait_for_service(timeout_sec=5.0)

    def send(self, order_id='', params=''):
        goal = Navigation.Goal(order_id=order_id, parameters_json=params)
        handle = _wait_future(self.action.send_goal_async(goal))
        return handle if handle.accepted else None

    def result_status(self, handle, timeout_sec=10.0):
        return _wait_future(handle.get_result_async(), timeout_sec).status

    def query(self, order_id):
        return _wait_future(self.get_order_client.call_async(GetOrder.Request(order_id=order_id)))

    def wait_state(self, order_id, state, timeout_sec=5.0):
        return _wait_until(
            lambda: (lambda r: r.found and r.order.state == state)(self.query(order_id)),
            timeout_sec)

    def start_heartbeat(self):
        self.heartbeat_timer = self.client_node.create_timer(
            0.05, lambda: self.heartbeat_pub.publish(FleetHeartbeat(source='test')))

    def stop_heartbeat(self):
        self.client_node.destroy_timer(self.heartbeat_timer)

    def close(self):
        self.executor.shutdown()
        self.client_executor.shutdown()
        self.cap.destroy_node()
        self.client_node.destroy_node()


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def robots():
    made = []
    yield made
    for robot in made:
        robot.close()


def test_an_order_is_queryable_by_its_order_id(tmp_path, robots):
    robot = _Robot(tmp_path)
    robots.append(robot)
    handle = robot.send('deliver-1')
    assert robot.result_status(handle) == GoalStatus.STATUS_SUCCEEDED
    assert robot.wait_state('deliver-1', OrderStatus.STATE_SUCCEEDED)
    order = robot.query('deliver-1').order
    assert order.goal_id == to_hex(handle.goal_id.uuid)
    assert order.action_name == f'/{robot.robot}/navigation'
    assert (order.finished_stamp.sec, order.finished_stamp.nanosec) >= (
        order.accepted_stamp.sec, order.accepted_stamp.nanosec)
    status = robot.cap.get_status()
    assert status.order_id == 'deliver-1'
    assert status.order_state == OrderStatus.STATE_SUCCEEDED
    assert status.link_state == CapabilityStatus.LINK_NOT_MONITORED


def test_without_order_id_the_goal_uuid_is_used(tmp_path, robots):
    robot = _Robot(tmp_path)
    robots.append(robot)
    handle = robot.send('')
    assert robot.result_status(handle) == GoalStatus.STATUS_SUCCEEDED
    assert robot.wait_state(to_hex(handle.goal_id.uuid), OrderStatus.STATE_SUCCEEDED)
    assert not robot.query('never-sent').found
    assert robot.send('refused', 'reject') is None
    assert not robot.query('refused').found


def test_running_preempted_and_canceled_orders(tmp_path, robots):
    robot = _Robot(tmp_path, delay=3.0)
    robots.append(robot)
    first = robot.send('first')
    assert robot.wait_state('first', OrderStatus.STATE_EXECUTING)
    assert robot.cap.get_status().busy
    second = robot.send('second')
    assert robot.result_status(first) == GoalStatus.STATUS_ABORTED
    assert robot.wait_state('first', OrderStatus.STATE_ABORTED)
    assert robot.wait_state('second', OrderStatus.STATE_EXECUTING)
    _wait_future(second.cancel_goal_async())
    assert robot.result_status(second) == GoalStatus.STATUS_CANCELED
    assert robot.wait_state('second', OrderStatus.STATE_CANCELED)


def test_order_history_keeps_only_the_configured_number(tmp_path, robots):
    robot = _Robot(tmp_path, params={'order_history': 2})
    robots.append(robot)
    for order_id in ('o1', 'o2', 'o3'):
        assert robot.result_status(robot.send(order_id)) == GoalStatus.STATUS_SUCCEEDED
    assert robot.wait_state('o3', OrderStatus.STATE_SUCCEEDED)
    assert not robot.query('o1').found
    assert robot.query('o2').found


def test_halt_active_goal_stops_it_and_goals_keep_working(tmp_path, robots):
    robot = _Robot(tmp_path, delay=5.0)
    robots.append(robot)
    server = robot.cap.get_action_server()
    server.halt_active_goal()  # idle: nothing to do
    handle = robot.send('halted')
    assert _wait_until(server.is_active, 3.0)
    server.halt_active_goal()
    assert robot.result_status(handle) == GoalStatus.STATUS_ABORTED
    assert server.goals_enabled()
    assert _wait_until(lambda: not server.is_shutdown_requested(), 3.0)
    server.delay = 0.0
    assert robot.result_status(robot.send('after')) == GoalStatus.STATUS_SUCCEEDED


def test_goal_accepted_callback_gets_the_order_id_of_accepted_goals_only():
    node = rclpy.create_node(_unique('bare_server'), start_parameter_services=False)
    client_node = rclpy.create_node(_unique('bare_client'), start_parameter_services=False)
    server = _TimedServer(node, _unique('bare_nav'))
    accepted = []
    server.set_goal_accepted_callback(lambda uuid, order: accepted.append((bytes(uuid), order)))
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.add_node(client_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        client = ActionClient(client_node, Navigation, server.get_action_name())
        assert client.wait_for_server(timeout_sec=5.0)
        handle = _wait_future(client.send_goal_async(Navigation.Goal(order_id='a')))
        _wait_future(handle.get_result_async())
        assert not _wait_future(
            client.send_goal_async(Navigation.Goal(parameters_json='reject'))).accepted
        server.set_goal_accepted_callback(None)
        later = _wait_future(client.send_goal_async(Navigation.Goal(order_id='b')))
        _wait_future(later.get_result_async())
        assert accepted == [(bytes(handle.goal_id.uuid), 'a')]
    finally:
        executor.shutdown()
        server.destroy()
        node.destroy_node()
        client_node.destroy_node()


def test_a_bad_on_link_lost_fails_to_configure(tmp_path, robots):
    robot = _Robot(tmp_path, params={'on_link_lost': 'explode'}, activate=False)
    robots.append(robot)
    assert robot.cap.trigger_configure() != TransitionCallbackReturn.SUCCESS


def test_link_lost_with_cancel_stops_the_goal(tmp_path, robots):
    robot = _Robot(tmp_path, delay=10.0, params={'link_timeout': 0.4, 'on_link_lost': 'cancel'})
    robots.append(robot)
    assert robot.cap.get_status().link_state == CapabilityStatus.LINK_WAITING
    handle = robot.send('while-waiting')
    time.sleep(1.2)
    assert robot.cap.get_action_server().is_active(), 'never heard: not stopped'
    robot.start_heartbeat()
    assert _wait_until(
        lambda: robot.cap.get_status().link_state == CapabilityStatus.LINK_OK, 3.0)
    robot.stop_heartbeat()
    assert robot.result_status(handle, 3.0) == GoalStatus.STATUS_ABORTED
    assert robot.cap.get_status().link_state == CapabilityStatus.LINK_LOST
    robot.start_heartbeat()
    assert _wait_until(
        lambda: robot.cap.get_status().link_state == CapabilityStatus.LINK_OK, 3.0)


def test_link_lost_with_continue_only_reports_it(tmp_path, robots):
    robot = _Robot(tmp_path, delay=2.0, params={'link_timeout': 0.3})
    robots.append(robot)
    robot.start_heartbeat()
    assert _wait_until(
        lambda: robot.cap.get_status().link_state == CapabilityStatus.LINK_OK, 3.0)
    handle = robot.send('carry-on')
    robot.stop_heartbeat()
    assert _wait_until(
        lambda: robot.cap.get_status().link_state == CapabilityStatus.LINK_LOST, 3.0)
    assert robot.result_status(handle) == GoalStatus.STATUS_SUCCEEDED


def test_the_session_publishes_the_fleet_heartbeat():
    session = FleetSession()
    try:
        node = rclpy.create_node(_unique('listener'), start_parameter_services=False)
        heard = []
        node.create_subscription(
            FleetHeartbeat, '/fleet_heartbeat', heard.append,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        deadline = time.monotonic() + 3.5
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        assert 2 <= len(heard) <= 5
        assert heard[0].source == '/mission_control'
        node.destroy_node()
    finally:
        session._teardown()


def test_the_session_gets_orders_and_keeps_the_capability_alive(tmp_path, robots):
    robot = _Robot(tmp_path, delay=0.05, params={'link_timeout': 2.5})
    robots.append(robot)
    session = FleetSession()
    try:
        handle = RobotHandle(robot.robot)
        session.add_robot(handle)
        session.discover_capabilities()
        assert handle.has_capability('navigation')
        handle.run_capability(
            'navigation', Navigation, Navigation.Goal(order_id='py-order'), 5.0)
        assert _wait_until(lambda: not handle.is_capability_running('navigation'), 5.0)
        assert _wait_until(
            lambda: (lambda o: o is not None and o.state == OrderStatus.STATE_SUCCEEDED)(
                handle.get_order('py-order')), 5.0)
        assert handle.get_order('never-sent') is None
        assert _wait_until(
            lambda: [c.link_state for c in handle.capabilities] == [CapabilityStatus.LINK_OK],
            4.0)
        assert _wait_until(
            lambda: [(c.order_id, c.order_state) for c in handle.capabilities] ==
            [('py-order', OrderStatus.STATE_SUCCEEDED)], 3.0)
    finally:
        session._teardown()
    assert _wait_until(
        lambda: robot.cap.get_status().link_state == CapabilityStatus.LINK_LOST, 5.0)


def test_unknown_robots_and_detached_handles():
    session = FleetSession()
    try:
        start = time.monotonic()
        assert session.get_order('nobody', 'x') is None
        assert time.monotonic() - start < 0.5
        assert session.robot_status('nobody') is None
    finally:
        session._teardown()
    detached = RobotHandle('detached')
    assert detached.get_order('x') is None
    assert detached.status() is None


def _status_of(robot, capabilities, sec):
    msg = RobotStatus(robot=robot)
    msg.stamp.sec = sec
    msg.capabilities = [
        CapabilityStatus(robot=robot, capability=c, action_name=f'/{robot}/{c}')
        for c in capabilities]
    return msg


def test_robot_status_merges_the_processes_of_a_robot():
    session = FleetSession()
    node = rclpy.create_node(_unique('status_source'), start_parameter_services=False)
    pub = node.create_publisher(
        RobotStatus, '/robot_status', QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
    try:
        robot = _unique('merged')
        handle = RobotHandle(robot)
        session.add_robot(handle)
        assert _wait_until(lambda: pub.get_subscription_count() > 0, 5.0)
        base = _status_of(robot, ['navigation'], 10)
        base.has_pose = True
        base.pose.pose.pose.position.x = 3.0
        arm = _status_of(robot, ['manipulation', 'perception'], 12)
        arm.has_battery = True
        arm.battery.percentage = 0.5
        pub.publish(base)
        pub.publish(arm)
        pub.publish(_status_of(_unique('other'), ['navigation'], 99))
        assert _wait_until(
            lambda: handle.status() is not None and len(handle.status().capabilities) == 3, 5.0)
        status = handle.status()
        assert status.stamp.sec == 12
        assert status.has_pose and status.pose.pose.pose.position.x == 3.0
        assert status.has_battery and abs(status.battery.percentage - 0.5) < 1e-6
        pub.publish(_status_of(robot, ['navigation'], 11))
        time.sleep(0.3)
        assert handle.status().has_pose
        assert handle.status().stamp.sec == 12
    finally:
        node.destroy_node()
        session._teardown()


def test_robot_status_publisher_reports_namespace_capabilities_and_topics():
    node = rclpy.create_node(
        'deployment_config', namespace=f'/{_unique("rsp")}',
        parameter_overrides=[
            Parameter('robot_status.rate_hz', value=0.0),
            Parameter('robot_status.pose_topic', value='amcl_pose')],
        start_parameter_services=False)
    try:
        publisher = RobotStatusPublisher(node, lambda: [CapabilityStatus(capability='nav')])
        status = publisher.make_status()
        assert status.robot == node.get_namespace()[1:]
        assert [c.capability for c in status.capabilities] == ['nav']
        assert not status.has_pose and not status.has_battery
        assert publisher._timer is None, 'rate 0 disables publishing'
    finally:
        node.destroy_node()
