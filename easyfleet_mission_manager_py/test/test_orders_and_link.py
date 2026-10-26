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

"""Unit tests for OrderHistory and LinkWatchdog (no ROS graph involved)."""

from action_msgs.msg import GoalStatus, GoalStatusArray
from builtin_interfaces.msg import Time
from easyfleet_interfaces.msg import CapabilityStatus, OrderStatus
from easyfleet_mission_manager_py.link_watchdog import LinkWatchdog
from easyfleet_mission_manager_py.order_history import is_terminal, OrderHistory, to_hex


def _uuid(seed: int) -> bytes:
    return bytes((seed + i) % 256 for i in range(16))


def _at(sec: int) -> Time:
    return Time(sec=sec)


def _statuses(*pairs) -> GoalStatusArray:
    msg = GoalStatusArray()
    for goal_id, state in pairs:
        status = GoalStatus(status=state)
        status.goal_info.goal_id.uuid = list(goal_id)
        msg.status_list.append(status)
    return msg


def test_hex_is_32_lowercase_digits():
    goal_id = bytearray(16)
    goal_id[0] = 0xab
    goal_id[15] = 0x0f
    assert to_hex(goal_id) == 'ab00000000000000000000000000000f'


def test_orders_use_their_order_id_or_the_goal_uuid():
    history = OrderHistory()
    assert history.latest() is None
    assert history.accepted(_uuid(1), 'pick-7', '/r/arm', _at(3)) == 'pick-7'
    hex_id = to_hex(_uuid(2))
    assert history.accepted(_uuid(2), '', '/r/arm', _at(4)) == hex_id
    pick = history.find('pick-7')
    assert pick.goal_id == to_hex(_uuid(1))
    assert pick.action_name == '/r/arm'
    assert pick.state == OrderStatus.STATE_ACCEPTED
    assert pick.accepted_stamp.sec == 3
    assert pick.finished_stamp.sec == 0
    assert history.find(hex_id) is not None
    assert history.latest().order_id == hex_id
    assert history.find('nope') is None
    assert history.find('') is None


def test_follows_the_states_and_stamps_the_finish():
    history = OrderHistory()
    history.accepted(_uuid(1), 'a', '/r/nav', _at(1))
    history.update(_statuses((_uuid(1), OrderStatus.STATE_EXECUTING)), _at(2))
    assert history.find('a').state == OrderStatus.STATE_EXECUTING
    history.update(_statuses((_uuid(1), OrderStatus.STATE_CANCELING)), _at(3))
    history.update(_statuses((_uuid(1), OrderStatus.STATE_CANCELED)), _at(4))
    order = history.find('a')
    assert order.state == OrderStatus.STATE_CANCELED
    assert order.finished_stamp.sec == 4


def test_a_finished_order_never_changes_again():
    history = OrderHistory()
    history.accepted(_uuid(1), 'a', '/r/nav', _at(1))
    history.update(_statuses((_uuid(1), OrderStatus.STATE_SUCCEEDED)), _at(5))
    history.update(_statuses((_uuid(1), OrderStatus.STATE_ABORTED)), _at(9))
    assert history.find('a').state == OrderStatus.STATE_SUCCEEDED
    assert history.find('a').finished_stamp.sec == 5


def test_unknown_goals_and_states_are_ignored():
    history = OrderHistory()
    history.accepted(_uuid(1), 'a', '/r/nav', _at(1))
    history.update(
        _statuses((_uuid(9), OrderStatus.STATE_SUCCEEDED), (_uuid(1), OrderStatus.STATE_UNKNOWN)),
        _at(2))
    assert len(history) == 1
    assert history.find('a').state == OrderStatus.STATE_ACCEPTED


def test_a_repeated_order_id_finds_the_newest_order():
    history = OrderHistory()
    history.accepted(_uuid(1), 'retry', '/r/nav', _at(1))
    history.update(_statuses((_uuid(1), OrderStatus.STATE_ABORTED)), _at(2))
    history.accepted(_uuid(2), 'retry', '/r/nav', _at(3))
    assert history.find('retry').goal_id == to_hex(_uuid(2))
    assert history.find('retry').state == OrderStatus.STATE_ACCEPTED


def test_finished_orders_are_dropped_first():
    history = OrderHistory(3)
    for i in (1, 2, 3):
        history.accepted(_uuid(i), str(i), '/r/nav', _at(i))
    history.update(
        _statuses((_uuid(2), OrderStatus.STATE_SUCCEEDED), (_uuid(3), OrderStatus.STATE_ABORTED)),
        _at(4))
    history.accepted(_uuid(4), '4', '/r/nav', _at(5))
    assert len(history) == 3
    assert history.find('1') is not None
    assert history.find('2') is None
    assert history.find('3') is not None


def test_without_finished_orders_the_oldest_goes_and_capacity_is_at_least_one():
    history = OrderHistory(2)
    for i in (1, 2, 3):
        history.accepted(_uuid(i), str(i), '/r/nav', _at(i))
    assert history.find('1') is None
    tiny = OrderHistory(0)
    tiny.accepted(_uuid(1), '1', '/r/nav', _at(1))
    tiny.accepted(_uuid(2), '2', '/r/nav', _at(2))
    assert len(tiny) == 1
    assert tiny.latest().order_id == '2'


def test_returned_orders_are_copies():
    history = OrderHistory()
    history.accepted(_uuid(1), 'a', '/r/nav', _at(1))
    history.find('a').state = OrderStatus.STATE_ABORTED
    assert history.find('a').state == OrderStatus.STATE_ACCEPTED


def test_terminal_states_match_goal_status():
    assert not is_terminal(OrderStatus.STATE_EXECUTING)
    assert not is_terminal(OrderStatus.STATE_CANCELING)
    assert is_terminal(OrderStatus.STATE_SUCCEEDED)
    assert is_terminal(OrderStatus.STATE_CANCELED)
    assert is_terminal(OrderStatus.STATE_ABORTED)
    assert OrderStatus.STATE_SUCCEEDED == GoalStatus.STATUS_SUCCEEDED
    assert OrderStatus.STATE_ABORTED == GoalStatus.STATUS_ABORTED


def test_watchdog_zero_or_negative_timeout_is_not_monitored():
    for timeout in (0.0, -1.0):
        watchdog = LinkWatchdog(timeout)
        assert not watchdog.monitored
        watchdog.heard(0.0)
        assert watchdog.update(3600.0) == CapabilityStatus.LINK_NOT_MONITORED


def test_watchdog_waits_ok_lost_and_recovers():
    watchdog = LinkWatchdog(2.0)
    assert watchdog.state == CapabilityStatus.LINK_WAITING
    assert watchdog.update(1000.0) == CapabilityStatus.LINK_WAITING
    watchdog.heard(10.0)
    assert watchdog.update(12.0) == CapabilityStatus.LINK_OK, 'exactly the timeout'
    assert watchdog.update(12.001) == CapabilityStatus.LINK_LOST
    watchdog.heard(13.0)
    assert watchdog.state == CapabilityStatus.LINK_OK


def test_watchdog_older_heartbeat_and_reset():
    watchdog = LinkWatchdog(1.0)
    watchdog.heard(5.0)
    watchdog.heard(0.0)
    assert watchdog.update(5.5) == CapabilityStatus.LINK_OK
    watchdog.reset(3.0)
    assert watchdog.state == CapabilityStatus.LINK_WAITING
    watchdog.reset(0.0)
    assert watchdog.state == CapabilityStatus.LINK_NOT_MONITORED
