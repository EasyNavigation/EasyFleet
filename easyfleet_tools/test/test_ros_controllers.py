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
Unit tests for easyfleet_tools' shared ROS-facing model.

No ROS node is required: message objects are plain Python objects, and
none of these functions/classes touch a live subscription.
"""

import time

from action_msgs.msg import GoalStatus, GoalStatusArray
from easyfleet_interfaces.msg import CapabilityStatus
from easyfleet_tools.cli.verb_utils import bbcode_to_ansi, resolve_capability
from easyfleet_tools.controller.ros_controllers import (
    _robot_from_base_link_frame,
    ALIVE_TIMEOUT,
    CapabilityInfo,
    generic_msg2text,
    log_msg2text,
    RobotInfo,
    status_msg2text,
)
from rcl_interfaces.msg import Log


# ─────────────────────────────────────────────────────────────────────
# CapabilityInfo.status
# ─────────────────────────────────────────────────────────────────────

def test_capability_status_inactive_without_any_heartbeat():
    cap = CapabilityInfo('robot_1', 'navigation', '/robot_1/navigation')
    assert cap.status == 'INACTIVE'


def test_capability_status_idle_with_fresh_heartbeat():
    cap = CapabilityInfo('robot_1', 'navigation', '/robot_1/navigation')
    cap.last_heartbeat = time.monotonic()
    cap.busy = False
    assert cap.status == 'IDLE'


def test_capability_status_busy_with_fresh_heartbeat():
    cap = CapabilityInfo('robot_1', 'navigation', '/robot_1/navigation')
    cap.last_heartbeat = time.monotonic()
    cap.busy = True
    assert cap.status == 'BUSY'


def test_capability_status_inactive_after_stale_heartbeat():
    cap = CapabilityInfo('robot_1', 'navigation', '/robot_1/navigation')
    cap.last_heartbeat = time.monotonic() - (ALIVE_TIMEOUT + 1.0)
    cap.busy = False
    assert cap.status == 'INACTIVE'


# ─────────────────────────────────────────────────────────────────────
# CapabilityInfo.display_name / set_description
# ─────────────────────────────────────────────────────────────────────

def test_display_name_falls_back_to_capability_type_without_description():
    cap = CapabilityInfo('robot_1', 'navigation', '/robot_1/navigation')
    assert cap.display_name == 'navigation'


def test_display_name_uses_description_json_when_present():
    cap = CapabilityInfo('robot_1', 'navigation', '/robot_1/navigation')
    cap.set_description('{"display_name": "Go somewhere"}')
    assert cap.display_name == 'Go somewhere'


def test_set_description_with_invalid_json_leaves_description_json_none():
    cap = CapabilityInfo('robot_1', 'navigation', '/robot_1/navigation')
    cap.set_description('not json')
    assert cap.description_json is None
    assert cap.display_name == 'navigation'


def test_set_description_with_non_object_json_leaves_description_json_none():
    cap = CapabilityInfo('robot_1', 'navigation', '/robot_1/navigation')
    cap.set_description('[1, 2, 3]')
    assert cap.description_json is None


# ─────────────────────────────────────────────────────────────────────
# RobotInfo.worst_status
# ─────────────────────────────────────────────────────────────────────

def test_worst_status_none_without_any_capability():
    robot = RobotInfo('robot_1')
    assert robot.worst_status is None


def test_worst_status_idle_when_all_capabilities_idle():
    robot = RobotInfo('robot_1')
    cap = CapabilityInfo('robot_1', 'navigation', '/robot_1/navigation')
    cap.last_heartbeat = time.monotonic()
    robot.capabilities['navigation'] = cap
    assert robot.worst_status == 'IDLE'


def test_worst_status_busy_wins_over_idle():
    robot = RobotInfo('robot_1')
    idle = CapabilityInfo('robot_1', 'navigation', '/robot_1/navigation')
    idle.last_heartbeat = time.monotonic()
    busy = CapabilityInfo('robot_1', 'manipulation', '/robot_1/manipulation')
    busy.last_heartbeat = time.monotonic()
    busy.busy = True
    robot.capabilities['navigation'] = idle
    robot.capabilities['manipulation'] = busy
    assert robot.worst_status == 'BUSY'


def test_worst_status_inactive_when_busy_absent_but_something_is_dead():
    robot = RobotInfo('robot_1')
    idle = CapabilityInfo('robot_1', 'navigation', '/robot_1/navigation')
    idle.last_heartbeat = time.monotonic()
    dead = CapabilityInfo('robot_1', 'manipulation', '/robot_1/manipulation')
    robot.capabilities['navigation'] = idle
    robot.capabilities['manipulation'] = dead
    assert robot.worst_status == 'INACTIVE'


# ─────────────────────────────────────────────────────────────────────
# _robot_from_base_link_frame
# ─────────────────────────────────────────────────────────────────────

def test_robot_from_base_link_frame_strips_suffix():
    assert _robot_from_base_link_frame('robot_1/base_link') == 'robot_1'


def test_robot_from_base_link_frame_unchanged_without_suffix():
    assert _robot_from_base_link_frame('map') == 'map'


# ─────────────────────────────────────────────────────────────────────
# status_msg2text
# ─────────────────────────────────────────────────────────────────────

def test_status_msg2text_no_goals_seen_yet():
    msg = GoalStatusArray()
    assert 'No goals seen yet' in status_msg2text(msg)


def test_status_msg2text_uses_the_most_recent_entry():
    msg = GoalStatusArray()
    first = GoalStatus()
    first.status = GoalStatus.STATUS_ABORTED
    second = GoalStatus()
    second.status = GoalStatus.STATUS_SUCCEEDED
    msg.status_list = [first, second]
    text = status_msg2text(msg)
    assert 'SUCCEEDED' in text
    assert 'ABORTED' not in text


def test_status_msg2text_executing():
    msg = GoalStatusArray()
    entry = GoalStatus()
    entry.status = GoalStatus.STATUS_EXECUTING
    msg.status_list = [entry]
    assert 'EXECUTING' in status_msg2text(msg)


# ─────────────────────────────────────────────────────────────────────
# generic_msg2text
# ─────────────────────────────────────────────────────────────────────

def test_generic_msg2text_contains_field_values():
    msg = CapabilityStatus()
    msg.robot = 'robot_1'
    msg.capability = 'navigation'
    msg.busy = True
    text = generic_msg2text(msg)
    assert 'robot_1' in text
    assert 'navigation' in text


# ─────────────────────────────────────────────────────────────────────
# log_msg2text
# ─────────────────────────────────────────────────────────────────────

def test_log_msg2text_debug_is_green():
    msg = Log()
    msg.level = Log.DEBUG
    msg.name = 'robot_1.navigation'
    msg.msg = 'starting up'
    text = log_msg2text(msg)
    assert '[green]' in text
    assert 'starting up' in text


def test_log_msg2text_warn_is_orange():
    msg = Log()
    msg.level = Log.WARN
    msg.name = 'robot_1.navigation'
    msg.msg = 'retrying'
    assert '[orange3]' in log_msg2text(msg)


def test_log_msg2text_error_is_red():
    msg = Log()
    msg.level = Log.ERROR
    msg.name = 'robot_1.navigation'
    msg.msg = 'failed'
    assert '[red]' in log_msg2text(msg)


def test_log_msg2text_unknown_level_falls_back_to_the_raw_number():
    msg = Log()
    msg.level = 99
    msg.name = 'robot_1.navigation'
    msg.msg = 'huh'
    assert '99' in log_msg2text(msg)


# ─────────────────────────────────────────────────────────────────────
# bbcode_to_ansi
# ─────────────────────────────────────────────────────────────────────

def test_bbcode_color_disabled_strips_tags():
    assert bbcode_to_ansi('[red]hello[/red]', enable_color=False) == 'hello'


def test_bbcode_color_enabled_red():
    result = bbcode_to_ansi('[red]hello[/red]', enable_color=True)
    assert '\033[31m' in result
    assert 'hello' in result


def test_bbcode_bold_red_compound_tag():
    result = bbcode_to_ansi('[bold red]fatal[/bold red]', enable_color=True)
    assert '\033[1m' in result
    assert '\033[31m' in result
    assert 'fatal' in result


def test_bbcode_bold_red_compound_tag_disabled():
    assert bbcode_to_ansi('[bold red]fatal[/bold red]', enable_color=False) == 'fatal'


# ─────────────────────────────────────────────────────────────────────
# resolve_capability
# ─────────────────────────────────────────────────────────────────────

class _FakeDiscovery:

    def __init__(self):
        self.robots = {}


def _make_discovery_with_one_capability():
    discovery = _FakeDiscovery()
    robot = RobotInfo('robot_1')
    cap = CapabilityInfo('robot_1', 'navigation', '/robot_1/navigation')
    robot.capabilities['navigation'] = cap
    discovery.robots['robot_1'] = robot
    return discovery, cap


def test_resolve_capability_by_robot_slash_capability():
    discovery, cap = _make_discovery_with_one_capability()
    assert resolve_capability(discovery, 'robot_1/navigation') is cap


def test_resolve_capability_falls_back_to_action_name_match():
    # capability_type ("navigation") deliberately differs from the tail
    # of action_name ("nav") -- only the action_name fallback path can
    # resolve this, not the "<robot>/<capability_type>" split.
    discovery = _FakeDiscovery()
    robot = RobotInfo('robot_1')
    cap = CapabilityInfo('robot_1', 'navigation', 'robot_1/nav')
    robot.capabilities['navigation'] = cap
    discovery.robots['robot_1'] = robot
    assert resolve_capability(discovery, 'robot_1/nav') is cap


def test_resolve_capability_returns_none_when_not_found():
    discovery, _ = _make_discovery_with_one_capability()
    assert resolve_capability(discovery, 'robot_2/navigation') is None
