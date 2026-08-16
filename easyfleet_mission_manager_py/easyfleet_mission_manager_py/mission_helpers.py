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
Shared by every mission script, mirroring easyfleet_mission_manager's mission_helpers.hpp.

The easyfleet_interfaces action types the mock capabilities speak, how
to build a demo goal and a feedback printer for each of them, and small
terminal/discovery helpers.
"""

import json
import threading
import time

from easyfleet_interfaces.action import Manipulation, Navigation, Perception

from . import ansi
from .output import safe_print
from .throttle import Throttle

__all__ = [
    'Manipulation', 'Navigation', 'Perception',
    'RUN_TIMEOUT_SEC',
    'find_robot_capability',
    'make_manipulation_feedback_printer', 'make_manipulation_goal',
    'make_navigation_feedback_printer', 'make_navigation_goal',
    'make_perception_feedback_printer', 'make_perception_goal',
    'print_section', 'print_step', 'spin_in_background',
]

# How long a mission script lets a capability run before stopping it, if
# it hasn't finished on its own by then.
RUN_TIMEOUT_SEC = 10.0


def spin_in_background(executor) -> threading.Thread:
    """Start `executor.spin()` on a background thread; block until it has actually begun."""
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    while not executor.is_spinning:
        time.sleep(0)
    return thread


def print_section(title: str) -> None:
    """Print a bold, boxed header marking a new phase of a mission script."""
    safe_print(f'\n{ansi.BOLD}{ansi.BLUE}== {title} =={ansi.RESET}')


def print_step(text: str) -> None:
    """Print a dim, indented line explaining what a phase is about to do (or just did)."""
    safe_print(f'  {ansi.DIM}{text}{ansi.RESET}')


def find_robot_capability(capabilities, robot: str, capability: str):
    """Find `robot`'s active capability of the given short type (e.g. "navigation")."""
    for info in capabilities:
        if info.robot == robot and info.capability == capability and info.active:
            return info
    return None


def make_navigation_goal(waypoint_id: str | None = None):
    """
    Build a Navigation goal.

    With no `waypoint_id`, a fixed demo pose; with one, targets a named
    waypoint via the `parameters_json: {"goal_id": ...}` convention the
    real EasyNav-backed navigation capability reads -- ignored by (and
    therefore also safe against) the mock navigation capability.
    """
    goal = Navigation.Goal()
    if waypoint_id is None:
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.pose.position.x = 2.0
        goal.target_pose.pose.position.y = 1.0
        goal.target_pose.pose.orientation.w = 1.0
    else:
        goal.parameters_json = json.dumps({'goal_id': waypoint_id})
    return goal


def make_navigation_feedback_printer(label: str):
    """Build a throttled feedback printer for navigation goals."""
    throttle = Throttle(0.5)

    def printer(feedback) -> None:
        if not throttle.ready():
            return
        safe_print(
            f'  {ansi.DIM}[{label}] {ansi.RESET}'
            f'distance_remaining={feedback.distance_remaining}m, '
            f'elapsed={feedback.navigation_time.sec}s, '
            f'recoveries={feedback.number_of_recoveries}')
    return printer


def make_manipulation_goal(pose=None):
    """Build a Manipulation goal: a fixed demo joint target, or an end-effector `pose`."""
    goal = Manipulation.Goal()
    if pose is None:
        goal.mode = Manipulation.Goal.MODE_JOINT_TARGET
        goal.joint_target.name = ['joint1']
        goal.joint_target.position = [1.0]
    else:
        goal.mode = Manipulation.Goal.MODE_POSE_TARGET
        goal.pose_target = pose
    return goal


def make_manipulation_feedback_printer(label: str):
    throttle = Throttle(0.5)

    def printer(feedback) -> None:
        if not throttle.ready():
            return
        safe_print(f'  {ansi.DIM}[{label}] {ansi.RESET}state={feedback.state}')
    return printer


def make_perception_goal(object_classes: list[str] | None = None):
    """Build a Perception goal detecting the given object classes (default: a demo "gato")."""
    goal = Perception.Goal()
    goal.object_classes = object_classes if object_classes is not None else ['gato']
    return goal


def make_perception_feedback_printer(label: str):
    throttle = Throttle(0.5)

    def printer(feedback) -> None:
        if not throttle.ready():
            return
        count = len(feedback.detections_3d.detections)
        safe_print(f'  {ansi.DIM}[{label}] {ansi.RESET}{count} cat(s) detected (3D + 2D)')
    return printer
