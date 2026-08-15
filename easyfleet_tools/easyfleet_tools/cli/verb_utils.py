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

import re


def resolve_capability(discovery, identifier: str):
    """
    Resolve a capability identifier typed on the command line.

    Looked up against an already-populated FleetDiscovery. Accepts
    either "<robot>/<capability>" (e.g. "robot_1/navigation") or a bare
    action_name (e.g. "/robot_1/navigation"), matching whichever form
    the user finds more natural to type. Returns None if nothing
    matches.
    """
    if '/' in identifier:
        robot_name, _, capability_type = identifier.rpartition('/')
        robot = discovery.robots.get(robot_name)
        if robot is not None:
            capability = robot.capabilities.get(capability_type)
            if capability is not None:
                return capability

    for robot in discovery.robots.values():
        for capability in robot.capabilities.values():
            if capability.action_name in (identifier, '/' + identifier):
                return capability
    return None


_TAGS = {
    'black': '30', 'red': '31', 'green': '32', 'yellow': '33',
    'blue': '34', 'magenta': '35', 'cyan': '36', 'white': '37',
    'orange3': '33', 'bold': '1', 'dim': '2', 'underline': '4'
}

_OPEN_TAG_RE = re.compile(r'\[([a-zA-Z0-9 ]+)\]')
_CLOSE_TAG_RE = re.compile(r'\[/([a-zA-Z0-9 ]+)\]')


def bbcode_to_ansi(s: str, enable_color: bool) -> str:
    if not enable_color:
        s = _OPEN_TAG_RE.sub('', s)
        s = _CLOSE_TAG_RE.sub('', s)
        return s

    # "bold red" (two space-separated tags in one pair, e.g. FATAL log
    # lines) -> two stacked ANSI codes.
    for tag, code in _TAGS.items():
        s = re.sub(
            rf'\[{tag}\](.*?)\[/\s*{tag}\]',
            lambda m: f'\033[{code}m{m.group(1)}\033[0m',
            s,
            flags=re.DOTALL | re.IGNORECASE
        )
    s = re.sub(
        r'\[bold red\](.*?)\[/\s*bold red\]',
        lambda m: f'\033[1m\033[31m{m.group(1)}\033[0m',
        s,
        flags=re.DOTALL | re.IGNORECASE
    )
    s = _OPEN_TAG_RE.sub('', s)
    s = _CLOSE_TAG_RE.sub('', s)
    return s
