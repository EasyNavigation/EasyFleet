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

"""CapabilityInfo + pretty-printers, mirroring capability_info.hpp/.cpp."""

from . import ansi
from .output import safe_print


class CapabilityInfo:
    """
    Everything known about one running capability instance.

    Gathered from its /capabilities (easyfleet_interfaces/CapabilityDescription)
    and /capabilities_status (easyfleet_interfaces/CapabilityStatus)
    messages. `action_name` is the unique identity: two robots can both
    offer a `capability` named "navigation", but each has its own
    `action_name`.
    """

    def __init__(
        self,
        robot: str = '',
        capability: str = '',
        action_name: str = '',
        description_json_raw: str = '',
        description_json: dict | None = None,
        description_json_valid: bool = False,
        active: bool = False,
        busy: bool = False,
    ):
        self.robot = robot
        self.capability = capability
        self.action_name = action_name
        self.description_json_raw = description_json_raw
        self.description_json: dict = description_json if description_json is not None else {}
        self.description_json_valid = description_json_valid
        self.active = active
        self.busy = busy


def _identity(info: CapabilityInfo) -> str:
    return f'{info.robot}/{info.capability}' if info.robot else info.capability


def _status_label(info: CapabilityInfo) -> str:
    if not info.active:
        return f'{ansi.RED}INACTIVE{ansi.RESET}'
    if info.busy:
        return f'{ansi.YELLOW}BUSY{ansi.RESET}'
    return f'{ansi.GREEN}IDLE{ansi.RESET}'


def _display_name(info: CapabilityInfo) -> str:
    if info.description_json_valid:
        return info.description_json.get('display_name', info.capability)
    return info.capability


def _bullet_list(lines: list[str], j: dict, key: str) -> None:
    values = j.get(key)
    if not isinstance(values, list):
        return
    lines.append(f'\n  {ansi.BOLD}{key}:{ansi.RESET}')
    for item in values:
        if isinstance(item, str):
            lines.append(f'    {ansi.DIM}-{ansi.RESET} {item}')


def print_capability_summary_line(info: CapabilityInfo) -> None:
    """Print a single summary line, e.g. for a "known capabilities" list."""
    safe_print(
        f'  {ansi.BOLD}{_identity(info)}{ansi.RESET} - {_display_name(info)}  '
        f'({ansi.DIM}{info.action_name}{ansi.RESET})  [{_status_label(info)}]')


def print_capability_info(info: CapabilityInfo) -> None:
    """Pretty-print the full capability description (requirements, effects, parameters, ...)."""
    title = f'{_identity(info)} -- {_display_name(info)}'
    bar = '=' * (len(title) + 4)

    lines = [
        f'{ansi.BOLD}{ansi.CYAN}{bar}\n  {title}   [{_status_label(info)}{ansi.CYAN}]\n'
        f'{bar}{ansi.RESET}',
        f'\n  {ansi.BOLD}Action:{ansi.RESET} {info.action_name}',
    ]

    if not info.description_json_valid:
        lines.append(
            f'{ansi.RED}  (could not parse the JSON description published on '
            f'/capabilities)\n{ansi.RESET}{ansi.DIM}{info.description_json_raw}{ansi.RESET}')
        safe_print('\n'.join(lines))
        return

    j = info.description_json
    lines.append(f"\n  {j.get('description', '(no description)')}")

    action = j.get('action')
    if isinstance(action, dict):
        lines.append(f"\n  {ansi.BOLD}Action type:{ansi.RESET} {action.get('type', '')}")

    _bullet_list(lines, j, 'requirements')
    _bullet_list(lines, j, 'effects')
    _bullet_list(lines, j, 'notes')

    parameters = j.get('parameters')
    if isinstance(parameters, dict):
        lines.append(f'\n  {ansi.BOLD}Parameters:{ansi.RESET}')
        for param_name, param_info in parameters.items():
            line = f'    {ansi.YELLOW}{param_name}{ansi.RESET}'
            if isinstance(param_info, dict) and 'type' in param_info:
                line += f" ({param_info['type']})"
            lines.append(line)
            if isinstance(param_info, dict) and 'description' in param_info:
                lines.append(f"        {ansi.DIM}{param_info['description']}{ansi.RESET}")

    safe_print('\n'.join(lines))
