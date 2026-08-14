#!/usr/bin/env python3

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

import atexit
import json
import os
import re
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException

from textual import events
from textual.app import App, ComposeResult
from textual.containers import Container, Grid, Horizontal, Vertical, VerticalScroll
from textual.widgets import Footer, Label, OptionList, RichLog, Static, Tab, Tabs
from textual.widgets.option_list import Option

from ..controller.ros_controllers import (
    CapabilityInfo,
    FleetDiscovery,
    GenericActionMonitor,
    RosoutMonitor,
    STATUS_COLOR,
)


# Prefer vendored textual package inside easyfleet_tools/vendor.
_vendor_dir = os.path.normpath(os.path.join(os.path.dirname(__file__), '..', 'vendor'))
if os.path.isdir(_vendor_dir) and _vendor_dir not in sys.path:
    sys.path.insert(0, _vendor_dir)


def _safe_id(name: str) -> str:
    """Turn a robot/capability name into a valid Textual widget/option id."""
    safe = re.sub(r'[^a-zA-Z0-9_-]', '_', name) or 'x'
    return safe if safe[0].isalpha() or safe[0] == '_' else f'x{safe}'


def _sync_options(option_list: OptionList, ordered_items: list) -> None:
    """
    Add/update `option_list`'s options to match `ordered_items`.

    `ordered_items` is a list of (id, text) pairs, in order; the current
    highlight is left untouched unless the list was empty before.
    Existing items are updated in place (`replace_option_prompt`), never
    removed -- a capability/robot that stops heartbeating goes INACTIVE
    in place, it doesn't vanish from the list.
    """
    known = getattr(option_list, '_ef_known_ids', None)
    if known is None:
        known = set()
        option_list._ef_known_ids = known
    was_empty = not known
    for option_id, text in ordered_items:
        if option_id in known:
            option_list.replace_option_prompt(option_id, text)
        else:
            option_list.add_option(Option(text, id=option_id))
            known.add(option_id)
    if was_empty and ordered_items and option_list.highlighted is None:
        option_list.highlighted = 0


def _reset_options(option_list: OptionList) -> None:
    option_list.clear_options()
    option_list._ef_known_ids = set()


def _highlighted_capability(option_list: OptionList, robot) -> CapabilityInfo | None:
    if option_list.highlighted is None:
        return None
    option_id = option_list.get_option_at_index(option_list.highlighted).id
    for capability in robot.capabilities.values():
        if _safe_id(capability.capability) == option_id:
            return capability
    return None


def _robot_option_text(name: str, status: str | None) -> str:
    color = STATUS_COLOR.get(status, 'white') if status else 'white'
    return f'[{color}]{name}[/{color}]'


def _capability_option_text(capability: CapabilityInfo) -> str:
    color = STATUS_COLOR[capability.status]
    return (
        f'{capability.capability} - {capability.display_name}  '
        f'[{color}]{capability.status}[/{color}]'
    )


def _capability_detail_text(capability: CapabilityInfo, monitor: GenericActionMonitor) -> str:
    if monitor is not None:
        status = monitor.last_status_text
        feedback = monitor.last_feedback_text
        result = monitor.last_result_text
    else:
        status = 'Goal status: [white]unknown[/white]'
        feedback = '(no feedback yet)'
        result = '(no result yet)'
    return (
        f'[bold]{capability.robot}/{capability.capability}[/bold]\n'
        f'{capability.action_name}\n\n'
        f'{status}\n\n'
        f'Feedback:\n{feedback}\n\n'
        f'Result:\n{result}'
    )


class EasyfleetTuiApp(App):
    """
    Read-only TUI for monitoring an EasyFleet system.

    A Mission tab (fleet-wide overview + drill-down into one
    robot/capability), one tab per discovered robot, and a fleet-wide
    Logs tab.
    """

    CSS = """
    Screen { layout: vertical; }
    #tabs { dock: top; }
    #pages { height: 1fr; width: 100%; }

    #page_mission { layout: horizontal; width: 100%; height: 100%; }
    #mission_left { width: 33%; height: 100%; layout: vertical; }
    #mission_right { width: 67%; height: 100%; layout: vertical; margin-left: 1; }

    .titled { layout: vertical; width: 100%; height: 1fr; margin-bottom: 1; }
    .title { padding: 0 1; height: auto; }
    .box { border: round; padding: 0 1; width: 100%; height: 1fr; overflow-y: auto; }
    .box_inner { width: 100%; height: auto; }

    #mission_clock { height: auto; border: round; padding: 0 1; margin-bottom: 1; }
    #mission_summary { height: auto; border: round; padding: 0 1; margin-bottom: 1; }
    #mission_warnings_block { height: 1fr; }
    #mission_warnings { height: 1fr; border: round; }

    .robot_page { layout: vertical; width: 100%; height: 100%; }
    .robot_header { height: auto; border: round; padding: 0 1; margin-bottom: 1; }
    .robot_grid { grid-size: 2 2; grid-gutter: 1; height: 1fr; width: 100%; }

    #page_logs { width: 100%; height: 100%; }
    #logs_view { height: 100%; width: 100%; border: round; }
    """

    BINDINGS = [
        ('q', 'quit', 'Salir'),
        ('ctrl+c', 'quit', 'Salir'),
    ]

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        rclpy.init(args=None)
        atexit.register(self._ros_shutdown)
        self.node = rclpy.create_node('easyfleet_tui')

        self.discovery = FleetDiscovery(self.node)
        self.rosout = RosoutMonitor(self.node)
        self.rosout.add_on_any(self._on_any_log_for_warnings_box)
        self.rosout.add_on_any(self._on_any_log_for_logs_tab)

        self._mission_selected_robot: str | None = None
        self._mission_selected_capability: str | None = None
        self._mission_monitor: GenericActionMonitor | None = None

        # robot_name -> dict of widget refs + selection/monitor state,
        # populated lazily as new robots are discovered.
        self._robot_widgets: dict = {}

        self._warnings_view: RichLog | None = None
        self._logs_view: RichLog | None = None

    def compose(self) -> ComposeResult:
        yield Tabs(
            Tab('Mission', id='tab_mission'),
            Tab('Logs', id='tab_logs'),
            id='tabs',
        )
        with Container(id='pages'):
            with Container(id='page_mission'):
                with Horizontal():
                    with Vertical(id='mission_left'):
                        yield Static('', id='mission_clock')
                        yield Static('', id='mission_summary')
                        with Vertical(id='mission_warnings_block'):
                            yield Label('Recent warnings/errors', classes='title')
                            self._warnings_view = RichLog(
                                id='mission_warnings', markup=True, wrap=True, max_lines=200)
                            yield self._warnings_view
                    with Vertical(id='mission_right'):
                        with Vertical(classes='titled'):
                            yield Label('Robots', classes='title')
                            yield OptionList(id='mission_robots', classes='box')
                        with Vertical(classes='titled'):
                            yield Label('Capabilities', classes='title')
                            yield OptionList(id='mission_caps', classes='box')
                        with Vertical(classes='titled'):
                            yield Label('Status / feedback', classes='title')
                            with VerticalScroll(classes='box'):
                                yield Static('', id='mission_detail', classes='box_inner')
            with Container(id='page_logs'):
                self._logs_view = RichLog(id='logs_view', markup=True, wrap=True, max_lines=1000)
                yield self._logs_view
        yield Footer()

    def on_mount(self) -> None:
        self._show_page('tab_mission')
        self.set_interval(0.05, self._ros_spin_once)
        self.set_interval(0.3, self._refresh_ui)
        self.set_interval(1.0, self._refresh_clock)

    # ---------- Tabs <-> pages ----------

    def on_tabs_tab_activated(self, event: Tabs.TabActivated) -> None:
        self._show_page(event.tab.id)

    def _show_page(self, tab_id: str) -> None:
        page_id = f'page_{tab_id[len("tab_"):]}'
        for container in self.query('#pages > Container'):
            container.display = container.id == page_id

    def on_key(self, event: events.Key) -> None:
        if event.key in '123456789':
            index = int(event.key) - 1
            tabs = self.query_one(Tabs)
            tab_ids = [tab.id for tab in tabs.query(Tab)]
            if index < len(tab_ids):
                tabs.active = tab_ids[index]

    # ---------- ROS polling ----------

    def _ros_spin_once(self) -> None:
        try:
            if rclpy.ok():
                rclpy.spin_once(self.node, timeout_sec=0.0)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass

    def _refresh_clock(self) -> None:
        self.query_one('#mission_clock', Static).update(time.strftime('%H:%M:%S'))

    # ---------- Periodic UI refresh ----------

    def _refresh_ui(self) -> None:
        for robot_name in self.discovery.robots:
            if robot_name not in self._robot_widgets:
                self._ensure_robot_tab(robot_name)

        self._refresh_summary()
        self._refresh_mission_tab()
        for robot_name, state in self._robot_widgets.items():
            self._refresh_robot_tab(robot_name, state)

    def _refresh_summary(self) -> None:
        robots = self.discovery.robots
        idle = sum(1 for r in robots.values() if r.worst_status == 'IDLE')
        busy = sum(1 for r in robots.values() if r.worst_status == 'BUSY')
        inactive = sum(1 for r in robots.values() if r.worst_status == 'INACTIVE')
        text = (
            f'Robots: {len(robots)}\n'
            f'[green]IDLE[/green]: {idle}   [yellow]BUSY[/yellow]: {busy}   '
            f'[red]INACTIVE[/red]: {inactive}'
        )
        self.query_one('#mission_summary', Static).update(text)

    def _refresh_mission_tab(self) -> None:
        robots_list = self.query_one('#mission_robots', OptionList)
        ordered = [
            (_safe_id(name), _robot_option_text(name, robot.worst_status))
            for name, robot in self.discovery.robots.items()
        ]
        _sync_options(robots_list, ordered)

        robot_name = None
        if robots_list.highlighted is not None:
            option = robots_list.get_option_at_index(robots_list.highlighted)
            robot_name = next(
                (n for n in self.discovery.robots if _safe_id(n) == option.id), None)

        caps_list = self.query_one('#mission_caps', OptionList)
        if robot_name != self._mission_selected_robot:
            _reset_options(caps_list)
            self._mission_selected_robot = robot_name
            self._mission_selected_capability = None
            self._set_mission_capability(None)

        capability = None
        if robot_name is not None:
            robot = self.discovery.robots[robot_name]
            ordered_caps = [
                (_safe_id(c.capability), _capability_option_text(c))
                for c in robot.capabilities.values()
            ]
            _sync_options(caps_list, ordered_caps)
            capability = _highlighted_capability(caps_list, robot)

        selected_type = capability.capability if capability is not None else None
        if selected_type != self._mission_selected_capability:
            self._mission_selected_capability = selected_type
            self._set_mission_capability(capability)

        detail = self.query_one('#mission_detail', Static)
        if capability is None:
            detail.update('(select a robot and a capability)')
        else:
            robot = self.discovery.robots[robot_name]
            text = _capability_detail_text(capability, self._mission_monitor)
            if robot.status_text:
                text += f'\n\nLast status marker: {robot.status_text}'
            detail.update(text)

    def _set_mission_capability(self, capability: CapabilityInfo | None) -> None:
        if self._mission_monitor is not None:
            self._mission_monitor.destroy()
            self._mission_monitor = None
        if capability is not None:
            self._mission_monitor = GenericActionMonitor(self.node, capability.action_name)

    # ---------- Per-robot tabs ----------

    def _ensure_robot_tab(self, robot_name: str) -> None:
        sid = _safe_id(robot_name)
        tab_id = f'tab_robot_{sid}'
        page_id = f'page_robot_{sid}'

        tabs = self.query_one(Tabs)
        tabs.add_tab(Tab(robot_name, id=tab_id), before='tab_logs')

        header = Static('', id=f'robot_header_{sid}', classes='robot_header')
        caps_list = OptionList(id=f'cap_list_{sid}', classes='box')
        desc = Static('', id=f'cap_desc_{sid}', classes='box_inner')
        execution = Static('', id=f'cap_exec_{sid}', classes='box_inner')
        logs_view = RichLog(id=f'cap_logs_{sid}', markup=True, wrap=True, max_lines=500)

        grid = Grid(
            Vertical(Label('Capabilities', classes='title'), caps_list, classes='titled'),
            Vertical(
                Label('Description', classes='title'),
                VerticalScroll(desc, classes='box'),
                classes='titled'),
            Vertical(
                Label('Execution', classes='title'),
                VerticalScroll(execution, classes='box'),
                classes='titled'),
            Vertical(Label('Logs', classes='title'), logs_view, classes='titled'),
            classes='robot_grid',
        )
        page = Container(header, grid, id=page_id, classes='robot_page')
        page.display = False
        self.query_one('#pages').mount(page)

        self._robot_widgets[robot_name] = {
            'header': header,
            'caps_list': caps_list,
            'desc': desc,
            'exec': execution,
            'logs_view': logs_view,
            'selected_capability': None,
            'monitor': None,
            'registered_logger': None,
            'log_callback': None,
        }

    def _refresh_robot_tab(self, robot_name: str, state: dict) -> None:
        robot = self.discovery.robots[robot_name]
        state['header'].update(robot.status_text or '(no status yet)')

        ordered_caps = [
            (_safe_id(c.capability), _capability_option_text(c))
            for c in robot.capabilities.values()
        ]
        _sync_options(state['caps_list'], ordered_caps)
        capability = _highlighted_capability(state['caps_list'], robot)

        selected_type = capability.capability if capability is not None else None
        if selected_type != state['selected_capability']:
            self._set_robot_tab_capability(robot_name, state, capability)

        if capability is None:
            state['desc'].update('(select a capability)')
            state['exec'].update('(select a capability)')
        else:
            if capability.description_json is not None:
                state['desc'].update(json.dumps(capability.description_json, indent=2))
            else:
                state['desc'].update(capability.description_json_raw or '(no description yet)')
            state['exec'].update(_capability_detail_text(capability, state['monitor']))

    def _set_robot_tab_capability(
            self, robot_name: str, state: dict, capability: CapabilityInfo | None) -> None:
        if state['monitor'] is not None:
            state['monitor'].destroy()
            state['monitor'] = None
        if state['registered_logger'] is not None:
            self.rosout.unregister(state['registered_logger'], state['log_callback'])
            state['registered_logger'] = None
            state['log_callback'] = None

        state['selected_capability'] = capability.capability if capability is not None else None
        if capability is None:
            return

        state['monitor'] = GenericActionMonitor(self.node, capability.action_name)
        logger_name = f'{capability.robot}.{capability.capability}'

        def log_callback(msg, text, logs_view=state['logs_view']):
            logs_view.write(text)

        state['log_callback'] = log_callback
        state['registered_logger'] = logger_name
        self.rosout.register(logger_name, log_callback)

    # ---------- Fleet-wide logs ----------

    def _on_any_log_for_warnings_box(self, msg, text: str) -> None:
        if msg.level >= 30 and self._warnings_view is not None:  # Log.WARN
            self._warnings_view.write(text)

    def _on_any_log_for_logs_tab(self, msg, text: str) -> None:
        if self._logs_view is not None:
            self._logs_view.write(text)

    # ---------- Shutdown ----------

    def _ros_shutdown(self) -> None:
        if rclpy.ok():
            try:
                self.node.destroy_node()
            except Exception:
                pass
            rclpy.shutdown()


if __name__ == '__main__':
    EasyfleetTuiApp().run()


def run_app() -> None:
    EasyfleetTuiApp().run()
