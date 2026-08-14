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

import os
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException

from ros2cli.node.strategy import add_arguments, NodeStrategy
from ros2cli.verb import VerbExtension

from ..cli.verb_utils import bbcode_to_ansi
from ..controller.ros_controllers import FleetDiscovery, STATUS_COLOR


def _render_fleet(discovery: FleetDiscovery) -> str:
    if not discovery.robots:
        return '(no robots discovered yet)'
    lines = []
    for robot_name in sorted(discovery.robots):
        robot = discovery.robots[robot_name]
        lines.append(f'[bold]{robot_name}[/bold]  {robot.status_text}')
        for capability_type in sorted(robot.capabilities):
            capability = robot.capabilities[capability_type]
            color = STATUS_COLOR[capability.status]
            lines.append(
                f'  {capability.capability} - {capability.display_name} '
                f'({capability.action_name})  [{color}]{capability.status}[/{color}]')
    return '\n'.join(lines)


class FleetVerb(VerbExtension):
    """Print every discovered robot and its capabilities, with IDLE/BUSY/INACTIVE status."""

    def add_arguments(self, parser, cli_name):
        add_arguments(parser)
        parser.add_argument(
            '--discovery-window', type=float, default=2.5,
            help='Seconds to listen before printing the first snapshot.')
        parser.add_argument(
            '--duration', type=float, default=0.0,
            help='If > 0, keep refreshing the snapshot for this many seconds.')

    def main(self, *, args):
        enable_color = sys.stdout.isatty() and not os.environ.get('NO_COLOR')
        with NodeStrategy(args) as node:
            discovery = FleetDiscovery(node)
            try:
                t_discover = time.time() + args.discovery_window
                while time.time() < t_discover:
                    rclpy.spin_once(node, timeout_sec=0.1)

                t_end = time.time() + args.duration
                while True:
                    text = bbcode_to_ansi(_render_fleet(discovery), enable_color)
                    if args.duration > 0:
                        sys.stdout.write('\033[1;1H\033[J')
                    sys.stdout.write(text + '\n')
                    sys.stdout.flush()
                    if time.time() >= t_end:
                        break
                    end_of_tick = time.time() + 1.0
                    while time.time() < end_of_tick:
                        rclpy.spin_once(node, timeout_sec=0.1)
            except (KeyboardInterrupt, ExternalShutdownException):
                pass
            return 0
