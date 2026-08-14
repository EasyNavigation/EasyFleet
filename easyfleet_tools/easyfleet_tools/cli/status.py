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
from ..controller.ros_controllers import FleetDiscovery


class StatusVerb(VerbExtension):
    """
    Stream each robot's current mission-status-marker text.

    The same line RViz shows floating above it.
    """

    def add_arguments(self, parser, cli_name):
        add_arguments(parser)
        parser.add_argument('--duration', type=float, default=30.0, help='Seconds to watch.')

    def main(self, *, args):
        enable_color = sys.stdout.isatty() and not os.environ.get('NO_COLOR')
        with NodeStrategy(args) as node:
            discovery = FleetDiscovery(node)
            try:
                t_end = time.time() + args.duration
                while time.time() < t_end:
                    rclpy.spin_once(node, timeout_sec=0.1)
                    if discovery.robots:
                        lines = [
                            f'[bold]{name}[/bold]: {discovery.robots[name].status_text or "—"}'
                            for name in sorted(discovery.robots)
                        ]
                        sys.stdout.write('\033[1;1H\033[J')
                        sys.stdout.write(bbcode_to_ansi('\n'.join(lines), enable_color) + '\n')
                        sys.stdout.flush()
            except (KeyboardInterrupt, ExternalShutdownException):
                pass
            return 0
