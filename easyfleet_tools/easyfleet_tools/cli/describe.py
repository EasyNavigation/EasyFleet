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

import json
import os
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException

from ros2cli.node.strategy import add_arguments, NodeStrategy
from ros2cli.verb import VerbExtension

from ..cli.verb_utils import bbcode_to_ansi, resolve_capability
from ..controller.ros_controllers import FleetDiscovery, STATUS_COLOR


class DescribeVerb(VerbExtension):
    """
    Pretty-print one capability's full JSON description.

    Requirements, effects, parameters, ..., as published on /capabilities.
    """

    def add_arguments(self, parser, cli_name):
        add_arguments(parser)
        parser.add_argument(
            'capability',
            help="Either '<robot>/<capability>' (e.g. robot_1/navigation) "
                 'or a bare action name (e.g. /robot_1/navigation).')
        parser.add_argument(
            '--discovery-window', type=float, default=2.5,
            help='Seconds to listen on /capabilities before giving up.')

    def main(self, *, args):
        enable_color = sys.stdout.isatty() and not os.environ.get('NO_COLOR')
        with NodeStrategy(args) as node:
            discovery = FleetDiscovery(node)
            try:
                t_end = time.time() + args.discovery_window
                while time.time() < t_end:
                    rclpy.spin_once(node, timeout_sec=0.1)
                    found = resolve_capability(discovery, args.capability)
                    # Break only once a heartbeat has been seen too, not just
                    # the (instant, transient-local) /capabilities
                    # description -- otherwise the status line below would
                    # almost always show a stale INACTIVE.
                    if found is not None and found.last_heartbeat is not None:
                        break
            except (KeyboardInterrupt, ExternalShutdownException):
                return 0

            capability = resolve_capability(discovery, args.capability)
            if capability is None:
                sys.stderr.write(
                    f"No capability matching '{args.capability}' was discovered "
                    f'within {args.discovery_window}s.\n')
                return 1

            color = STATUS_COLOR[capability.status]
            header = (
                f'[bold]{capability.robot}/{capability.capability}[/bold] -- '
                f'{capability.display_name}   [{color}]{capability.status}[/{color}]\n'
                f'Action: {capability.action_name}\n'
            )
            sys.stdout.write(bbcode_to_ansi(header, enable_color))
            if capability.description_json is None:
                sys.stdout.write(
                    bbcode_to_ansi(
                        '[red](could not parse the JSON description published on '
                        '/capabilities)[/red]\n', enable_color))
                sys.stdout.write(capability.description_json_raw + '\n')
            else:
                sys.stdout.write(json.dumps(capability.description_json, indent=2) + '\n')
            sys.stdout.flush()
            return 0
