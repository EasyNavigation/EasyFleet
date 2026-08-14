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

from ..cli.verb_utils import bbcode_to_ansi, resolve_capability
from ..controller.ros_controllers import FleetDiscovery, GenericActionMonitor


class WatchVerb(VerbExtension):
    """
    Live goal status + feedback stream for one capability's action.

    Read-only, works on a goal sent by anyone (e.g. a mission script),
    not just one this command itself sent.
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
        parser.add_argument('--duration', type=float, default=30.0, help='Seconds to watch.')

    def main(self, *, args):
        enable_color = sys.stdout.isatty() and not os.environ.get('NO_COLOR')
        with NodeStrategy(args) as node:
            discovery = FleetDiscovery(node)
            try:
                t_end = time.time() + args.discovery_window
                while time.time() < t_end:
                    rclpy.spin_once(node, timeout_sec=0.1)
                    if resolve_capability(discovery, args.capability) is not None:
                        break
            except (KeyboardInterrupt, ExternalShutdownException):
                return 0

            capability = resolve_capability(discovery, args.capability)
            if capability is None:
                sys.stderr.write(
                    f"No capability matching '{args.capability}' was discovered "
                    f'within {args.discovery_window}s.\n')
                return 1

            monitor = GenericActionMonitor(node, capability.action_name)
            try:
                t_end = time.time() + args.duration
                while time.time() < t_end:
                    rclpy.spin_once(node, timeout_sec=0.1)
                    text = (
                        f'{capability.robot}/{capability.capability} '
                        f'({capability.action_name})\n\n'
                        f'{monitor.last_status_text}\n\n'
                        f'Feedback:\n{monitor.last_feedback_text}\n\n'
                        f'Result:\n{monitor.last_result_text}\n'
                    )
                    sys.stdout.write('\033[1;1H\033[J')
                    sys.stdout.write(bbcode_to_ansi(text, enable_color))
                    sys.stdout.flush()
            except (KeyboardInterrupt, ExternalShutdownException):
                pass
            finally:
                monitor.destroy()
            return 0
