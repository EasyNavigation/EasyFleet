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
from ..controller.ros_controllers import RosoutMonitor


class LogsVerb(VerbExtension):
    """
    Tail /rosout, colored by level.

    Optionally filtered to one robot/capability -- the CLI equivalent
    of each capability's log box in the TUI.
    """

    def add_arguments(self, parser, cli_name):
        add_arguments(parser)
        parser.add_argument('--robot', help='Only show logs from this robot.')
        parser.add_argument(
            '--capability',
            help='Only show logs from this capability type (requires --robot).')
        parser.add_argument('--duration', type=float, default=30.0, help='Seconds to tail.')

    def main(self, *, args):
        enable_color = sys.stdout.isatty() and not os.environ.get('NO_COLOR')
        logger_filter = None
        if args.robot and args.capability:
            logger_filter = f'{args.robot}.{args.capability}'

        with NodeStrategy(args) as node:
            monitor = RosoutMonitor(node)

            def on_log(msg, text):
                if logger_filter is not None and msg.name != logger_filter:
                    return
                if args.robot and args.capability is None and \
                        not msg.name.startswith(args.robot + '.'):
                    return
                sys.stdout.write(bbcode_to_ansi(text, enable_color) + '\n')
                sys.stdout.flush()

            monitor.add_on_any(on_log)
            try:
                t_end = time.time() + args.duration
                while time.time() < t_end:
                    rclpy.spin_once(node, timeout_sec=0.1)
            except (KeyboardInterrupt, ExternalShutdownException):
                pass
            return 0
