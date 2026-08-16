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

"""Serialized terminal output, mirroring easyfleet_mission_manager's output.hpp."""

import threading

_output_lock = threading.Lock()


def safe_print(line: str) -> None:
    """Print `line`, serialized across every thread that calls this."""
    with _output_lock:
        print(line, flush=True)
