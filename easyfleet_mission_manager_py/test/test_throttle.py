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

"""Unit tests for Throttle. No ROS node required."""

import time

from easyfleet_mission_manager_py.throttle import Throttle


def test_first_call_is_always_ready():
    throttle = Throttle(1.0)
    assert throttle.ready() is True


def test_immediate_second_call_is_not_ready():
    throttle = Throttle(1.0)
    assert throttle.ready() is True
    assert throttle.ready() is False


def test_ready_again_after_interval_elapses():
    throttle = Throttle(0.05)
    assert throttle.ready() is True
    time.sleep(0.08)
    assert throttle.ready() is True
