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

"""
The Python-API mission script for this scenario.

Written against easyfleet_mission_manager_py's RobotHandle/
SimpleController -- see
easyfleet_easynav_collaboration_simple_api_deployment/src/main_easynav_collaboration.cpp
for the C++ sibling this is meant to behave identically to (same
robots, same waypoints, same five phases, same status markers). Status
markers above each robot in RViz are not set here explicitly:
RobotHandle.run_capability() publishes them automatically for the life
of each goal.

Mission control demo for this deployment scenario: two robots, both
real-EasyNav-navigating on the same shared map --
  - robot_1: navigation (real EasyNav), perception (mock)
  - robot_2: navigation (real EasyNav), manipulation (mock)

A five-phase choreography exercising real navigation end to end (goals
are left to actually SUCCEED, not stopped early -- see LONG_TIMEOUT_SEC
below -- except where phase 4 deliberately cancels one):
  1. robot_1 -> "kitchen" (perceiving throughout) while robot_2 ->
     "dock", simultaneously. Waits for both navigations to actually
     finish (not a timeout).
  2. robot_1 -> "kitchen_standby" (1m short of "kitchen", still facing
     it, perception still running from phase 1) to clear space, while
     robot_2 -> "kitchen", simultaneously.
  3. Once robot_2 is at "kitchen" (and robot_1's perception is
     stopped, its job here done), robot_2 runs manipulation for up to
     10s.
  4. robot_1 -> "dock" while robot_2 -> "dock" too, simultaneously;
     10s into robot_1's navigation it is explicitly stopped and
     redirected to "charging_station" instead.
  5. Once both navigations from phase 4 finish, both robots return to
     their own starting pose ("robot_1_home"/"robot_2_home") and any
     still-running capability goal is stopped.
"""

from easyfleet_mission_manager_py import (
    ansi,
    make_manipulation_goal,
    make_navigation_goal,
    make_perception_goal,
    Manipulation,
    Navigation,
    Perception,
    print_section,
    print_step,
    RobotHandle,
    SimpleController,
)
from easyfleet_mission_manager_py.output import safe_print
import rclpy

# This demo is specific to this deployment scenario, which defines
# exactly these two robots (see .../launch/).
ROBOT_1 = 'robot_1'
ROBOT_2 = 'robot_2'

# Named waypoints configured on both robots' navigation capability, see
# config/navigation_params.yaml -- real, reachable points on the shared
# home2 map.
DOCK = 'dock'
KITCHEN = 'kitchen'
KITCHEN_STANDBY = 'kitchen_standby'
CHARGING_STATION = 'charging_station'
ROBOT_1_HOME = 'robot_1_home'
ROBOT_2_HOME = 'robot_2_home'

# Real navigation goals are meant to run to actual completion in this
# mission -- this is a generous safety net, not the expected way a
# goal ends, unlike RUN_TIMEOUT_SEC (10s), which is both too short for
# real navigation and -- for every navigation goal below except one --
# not what's wanted here.
LONG_TIMEOUT_SEC = 180.0


def main(args=None):
    rclpy.init(args=args)

    controller = SimpleController()

    robot_1 = RobotHandle(ROBOT_1)
    robot_2 = RobotHandle(ROBOT_2)

    controller.add_robot(robot_1)
    controller.add_robot(robot_2)

    safe_print(
        f'{ansi.BOLD}{ansi.CYAN}Control Center -- two-robot "easynav" mission '
        f'({ROBOT_1}, {ROBOT_2}){ansi.RESET}')

    print_section('Phase 0: Discovering capabilities')
    print_step('listening on /capabilities and /capabilities_status ...')
    controller.discover_capabilities()

    robot_1.print_capabilities()
    robot_2.print_capabilities()

    if not robot_1.has_capability('navigation') or not robot_2.has_capability('navigation'):
        safe_print(
            f'{ansi.BOLD}{ansi.RED}Both robots must have a navigation capability for this '
            f'demo to work.{ansi.RESET}')
        controller.shutdown()
        return 1

    if not robot_1.has_capability('perception'):
        safe_print(
            f'{ansi.BOLD}{ansi.RED}Robot 1 must have a perception capability for this demo '
            f'to work.{ansi.RESET}')
        controller.shutdown()
        return 1

    if not robot_2.has_capability('manipulation'):
        safe_print(
            f'{ansi.BOLD}{ansi.RED}Robot 2 must have a manipulation capability for this demo '
            f'to work.{ansi.RESET}')
        controller.shutdown()
        return 1

    # Phase 1: robot_1 -> "kitchen" (perceiving throughout) while
    # robot_2 -> "dock", simultaneously. robot_1's perception is
    # started here and kept running (untouched) across phase 2 too --
    # it's only stopped once robot_1 actually reaches
    # "kitchen_standby" at the end of phase 2, see there.

    print_section(
        f'Phase 1: {ROBOT_1} -> "{KITCHEN}" (perceiving), {ROBOT_2} -> "{DOCK}" '
        '(simultaneously)')

    robot_1.run_capability(
        'navigation', Navigation, make_navigation_goal(KITCHEN), timeout_sec=LONG_TIMEOUT_SEC)
    robot_1.run_capability(
        'perception', Perception, make_perception_goal(), timeout_sec=LONG_TIMEOUT_SEC)
    robot_2.run_capability(
        'navigation', Navigation, make_navigation_goal(DOCK), timeout_sec=LONG_TIMEOUT_SEC)

    while robot_1.is_capability_running('navigation') or robot_2.is_capability_running(
            'navigation'):
        controller.spin_some()

    print_step('Phase 1 done: both robots reached their waypoint.')

    # Phase 2: robot_1 -> "kitchen_standby" (still perceiving) while
    # robot_2 -> "kitchen", simultaneously.
    print_section(
        f'Phase 2: {ROBOT_1} -> "{KITCHEN_STANDBY}" (clearing space, still perceiving), '
        f'{ROBOT_2} -> "{KITCHEN}" (simultaneously)')

    # robot_1's perception keeps running, untouched, from phase 1 --
    # no need to (re-)issue it here; doing so would just preempt the
    # still-running goal with an identical one.
    robot_1.run_capability(
        'navigation', Navigation, make_navigation_goal(KITCHEN_STANDBY),
        timeout_sec=LONG_TIMEOUT_SEC)
    robot_2.run_capability(
        'navigation', Navigation, make_navigation_goal(KITCHEN), timeout_sec=LONG_TIMEOUT_SEC)

    while robot_1.is_capability_running('navigation') or robot_2.is_capability_running(
            'navigation'):
        controller.spin_some()

    robot_1.stop_capability('perception')

    print_step(f'Phase 2 done: {ROBOT_1} clear of "{KITCHEN}", {ROBOT_2} there.')

    # Phase 3: robot_2, now at "kitchen", runs manipulation for up to
    # 10s (the mock's own configured duration finishes well within
    # that).
    print_section(f'Phase 3: {ROBOT_2} manipulation at "{KITCHEN}"')

    # Default timeout (RUN_TIMEOUT_SEC, 10s) already matches what this
    # phase wants.
    robot_2.run_capability('manipulation', Manipulation, make_manipulation_goal())
    controller.spin_for(10.0)

    robot_2.stop_capability('manipulation')

    print_step('Phase 3 done.')

    # Phase 4: robot_1 -> "dock" while robot_2 -> "dock" too (robot_2
    # was still at "kitchen" from phase 3, so this is a real move, not
    # a no-op), simultaneously; 10s into robot_1's navigation it is
    # explicitly stopped (via run_capability's own timeout-then-cancel
    # behavior) and redirected to "charging_station" instead.
    print_section(
        f'Phase 4: {ROBOT_1} -> "{DOCK}" (canceled after 10s) -> "{CHARGING_STATION}", '
        f'{ROBOT_2} -> "{DOCK}" (simultaneously)')

    # Default timeout (RUN_TIMEOUT_SEC, 10s) here is deliberate: this
    # is the goal meant to be capped and redirected below, unlike
    # every other navigation goal in this mission.
    robot_1.run_capability('navigation', Navigation, make_navigation_goal(DOCK))
    robot_2.run_capability(
        'navigation', Navigation, make_navigation_goal(DOCK), timeout_sec=LONG_TIMEOUT_SEC)

    controller.spin_for(10.0)

    # A new goal on the same capability preempts the one still in
    # flight (server-side preemption) -- no explicit stop_capability()
    # needed first.
    robot_1.run_capability(
        'navigation', Navigation, make_navigation_goal(CHARGING_STATION),
        timeout_sec=LONG_TIMEOUT_SEC)

    while robot_1.is_capability_running('navigation') or robot_2.is_capability_running(
            'navigation'):
        controller.spin_some()

    print_step(
        f'Phase 4 done: {ROBOT_1} at "{CHARGING_STATION}", {ROBOT_2} at "{DOCK}".')

    # Phase 5: both robots return to their own starting pose,
    # simultaneously.
    print_section(
        f'Phase 5: {ROBOT_1} -> "{ROBOT_1_HOME}", {ROBOT_2} -> "{ROBOT_2_HOME}" '
        '(simultaneously)')

    robot_1.run_capability(
        'navigation', Navigation, make_navigation_goal(ROBOT_1_HOME), timeout_sec=LONG_TIMEOUT_SEC)
    robot_2.run_capability(
        'navigation', Navigation, make_navigation_goal(ROBOT_2_HOME), timeout_sec=LONG_TIMEOUT_SEC)

    while robot_1.is_capability_running('navigation') or robot_2.is_capability_running(
            'navigation'):
        controller.spin_some()

    # Safety net: nothing should still be running by this point
    # (perception was stopped after phase 2, manipulation after phase
    # 3, and every navigation goal above already ran to completion or
    # was explicitly redirected), but stop anything left active
    # regardless before declaring the mission over.
    robot_1.stop_capability('perception')
    robot_2.stop_capability('manipulation')

    print_step('Phase 5 done: both robots back at their starting pose.')
    print_section('Mission complete')

    controller.shutdown()

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
