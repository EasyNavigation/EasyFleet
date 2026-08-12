<p align="center">
  <img src="docs/easyfleet_white_back.png" alt="EasyFleet logo" width="400">
</p>

# EasyFleet

[![rolling](https://github.com/EasyNavigation/EasyFleet/actions/workflows/rolling.yaml/badge.svg?branch=rolling)](https://github.com/EasyNavigation/EasyFleet/actions/workflows/rolling.yaml)
[![codecov](https://codecov.io/github/EasyNavigation/EasyFleet/graph/badge.svg)](https://codecov.io/github/EasyNavigation/EasyFleet)

A ROS 2 (Rolling) workspace exploring a **capability-oriented architecture**
for multi-robot, multi-skill fleets. It provides:

- **`easyfleet_core`** — a small, reusable C++ framework for building
  self-describing, lifecycle-managed ROS 2 actions ("capabilities"), plus a
  comfortable client-side API for calling them. A `Capability<ActionServerT>`
  resolves its own robot identity from its ROS namespace at construction
  time, so the same node binary announces itself correctly whether it's
  launched as `/robot1/navigation` or `/robot2/navigation`. It also provides
  the per-domain `NavigationActionServerBase` / `ManipulationActionServerBase`
  / `PerceptionActionServerBase` classes: each fixes `ActionServerBase` to
  the matching `easyfleet_interfaces` action, so a concrete backend only has
  to subclass one of them and implement `on_goal_received()`/`on_execute()`
  — it never has to spell out the action type itself.
- **`easyfleet_interfaces`** — the architecture-level message contract
  (`CapabilityDescription`, `CapabilityStatus`) every capability publishes on
  `/capabilities` and `/capabilities_status`, regardless of manufacturer or
  implementation language, plus the **`Navigation`**, **`Manipulation`** and
  **`Perception`** actions: general-purpose, framework-agnostic interfaces
  any robot's implementation of those capability classes can be expressed
  through (no dependency on Nav2, MoveIt, or any other specific stack — see
  [Interfaces](#interfaces) below).
- **`easyfleet_mission_manager`** — a C++ library of reusable mission-scripting
  building blocks: capability discovery (via `/capabilities` and
  `/capabilities_status`), pretty-printing helpers, and a generic
  `run_capability<ActionT>()` helper built on `easyfleet_core::CapabilityClient`.
  Ships no executables of its own — it's meant to grow into the shared logic
  behind any mission manager, demo or otherwise.
- **`easyfleet_easynav_navigation`** — the first **real** capability backend:
  drives an actual [EasyNav](../EasyNavigation) navigation stack (via
  `easynav::GoalManagerClient`), structured internally as a
  BehaviorTree.CPP tree (`StartOff` → `Navigate` → `Finish`). The reference
  example of a real `easyfleet_core::NavigationActionServerBase` subclass —
  see [Extending EasyFleet](#extending-easyfleet) below.
- **`easyfleet_example_deployments`** — the example deployments: a
  "fake"/mock implementation of each capability (`navigation`,
  `manipulation`, `perception`, each a subclass of the matching
  `easyfleet_core` `*ActionServerBase`), the `alone`/`collaboration` example
  mission scripts built on `easyfleet_mission_manager`, and the launch files
  + per-robot JSON/parameter configuration that assemble them into concrete
  multi-robot scenarios — plus an `easynav` scenario that swaps the mock
  navigation capability for `easyfleet_easynav_navigation` against a real
  (if `Dummy`-plugin-configured) EasyNav stack.

Everything under `easyfleet_example_deployments/` capability-wise is a
**mock**: it doesn't drive a real robot or run a real detector/manipulator.
The goal of this repo is to validate the architecture and plumbing of a
multi-robot, multi-capability system (discovery, namespacing, lifecycle,
preemption, cancellation, feedback) before wiring in real
navigation/perception/manipulation stacks. The intended real-world shape is
a control computer running an LLM-generated Behavior Tree (via
BehaviorTreeCPP) whose action nodes are `easyfleet_core::CapabilityClient`s;
the example mission scripts here are a first approximation of that control
computer.

## Repository layout

```
EasyFleet/
├── easyfleet_core/                  ActionServerBase, ActionClient, Capability, CapabilityClient,
│                                     Navigation/Manipulation/PerceptionActionServerBase
├── easyfleet_interfaces/            CapabilityDescription/CapabilityStatus + Navigation/Manipulation/Perception actions
├── easyfleet_mission_manager/       Capability discovery + mission-scripting helper library (no executables)
├── easyfleet_easynav_navigation/    Real navigation capability, backed by EasyNav + BehaviorTree.CPP
│   ├── include|src/easyfleet_easynav_navigation/bt_nodes/{start_off,navigate,finish}.hpp|cpp
│   ├── include|src/easyfleet_easynav_navigation/easynav_navigation_capability.hpp|cpp
│   └── behavior_trees/navigate.xml
└── easyfleet_example_deployments/   Fake capability implementations, example missions, launch/config
    ├── include|src/easyfleet_example_deployments/{navigation,manipulation,perception}_fake_capability.hpp|cpp
    ├── src/{navigation,manipulation,perception}_fake_capability_main.cpp
    ├── src/main_{alone,collaboration,easynav}.cpp
    ├── launch/{alone,collaboration,easynav}/    each with a top-level <scenario>_launch.yaml
    │                                             including per-robot <robot>_launch.yaml files
    │                                             (easynav also has a real-navigation variant:
    │                                             easynav_gazebo_launch.yaml -> easynav_robot_gazebo_launch.yaml)
    └── config/{alone,collaboration,easynav}/<robot>/   per-robot JSON/parameter files
```

## Extending EasyFleet

Any integrator can add their own backend for a capability domain by
subclassing the matching `easyfleet_core` base class in their own package,
without touching `easyfleet_core` or `easyfleet_example_deployments`:

```cpp
// my_nav2_capability/include/my_nav2_capability/navigation_nav2_capability.hpp
class NavigationNav2ActionServer : public easyfleet_core::NavigationActionServerBase
{
  // implement on_goal_received()/on_execute() by delegating to a real
  // nav2_msgs/action/NavigateToPose client internally
};
class NavigationNav2Capability : public easyfleet_core::Capability<NavigationNav2ActionServer>
{
  explicit NavigationNav2Capability(const rclcpp::NodeOptions & options = {})
  : easyfleet_core::Capability<NavigationNav2ActionServer>("navigation", options) {}
};
```

Two real reference implementations of this exact pattern exist in this
repo, at opposite ends of the "mock vs. real" spectrum:
- `easyfleet_example_deployments`'s `navigation_fake_capability.hpp`/`.cpp`
  — a mock, simulates progress on a timer, no external dependency.
- `easyfleet_easynav_navigation` — a real backend, driving an actual
  EasyNav navigation stack (see [below](#the-easynav-backed-navigation-capability)).

Either is a good template for a new `navigation_<your_backend>_capability`
(or `manipulation_*`/`perception_*`) package. Everything else (announcing
on `/capabilities`, the heartbeat, preemption bookkeeping, discovery,
`CapabilityClient`) is handled by `easyfleet_core` and needs no changes.

### The EasyNav-backed navigation capability

`easyfleet_easynav_navigation` drives [EasyNav](../EasyNavigation) via
`easynav::GoalManagerClient`, which only accepts raw poses — it has no
concept of a named waypoint. So this capability owns that mapping itself:
`navigation.waypoint_ids` + `navigation.waypoints.<id>.{frame_id,x,y,yaw}`
parameters define a fixed set of named waypoints, and an incoming
`Navigation` goal selects one via `parameters_json: {"goal_id": "<id>"}`
(`target_pose`/`waypoints` on the goal itself go unused by this backend —
see [Interfaces](#interfaces)).

Internally, each goal ticks a `behaviortree_cpp` v4 tree
(`behavior_tree_xml`, default
`behavior_trees/navigate.xml`) with three custom `StatefulActionNode`s:
- **`StartOff`** / **`Finish`** — bookends, no ports, just take 2s and print
  a message.
- **`Navigate`** — reads the target waypoint id from its `goal_id` input
  port, resolves it, and calls `GoalManagerClient::send_goal()`.

One `GoalManagerClient` is kept alive for the capability's whole lifetime
(not recreated per ROS goal) — so when a new goal preempts an in-flight
one, its fresh `Navigate` node redirects EasyNav to the new target via that
same client, which EasyNav treats as a preemption of the goal it already
has from that client id, no special-cased "resume in place" logic needed.
A genuine cancellation (or capability shutdown), on the other hand, calls
`GoalManagerClient::cancel()` before settling the goal — the robot actually
stops, rather than continuing to navigate toward an abandoned goal.

The `easynav` scenario in `easyfleet_example_deployments` runs this
capability against real EasyNav (`easynav_system system_main`) configured
with its own `Dummy*` plugins — see
[Multi-robot deployments](#multi-robot-deployments).

## Interfaces

`easyfleet_interfaces` defines three actions general enough that any robot's
implementation of a capability class — any navigation stack, any
manipulator, any perception modality — can express itself through the same
interface, with a `parameters_json` escape hatch (same convention as
`CapabilityDescription.description_json`) for stack-specific tuning:

- **`Navigation`**: go to `target_pose`, optionally via ordered `waypoints`.
- **`Manipulation`**: reach a `joint_target`, a `pose_target`, or run a
  `named_task`, selected by `mode`.
- **`Perception`**: detect/report `object_classes`, once or `continuous`ly.

All three share a result-code convention (`SUCCESS=0`, `REJECTED=1`,
`ABORTED=2`, `CANCELED=3`, `TIMEOUT=4`, with capability-specific codes
starting at `10`) — a documented convention, not enforced by the type
system, since `.action` files can't share constants across packages.

## What's a "capability"?

A capability is a `rclcpp_lifecycle::LifecycleNode` (`easyfleet_core::Capability<ActionServerT>`)
that wraps exactly one ROS 2 action and, once activated:

- Publishes a `CapabilityDescription` (`easyfleet_interfaces`) on the
  **reliable, transient-local** `/capabilities` topic, so anyone subscribing
  late still gets it. It carries `robot`, `capability`, `action_name` (all
  resolved from the node's actual ROS namespace at runtime) and the raw JSON
  description (what it does, requirements, effects, parameters) read from
  its `capabilities_file` parameter.
- Publishes a 1 Hz `CapabilityStatus` heartbeat (same `robot`/`capability`/
  `action_name` identity, plus a `busy` flag) on `/capabilities_status` for
  as long as it stays active — this is how a mission manager knows a
  capability is not just registered but actually alive, and whether it
  currently has a goal executing (`BUSY`) or is free to be called (`IDLE`),
  without having to send it a goal just to find out.
- Exposes a `<action_name>.allow_preemption` parameter controlling whether a
  new goal can replace ("preempt") one that's currently running, or must be
  rejected until the current one finishes or is canceled.
- Can always be told to stop early: canceling the goal (or, from client
  code, `CapabilityClient::cancel()`) stops whatever the capability was
  doing — the robot stops moving, the perception stream stops reporting
  detections, the manipulator stops executing its trajectory, etc.
- Logs a startup line naming its robot and capability, and a start/finish
  line around every goal it executes, so `ros2 launch` output stays legible
  even with several robots and capabilities running at once.

## Multi-robot deployments

Because a capability resolves its `robot`/`action_name` identity from its
own ROS namespace, the same node binary (e.g. `navigation_fake_capability_node`)
can be launched under any namespace and will announce itself correctly —
`easyfleet_example_deployments` uses this to assemble several example
scenarios, each self-contained under its own `launch/<scenario>/` +
`config/<scenario>/`:

| Scenario | Robot | Capabilities | Launch file |
|---|---|---|---|
| `alone` | `robot_1` | navigation, manipulation, perception (mock) | `easyfleet_example_deployments/launch/alone/alone_launch.yaml` |
| `collaboration` | `robot_1` | navigation, perception (mock) | `easyfleet_example_deployments/launch/collaboration/collaboration_launch.yaml` |
| `collaboration` | `robot_2` | navigation, perception (mock) | (same) |
| `collaboration` | `robot_3` | navigation, manipulation (mock) | (same) |
| `easynav` | `easynav_robot` | navigation (real EasyNav backend, `Dummy*` plugins) + `easynav_system system_main` | `easyfleet_example_deployments/launch/easynav/easynav_launch.yaml` |
| `easynav` (real navigation) | *(unnamespaced)* | navigation (real EasyNav backend, real costmap/localizer/planner/controller) + `easynav_system system_main` + `rviz2` | `easyfleet_example_deployments/launch/easynav/easynav_gazebo_launch.yaml` |

Each robot has its own per-robot launch file (e.g.
`easyfleet_example_deployments/launch/collaboration/robot_3_launch.yaml`) that
can also be run standalone, and its own JSON/parameter files under
`easyfleet_example_deployments/config/<scenario>/<robot>/`. Namespacing keeps
two robots that share a capability type (e.g. `navigation` on both
`robot_1` and `robot_3`) fully distinct: their resolved `action_name`s
(`/robot_1/navigation` vs. `/robot_3/navigation`) never collide, and each
can be discovered and called independently. The real-navigation `easynav`
variant is the one exception: it is deliberately **not** namespaced (see
[below](#easynav-real-navigation-scenario)), so it only ever runs one robot
at a time.

## Packages

| Package | Type | What it is |
|---|---|---|
| `easyfleet_core` | C++ library | `ActionServerBase<ActionT>`, `ActionClient<ActionT>`, `Capability<ActionServerT>`, `CapabilityClient<ActionT>`, plus `Navigation`/`Manipulation`/`PerceptionActionServerBase` |
| `easyfleet_interfaces` | Interface package | `CapabilityDescription`, `CapabilityStatus`, and the `Navigation`/`Manipulation`/`Perception` actions |
| `easyfleet_mission_manager` | C++ library | Capability discovery/print helpers and the generic `run_capability<ActionT>()` helper — no executables of its own |
| `easyfleet_easynav_navigation` | C++ library + 1 executable | Real `navigation` capability backed by EasyNav + BehaviorTree.CPP (`easynav_navigation_capability_node`) |
| `easyfleet_example_deployments` | C++ library + 6 executables + launch/config | The 3 fake capabilities (`navigation_fake_capability_node`, `manipulation_fake_capability_node`, `perception_fake_capability_node`), the 3 example missions (`alone_mission_node`, `collaboration_mission_node`, `easynav_mission_node`), and the `alone`/`collaboration`/`easynav` scenario assembly (the `easynav` scenario's robot itself runs executables from `easyfleet_easynav_navigation` and `easynav_system` instead of the fake capabilities) |

Note: `easyfleet_example_deployments` intentionally keeps its own descriptive
name rather than an `easyfleet_*` prefix, distinguishing it as an
example/deployment package rather than core EasyFleet infrastructure.

## Prerequisites

- Ubuntu with **ROS 2 Rolling** installed (this workspace is pixi-managed;
  enter the environment with e.g. `pixi-set-ros rolling` before building).
- `colcon` and the usual ROS 2 build tooling.
- `nlohmann-json3-dev` (used by `easyfleet_mission_manager` and
  `easyfleet_easynav_navigation` to parse JSON) and `behaviortree_cpp`
  (used by `easyfleet_easynav_navigation`) — installed automatically by
  `rosdep`/`pixi` below.
- [EasyNavigation](../EasyNavigation) (specifically `easynav_system`) built
  in the same workspace — required by `easyfleet_easynav_navigation` and
  the `easynav` example scenario.
- For the `easynav` (real navigation) scenario specifically:
  [easynav_indoor_testcase](../easynav_indoor_testcase) built in the same
  workspace (its `home2` map and the EasyNav plugins used by
  `costmap.rpp.params.yaml` — `easynav_costmap_localizer`,
  `easynav_costmap_maps_manager`, `easynav_costmap_planner`,
  `easynav_regulated_pp_controller`), `rviz2`, and a real robot or Gazebo
  simulation publishing `scan_raw`/odometry/TF, started separately.

## Building

```bash
cd ~/ros/ros2/easyfleet_ws
pixi-set-ros rolling   # or: eval "$(pixi shell-hook -e rolling --frozen)"
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Running

Each capability node **activates itself on startup** — no external lifecycle
manager or `ros2 lifecycle set` calls are needed. Bring up a scenario from
`easyfleet_example_deployments`, then run the matching example mission,
also in `easyfleet_example_deployments`.

### `alone` scenario

A single robot, `robot_1`, carrying all three capabilities, all namespaced
under `/robot_1`.

Terminal 1:

```bash
ros2 launch easyfleet_example_deployments alone_launch.yaml
```

Terminal 2, once it's up:

```bash
ros2 run easyfleet_example_deployments alone_mission_node
```

`alone_mission_node` will, in order, print what it's doing as it happens:

1. **Discover** which of `robot_1`'s capabilities are currently active and
   print each one's full description (from its JSON on `/capabilities`).
2. Run `navigation`, `manipulation` and `perception` **sequentially**, each
   for up to 10 seconds or until it finishes on its own (whichever comes
   first), printing live feedback.
3. Run all three **in parallel**, same 10-second rule, printing interleaved
   live feedback from all of them.

### `collaboration` scenario

Three robots: `robot_1` and `robot_2` (navigation + perception each), and
`robot_3` (navigation + manipulation).

Terminal 1:

```bash
ros2 launch easyfleet_example_deployments collaboration_launch.yaml
```

Terminal 2, once it's up:

```bash
ros2 run easyfleet_example_deployments collaboration_mission_node
```

`collaboration_mission_node` runs a two-phase mission:

1. **Discover** all six active capabilities across the three robots and
   print each one's full description.
2. **Phase 1**: `robot_1` and `robot_2` run `navigation` and `perception`
   together, all four at once, for up to 10 seconds, with feedback labeled
   by each capability's resolved action name (e.g. `[/robot_1/navigation]`
   vs. `[/robot_2/navigation]`) so the interleaved output stays legible.
3. **Phase 2**: `robot_3` runs `navigation` to completion, then, once it
   finishes, runs `manipulation`.

### `easynav` scenario

A single robot, `easynav_robot`, whose `navigation` capability is the real
EasyNav-backed one (`easyfleet_easynav_navigation`), not the mock — EasyNav
itself (`easynav_system system_main`) runs configured with its own
`Dummy*` plugins (see
[The EasyNav-backed navigation capability](#the-easynav-backed-navigation-capability)).

Terminal 1:

```bash
ros2 launch easyfleet_example_deployments easynav_launch.yaml
```

Terminal 2, once it's up:

```bash
ros2 run easyfleet_example_deployments easynav_mission_node
```

`easynav_mission_node` will, in order, print what it's doing as it happens:

1. **Discover** `easynav_robot`'s `navigation` capability and print its full
   description (from its JSON on `/capabilities`).
2. Send it to the `dock` waypoint (one of the three configured in
   `config/easynav/easynav_robot/navigation_params.yaml` — `dock`,
   `kitchen`, `charging_station`) for up to 10 seconds. With the `Dummy*`
   EasyNav plugins, the goal is accepted and feedback keeps arriving, but —
   same as `perception_fake_capability` — it never completes on its own
   (`DummyLocalizer` never reports a robot pose), so the mission stops it
   once the 10s elapse, which sends EasyNav an explicit cancellation.
3. Send a goal to `dock` again, then, 3 seconds later, a second goal to
   `kitchen` while the first is still running: the first goal is aborted at
   the ROS level (preempted), but EasyNav itself is redirected to `kitchen`
   without stopping first — demonstrating EasyNav-level preemption via the
   capability's persistent `GoalManagerClient`.

To poke at it by hand instead, in Terminal 2:

```bash
ros2 action send_goal /easynav_robot/navigation easyfleet_interfaces/action/Navigation \
  "{parameters_json: '{\"goal_id\": \"dock\"}'}" --feedback
```

`Ctrl-C` or `ros2 action cancel` stops it (sends EasyNav an explicit
cancellation); sending a second goal while the first is still running
preempts it the same way `easynav_mission_node`'s Phase 3 does.

### `easynav` (real navigation) scenario

The same `navigation` capability, but with EasyNav itself configured for
*real* navigation instead of the `Dummy*` plugins: `AMCLLocalizer` +
`CostmapMapsManager` (on `src/easynav_indoor_testcase`'s `home2` map) +
`CostmapPlanner` + `RegulatedPurePursuitController` (see
`config/easynav/easynav_robot/easynav_system.costmap_rpp.params.yaml`,
copied from that package's `costmap.rpp.params.yaml`). Goals sent here
actually complete once the robot reaches the waypoint. For use against a
real robot, or a Gazebo simulation of one — **start that separately**
(this launch file doesn't bring up Gazebo, the same way
`easynav_indoor_testcase`'s own `easynav_costmap_rpp.launch.py` doesn't).

Unlike every other scenario, this one is **deliberately unnamespaced**:
`system_main`'s `sensors_node`/`localizer_node`/`controller_node` need to
see the same `scan_raw` topic and TF frames the robot/Gazebo publishes
(typically unnamespaced), and the navigation capability is launched
unnamespaced alongside it so its `GoalManagerClient`'s relative
`easynav_control` topic still resolves to `system_main`'s own
`GoalManager`. That also means it only ever runs one robot at a time.

The three configured waypoints (`dock`, `kitchen`, `charging_station`, in
`config/easynav/easynav_robot/navigation_params.yaml`) are real, reachable
points on the `home2` map, not placeholders.

Terminal 1 — with the robot/Gazebo simulation already running and
publishing `scan_raw`/odometry/TF:

```bash
ros2 launch easyfleet_example_deployments easynav_gazebo_launch.yaml
```

This also opens an RViz window (same config as
`easynav_costmap_rpp.launch.py`'s).

Terminal 2:

```bash
ros2 action send_goal /navigation easyfleet_interfaces/action/Navigation \
  "{parameters_json: '{\"goal_id\": \"dock\"}'}" --feedback
```

(No `easynav_gazebo_mission_node` yet — poke at it by hand with
`ros2 action send_goal`/`ros2 action cancel` as above, or reuse
`easynav_mission_node`'s pattern against `/navigation` instead of
`/easynav_robot/navigation`.)

### Poking at a capability by hand

Every capability's action name is namespaced by robot (`/robot_1/...`,
`/robot_2/...`, `/robot_3/...`). For example, against the `alone` scenario:

```bash
# What's running, and what does it look like?
ros2 action list -t
ros2 lifecycle get /robot_1/navigation
ros2 topic echo /capabilities --once
ros2 topic echo /capabilities_status

# Send a navigation goal
ros2 action send_goal /robot_1/navigation easyfleet_interfaces/action/Navigation \
  "{target_pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 2.0, z: 0.0}}}}" \
  --feedback

# Send a perception goal (only the 'gato' class is supported by the mock)
ros2 action send_goal /robot_1/perception easyfleet_interfaces/action/Perception \
  "{object_classes: ['gato']}" --feedback
```

Perception (and any preempted/canceled goal) runs until stopped: press
Ctrl-C on the `send_goal` command, or call `ros2 action cancel` /
`CapabilityClient::cancel()` from another terminal/process, to stop it
early.

## Running the tests

```bash
colcon test --packages-select easyfleet_core easyfleet_interfaces easyfleet_mission_manager easyfleet_easynav_navigation easyfleet_example_deployments
colcon test-result --verbose
```

Each test binary that touches ROS communication runs on its own
`ROS_DOMAIN_ID`, so the test suite is safe to run even while you have
capability nodes launched interactively (as above) — they won't cross-talk
on the shared `/capabilities` topics.

## Known limitations

### Capability discovery is a one-shot snapshot, not a live view

`easyfleet_mission_manager::discover_capabilities()` subscribes to
`/capabilities` and `/capabilities_status`, waits for a fixed window (2.5s
by default), and returns whatever it collected — a snapshot. That's good
enough for the example missions in this repo: they discover once at startup
and then run a short, scripted sequence against whatever they found.

It is **not** good enough for the architecture this repo is a stand-in for:
a control computer running a long-lived, LLM-generated Behavior Tree that
calls robot capabilities over the course of a mission that can last
minutes or hours. In that setting, treating discovery as a one-time
snapshot breaks down in a few concrete ways:

- **Capabilities can appear or disappear mid-mission.** A robot can be
  launched, crash, or be taken down for maintenance after the BT has
  already started. A snapshot taken at startup has no way to reflect that;
  the BT would keep trying to call a capability that's no longer there (or
  never learn about one that just came online).
- **"Discover once" doesn't compose with "run for a long time."** The
  moment discovery finishes, its picture of the world is already stale.
  Nothing here currently re-checks it.
- **There's no staleness policy.** A capability is "active" if at least one
  heartbeat arrived inside the discovery window, full stop — there's no
  debouncing (a single dropped heartbeat instantly reads as "gone") and no
  notion of "was active, hasn't been seen in N seconds, might still recover."
- **Turning it into a live view is nontrivial, not just "remove the
  `sleep_for`".** It would mean replacing the bounded collect-then-return
  function with a background-updated registry that the rest of the program
  reads from concurrently (thread-safety, not just a `std::map` behind a
  `std::mutex` as today, but something callers can iterate over safely
  while it's being mutated from a subscription callback). It also raises
  design questions this repo currently sidesteps entirely: what should
  happen to a goal that's already in flight when its capability's
  heartbeat stops arriving? Does the BT get notified, or does it just find
  out the hard way when `wait_for_result()` never returns? How long is
  "missing" before a capability is presumed gone rather than just late?

None of this is implemented here. It's the single biggest gap between this
mock/demo architecture and the real control-computer scenario it's meant to
validate the plumbing for.

## License

Apache-2.0 (see `easyfleet_core/LICENSE`).
