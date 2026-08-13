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
- **`easyfleet_easynav_navigation`** — the resources EasyFleet itself
  provides for building a capability backed by an actual
  [EasyNav](../EasyNavigation) navigation stack (via
  `easynav::GoalManagerClient`). For now, just the **`Navigate`**
  BehaviorTree.CPP node: resolves a named waypoint and drives EasyNav to
  it. Built as a BT.CPP plugin (`BT_REGISTER_NODES`, loaded by path at
  runtime) rather than a library a capability links against at compile
  time, so any manufacturer/integrator can build their own EasyNav-backed
  capability — their own tree, bookend nodes, action-server class — around
  it, without a compile-time dependency on this package beyond this one
  node. See [Extending EasyFleet](#extending-easyfleet) below.
- **`easyfleet_example_deployments/`** — not a package itself, but a
  container directory for three example deployment packages, each a
  self-contained scenario (own launch/config/mission script):
  - **`easyfleet_fake_alone_deployment`** — a single robot, `robot_1`,
    carrying "fake"/mock implementations of all three capabilities
    (`navigation`, `manipulation`, `perception`, each a subclass of the
    matching `easyfleet_core` `*ActionServerBase`), plus the `alone`
    example mission script built on `easyfleet_mission_manager`.
  - **`easyfleet_fake_collaboration_deployment`** — three robots sharing
    the same fake capability implementations, collaborating on a mission,
    plus the `collaboration` example mission script.
  - **`easyfleet_easynav_deployment`** — the `easynav` scenario: this
    package's own `easyfleet_core::NavigationActionServerBase` subclass,
    driving EasyNav through a BehaviorTree.CPP tree (`StartOff` →
    `Navigate` → `Finish`) — the real `Navigate` node reused from
    `easyfleet_easynav_navigation` as a plugin, `StartOff`/`Finish` (fake
    bookends) supplied by this package the same way. Runs against a real
    EasyNav navigation stack (costmap localizer/maps-manager/planner +
    regulated pure-pursuit controller) for use with a real robot or a
    Gazebo simulation of one, unnamespaced by default or namespaced via a
    launch argument.

`easyfleet_fake_alone_deployment`/`easyfleet_fake_collaboration_deployment`
are, capability-wise, entirely **mocks**: they don't drive a real robot or
run a real detector/manipulator. The goal of this repo is to validate the
architecture and plumbing of a multi-robot, multi-capability system
(discovery, namespacing, lifecycle, preemption, cancellation, feedback)
before wiring in real navigation/perception/manipulation stacks. The
intended real-world shape is a control computer running an LLM-generated
Behavior Tree (via BehaviorTreeCPP) whose action nodes are
`easyfleet_core::CapabilityClient`s; the example mission scripts here are a
first approximation of that control computer.

## Repository layout

```
EasyFleet/
├── easyfleet_core/                  ActionServerBase, ActionClient, Capability, CapabilityClient,
│                                     Navigation/Manipulation/PerceptionActionServerBase
├── easyfleet_interfaces/            CapabilityDescription/CapabilityStatus + Navigation/Manipulation/Perception actions
├── easyfleet_mission_manager/       Capability discovery + mission-scripting helper library (no executables)
├── easyfleet_easynav_navigation/    Navigate BT.CPP node (plugin) -- the one resource EasyFleet provides for EasyNav
│   └── include|src/easyfleet_easynav_navigation/bt_nodes/navigate.hpp|cpp  -> libeasynav_navigate_bt_node.so
└── easyfleet_example_deployments/   Not a package -- a container for 3 example deployment packages
    ├── easyfleet_fake_alone_deployment/            "alone" scenario: robot_1, all 3 fake capabilities
    │   ├── include|src/easyfleet_fake_alone_deployment/{navigation,manipulation,perception}_fake_capability.hpp|cpp
    │   ├── src/robot_node.cpp                       -> robot_node (the robot: hosts robot_1's capabilities)
    │   ├── src/main_alone.cpp                       -> alone_mission_node (mission control, launched separately)
    │   ├── launch/{alone_launch.yaml, robot_1_launch.yaml}
    │   └── config/robot_1/                          per-robot JSON/parameter files, incl. robot_params.yaml
    ├── easyfleet_fake_collaboration_deployment/    "collaboration" scenario: robot_1/2/3, same 3 fake capabilities
    │   ├── (same include|src|launch shape as the alone package, its own copy)
    │   ├── src/robot_node.cpp                       -> robot_node (reused unchanged across all 3 robots)
    │   ├── src/main_collaboration.cpp               -> collaboration_mission_node
    │   └── config/{robot_1,robot_2,robot_3}/
    └── easyfleet_easynav_deployment/                "easynav" scenario: real navigation capability
        ├── include|src/easyfleet_easynav_deployment/easynav_navigation_capability.hpp|cpp -> easynav_navigation_capability_node
        ├── include|src/easyfleet_easynav_deployment/bt_nodes/{start_off,finish}.hpp|cpp    -> libstart_off_bt_node.so / libfinish_bt_node.so
        ├── behavior_trees/navigate.xml                 references StartOff/Navigate/Finish by name
        ├── src/main_easynav.cpp                        -> easynav_mission_node (mission control)
        ├── launch/{easynav_gazebo_launch.yaml -> easynav_robot_gazebo_launch.yaml}
        └── config/easynav_robot/
```

Every scenario's top-level launch file (`alone_launch.yaml`, `collaboration_launch.yaml`,
`easynav_gazebo_launch.yaml`) launches **both** halves
of the deployment: the robot (one process per robot, hosting its
capabilities) and mission control (the example mission script), the latter
delayed a few seconds so the robot has finished activating first — see
[Robot vs. mission control](#robot-vs-mission-control) below.

`easyfleet_fake_alone_deployment` and `easyfleet_fake_collaboration_deployment`
each carry their own full copy of the three fake capability implementations
(library + executables + tests) — a deliberate choice to keep each
deployment package fully self-contained rather than introduce a 4th shared
package or an asymmetric dependency between the two.

## Extending EasyFleet

Any integrator can add their own backend for a capability domain by
subclassing the matching `easyfleet_core` base class in their own package,
without touching `easyfleet_core` or any of the example deployment
packages:

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
- `easyfleet_fake_alone_deployment`'s (or
  `easyfleet_fake_collaboration_deployment`'s — identical copies)
  `navigation_fake_capability.hpp`/`.cpp` — a mock, simulates progress on a
  timer, no external dependency.
- `easyfleet_easynav_deployment`'s `easynav_navigation_capability.hpp`/`.cpp`
  — a real backend, driving an actual EasyNav navigation stack (see
  [below](#the-easynav-backed-navigation-capability)).

Either is a good template for a new `navigation_<your_backend>_capability`
(or `manipulation_*`/`perception_*`) package. Everything else (announcing
on `/capabilities`, the heartbeat, preemption bookkeeping, discovery,
`CapabilityClient`) is handled by `easyfleet_core` and needs no changes.

### The EasyNav-backed navigation capability

**How the two packages split responsibility.** `easynav::GoalManagerClient`
only accepts raw poses — it has no concept of a named waypoint. Resolving
that, and everything else about turning a ROS goal into a running
BehaviorTree.CPP tree, is a manufacturer/integrator choice, not something
EasyFleet prescribes — so `easyfleet_easynav_deployment` owns the actual
capability class (`EasynavNavigationActionServer`), reusing only
`easyfleet_easynav_navigation`'s **`Navigate`** node, the one resource
EasyFleet itself provides for talking to EasyNav.

Every BT node in this tree — including `Navigate` — is built as a
[BT.CPP plugin](https://www.behaviortree.dev/docs/guides/plugins)
(`BT_REGISTER_NODES`, a `SHARED` library with `BT_PLUGIN_EXPORT`), loaded
by path at runtime via `BT::BehaviorTreeFactory::registerFromPlugin()`, not
linked at compile time. This is what lets `easyfleet_easynav_deployment`
reuse `easyfleet_easynav_navigation`'s `Navigate` node with zero
compile-time coupling: the capability class never `#include`s it, it just
loads whatever `.so` paths the `bt_plugins` parameter lists (see
`launch/easynav_robot_gazebo_launch.yaml`) — the same mechanism a *different*
deployment package would use to reuse `Navigate` with its *own* bookend
nodes and tree.

- `navigation.waypoint_ids` + `navigation.waypoints.<id>.{frame_id,x,y,yaw}`
  parameters (declared by `EasynavNavigationActionServer`) define a fixed
  set of named waypoints, and an incoming `Navigation` goal selects one via
  `parameters_json: {"goal_id": "<id>"}` (`target_pose`/`waypoints` on the
  goal itself go unused by this backend — see [Interfaces](#interfaces)).
- Each goal ticks a `behaviortree_cpp` v4 tree (`behavior_tree_xml`,
  `easyfleet_easynav_deployment/behavior_trees/navigate.xml`) with three
  `StatefulActionNode`s: **`StartOff`** / **`Finish`** (this package's own
  fake bookends — no ports, just take 2s and print a message) and
  **`Navigate`** (from `easyfleet_easynav_navigation`) — reads the target
  waypoint id from its `goal_id` input port and resolves it against a
  registry read off the *blackboard* (not constructor arguments — a
  plugin's node type is only ever instantiated as `Navigate(name, config)`
  by the factory), then calls `GoalManagerClient::send_goal()`.

One `GoalManagerClient` is kept alive for the capability's whole lifetime
(not recreated per ROS goal) — so when a new goal preempts an in-flight
one, its fresh `Navigate` node redirects EasyNav to the new target via that
same client, which EasyNav treats as a preemption of the goal it already
has from that client id, no special-cased "resume in place" logic needed.
A genuine cancellation (or capability shutdown), on the other hand, calls
`GoalManagerClient::cancel()` before settling the goal — the robot actually
stops, rather than continuing to navigate toward an abandoned goal.

The `easynav` scenario, in `easyfleet_easynav_deployment`, runs this
capability against a real EasyNav navigation stack (`easynav_system
system_main`) — see [Multi-robot deployments](#multi-robot-deployments).

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

## Robot vs. mission control

Every deployment scenario is deliberately split into two kinds of process
that never link against each other's implementation, only ever talk over
ROS actions/topics:

- **The robot** — one process per robot, hosting that robot's capabilities.
  For `easyfleet_fake_alone_deployment`/`easyfleet_fake_collaboration_deployment`
  this is `robot_node`: it reads a `capabilities` parameter (e.g.
  `["navigation", "manipulation", "perception"]`, see
  `config/<robot>/robot_params.yaml`) and constructs exactly that subset,
  each its own `easyfleet_core::Capability` lifecycle node, sharing one
  executor (safe: each capability's actual goal execution runs on its own
  worker thread inside `easyfleet_core::ActionServerBase`, so one capability
  can't starve another sharing the same process). The same `robot_node`
  binary is reused unchanged across every robot in a scenario — only its
  parameters differ. For `easyfleet_easynav_deployment`, "the robot" is
  `easynav_system system_main` + `easynav_navigation_capability_node`
  together (EasyNav itself is a separate package's process, so it can't be
  merged into one binary the way the fake capabilities are).
- **Mission control** — a separate process (`alone_mission_node`,
  `collaboration_mission_node`, `easynav_mission_node`) that discovers the
  robot's capabilities via `/capabilities` and drives them through
  `CapabilityClient`, exactly the way any other operator/control-computer
  program would. It has no compile-time dependency on any capability
  implementation, only on `easyfleet_interfaces`' action types.

Each scenario's top-level launch file starts both: the robot immediately,
then mission control after a few seconds' delay (a `timer` action) so the
robot has finished activating — and published its `/capabilities`
announcement — before mission control's discovery window opens (see
[Known limitations](#capability-discovery-is-a-one-shot-snapshot-not-a-live-view)
for why that window is fixed and short). Either half can also be run
standalone in its own terminal (see [Running](#running) below), which is
how you'd poke at a robot by hand without a scripted mission at all.

## Multi-robot deployments

Because a capability resolves its `robot`/`action_name` identity from its
own ROS namespace, the same `robot_node` binary can be launched under any
namespace (with different `capabilities`/`config_subdir` parameters) and
will announce itself correctly —
each of the three packages under `easyfleet_example_deployments/` uses this
to assemble one example scenario, self-contained under its own `launch/` +
`config/`:

| Scenario | Package | Robot | Capabilities | Launch file |
|---|---|---|---|---|
| `alone` | `easyfleet_fake_alone_deployment` | `robot_1` | navigation, manipulation, perception (mock) | `launch/alone_launch.yaml` |
| `collaboration` | `easyfleet_fake_collaboration_deployment` | `robot_1` | navigation, perception (mock) | `launch/collaboration_launch.yaml` |
| `collaboration` | `easyfleet_fake_collaboration_deployment` | `robot_2` | navigation, perception (mock) | (same) |
| `collaboration` | `easyfleet_fake_collaboration_deployment` | `robot_3` | navigation, manipulation (mock) | (same) |
| `easynav` | `easyfleet_easynav_deployment` | *(unnamespaced by default, via `robot_namespace`)* | navigation (real EasyNav backend, real costmap/localizer/planner/controller) + `easynav_system system_main` + `rviz2` | `launch/easynav_gazebo_launch.yaml` |

Each robot has its own per-robot launch file (e.g.
`easyfleet_fake_collaboration_deployment/launch/robot_3_launch.yaml`) that
can also be run standalone, and its own JSON/parameter files under that
package's `config/<robot>/`. Namespacing keeps two robots that share a
capability type (e.g. `navigation` on both `robot_1` and `robot_3`) fully
distinct: their resolved `action_name`s (`/robot_1/navigation` vs.
`/robot_3/navigation`) never collide, and each can be discovered and called
independently.

The `easynav` scenario takes this a step further: rather than hardcoding a
robot id, `easynav_robot_gazebo_launch.yaml` exposes it as a
`robot_namespace` launch argument (`push_ros_namespace`, plus a matching
`tf_prefix` override on `system_main` and namespace-prefixed waypoint
`frame_id`s — see [below](#easynav-scenario)), forwarded through the
top-level `easynav_gazebo_launch.yaml`. `system_main`'s own
`config/easynav_robot/easynav_system.costmap_rpp.params.yaml` uses the
`/**/<node_name>` wildcard key, so it loads correctly whichever namespace
(or none) `system_main` ends up under — the same pattern
`config/easynav_robot/navigation_params.yaml` already used. This is one
robot at a time, not true multi-robot yet (only one `GoalManagerClient` per
capability, one waypoint set) — but it's what lets a namespaced `easynav`
robot coexist on the same `ros2` graph as, say, an `alone`/`collaboration`
`robot_1`, and is the namespace-per-robot convention full multi-robot
support (see [easynav_playground_kobuki](../easynav_playground_kobuki))
will build on.

## Packages

| Package | Type | What it is |
|---|---|---|
| `easyfleet_core` | C++ library | `ActionServerBase<ActionT>`, `ActionClient<ActionT>`, `Capability<ActionServerT>`, `CapabilityClient<ActionT>`, plus `Navigation`/`Manipulation`/`PerceptionActionServerBase` |
| `easyfleet_interfaces` | Interface package | `CapabilityDescription`, `CapabilityStatus`, and the `Navigation`/`Manipulation`/`Perception` actions |
| `easyfleet_mission_manager` | C++ library | Capability discovery/print helpers and the generic `run_capability<ActionT>()` helper — no executables of its own |
| `easyfleet_easynav_navigation` | BT.CPP plugin library | The `Navigate` BT node (`libeasynav_navigate_bt_node.so`), loaded by path — no executable of its own |
| `easyfleet_fake_alone_deployment` | C++ library + executables + launch/config | The `alone` scenario: all 3 fake capabilities hosted by `robot_node` on `robot_1`, + `alone_mission_node` (mission control) |
| `easyfleet_fake_collaboration_deployment` | C++ library + executables + launch/config | The `collaboration` scenario: its own copy of the same 3 fake capabilities, `robot_node` reused across `robot_1`/`robot_2`/`robot_3`, + `collaboration_mission_node` |
| `easyfleet_easynav_deployment` | C++ library + executables + launch/config | The `easynav` scenario: `easynav_navigation_capability_node` (this package's own capability class, reusing `easyfleet_easynav_navigation`'s `Navigate` plugin) + `easynav_mission_node`, plus `libstart_off_bt_node.so`/`libfinish_bt_node.so` and the launch/config assembly for real navigation against a real robot or Gazebo |

All three live under `easyfleet_example_deployments/`, a plain container
directory (no `package.xml` of its own) rather than a package.

## Prerequisites

- Ubuntu with **ROS 2 Rolling** installed (this workspace is pixi-managed;
  enter the environment with e.g. `pixi-set-ros rolling` before building).
- `colcon` and the usual ROS 2 build tooling.
- `nlohmann-json3-dev` (used by `easyfleet_mission_manager` and
  `easyfleet_easynav_deployment` to parse JSON) and `behaviortree_cpp`
  (used by `easyfleet_easynav_navigation`/`easyfleet_easynav_deployment`)
  — installed automatically by `rosdep`/`pixi` below.
- [EasyNavigation](../EasyNavigation) (specifically `easynav_system`) built
  in the same workspace — required by `easyfleet_easynav_deployment` and
  the `easynav` example scenario.
- For the `easynav` scenario specifically:
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
manager or `ros2 lifecycle set` calls are needed. Each scenario's top-level
launch file brings up both the robot and mission control together (see
[Robot vs. mission control](#robot-vs-mission-control)) — one command, one
terminal.

### `alone` scenario

A single robot, `robot_1`, carrying all three capabilities, all namespaced
under `/robot_1`, hosted together by `robot_node`.

```bash
ros2 launch easyfleet_fake_alone_deployment alone_launch.yaml
```

(To run just the robot, without mission control automatically following —
e.g. to poke at it by hand instead — use
`ros2 launch easyfleet_fake_alone_deployment robot_1_launch.yaml`, or
`ros2 run easyfleet_fake_alone_deployment alone_mission_node` separately
once it's up.)

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
`robot_3` (navigation + manipulation) — each hosted by its own `robot_node`
process (same binary, different `capabilities`/`config_subdir` parameters).

```bash
ros2 launch easyfleet_fake_collaboration_deployment collaboration_launch.yaml
```

(To run just the robots, without mission control automatically following,
or a single robot standalone — e.g.
`ros2 launch easyfleet_fake_collaboration_deployment robot_3_launch.yaml`
— see [Robot vs. mission control](#robot-vs-mission-control).)

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

A single robot whose `navigation` capability is the real EasyNav-backed
one (`easyfleet_easynav_deployment`), not the mock: `AMCLLocalizer` +
`CostmapMapsManager` (on `src/easynav_indoor_testcase`'s `home2` map) +
`CostmapPlanner` + `RegulatedPurePursuitController` (see
`config/easynav_robot/easynav_system.costmap_rpp.params.yaml`,
copied from that package's `costmap.rpp.params.yaml`). Goals sent here
actually complete once the robot reaches the waypoint. For use against a
real robot, or a Gazebo simulation of one — **start that separately**
(this launch file doesn't bring up Gazebo, the same way
`easynav_indoor_testcase`'s own `easynav_costmap_rpp.launch.py` doesn't).

Defaults to **unnamespaced** (`robot_namespace:=""`): `system_main`'s
`sensors_node`/`localizer_node`/
`controller_node` need to see the same `scan_raw` topic and TF frames the
robot/Gazebo publishes, which are themselves unnamespaced by default for a
single real robot, and the navigation capability is launched in the same
namespace alongside it so its `GoalManagerClient`'s relative
`easynav_control` topic still resolves to `system_main`'s own
`GoalManager`. Pass `robot_namespace:=robot_1` to run it namespaced instead
— e.g. against [easynav_playground_kobuki](../easynav_playground_kobuki)'s
`playground_monorobot_kobuki.launch.py`, which spawns its Kobuki under that
same namespace (`config/monorobot_config.yaml`). Gazebo's namespaced
*topics* (`scan_raw`, `odom`, ...) resolve correctly either way since
`push_ros_namespace` handles that automatically — but `kobuki_description`
also unconditionally prefixes every TF *frame name* with the robot's
namespace (`robot_1/base_link`, not `base_link`), independently of whether
topics are namespaced, so `system_main`'s own `tf_prefix` parameter is set
to the same `robot_namespace` value to match (it prefixes every `TFInfo`
frame, `map_frame` included — see `easynav_common/RTTFBuffer.hpp`); empty
by default, so the unnamespaced case is unaffected. Either way this only
ever runs one robot at a time (one `GoalManagerClient`, one waypoint set)
— true multi-robot support is future work (see
[Multi-robot deployments](#multi-robot-deployments)).

The three configured waypoints (`dock`, `kitchen`, `charging_station`, in
`config/easynav_robot/navigation_params.yaml`) are real, reachable
points on the `home2` map, not placeholders. Their `frame_id` (`"map"` in
the YAML) is namespace-prefixed the same way `tf_prefix` prefixes
`system_main`'s own `map_frame` (`EasynavNavigationActionServer`'s
constructor, using the capability's own resolved namespace) — without
this, `easynav_costmap_planner` silently rejects every goal whose frame
doesn't match `tf_info.map_frame` once namespaced, which is exactly the
failure mode if this ever gets out of sync (a `"Goals frame is not 'map':
..."` WARN from `system_main`, easy to miss among the rest of the launch
output).

With the robot/Gazebo simulation already running and publishing
`scan_raw`/odometry/TF:

```bash
ros2 launch easyfleet_easynav_deployment easynav_gazebo_launch.yaml
# or, namespaced to match a Gazebo robot spawned under "robot_1":
# ... robot_namespace:=robot_1
```

This launches the robot (`system_main` + the navigation capability),
opens an RViz window (same config as `easynav_costmap_rpp.launch.py`'s),
and, a few seconds later, `easynav_mission_node`. It doesn't hardcode a
robot name — it finds whichever `navigation` capability is currently
active — then, in order, prints what it's doing as it happens:

1. **Discover** the active `navigation` capability and print its full
   description (from its JSON on `/capabilities`).
2. Send it to the `dock` waypoint (one of the three configured in
   `config/easynav_robot/navigation_params.yaml` — `dock`, `kitchen`,
   `charging_station`), printing live feedback until it completes (or
   `kRunTimeout` elapses as a safety net).
3. Send a goal to `dock` again, then, 3 seconds later, a second goal to
   `kitchen` while the first is still running: the first goal is aborted at
   the ROS level (preempted), but EasyNav itself is redirected to `kitchen`
   without stopping first — demonstrating EasyNav-level preemption via the
   capability's persistent `GoalManagerClient`.

RViz is launched inside the same `robot_namespace` group, with every
absolute topic that `easynav_costmap.rviz` references remapped to its
relative equivalent (so it resolves under `robot_1` too when namespaced)
and its Fixed Frame overridden to match — no manual topic editing needed
either way. This also means RViz's **2D Goal Pose** arrow tool works
directly: it publishes to (the now-namespaced) `goal_pose`, the same
relative topic `easynav_system`'s `GoalManager` subscribes to, so dragging
a goal in RViz drives the robot exactly like `send_goal` below, no code
involved.

To poke at it by hand instead, from another terminal (prefix the action
name with `/robot_1` too if you launched it namespaced):

```bash
ros2 action send_goal /navigation easyfleet_interfaces/action/Navigation \
  "{parameters_json: '{\"goal_id\": \"dock\"}'}" --feedback
```

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
colcon test --packages-select easyfleet_core easyfleet_interfaces easyfleet_mission_manager easyfleet_easynav_navigation easyfleet_fake_alone_deployment easyfleet_fake_collaboration_deployment easyfleet_easynav_deployment
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
