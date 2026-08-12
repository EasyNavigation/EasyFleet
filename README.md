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
- **`easyfleet_example_deployments`** — the reference example: a "fake"/mock
  implementation of each capability (`navigation`, `manipulation`,
  `perception`, each a subclass of the matching `easyfleet_core`
  `*ActionServerBase`), the `alone`/`collaboration` example mission scripts
  built on `easyfleet_mission_manager`, and the launch files + per-robot
  JSON/parameter configuration that assemble them into concrete multi-robot
  scenarios. A real integrator would add their own backend (Nav2, EasyNav, a
  vendor SDK, ...) as a sibling package following the same pattern — see
  [Extending EasyFleet](#extending-easyfleet) below.

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
└── easyfleet_example_deployments/   Fake capability implementations, example missions, launch/config
    ├── include|src/easyfleet_example_deployments/{navigation,manipulation,perception}_fake_capability.hpp|cpp
    ├── src/{navigation,manipulation,perception}_fake_capability_main.cpp
    ├── src/main_{alone,collaboration}.cpp
    ├── launch/{alone,collaboration}/
    └── config/{alone,collaboration}/
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

`easyfleet_example_deployments`'s `navigation_fake_capability.hpp`/`.cpp`
(the mock) is the reference implementation of this exact pattern — a
`navigation_easynav_capability` or `navigation_nav2_capability` package
would follow it structurally, just replacing the mock's simulated
progress in `on_execute()` with real calls into that backend's stack.
Everything else (announcing on `/capabilities`, the heartbeat, preemption,
discovery, `CapabilityClient`) is handled by `easyfleet_core` and needs no
changes.

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
`easyfleet_example_deployments` uses this to assemble two example scenarios:

| Scenario | Robot | Capabilities | Launch file |
|---|---|---|---|
| `alone` | `robot_1` | navigation, manipulation, perception | `easyfleet_example_deployments/launch/alone/alone_launch.yaml` |
| `collaboration` | `robot_1` | navigation, perception | `easyfleet_example_deployments/launch/collaboration/collaboration_launch.yaml` |
| `collaboration` | `robot_2` | navigation, perception | (same) |
| `collaboration` | `robot_3` | navigation, manipulation | (same) |

Each robot has its own per-robot launch file (e.g.
`easyfleet_example_deployments/launch/collaboration/robot_3_launch.yaml`) that
can also be run standalone, and its own JSON/parameter files under
`easyfleet_example_deployments/config/<scenario>/<robot>/`. Namespacing keeps
two robots that share a capability type (e.g. `navigation` on both
`robot_1` and `robot_3`) fully distinct: their resolved `action_name`s
(`/robot_1/navigation` vs. `/robot_3/navigation`) never collide, and each
can be discovered and called independently.

## Packages

| Package | Type | What it is |
|---|---|---|
| `easyfleet_core` | C++ library | `ActionServerBase<ActionT>`, `ActionClient<ActionT>`, `Capability<ActionServerT>`, `CapabilityClient<ActionT>`, plus `Navigation`/`Manipulation`/`PerceptionActionServerBase` |
| `easyfleet_interfaces` | Interface package | `CapabilityDescription`, `CapabilityStatus`, and the `Navigation`/`Manipulation`/`Perception` actions |
| `easyfleet_mission_manager` | C++ library | Capability discovery/print helpers and the generic `run_capability<ActionT>()` helper — no executables of its own |
| `easyfleet_example_deployments` | C++ library + 5 executables + launch/config | The 3 fake capabilities (`navigation_fake_capability_node`, `manipulation_fake_capability_node`, `perception_fake_capability_node`), the 2 example missions (`alone_mission_node`, `collaboration_mission_node`), and the `alone`/`collaboration` scenario assembly |

Note: `easyfleet_example_deployments` intentionally keeps its own descriptive
name rather than an `easyfleet_*` prefix, distinguishing it as an
example/deployment package rather than core EasyFleet infrastructure.

## Prerequisites

- Ubuntu with **ROS 2 Rolling** installed (this workspace is pixi-managed;
  enter the environment with e.g. `pixi-set-ros rolling` before building).
- `colcon` and the usual ROS 2 build tooling.
- `nlohmann-json3-dev` (used by `easyfleet_mission_manager` to parse
  capability JSON descriptions) — installed automatically by `rosdep` below.

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
colcon test --packages-select easyfleet_core easyfleet_interfaces easyfleet_mission_manager easyfleet_example_deployments
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
