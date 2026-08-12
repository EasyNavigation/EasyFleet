# arquimea_project

A ROS 2 (Rolling) workspace exploring a **capability-oriented architecture** for
multi-robot, multi-skill fleets. It provides:

- **`arch_mockup`** — a small, reusable C++ framework for building
  self-describing, lifecycle-managed ROS 2 actions ("capabilities"), plus a
  comfortable client-side API for calling them. A `Capability<ActionServerT>`
  resolves its own robot identity from its ROS namespace at construction
  time, so the same node binary announces itself correctly whether it's
  launched as `/robot1/navigation` or `/robot2/navigation`.
- **`arch_mockup_interfaces`** — the architecture-level message contract
  (`CapabilityDescription`, `CapabilityStatus`) every capability publishes on
  `/capabilities` and `/capabilities_status`, regardless of manufacturer or
  implementation language.
- Three **example mock capabilities** built on that framework: `navigation`
  (wraps `nav2_msgs/action/NavigateToPose`), `manipulation` (wraps
  `moveit_msgs/action/ExecuteTrajectory`), and `perception` (wraps a custom
  `perception_interfaces/action/Perception`).
- **`deployments`** — assembles concrete multi-robot scenarios out of the
  capability packages above: which robot has which capabilities, under what
  namespace, with what parameters. Contains no code of its own, only launch
  files and per-robot JSON/parameter configuration.
- **`control_center`** — mission scripts that discover whatever capabilities
  are currently running, print their self-description, and exercise them
  both sequentially and in parallel, showing live feedback as it happens.
  One mission script per deployment scenario (see [Running](#running)).

Everything under `capabilities/` is a **mock**: it doesn't drive a real robot
or run a real detector/manipulator. The goal of this repo is to validate the
architecture and plumbing of a multi-robot, multi-capability system
(discovery, namespacing, lifecycle, preemption, cancellation, feedback)
before wiring in real navigation/perception/manipulation stacks. The
intended real-world shape is a control computer running an LLM-generated
Behavior Tree (via BehaviorTreeCPP) whose action nodes are
`arch_mockup::CapabilityClient`s; `control_center` is a first approximation
of that control computer.

## Repository layout

```
arquimea_project/
├── architecture/                    Core, reusable framework
│   ├── arch_mockup/                 ActionServerBase, ActionClient, Capability, CapabilityClient
│   ├── arch_mockup_interfaces/      CapabilityDescription / CapabilityStatus (the architecture's message contract)
│   └── control/
│       └── control_center/          Mission scripts (one per deployment scenario)
├── capabilities/                    Concrete example capabilities built on arch_mockup
│   ├── navigation/
│   │   ├── nav2_common/             Vendored: not yet released as a binary package for Rolling
│   │   ├── nav2_msgs/               Vendored: not yet released as a binary package for Rolling
│   │   └── navigation_capability/   Mock nav2_msgs/NavigateToPose action
│   ├── manipulation/
│   │   └── manipulation_capability/ Mock moveit_msgs/ExecuteTrajectory action
│   └── perception/
│       ├── perception_interfaces/   Defines perception_interfaces/action/Perception
│       └── perception_capability/   Mock perception_interfaces/Perception action
└── deployments/                     Launch files + per-robot JSON/parameter config for full scenarios
    ├── launch/{alone,collaboration}/
    └── config/{alone,collaboration}/
```

Capability packages ship **only** a library and a node executable — no
default launch file or JSON config of their own anymore. All of that (which
robot, which namespace, which parameters, which capabilities-description
JSON) lives in `deployments`, so a given capability's actual identity is
entirely a property of how it's deployed, not of the package itself.

## What's a "capability"?

A capability is a `rclcpp_lifecycle::LifecycleNode` (`arch_mockup::Capability<ActionServerT>`)
that wraps exactly one ROS 2 action and, once activated:

- Publishes a `CapabilityDescription` (`arch_mockup_interfaces`) on the
  **reliable, transient-local** `/capabilities` topic, so anyone subscribing
  late still gets it. It carries `robot`, `capability`, `action_name` (all
  resolved from the node's actual ROS namespace at runtime) and the raw JSON
  description (what it does, requirements, effects, parameters) read from
  its `capabilities_file` parameter.
- Publishes a 1 Hz `CapabilityStatus` heartbeat (same `robot`/`capability`/
  `action_name` identity, plus a `busy` flag) on `/capabilities_status` for
  as long as it stays active — this is how `control_center` knows a
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
own ROS namespace, the same node binary (e.g. `navigation_capability_node`)
can be launched under any namespace and will announce itself correctly —
`deployments` uses this to assemble two example scenarios:

| Scenario | Robot | Capabilities | Launch file |
|---|---|---|---|
| `alone` | `robot_1` | navigation, manipulation, perception | `deployments/launch/alone/alone_launch.yaml` |
| `collaboration` | `robot_1` | navigation, perception | `deployments/launch/collaboration/collaboration_launch.yaml` |
| `collaboration` | `robot_2` | navigation, perception | (same) |
| `collaboration` | `robot_3` | navigation, manipulation | (same) |

Each robot has its own per-robot launch file (e.g.
`deployments/launch/collaboration/robot_3_launch.yaml`) that can also be run
standalone, and its own JSON/parameter files under
`deployments/config/<scenario>/<robot>/`. Namespacing keeps two robots that
share a capability type (e.g. `navigation` on both `robot_1` and `robot_3`)
fully distinct: their resolved `action_name`s (`/robot_1/navigation` vs.
`/robot_3/navigation`) never collide, and each can be discovered and called
independently.

## Packages

| Package | Type | What it is |
|---|---|---|
| `arch_mockup` | C++ library | `ActionServerBase<ActionT>`, `ActionClient<ActionT>`, `Capability<ActionServerT>`, `CapabilityClient<ActionT>` |
| `arch_mockup_interfaces` | Interface package | `CapabilityDescription`, `CapabilityStatus` — the message contract every capability must respect |
| `control_center` | C++ executables | `control_center_alone_node`, `control_center_collaboration_node` — mission scripts, one per deployment scenario |
| `nav2_common`, `nav2_msgs` | Vendored | Upstream Nav2 packages, included because they aren't released for Rolling yet |
| `navigation_capability` | C++ library + executable | Mock `navigation` capability (`nav2_msgs/NavigateToPose`) |
| `manipulation_capability` | C++ library + executable | Mock `manipulation` capability (`moveit_msgs/ExecuteTrajectory`) |
| `perception_interfaces` | Interface package | Defines the `Perception` action |
| `perception_capability` | C++ library + executable | Mock `perception` capability (`Perception`) |
| `deployments` | Launch + config only | Assembles the `alone` and `collaboration` scenarios out of the packages above |

## Prerequisites

- Ubuntu with **ROS 2 Rolling** installed.
- `colcon` and the usual ROS 2 build tooling.
- `nlohmann-json3-dev` (used by `control_center` to parse capability JSON
  descriptions) and `moveit_msgs` — both installed automatically by
  `rosdep` below.

## Building

```bash
mkdir -p ~/arquimea_ws/src
cd ~/arquimea_ws/src
# place (or clone) this repository here as `arquimea_project`

cd ~/arquimea_ws
source /opt/ros/rolling/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Running

Each capability node **activates itself on startup** — no external lifecycle
manager or `ros2 lifecycle set` calls are needed. Bring up a scenario from
`deployments`, then run the matching `control_center` mission script.

### `alone` scenario

A single robot, `robot_1`, carrying all three capabilities, all namespaced
under `/robot_1`.

Terminal 1:

```bash
ros2 launch deployments alone_launch.yaml
```

Terminal 2, once it's up:

```bash
ros2 run control_center control_center_alone_node
```

`control_center_alone_node` will, in order, print what it's doing as it
happens:

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
ros2 launch deployments collaboration_launch.yaml
```

Terminal 2, once it's up:

```bash
ros2 run control_center control_center_collaboration_node
```

`control_center_collaboration_node` runs a two-phase mission:

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
ros2 action send_goal /robot_1/navigation nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 2.0, z: 0.0}}}}" \
  --feedback

# Send a perception goal (only the 'gato' class is supported by the mock)
ros2 action send_goal /robot_1/perception perception_interfaces/action/Perception \
  "{objects: ['gato']}" --feedback
```

Perception (and any preempted/canceled goal) runs until stopped: press
Ctrl-C on the `send_goal` command, or call `ros2 action cancel` /
`CapabilityClient::cancel()` from another terminal/process, to stop it
early.

## Running the tests

```bash
colcon test --packages-select arch_mockup arch_mockup_interfaces navigation_capability manipulation_capability perception_capability perception_interfaces control_center deployments
colcon test-result --verbose
```

Each test binary that touches ROS communication runs on its own
`ROS_DOMAIN_ID`, so the test suite is safe to run even while you have
capability nodes launched interactively (as above) — they won't cross-talk
on the shared `/capabilities` topics.

## Known limitations

### Capability discovery is a one-shot snapshot, not a live view

`control_center::discover_capabilities()` subscribes to `/capabilities` and
`/capabilities_status`, waits for a fixed window (2.5s by default), and
returns whatever it collected — a snapshot. That's good enough for the
mission scripts in this repo: they discover once at startup and then run a
short, scripted sequence against whatever they found.

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

Apache-2.0 (see `architecture/arch_mockup/LICENSE`).
