<p align="center">
  <img src="docs/easyfleet_white_back.png" alt="EasyFleet logo" width="400">
</p>

# EasyFleet

[![rolling](https://github.com/EasyNavigation/EasyFleet/actions/workflows/rolling.yaml/badge.svg?branch=rolling)](https://github.com/EasyNavigation/EasyFleet/actions/workflows/rolling.yaml)
[![codecov](https://codecov.io/github/EasyNavigation/EasyFleet/graph/badge.svg)](https://codecov.io/github/EasyNavigation/EasyFleet)

Web: [https://easyfleet.github.io](https://easyfleet.github.io/)

Doxygen documentation: [https://EasyNavigation.github.io/EasyFleet/](https://EasyNavigation.github.io/EasyFleet/)

**EasyFleet** is an open-source, ROS 2 framework for orchestrating **fleets
of multi-skill robots**, designed to be:

✅ **Capability-oriented**, not stack-specific: any robot skill — navigation, manipulation, perception, or a custom one — is exposed the same way, as a self-describing, lifecycle-managed ROS 2 action.  
🧩 **Backend-agnostic**, through a small set of base classes any integrator subclasses once — EasyFleet ships a real [EasyNavigation](https://easynavigation.github.io/)-backed navigation capability out of the box, but nothing in the architecture assumes it.  
🐝 **Fleet-aware from the ground up**: robots announce their own identity and capabilities at runtime, so a mission controller can discover, call, and monitor any number of robots without hardcoding who they are.  
🗺️ **Centrally coordinated when it helps**: a fleet-wide Navigation Manager publishes one shared map and route graph, and watches every robot's planned path to prevent collisions.  
🚀 **Lightweight and simple to deploy**, using plain ROS 2 nodes, actions, and parameter files — no external orchestration framework required.

EasyFleet is developed by the **[Intelligent Robotics Lab](https://intelligentroboticslab.gsyc.urjc.es/)** at **Universidad Rey Juan Carlos**, as the fleet-coordination layer that sits on top of a per-robot navigation stack such as **EasyNavigation**.

## 📦 Packages

| Package | Description |
|-------------|-------------|
| [**easyfleet_core**](easyfleet_core) | `ActionServerBase`/`ActionClient`, `Capability`/`CapabilityClient`, `Deployment`/`CapabilityFactory`, plus the `Navigation`/`Manipulation`/`Perception` capability base classes. |
| [**easyfleet_interfaces**](easyfleet_interfaces) | `CapabilityDescription`, `CapabilityStatus`, and the `Navigation`/`Manipulation`/`Perception` actions every capability speaks. |
| [**easyfleet_mission_manager**](easyfleet_mission_manager) | `FleetSession`, `RobotHandle`, `SimpleController`: the client-side API for discovering and driving a fleet's capabilities. |
| [**easyfleet_navigation_manager**](easyfleet_navigation_manager) | `navigation_manager_node`: a shared map/route graph for the whole fleet, live-editable from RViz, plus multi-robot path-conflict detection and pause/resume. |
| [**easyfleet_easynav_navigation**](easyfleet_easynav_navigation) | The `Navigate` BehaviorTree.CPP plugin node — EasyFleet's own EasyNav-backed navigation capability building block. |
| [**easyfleet_example_deployments**](easyfleet_example_deployments) | Four self-contained example deployments, from mocked single-robot to two robots both running real EasyNav navigation at once. |
| [**easyfleet_tools**](easyfleet_tools) | A read-only TUI/CLI for monitoring a running fleet. |

---

## 👥 Project Maintainers

| Name | Organization | GitHub | Role |
|------|---------------|--------|------|
| Francisco Martín Rico | Universidad Rey Juan Carlos | [fmrico](https://github.com/fmrico) | Project Lead |

## License

Apache-2.0 (see `easyfleet_core/LICENSE`).
