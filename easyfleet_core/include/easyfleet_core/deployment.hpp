// Copyright 2026 Intelligent Robotics Lab
//
// This file is part of the project EasyFleet
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef EASYFLEET_CORE__DEPLOYMENT_HPP_
#define EASYFLEET_CORE__DEPLOYMENT_HPP_

#include <memory>
#include <string>
#include <vector>

#include "pluginlib/class_loader.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"

#include "easyfleet_core/capability_factory.hpp"
#include "easyfleet_core/capability_node_base.hpp"

namespace easyfleet
{

/// Everything one robot's process needs: load its capabilities, bring them
/// up, run them until shutdown, tear them down.
/**
 * Replaces the longhand configure/check/activate/check/spin/deactivate/
 * cleanup/shutdown dance every `robot_node.cpp` currently repeats by hand,
 * plus the hardcoded `if (name == "perception") ... if (name == "manipulation")`
 * factory function each one writes to turn a configured name into a
 * concrete capability class -- `add_capability()` does that generically,
 * by loading a pluginlib-registered `CapabilityFactory` (see
 * `capability_factory.hpp`).
 *
 * One `Deployment` per process, one process per robot -- matching reality
 * (a real fleet's robots each have their own onboard computer, so a
 * deployment abstraction spanning several robots in one process would be
 * modeling something that doesn't exist outside a single-machine sim).
 * Each robot's identity comes from wherever the process itself is
 * launched under a ROS namespace (a launch file's `push_ros_namespace`,
 * or directly on a real robot's own computer) -- `add_capability()` needs
 * no namespace of its own to apply, capabilities just inherit the
 * process's.
 *
 * Usage, spelling capabilities out by hand:
 * \code
 * easyfleet::init(argc, argv);
 *
 * easyfleet::Deployment deployment("robot_1");
 * deployment.add_capability("perception/cats");
 * deployment.add_capability("navigation/easynav");
 *
 * deployment.start();
 * deployment.run();
 * deployment.shutdown();
 * \endcode
 *
 * Usage, letting parameters say which capabilities to load (see
 * `add_capabilities_from_parameters()`) -- the common case, and what makes
 * one `deployment.cpp`-style executable reusable, unchanged, across every
 * robot in a scenario:
 * \code
 * easyfleet::init(argc, argv);
 *
 * easyfleet::Deployment deployment;
 * deployment.add_capabilities_from_parameters("my_deployment_package");
 *
 * deployment.start();
 * deployment.run();
 * deployment.shutdown();
 * \endcode
 */
class Deployment
{
public:
  /// Robot identity inferred from this process's own ROS namespace (e.g.
  /// a process launched under `/robot_1` becomes `"robot_1"`) -- matching
  /// how that identity is already determined by however the process was
  /// launched, with no code needing to repeat it. The common case; see the
  /// class-level doc comment's second example.
  Deployment();

  /// @param name This robot's identity, e.g. "robot_1" -- used for log
  ///   messages only. Not applied as a ROS namespace: that's already
  ///   determined by however this process itself was launched (see the
  ///   class-level doc comment). Prefer the no-arg constructor unless a
  ///   name other than this process's own namespace is genuinely needed.
  explicit Deployment(std::string name);

  const std::string & name() const noexcept;

  /// Loads, constructs and starts hosting one capability. Safe to call
  /// only before `start()`.
  /**
   * @param plugin_lookup_name Pluginlib lookup name for a
   *   `CapabilityFactory` plugin -- the exact string in that plugin's
   *   `<class name="...">` entry, e.g. `"perception/cats"`. The substring
   *   before the first `/` (or the whole string, if there is no `/`)
   *   becomes the capability's actual name -- the ROS node name, the
   *   action name, and the `capability` identity field published on
   *   `/capabilities` -- so `"perception/cats"` and `"perception/dogs"`
   *   both still announce themselves as simply `"perception"`, matching
   *   whatever a mission script looks up regardless of which concrete
   *   implementation is behind it.
   * @param options Extra node options (e.g. a `capabilities_file`
   *   parameter override -- see any existing `robot_node.cpp` for how
   *   that path is computed) merged onto the capability's constructor
   *   call.
   * @throws pluginlib::PluginlibException if `plugin_lookup_name` isn't a
   *   registered `CapabilityFactory` plugin visible on this process's
   *   plugin search path (i.e. the owning package isn't a dependency, or
   *   its `plugins.xml` isn't exported/found).
   */
  void add_capability(
    const std::string & plugin_lookup_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// Every capability added so far, in `add_capability()` order.
  const std::vector<easyfleet_core::CapabilityNodeBase::SharedPtr> & capabilities() const noexcept;

  /// Reads which capabilities to host from this process's own
  /// `"capabilities"` (string list) and `"config_subdir"` parameters --
  /// the same convention `robot_node.cpp`'s own hardcoded factory function
  /// already reads by hand -- resolves each one's `capabilities_file` JSON
  /// path (`share/<package_name>/config/<config_subdir>/<name>.json`, via
  /// `ament_index_cpp`), and calls `add_capability()` for each. Safe to
  /// call only before `start()`, and safe to mix with explicit
  /// `add_capability()` calls (this just calls it in a loop).
  /**
   * Exits the process (loud, matching `start()`'s own philosophy) if
   * `"capabilities"` turns out empty -- nothing configured to host is
   * always a mistake, not a valid "do nothing" request.
   *
   * @param package_name The package whose `share/config/` directory holds
   *   each capability's `<name>.json` description -- almost always the
   *   caller's own package.
   */
  void add_capabilities_from_parameters(const std::string & package_name);

  /// Configures, then activates, every added capability, in the order
  /// they were added.
  /**
   * Fails loud, on purpose: the first capability that doesn't reach the
   * expected state logs exactly which capability failed (and at which
   * transition), then the process exits -- a half-started robot silently
   * running with one dead capability is worse than one that refuses to
   * start at all, especially for a beginner who won't think to go
   * looking for it.
   */
  void start();

  /// Adds every capability's node to one shared executor and blocks until
  /// SIGINT/SIGTERM (see `spin_until_shutdown()`), then deactivates and
  /// cleans up every capability -- in reverse order -- before returning.
  /// Requires `init()` to have been called (its non-default signal
  /// handling is what lets this shut lifecycle nodes down cleanly).
  void run();

  /// Shuts down every capability's lifecycle node, then calls
  /// `rclcpp::shutdown()` -- the ROS context teardown, distinct from
  /// `run()`'s own already-completed per-capability deactivate/cleanup.
  void shutdown();

private:
  std::string name_;
  std::vector<easyfleet_core::CapabilityNodeBase::SharedPtr> capabilities_;

  /// Lazily constructed on the first `add_capability()` call. Kept alive
  /// for this `Deployment`'s whole life -- pluginlib requires the loader
  /// that created a plugin instance to outlive that instance. One loader
  /// suffices for every plugin regardless of which package it comes from:
  /// pluginlib resolves `CapabilityFactory` plugins from the whole ament
  /// index, not just `easyfleet_core` itself.
  std::unique_ptr<pluginlib::ClassLoader<easyfleet_core::CapabilityFactory>> loader_;

  rclcpp::executors::SingleThreadedExecutor executor_;

  /// Lazily constructed the first time it's needed (by the no-arg
  /// constructor, or by `add_capabilities_from_parameters()`, whichever
  /// runs first) -- a throwaway node that exists only to read this
  /// process's namespace and/or its `"capabilities"`/`"config_subdir"`
  /// parameters. Never spun, never added to `executor_`: each capability
  /// already carries its own distinct name, so this just needs one
  /// distinct from theirs.
  rclcpp::Node::SharedPtr config_node();
  rclcpp::Node::SharedPtr config_node_;
};

}  // namespace easyfleet

#endif  // EASYFLEET_CORE__DEPLOYMENT_HPP_
