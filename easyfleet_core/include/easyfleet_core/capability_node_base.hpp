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

#ifndef EASYFLEET_CORE__CAPABILITY_NODE_BASE_HPP_
#define EASYFLEET_CORE__CAPABILITY_NODE_BASE_HPP_

#include <memory>
#include <string>

#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

namespace easyfleet_core
{

/// @brief Non-template handle to an already-constructed `Capability<ActionServerT>`
/// instance, exposing exactly the surface `Robot`/`Deployment` need to drive
/// it -- lifecycle transitions, its node (to add to an executor), and its
/// identity -- without needing to know `ActionServerT`.
///
/// This exists purely so `Robot::add_capability()` can hold a
/// runtime-selected, pluginlib-loaded capability (see `CapabilityFactory`)
/// in a single homogeneous container: `Capability<ActionServerT>` itself
/// stays exactly as it is today (still template, still constructed eagerly
/// with `(capability_name, options)`, still usable standalone the way every
/// existing deployment already uses it) -- it just additionally implements
/// this small interface.
class CapabilityNodeBase
{
public:
  /// @brief Shared pointer to a `CapabilityNodeBase`.
  using SharedPtr = std::shared_ptr<CapabilityNodeBase>;
  /// @brief Lifecycle transition result type shared with `rclcpp_lifecycle`.
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  virtual ~CapabilityNodeBase() = default;

  // Named *_node() rather than plain configure()/activate()/etc.: those
  // names are already taken, with different signatures, by
  // rclcpp_lifecycle::LifecycleNode itself (which Capability<ActionServerT>
  // also derives from). Reusing them here would silently *hide* (not
  // override) LifecycleNode's own overloads for any code holding a
  // Capability<T>* directly -- including the one-arg configure(CallbackReturn&)
  // overload existing code (test_capability.cpp) already calls -- breaking
  // it. Distinct names sidestep that entirely.
  /// @brief Drive the underlying lifecycle node from UNCONFIGURED to INACTIVE.
  /// @return The resulting lifecycle transition outcome.
  virtual CallbackReturn configure_node() = 0;
  /// @brief Drive the underlying lifecycle node from INACTIVE to ACTIVE.
  /// @return The resulting lifecycle transition outcome.
  virtual CallbackReturn activate_node() = 0;
  /// @brief Drive the underlying lifecycle node from ACTIVE to INACTIVE.
  /// @return The resulting lifecycle transition outcome.
  virtual CallbackReturn deactivate_node() = 0;
  /// @brief Drive the underlying lifecycle node from INACTIVE to UNCONFIGURED.
  /// @return The resulting lifecycle transition outcome.
  virtual CallbackReturn cleanup_node() = 0;
  /// @brief Drive the underlying lifecycle node to FINALIZED, from whatever state
  /// it is currently in.
  /// @return The resulting lifecycle transition outcome.
  virtual CallbackReturn shutdown_node() = 0;
  /// @brief The underlying lifecycle node's current state id.
  /// @return The underlying lifecycle node's current state id (one of the
  ///   `lifecycle_msgs::msg::State::PRIMARY_STATE_*` constants).
  virtual uint8_t get_current_state_id() const = 0;

  /// @brief So a `Deployment` can add the underlying node to its shared executor
  /// without knowing its concrete type.
  /// @return The underlying node's base interface.
  virtual rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() = 0;

  /// @brief The `capability` identity field published on /capabilities and /capabilities_status.
  /// @return The `capability` identity field published on /capabilities
  ///   and /capabilities_status.
  virtual const std::string & get_capability_name() const = 0;
  /// @brief The `robot` identity field published on /capabilities and /capabilities_status.
  /// @return The `robot` identity field published on /capabilities and
  ///   /capabilities_status.
  virtual const std::string & get_robot_name() const = 0;
};

}  // namespace easyfleet_core

#endif  // EASYFLEET_CORE__CAPABILITY_NODE_BASE_HPP_
