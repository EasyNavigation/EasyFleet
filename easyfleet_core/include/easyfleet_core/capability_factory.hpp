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

#ifndef EASYFLEET_CORE__CAPABILITY_FACTORY_HPP_
#define EASYFLEET_CORE__CAPABILITY_FACTORY_HPP_

#include <memory>
#include <string>

#include "rclcpp/node_options.hpp"

#include "easyfleet_core/capability.hpp"
#include "easyfleet_core/capability_node_base.hpp"

namespace easyfleet_core
{

/// @brief Pluginlib base class for "a thing that can build a capability node by
/// name". Not itself default-constructible-and-ready like a typical
/// pluginlib strategy plugin, because `Capability<ActionServerT>` is a real
/// `rclcpp_lifecycle::LifecycleNode` and its base class needs a name at
/// *construction* time, not after -- so instead of loading the capability
/// node itself through pluginlib (which would force it to become
/// default-constructible, breaking every existing use of `Capability<T>`),
/// pluginlib loads this small, genuinely-default-constructible *factory*
/// object, and the factory does the eager construction on `Robot`'s behalf.
class CapabilityFactory
{
public:
  /// @brief Shared pointer to a `CapabilityFactory`.
  using SharedPtr = std::shared_ptr<CapabilityFactory>;

  virtual ~CapabilityFactory() = default;

  /// @brief Constructs and returns one capability node instance ready for the
  /// caller to `configure()`/`activate()`.
  /// @param capability_name Passed straight through to `Capability<T>`'s own
  ///   constructor -- the node name, the action name, and the `capability`
  ///   identity field published on /capabilities.
  /// @param options Node options `Robot::add_capability()` has already
  ///   stamped with this robot's namespace (see `robot.hpp`), plus whatever
  ///   parameter overrides the caller supplied.
  /// @return The newly constructed capability node, through its
  ///   `CapabilityNodeBase` interface.
  virtual std::shared_ptr<CapabilityNodeBase> create(
    const std::string & capability_name,
    const rclcpp::NodeOptions & options) const = 0;
};

/// @brief Boilerplate-free `CapabilityFactory` for any `Capability<ActionServerT>`.
/// A concrete backend needs nothing beyond one line to become
/// plugin-loadable -- see the .cpp of any capability implementation for the
/// matching `PLUGINLIB_EXPORT_CLASS` registration:
/// \code
/// PLUGINLIB_EXPORT_CLASS(
///   (easyfleet_core::CapabilityFactoryFor<PerceptionFakeActionServer>),
///   easyfleet_core::CapabilityFactory)
/// \endcode
template<typename ActionServerT>
class CapabilityFactoryFor : public CapabilityFactory
{
public:
  std::shared_ptr<CapabilityNodeBase> create(
    const std::string & capability_name,
    const rclcpp::NodeOptions & options) const override
  {
    return std::make_shared<Capability<ActionServerT>>(capability_name, options);
  }
};

}  // namespace easyfleet_core

#endif  // EASYFLEET_CORE__CAPABILITY_FACTORY_HPP_
