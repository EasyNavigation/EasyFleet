// Copyright 2026 Intelligent Robotics Lab
//
// This file is part of the projects Arquimea-URJC and AURORAS
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

#ifndef ARCH_MOCKUP__CAPABILITY_HPP_
#define ARCH_MOCKUP__CAPABILITY_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "arch_mockup/action_server_base.hpp"
#include "arch_mockup_interfaces/msg/capability_description.hpp"
#include "arch_mockup_interfaces/msg/capability_status.hpp"

namespace arch_mockup
{

/// Lifecycle node that advertises a single action as a "capability".
/**
 * `Capability<ActionServerT>` is a `rclcpp_lifecycle::LifecycleNode` that:
 *  - owns an instance of `ActionServerT` (a concrete subclass of
 *    `ActionServerBase<ActionType>`, constructed as
 *    `ActionServerT(rclcpp_lifecycle::LifecycleNode *, const std::string &)`),
 *    exposed through its `ActionServerBase<ActionType>::SharedPtr` base type;
 *  - on `on_activate`, reads the file named by the `capabilities_file`
 *    string parameter and publishes it, together with its own runtime
 *    identity (namespace and fully-qualified action name), as an
 *    `arch_mockup_interfaces/CapabilityDescription` on the reliable,
 *    transient-local `/capabilities` topic, so late subscribers still get it;
 *  - while active, publishes an `arch_mockup_interfaces/CapabilityStatus`
 *    heartbeat once a second on `/capabilities_status`.
 *
 * `ActionServerT` must publicly inherit `ActionServerBase<SomeActionType>`
 * and be constructible as `ActionServerT(LifecycleNode *, const std::string &)`.
 *
 * The `robot`/`capability`/`action_name` identity published on both topics
 * is resolved from this node's actual namespace at construction time, so a
 * capability launched under namespace `robot1` announces itself as such
 * without any code changes -- this is what lets a control center tell
 * apart, say, `robot1`'s and `robot2`'s `navigation` capabilities.
 */
template<typename ActionServerT>
class Capability : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using ActionServerBaseT = ActionServerBase<typename ActionServerT::ActionType>;

  /// @param capability_name Used as the node name, the action name of the
  ///   contained action server, and the `capability` identity field.
  explicit Capability(
    const std::string & capability_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~Capability() override;

  /// The action server backing this capability.
  typename ActionServerBaseT::SharedPtr get_action_server() const noexcept;

  const std::string & get_capability_name() const noexcept;

  /// This node's namespace, without the leading '/' (empty if none), e.g.
  /// "robot1". This is the `robot` identity field published on
  /// /capabilities and /capabilities_status.
  const std::string & get_robot_name() const noexcept;

  /// Fully-qualified name of the action this capability exposes, e.g.
  /// "/robot1/navigation". This is the `action_name` identity field
  /// published on /capabilities and /capabilities_status.
  const std::string & get_resolved_action_name() const noexcept;

protected:
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

private:
  void publish_heartbeat();
  void stop_heartbeat();

  std::string capability_name_;
  std::string robot_name_;
  std::string resolved_action_name_;
  typename ActionServerBaseT::SharedPtr action_server_;

  rclcpp_lifecycle::LifecyclePublisher<
    arch_mockup_interfaces::msg::CapabilityDescription>::SharedPtr capabilities_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
    arch_mockup_interfaces::msg::CapabilityStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

}  // namespace arch_mockup

#include "arch_mockup/detail/capability_impl.hpp"

#endif  // ARCH_MOCKUP__CAPABILITY_HPP_
