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

#ifndef EASYFLEET_CORE__CAPABILITY_HPP_
#define EASYFLEET_CORE__CAPABILITY_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "easyfleet_core/action_server_base.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "easyfleet_core/capability_node_base.hpp"
#include "easyfleet_core/link_watchdog.hpp"
#include "easyfleet_core/order_history.hpp"
#include "easyfleet_interfaces/msg/capability_description.hpp"
#include "easyfleet_interfaces/msg/capability_status.hpp"
#include "easyfleet_interfaces/msg/fleet_heartbeat.hpp"
#include "easyfleet_interfaces/srv/get_order.hpp"

namespace easyfleet_core
{

/// @brief Lifecycle node that advertises a single action as a "capability".
/**
 * `Capability<ActionServerT>` is a `rclcpp_lifecycle::LifecycleNode` that:
 *  - owns an instance of `ActionServerT` (a concrete subclass of
 *    `ActionServerBase<ActionType>`, constructed as
 *    `ActionServerT(rclcpp_lifecycle::LifecycleNode &, const std::string &)`),
 *    exposed through its `ActionServerBase<ActionType>::SharedPtr` base type;
 *  - on `on_activate`, reads the file named by the `capabilities_file`
 *    string parameter and publishes it, together with its own runtime
 *    identity (namespace and fully-qualified action name), as an
 *    `easyfleet_interfaces/CapabilityDescription` on the reliable,
 *    transient-local `/capabilities` topic, so late subscribers still get it;
 *  - while active, publishes an `easyfleet_interfaces/CapabilityStatus`
 *    heartbeat once a second on `/capabilities_status`;
 *  - keeps its recent orders (`<capability>.order_history`, 20 by default),
 *    queryable on the `~/get_order` service (e.g. /robot1/navigation/get_order);
 *  - with `<capability>.link_timeout` > 0 s, watches `/fleet_heartbeat` and,
 *    when it is not heard for that long and `<capability>.on_link_lost` is
 *    "cancel", stops the running goal ("continue", the default, only reports it).
 *
 * `ActionServerT` must publicly inherit `ActionServerBase<SomeActionType>`
 * and be constructible as `ActionServerT(LifecycleNode &, const std::string &)`.
 *
 * The `robot`/`capability`/`action_name` identity published on both topics
 * is resolved from this node's actual namespace at construction time, so a
 * capability launched under namespace `robot1` announces itself as such
 * without any code changes -- this is what lets a control center tell
 * apart, say, `robot1`'s and `robot2`'s `navigation` capabilities.
 *
 * Also implements `CapabilityNodeBase`, so a `Robot`/`Deployment` that
 * doesn't know (and doesn't need to know) `ActionServerT` can still drive
 * this instance's lifecycle and add its node to an executor -- see
 * `capability_node_base.hpp` and `capability_factory.hpp`.
 */
template<typename ActionServerT>
class Capability : public rclcpp_lifecycle::LifecycleNode, public CapabilityNodeBase
{
public:
  /// @brief Lifecycle transition result type shared with `rclcpp_lifecycle`.
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  /// @brief The `ActionServerBase<...>` specialization backing this capability.
  using ActionServerBaseT = ActionServerBase<typename ActionServerT::ActionType>;

  /// @brief Constructs the capability node.
  /// @param capability_name Used as the node name, the action name of the
  ///   contained action server, and the `capability` identity field.
  /// @param options Forwarded to the underlying `LifecycleNode`.
  explicit Capability(
    const std::string & capability_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~Capability() override;

  /// @brief The action server backing this capability.
  /// @return The contained `ActionServerT`, through its
  ///   `ActionServerBaseT::SharedPtr` base type.
  typename ActionServerBaseT::SharedPtr get_action_server() const noexcept;

  /// @brief The `capability` identity field published on /capabilities and /capabilities_status.
  /// @return The `capability` identity field published on /capabilities
  ///   and /capabilities_status.
  const std::string & get_capability_name() const noexcept override;

  /// @brief This node's namespace, without the leading '/' (empty if none), e.g.
  /// "robot1". This is the `robot` identity field published on
  /// /capabilities and /capabilities_status.
  /// @return This node's resolved robot name.
  const std::string & get_robot_name() const noexcept override;

  /// @brief Fully-qualified name of the action this capability exposes, e.g.
  /// "/robot1/navigation". This is the `action_name` identity field
  /// published on /capabilities and /capabilities_status.
  /// @return The resolved, fully-qualified action name.
  const std::string & get_resolved_action_name() const noexcept;

  /// @brief The recent orders of this capability.
  const OrderHistory & get_order_history() const noexcept;

  /// @brief This capability's status, as in its heartbeat.
  easyfleet_interfaces::msg::CapabilityStatus get_status() const override;

  // -- CapabilityNodeBase --
  //
  // Named *_node() (see capability_node_base.hpp) precisely so these don't
  // collide with rclcpp_lifecycle::LifecycleNode's own same-purpose
  // configure()/activate()/.../shutdown() member functions -- those keep
  // working completely unchanged (e.g. test_capability.cpp's
  // `capability_->configure(cb)` one-arg overload) since nothing here
  // hides or shadows them. Each *_node() implementation (capability_impl.hpp)
  // just calls the corresponding LifecycleNode transition and returns its
  // CallbackReturn. get_node_base_interface() is the one exception: its
  // name/signature/return type happen to exactly match LifecycleNode's own,
  // so this override *does* hide that one -- harmlessly, since it forwards
  // to it internally and nothing in this codebase calls it on a
  // `Capability<T>` directly today.
  CallbackReturn configure_node() override;
  CallbackReturn activate_node() override;
  CallbackReturn deactivate_node() override;
  CallbackReturn cleanup_node() override;
  CallbackReturn shutdown_node() override;
  uint8_t get_current_state_id() const override;
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() override;

protected:
  /// @brief Reads `capabilities_file` and configures the contained `ActionServerT`.
  /// @param previous_state State this node is transitioning from.
  /// @return `CallbackReturn::SUCCESS` on success, `CallbackReturn::FAILURE`
  ///   otherwise.
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  /// @brief Publishes the capability description and starts the status heartbeat.
  /// @param previous_state State this node is transitioning from.
  /// @return `CallbackReturn::SUCCESS` on success, `CallbackReturn::FAILURE`
  ///   otherwise.
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  /// @brief Stops the status heartbeat.
  /// @param previous_state State this node is transitioning from.
  /// @return `CallbackReturn::SUCCESS` on success, `CallbackReturn::FAILURE`
  ///   otherwise.
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  /// @brief Releases the resources acquired in `on_configure()`.
  /// @param previous_state State this node is transitioning from.
  /// @return `CallbackReturn::SUCCESS` on success, `CallbackReturn::FAILURE`
  ///   otherwise.
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  /// @brief Stops the status heartbeat if still running.
  /// @param previous_state State this node is transitioning from.
  /// @return `CallbackReturn::SUCCESS` on success, `CallbackReturn::FAILURE`
  ///   otherwise.
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

private:
  void publish_heartbeat();
  void stop_heartbeat();
  void check_link();
  void on_get_order(
    const std::shared_ptr<easyfleet_interfaces::srv::GetOrder::Request> request,
    std::shared_ptr<easyfleet_interfaces::srv::GetOrder::Response> response);

  std::string capability_name_;
  std::string robot_name_;
  std::string resolved_action_name_;
  typename ActionServerBaseT::SharedPtr action_server_;

  rclcpp_lifecycle::LifecyclePublisher<
    easyfleet_interfaces::msg::CapabilityDescription>::SharedPtr capabilities_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
    easyfleet_interfaces::msg::CapabilityStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  std::string order_history_param_;
  std::string link_timeout_param_;
  std::string on_link_lost_param_;
  std::unique_ptr<OrderHistory> order_history_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr goal_status_sub_;
  rclcpp::Service<easyfleet_interfaces::srv::GetOrder>::SharedPtr get_order_srv_;

  LinkWatchdog link_watchdog_;
  bool cancel_on_link_lost_{false};
  rclcpp::Subscription<easyfleet_interfaces::msg::FleetHeartbeat>::SharedPtr fleet_heartbeat_sub_;
  rclcpp::TimerBase::SharedPtr link_timer_;
};

}  // namespace easyfleet_core

#include "easyfleet_core/detail/capability_impl.hpp"

#endif  // EASYFLEET_CORE__CAPABILITY_HPP_
