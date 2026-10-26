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


#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "easyfleet_core/capability.hpp"
#include "easyfleet_core/navigation_action_server_base.hpp"
#include "easyfleet_interfaces/msg/capability_status.hpp"
#include "easyfleet_interfaces/msg/fleet_heartbeat.hpp"
#include "easyfleet_interfaces/srv/get_order.hpp"
#include "test_utils.hpp"

using namespace std::chrono_literals;
using easyfleet_core::OrderHistory;
using easyfleet_core_test::spin_in_background;
using easyfleet_core_test::unique_test_name;
using easyfleet_core_test::wait_until;
using easyfleet_interfaces::msg::CapabilityStatus;
using easyfleet_interfaces::msg::FleetHeartbeat;
using easyfleet_interfaces::msg::OrderStatus;
using easyfleet_interfaces::srv::GetOrder;
using Navigation = easyfleet_interfaces::action::Navigation;

namespace
{

/// Navigation server that holds each goal for `<name>.mock_delay_seconds`.
class TestNavigationServer : public easyfleet_core::NavigationActionServerBase
{
public:
  TestNavigationServer(rclcpp_lifecycle::LifecycleNode & node, const std::string & name)
  : easyfleet_core::NavigationActionServerBase(node, name)
  {
    delay_s_ = node.declare_parameter(name + ".mock_delay_seconds", 0.0);
  }

  ~TestNavigationServer() override
  {
    stop();
  }

protected:
  rclcpp_action::GoalResponse on_goal_received(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const Navigation::Goal> goal) override
  {
    return goal->parameters_json == "reject" ?
           rclcpp_action::GoalResponse::REJECT : rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  void on_execute(const GoalHandleSharedPtr goal_handle) override
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(delay_s_));
    while (std::chrono::steady_clock::now() < deadline) {
      if (goal_handle->is_canceling()) {
        goal_handle->canceled(std::make_shared<Navigation::Result>());
        return;
      }
      if (is_shutdown_requested() || is_preempt_requested()) {
        goal_handle->abort(std::make_shared<Navigation::Result>());
        return;
      }
      std::this_thread::sleep_for(5ms);
    }
    goal_handle->succeed(std::make_shared<Navigation::Result>());
  }

private:
  double delay_s_{0.0};
};

using NavCapability = easyfleet_core::Capability<TestNavigationServer>;
using GoalHandle = rclcpp_action::ClientGoalHandle<Navigation>;

std::string make_temp_file()
{
  static std::atomic<uint64_t> counter{0};
  const std::string path = "/tmp/easyfleet_core_test_orders_" +
    std::to_string(::getpid()) + "_" + std::to_string(counter.fetch_add(1)) + ".json";
  std::ofstream(path) << "{}";
  return path;
}

class OrdersTest : public ::testing::Test
{
protected:
  void TearDown() override
  {
    if (executor_) {
      executor_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    heartbeat_timer_.reset();
    client_.reset();
    client_node_.reset();
    capability_.reset();
    executor_.reset();
    std::remove(path_.c_str());
  }

  /// Creates and activates the capability, under `ns` if not empty.
  /// Each goal takes `delay_s`; `params` names are relative to the capability
  /// (e.g. "link_timeout" for "<capability>.link_timeout").
  void start(
    double delay_s = 0.0, std::vector<rclcpp::Parameter> params = {},
    const std::string & ns = "", bool activate = true)
  {
    name_ = unique_test_name("nav");
    path_ = make_temp_file();
    for (auto & param : params) {
      param = rclcpp::Parameter(name_ + "." + param.get_name(), param.get_parameter_value());
    }
    params.emplace_back("capabilities_file", path_);
    params.emplace_back(name_ + ".mock_delay_seconds", delay_s);
    rclcpp::NodeOptions options;
    options.parameter_overrides(params);
    if (!ns.empty()) {
      options.arguments({"--ros-args", "-r", "__ns:=/" + ns});
    }
    capability_ = std::make_shared<NavCapability>(name_, options);
    client_node_ = std::make_shared<rclcpp::Node>(unique_test_name("orders_client"));
    client_ = rclcpp_action::create_client<Navigation>(
      client_node_, capability_->get_resolved_action_name());
    get_order_ = client_node_->create_client<GetOrder>(
      capability_->get_resolved_action_name() + "/get_order");
    heartbeat_pub_ = client_node_->create_publisher<FleetHeartbeat>(
      "/fleet_heartbeat", rclcpp::QoS(1));
    status_sub_ = client_node_->create_subscription<CapabilityStatus>(
      "/capabilities_status", rclcpp::QoS(10).reliable(),
      [this](const CapabilityStatus & msg) {
        if (msg.capability == name_) {
          std::lock_guard<std::mutex> lock(mutex_);
          statuses_.push_back(msg);
        }
      });
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
      rclcpp::ExecutorOptions(), 2);
    executor_->add_node(capability_->get_node_base_interface());
    executor_->add_node(client_node_);
    spin_thread_ = spin_in_background(*executor_);
    if (activate) {
      capability_->configure();
      capability_->activate();
      ASSERT_TRUE(client_->wait_for_action_server(5s));
      ASSERT_TRUE(get_order_->wait_for_service(5s));
    }
  }

  GoalHandle::SharedPtr send(const std::string & order_id, const std::string & params = "")
  {
    Navigation::Goal goal;
    goal.order_id = order_id;
    goal.parameters_json = params;
    auto future = client_->async_send_goal(goal);
    if (future.wait_for(5s) != std::future_status::ready) {
      ADD_FAILURE() << "no goal response";
      return nullptr;
    }
    return future.get();
  }

  rclcpp_action::ResultCode result_of(const GoalHandle::SharedPtr & handle)
  {
    auto future = client_->async_get_result(handle);
    if (future.wait_for(10s) != std::future_status::ready) {
      ADD_FAILURE() << "no result";
      return rclcpp_action::ResultCode::UNKNOWN;
    }
    return future.get().code;
  }

  GetOrder::Response query(const std::string & order_id)
  {
    auto request = std::make_shared<GetOrder::Request>();
    request->order_id = order_id;
    auto future = get_order_->async_send_request(request);
    if (future.wait_for(5s) != std::future_status::ready) {
      ADD_FAILURE() << "no get_order response";
      return GetOrder::Response();
    }
    return *future.get();
  }

  /// Waits until get_order reports `state` for `order_id`.
  bool wait_state(const std::string & order_id, uint8_t state, std::chrono::seconds timeout = 5s)
  {
    return wait_until(
      [&] {
        const auto r = query(order_id);
        return r.found && r.order.state == state;
      }, timeout, 50ms);
  }

  /// Publishes /fleet_heartbeat every `period` until stopped.
  void start_heartbeat(std::chrono::milliseconds period = 50ms)
  {
    heartbeat_timer_ = client_node_->create_wall_timer(
      period, [this]() {
        FleetHeartbeat msg;
        msg.source = "test";
        heartbeat_pub_->publish(msg);
      });
  }

  void stop_heartbeat()
  {
    heartbeat_timer_->cancel();
  }

  std::vector<CapabilityStatus> statuses()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return statuses_;
  }

  std::string name_;
  std::string path_;
  std::shared_ptr<NavCapability> capability_;
  rclcpp::Node::SharedPtr client_node_;
  rclcpp_action::Client<Navigation>::SharedPtr client_;
  rclcpp::Client<GetOrder>::SharedPtr get_order_;
  rclcpp::Publisher<FleetHeartbeat>::SharedPtr heartbeat_pub_;
  rclcpp::Subscription<CapabilityStatus>::SharedPtr status_sub_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  std::mutex mutex_;
  std::vector<CapabilityStatus> statuses_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
};

TEST_F(OrdersTest, AnOrderIsQueryableByItsOrderIdWithItsFinalState)
{
  start();
  auto handle = send("deliver-1");
  ASSERT_TRUE(handle);
  EXPECT_EQ(result_of(handle), rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_TRUE(wait_state("deliver-1", OrderStatus::STATE_SUCCEEDED));

  const auto r = query("deliver-1");
  EXPECT_EQ(r.order.order_id, "deliver-1");
  EXPECT_EQ(r.order.goal_id, OrderHistory::to_hex(handle->get_goal_id()));
  EXPECT_EQ(r.order.action_name, capability_->get_resolved_action_name());
  const auto accepted = rclcpp::Time(r.order.accepted_stamp);
  const auto finished = rclcpp::Time(r.order.finished_stamp);
  EXPECT_GT(accepted.nanoseconds(), 0);
  EXPECT_GE(finished.nanoseconds(), accepted.nanoseconds());
}

TEST_F(OrdersTest, WithoutOrderIdTheGoalUuidIsTheOrderId)
{
  start();
  auto handle = send("");
  ASSERT_TRUE(handle);
  EXPECT_EQ(result_of(handle), rclcpp_action::ResultCode::SUCCEEDED);
  const auto hex = OrderHistory::to_hex(handle->get_goal_id());
  ASSERT_TRUE(wait_state(hex, OrderStatus::STATE_SUCCEEDED));
  EXPECT_EQ(query(hex).order.goal_id, hex);
}

TEST_F(OrdersTest, UnknownAndRejectedOrdersAreNotFound)
{
  start();
  EXPECT_FALSE(query("never-sent").found);
  EXPECT_FALSE(send("refused", "reject"));
  EXPECT_FALSE(query("refused").found);
  EXPECT_FALSE(query("").found);
}

TEST_F(OrdersTest, ARunningOrderIsExecutingAndTheHeartbeatCarriesIt)
{
  start(2.5);
  auto handle = send("long-1");
  ASSERT_TRUE(handle);
  ASSERT_TRUE(wait_state("long-1", OrderStatus::STATE_EXECUTING));
  EXPECT_EQ(rclcpp::Time(query("long-1").order.finished_stamp).nanoseconds(), 0);

  const auto running = capability_->get_status();
  EXPECT_TRUE(running.busy);
  EXPECT_EQ(running.order_id, "long-1");
  EXPECT_EQ(running.order_state, OrderStatus::STATE_EXECUTING);
  ASSERT_TRUE(
    wait_until(
      [&] {
        for (const auto & s : statuses()) {
          if (s.busy && s.order_id == "long-1" && s.order_state == OrderStatus::STATE_EXECUTING) {
            return true;
          }
        }
        return false;
      }, 3s));

  EXPECT_EQ(result_of(handle), rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_TRUE(wait_state("long-1", OrderStatus::STATE_SUCCEEDED));
  // Idle: the heartbeat keeps the last order and its final state.
  ASSERT_TRUE(
    wait_until(
      [&] {
        const auto all = statuses();
        return !all.empty() && !all.back().busy && all.back().order_id == "long-1" &&
               all.back().order_state == OrderStatus::STATE_SUCCEEDED;
      }, 3s));
}

TEST_F(OrdersTest, BeforeAnyOrderTheHeartbeatHasNone)
{
  start();
  const auto status = capability_->get_status();
  EXPECT_EQ(status.order_id, "");
  EXPECT_EQ(status.order_state, OrderStatus::STATE_UNKNOWN);
  EXPECT_FALSE(status.busy);
}

TEST_F(OrdersTest, CanceledAndPreemptedOrdersKeepTheirOwnStates)
{
  start(3.0);
  auto first = send("first");
  ASSERT_TRUE(first);
  ASSERT_TRUE(wait_state("first", OrderStatus::STATE_EXECUTING));
  auto second = send("second");
  ASSERT_TRUE(second);
  EXPECT_EQ(result_of(first), rclcpp_action::ResultCode::ABORTED);
  ASSERT_TRUE(wait_state("first", OrderStatus::STATE_ABORTED));
  ASSERT_TRUE(wait_state("second", OrderStatus::STATE_EXECUTING));

  auto cancel = client_->async_cancel_goal(second);
  ASSERT_EQ(cancel.wait_for(5s), std::future_status::ready);
  EXPECT_EQ(result_of(second), rclcpp_action::ResultCode::CANCELED);
  ASSERT_TRUE(wait_state("second", OrderStatus::STATE_CANCELED));
  EXPECT_EQ(query("first").order.state, OrderStatus::STATE_ABORTED);
}

TEST_F(OrdersTest, OnlyTheConfiguredNumberOfOrdersIsKept)
{
  start(0.0, {rclcpp::Parameter("order_history", 2)});
  for (const auto * id : {"o1", "o2", "o3"}) {
    auto handle = send(id);
    ASSERT_TRUE(handle);
    EXPECT_EQ(result_of(handle), rclcpp_action::ResultCode::SUCCEEDED);
  }
  ASSERT_TRUE(wait_state("o3", OrderStatus::STATE_SUCCEEDED));
  EXPECT_FALSE(query("o1").found);
  EXPECT_TRUE(query("o2").found);
  EXPECT_EQ(capability_->get_order_history().size(), 2u);
}

TEST_F(OrdersTest, TheServiceLivesUnderTheRobotAndTheCapability)
{
  start(0.0, {}, "robot_orders");
  EXPECT_EQ(
    get_order_->get_service_name(), "/robot_orders/" + name_ + "/get_order");
  auto handle = send("ns-order");
  ASSERT_TRUE(handle);
  EXPECT_EQ(result_of(handle), rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_TRUE(wait_state("ns-order", OrderStatus::STATE_SUCCEEDED));
  EXPECT_EQ(query("ns-order").order.action_name, "/robot_orders/" + name_);
}

TEST_F(OrdersTest, TheHistorySurvivesDeactivateAndActivate)
{
  start();
  auto handle = send("kept");
  ASSERT_TRUE(handle);
  EXPECT_EQ(result_of(handle), rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_TRUE(wait_state("kept", OrderStatus::STATE_SUCCEEDED));
  capability_->deactivate();
  EXPECT_TRUE(query("kept").found) << "the service stays up while inactive";
  capability_->activate();
  EXPECT_EQ(query("kept").order.state, OrderStatus::STATE_SUCCEEDED);
}

TEST_F(OrdersTest, AnOrderStoppedByDeactivateIsAborted)
{
  start(5.0);
  auto handle = send("stopped");
  ASSERT_TRUE(handle);
  ASSERT_TRUE(wait_state("stopped", OrderStatus::STATE_EXECUTING));
  capability_->deactivate();
  EXPECT_EQ(result_of(handle), rclcpp_action::ResultCode::ABORTED);
  ASSERT_TRUE(wait_state("stopped", OrderStatus::STATE_ABORTED));
}

TEST_F(OrdersTest, ABadOnLinkLostFailsToConfigure)
{
  start(0.0, {rclcpp::Parameter("on_link_lost", "explode")}, "", false);
  capability_->configure();
  EXPECT_EQ(
    capability_->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(OrdersTest, WithoutLinkTimeoutTheLinkIsNotMonitored)
{
  start(1.0);
  EXPECT_EQ(capability_->get_status().link_state, CapabilityStatus::LINK_NOT_MONITORED);
  auto handle = send("unmonitored");
  ASSERT_TRUE(handle);
  EXPECT_EQ(result_of(handle), rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_EQ(capability_->get_status().link_state, CapabilityStatus::LINK_NOT_MONITORED);
}

TEST_F(OrdersTest, LinkLostWithCancelStopsTheRunningGoalAndGoalsKeepWorking)
{
  start(
    10.0, {rclcpp::Parameter("link_timeout", 0.4), rclcpp::Parameter("on_link_lost", "cancel")});
  EXPECT_EQ(capability_->get_status().link_state, CapabilityStatus::LINK_WAITING);

  // Never heard: a goal is not stopped however long it runs.
  auto waiting = send("while-waiting");
  ASSERT_TRUE(waiting);
  std::this_thread::sleep_for(1200ms);
  EXPECT_EQ(capability_->get_status().link_state, CapabilityStatus::LINK_WAITING);
  EXPECT_TRUE(capability_->get_action_server()->is_active());

  start_heartbeat();
  ASSERT_TRUE(
    wait_until(
      [&] {return capability_->get_status().link_state == CapabilityStatus::LINK_OK;}, 3s));
  EXPECT_TRUE(capability_->get_action_server()->is_active());

  stop_heartbeat();
  const auto lost_at = std::chrono::steady_clock::now();
  EXPECT_EQ(result_of(waiting), rclcpp_action::ResultCode::ABORTED);
  EXPECT_LT(std::chrono::steady_clock::now() - lost_at, 2s) << "stopped soon after the timeout";
  EXPECT_EQ(capability_->get_status().link_state, CapabilityStatus::LINK_LOST);
  ASSERT_TRUE(wait_state("while-waiting", OrderStatus::STATE_ABORTED));
  ASSERT_TRUE(
    wait_until(
      [&] {
        for (const auto & s : statuses()) {
          if (s.link_state == CapabilityStatus::LINK_LOST) {
            return true;
          }
        }
        return false;
      }, 3s)) << "the heartbeat reports the loss";

  // A goal sent while lost is accepted: the policy acts on the loss itself.
  auto during = send("while-lost");
  ASSERT_TRUE(during);
  std::this_thread::sleep_for(600ms);
  EXPECT_TRUE(capability_->get_action_server()->is_active());

  start_heartbeat();
  ASSERT_TRUE(
    wait_until(
      [&] {return capability_->get_status().link_state == CapabilityStatus::LINK_OK;}, 3s));
  auto cancel = client_->async_cancel_goal(during);
  ASSERT_EQ(cancel.wait_for(5s), std::future_status::ready);
  EXPECT_EQ(result_of(during), rclcpp_action::ResultCode::CANCELED);
}

TEST_F(OrdersTest, LinkLostWithContinueOnlyReportsIt)
{
  start(2.0, {rclcpp::Parameter("link_timeout", 0.3)});
  start_heartbeat();
  ASSERT_TRUE(
    wait_until(
      [&] {return capability_->get_status().link_state == CapabilityStatus::LINK_OK;}, 3s));
  auto handle = send("carry-on");
  ASSERT_TRUE(handle);
  stop_heartbeat();
  ASSERT_TRUE(
    wait_until(
      [&] {return capability_->get_status().link_state == CapabilityStatus::LINK_LOST;}, 3s));
  EXPECT_TRUE(capability_->get_action_server()->is_active());
  EXPECT_EQ(result_of(handle), rclcpp_action::ResultCode::SUCCEEDED);
}

TEST_F(OrdersTest, ReactivatingWaitsForTheLinkAgain)
{
  start(0.0, {rclcpp::Parameter("link_timeout", 0.3)});
  start_heartbeat();
  ASSERT_TRUE(
    wait_until(
      [&] {return capability_->get_status().link_state == CapabilityStatus::LINK_OK;}, 3s));
  stop_heartbeat();
  std::this_thread::sleep_for(300ms);  // no heartbeat left in flight
  capability_->deactivate();
  capability_->activate();
  EXPECT_EQ(capability_->get_status().link_state, CapabilityStatus::LINK_WAITING);
}

TEST_F(OrdersTest, StatusThroughTheTypeErasedBase)
{
  start();
  easyfleet_core::CapabilityNodeBase::SharedPtr base = capability_;
  const auto status = base->get_status();
  EXPECT_EQ(status.capability, name_);
  EXPECT_EQ(status.action_name, capability_->get_resolved_action_name());
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
