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

#ifndef EASYFLEET_CORE__TEST__TEST_FIBONACCI_SERVER_HPP_
#define EASYFLEET_CORE__TEST__TEST_FIBONACCI_SERVER_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include "example_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "easyfleet_core/action_server_base.hpp"

namespace easyfleet_core_test
{

using Fibonacci = example_interfaces::action::Fibonacci;

// Magic `order` values used to steer test-controlled behavior without
// needing extra fields on the goal message.
constexpr int32_t kOrderThrow = -100;
constexpr int32_t kOrderForgetToSettle = -101;

/// Minimal, fully test-controllable action server built on top of
/// easyfleet_core::ActionServerBase<Fibonacci>. `order` drives how many
/// feedback steps are published (and therefore how long the goal runs),
/// which is what tests use to script preemption/cancellation races.
class TestFibonacciServer
  : public rclcpp::Node, public easyfleet_core::ActionServerBase<Fibonacci>
{
public:
  explicit TestFibonacciServer(
    const std::string & node_name = "test_fibonacci_server",
    bool default_allow_preemption = true,
    const std::string & action_name = "fibonacci",
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node(node_name, options),
    easyfleet_core::ActionServerBase<Fibonacci>(*this, action_name, default_allow_preemption)
  {
  }

  void set_step_delay(std::chrono::milliseconds delay)
  {
    step_delay_ms_.store(delay.count());
  }

  int goals_started() const {return goals_started_.load();}
  int goals_preempted() const {return goals_preempted_.load();}
  int on_preempted_calls() const {return on_preempted_calls_.load();}

protected:
  rclcpp_action::GoalResponse on_goal_received(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const Fibonacci::Goal> goal) override
  {
    if (goal->order < 0 && goal->order != kOrderThrow && goal->order != kOrderForgetToSettle) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse on_cancel_requested(
    const GoalHandleSharedPtr /*goal_handle*/) override
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void on_preempted(const GoalHandleSharedPtr & preempted_goal_handle) override
  {
    (void)preempted_goal_handle;
    on_preempted_calls_.fetch_add(1);
  }

  void on_execute(const GoalHandleSharedPtr goal_handle) override
  {
    goals_started_.fetch_add(1);
    const auto order = goal_handle->get_goal()->order;

    if (order == kOrderThrow) {
      throw std::runtime_error("boom");
    }
    if (order == kOrderForgetToSettle) {
      return;
    }

    auto feedback = std::make_shared<Fibonacci::Feedback>();
    feedback->sequence.push_back(0);
    if (order > 0) {
      feedback->sequence.push_back(1);
    }

    for (int32_t i = 0; i < order; ++i) {
      if (goal_handle->is_canceling()) {
        auto result = std::make_shared<Fibonacci::Result>();
        result->sequence = feedback->sequence;
        goal_handle->canceled(result);
        return;
      }
      if (is_preempt_requested()) {
        goals_preempted_.fetch_add(1);
        auto result = std::make_shared<Fibonacci::Result>();
        result->sequence = feedback->sequence;
        goal_handle->abort(result);
        return;
      }
      if (is_shutdown_requested()) {
        // Not a client-requested cancellation (goal_handle->is_canceling() is
        // false here), so the only valid direct transition from EXECUTING is
        // abort(); canceled() would require having gone through CANCELING.
        auto result = std::make_shared<Fibonacci::Result>();
        result->sequence = feedback->sequence;
        goal_handle->abort(result);
        return;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(step_delay_ms_.load()));

      const auto n = feedback->sequence.size();
      feedback->sequence.push_back(feedback->sequence[n - 1] + feedback->sequence[n - 2]);
      goal_handle->publish_feedback(feedback);
    }

    auto result = std::make_shared<Fibonacci::Result>();
    result->sequence = feedback->sequence;
    goal_handle->succeed(result);
  }

private:
  std::atomic<int64_t> step_delay_ms_{20};
  std::atomic<int> goals_started_{0};
  std::atomic<int> goals_preempted_{0};
  std::atomic<int> on_preempted_calls_{0};
};

}  // namespace easyfleet_core_test

#endif  // EASYFLEET_CORE__TEST__TEST_FIBONACCI_SERVER_HPP_
