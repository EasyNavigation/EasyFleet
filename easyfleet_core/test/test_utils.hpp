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

#ifndef EASYFLEET_CORE__TEST__TEST_UTILS_HPP_
#define EASYFLEET_CORE__TEST__TEST_UTILS_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>
#include <thread>

#include "rclcpp/executor.hpp"

namespace easyfleet_core_test
{

/// Starts `executor.spin()` on a background thread and blocks until it has
/// actually begun spinning before returning it. `Executor::cancel()` only
/// reliably interrupts a `spin()` that has already started: calling it any
/// earlier races with the thread below and can block `join()` forever (this
/// is the root cause behind sporadic test hangs seen when a fixture's
/// TearDown() runs immediately after SetUp(), before the spin thread was
/// scheduled).
inline std::thread spin_in_background(rclcpp::Executor & executor)
{
  std::thread thread([&executor] {executor.spin();});
  while (!executor.is_spinning()) {
    std::this_thread::yield();
  }
  return thread;
}

/// Appends a process-wide unique suffix to `base`. Node/action names are
/// reused across many tests in the same binary; giving each test fixture
/// fresh names avoids DDS entities from a just-destroyed node/action server
/// racing with newly-created ones of the same name, which otherwise shows up
/// as sporadic hangs when the whole suite runs back to back.
inline std::string unique_test_name(const std::string & base)
{
  static std::atomic<uint64_t> counter{0};
  return base + "_" + std::to_string(counter.fetch_add(1));
}

/// Polls `pred` until it returns true or `timeout` elapses. Returns the last
/// value of `pred`. Used instead of fixed sleeps to keep integration tests
/// fast and non-flaky.
template<typename Predicate>
bool wait_until(
  Predicate pred,
  std::chrono::milliseconds timeout,
  std::chrono::milliseconds poll = std::chrono::milliseconds(10))
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (pred()) {
      return true;
    }
    std::this_thread::sleep_for(poll);
  }
  return pred();
}

}  // namespace easyfleet_core_test

#endif  // EASYFLEET_CORE__TEST__TEST_UTILS_HPP_
