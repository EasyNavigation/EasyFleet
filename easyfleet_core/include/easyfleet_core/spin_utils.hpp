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

#ifndef EASYFLEET_CORE__SPIN_UTILS_HPP_
#define EASYFLEET_CORE__SPIN_UTILS_HPP_

#include "rclcpp/executor.hpp"

namespace easyfleet_core
{

/// @brief Spins `executor` until SIGINT/SIGTERM, then returns -- with the ROS
/// context still valid, unlike a plain `executor.spin()` under rclcpp's own
/// default signal handling.
/**
 * rclcpp's default SIGINT/SIGTERM handler calls `rclcpp::shutdown()`
 * *before* a blocking `spin()` call returns, and `rclcpp::shutdown()`
 * invalidates the context (publishers included) as its very first step --
 * so a lifecycle node's usual post-spin `deactivate()`/`cleanup()` sequence
 * fails ("publisher's context is invalid") and never actually runs. This
 * function sidesteps that: it installs its own SIGINT/SIGTERM handlers that
 * just cancel `executor`, so `spin()` returns while the context is still
 * valid, leaving the caller free to deactivate/clean up before calling
 * `rclcpp::shutdown()` itself.
 *
 * Requires the caller to have called `rclcpp::init()` with
 * `SignalHandlerOptions::None` -- otherwise rclcpp's own handler still
 * fires (in addition to this one) and shuts the context down regardless.
 *
 * @param executor Executor to spin and, on SIGINT/SIGTERM, cancel.
 */
void spin_until_shutdown(rclcpp::Executor & executor);

}  // namespace easyfleet_core

#endif  // EASYFLEET_CORE__SPIN_UTILS_HPP_
