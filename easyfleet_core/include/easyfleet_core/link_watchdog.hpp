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


#ifndef EASYFLEET_CORE__LINK_WATCHDOG_HPP_
#define EASYFLEET_CORE__LINK_WATCHDOG_HPP_

#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>

namespace easyfleet_core
{

/// @brief Whether the fleet's heartbeat is still heard, with a timeout.
/**
 * States are those of CapabilityStatus.LINK_*: NOT_MONITORED with a timeout of
 * 0 (or less), WAITING until the first heartbeat, then OK or LOST depending
 * on the age of the last one. Thread-safe.
 */
class LinkWatchdog
{
public:
  using Clock = std::chrono::steady_clock;

  /// @param timeout_s Seconds without a heartbeat to consider the link lost.
  explicit LinkWatchdog(double timeout_s = 0.0);

  /// @brief Changes the timeout; the link is WAITING again.
  void reset(double timeout_s);

  /// @brief A heartbeat was heard.
  void heard(Clock::time_point now);

  /// @brief Recomputes the state.
  /// @return The new state.
  uint8_t update(Clock::time_point now);

  /// @brief The state as of the last heard() or update().
  [[nodiscard]] uint8_t state() const;

  /// @brief Whether a timeout is set.
  [[nodiscard]] bool monitored() const;

private:
  mutable std::mutex mutex_;
  std::optional<Clock::duration> timeout_;
  std::optional<Clock::time_point> last_heard_;
  uint8_t state_;
};

}  // namespace easyfleet_core

#endif  // EASYFLEET_CORE__LINK_WATCHDOG_HPP_
