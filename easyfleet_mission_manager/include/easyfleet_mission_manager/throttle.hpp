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

#ifndef EASYFLEET_MISSION_MANAGER__THROTTLE_HPP_
#define EASYFLEET_MISSION_MANAGER__THROTTLE_HPP_

#include <chrono>
#include <memory>

namespace easyfleet_mission_manager
{

/// Value-semantics rate limiter: copies (e.g. captured by value into a
/// `std::function`) share the same underlying clock, so a feedback callback
/// that fires at, say, 20 Hz can be throttled down to a readable printing
/// rate without flooding the terminal.
class Throttle
{
public:
  explicit Throttle(std::chrono::milliseconds interval)
  : interval_(interval), last_(std::make_shared<std::chrono::steady_clock::time_point>())
  {
  }

  bool ready() const
  {
    const auto now = std::chrono::steady_clock::now();
    if (now - *last_ >= interval_) {
      *last_ = now;
      return true;
    }
    return false;
  }

private:
  std::chrono::milliseconds interval_;
  std::shared_ptr<std::chrono::steady_clock::time_point> last_;
};

}  // namespace easyfleet_mission_manager

#endif  // EASYFLEET_MISSION_MANAGER__THROTTLE_HPP_
