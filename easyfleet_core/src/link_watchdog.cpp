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


#include "easyfleet_core/link_watchdog.hpp"

#include "easyfleet_interfaces/msg/capability_status.hpp"

namespace easyfleet_core
{

using easyfleet_interfaces::msg::CapabilityStatus;

LinkWatchdog::LinkWatchdog(double timeout_s)
{
  reset(timeout_s);
}

void LinkWatchdog::reset(double timeout_s)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_heard_.reset();
  if (timeout_s > 0.0) {
    timeout_ = std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(timeout_s));
    state_ = CapabilityStatus::LINK_WAITING;
  } else {
    timeout_.reset();
    state_ = CapabilityStatus::LINK_NOT_MONITORED;
  }
}

void LinkWatchdog::heard(Clock::time_point now)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!timeout_) {
    return;
  }
  if (!last_heard_ || now > *last_heard_) {
    last_heard_ = now;
  }
  state_ = CapabilityStatus::LINK_OK;
}

uint8_t LinkWatchdog::update(Clock::time_point now)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (timeout_ && last_heard_) {
    state_ = now - *last_heard_ > *timeout_ ?
      CapabilityStatus::LINK_LOST : CapabilityStatus::LINK_OK;
  }
  return state_;
}

uint8_t LinkWatchdog::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

bool LinkWatchdog::monitored() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return timeout_.has_value();
}

}  // namespace easyfleet_core
