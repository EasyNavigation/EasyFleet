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

#include <chrono>

#include "easyfleet_core/link_watchdog.hpp"
#include "easyfleet_interfaces/msg/capability_status.hpp"

namespace
{

using namespace std::chrono_literals;
using easyfleet_core::LinkWatchdog;
using easyfleet_interfaces::msg::CapabilityStatus;

const auto kT0 = LinkWatchdog::Clock::now();

TEST(LinkWatchdog, ZeroOrNegativeTimeoutIsNotMonitored)
{
  for (const double timeout : {0.0, -1.0}) {
    LinkWatchdog watchdog(timeout);
    EXPECT_FALSE(watchdog.monitored());
    EXPECT_EQ(watchdog.state(), CapabilityStatus::LINK_NOT_MONITORED);
    watchdog.heard(kT0);
    EXPECT_EQ(watchdog.update(kT0 + 1h), CapabilityStatus::LINK_NOT_MONITORED);
  }
}

TEST(LinkWatchdog, WaitsForTheFirstHeartbeatHoweverLong)
{
  LinkWatchdog watchdog(1.0);
  EXPECT_TRUE(watchdog.monitored());
  EXPECT_EQ(watchdog.state(), CapabilityStatus::LINK_WAITING);
  EXPECT_EQ(watchdog.update(kT0 + 1h), CapabilityStatus::LINK_WAITING);
}

TEST(LinkWatchdog, OkWhileHeardAndLostAfterTheTimeout)
{
  LinkWatchdog watchdog(2.0);
  watchdog.heard(kT0);
  EXPECT_EQ(watchdog.state(), CapabilityStatus::LINK_OK);
  EXPECT_EQ(watchdog.update(kT0 + 1s), CapabilityStatus::LINK_OK);
  EXPECT_EQ(watchdog.update(kT0 + 2s), CapabilityStatus::LINK_OK) << "exactly the timeout";
  EXPECT_EQ(watchdog.update(kT0 + 2s + 1ms), CapabilityStatus::LINK_LOST);
  EXPECT_EQ(watchdog.state(), CapabilityStatus::LINK_LOST);
}

TEST(LinkWatchdog, RecoversWithTheNextHeartbeat)
{
  LinkWatchdog watchdog(0.5);
  watchdog.heard(kT0);
  EXPECT_EQ(watchdog.update(kT0 + 1s), CapabilityStatus::LINK_LOST);
  watchdog.heard(kT0 + 2s);
  EXPECT_EQ(watchdog.state(), CapabilityStatus::LINK_OK);
  EXPECT_EQ(watchdog.update(kT0 + 2s + 400ms), CapabilityStatus::LINK_OK);
  EXPECT_EQ(watchdog.update(kT0 + 3s), CapabilityStatus::LINK_LOST);
}

TEST(LinkWatchdog, AnOlderHeartbeatDoesNotMoveTheLastOneBack)
{
  LinkWatchdog watchdog(1.0);
  watchdog.heard(kT0 + 5s);
  watchdog.heard(kT0);
  EXPECT_EQ(watchdog.update(kT0 + 5s + 500ms), CapabilityStatus::LINK_OK);
}

TEST(LinkWatchdog, ResetForgetsWhatWasHeard)
{
  LinkWatchdog watchdog(1.0);
  watchdog.heard(kT0);
  watchdog.reset(3.0);
  EXPECT_EQ(watchdog.state(), CapabilityStatus::LINK_WAITING);
  EXPECT_EQ(watchdog.update(kT0 + 10s), CapabilityStatus::LINK_WAITING);
  watchdog.reset(0.0);
  EXPECT_FALSE(watchdog.monitored());
  EXPECT_EQ(watchdog.state(), CapabilityStatus::LINK_NOT_MONITORED);
}

TEST(LinkWatchdog, SubSecondTimeouts)
{
  LinkWatchdog watchdog(0.05);
  watchdog.heard(kT0);
  EXPECT_EQ(watchdog.update(kT0 + 40ms), CapabilityStatus::LINK_OK);
  EXPECT_EQ(watchdog.update(kT0 + 60ms), CapabilityStatus::LINK_LOST);
}

}  // namespace
