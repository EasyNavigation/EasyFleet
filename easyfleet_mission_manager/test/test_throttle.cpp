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

#include <chrono>
#include <thread>

#include "easyfleet_mission_manager/throttle.hpp"

#include "gtest/gtest.h"

using easyfleet_mission_manager::Throttle;

TEST(ThrottleTest, FirstCallIsAlwaysReady)
{
  Throttle throttle(std::chrono::milliseconds(1000));
  EXPECT_TRUE(throttle.ready());
}

TEST(ThrottleTest, ImmediateSecondCallIsNotReady)
{
  Throttle throttle(std::chrono::milliseconds(1000));
  ASSERT_TRUE(throttle.ready());
  EXPECT_FALSE(throttle.ready());
}

TEST(ThrottleTest, ReadyAgainAfterIntervalElapses)
{
  Throttle throttle(std::chrono::milliseconds(50));
  ASSERT_TRUE(throttle.ready());
  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  EXPECT_TRUE(throttle.ready());
}

TEST(ThrottleTest, ZeroIntervalIsAlwaysReady)
{
  Throttle throttle(std::chrono::milliseconds(0));
  EXPECT_TRUE(throttle.ready());
  EXPECT_TRUE(throttle.ready());
  EXPECT_TRUE(throttle.ready());
}

TEST(ThrottleTest, CopiesShareTheUnderlyingClock)
{
  Throttle original(std::chrono::milliseconds(1000));
  ASSERT_TRUE(original.ready());

  Throttle copy = original;
  // The copy was taken after the "ready" call above, so it should still be
  // within the same throttling window as the original -- this is the
  // documented guarantee that lets a Throttle be captured by value into a
  // std::function and still throttle correctly.
  EXPECT_FALSE(copy.ready());
  EXPECT_FALSE(original.ready());
}

TEST(ThrottleTest, ReadyOnCopyAlsoResetsTheOriginal)
{
  Throttle original(std::chrono::milliseconds(50));
  ASSERT_TRUE(original.ready());
  std::this_thread::sleep_for(std::chrono::milliseconds(80));

  Throttle copy = original;
  ASSERT_TRUE(copy.ready());
  // The copy's successful ready() call should reset the shared clock, so
  // the original immediately observes it too.
  EXPECT_FALSE(original.ready());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
