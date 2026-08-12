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

#include "easyfleet_core/detail/param_utils.hpp"

using easyfleet_core::detail::sanitize_identifier;
using easyfleet_core::detail::sanitize_parameter_name;

TEST(SanitizeParameterName, StripsLeadingSlash)
{
  EXPECT_EQ(sanitize_parameter_name("/fibonacci"), "fibonacci");
}

TEST(SanitizeParameterName, ReplacesInnerSlashesWithDots)
{
  EXPECT_EQ(sanitize_parameter_name("/robot/follow_path"), "robot.follow_path");
}

TEST(SanitizeParameterName, LeavesPlainNameUnchanged)
{
  EXPECT_EQ(sanitize_parameter_name("fibonacci"), "fibonacci");
}

TEST(SanitizeParameterName, EmptyNameFallsBackToDefault)
{
  EXPECT_EQ(sanitize_parameter_name(""), "action");
  EXPECT_EQ(sanitize_parameter_name("/"), "action");
}

TEST(SanitizeIdentifier, ReplacesInvalidCharactersWithUnderscore)
{
  EXPECT_EQ(sanitize_identifier("robot/follow_path"), "robot_follow_path");
  EXPECT_EQ(sanitize_identifier("a.b-c"), "a_b_c");
}

TEST(SanitizeIdentifier, PrefixesLeadingDigit)
{
  EXPECT_EQ(sanitize_identifier("1abc"), "_1abc");
}

TEST(SanitizeIdentifier, HandlesEmptyString)
{
  EXPECT_EQ(sanitize_identifier(""), "_");
}
