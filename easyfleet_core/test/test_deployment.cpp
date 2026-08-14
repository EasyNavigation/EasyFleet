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

#include <atomic>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include "easyfleet_core/deployment.hpp"

#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/state.hpp"
#include "pluginlib/exceptions.hpp"
#include "rclcpp/rclcpp.hpp"

// easyfleet_core has no *real* capability backend of its own (the fake/real
// ones all live in downstream deployment packages), but it does register a
// test-only CapabilityFactory plugin (test_capability_factory_plugin.cpp /
// test_plugins.xml, lookup names "test_capability/fibonacci" and
// "test_capability_two/fibonacci") exclusively for this test binary, so the
// success path -- add_capability() actually loading something,
// start() actually configuring/activating it, add_capabilities_from_parameters()
// actually reading ROS parameters -- can be exercised here directly, not
// just the "unknown plugin" failure path. Deployment::run() (blocks on
// SIGINT/SIGTERM) and Deployment::shutdown() (calls rclcpp::shutdown(),
// which would break every test running after it in this same process) are
// intentionally not exercised here -- see test_deployment_shutdown.cpp for
// shutdown(), in its own isolated process.

namespace
{

std::vector<std::string> g_temp_files;

std::string make_capabilities_file(const std::string & content)
{
  static std::atomic<uint64_t> counter{0};
  const std::string path = "/tmp/easyfleet_core_test_deployment_capabilities_" +
    std::to_string(::getpid()) + "_" + std::to_string(counter.fetch_add(1)) + ".json";
  std::ofstream file(path);
  file << content;
  g_temp_files.push_back(path);
  return path;
}

rclcpp::NodeOptions options_with_capabilities_file(const std::string & content)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {rclcpp::Parameter("capabilities_file", make_capabilities_file(content))});
  return options;
}

}  // namespace

TEST(DeploymentTest, NameReturnsWhatWasConstructedWith)
{
  easyfleet::Deployment deployment("robot_1");
  EXPECT_EQ(deployment.name(), "robot_1");
}

TEST(DeploymentTest, StartsWithNoCapabilities)
{
  easyfleet::Deployment deployment("robot_1");
  EXPECT_TRUE(deployment.capabilities().empty());
}

TEST(DeploymentTest, AddCapabilityWithUnknownPluginThrows)
{
  easyfleet::Deployment deployment("robot_1");
  EXPECT_THROW(
    deployment.add_capability("nonexistent_type/nonexistent_plugin"),
    pluginlib::PluginlibException);
}

TEST(DeploymentTest, AddCapabilityWithKnownPluginSucceeds)
{
  easyfleet::Deployment deployment("robot_1");
  deployment.add_capability("test_capability/fibonacci");
  ASSERT_EQ(deployment.capabilities().size(), 1u);
  EXPECT_EQ(deployment.capabilities()[0]->get_capability_name(), "test_capability");
}

TEST(DeploymentTest, CapabilityNameIsDerivedFromSubstringBeforeFirstSlash)
{
  easyfleet::Deployment deployment("robot_1");
  deployment.add_capability("test_capability_two/fibonacci");
  ASSERT_EQ(deployment.capabilities().size(), 1u);
  EXPECT_EQ(deployment.capabilities()[0]->get_capability_name(), "test_capability_two");
}

TEST(DeploymentTest, AddCapabilityAcceptsALookupNameWithNoSlash)
{
  // Not a registered plugin name, but exercises the "no '/' at all" branch
  // of the capability-name derivation before pluginlib rejects it -- the
  // whole string becomes the (never-used, since loading fails) capability
  // name in that branch.
  easyfleet::Deployment deployment("robot_1");
  EXPECT_THROW(
    deployment.add_capability("no_slash_at_all"),
    pluginlib::PluginlibException);
}

TEST(DeploymentTest, AddingMultipleCapabilitiesAccumulatesInOrder)
{
  easyfleet::Deployment deployment("robot_1");
  deployment.add_capability("test_capability/fibonacci");
  deployment.add_capability("test_capability_two/fibonacci");
  ASSERT_EQ(deployment.capabilities().size(), 2u);
  EXPECT_EQ(deployment.capabilities()[0]->get_capability_name(), "test_capability");
  EXPECT_EQ(deployment.capabilities()[1]->get_capability_name(), "test_capability_two");
}

TEST(DeploymentTest, StartConfiguresAndActivatesEveryCapability)
{
  easyfleet::Deployment deployment("robot_1");
  deployment.add_capability(
    "test_capability/fibonacci", options_with_capabilities_file(R"({"name":"test"})"));
  deployment.add_capability(
    "test_capability_two/fibonacci", options_with_capabilities_file(R"({"name":"test2"})"));

  deployment.start();

  for (const auto & capability : deployment.capabilities()) {
    EXPECT_EQ(
      capability->get_current_state_id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  }
}

TEST(DeploymentTest, AddCapabilitiesFromParametersLoadsEveryConfiguredCapability)
{
  // Deployment::add_capabilities_from_parameters() reads its "capabilities"/
  // "config_subdir" values from this *process's* global parameter
  // overrides (see main() below) -- there's no other way to feed them into
  // Deployment's internal, lazily-created config node.
  easyfleet::Deployment deployment("robot_1");
  deployment.add_capabilities_from_parameters("easyfleet_core");

  ASSERT_EQ(deployment.capabilities().size(), 2u);
  EXPECT_EQ(deployment.capabilities()[0]->get_capability_name(), "test_capability");
  EXPECT_EQ(deployment.capabilities()[1]->get_capability_name(), "test_capability_two");
}

int main(int argc, char ** argv)
{
  // Synthesizes the --ros-args this binary is invoked with, rather than
  // forwarding ctest's own argv, specifically so
  // AddCapabilitiesFromParametersLoadsEveryConfiguredCapability has
  // "capabilities"/"config_subdir" global parameter overrides to read --
  // Deployment::config_node() always uses a plain, default-options
  // rclcpp::Node, which only ever sees parameters set this way. Harmless
  // to every other test in this binary: nothing else declares parameters
  // under these names.
  (void)argc;
  (void)argv;
  std::vector<const char *> fake_argv{
    "test_deployment",
    "--ros-args",
    "-p", "capabilities:=[test_capability/fibonacci, test_capability_two/fibonacci]",
    "-p", "config_subdir:=test",
  };
  int fake_argc = static_cast<int>(fake_argv.size());
  rclcpp::init(fake_argc, fake_argv.data());
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  for (const auto & path : g_temp_files) {
    std::remove(path.c_str());
  }
  return result;
}
