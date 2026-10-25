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
#include <chrono>
#include <fstream>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include "easyfleet_core/init.hpp"
#include "easyfleet_mission_manager/fleet_session.hpp"
#include "easyfleet_mission_manager/mission_helpers.hpp"
#include "easyfleet_mission_manager/robot_handle.hpp"
#include "easyfleet_mission_manager/simple_controller.hpp"

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "test_nav_fake_capability.hpp"

using easyfleet_mission_manager::make_manipulation_goal;
using easyfleet_mission_manager::make_navigation_goal;
using easyfleet_mission_manager::Manipulation;
using easyfleet_mission_manager::Navigation;
using easyfleet_mission_manager_test::TestNavCapability;

namespace
{

std::string unique_test_name(const std::string & prefix)
{
  static std::atomic<uint64_t> counter{0};
  return prefix + "_" + std::to_string(::getpid()) + "_" + std::to_string(counter.fetch_add(1));
}

std::string make_capabilities_file(const std::string & content)
{
  static std::atomic<uint64_t> counter{0};
  const std::string path = "/tmp/easyfleet_mission_manager_test_capabilities_" +
    std::to_string(::getpid()) + "_" + std::to_string(counter.fetch_add(1)) + ".json";
  std::ofstream file(path);
  file << content;
  return path;
}

/// Brings up one TestNavCapability, namespaced under `robot_name`, spun on
/// its own executor/thread -- simulating a robot's own, independent
/// process the way discover_capabilities()/RobotHandle actually talk to
/// one over ROS, not a shortcut into the same process.
class FakeRobotProcess
{
public:
  explicit FakeRobotProcess(
    const std::string & robot_name, double mock_duration_s = 0.0, bool reject_cancel = false,
    bool start_active = true)
  {
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "-r", "__ns:=/" + robot_name});
    options.parameter_overrides(
      {rclcpp::Parameter("capabilities_file", make_capabilities_file(R"({"name":"navigation"})")),
        rclcpp::Parameter("navigation.mock_duration_seconds", mock_duration_s),
        rclcpp::Parameter("navigation.reject_cancel", reject_cancel)});
    capability_ = std::make_shared<TestNavCapability>("navigation", options);
    executor_.add_node(capability_->get_node_base_interface());
    spin_thread_ = std::thread([this] {executor_.spin();});
    while (!executor_.is_spinning()) {
      std::this_thread::yield();
    }
    capability_->configure_node();
    if (start_active) {
      capability_->activate_node();
    }
  }

  ~FakeRobotProcess()
  {
    executor_.cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
  }

  void activate() {capability_->activate_node();}
  void deactivate() {capability_->deactivate_node();}

  /// Simulates a hung process: no more heartbeats, no more answers.
  void freeze()
  {
    executor_.cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
  }

  int cancel_requests() const
  {
    const auto server =
      std::dynamic_pointer_cast<easyfleet_mission_manager_test::TestNavActionServer>(
      capability_->get_action_server());
    return server ? server->cancel_requests() : -1;
  }

private:
  std::shared_ptr<TestNavCapability> capability_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_thread_;
};

/// Spins `condition` up to `timeout`, sleeping briefly between checks.
bool wait_until(const std::function<bool()> & condition, std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (condition()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return condition();
}
}  // namespace

TEST(RobotHandleTest, NameReturnsWhatWasConstructedWith)
{
  easyfleet::RobotHandle robot(unique_test_name("robot"));
  EXPECT_FALSE(robot.name().empty());
}

TEST(RobotHandleTest, HasNoCapabilitiesBeforeAnyDiscovery)
{
  easyfleet::RobotHandle robot(unique_test_name("robot"));
  EXPECT_FALSE(robot.has_capability("navigation"));
  EXPECT_TRUE(robot.capabilities().empty());
}

TEST(RobotHandleTest, CapabilityStateIsIdleBeforeEverRunning)
{
  easyfleet::RobotHandle robot(unique_test_name("robot"));
  EXPECT_EQ(robot.capability_state("navigation"), easyfleet::CapabilityState::IDLE);
  EXPECT_FALSE(robot.is_capability_running("navigation"));
}

TEST(RobotHandleTest, IsAliveIsFalseWithoutAnyHeartbeatSeen)
{
  easyfleet::RobotHandle robot(unique_test_name("robot"));
  EXPECT_FALSE(robot.is_alive("navigation"));
}

TEST(RobotHandleTest, RunCapabilityOnAHandleNeverAddedToAFleetSessionThrows)
{
  // A RobotHandle that was never passed to FleetSession::add_robot() (or
  // SimpleController::add_robot()) has no session_ to reach a node/executor
  // through -- this must fail loudly, not silently no-op, since it's
  // otherwise indistinguishable from a properly attached robot that simply
  // hasn't announced this capability yet.
  easyfleet::RobotHandle robot(unique_test_name("robot"));
  EXPECT_THROW(
    robot.run_capability<Navigation>("navigation", make_navigation_goal("kitchen")),
    std::logic_error);
}

TEST(FleetSessionTest, FindRobotReturnsNulloptBeforeAdding)
{
  easyfleet::FleetSession session;
  EXPECT_FALSE(session.find_robot("robot_1"));
  EXPECT_TRUE(session.robots().empty());
}

TEST(FleetSessionTest, AddRobotMakesItFindableAndAttached)
{
  easyfleet::FleetSession session;
  easyfleet::RobotHandle robot(unique_test_name("robot"));
  session.add_robot(robot);

  ASSERT_EQ(session.robots().size(), 1u);
  auto found = session.find_robot(robot.name());
  ASSERT_TRUE(found);
  EXPECT_EQ(&found->get(), &robot);
}

TEST(FleetSessionRobotHandleIntegrationTest, DiscoverRunAndSucceed)
{
  const auto robot_name = unique_test_name("robot");
  FakeRobotProcess fake_robot(robot_name, /*mock_duration_s=*/ 0.05);

  easyfleet::FleetSession session;
  easyfleet::RobotHandle robot(robot_name);
  session.add_robot(robot);

  session.discover_capabilities();
  ASSERT_TRUE(robot.has_capability("navigation"));

  robot.run_capability<Navigation>(
    "navigation", make_navigation_goal("kitchen"), std::chrono::seconds(5));
  EXPECT_TRUE(robot.is_capability_running("navigation"));

  ASSERT_TRUE(
    wait_until(
      [&] {return !robot.is_capability_running("navigation");},
      std::chrono::milliseconds(3000)));
  EXPECT_EQ(robot.capability_state("navigation"), easyfleet::CapabilityState::SUCCEEDED);
}

TEST(FleetSessionRobotHandleIntegrationTest, RunCapabilityWithAMismatchedActionTypeThrows)
{
  // A capability_type means the same ActionT for the life of a RobotHandle,
  // fixed by the first run_capability<ActionT>() call for it -- a later
  // call for the same capability_type with a *different* ActionT is a
  // caller bug (not, say, a legitimately different backend), so it must be
  // rejected loudly rather than silently sending a goal of the wrong type.
  const auto robot_name = unique_test_name("robot");
  FakeRobotProcess fake_robot(robot_name, /*mock_duration_s=*/ 30.0);

  easyfleet::FleetSession session;
  easyfleet::RobotHandle robot(robot_name);
  session.add_robot(robot);
  session.discover_capabilities();
  ASSERT_TRUE(robot.has_capability("navigation"));

  robot.run_capability<Navigation>(
    "navigation", make_navigation_goal("kitchen"), std::chrono::seconds(30));
  ASSERT_TRUE(
    wait_until(
      [&] {return robot.is_capability_running("navigation");}, std::chrono::milliseconds(500)));

  EXPECT_THROW(
    robot.run_capability<Manipulation>(
      "navigation", make_manipulation_goal(), std::chrono::seconds(5)),
    std::logic_error);

  robot.stop_capability("navigation");
}

TEST(FleetSessionRobotHandleIntegrationTest, StopCapabilityCancelsALongRunningGoal)
{
  const auto robot_name = unique_test_name("robot");
  FakeRobotProcess fake_robot(robot_name, /*mock_duration_s=*/ 30.0);

  easyfleet::FleetSession session;
  easyfleet::RobotHandle robot(robot_name);
  session.add_robot(robot);
  session.discover_capabilities();
  ASSERT_TRUE(robot.has_capability("navigation"));

  robot.run_capability<Navigation>(
    "navigation", make_navigation_goal("kitchen"), std::chrono::seconds(60));
  ASSERT_TRUE(
    wait_until(
      [&] {return robot.is_capability_running("navigation");}, std::chrono::milliseconds(500)));

  robot.stop_capability("navigation");

  ASSERT_TRUE(
    wait_until(
      [&] {return !robot.is_capability_running("navigation");},
      std::chrono::milliseconds(3000)));
  EXPECT_EQ(robot.capability_state("navigation"), easyfleet::CapabilityState::CANCELED);
}

TEST(FleetSessionRobotHandleIntegrationTest, RunCapabilityTimeoutStopsItAutomatically)
{
  const auto robot_name = unique_test_name("robot");
  FakeRobotProcess fake_robot(robot_name, /*mock_duration_s=*/ 30.0);

  easyfleet::FleetSession session;
  easyfleet::RobotHandle robot(robot_name);
  session.add_robot(robot);
  session.discover_capabilities();
  ASSERT_TRUE(robot.has_capability("navigation"));

  // A short client-side timeout, well under the goal's own (much longer)
  // mock duration: only spin_some()'s periodic check_timeout() should be
  // what stops it.
  robot.run_capability<Navigation>(
    "navigation", make_navigation_goal("kitchen"), std::chrono::seconds(1));

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(3000);
  while (robot.is_capability_running("navigation") &&
    std::chrono::steady_clock::now() < deadline)
  {
    session.spin_some();
  }

  EXPECT_EQ(robot.capability_state("navigation"), easyfleet::CapabilityState::CANCELED);
}

TEST(FleetSessionRobotHandleIntegrationTest, RejectedGoalReportsRejectedSpecifically)
{
  // CapabilityState reports the real, specific outcome -- a goal the
  // server itself rejects must read as REJECTED, not a generic failure
  // bucket, and not CANCELED (which is reserved for a mission
  // script/timeout deliberately stopping an in-flight goal).
  const auto robot_name = unique_test_name("robot");
  FakeRobotProcess fake_robot(robot_name);

  easyfleet::FleetSession session;
  easyfleet::RobotHandle robot(robot_name);
  session.add_robot(robot);
  session.discover_capabilities();
  ASSERT_TRUE(robot.has_capability("navigation"));

  robot.run_capability<Navigation>(
    "navigation",
    make_navigation_goal(easyfleet_mission_manager_test::kRejectWaypointId),
    std::chrono::seconds(5));

  ASSERT_TRUE(
    wait_until(
      [&] {return !robot.is_capability_running("navigation");},
      std::chrono::milliseconds(3000)));
  EXPECT_EQ(robot.capability_state("navigation"), easyfleet::CapabilityState::REJECTED);
}

TEST(FleetSessionRobotHandleIntegrationTest, PreemptingGoalWinsOverALateArrivingStaleOutcome)
{
  // Regression test: a second run_capability() call while the first goal
  // is still in flight preempts it server-side (ActionServerBase's own
  // preemption handling). The *first* goal's own client-side outcome
  // callback can still arrive after the second goal has already settled --
  // without generation tracking in RunningCapability, that late callback
  // used to stomp capability_state() with the superseded goal's outcome
  // (observed live as spurious FAILED results for a robot whose goal had
  // actually been preempted and redirected successfully).
  // TestNavActionServer's mock duration is fixed per-node, not per-goal --
  // both the preempted and the preempting goal below take this long, so
  // this only needs to be short enough to keep the test fast, not "first
  // long, second short".
  const auto robot_name = unique_test_name("robot");
  FakeRobotProcess fake_robot(robot_name, /*mock_duration_s=*/ 1.0);

  easyfleet::FleetSession session;
  easyfleet::RobotHandle robot(robot_name);
  session.add_robot(robot);
  session.discover_capabilities();
  ASSERT_TRUE(robot.has_capability("navigation"));

  robot.run_capability<Navigation>(
    "navigation", make_navigation_goal("kitchen"), std::chrono::seconds(30));
  ASSERT_TRUE(
    wait_until(
      [&] {return robot.is_capability_running("navigation");}, std::chrono::milliseconds(500)));

  // Preempt partway through the first goal's ~1s mock duration -- the
  // first goal's own eventual (ABORTED, from being preempted) outcome
  // arrives strictly after this point.
  robot.run_capability<Navigation>(
    "navigation", make_navigation_goal("charging_station"), std::chrono::seconds(30));

  ASSERT_TRUE(
    wait_until(
      [&] {return !robot.is_capability_running("navigation");},
      std::chrono::milliseconds(4000)));
  EXPECT_EQ(robot.capability_state("navigation"), easyfleet::CapabilityState::SUCCEEDED);

  // The first goal's own late-arriving ABORTED outcome, if it hadn't been
  // filtered out, would land some time after the preempting goal already
  // succeeded -- give it a window to (not) do so.
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  EXPECT_EQ(robot.capability_state("navigation"), easyfleet::CapabilityState::SUCCEEDED);
}

TEST(FleetSessionRobotHandleIntegrationTest, IsAliveTrueAfterDiscoveryOfAnActiveCapability)
{
  const auto robot_name = unique_test_name("robot");
  FakeRobotProcess fake_robot(robot_name);

  easyfleet::FleetSession session;
  easyfleet::RobotHandle robot(robot_name);
  session.add_robot(robot);
  session.discover_capabilities();

  // The heartbeat subscription is separate from (and persists past)
  // discovery -- give it a moment to see at least the first 1 Hz beat.
  ASSERT_TRUE(
    wait_until([&] {return robot.is_alive("navigation");}, std::chrono::milliseconds(2000)));
}

TEST(FleetSessionLiveRegistryTest, CapabilityThatAppearsAfterDiscoveryIsSeenWithoutRediscovering)
{
  const auto robot_name = unique_test_name("robot");
  easyfleet::FleetSession session;
  easyfleet::RobotHandle robot(robot_name);
  session.add_robot(robot);
  session.discover_capabilities(std::chrono::milliseconds(500));
  EXPECT_FALSE(robot.has_capability("navigation"));

  FakeRobotProcess fake_robot(robot_name);
  EXPECT_TRUE(
    wait_until(
      [&] {return robot.has_capability("navigation");}, std::chrono::milliseconds(4000)));
  EXPECT_EQ(robot.capabilities().size(), 1u);
  EXPECT_TRUE(robot.capabilities()[0].description_json_valid);
}

TEST(FleetSessionLiveRegistryTest, DeactivatedCapabilityStopsBeingAvailable)
{
  const auto robot_name = unique_test_name("robot");
  FakeRobotProcess fake_robot(robot_name);
  easyfleet::FleetSession session;
  easyfleet::RobotHandle robot(robot_name);
  session.add_robot(robot);
  session.discover_capabilities();
  ASSERT_TRUE(robot.has_capability("navigation"));
  ASSERT_TRUE(robot.is_alive("navigation"));

  fake_robot.deactivate();
  EXPECT_TRUE(
    wait_until(
      [&] {return !robot.has_capability("navigation");}, std::chrono::milliseconds(5000)));
  EXPECT_FALSE(robot.is_alive("navigation"));
  // Still known (its description is retained), just not active.
  ASSERT_EQ(robot.capabilities().size(), 1u);
  EXPECT_FALSE(robot.capabilities()[0].active);

  fake_robot.activate();
  EXPECT_TRUE(
    wait_until(
      [&] {return robot.has_capability("navigation");}, std::chrono::milliseconds(4000)));
}

TEST(FleetSessionLiveRegistryTest, InactiveCapabilityIsNotRun)
{
  const auto robot_name = unique_test_name("robot");
  FakeRobotProcess fake_robot(robot_name, 0.0, false, /*start_active=*/ false);
  easyfleet::FleetSession session;
  easyfleet::RobotHandle robot(robot_name);
  session.add_robot(robot);
  session.discover_capabilities(std::chrono::milliseconds(1500));
  EXPECT_FALSE(robot.has_capability("navigation"));

  robot.run_capability<Navigation>("navigation", make_navigation_goal("kitchen"));
  EXPECT_EQ(robot.capability_state("navigation"), easyfleet::CapabilityState::IDLE);
}

TEST(FleetSessionLiveRegistryTest, CapabilitiesOfOtherRobotsAreNotMixedIn)
{
  const auto robot_a = unique_test_name("robot_a");
  const auto robot_b = unique_test_name("robot_b");
  FakeRobotProcess fake_a(robot_a);
  FakeRobotProcess fake_b(robot_b);
  easyfleet::FleetSession session;
  easyfleet::RobotHandle handle_a(robot_a);
  session.add_robot(handle_a);
  session.discover_capabilities();
  ASSERT_EQ(handle_a.capabilities().size(), 1u);
  EXPECT_EQ(handle_a.capabilities()[0].robot, robot_a);
  EXPECT_EQ(session.capabilities_of(robot_b).size(), 1u);
}

TEST(RunningCapabilityLivenessTest, GoalOnACapabilityThatHangsEndsUnreachable)
{
  const auto robot_name = unique_test_name("robot");
  FakeRobotProcess fake_robot(robot_name, /*mock_duration_s=*/ 30.0);
  easyfleet::FleetSession session;
  easyfleet::RobotHandle robot(robot_name);
  session.add_robot(robot);
  session.discover_capabilities();
  ASSERT_TRUE(robot.has_capability("navigation"));

  robot.run_capability<Navigation>(
    "navigation", make_navigation_goal("kitchen"), std::chrono::seconds(60));
  session.spin_for(std::chrono::milliseconds(300));
  ASSERT_TRUE(robot.is_capability_running("navigation"));

  fake_robot.freeze();
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
  while (robot.is_capability_running("navigation") &&
    std::chrono::steady_clock::now() < deadline)
  {
    session.spin_some();
  }
  EXPECT_EQ(robot.capability_state("navigation"), easyfleet::CapabilityState::UNREACHABLE);
}

TEST(RunningCapabilityLivenessTest, TimeoutCancelsOnlyOnceAndEndsTimeoutIfIgnored)
{
  const auto robot_name = unique_test_name("robot");
  FakeRobotProcess fake_robot(
    robot_name, /*mock_duration_s=*/ 30.0, /*reject_cancel=*/ true);
  easyfleet::FleetSession session;
  easyfleet::RobotHandle robot(robot_name);
  session.add_robot(robot);
  session.discover_capabilities();
  ASSERT_TRUE(robot.has_capability("navigation"));

  robot.run_capability<Navigation>(
    "navigation", make_navigation_goal("kitchen"), std::chrono::seconds(1));
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (robot.is_capability_running("navigation") &&
    std::chrono::steady_clock::now() < deadline)
  {
    session.spin_some();
  }
  // The capability is alive but ignored the cancel: given up after the grace period.
  EXPECT_EQ(robot.capability_state("navigation"), easyfleet::CapabilityState::TIMEOUT);
  EXPECT_EQ(fake_robot.cancel_requests(), 1) << "the cancel must not be repeated";
}

TEST(RunningCapabilityLivenessTest, StopOnAGoalThatIgnoresCancelEndsTimeout)
{
  const auto robot_name = unique_test_name("robot");
  FakeRobotProcess fake_robot(
    robot_name, /*mock_duration_s=*/ 30.0, /*reject_cancel=*/ true);
  easyfleet::FleetSession session;
  easyfleet::RobotHandle robot(robot_name);
  session.add_robot(robot);
  session.discover_capabilities();

  robot.run_capability<Navigation>(
    "navigation", make_navigation_goal("kitchen"), std::chrono::seconds(60));
  session.spin_for(std::chrono::milliseconds(300));
  robot.stop_capability("navigation");
  robot.stop_capability("navigation");
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (robot.is_capability_running("navigation") &&
    std::chrono::steady_clock::now() < deadline)
  {
    session.spin_some();
  }
  EXPECT_EQ(robot.capability_state("navigation"), easyfleet::CapabilityState::TIMEOUT);
  EXPECT_EQ(fake_robot.cancel_requests(), 1);
}

TEST(RunningCapabilityLivenessTest, StopWhenIdleSendsNoCancel)
{
  const auto robot_name = unique_test_name("robot");
  FakeRobotProcess fake_robot(robot_name);
  easyfleet::FleetSession session;
  easyfleet::RobotHandle robot(robot_name);
  session.add_robot(robot);
  session.discover_capabilities();

  robot.run_capability<Navigation>("navigation", make_navigation_goal("kitchen"));
  ASSERT_TRUE(
    wait_until(
      [&] {return !robot.is_capability_running("navigation");},
      std::chrono::milliseconds(3000)));
  robot.stop_capability("navigation");
  session.spin_for(std::chrono::milliseconds(300));
  EXPECT_EQ(fake_robot.cancel_requests(), 0);
  EXPECT_EQ(robot.capability_state("navigation"), easyfleet::CapabilityState::SUCCEEDED);
}

TEST(RunningCapabilityLivenessTest, StopThenRunImmediatelyReportsTheNewGoal)
{
  const auto robot_name = unique_test_name("robot");
  FakeRobotProcess fake_robot(robot_name, /*mock_duration_s=*/ 30.0);
  easyfleet::FleetSession session;
  easyfleet::RobotHandle robot(robot_name);
  session.add_robot(robot);
  session.discover_capabilities();

  robot.run_capability<Navigation>(
    "navigation", make_navigation_goal("kitchen"), std::chrono::seconds(60));
  session.spin_for(std::chrono::milliseconds(300));
  robot.stop_capability("navigation");
  robot.run_capability<Navigation>(
    "navigation", make_navigation_goal("hall"), std::chrono::seconds(60));
  // The first goal's CANCELED arrives meanwhile and must be ignored.
  session.spin_for(std::chrono::milliseconds(1500));
  EXPECT_EQ(robot.capability_state("navigation"), easyfleet::CapabilityState::RUNNING);
}

TEST(FleetSessionLifetimeTest, AddingTheSameRobotTwiceThrows)
{
  easyfleet::FleetSession session;
  easyfleet::RobotHandle robot(unique_test_name("robot"));
  session.add_robot(robot);
  EXPECT_THROW(session.add_robot(robot), std::logic_error);
  EXPECT_EQ(session.robots().size(), 1u);
}

TEST(FleetSessionLifetimeTest, RobotDestroyedBeforeTheSessionLeavesIt)
{
  const auto robot_name = unique_test_name("robot");
  FakeRobotProcess fake_robot(robot_name, /*mock_duration_s=*/ 30.0);
  easyfleet::FleetSession session;
  {
    easyfleet::RobotHandle robot(robot_name);
    session.add_robot(robot);
    session.discover_capabilities();
    robot.run_capability<Navigation>(
      "navigation", make_navigation_goal("kitchen"), std::chrono::seconds(60));
    session.spin_for(std::chrono::milliseconds(200));
  }
  EXPECT_TRUE(session.robots().empty());
  EXPECT_FALSE(session.find_robot(robot_name));
  session.spin_for(std::chrono::milliseconds(200));  // must not touch the dead handle
}

TEST(FleetSessionLifetimeTest, SessionDestroyedBeforeTheRobotDetachesIt)
{
  const auto robot_name = unique_test_name("robot");
  FakeRobotProcess fake_robot(robot_name, /*mock_duration_s=*/ 30.0);
  easyfleet::RobotHandle robot(robot_name);
  {
    easyfleet::FleetSession session;
    session.add_robot(robot);
    session.discover_capabilities();
    robot.run_capability<Navigation>(
      "navigation", make_navigation_goal("kitchen"), std::chrono::seconds(60));
    session.spin_for(std::chrono::milliseconds(200));
  }
  EXPECT_TRUE(robot.capabilities().empty());
  EXPECT_FALSE(robot.has_capability("navigation"));
  EXPECT_FALSE(robot.is_alive("navigation"));
  EXPECT_EQ(robot.capability_state("navigation"), easyfleet::CapabilityState::IDLE);
  EXPECT_THROW(
    robot.run_capability<Navigation>("navigation", make_navigation_goal("kitchen")),
    std::logic_error);

  // And it can join a new session.
  easyfleet::FleetSession other;
  other.add_robot(robot);
  other.discover_capabilities();
  EXPECT_TRUE(robot.has_capability("navigation"));
}

TEST(SimpleControllerTest, ForwardsToAWorkingFleetSession)
{
  const auto robot_name = unique_test_name("robot");
  FakeRobotProcess fake_robot(robot_name, /*mock_duration_s=*/ 0.05);

  easyfleet::SimpleController controller;
  easyfleet::RobotHandle robot(robot_name);
  controller.add_robot(robot);

  ASSERT_EQ(controller.robots().size(), 1u);
  ASSERT_TRUE(controller.find_robot(robot_name));

  controller.discover_capabilities();
  ASSERT_TRUE(robot.has_capability("navigation"));

  robot.run_capability<Navigation>(
    "navigation", make_navigation_goal("kitchen"), std::chrono::seconds(5));
  controller.spin_for(std::chrono::milliseconds(500));

  ASSERT_TRUE(
    wait_until(
      [&] {return !robot.is_capability_running("navigation");},
      std::chrono::milliseconds(3000)));
  EXPECT_EQ(robot.capability_state("navigation"), easyfleet::CapabilityState::SUCCEEDED);
  EXPECT_NE(controller.node(), nullptr);
}

int main(int argc, char ** argv)
{
  easyfleet::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
