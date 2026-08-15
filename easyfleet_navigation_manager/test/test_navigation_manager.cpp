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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "easyfleet_interfaces/msg/capability_status.hpp"
#include "easyfleet_navigation_manager/costmap_map_publisher.hpp"
#include "easyfleet_navigation_manager/navigation_manager_node.hpp"
#include "easyfleet_navigation_manager/routes_publisher.hpp"
#include "easynav_interfaces/msg/navigation_control.hpp"
#include "easynav_routes_maps_manager/msg/routes_map.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

// These tests deliberately load the exact same real map/routes files
// every EasyFleet real-EasyNav deployment already points its own
// per-robot maps_manager_node at (see
// easyfleet_easynav_deployment/config/easynav_robot/easynav_system.costmap_rpp.params.yaml):
// package "easynav_indoor_testcase", "maps/home2.yaml"/"maps/routes_1.yaml".
// That both keeps these tests fixture-free (no need to write a
// temporary map/PGM/YAML to disk just to have something loadable) and
// exercises the exact real-world path this package is meant for.

namespace
{
rclcpp::NodeOptions make_options(const std::vector<rclcpp::Parameter> & params)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(params);
  return options;
}

// A straight-line path from (x0, y0) to (x1, y1), sampled every ~step_m --
// mirrors conflict_detector_tests.cpp's own helper (kept separate since
// these are independent test binaries).
nav_msgs::msg::Path make_straight_path(
  double x0, double y0, double x1, double y1, double step_m = 0.2)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";

  const double dx = x1 - x0;
  const double dy = y1 - y0;
  const double length = std::hypot(dx, dy);
  const int n = std::max(1, static_cast<int>(length / step_m));

  for (int i = 0; i <= n; ++i) {
    const double t = static_cast<double>(i) / n;
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = x0 + t * dx;
    pose.pose.position.y = y0 + t * dy;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  return path;
}

// Minimal fake GoalManager: replies to PAUSE/RESUME with PAUSED/RESUMED,
// addressed back to whoever sent the request -- exactly the part of the
// real GoalManager::control_callback() protocol RobotNavigationWatcher
// itself speaks (see GoalManager.cpp's PAUSE/RESUME cases).
class FakeGoalManagerServer
{
public:
  explicit FakeGoalManagerServer(const std::string & robot_ns)
  {
    rclcpp::NodeOptions options;
    options.use_global_arguments(false);
    node_ = std::make_shared<rclcpp::Node>("fake_gm_server", "/" + robot_ns, options);

    pub_ = node_->create_publisher<easynav_interfaces::msg::NavigationControl>(
      "easynav_control", 100);
    sub_ = node_->create_subscription<easynav_interfaces::msg::NavigationControl>(
      "easynav_control", 100,
      [this](easynav_interfaces::msg::NavigationControl::UniquePtr msg) {
        if (msg->user_id == "fake_gm_server") {return;}

        easynav_interfaces::msg::NavigationControl response;
        response.user_id = "fake_gm_server";
        response.nav_current_user_id = msg->user_id;

        if (msg->type == easynav_interfaces::msg::NavigationControl::PAUSE) {
          paused_ = true;
          response.type = easynav_interfaces::msg::NavigationControl::PAUSED;
          pub_->publish(response);
        } else if (msg->type == easynav_interfaces::msg::NavigationControl::RESUME) {
          paused_ = false;
          response.type = easynav_interfaces::msg::NavigationControl::RESUMED;
          pub_->publish(response);
        }
      });
  }

  [[nodiscard]] rclcpp::Node::SharedPtr get_node() const {return node_;}
  [[nodiscard]] bool paused() const {return paused_;}

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<easynav_interfaces::msg::NavigationControl>::SharedPtr pub_;
  rclcpp::Subscription<easynav_interfaces::msg::NavigationControl>::SharedPtr sub_;
  bool paused_ {false};
};

}  // namespace

class NavigationManagerTest : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }
};

TEST_F(NavigationManagerTest, CostmapMapPublisherPublishesTheConfiguredMap)
{
  auto node = rclcpp::Node::make_shared(
    "test_costmap_map_publisher", make_options(
  {
    rclcpp::Parameter("map.package", std::string("easynav_indoor_testcase")),
    rclcpp::Parameter("map.map_path_file", std::string("maps/home2.yaml")),
  }));

  easyfleet::CostmapMapPublisher publisher;
  ASSERT_NO_THROW(publisher.publish(*node));

  nav_msgs::msg::OccupancyGrid::SharedPtr received;
  auto sub = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/global_map", rclcpp::QoS(1).transient_local().reliable(),
    [&received](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {received = msg;});

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  executor.spin_some();

  ASSERT_TRUE(received != nullptr);
  EXPECT_EQ(received->header.frame_id, "map");
  EXPECT_GT(received->info.width, 0u);
  EXPECT_GT(received->info.height, 0u);
  EXPECT_NEAR(received->info.resolution, 0.05f, 1e-6f);
}

TEST_F(NavigationManagerTest, CostmapMapPublisherThrowsWhenUnconfigured)
{
  auto node = rclcpp::Node::make_shared("test_costmap_map_publisher_unconfigured");

  easyfleet::CostmapMapPublisher publisher;
  EXPECT_THROW(publisher.publish(*node), std::runtime_error);
}

TEST_F(NavigationManagerTest, RoutesPublisherPublishesTheConfiguredRoutes)
{
  auto node = rclcpp::Node::make_shared(
    "test_routes_publisher", make_options(
  {
    rclcpp::Parameter("routes.package", std::string("easynav_indoor_testcase")),
    rclcpp::Parameter("routes.map_path_file", std::string("maps/routes_1.yaml")),
  }));

  easyfleet::RoutesPublisher publisher;
  ASSERT_NO_THROW(publisher.publish(*node));

  easynav_routes_maps_manager::msg::RoutesMap::SharedPtr received;
  auto sub = node->create_subscription<easynav_routes_maps_manager::msg::RoutesMap>(
    "/global_routes", rclcpp::QoS(1).transient_local().reliable(),
    [&received](easynav_routes_maps_manager::msg::RoutesMap::SharedPtr msg) {received = msg;});

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  executor.spin_some();

  ASSERT_TRUE(received != nullptr);
  ASSERT_EQ(received->routes.size(), 5u);
  EXPECT_EQ(received->routes[0].id, "route1");
  EXPECT_EQ(received->routes[4].id, "route8");
}

TEST_F(NavigationManagerTest, RoutesPublisherThrowsWhenUnconfigured)
{
  auto node = rclcpp::Node::make_shared("test_routes_publisher_unconfigured");

  easyfleet::RoutesPublisher publisher;
  EXPECT_THROW(publisher.publish(*node), std::runtime_error);
}

TEST_F(NavigationManagerTest, NodeConstructsAndPublishesBothWhenConfiguredWithCostmap)
{
  auto node = std::make_shared<easyfleet::NavigationManagerNode>(
    make_options(
  {
    rclcpp::Parameter("map_type", std::string("costmap")),
    rclcpp::Parameter("map.package", std::string("easynav_indoor_testcase")),
    rclcpp::Parameter("map.map_path_file", std::string("maps/home2.yaml")),
    rclcpp::Parameter("routes.package", std::string("easynav_indoor_testcase")),
    rclcpp::Parameter("routes.map_path_file", std::string("maps/routes_1.yaml")),
    // No capabilities are announced in this test process: keep the
    // (otherwise irrelevant here) robot-discovery window short rather
    // than blocking construction for its 2.5s default.
    rclcpp::Parameter("robots.discovery_window_sec", 0.05),
  }));

  nav_msgs::msg::OccupancyGrid::SharedPtr received_map;
  auto map_sub = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/global_map", rclcpp::QoS(1).transient_local().reliable(),
    [&received_map](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {received_map = msg;});

  easynav_routes_maps_manager::msg::RoutesMap::SharedPtr received_routes;
  auto routes_sub = node->create_subscription<easynav_routes_maps_manager::msg::RoutesMap>(
    "/global_routes", rclcpp::QoS(1).transient_local().reliable(),
    [&received_routes](easynav_routes_maps_manager::msg::RoutesMap::SharedPtr msg) {
      received_routes = msg;
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  executor.spin_some();

  ASSERT_TRUE(received_map != nullptr);
  ASSERT_TRUE(received_routes != nullptr);
  EXPECT_EQ(received_routes->routes.size(), 5u);
}

// The real regression test for the conflict monitor: two fake robots
// ("robot_a"/"robot_b", each with a raw nav_msgs/Path publisher and a
// FakeGoalManagerServer standing in for their own EasyNav instance),
// bypassing robot discovery via `robots.static_list` for determinism.
// Publishing converging paths must pause the robot farther from its own
// goal (robot_b); diverging the paths afterward must resume it.
TEST_F(NavigationManagerTest, PausesTheRobotFartherFromGoalOnConvergingPaths)
{
  FakeGoalManagerServer fake_server_a("robot_a");
  FakeGoalManagerServer fake_server_b("robot_b");

  rclcpp::executors::SingleThreadedExecutor fake_executor;
  fake_executor.add_node(fake_server_a.get_node());
  fake_executor.add_node(fake_server_b.get_node());
  std::thread fake_thread([&fake_executor] {fake_executor.spin();});
  while (!fake_executor.is_spinning()) {std::this_thread::yield();}

  auto node = std::make_shared<easyfleet::NavigationManagerNode>(
    make_options(
  {
    rclcpp::Parameter("map_type", std::string("costmap")),
    rclcpp::Parameter("map.package", std::string("easynav_indoor_testcase")),
    rclcpp::Parameter("map.map_path_file", std::string("maps/home2.yaml")),
    rclcpp::Parameter("routes.package", std::string("easynav_indoor_testcase")),
    rclcpp::Parameter("routes.map_path_file", std::string("maps/routes_1.yaml")),
    rclcpp::Parameter(
      "robots.static_list", std::vector<std::string>{"robot_a", "robot_b"}),
    rclcpp::Parameter("conflict.check_rate_hz", 20.0),
  }));

  rclcpp::NodeOptions path_pub_options;
  path_pub_options.use_global_arguments(false);
  auto path_pub_node_a = std::make_shared<rclcpp::Node>(
    "fake_path_pub", "/robot_a", path_pub_options);
  auto path_pub_a = path_pub_node_a->create_publisher<nav_msgs::msg::Path>(
    "planner_node/simple/path", 10);
  auto path_pub_node_b = std::make_shared<rclcpp::Node>(
    "fake_path_pub", "/robot_b", path_pub_options);
  auto path_pub_b = path_pub_node_b->create_publisher<nav_msgs::msg::Path>(
    "planner_node/simple/path", 10);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // robot_a: short remaining path (close to its goal).
  // robot_b: long remaining path (far from its goal), heading through
  // robot_a's own lookahead segment -- see
  // ConflictDetectorTest.ConvergingPathsPauseTheLongerOne for the same
  // geometry, proven in isolation there.
  const auto path_a = make_straight_path(0.0, 0.0, 1.0, 0.0);
  const auto path_b_converging = make_straight_path(0.0, 1.0, 0.0, -4.0);

  auto spin_until = [&](std::function<bool()> predicate, const nav_msgs::msg::Path & pb) {
      const auto start = std::chrono::steady_clock::now();
      while (!predicate() && std::chrono::steady_clock::now() - start < std::chrono::seconds(5)) {
        path_pub_a->publish(path_a);
        path_pub_b->publish(pb);
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
      return predicate();
    };

  ASSERT_TRUE(spin_until([&] {return fake_server_b.paused();}, path_b_converging));
  EXPECT_FALSE(fake_server_a.paused());

  // Diverge robot_b's path (heads away from robot_a instead) -- the
  // conflict must clear and robot_b must be resumed.
  const auto path_b_diverging = make_straight_path(0.0, 1.0, 0.0, 5.0);

  ASSERT_TRUE(spin_until([&] {return !fake_server_b.paused();}, path_b_diverging));

  fake_executor.cancel();
  fake_thread.join();
}

// Without robots.static_list, the robot set is tracked dynamically from
// /capabilities_status heartbeats -- a robot is watched while its
// heartbeats keep arriving, and dropped once they've been stale for
// longer than robots.staleness_sec. All through plain subscription
// callbacks + reconcile_watchers()'s own timer on this one node -- no
// extra rclcpp::Node/executor/thread, matching the same design as the
// conflict-check timer itself.
TEST_F(NavigationManagerTest, TracksRobotsDynamicallyViaCapabilityHeartbeats)
{
  auto node = std::make_shared<easyfleet::NavigationManagerNode>(
    make_options(
  {
    rclcpp::Parameter("map_type", std::string("costmap")),
    rclcpp::Parameter("map.package", std::string("easynav_indoor_testcase")),
    rclcpp::Parameter("map.map_path_file", std::string("maps/home2.yaml")),
    rclcpp::Parameter("routes.package", std::string("easynav_indoor_testcase")),
    rclcpp::Parameter("routes.map_path_file", std::string("maps/routes_1.yaml")),
    rclcpp::Parameter("robots.staleness_sec", 0.3),
    rclcpp::Parameter("robots.rescan_rate_hz", 20.0),
    rclcpp::Parameter("conflict.check_rate_hz", 20.0),
  }));

  rclcpp::NodeOptions status_pub_options;
  status_pub_options.use_global_arguments(false);
  auto status_pub_node = std::make_shared<rclcpp::Node>("fake_status_pub", status_pub_options);
  auto status_pub = status_pub_node->create_publisher<easyfleet_interfaces::msg::CapabilityStatus>(
    "/capabilities_status", rclcpp::QoS(10).reliable());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(status_pub_node);

  easyfleet_interfaces::msg::CapabilityStatus status;
  status.robot = "robot_a";
  status.capability = "navigation";
  status.action_name = "/robot_a/navigation";

  auto spin_while_publishing = [&](std::function<bool()> predicate, bool publish_heartbeat) {
      const auto start = std::chrono::steady_clock::now();
      while (!predicate() && std::chrono::steady_clock::now() - start < std::chrono::seconds(5)) {
        if (publish_heartbeat) {
          status_pub->publish(status);
        }
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
      return predicate();
    };

  auto is_watched = [&] {
      const auto robots = node->get_watched_robots();
      return std::find(robots.begin(), robots.end(), "robot_a") != robots.end();
    };

  ASSERT_TRUE(spin_while_publishing(is_watched, true));

  // Stop the heartbeat: once it's been stale for longer than
  // robots.staleness_sec, the watcher must be dropped.
  ASSERT_TRUE(spin_while_publishing([&] {return !is_watched();}, false));

  // Heartbeats resume: the robot must be watched again.
  ASSERT_TRUE(spin_while_publishing(is_watched, true));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
