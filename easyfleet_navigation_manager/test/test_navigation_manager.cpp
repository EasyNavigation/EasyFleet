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
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "easyfleet_interfaces/msg/capability_status.hpp"
#include "easyfleet_navigation_manager/costmap_map_publisher.hpp"
#include "easyfleet_navigation_manager/navigation_manager_node.hpp"
#include "easyfleet_navigation_manager/routes_publisher.hpp"
#include "easynav_interfaces/msg/navigation_control.hpp"
#include "easynav_routes_maps_manager/msg/routes_map.hpp"
#include "easynav_routes_maps_manager/route_io.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"

// Map and routes fixtures are written once per process to a temporary
// directory (see fixtures()) and loaded by absolute path, so these tests
// don't depend on any other package's data.

namespace
{
rclcpp::NodeOptions make_options(const std::vector<rclcpp::Parameter> & params)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(params);
  return options;
}

/// Paths of the map/routes files every test loads.
struct Fixtures
{
  std::string map_yaml;
  std::string routes_yaml;
};

/// A 20x10 free map at 0.05 m/cell, and five routes with non-contiguous ids.
const Fixtures & fixtures()
{
  static const Fixtures files = [] {
      const auto dir = std::filesystem::temp_directory_path() /
        ("easyfleet_navigation_manager_test_" + std::to_string(::getpid()));
      std::filesystem::create_directories(dir);
      {
        std::ofstream pgm(dir / "map.pgm", std::ios::binary);
        pgm << "P5\n20 10\n255\n" << std::string(20 * 10, static_cast<char>(254));
      }
      {
        std::ofstream yaml(dir / "map.yaml");
        yaml << "image: map.pgm\nmode: trinary\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\n"
          "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n";
      }
      easynav::RoutesMap routes;
      for (const auto & id : {"route1", "route2", "route3", "route5", "route8"}) {
        easynav::RouteSegment segment;
        segment.id = id;
        segment.start.orientation.w = 1.0;
        segment.end.position.x = 1.0;
        segment.end.orientation.w = 1.0;
        routes.push_back(segment);
      }
      std::string error;
      const auto routes_path = (dir / "routes.yaml").string();
      if (!easynav::save_routes_to_yaml(routes_path, routes, error)) {
        throw std::runtime_error("could not write routes fixture: " + error);
      }
      return Fixtures{(dir / "map.yaml").string(), routes_path};
    }();
  return files;
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
          pause_requests_++;
          if (idle_) {
            response.type = easynav_interfaces::msg::NavigationControl::REJECT;
          } else {
            paused_ = true;
            response.type = easynav_interfaces::msg::NavigationControl::PAUSED;
          }
          pub_->publish(response);
        } else if (msg->type == easynav_interfaces::msg::NavigationControl::RESUME) {
          resume_requests_++;
          paused_ = false;
          response.type = easynav_interfaces::msg::NavigationControl::RESUMED;
          pub_->publish(response);
        }
      });
  }

  [[nodiscard]] rclcpp::Node::SharedPtr get_node() const {return node_;}
  [[nodiscard]] bool paused() const {return paused_;}
  [[nodiscard]] int pause_requests() const {return pause_requests_;}
  [[nodiscard]] int resume_requests() const {return resume_requests_;}

  /// A new goal from the robot's own client: GoalManager clears the pause.
  void new_goal()
  {
    paused_ = false;
    idle_ = false;
    report(easynav_interfaces::msg::NavigationControl::ACCEPT, "robot_client");
  }

  /// The robot reached its goal: GoalManager goes idle.
  void finish()
  {
    paused_ = false;
    idle_ = true;
    report(easynav_interfaces::msg::NavigationControl::FINISHED, "robot_client");
  }

  /// Someone else (an operator tool) pauses the robot.
  void operator_pause()
  {
    paused_ = true;
    report(easynav_interfaces::msg::NavigationControl::PAUSED, "operator");
  }

private:
  void report(uint8_t type, const std::string & addressee)
  {
    easynav_interfaces::msg::NavigationControl msg;
    msg.type = type;
    msg.user_id = "fake_gm_server";
    msg.nav_current_user_id = addressee;
    pub_->publish(msg);
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<easynav_interfaces::msg::NavigationControl>::SharedPtr pub_;
  rclcpp::Subscription<easynav_interfaces::msg::NavigationControl>::SharedPtr sub_;
  std::atomic_bool paused_ {false};
  std::atomic_bool idle_ {false};
  std::atomic_int pause_requests_ {0};
  std::atomic_int resume_requests_ {0};
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
    rclcpp::Parameter("map.package", std::string()),
    rclcpp::Parameter("map.map_path_file", fixtures().map_yaml),
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
    rclcpp::Parameter("routes.package", std::string()),
    rclcpp::Parameter("routes.map_path_file", fixtures().routes_yaml),
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

// A late-joining subscriber reliably getting the retained
// transient_local sample turned out NOT to be guaranteed in practice
// for visualization_msgs/MarkerArray specifically (verified outside
// this codebase with a minimal std_msgs/String control case using the
// exact same QoS, which *did* always deliver) -- so
// /global_routes_markers is volatile, kept alive by a low-rate
// republish timer instead of durability-service replay (see the class
// doc comment: a topic that needs periodic republishing to reach late
// joiners isn't really transient_local). This test drives that timer
// fast (`routes.markers_republish_rate_hz`) and asserts more than one
// message actually arrives.
TEST_F(NavigationManagerTest, RoutesPublisherPeriodicallyRepublishesMarkers)
{
  auto node = rclcpp::Node::make_shared(
    "test_routes_publisher_markers_timer", make_options(
  {
    rclcpp::Parameter("routes.package", std::string()),
    rclcpp::Parameter("routes.map_path_file", fixtures().routes_yaml),
    rclcpp::Parameter("routes.markers_republish_rate_hz", 20.0),
  }));

  easyfleet::RoutesPublisher publisher;
  ASSERT_NO_THROW(publisher.publish(*node));

  int received_count = 0;
  auto sub = node->create_subscription<visualization_msgs::msg::MarkerArray>(
    "/global_routes_markers", rclcpp::QoS(10).reliable(),
    [&received_count](visualization_msgs::msg::MarkerArray::SharedPtr) {++received_count;});

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  const auto start = std::chrono::steady_clock::now();
  while (received_count < 3 &&
    std::chrono::steady_clock::now() - start < std::chrono::seconds(5))
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  EXPECT_GE(received_count, 3);
}

// routes.markers_republish_rate_hz is used as a timer-period divisor
// (1.0 / rate) -- a zero or negative value would produce an invalid
// duration, so it must be rejected up front rather than handed to
// create_wall_timer().
TEST_F(NavigationManagerTest, RoutesPublisherThrowsWhenMarkersRepublishRateIsNonPositive)
{
  auto node = rclcpp::Node::make_shared(
    "test_routes_publisher_bad_rate", make_options(
  {
    rclcpp::Parameter("routes.package", std::string()),
    rclcpp::Parameter("routes.map_path_file", fixtures().routes_yaml),
    rclcpp::Parameter("routes.markers_republish_rate_hz", 0.0),
  }));

  easyfleet::RoutesPublisher publisher;
  EXPECT_THROW(publisher.publish(*node), std::runtime_error);
}

TEST_F(NavigationManagerTest, RoutesPublisherThrowsWhenUnconfigured)
{
  auto node = rclcpp::Node::make_shared("test_routes_publisher_unconfigured");

  easyfleet::RoutesPublisher publisher;
  EXPECT_THROW(publisher.publish(*node), std::runtime_error);
}

// Exercises the interactive-marker editing mechanism end-to-end, via the
// real /global_routes_imarkers/feedback topic the InteractiveMarkerServer
// itself subscribes to (see interactive_marker_server.py's own
// `namespace + '/feedback'` convention, mirrored by the C++
// implementation) -- not by calling RoutesPublisher's private handler
// directly. Confirms the user's explicit requirement #3: every
// geometry-changing edit (drag/add/remove) must both republish on
// /global_routes and re-save to the routes YAML file, while merely
// toggling edit mode must do neither.
TEST_F(NavigationManagerTest, InteractiveMarkerEditRepublishesAndSavesRoutes)
{
  const std::string yaml_path =
    (std::filesystem::temp_directory_path() / "easyfleet_test_routes_edit.yaml").string();
  std::filesystem::remove(yaml_path);

  auto node = rclcpp::Node::make_shared(
    "test_routes_publisher_edit", make_options(
  {
    rclcpp::Parameter("routes.package", std::string()),
    // Absolute path: no package needed, and the shared fixture is never touched.
    rclcpp::Parameter("routes.map_path_file", yaml_path),
  }));

  easyfleet::RoutesPublisher publisher;
  ASSERT_NO_THROW(publisher.publish(*node));

  // No file exists yet at yaml_path, so load_routes_from_yaml() fell
  // back to its single default segment, id "route0".
  ASSERT_FALSE(std::filesystem::exists(yaml_path));

  easynav_routes_maps_manager::msg::RoutesMap::SharedPtr received;
  auto sub = node->create_subscription<easynav_routes_maps_manager::msg::RoutesMap>(
    "/global_routes", rclcpp::QoS(1).transient_local().reliable(),
    [&received](easynav_routes_maps_manager::msg::RoutesMap::SharedPtr msg) {received = msg;});

  auto feedback_pub = node->create_publisher<visualization_msgs::msg::InteractiveMarkerFeedback>(
    "/global_routes_imarkers/feedback", rclcpp::QoS(1).reliable());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  executor.spin_some();

  ASSERT_TRUE(received != nullptr);
  received.reset();

  auto spin_for = [&](std::chrono::milliseconds duration) {
      const auto start = std::chrono::steady_clock::now();
      while (std::chrono::steady_clock::now() - start < duration) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    };

  // 1) Toggle edit mode on: reveals the draggable start/end markers, but
  // is purely a UI-editor-state change -- must NOT republish or save.
  visualization_msgs::msg::InteractiveMarkerFeedback toggle_fb;
  toggle_fb.marker_name = "route0_mode";
  toggle_fb.control_name = "toggle_edit";
  toggle_fb.event_type = visualization_msgs::msg::InteractiveMarkerFeedback::BUTTON_CLICK;
  toggle_fb.pose.orientation.w = 1.0;

  feedback_pub->publish(toggle_fb);
  spin_for(std::chrono::milliseconds(300));

  EXPECT_TRUE(received == nullptr);
  EXPECT_FALSE(std::filesystem::exists(yaml_path));

  // 2) Drag the "route0_start" endpoint to a new pose: a
  // geometry-changing edit, must republish on /global_routes AND save to
  // yaml_path.
  visualization_msgs::msg::InteractiveMarkerFeedback drag_fb;
  drag_fb.marker_name = "route0_start";
  drag_fb.control_name = "move_x";
  drag_fb.event_type = visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE;
  drag_fb.pose.position.x = 3.5;
  drag_fb.pose.position.y = -2.0;
  drag_fb.pose.orientation.w = 1.0;

  // While dragging: nothing is sent to the robots, nothing is saved.
  feedback_pub->publish(drag_fb);
  {
    const auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(500)) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }
  EXPECT_TRUE(received == nullptr);
  EXPECT_FALSE(std::filesystem::exists(yaml_path));

  // Releasing the endpoint commits the edit.
  drag_fb.event_type = visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP;
  feedback_pub->publish(drag_fb);

  {
    const auto start = std::chrono::steady_clock::now();
    while (received == nullptr &&
      std::chrono::steady_clock::now() - start < std::chrono::seconds(5))
    {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }

  ASSERT_TRUE(received != nullptr);
  ASSERT_EQ(received->routes.size(), 1u);
  EXPECT_NEAR(received->routes[0].start.position.x, 3.5, 1e-6);
  EXPECT_NEAR(received->routes[0].start.position.y, -2.0, 1e-6);

  ASSERT_TRUE(std::filesystem::exists(yaml_path));
  {
    const auto saved = easynav::load_routes_from_yaml(yaml_path);
    ASSERT_EQ(saved.size(), 1u);
    EXPECT_NEAR(saved[0].start.position.x, 3.5, 1e-6);
    EXPECT_NEAR(saved[0].start.position.y, -2.0, 1e-6);
  }

  // 3) Add a new segment from "route0_end": also a geometry-changing
  // edit, republish + save again, and the new segment gets a fresh id.
  received.reset();
  visualization_msgs::msg::InteractiveMarkerFeedback add_fb;
  add_fb.marker_name = "route0_end";
  add_fb.control_name = "add_segment";
  add_fb.event_type = visualization_msgs::msg::InteractiveMarkerFeedback::BUTTON_CLICK;
  add_fb.pose.position.x = 1.0;
  add_fb.pose.orientation.w = 1.0;

  feedback_pub->publish(add_fb);

  {
    const auto start = std::chrono::steady_clock::now();
    while ((received == nullptr || received->routes.size() < 2) &&
      std::chrono::steady_clock::now() - start < std::chrono::seconds(5))
    {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }

  ASSERT_TRUE(received != nullptr);
  ASSERT_EQ(received->routes.size(), 2u);
  EXPECT_EQ(received->routes[1].id, "route1");
  {
    const auto saved = easynav::load_routes_from_yaml(yaml_path);
    EXPECT_EQ(saved.size(), 2u);
  }

  // 4) Remove the original segment via "route0_start": back down to one
  // segment, republished and saved.
  received.reset();
  visualization_msgs::msg::InteractiveMarkerFeedback remove_fb;
  remove_fb.marker_name = "route0_start";
  remove_fb.control_name = "remove_segment";
  remove_fb.event_type = visualization_msgs::msg::InteractiveMarkerFeedback::BUTTON_CLICK;
  remove_fb.pose.orientation.w = 1.0;

  feedback_pub->publish(remove_fb);

  {
    const auto start = std::chrono::steady_clock::now();
    while ((received == nullptr || received->routes.size() != 1) &&
      std::chrono::steady_clock::now() - start < std::chrono::seconds(5))
    {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }

  ASSERT_TRUE(received != nullptr);
  ASSERT_EQ(received->routes.size(), 1u);
  EXPECT_EQ(received->routes[0].id, "route1");
  {
    const auto saved = easynav::load_routes_from_yaml(yaml_path);
    ASSERT_EQ(saved.size(), 1u);
    EXPECT_EQ(saved[0].id, "route1");
  }

  std::filesystem::remove(yaml_path);
}

TEST_F(NavigationManagerTest, EditsGoToSavePathFileAndStrayMouseUpOnAButtonDoesNothing)
{
  const auto save_path =
    (std::filesystem::temp_directory_path() /
    ("easyfleet_test_routes_saved_" + std::to_string(::getpid()) + ".yaml")).string();
  std::filesystem::remove(save_path);
  const auto original = easynav::load_routes_from_yaml(fixtures().routes_yaml);

  auto node = rclcpp::Node::make_shared(
    "test_routes_publisher_save_path", make_options(
  {
    rclcpp::Parameter("routes.map_path_file", fixtures().routes_yaml),
    rclcpp::Parameter("routes.save_path_file", save_path),
  }));
  easyfleet::RoutesPublisher publisher;
  ASSERT_NO_THROW(publisher.publish(*node));

  auto feedback_pub = node->create_publisher<visualization_msgs::msg::InteractiveMarkerFeedback>(
    "/global_routes_imarkers/feedback", rclcpp::QoS(1).reliable());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spin_for = [&](std::chrono::milliseconds duration) {
      const auto start = std::chrono::steady_clock::now();
      while (std::chrono::steady_clock::now() - start < duration) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    };
  spin_for(std::chrono::milliseconds(300));

  // Edit mode first: the endpoint markers (and their buttons) only exist then.
  visualization_msgs::msg::InteractiveMarkerFeedback toggle_fb;
  toggle_fb.marker_name = "route1_mode";
  toggle_fb.control_name = "toggle_edit";
  toggle_fb.event_type = visualization_msgs::msg::InteractiveMarkerFeedback::BUTTON_CLICK;
  feedback_pub->publish(toggle_fb);
  spin_for(std::chrono::milliseconds(300));

  // RViz sends MOUSE_UP along with BUTTON_CLICK: alone it must not add a segment.
  visualization_msgs::msg::InteractiveMarkerFeedback add_fb;
  add_fb.marker_name = "route1_end";
  add_fb.control_name = "add_segment";
  add_fb.event_type = visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP;
  add_fb.pose.orientation.w = 1.0;
  feedback_pub->publish(add_fb);
  spin_for(std::chrono::milliseconds(300));
  EXPECT_FALSE(std::filesystem::exists(save_path));

  add_fb.event_type = visualization_msgs::msg::InteractiveMarkerFeedback::BUTTON_CLICK;
  feedback_pub->publish(add_fb);
  const auto start = std::chrono::steady_clock::now();
  while (!std::filesystem::exists(save_path) &&
    std::chrono::steady_clock::now() - start < std::chrono::seconds(5))
  {
    spin_for(std::chrono::milliseconds(20));
  }
  ASSERT_TRUE(std::filesystem::exists(save_path));
  EXPECT_EQ(easynav::load_routes_from_yaml(save_path).size(), original.size() + 1);
  EXPECT_EQ(easynav::load_routes_from_yaml(fixtures().routes_yaml).size(), original.size())
    << "the loaded file must stay untouched";
  std::filesystem::remove(save_path);
}

TEST_F(NavigationManagerTest, RelativeMapPathWithoutPackageThrows)
{
  auto node = rclcpp::Node::make_shared(
    "test_costmap_relative_no_package", make_options(
  {
    rclcpp::Parameter("map.map_path_file", std::string("maps/some.yaml")),
  }));
  easyfleet::CostmapMapPublisher publisher;
  EXPECT_THROW(publisher.publish(*node), std::runtime_error);
}

TEST_F(NavigationManagerTest, NodeConstructsAndPublishesBothWhenConfiguredWithCostmap)
{
  auto node = std::make_shared<easyfleet::NavigationManagerNode>(
    make_options(
  {
    rclcpp::Parameter("map_type", std::string("costmap")),
    rclcpp::Parameter("map.package", std::string()),
    rclcpp::Parameter("map.map_path_file", fixtures().map_yaml),
    rclcpp::Parameter("routes.package", std::string()),
    rclcpp::Parameter("routes.map_path_file", fixtures().routes_yaml),
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
    rclcpp::Parameter("map.package", std::string()),
    rclcpp::Parameter("map.map_path_file", fixtures().map_yaml),
    rclcpp::Parameter("routes.package", std::string()),
    rclcpp::Parameter("routes.map_path_file", fixtures().routes_yaml),
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

// Every robot's own EasyNav instance applies its tf_prefix to its local
// "map" frame (e.g. robot_1's own frame is literally "robot_1/map"),
// while /global_map and /global_routes are both stamped with the plain
// "map" frame -- so for the two to line up in one TF tree, the node
// must also broadcast a static, identity "map" -> "<robot_id>/map"
// transform per watched robot, on /tf_static.
TEST_F(NavigationManagerTest, PublishesStaticMapToRobotMapTransforms)
{
  auto node = std::make_shared<easyfleet::NavigationManagerNode>(
    make_options(
  {
    rclcpp::Parameter("map_type", std::string("costmap")),
    rclcpp::Parameter("map.package", std::string()),
    rclcpp::Parameter("map.map_path_file", fixtures().map_yaml),
    rclcpp::Parameter("routes.package", std::string()),
    rclcpp::Parameter("routes.map_path_file", fixtures().routes_yaml),
    rclcpp::Parameter(
      "robots.static_list", std::vector<std::string>{"robot_a", "robot_b"}),
  }));

  std::vector<geometry_msgs::msg::TransformStamped> received_transforms;
  auto tf_sub = node->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf_static", rclcpp::QoS(10).transient_local().reliable(),
    [&received_transforms](tf2_msgs::msg::TFMessage::SharedPtr msg) {
      for (const auto & t : msg->transforms) {
        received_transforms.push_back(t);
      }
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  const auto start = std::chrono::steady_clock::now();
  while (received_transforms.size() < 2 &&
    std::chrono::steady_clock::now() - start < std::chrono::seconds(5))
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  auto find_transform = [&](const std::string & child_frame_id) {
      return std::find_if(
        received_transforms.begin(), received_transforms.end(),
        [&](const geometry_msgs::msg::TransformStamped & t) {
          return t.child_frame_id == child_frame_id;
        });
    };

  const auto tf_a = find_transform("robot_a/map");
  ASSERT_NE(tf_a, received_transforms.end());
  EXPECT_EQ(tf_a->header.frame_id, "map");
  EXPECT_NEAR(tf_a->transform.translation.x, 0.0, 1e-9);
  EXPECT_NEAR(tf_a->transform.translation.y, 0.0, 1e-9);
  EXPECT_NEAR(tf_a->transform.translation.z, 0.0, 1e-9);
  EXPECT_NEAR(tf_a->transform.rotation.w, 1.0, 1e-9);

  const auto tf_b = find_transform("robot_b/map");
  ASSERT_NE(tf_b, received_transforms.end());
  EXPECT_EQ(tf_b->header.frame_id, "map");
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
    rclcpp::Parameter("map.package", std::string()),
    rclcpp::Parameter("map.map_path_file", fixtures().map_yaml),
    rclcpp::Parameter("routes.package", std::string()),
    rclcpp::Parameter("routes.map_path_file", fixtures().routes_yaml),
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

namespace
{

tf2_msgs::msg::TFMessage make_tf(const std::string & parent, const std::string & child)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = parent;
  transform.child_frame_id = child;
  transform.transform.rotation.w = 1.0;
  tf2_msgs::msg::TFMessage msg;
  msg.transforms.push_back(transform);
  return msg;
}

struct RelayedTf
{
  std::set<std::string> dynamic_frames;
  std::set<std::string> static_frames;
  int base_footprint_count {0};  // robot_a/base_footprint messages on /tf
};

/// Runs a node that watches robot_a and robot_b and records what reaches
/// /tf and /tf_static while each robot publishes on its own TF topics, for 2 s
/// at 50 Hz (plus a wheel frame that only a full relay lets through).
RelayedTf relayed_frames(
  bool relay_tf, const std::vector<std::string> & frames = {}, double rate_hz = 0.0)
{
  auto node = std::make_shared<easyfleet::NavigationManagerNode>(
    make_options(
    {
      rclcpp::Parameter("map_type", std::string("costmap")),
      rclcpp::Parameter("map.map_path_file", fixtures().map_yaml),
      rclcpp::Parameter("routes.map_path_file", fixtures().routes_yaml),
      rclcpp::Parameter("robots.static_list", std::vector<std::string>{"robot_a", "robot_b"}),
      rclcpp::Parameter("robots.relay_tf", relay_tf),
      rclcpp::Parameter("robots.relay_tf_frames", frames),
      rclcpp::Parameter("robots.relay_tf_rate_hz", rate_hz),
  }));

  rclcpp::NodeOptions options;
  options.use_global_arguments(false);
  auto robots = std::make_shared<rclcpp::Node>("fake_robot_tf", options);
  auto tf_a = robots->create_publisher<tf2_msgs::msg::TFMessage>("/robot_a/tf", 10);
  auto tf_b = robots->create_publisher<tf2_msgs::msg::TFMessage>("/robot_b/tf", 10);
  auto static_a = robots->create_publisher<tf2_msgs::msg::TFMessage>(
    "/robot_a/tf_static", rclcpp::QoS(1).transient_local());
  auto static_b = robots->create_publisher<tf2_msgs::msg::TFMessage>(
    "/robot_b/tf_static", rclcpp::QoS(1).transient_local());

  RelayedTf relayed;
  auto & dynamic_frames = relayed.dynamic_frames;
  auto & static_frames = relayed.static_frames;
  auto listener = std::make_shared<rclcpp::Node>("tf_listener", options);
  auto tf_sub = listener->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf", rclcpp::QoS(100),
    [&](tf2_msgs::msg::TFMessage::SharedPtr msg) {
      for (const auto & tr : msg->transforms) {
        dynamic_frames.insert(tr.child_frame_id);
        relayed.base_footprint_count += tr.child_frame_id == "robot_a/base_footprint";
      }
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(robots);
  executor.add_node(listener);
  static_a->publish(make_tf("robot_a/base_footprint", "robot_a/base_link"));
  static_b->publish(make_tf("robot_b/base_footprint", "robot_b/base_link"));
  const auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < std::chrono::seconds(2)) {
    tf_a->publish(make_tf("robot_a/odom", "robot_a/base_footprint"));
    tf_a->publish(make_tf("robot_a/base_link", "robot_a/wheel_left_link"));
    tf_b->publish(make_tf("robot_b/odom", "robot_b/base_footprint"));
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  // A late subscriber must get every robot's static frames, not only the last.
  auto late = std::make_shared<rclcpp::Node>("late_tf_static_listener", options);
  auto static_sub = late->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf_static", rclcpp::QoS(10).transient_local().reliable(),
    [&](tf2_msgs::msg::TFMessage::SharedPtr msg) {
      for (const auto & tr : msg->transforms) {
        static_frames.insert(tr.child_frame_id);
      }
    });
  executor.add_node(late);
  const auto late_start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - late_start < std::chrono::seconds(1)) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return relayed;
}

}  // namespace

TEST_F(NavigationManagerTest, RelaysEveryWatchedRobotsOwnTfOntoTheFleetTf)
{
  const auto relayed = relayed_frames(true);
  const auto & dynamic_frames = relayed.dynamic_frames;
  const auto & static_frames = relayed.static_frames;
  EXPECT_TRUE(dynamic_frames.count("robot_a/wheel_left_link")) << "no filter: everything";
  EXPECT_TRUE(dynamic_frames.count("robot_a/base_footprint"));
  EXPECT_TRUE(dynamic_frames.count("robot_b/base_footprint"));
  EXPECT_TRUE(static_frames.count("robot_a/base_link"));
  EXPECT_TRUE(static_frames.count("robot_b/base_link"));
  // The manager's own map -> <robot>/map frames are still there too.
  EXPECT_TRUE(static_frames.count("robot_a/map"));
}

TEST_F(NavigationManagerTest, DoesNotRelayTfUnlessAskedTo)
{
  const auto relayed = relayed_frames(false);
  const auto & dynamic_frames = relayed.dynamic_frames;
  const auto & static_frames = relayed.static_frames;
  EXPECT_TRUE(dynamic_frames.empty());
  EXPECT_FALSE(static_frames.count("robot_a/base_link"));
  EXPECT_TRUE(static_frames.count("robot_a/map"));
}

TEST_F(NavigationManagerTest, RelaysOnlyTheSelectedFrames)
{
  const auto relayed = relayed_frames(true, {"base_footprint", "base_link"});
  EXPECT_TRUE(relayed.dynamic_frames.count("robot_a/base_footprint"));
  EXPECT_TRUE(relayed.dynamic_frames.count("robot_b/base_footprint"));
  EXPECT_FALSE(relayed.dynamic_frames.count("robot_a/wheel_left_link"))
    << "robot-local frames stay inside the robot";
  EXPECT_TRUE(relayed.static_frames.count("robot_a/base_link"));
  EXPECT_TRUE(relayed.static_frames.count("robot_b/base_link"));
}

TEST_F(NavigationManagerTest, RelayedFramesAreRateLimited)
{
  // ~100 messages published in 2 s; at 5 Hz only a handful may pass.
  const auto unlimited = relayed_frames(true, {"base_footprint"});
  const auto limited = relayed_frames(true, {"base_footprint"}, 5.0);
  EXPECT_GT(unlimited.base_footprint_count, 40);
  EXPECT_GE(limited.base_footprint_count, 5);
  EXPECT_LT(limited.base_footprint_count * 3, unlimited.base_footprint_count);
}

namespace
{

/// Map -> odom -> base_footprint of `robot`, with base_footprint at (x, y).
tf2_msgs::msg::TFMessage robot_at(const std::string & robot, double x, double y, int32_t sec)
{
  auto msg = make_tf(robot + "/map", robot + "/odom");
  msg.transforms[0].header.stamp.sec = sec;
  auto base = make_tf(robot + "/odom", robot + "/base_footprint");
  base.transforms[0].header.stamp.sec = sec;
  base.transforms[0].transform.translation.x = x;
  base.transforms[0].transform.translation.y = y;
  msg.transforms.push_back(base.transforms[0]);
  return msg;
}

/// Points of `cloud` (x, y).
std::vector<std::pair<float, float>> cloud_points(const sensor_msgs::msg::PointCloud2 & cloud)
{
  std::vector<std::pair<float, float>> out;
  if (cloud.width == 0) {
    return out;
  }
  sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y(cloud, "y");
  for (; x != x.end(); ++x, ++y) {
    out.emplace_back(*x, *y);
  }
  return out;
}

std::shared_ptr<easyfleet::NavigationManagerNode> obstacles_node(bool share, bool relay)
{
  return std::make_shared<easyfleet::NavigationManagerNode>(
    make_options(
    {
      rclcpp::Parameter("map_type", std::string("costmap")),
      rclcpp::Parameter("map.map_path_file", fixtures().map_yaml),
      rclcpp::Parameter("routes.map_path_file", fixtures().routes_yaml),
      rclcpp::Parameter("robots.static_list", std::vector<std::string>{"robot_a", "robot_b"}),
      rclcpp::Parameter("robots.relay_tf", relay),
      rclcpp::Parameter("robots.share_obstacles", share),
      rclcpp::Parameter("robots.obstacle_radius_m", 0.3),
      rclcpp::Parameter("robots.obstacles_pose_timeout_sec", 0.5),
  }));
}

}  // namespace

TEST_F(NavigationManagerTest, SharesEachRobotAsAnObstacleOfTheOthers)
{
  auto node = obstacles_node(true, false);
  rclcpp::NodeOptions options;
  options.use_global_arguments(false);
  auto robots = std::make_shared<rclcpp::Node>("fake_robots", options);
  auto tf_a = robots->create_publisher<tf2_msgs::msg::TFMessage>("/robot_a/tf", 10);
  auto tf_b = robots->create_publisher<tf2_msgs::msg::TFMessage>("/robot_b/tf", 10);
  std::optional<sensor_msgs::msg::PointCloud2> cloud_a, cloud_b;
  auto sub_a = robots->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/robot_a/fleet_obstacles", rclcpp::QoS(1),
    [&](sensor_msgs::msg::PointCloud2::SharedPtr msg) {cloud_a = *msg;});
  auto sub_b = robots->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/robot_b/fleet_obstacles", rclcpp::QoS(1),
    [&](sensor_msgs::msg::PointCloud2::SharedPtr msg) {cloud_b = *msg;});
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(robots);

  // Sim time only goes forward: an older transform is ignored.
  int32_t sec = 10;
  auto run = [&](std::chrono::milliseconds duration, bool publish_b) {
      const auto start = std::chrono::steady_clock::now();
      while (std::chrono::steady_clock::now() - start < duration) {
        tf_a->publish(robot_at("robot_a", 1.0, 2.0, sec));
        if (publish_b) {
          tf_b->publish(robot_at("robot_b", 6.0, -3.0, sec));
        }
        ++sec;
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
    };
  run(std::chrono::milliseconds(1500), true);

  ASSERT_TRUE(cloud_a);
  ASSERT_TRUE(cloud_b);
  EXPECT_EQ(cloud_a->header.frame_id, "robot_a/map");
  EXPECT_EQ(cloud_b->header.frame_id, "robot_b/map");
  EXPECT_GT(cloud_a->header.stamp.sec, 10) << "stamped with the robot's own TF time";
  const auto a = cloud_points(*cloud_a);
  const auto b = cloud_points(*cloud_b);
  ASSERT_FALSE(a.empty());
  ASSERT_FALSE(b.empty());
  for (const auto & [x, y] : a) {
    EXPECT_LE(std::hypot(x - 6.0, y + 3.0), 0.3 + 1e-4) << "robot_a sees only robot_b";
  }
  for (const auto & [x, y] : b) {
    EXPECT_LE(std::hypot(x - 1.0, y - 2.0), 0.3 + 1e-4) << "robot_b sees only robot_a";
  }

  // robot_b goes silent: after the pose timeout robot_a has nobody to see and gets no more
  // clouds; never an empty one.
  int clouds_a = 0;
  auto count_a = robots->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/robot_a/fleet_obstacles", rclcpp::QoS(1),
    [&](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      ++clouds_a;
      EXPECT_GT(msg->width, 0u) << "never an empty cloud";
    });
  run(std::chrono::milliseconds(800), false);  // longer than the 0.5 s pose timeout
  clouds_a = 0;
  run(std::chrono::milliseconds(1000), false);
  EXPECT_EQ(clouds_a, 0);
}

TEST_F(NavigationManagerTest, SharesNoObstaclesUnlessAskedTo)
{
  auto node = obstacles_node(false, true);
  rclcpp::NodeOptions options;
  options.use_global_arguments(false);
  auto robots = std::make_shared<rclcpp::Node>("fake_robots_off", options);
  auto tf_a = robots->create_publisher<tf2_msgs::msg::TFMessage>("/robot_a/tf", 10);
  auto tf_b = robots->create_publisher<tf2_msgs::msg::TFMessage>("/robot_b/tf", 10);
  int clouds = 0;
  auto sub_a = robots->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/robot_a/fleet_obstacles", rclcpp::QoS(1),
    [&](sensor_msgs::msg::PointCloud2::SharedPtr) {++clouds;});
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(robots);
  const auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < std::chrono::seconds(1)) {
    tf_a->publish(robot_at("robot_a", 1.0, 2.0, 10));
    tf_b->publish(robot_at("robot_b", 6.0, -3.0, 10));
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  EXPECT_EQ(clouds, 0);
}

/// Two robots (fake GoalManagers + path publishers) watched by a
/// NavigationManagerNode, for the pause/resume reconciliation tests.
class ConflictMonitorTest : public NavigationManagerTest
{
protected:
  void SetUp() override
  {
    fake_a_ = std::make_unique<FakeGoalManagerServer>("robot_a");
    fake_b_ = std::make_unique<FakeGoalManagerServer>("robot_b");
    fake_executor_.add_node(fake_a_->get_node());
    fake_executor_.add_node(fake_b_->get_node());
    fake_thread_ = std::thread([this] {fake_executor_.spin();});
    while (!fake_executor_.is_spinning()) {std::this_thread::yield();}

    rclcpp::NodeOptions options;
    options.use_global_arguments(false);
    path_node_ = std::make_shared<rclcpp::Node>("fake_path_pub", options);
    path_pub_a_ = path_node_->create_publisher<nav_msgs::msg::Path>(
      "/robot_a/planner_node/simple/path", 10);
    path_pub_b_ = path_node_->create_publisher<nav_msgs::msg::Path>(
      "/robot_b/planner_node/simple/path", 10);
    status_pub_ = path_node_->create_publisher<easyfleet_interfaces::msg::CapabilityStatus>(
      "/capabilities_status", rclcpp::QoS(10).reliable());
  }

  void TearDown() override
  {
    fake_executor_.cancel();
    fake_thread_.join();
  }

  void make_node(std::vector<rclcpp::Parameter> extra, bool static_list = true)
  {
    std::vector<rclcpp::Parameter> params{
      rclcpp::Parameter("map_type", std::string("costmap")),
      rclcpp::Parameter("map.map_path_file", fixtures().map_yaml),
      rclcpp::Parameter("routes.map_path_file", fixtures().routes_yaml),
      rclcpp::Parameter("conflict.check_rate_hz", 20.0),
    };
    if (static_list) {
      params.emplace_back(
        "robots.static_list", std::vector<std::string>{"robot_a", "robot_b"});
    }
    params.insert(params.end(), extra.begin(), extra.end());
    node_ = std::make_shared<easyfleet::NavigationManagerNode>(make_options(params));
    executor_.add_node(node_);
  }

  /// Spins until `predicate` holds (or `timeout`), publishing the given paths
  /// (and heartbeats for the given robots) every 20 ms.
  bool spin_until(
    const std::function<bool()> & predicate, const nav_msgs::msg::Path * path_a,
    const nav_msgs::msg::Path * path_b, std::chrono::milliseconds timeout = std::chrono::seconds(5),
    const std::vector<std::string> & heartbeats = {})
  {
    const auto start = std::chrono::steady_clock::now();
    while (!predicate() && std::chrono::steady_clock::now() - start < timeout) {
      if (path_a) {path_pub_a_->publish(*path_a);}
      if (path_b) {path_pub_b_->publish(*path_b);}
      for (const auto & robot : heartbeats) {
        easyfleet_interfaces::msg::CapabilityStatus status;
        status.robot = robot;
        status.capability = "navigation";
        status.action_name = "/" + robot + "/navigation";
        status_pub_->publish(status);
      }
      executor_.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return predicate();
  }

  /// Lets the manager's watchers and the fake GoalManagers find each other,
  /// so reports sent right after aren't lost.
  void wait_for_discovery()
  {
    spin_until(
      [&] {
        return fake_a_->get_node()->count_subscribers("/robot_a/easynav_control") >= 2 &&
               fake_b_->get_node()->count_subscribers("/robot_b/easynav_control") >= 2;
      }, nullptr, nullptr);
    holds_for([] {return true;}, nullptr, nullptr, std::chrono::milliseconds(200));
  }

  /// Spins for `duration` regardless; returns whether `predicate` held throughout.
  bool holds_for(
    const std::function<bool()> & predicate, const nav_msgs::msg::Path * path_a,
    const nav_msgs::msg::Path * path_b, std::chrono::milliseconds duration)
  {
    bool held = true;
    spin_until([&] {held = held && predicate(); return false;}, path_a, path_b, duration);
    return held && predicate();
  }

  const nav_msgs::msg::Path path_a_ = make_straight_path(0.0, 0.0, 1.0, 0.0);
  const nav_msgs::msg::Path path_b_converging_ = make_straight_path(0.0, 1.0, 0.0, -4.0);
  const nav_msgs::msg::Path path_b_diverging_ = make_straight_path(0.0, 1.0, 0.0, 5.0);

  std::unique_ptr<FakeGoalManagerServer> fake_a_;
  std::unique_ptr<FakeGoalManagerServer> fake_b_;
  rclcpp::executors::SingleThreadedExecutor fake_executor_;
  std::thread fake_thread_;
  rclcpp::Node::SharedPtr path_node_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_a_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_b_;
  rclcpp::Publisher<easyfleet_interfaces::msg::CapabilityStatus>::SharedPtr status_pub_;
  std::shared_ptr<easyfleet::NavigationManagerNode> node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
};

TEST_F(ConflictMonitorTest, RePausesWhenANewGoalClearsThePause)
{
  make_node({});
  executor_.add_node(path_node_);
  ASSERT_TRUE(spin_until([&] {return fake_b_->paused();}, &path_a_, &path_b_converging_));

  // robot_b's own client preempts its goal: EasyNav clears the pause flag.
  fake_b_->new_goal();
  ASSERT_TRUE(
    spin_until(
      [&] {return fake_b_->pause_requests() >= 2 && fake_b_->paused();},
      &path_a_, &path_b_converging_));
  EXPECT_FALSE(fake_a_->paused());
}

TEST_F(ConflictMonitorTest, DoesNotFloodPauseRequestsWhileConfirmed)
{
  make_node({});
  executor_.add_node(path_node_);
  ASSERT_TRUE(spin_until([&] {return fake_b_->paused();}, &path_a_, &path_b_converging_));
  holds_for([] {return true;}, &path_a_, &path_b_converging_, std::chrono::milliseconds(1000));
  EXPECT_EQ(fake_b_->pause_requests(), 1);
}

TEST_F(ConflictMonitorTest, LeavesAnOperatorPauseAlone)
{
  make_node({});
  executor_.add_node(path_node_);
  wait_for_discovery();
  fake_a_->operator_pause();
  EXPECT_TRUE(
    holds_for(
      [&] {return fake_a_->paused();}, &path_a_, &path_b_diverging_,
      std::chrono::milliseconds(1500)));
  EXPECT_EQ(fake_a_->resume_requests(), 0);
}

TEST_F(ConflictMonitorTest, IgnoresARobotThatIsNotNavigating)
{
  make_node({});
  executor_.add_node(path_node_);
  wait_for_discovery();
  fake_b_->finish();
  fake_a_->finish();
  holds_for([] {return true;}, nullptr, nullptr, std::chrono::milliseconds(300));
  // Both still publish (stale) paths that converge, but neither navigates.
  EXPECT_TRUE(
    holds_for(
      [&] {return !fake_a_->paused() && !fake_b_->paused();}, &path_a_, &path_b_converging_,
      std::chrono::milliseconds(1500)));
}

TEST_F(ConflictMonitorTest, AStalePathStopsCountingAndReleasesThePause)
{
  make_node({rclcpp::Parameter("conflict.path_timeout_sec", 0.5)});
  executor_.add_node(path_node_);
  ASSERT_TRUE(spin_until([&] {return fake_b_->paused();}, &path_a_, &path_b_converging_));

  // robot_a's planner goes silent (its process died): its last path must
  // not keep robot_b stopped.
  ASSERT_TRUE(spin_until([&] {return !fake_b_->paused();}, nullptr, &path_b_converging_));
}

TEST_F(ConflictMonitorTest, ARobotPausedTooLongGetsPriority)
{
  make_node({rclcpp::Parameter("conflict.max_pause_sec", 1.0)});
  executor_.add_node(path_node_);
  ASSERT_TRUE(spin_until([&] {return fake_b_->paused();}, &path_a_, &path_b_converging_));
  EXPECT_FALSE(fake_a_->paused());

  // The conflict never clears: after max_pause_sec the roles swap.
  ASSERT_TRUE(
    spin_until(
      [&] {return fake_a_->paused() && !fake_b_->paused();}, &path_a_, &path_b_converging_,
      std::chrono::seconds(4)));
}

TEST_F(ConflictMonitorTest, ResumesARobotItPausedBeforeDroppingIt)
{
  make_node(
    {rclcpp::Parameter("robots.staleness_sec", 0.5),
      rclcpp::Parameter("robots.rescan_rate_hz", 20.0)}, /*static_list=*/ false);
  executor_.add_node(path_node_);
  const std::vector<std::string> both{"robot_a", "robot_b"};
  ASSERT_TRUE(
    spin_until(
      [&] {return fake_b_->paused();}, &path_a_, &path_b_converging_,
      std::chrono::seconds(5), both));

  // robot_b's heartbeat goes stale: the manager stops watching it, but must
  // not leave it paused behind.
  ASSERT_TRUE(
    spin_until(
      [&] {return !fake_b_->paused();}, &path_a_, &path_b_converging_,
      std::chrono::seconds(5), {"robot_a"}));
  EXPECT_GE(fake_b_->resume_requests(), 1);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
