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
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "easyfleet_interfaces/action/manipulation.hpp"
#include "easyfleet_interfaces/action/navigation.hpp"
#include "easyfleet_interfaces/action/perception.hpp"

using namespace std::chrono_literals;
using Navigation = easyfleet_interfaces::action::Navigation;
using Perception = easyfleet_interfaces::action::Perception;
using Manipulation = easyfleet_interfaces::action::Manipulation;

namespace
{

/// What a fake server does with a goal.
/// Detect streams feedback: detections, distance_remaining or progress.
enum class Behavior {Succeed, Abort, Reject, Run, Detect};

/// Fake capability server of one action, recording goals and cancels.
template<typename ActionT>
class FakeServer
{
public:
  using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;

  FakeServer(rclcpp::Node::SharedPtr node, const std::string & name)
  {
    server_ = rclcpp_action::create_server<ActionT>(
      node, name,
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const typename ActionT::Goal> goal) {
        std::lock_guard<std::mutex> lock(mutex_);
        goals_.push_back(*goal);
        return behavior_ == Behavior::Reject ? rclcpp_action::GoalResponse::REJECT :
               rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](std::shared_ptr<GoalHandle>) {
        cancels_.fetch_add(1);
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](std::shared_ptr<GoalHandle> handle) {
        threads_.emplace_back([this, handle] {execute(handle);});
      });
  }

  ~FakeServer()
  {
    stop_.store(true);
    for (auto & thread : threads_) {
      thread.join();
    }
  }

  void set(Behavior behavior, std::vector<double> detections = {})
  {
    std::lock_guard<std::mutex> lock(mutex_);
    behavior_ = behavior;
    detections_ = std::move(detections);
  }

  std::vector<typename ActionT::Goal> goals()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return goals_;
  }

  int cancels() const {return cancels_.load();}

  /// Goals that ended because they were canceled.
  int canceled() const {return canceled_.load();}

  /// Fills the detections of a perception feedback or result.
  template<typename MsgT>
  static void detect(MsgT & msg, std::size_t n)
  {
    msg.detections_3d.detections.resize(n);
    msg.detections_2d.detections.resize(n);
  }

private:
  void execute(std::shared_ptr<GoalHandle> handle)
  {
    Behavior behavior;
    std::vector<double> detections;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      behavior = behavior_;
      detections = detections_;
    }
    auto result = std::make_shared<typename ActionT::Result>();
    std::size_t step = 0;
    const auto start = std::chrono::steady_clock::now();
    while (!stop_.load() && rclcpp::ok()) {
      if (handle->is_canceling()) {
        result->error_code = ActionT::Result::CANCELED;
        handle->canceled(result);
        canceled_.fetch_add(1);
        return;
      }
      if (behavior == Behavior::Succeed && std::chrono::steady_clock::now() - start > 100ms) {
        result->error_code = ActionT::Result::SUCCESS;
        handle->succeed(result);
        return;
      }
      if (behavior == Behavior::Abort && std::chrono::steady_clock::now() - start > 100ms) {
        result->error_code = ActionT::Result::ABORTED;
        result->error_msg = "fake abort";
        handle->abort(result);
        return;
      }
      // Streams like a real one: the last value is repeated, so an early
      // feedback missed by a client that is still subscribing is not lost.
      if (behavior == Behavior::Detect && !detections.empty()) {
        auto feedback = std::make_shared<typename ActionT::Feedback>();
        const double value = detections[std::min(step++, detections.size() - 1)];
        if constexpr (std::is_same_v<ActionT, Perception>) {
          detect(*feedback, static_cast<std::size_t>(value));
        } else if constexpr (std::is_same_v<ActionT, Navigation>) {
          feedback->distance_remaining = static_cast<float>(value);
        } else {
          feedback->progress = static_cast<float>(value);
        }
        handle->publish_feedback(feedback);
      }
      std::this_thread::sleep_for(20ms);
    }
    result->error_code = ActionT::Result::ABORTED;
    handle->abort(result);
  }

  typename rclcpp_action::Server<ActionT>::SharedPtr server_;
  std::mutex mutex_;
  Behavior behavior_{Behavior::Succeed};
  std::vector<double> detections_;
  std::vector<typename ActionT::Goal> goals_;
  std::atomic<int> cancels_{0};
  std::atomic<int> canceled_{0};
  std::atomic_bool stop_{false};
  std::vector<std::thread> threads_;
};

class CapabilityNodesTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    node_ = rclcpp::Node::make_shared("fake_capabilities");
    navigation_ = std::make_unique<FakeServer<Navigation>>(node_, "/robot_1/navigation");
    perception_ = std::make_unique<FakeServer<Perception>>(node_, "/robot_1/perception");
    manipulation_ = std::make_unique<FakeServer<Manipulation>>(node_, "/robot_2/manipulation");
    other_navigation_ = std::make_unique<FakeServer<Navigation>>(node_, "/custom/nav");
    executor_.add_node(node_);
    spin_ = std::thread([this] {executor_.spin();});
    factory_.registerFromPlugin(EASYFLEET_BT_NODES_PLUGIN);
  }

  void TearDown() override
  {
    tree_.reset();
    executor_.cancel();
    spin_.join();
    navigation_.reset();
    perception_.reset();
    manipulation_.reset();
    other_navigation_.reset();
  }

  /// Builds a tree with one node.
  void make_tree(const std::string & node_xml)
  {
    tree_ = std::make_unique<BT::Tree>(factory_.createTreeFromText(
        "<root BTCPP_format=\"4\"><BehaviorTree ID=\"Main\">" + node_xml +
        "</BehaviorTree></root>"));
  }

  /// Ticks until the tree ends or `timeout` elapses (then RUNNING).
  BT::NodeStatus run(std::chrono::milliseconds timeout = 5s)
  {
    const auto end = std::chrono::steady_clock::now() + timeout;
    BT::NodeStatus status = tree_->tickOnce();
    while (status == BT::NodeStatus::RUNNING && std::chrono::steady_clock::now() < end) {
      std::this_thread::sleep_for(10ms);
      status = tree_->tickOnce();
    }
    return status;
  }

  /// Waits until `pred` holds or 2 s elapse.
  template<typename Pred>
  bool eventually(Pred pred)
  {
    const auto end = std::chrono::steady_clock::now() + 2s;
    while (!pred() && std::chrono::steady_clock::now() < end) {
      std::this_thread::sleep_for(10ms);
    }
    return pred();
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::MultiThreadedExecutor executor_;
  std::thread spin_;
  std::unique_ptr<FakeServer<Navigation>> navigation_;
  std::unique_ptr<FakeServer<Perception>> perception_;
  std::unique_ptr<FakeServer<Manipulation>> manipulation_;
  std::unique_ptr<FakeServer<Navigation>> other_navigation_;
  BT::BehaviorTreeFactory factory_;
  std::unique_ptr<BT::Tree> tree_;
};

TEST_F(CapabilityNodesTest, NavigationSucceedsAndSendsTheWaypointId)
{
  make_tree("<CallNavigation robot=\"robot_1\" goal_id=\"dock_3\"/>");
  EXPECT_EQ(run(), BT::NodeStatus::SUCCESS);
  const auto goals = navigation_->goals();
  ASSERT_EQ(goals.size(), 1u);
  EXPECT_EQ(goals[0].parameters_json, "{\"goal_id\":\"dock_3\"}");
}

TEST_F(CapabilityNodesTest, NavigationFailsWhenTheGoalAborts)
{
  navigation_->set(Behavior::Abort);
  make_tree("<CallNavigation robot=\"robot_1\" goal_id=\"dock_3\"/>");
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
}

TEST_F(CapabilityNodesTest, NavigationFailsWhenTheGoalIsRejected)
{
  navigation_->set(Behavior::Reject);
  make_tree("<CallNavigation robot=\"robot_1\" goal_id=\"dock_3\"/>");
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
}

TEST_F(CapabilityNodesTest, FailsSoonWhenTheRobotHasNoSuchCapability)
{
  make_tree("<CallNavigation robot=\"robot_9\" goal_id=\"dock_3\" server_timeout=\"0.3\"/>");
  const auto start = std::chrono::steady_clock::now();
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
  EXPECT_LT(std::chrono::steady_clock::now() - start, 2s);
}

TEST_F(CapabilityNodesTest, FailsWithoutRequiredPorts)
{
  make_tree("<CallNavigation goal_id=\"dock_3\"/>");
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
  make_tree("<CallNavigation robot=\"robot_1\"/>");
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
  make_tree("<CallPerception robot=\"robot_1\" object_classes=\" ; \"/>");
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
  make_tree("<CallManipulation robot=\"robot_2\"/>");
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
  EXPECT_TRUE(navigation_->goals().empty());
  EXPECT_TRUE(perception_->goals().empty());
  EXPECT_TRUE(manipulation_->goals().empty());
}

TEST_F(CapabilityNodesTest, ActionNamePortOverridesTheRobot)
{
  make_tree("<CallNavigation robot=\"robot_1\" action_name=\"/custom/nav\" goal_id=\"a\"/>");
  EXPECT_EQ(run(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(other_navigation_->goals().size(), 1u);
  EXPECT_TRUE(navigation_->goals().empty());
}

TEST_F(CapabilityNodesTest, HaltCancelsTheGoal)
{
  navigation_->set(Behavior::Run);
  make_tree("<CallNavigation robot=\"robot_1\" goal_id=\"far\"/>");
  EXPECT_EQ(run(500ms), BT::NodeStatus::RUNNING);
  tree_->haltTree();
  EXPECT_TRUE(eventually([this] {return navigation_->canceled() == 1;}));
}

TEST_F(CapabilityNodesTest, DestroyingTheTreeCancelsTheGoal)
{
  navigation_->set(Behavior::Run);
  make_tree("<CallNavigation robot=\"robot_1\" goal_id=\"far\"/>");
  EXPECT_EQ(run(500ms), BT::NodeStatus::RUNNING);
  tree_.reset();
  EXPECT_TRUE(eventually([this] {return navigation_->canceled() == 1;}));
}

TEST_F(CapabilityNodesTest, RunsAgainAfterEndingAndAfterBeingHalted)
{
  make_tree("<CallNavigation robot=\"robot_1\" goal_id=\"a\"/>");
  EXPECT_EQ(run(), BT::NodeStatus::SUCCESS);
  navigation_->set(Behavior::Run);
  EXPECT_EQ(run(300ms), BT::NodeStatus::RUNNING);
  tree_->haltTree();
  navigation_->set(Behavior::Succeed);
  EXPECT_EQ(run(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(navigation_->goals().size(), 3u);
  EXPECT_TRUE(eventually([this] {return navigation_->canceled() == 1;}));
}

TEST_F(CapabilityNodesTest, PerceptionSucceedsOnEnoughDetectionsAndStopsTheCapability)
{
  perception_->set(Behavior::Detect, {0, 1, 2, 3, 5});
  make_tree(
    "<CallPerception robot=\"robot_1\" object_classes=\"package; box\" min_detections=\"3\" "
    "detected=\"{n}\"/>");
  EXPECT_EQ(run(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(tree_->rootBlackboard()->get<int>("n"), 3);
  const auto goals = perception_->goals();
  ASSERT_EQ(goals.size(), 1u);
  EXPECT_EQ(goals[0].object_classes, (std::vector<std::string>{"package", "box"}));
  EXPECT_TRUE(eventually([this] {return perception_->canceled() == 1;}));
}

TEST_F(CapabilityNodesTest, PerceptionFailsAfterItsTimeoutAndStopsTheCapability)
{
  perception_->set(Behavior::Detect, {0, 0, 1});
  make_tree(
    "<CallPerception robot=\"robot_1\" object_classes=\"package\" min_detections=\"2\" "
    "timeout=\"0.5\"/>");
  const auto start = std::chrono::steady_clock::now();
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
  EXPECT_GE(std::chrono::steady_clock::now() - start, 500ms);
  EXPECT_TRUE(eventually([this] {return perception_->canceled() == 1;}));
}

TEST_F(CapabilityNodesTest, PerceptionThatEndsWithoutDetectionsFails)
{
  perception_->set(Behavior::Succeed);
  make_tree("<CallPerception robot=\"robot_1\" object_classes=\"package\"/>");
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
}

TEST_F(CapabilityNodesTest, ManipulationRunsTheNamedTask)
{
  make_tree("<CallManipulation robot=\"robot_2\" named_task=\"pick\"/>");
  EXPECT_EQ(run(), BT::NodeStatus::SUCCESS);
  const auto goals = manipulation_->goals();
  ASSERT_EQ(goals.size(), 1u);
  EXPECT_EQ(goals[0].mode, Manipulation::Goal::MODE_NAMED_TASK);
  EXPECT_EQ(goals[0].named_task, "pick");
}

TEST_F(CapabilityNodesTest, ManipulationFailsWhenTheTaskAborts)
{
  manipulation_->set(Behavior::Abort);
  make_tree("<CallManipulation robot=\"robot_2\" named_task=\"pick\"/>");
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
}

TEST_F(CapabilityNodesTest, NavigationCompletionFollowsTheDistanceRemaining)
{
  navigation_->set(Behavior::Detect, {0.0, 10.0, 8.0, 5.0, 2.5});
  make_tree("<CallNavigation robot=\"robot_1\" goal_id=\"far\"/>");
  auto completion = [this] {
      float value = -1.0f;
      (void)tree_->rootBlackboard()->get<float>("completion", value);
      return value;
    };
  EXPECT_EQ(tree_->tickOnce(), BT::NodeStatus::RUNNING);
  EXPECT_FLOAT_EQ(completion(), 0.0f);
  float last = 0.0f;
  const auto end = std::chrono::steady_clock::now() + 3s;
  while (completion() < 0.75f && std::chrono::steady_clock::now() < end) {
    EXPECT_EQ(tree_->tickOnce(), BT::NodeStatus::RUNNING);
    EXPECT_GE(completion(), last);
    EXPECT_LE(completion(), 1.0f);
    last = completion();
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_FLOAT_EQ(completion(), 0.75f);
  tree_->haltTree();
  // A new goal starts from 0.
  navigation_->set(Behavior::Run);
  EXPECT_EQ(tree_->tickOnce(), BT::NodeStatus::RUNNING);
  EXPECT_FLOAT_EQ(completion(), 0.0f);
}

TEST_F(CapabilityNodesTest, ManipulationCompletionIsItsProgress)
{
  manipulation_->set(Behavior::Detect, {0.1, 0.4, 1.7});
  make_tree("<CallManipulation robot=\"robot_2\" named_task=\"pick\"/>");
  auto completion = [this] {
      float value = -1.0f;
      (void)tree_->rootBlackboard()->get<float>("completion", value);
      return value;
    };
  const auto end = std::chrono::steady_clock::now() + 3s;
  while (completion() < 1.0f && std::chrono::steady_clock::now() < end) {
    EXPECT_EQ(tree_->tickOnce(), BT::NodeStatus::RUNNING);
    std::this_thread::sleep_for(10ms);
  }
  // Progress above 1 is clamped.
  EXPECT_FLOAT_EQ(completion(), 1.0f);
}

TEST_F(CapabilityNodesTest, PortsComeFromTheBlackboard)
{
  make_tree("<CallNavigation robot=\"{r}\" goal_id=\"{to}\"/>");
  tree_->rootBlackboard()->set<std::string>("r", "robot_1");
  tree_->rootBlackboard()->set<std::string>("to", "bay_2");
  EXPECT_EQ(run(), BT::NodeStatus::SUCCESS);
  ASSERT_EQ(navigation_->goals().size(), 1u);
  EXPECT_EQ(navigation_->goals()[0].parameters_json, "{\"goal_id\":\"bay_2\"}");
}

TEST_F(CapabilityNodesTest, SequenceOfCapabilitiesOfTwoRobots)
{
  perception_->set(Behavior::Detect, {1});
  make_tree(
    "<Sequence>"
    "<CallNavigation robot=\"robot_1\" goal_id=\"bay_1\"/>"
    "<CallPerception robot=\"robot_1\" object_classes=\"package\"/>"
    "<CallManipulation robot=\"robot_2\" named_task=\"pick\"/>"
    "</Sequence>");
  EXPECT_EQ(run(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(navigation_->goals().size(), 1u);
  EXPECT_EQ(perception_->goals().size(), 1u);
  EXPECT_EQ(manipulation_->goals().size(), 1u);
}

}  // namespace
