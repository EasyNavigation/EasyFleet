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

#include "easyfleet_fake_collaboration_deployment/perception_fake_capability.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <random>
#include <string>
#include <thread>
#include <vector>

namespace easyfleet_fake_collaboration_deployment
{

namespace
{
bool requests_target_class(
  const std::vector<std::string> & object_classes, const std::string & target_class)
{
  return std::any_of(
    object_classes.begin(), object_classes.end(),
    [&target_class](const std::string & requested) {
      std::string lower = requested;
      std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
      return lower.find(target_class) != std::string::npos;
    });
}
}  // namespace

PerceptionFakeActionServer::PerceptionFakeActionServer(
  rclcpp_lifecycle::LifecycleNode & node,
  const std::string & action_name)
: easyfleet_core::PerceptionActionServerBase(node, action_name),
  random_engine_(std::random_device{}())
{
  target_class_ = node.declare_parameter(action_name + ".target_class", std::string("gato"));
  frame_id_ = node.declare_parameter(action_name + ".frame_id", std::string("camera_link"));
  detection_rate_hz_ = node.declare_parameter(action_name + ".detection_rate_hz", 20.0);

  std::transform(target_class_.begin(), target_class_.end(), target_class_.begin(), ::tolower);
}

rclcpp_action::GoalResponse PerceptionFakeActionServer::on_goal_received(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const Goal> goal)
{
  if (goal->object_classes.empty() ||
    !requests_target_class(goal->object_classes, target_class_))
  {
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

PerceptionFakeActionServer::Feedback PerceptionFakeActionServer::make_random_detections()
{
  std::uniform_int_distribution<int> count_dist(1, 3);
  std::uniform_real_distribution<double> xy_dist(-3.0, 3.0);
  std::uniform_real_distribution<double> z_dist(0.0, 1.0);
  std::uniform_real_distribution<double> pixel_x_dist(0.0, 640.0);
  std::uniform_real_distribution<double> pixel_y_dist(0.0, 480.0);
  std::uniform_real_distribution<double> score_dist(0.6, 0.99);

  Feedback feedback;
  feedback.detections_3d.header.frame_id = frame_id_;
  feedback.detections_2d.header.frame_id = frame_id_;

  const int num_cats = count_dist(random_engine_);
  for (int i = 0; i < num_cats; ++i) {
    const double score = score_dist(random_engine_);
    const std::string id = "gato_" + std::to_string(i);

    vision_msgs::msg::Detection3D detection_3d;
    detection_3d.header.frame_id = frame_id_;
    detection_3d.id = id;
    vision_msgs::msg::ObjectHypothesisWithPose hypothesis_3d;
    hypothesis_3d.hypothesis.class_id = target_class_;
    hypothesis_3d.hypothesis.score = score;
    hypothesis_3d.pose.pose.position.x = xy_dist(random_engine_);
    hypothesis_3d.pose.pose.position.y = xy_dist(random_engine_);
    hypothesis_3d.pose.pose.position.z = z_dist(random_engine_);
    detection_3d.results.push_back(hypothesis_3d);
    detection_3d.bbox.center = hypothesis_3d.pose.pose;
    detection_3d.bbox.size.x = 0.3;
    detection_3d.bbox.size.y = 0.3;
    detection_3d.bbox.size.z = 0.2;
    feedback.detections_3d.detections.push_back(detection_3d);

    vision_msgs::msg::Detection2D detection_2d;
    detection_2d.header.frame_id = frame_id_;
    detection_2d.id = id;
    vision_msgs::msg::ObjectHypothesisWithPose hypothesis_2d;
    hypothesis_2d.hypothesis.class_id = target_class_;
    hypothesis_2d.hypothesis.score = score;
    detection_2d.results.push_back(hypothesis_2d);
    detection_2d.bbox.center.position.x = pixel_x_dist(random_engine_);
    detection_2d.bbox.center.position.y = pixel_y_dist(random_engine_);
    detection_2d.bbox.size_x = 100.0;
    detection_2d.bbox.size_y = 100.0;
    feedback.detections_2d.detections.push_back(detection_2d);
  }

  return feedback;
}

void PerceptionFakeActionServer::on_execute(const GoalHandleSharedPtr goal_handle)
{
  // `continuous`/`max_rate_hz` on the goal are accepted but not honored by
  // this mock: it always streams forever at detection_rate_hz_ until
  // canceled, preempted or shut down, regardless of what the goal requested.
  const auto period = std::chrono::duration<double>(1.0 / detection_rate_hz_);

  while (true) {
    if (goal_handle->is_canceling()) {
      auto result = std::make_shared<Result>();
      result->error_code = Result::CANCELED;
      goal_handle->canceled(result);
      return;
    }
    if (is_preempt_requested()) {
      auto result = std::make_shared<Result>();
      result->error_code = Result::ABORTED;
      result->error_msg = "Perception goal preempted by a newer goal.";
      goal_handle->abort(result);
      return;
    }
    if (is_shutdown_requested()) {
      auto result = std::make_shared<Result>();
      result->error_code = Result::ABORTED;
      result->error_msg = "Perception capability is shutting down.";
      goal_handle->abort(result);
      return;
    }

    auto feedback = std::make_shared<Feedback>(make_random_detections());
    goal_handle->publish_feedback(feedback);

    std::this_thread::sleep_for(period);
  }
}

PerceptionFakeCapability::PerceptionFakeCapability(const rclcpp::NodeOptions & options)
: easyfleet_core::Capability<PerceptionFakeActionServer>("perception", options)
{
}

}  // namespace easyfleet_fake_collaboration_deployment
