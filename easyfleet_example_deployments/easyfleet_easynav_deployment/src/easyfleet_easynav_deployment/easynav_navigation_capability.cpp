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

#include "easyfleet_easynav_deployment/easynav_navigation_capability.hpp"

#include <cctype>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "nlohmann/json.hpp"

namespace easyfleet_easynav_deployment
{

namespace
{

std::string sanitize_identifier(const std::string & text)
{
  std::string result = text;
  for (auto & c : result) {
    if (!std::isalnum(static_cast<unsigned char>(c))) {
      c = '_';
    }
  }
  if (result.empty() || std::isdigit(static_cast<unsigned char>(result.front()))) {
    result.insert(result.begin(), '_');
  }
  return result;
}

/// Extracts the "goal_id" string field from a Navigation goal's
/// parameters_json, if present and well-formed. This backend expects every
/// goal to carry one; target_pose/waypoints on the goal itself go unused.
std::optional<std::string> extract_goal_id(const std::string & parameters_json)
{
  if (parameters_json.empty()) {
    return std::nullopt;
  }
  try {
    const auto json = nlohmann::json::parse(parameters_json);
    if (json.contains("goal_id") && json["goal_id"].is_string()) {
      return json["goal_id"].get<std::string>();
    }
  } catch (const nlohmann::json::parse_error &) {
  }
  return std::nullopt;
}

}  // namespace

EasynavNavigationActionServer::EasynavNavigationActionServer(
  rclcpp_lifecycle::LifecycleNode * node,
  const std::string & action_name)
: easyfleet_core::NavigationActionServerBase(node, action_name)
{
  const auto waypoint_ids = node->declare_parameter(
    action_name + ".waypoint_ids", std::vector<std::string>());

  auto waypoints = std::make_shared<std::map<std::string, geometry_msgs::msg::PoseStamped>>();
  for (const auto & id : waypoint_ids) {
    const std::string prefix = action_name + ".waypoints." + id + ".";
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = node->declare_parameter(prefix + "frame_id", std::string("map"));
    pose.pose.position.x = node->declare_parameter(prefix + "x", 0.0);
    pose.pose.position.y = node->declare_parameter(prefix + "y", 0.0);
    const double yaw = node->declare_parameter(prefix + "yaw", 0.0);
    pose.pose.orientation.z = std::sin(yaw / 2.0);
    pose.pose.orientation.w = std::cos(yaw / 2.0);
    (*waypoints)[id] = pose;
  }
  waypoints_ = waypoints;

  // Declared unprefixed (like "capabilities_file"), not under
  // "<action_name>.", so that launch-time inline parameter overrides for it
  // do not share a namespace with "<action_name>.waypoint_ids"/"waypoints"
  // -- combining them under the same from-file + inline-override node was
  // observed to make the array-typed waypoint_ids load empty. Same reasoning
  // applies to bt_plugins below.
  behavior_tree_xml_ = node->declare_parameter("behavior_tree_xml", std::string());
  tick_rate_hz_ = node->declare_parameter(action_name + ".tick_rate_hz", 10.0);
  const auto bt_plugins = node->declare_parameter("bt_plugins", std::vector<std::string>());

  // Internal node dedicated to the GoalManagerClient, spun on its own
  // background thread for the whole lifetime of this action server.
  const std::string internal_node_name = sanitize_identifier(
    std::string(node->get_name()) + "_" + action_name + "_gm_client");
  rclcpp::NodeOptions internal_options;
  internal_options.start_parameter_services(false);
  internal_options.start_parameter_event_publisher(false);
  internal_options.use_global_arguments(false);
  internal_node_ = std::make_shared<rclcpp::Node>(
    internal_node_name, node->get_namespace(), internal_options);

  gm_client_ = easynav::GoalManagerClient::make_shared(internal_node_);

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(internal_node_);
  spin_thread_ = std::thread([this] {executor_->spin();});
  while (!executor_->is_spinning()) {
    std::this_thread::yield();
  }

  // Every BT node this tree can use -- including this package's own
  // Navigate -- is loaded as a plugin by path, so this class has no
  // compile-time dependency on any specific node implementation. See the
  // class doc comment.
  for (const auto & plugin_path : bt_plugins) {
    factory_.registerFromPlugin(plugin_path);
  }
}

EasynavNavigationActionServer::~EasynavNavigationActionServer()
{
  if (executor_) {
    executor_->cancel();
  }
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
}

rclcpp_action::GoalResponse EasynavNavigationActionServer::on_goal_received(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const Goal> goal)
{
  const auto goal_id = extract_goal_id(goal->parameters_json);
  if (!goal_id || waypoints_->find(*goal_id) == waypoints_->end()) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void EasynavNavigationActionServer::on_execute(const GoalHandleSharedPtr goal_handle)
{
  const auto goal = goal_handle->get_goal();
  // on_goal_received() already validated that this resolves to a known
  // waypoint, so it is safe to dereference here.
  const auto goal_id = *extract_goal_id(goal->parameters_json);

  auto blackboard = BT::Blackboard::create();
  blackboard->set("goal_id", goal_id);
  // Read by Navigate (see bt_nodes/navigate.hpp): this plugin-loaded node
  // gets its dependencies from the blackboard rather than constructor
  // arguments, since a BT.CPP plugin's node type is only ever instantiated
  // as `Navigate(name, config)` by the factory.
  blackboard->set("waypoints", waypoints_);
  blackboard->set("gm_client", gm_client_);

  BT::Tree tree;
  try {
    tree = factory_.createTreeFromFile(behavior_tree_xml_, blackboard);
  } catch (const std::exception & e) {
    auto result = std::make_shared<Result>();
    result->error_code = Result::ABORTED;
    result->error_msg = std::string("Failed to load behavior tree: ") + e.what();
    goal_handle->abort(result);
    return;
  }

  auto feedback = std::make_shared<Feedback>();
  rclcpp::Rate rate(tick_rate_hz_);
  BT::NodeStatus status = BT::NodeStatus::RUNNING;

  while (status == BT::NodeStatus::RUNNING) {
    if (goal_handle->is_canceling()) {
      gm_client_->cancel();
      auto result = std::make_shared<Result>();
      result->error_code = Result::CANCELED;
      result->error_msg = "Navigation canceled by client.";
      goal_handle->canceled(result);
      return;
    }
    if (is_preempt_requested()) {
      // Deliberately do not touch gm_client_/the tree: the preempting
      // goal's fresh Navigate node will redirect EasyNav via the same
      // shared client (see bt_nodes/navigate.hpp).
      auto result = std::make_shared<Result>();
      result->error_code = Result::ABORTED;
      result->error_msg = "Navigation preempted by a newer goal.";
      goal_handle->abort(result);
      return;
    }
    if (is_shutdown_requested()) {
      gm_client_->cancel();
      auto result = std::make_shared<Result>();
      result->error_code = Result::ABORTED;
      result->error_msg = "Navigation capability is shutting down.";
      goal_handle->abort(result);
      return;
    }

    status = tree.tickOnce();

    const auto & control_feedback = gm_client_->get_feedback();
    feedback->current_pose = control_feedback.current_pose;
    feedback->navigation_time = control_feedback.navigation_time;
    feedback->estimated_time_remaining = control_feedback.estimated_time_remaining;
    feedback->distance_remaining = control_feedback.distance_to_goal;
    goal_handle->publish_feedback(feedback);

    rate.sleep();
  }

  auto result = std::make_shared<Result>();
  if (status == BT::NodeStatus::SUCCESS) {
    result->error_code = Result::SUCCESS;
    result->final_pose = gm_client_->get_result().current_pose;
    goal_handle->succeed(result);
  } else {
    result->error_code = Result::ABORTED;
    result->error_msg = "Navigation behavior tree did not succeed.";
    goal_handle->abort(result);
  }
}

EasynavNavigationCapability::EasynavNavigationCapability(const rclcpp::NodeOptions & options)
: easyfleet_core::Capability<EasynavNavigationActionServer>("navigation", options)
{
}

}  // namespace easyfleet_easynav_deployment
