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

#ifndef EASYFLEET_NAVIGATION_MANAGER__FLEET_OBSTACLES_HPP_
#define EASYFLEET_NAVIGATION_MANAGER__FLEET_OBSTACLES_HPP_

#include <chrono>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace easyfleet
{

/// @brief Where a robot is in its own "<robot>/map" frame.
struct RobotPose
{
  double x {0.0};
  double y {0.0};
  /// @brief Stamp of the newest transform it was computed from.
  builtin_interfaces::msg::Time stamp;
  /// @brief When it was last updated.
  std::chrono::steady_clock::time_point received;
};

/// @brief Tracks where one robot is from its own TF messages, chaining
/// "<robot>/map" -> ... -> "<robot>/<base_frame>" (e.g. map -> odom ->
/// base_footprint).
class RobotPoseTracker
{
public:
  /// @param robot_id Robot namespace and frame prefix, e.g. "robot_1".
  /// @param base_frame Frame placed in the map, without the prefix.
  RobotPoseTracker(std::string robot_id, std::string base_frame);

  /// @brief Takes the transforms of a /<robot>/tf or /<robot>/tf_static message.
  /// @param msg The message.
  /// @param now When it arrived.
  void update(const tf2_msgs::msg::TFMessage & msg, std::chrono::steady_clock::time_point now);

  /// @brief The robot's pose, if the chain from its map to its base is known.
  /// @return The pose, or nothing.
  [[nodiscard]] std::optional<RobotPose> pose() const {return pose_;}

private:
  void recompute(std::chrono::steady_clock::time_point now);

  std::string map_frame_;
  std::string base_frame_;
  /// @brief child frame -> latest transform to it from its parent.
  std::map<std::string, geometry_msgs::msg::TransformStamped> transforms_;
  std::optional<RobotPose> pose_;
};

/// @brief Shape of each robot in the obstacle clouds.
struct FleetObstacleParams
{
  /// @brief Radius (m) of the disc drawn for each robot.
  double radius_m {0.25};
  /// @brief Height (m) of the points, above the map plane.
  double height_m {0.25};
  /// @brief Distance (m) between neighbouring points of the disc.
  double spacing_m {0.05};
};

/// @brief The other robots, as an obstacle cloud for `receiver`.
/**
 * One disc of points per robot of `poses` other than `receiver`, in the
 * frame "<receiver>/map": every robot of the fleet shares the map, so a
 * position in one robot's map is the same position in another's. Stamped
 * with the receiver's own latest stamp, so its TF can place the points
 * right away; empty if the receiver's pose is unknown.
 * @param receiver Robot the cloud is for.
 * @param poses Pose of each robot, by robot id.
 * @param params Shape of each robot.
 * @return The cloud (x, y, z as float32).
 */
sensor_msgs::msg::PointCloud2 make_fleet_obstacle_cloud(
  const std::string & receiver, const std::map<std::string, RobotPose> & poses,
  const FleetObstacleParams & params);

}  // namespace easyfleet

#endif  // EASYFLEET_NAVIGATION_MANAGER__FLEET_OBSTACLES_HPP_
