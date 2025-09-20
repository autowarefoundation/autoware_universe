// Copyright 2025 TIER IV, Inc.
//
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

#ifndef AUTOWARE__TRAJECTORY_RANKER__METRICS__METRICS_UTILS_HPP_
#define AUTOWARE__TRAJECTORY_RANKER__METRICS__METRICS_UTILS_HPP_

#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <memory>
#include <vector>

namespace autoware::trajectory_ranker::metrics::utils
{

using autoware_perception_msgs::msg::PredictedObjects;
using autoware_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::Pose;
using vehicle_info_utils::VehicleInfo;

using TrajectoryPoints = std::vector<TrajectoryPoint>;

/**
 * @brief Transforms a point from global to vehicle-relative coordinates (2D)
 * @param point Point in global coordinates
 * @param origin Vehicle pose (origin of relative coordinate system)
 * @return Point in vehicle-relative coordinates
 */
geometry_msgs::msg::Point transform_to_relative_coordinate2_d(
  const geometry_msgs::msg::Point & point, const geometry_msgs::msg::Pose & origin);

/**
 * @brief Calculates the turning radius to reach a target point
 * @param target Target point in global coordinates
 * @param current_pose Current vehicle pose
 * @return Turning radius [m] (large value if path is nearly straight)
 */
float calc_radius(
  const geometry_msgs::msg::Point & target, const geometry_msgs::msg::Pose & current_pose);

/**
 * @brief Calculates the path curvature to reach a target point
 * @param target Target point in global coordinates
 * @param current_pose Current vehicle pose
 * @return Curvature [1/m] (inverse of radius)
 */
float curvature(
  const geometry_msgs::msg::Point & target, const geometry_msgs::msg::Pose & current_pose);

/**
 * @brief Calculates pure pursuit curvature for trajectory following
 * @param points Trajectory points to follow
 * @param ego_pose Current ego vehicle pose
 * @return Curvature [1/m] using 10m lookahead distance
 */
float pure_pursuit(const std::shared_ptr<TrajectoryPoints> & points, const Pose & ego_pose);

/**
 * @brief Calculates steering wheel angle command using pure pursuit
 * @param points Trajectory points to follow
 * @param ego_pose Current ego vehicle pose
 * @param wheel_base Vehicle wheelbase [m]
 * @return Steering wheel angle [rad]
 */
float steer_command(
  const std::shared_ptr<TrajectoryPoints> & points, const Pose & ego_pose, const float wheel_base);

/**
 * @brief Calculates minimum time to collision with all predicted objects
 * @param points Ego trajectory points
 * @param objects Predicted objects with their future paths
 * @param idx Index of ego trajectory point to evaluate
 * @return Time to collision [s] (capped at 10s maximum)
 */
float time_to_collision(
  const std::shared_ptr<TrajectoryPoints> & points,
  const std::shared_ptr<PredictedObjects> & objects, const size_t idx, const float max_ttc_value);

/**
 * @brief Calculates time to collision between two moving points
 * @param point1 First trajectory point with velocity
 * @param point2 Second trajectory point with velocity
 * @return Time to collision [s] (infinity if no collision)
 */
float time_to_collision(const TrajectoryPoint & point1, const TrajectoryPoint & point2);

/**
 * @brief Calculates time to collision with a predicted object at a specific time
 * @param ego_point Ego trajectory point
 * @param duration Time offset for object prediction
 * @param object Predicted object with future path
 * @return Time to collision [s] (capped at 10s maximum)
 */
float time_to_collision(
  const TrajectoryPoint & ego_point, const rclcpp::Duration & duration,
  const autoware_perception_msgs::msg::PredictedObject & object, const float max_ttc_value);

}  // namespace autoware::trajectory_ranker::metrics::utils

#endif  // AUTOWARE__TRAJECTORY_RANKER__METRICS__METRICS_UTILS_HPP_
