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

#ifndef AUTOWARE__SIMPL__CONVERSION__PREDICTED_OBJECT_HPP_
#define AUTOWARE__SIMPL__CONVERSION__PREDICTED_OBJECT_HPP_

#include "autoware/simpl/archetype/datatype.hpp"
#include "autoware/simpl/archetype/prediction.hpp"

#include <rclcpp/duration.hpp>

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/predicted_path.hpp>
#include <autoware_perception_msgs/msg/tracked_object.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/header.hpp>

#include <map>
#include <string>
#include <vector>

namespace autoware::simpl::conversion
{
/**
 * @brief Convert to predicted `Pose` from `PredictedState` and `TrackedObject`.
 *
 * @param predicted_state Predicted state.
 * @param tracked_object Tracked object. Only z and orientation will be used.
 */
inline geometry_msgs::msg::Pose to_predicted_pose(
  const archetype::PredictedState & predicted_state,
  const autoware_perception_msgs::msg::TrackedObject & tracked_object)
{
  geometry_msgs::msg::Pose predicted_pose;
  predicted_pose.position.x = predicted_state.x;
  predicted_pose.position.y = predicted_state.y;
  predicted_pose.position.z = tracked_object.kinematics.pose_with_covariance.pose.position.z;
  predicted_pose.orientation = tracked_object.kinematics.pose_with_covariance.pose.orientation;
  return predicted_pose;
}

/**
 * @brief Convert to `PredictedPath` with `Prediction` and `TrackedObject`.
 *
 * @param confidence
 * @param waypoints
 * @param tracked_object
 */
inline autoware_perception_msgs::msg::PredictedPath to_predicted_mode(
  double confidence, const std::vector<archetype::PredictedState> & waypoints,
  const autoware_perception_msgs::msg::TrackedObject & tracked_object)
{
  autoware_perception_msgs::msg::PredictedPath predicted_path;
  predicted_path.confidence = confidence;
  predicted_path.path.reserve(waypoints.size());
  predicted_path.time_step = rclcpp::Duration::from_seconds(0.1);
  for (const auto & predicted_state : waypoints) {
    predicted_path.path.emplace_back(to_predicted_pose(predicted_state, tracked_object));
    if (predicted_path.path.size() >= predicted_path.path.max_size()) {
      break;
    }
  }
  return predicted_path;
}

/**
 * @brief Initialize `PredictedObject` with `TrackedObject`.
 *
 * @param tracked_object Tracked object message.
 */
inline autoware_perception_msgs::msg::PredictedObject from_tracked_object(
  const autoware_perception_msgs::msg::TrackedObject & tracked_object)
{
  autoware_perception_msgs::msg::PredictedObject predicted_object;
  predicted_object.classification = tracked_object.classification;
  predicted_object.object_id = tracked_object.object_id;
  predicted_object.shape = tracked_object.shape;
  predicted_object.existence_probability = tracked_object.existence_probability;

  predicted_object.kinematics.initial_pose_with_covariance =
    tracked_object.kinematics.pose_with_covariance;
  predicted_object.kinematics.initial_twist_with_covariance =
    tracked_object.kinematics.twist_with_covariance;
  predicted_object.kinematics.initial_acceleration_with_covariance =
    tracked_object.kinematics.acceleration_with_covariance;

  return predicted_object;
}

/**
 * @brief Convert to `PredictedObject` with `Prediction` and `TracedObject`.
 *
 * @param prediction Predicted trajectories for a single object.
 * @param tracked_object Tracked object.
 */
inline autoware_perception_msgs::msg::PredictedObject to_predicted_object(
  const archetype::Prediction & prediction,
  const autoware_perception_msgs::msg::TrackedObject & tracked_object)
{
  auto predicted_object = from_tracked_object(tracked_object);
  for (const auto & [confidence, waypoints] : prediction) {
    auto predicted_mode = to_predicted_mode(confidence, waypoints, tracked_object);
    predicted_object.kinematics.predicted_paths.emplace_back(predicted_mode);
  }
  return predicted_object;
}

/**
 * @brief Convert to `PredictedObjects` with `Predictions` and mapped `TrackedObjects`.
 *
 * @param header Message header.
 * @param predictions Predicted trajectories for each agent.
 * @param tracked_objects Tracked objects mapped by its agent IDs.
 */
inline autoware_perception_msgs::msg::PredictedObjects to_predicted_objects(
  const std_msgs::msg::Header & header, const archetype::Predictions & predictions,
  const std::map<std::string, autoware_perception_msgs::msg::TrackedObject> & tracked_objects)
{
  autoware_perception_msgs::msg::PredictedObjects outputs;
  outputs.header = header;
  for (const auto & prediction : predictions) {
    const auto & tracked_object = tracked_objects.at(prediction.agent_id);
    const auto predicted_object = to_predicted_object(prediction, tracked_object);
    outputs.objects.emplace_back(predicted_object);
  }
  return outputs;
}
}  // namespace autoware::simpl::conversion
#endif  // AUTOWARE__SIMPL__CONVERSION__PREDICTED_OBJECT_HPP_
