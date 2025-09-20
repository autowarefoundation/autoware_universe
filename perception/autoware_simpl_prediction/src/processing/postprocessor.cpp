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

#include "autoware/simpl_prediction/processing/postprocessor.hpp"

#include <autoware_utils_geometry/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::simpl_prediction::processing
{
namespace
{
/**
 * @brief Convert predicted confidence and waypoints to `PredictedPath` with `TrackedObject`.
 *
 * @param confidence Predicted confidence.
 * @param waypoints Predicted waypoints.
 * @param tracked_object Tracked object.
 */
PostProcessor::PredictedPath to_predicted_path(
  double confidence, const std::vector<std::pair<double, double>> & waypoints,
  const PostProcessor::TrackedObject & tracked_object)
{
  PostProcessor::PredictedPath output;
  output.confidence = confidence;
  output.time_step = rclcpp::Duration::from_seconds(0.1);

  output.path.resize(waypoints.size() + 1);
  output.path.at(0) = tracked_object.kinematics.pose_with_covariance.pose;
  for (size_t i = 0; i < waypoints.size(); ++i) {
    const auto & [x, y] = waypoints.at(i);

    const auto position = geometry_msgs::build<geometry_msgs::msg::Point>().x(x).y(y).z(
      tracked_object.kinematics.pose_with_covariance.pose.position.z);

    const auto yaw =
      autoware_utils_geometry::calc_azimuth_angle(output.path.at(i).position, position);
    const auto orientation = autoware_utils_geometry::create_quaternion_from_yaw(yaw);

    output.path.at(i + 1) =
      geometry_msgs::build<geometry_msgs::msg::Pose>().position(position).orientation(orientation);
  }
  return output;
}

/**
 * @brief Construct `PredictedObject` with `TrackedObject`.
 *
 * @param input Tracked object.
 */
PostProcessor::PredictedObject from_tracked_object(const PostProcessor::TrackedObject & input)
{
  PostProcessor::PredictedObject output;
  output.classification = input.classification;
  output.object_id = input.object_id;
  output.shape = input.shape;
  output.existence_probability = input.existence_probability;

  output.kinematics.initial_pose_with_covariance = input.kinematics.pose_with_covariance;
  output.kinematics.initial_twist_with_covariance = input.kinematics.twist_with_covariance;
  output.kinematics.initial_acceleration_with_covariance =
    input.kinematics.acceleration_with_covariance;

  return output;
}

/**
 * @brief Perform 2D transformation from local to map coordinate frame.
 *
 * @param local_x X position in object local frame.
 * @param local_y Y position in object local frame.
 * @param object
 * @return std::pair<double, double>
 */
std::pair<double, double> local_to_global(
  double local_x, double local_y, const PostProcessor::TrackedObject & object)
{
  const auto & map_pose = object.kinematics.pose_with_covariance.pose;

  const auto map_yaw = autoware_utils_geometry::get_rpy(map_pose.orientation).z;

  auto map_cos = std::cos(map_yaw);
  auto map_sin = std::sin(map_yaw);

  auto map_x = local_x * map_cos - local_y * map_sin + map_pose.position.x;
  auto map_y = local_x * map_sin + local_y * map_cos + map_pose.position.y;

  return {map_x, map_y};
}
}  // namespace

PostProcessor::PostProcessor(size_t num_mode, size_t num_future, double score_threshold)
: num_mode_(num_mode), num_future_(num_future), score_threshold_(score_threshold)
{
}

PostProcessor::output_type PostProcessor::process(
  const std::vector<float> & scores, const std::vector<float> & trajectories,
  const std::vector<std::string> & agent_ids, const Header & header,
  const std::unordered_map<std::string, TrackedObject> & tracked_object_map) const
{
  constexpr size_t num_attribute = 4;

  // NOTE: Predicted paths with confidence lower than threshold are filtered out.
  // If all modes are filtered out, the object containing no path is published.
  std::vector<PredictedObject> predicted_objects;
  for (size_t n = 0; n < agent_ids.size(); ++n) {
    const auto & tracked_object = tracked_object_map.at(agent_ids.at(n));

    auto predicted_object = from_tracked_object(tracked_object);

    const auto mode_scores = scores.data() + n * num_mode_;
    const auto mode_indices = sort_by_score(mode_scores);
    for (size_t m : mode_indices) {
      const auto & confidence = static_cast<double>(*(mode_scores + m));

      // skip if the score is lower than threshold
      if (confidence < score_threshold_) {
        continue;
      }

      std::vector<std::pair<double, double>> waypoints;
      for (size_t t = 0; t < num_future_; ++t) {
        // Index: (n * M * Tf + m * Tf + t) * Dp
        const auto trajectory_idx =
          (n * num_mode_ * num_future_ + m * num_future_ + t) * num_attribute;

        // (x, y, vx, vy): w.r.t object local
        const double ox = static_cast<double>(trajectories.at(trajectory_idx));
        const double oy = static_cast<double>(trajectories.at(trajectory_idx + 1));

        // Transform from current state centric to map coordinate
        waypoints.emplace_back(local_to_global(ox, oy, tracked_object));
      }
      predicted_object.kinematics.predicted_paths.emplace_back(
        to_predicted_path(confidence, waypoints, tracked_object));
    }
    predicted_objects.emplace_back(predicted_object);
  }

  return autoware_perception_msgs::build<PredictedObjects>().header(header).objects(
    predicted_objects);
}

std::vector<size_t> PostProcessor::sort_by_score(const float * scores) const
{
  std::vector<size_t> output(num_mode_);
  for (size_t i = 0; i < num_mode_; ++i) {
    output[i] = i;
  }
  std::sort(
    output.begin(), output.end(), [scores](size_t a, size_t b) { return scores[a] > scores[b]; });
  return output;
}
}  // namespace autoware::simpl_prediction::processing
