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

#ifndef AUTOWARE__IMAGE_PROJECTION_BASED_FUSION__UTILS__SIZE_VALIDATION_HPP_
#define AUTOWARE__IMAGE_PROJECTION_BASED_FUSION__UTILS__SIZE_VALIDATION_HPP_

#include <autoware_perception_msgs/msg/detected_object.hpp>
#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <cmath>
#include <map>
#include <string>
#include <limits>

namespace autoware::image_projection_based_fusion
{

using autoware_perception_msgs::msg::ObjectClassification;

/**
 * @brief Size constraints for different object classes
 */
struct ObjectSizeConstraints
{
  double min_length;  // x dimension (front-back) [m]
  double max_length;
  double min_width;  // y dimension (side-to-side) [m]
  double max_width;
};

/**
 * @brief Default pedestrian size constraints based on real-world measurements
 */
inline ObjectSizeConstraints getDefaultPedestrianConstraints()
{
  ObjectSizeConstraints constraints;
  constraints.min_length = 0.2;             // Very thin from front
  constraints.max_length = 0.8;             // Wide stance or carrying items
  constraints.min_width = 0.3;              // Minimum shoulder width
  constraints.max_width = 1.0;              // With bags/umbrella
  return constraints;
}

/**
 * @brief Result of size validation
 */
struct SizeValidationResult
{
  bool is_valid = false;
  double size_score = 0.0;  // 0.0 to 1.0
};

/**
 * @brief Pedestrian size validation parameters
 */
struct PedestrianSizeValidationParams
{
  bool enable_size_validation = true;

  // 3D size constraints (x-y footprint only; no z-axis range)
  double min_width = 0.1;
  double max_width = 1.0;
};

/**
 * @brief Calculate 3D bounding box dimensions from pointcloud cluster
 * @param cluster PointCloud2 cluster data
 * @param length Output: length (x dimension) in meters
 * @param width Output: width (y dimension) in meters
 * @return True if dimensions were successfully calculated (x-y footprint only; no z-axis)
 */
inline bool calculateClusterDimensions(
  const sensor_msgs::msg::PointCloud2 & cluster, double & length, double & width)
{
  if (cluster.data.empty()) {
    return false;
  }

  // Initialize min/max values (x-y only)
  double min_x = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double min_y = std::numeric_limits<double>::max();
  double max_y = std::numeric_limits<double>::lowest();

  // Iterate through all points in the cluster
  size_t valid_points = 0;
  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(cluster, "x"),
       iter_y(cluster, "y"), iter_z(cluster, "z");
       iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    // Skip invalid points
    if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) {
      continue;
    }

    min_x = std::min(min_x, static_cast<double>(*iter_x));
    max_x = std::max(max_x, static_cast<double>(*iter_x));
    min_y = std::min(min_y, static_cast<double>(*iter_y));
    max_y = std::max(max_y, static_cast<double>(*iter_y));
    valid_points++;
  }

  if (valid_points == 0) {
    return false;
  }

  // Calculate dimensions (x-y footprint only)
  length = max_x - min_x;
  width = max_y - min_y;

  // Ensure non-zero dimensions
  if (length <= 0.0 || width <= 0.0) {
    return false;
  }

  return true;
}

/**
 * @brief Validate 3D size of a detected object against pedestrian constraints
 * @param cluster PointCloud2 cluster data to extract dimensions from
 * @param params Validation parameters
 * @return Validation result with score and rejection reason if invalid
 */
inline SizeValidationResult validatePedestrian3DSize(
  const sensor_msgs::msg::PointCloud2 & cluster,
  const PedestrianSizeValidationParams & params)
{
  SizeValidationResult result;

  // Calculate dimensions from pointcloud cluster (x-y footprint only)
  double length, width;
  if (!calculateClusterDimensions(cluster, length, width)) {
    result.is_valid = false;
    return result;
  }

  // Check width
  if (width < params.min_width) {
    result.is_valid = false;
    return result;
  }
  if (width > params.max_width) {
    result.is_valid = false;
    return result;
  }

  // Calculate size score based on how well width matches typical pedestrian
  const double typical_width = 0.5;
  const double width_score =
    1.0 - std::abs(width - typical_width) / (params.max_width - params.min_width);

  result.is_valid = true;
  result.size_score = std::max(0.0, width_score);
  return result;
}

/**
 * @brief Comprehensive pedestrian size validation combining 3D and 2D checks
 * @param cluster PointCloud2 cluster data to extract 3D dimensions from
 * @param cluster_roi The projected cluster ROI
 * @param image_roi The detected image ROI
 * @param params Validation parameters
 * @return True if the object passes pedestrian size validation
 */
inline bool validatePedestrianSize(
  const sensor_msgs::msg::PointCloud2 & cluster,
  [[maybe_unused]] const sensor_msgs::msg::RegionOfInterest & cluster_roi,
  const PedestrianSizeValidationParams & params)
{
  if (!params.enable_size_validation) {
    return true;  // Validation disabled
  }

  // Check 3D size from pointcloud cluster
  if (!cluster.data.empty()) {
    auto size_result = validatePedestrian3DSize(cluster, params);
    if (!size_result.is_valid) {
      return false;
    }
  }

  return true;
}

/**
 * @brief Check if the given label is a pedestrian
 * @param label Object classification label
 * @return True if the label is PEDESTRIAN
 */
inline bool isPedestrianLabel(const uint8_t label)
{
  return label == ObjectClassification::PEDESTRIAN;
}

}  // namespace autoware::image_projection_based_fusion

#endif  // AUTOWARE__IMAGE_PROJECTION_BASED_FUSION__UTILS__SIZE_VALIDATION_HPP_
