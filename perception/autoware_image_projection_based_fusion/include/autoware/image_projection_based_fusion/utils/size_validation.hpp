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
  double min_height;  // z dimension [m]
  double max_height;
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
  constraints.min_height = 0.8;             // Child or crouching
  constraints.max_height = 2.2;             // Tall adult
  return constraints;
}

/**
 * @brief Result of size validation
 */
struct SizeValidationResult
{
  bool is_valid = false;
  double size_score = 0.0;  // 0.0 to 1.0
  std::string rejection_reason;
};

/**
 * @brief Result of aspect ratio validation
 */
struct AspectRatioValidationResult
{
  bool is_valid = false;
  double aspect_ratio = 0.0;
  double confidence = 0.0;
};

/**
 * @brief Pedestrian size validation parameters
 */
struct PedestrianSizeValidationParams
{
  bool enable_size_validation = true;
  bool enable_aspect_ratio_validation = true;
  bool enable_3d_size_validation = true;

  // 3D size constraints
  double min_height = 0.8;
  double max_height = 2.2;
  double min_width = 0.3;
  double max_width = 1.0;
};

/**
 * @brief Calculate 3D bounding box dimensions from pointcloud cluster
 * @param cluster PointCloud2 cluster data
 * @param length Output: length (x dimension) in meters
 * @param width Output: width (y dimension) in meters
 * @param height Output: height (z dimension) in meters
 * @return True if dimensions were successfully calculated
 */
inline bool calculateClusterDimensions(
  const sensor_msgs::msg::PointCloud2 & cluster, double & length, double & width, double & height)
{
  if (cluster.data.empty()) {
    return false;
  }

  // Initialize min/max values
  double min_x = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double min_y = std::numeric_limits<double>::max();
  double max_y = std::numeric_limits<double>::lowest();
  double min_z = std::numeric_limits<double>::max();
  double max_z = std::numeric_limits<double>::lowest();

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
    min_z = std::min(min_z, static_cast<double>(*iter_z));
    max_z = std::max(max_z, static_cast<double>(*iter_z));
    valid_points++;
  }

  if (valid_points == 0) {
    return false;
  }

  // Calculate dimensions
  length = max_x - min_x;
  width = max_y - min_y;
  height = max_z - min_z;

  // Ensure non-zero dimensions
  if (length <= 0.0 || width <= 0.0 || height <= 0.0) {
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

  // Calculate dimensions from pointcloud cluster
  double length, width, height;
  if (!calculateClusterDimensions(cluster, length, width, height)) {
    result.is_valid = false;
    result.rejection_reason = "Failed to calculate cluster dimensions from pointcloud";
    return result;
  }

  // Check height
  if (height < params.min_height) {
    result.is_valid = false;
    result.rejection_reason =
      "Height too small: " + std::to_string(height) + "m < " + std::to_string(params.min_height) + "m";
    return result;
  }
  if (height > params.max_height) {
    result.is_valid = false;
    result.rejection_reason =
      "Height too large: " + std::to_string(height) + "m > " + std::to_string(params.max_height) + "m";
    return result;
  }

  // Check width
  if (width < params.min_width) {
    result.is_valid = false;
    result.rejection_reason =
      "Width too small: " + std::to_string(width) + "m < " + std::to_string(params.min_width) + "m";
    return result;
  }
  if (width > params.max_width) {
    result.is_valid = false;
    result.rejection_reason =
      "Width too large: " + std::to_string(width) + "m > " + std::to_string(params.max_width) + "m";
    return result;
  }

  // Check that pedestrian is upright (height should be greater than length and width)
  if (height < length || height < width) {
    result.is_valid = false;
    result.rejection_reason = "Not upright: height=" + std::to_string(height) +
                              "m, length=" + std::to_string(length) +
                              "m, width=" + std::to_string(width) + "m";
    return result;
  }

  // Calculate size score based on how well it matches typical pedestrian dimensions
  // Typical adult: ~0.5m length, ~0.5m width, ~1.7m height
  const double typical_height = 1.7;
  const double typical_width = 0.5;

  const double height_score =
    1.0 - std::abs(height - typical_height) / (params.max_height - params.min_height);
  const double width_score =
    1.0 - std::abs(width - typical_width) / (params.max_width - params.min_width);

  result.is_valid = true;
  result.size_score = std::max(0.0, (height_score + width_score) / 2.0);
  return result;
}

/**
 * @brief Validate 2D aspect ratio of ROI against pedestrian constraints
 * @param roi The region of interest to validate
 * @param params Validation parameters
 * @return Validation result with aspect ratio and confidence
 */
inline AspectRatioValidationResult validatePedestrianAspectRatio(
  const sensor_msgs::msg::RegionOfInterest & roi, const PedestrianSizeValidationParams & params)
{
  AspectRatioValidationResult result;

  if (roi.width == 0 || roi.height == 0) {
    result.is_valid = false;
    return result;
  }

  // Calculate aspect ratio (height / width)
  result.aspect_ratio =
    static_cast<double>(roi.height) / static_cast<double>(roi.width);

  // Calculate confidence based on how typical the aspect ratio is
  // For pedestrians, aspect ratio around 2.0-3.0 is most typical (full body visible)
  const double ideal_pedestrian_aspect_ratio = 2.5;
  const double deviation = std::abs(result.aspect_ratio - ideal_pedestrian_aspect_ratio);
  result.confidence = std::max(0.0, 1.0 - deviation / 2.0);

  result.is_valid = true;
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
  const sensor_msgs::msg::RegionOfInterest & cluster_roi,
  const sensor_msgs::msg::RegionOfInterest & image_roi,
  const PedestrianSizeValidationParams & params)
{
  if (!params.enable_size_validation) {
    return true;  // Validation disabled
  }

  // Check 2D aspect ratio (most important for pedestrians)
  if (params.enable_aspect_ratio_validation) {
    // Validate cluster ROI aspect ratio
    auto cluster_aspect_result = validatePedestrianAspectRatio(cluster_roi, params);
    if (!cluster_aspect_result.is_valid) {
      return false;
    }

    // Validate image ROI aspect ratio
    auto image_aspect_result = validatePedestrianAspectRatio(image_roi, params);
    if (!image_aspect_result.is_valid) {
      return false;
    }
  }

  // Check 3D size from pointcloud cluster
  if (params.enable_3d_size_validation) {
    if (!cluster.data.empty()) {
      auto size_result = validatePedestrian3DSize(cluster, params);
      if (!size_result.is_valid) {
        return false;
      }
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
