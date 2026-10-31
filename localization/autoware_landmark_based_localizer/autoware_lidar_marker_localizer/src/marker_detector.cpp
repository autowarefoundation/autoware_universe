// Copyright 2026 Autoware Foundation
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

#include "marker_detector.hpp"

#include <autoware_utils/geometry/geometry.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace autoware::lidar_marker_localizer
{

template <typename PointT>
DetectionResult detect_landmarks(
  const pcl::PointCloud<PointT> & points, const std_msgs::msg::Header & header,
  const DetectionParam & param)
{
  // TODO(YamatoAndo)
  // Transform sensor_frame to base_link
  DetectionResult result;

  if (points.empty()) {
    result.status = DetectionStatus::no_points;
    return result;
  }

  // Use parameters for initial lower/upper ring id
  uint16_t lower_ring_id = static_cast<uint16_t>(param.lower_ring_id_init);
  uint16_t upper_ring_id = static_cast<uint16_t>(param.upper_ring_id_init);
  for (const auto & point : points.points) {
    lower_ring_id = std::min(get_ring_id(point), lower_ring_id);
    upper_ring_id = std::max(get_ring_id(point), upper_ring_id);
  }
  uint16_t ring_num = upper_ring_id - lower_ring_id + 1;

  std::vector<pcl::PointCloud<PointT>> ring_points(ring_num);

  float min_x = std::numeric_limits<float>::max();
  float max_x = std::numeric_limits<float>::lowest();
  for (const auto & point : points.points) {
    ring_points[get_ring_id(point) - lower_ring_id].push_back(point);
    min_x = std::min(min_x, point.x);
    max_x = std::max(max_x, point.x);
  }

  // Check that the leaf size is not too small, given the size of the data
  const int bin_num = static_cast<int>((max_x - min_x) / param.resolution + 1);

  if (bin_num < static_cast<int>(param.intensity_pattern.size())) {
    result.status = DetectionStatus::bin_num_too_small;
    return result;
  }

  // initialize variables
  constexpr int max_vote_percentage = 100;  // Maximum vote percentage for visualization
  std::vector<int> vote(bin_num, 0);
  std::vector<float> reference_ring_y(bin_num, std::numeric_limits<float>::max());

  nav_msgs::msg::OccupancyGrid center_intensity_grid_msg;
  center_intensity_grid_msg.header = header;
  center_intensity_grid_msg.header.frame_id = "base_link";
  center_intensity_grid_msg.info.map_load_time = center_intensity_grid_msg.header.stamp;
  center_intensity_grid_msg.info.resolution = param.resolution;
  center_intensity_grid_msg.info.width = bin_num;
  center_intensity_grid_msg.info.height = ring_num;
  center_intensity_grid_msg.info.origin.position.x = min_x;
  if (param.road_surface_mode) {
    center_intensity_grid_msg.info.origin.position.y =
      param.marker_to_vehicle_offset_y -
      center_intensity_grid_msg.info.height * center_intensity_grid_msg.info.resolution / 2.0;
    center_intensity_grid_msg.info.origin.position.z = param.marker_height_from_ground;
    if (param.marker_to_vehicle_offset_y >= 0) {
      center_intensity_grid_msg.info.origin.position.y +=
        center_intensity_grid_msg.info.height * center_intensity_grid_msg.info.resolution;
      center_intensity_grid_msg.info.origin.orientation =
        autoware_utils_geometry::create_quaternion_from_rpy(M_PI, 0.0, 0.0);
    } else {
      center_intensity_grid_msg.info.origin.orientation =
        autoware_utils_geometry::create_quaternion_from_rpy(0.0, 0.0, 0.0);
    }
  } else {
    center_intensity_grid_msg.info.origin.position.y = param.marker_to_vehicle_offset_y;
    center_intensity_grid_msg.info.origin.position.z =
      param.marker_height_from_ground +
      center_intensity_grid_msg.info.height * center_intensity_grid_msg.info.resolution / 2.0;
    center_intensity_grid_msg.info.origin.orientation =
      autoware_utils_geometry::create_quaternion_from_rpy(-M_PI / 2.0, 0.0, 0.0);
  }
  center_intensity_grid_msg.data = std::vector<int8_t>(
    center_intensity_grid_msg.info.width * center_intensity_grid_msg.info.height, -1);

  nav_msgs::msg::OccupancyGrid positive_grid_msg;
  positive_grid_msg = center_intensity_grid_msg;

  nav_msgs::msg::OccupancyGrid negative_grid_msg;
  negative_grid_msg = center_intensity_grid_msg;

  nav_msgs::msg::OccupancyGrid matched_grid_msg;
  matched_grid_msg = center_intensity_grid_msg;

  // for each ring
  for (const auto & one_ring : ring_points) {
    if (one_ring.empty()) {
      continue;
    }

    std::vector<double> intensity_sum(bin_num, 0.0);
    std::vector<int> intensity_num(bin_num, 0);
    std::vector<double> average_intensity(bin_num, 0.0);
    const size_t ring_id =
      get_ring_id(one_ring.front()) - get_ring_id(ring_points.front().points.front());

    for (const auto & point : one_ring.points) {
      const int bin_index = static_cast<int>((point.x - min_x) / param.resolution);
      intensity_sum[bin_index] += point.intensity;
      intensity_num[bin_index]++;
      if (
        (get_ring_id(point) == param.reference_ring_number) ||
        (param.reference_ring_number == std::numeric_limits<uint8_t>::max())) {
        reference_ring_y[bin_index] = std::min(reference_ring_y[bin_index], point.y);
      }
    }

    // calc average
    for (int bin_index = 0; bin_index < bin_num; bin_index++) {
      if (intensity_num[bin_index] == 0) {
        continue;
      }

      average_intensity[bin_index] = intensity_sum[bin_index] / intensity_num[bin_index];
    }

    // pattern matching
    for (size_t i = 0; i <= bin_num - param.intensity_pattern.size(); i++) {
      int64_t pos = 0;
      int64_t neg = 0;
      double min_intensity = std::numeric_limits<double>::max();
      double max_intensity = std::numeric_limits<double>::lowest();

      // find max_min
      for (size_t j = 0; j < param.intensity_pattern.size(); j++) {
        if (intensity_num[i + j] == 0) {
          continue;
        }

        min_intensity = std::min(min_intensity, average_intensity[i + j]);
        max_intensity = std::max(max_intensity, average_intensity[i + j]);
      }

      if (max_intensity <= min_intensity) {
        continue;
      }

      const double center_intensity = (max_intensity - min_intensity) / 2.0 + min_intensity;

      for (size_t j = 0; j < param.intensity_pattern.size(); j++) {
        if (intensity_num[i + j] == 0) {
          continue;
        }

        if (param.intensity_pattern[j] == 1) {
          // check positive
          if (
            average_intensity[i + j] >
            center_intensity + static_cast<double>(param.match_intensity_difference_threshold)) {
            pos++;
          }
        } else if (param.intensity_pattern[j] == -1) {
          // check negative
          if (
            average_intensity[i + j] <
            center_intensity - static_cast<double>(param.match_intensity_difference_threshold)) {
            neg++;
          }
        } else {
          // ignore param.intensity_pattern[j] == 0
        }
      }
      const size_t bin_position = i + param.intensity_pattern.size() / 2 + ring_id * bin_num;
      center_intensity_grid_msg.data[bin_position] =
        std::min(static_cast<int>(center_intensity), max_vote_percentage);
      positive_grid_msg.data[bin_position] = std::min(
        static_cast<int>(pos * (max_vote_percentage / param.positive_match_num_threshold)),
        max_vote_percentage);
      negative_grid_msg.data[bin_position] = std::min(
        static_cast<int>(neg * (max_vote_percentage / param.negative_match_num_threshold)),
        max_vote_percentage);

      if (pos >= param.positive_match_num_threshold && neg >= param.negative_match_num_threshold) {
        matched_grid_msg.data[bin_position] = max_vote_percentage;
        vote[i]++;
      }
    }
  }

  std::vector<landmark_manager::Landmark> detected_landmarks;

  for (size_t i = 0; i < bin_num - param.intensity_pattern.size(); i++) {
    if (vote[i] > param.vote_threshold_for_detect_marker) {
      const double bin_position =
        static_cast<double>(i) + static_cast<double>(param.intensity_pattern.size()) / 2.0;
      geometry_msgs::msg::Pose marker_pose_on_base_link;
      marker_pose_on_base_link.position.x = bin_position * param.resolution + min_x;
      if (param.road_surface_mode) {
        marker_pose_on_base_link.position.y = param.marker_to_vehicle_offset_y;
      } else {
        marker_pose_on_base_link.position.y = reference_ring_y[i];
      }
      marker_pose_on_base_link.position.z = param.marker_height_from_ground;
      marker_pose_on_base_link.orientation =
        autoware_utils::create_quaternion_from_rpy(M_PI_2, 0.0, 0.0);  // TODO(YamatoAndo)
      detected_landmarks.push_back(landmark_manager::Landmark{"0", marker_pose_on_base_link});
    }
  }

  nav_msgs::msg::OccupancyGrid vote_grid_msg;
  vote_grid_msg.header = header;
  vote_grid_msg.header.frame_id = "base_link";
  vote_grid_msg.info.map_load_time = vote_grid_msg.header.stamp;
  vote_grid_msg.info.resolution = param.resolution;
  vote_grid_msg.info.width = bin_num;
  vote_grid_msg.info.height = 1;
  vote_grid_msg.info.origin.position.x = min_x;
  vote_grid_msg.info.origin.position.y = (param.marker_to_vehicle_offset_y >= 0.0) ? 1.0 : -1.0;
  vote_grid_msg.info.origin.position.z = 0.0;
  vote_grid_msg.info.origin.orientation.x = 0.0;
  vote_grid_msg.info.origin.orientation.y = 0.0;
  vote_grid_msg.info.origin.orientation.z = 0.0;
  vote_grid_msg.info.origin.orientation.w = 1.0;
  vote_grid_msg.data = std::vector<int8_t>(vote_grid_msg.info.width * vote_grid_msg.info.height);

  int vote_max = 1;  // NOTE: set non-zero to avoid zero division error
  for (int i = 0; i < bin_num; i++) {
    vote_max = std::max(vote[i], vote_max);
  }

  for (size_t i = 0; i < bin_num - param.intensity_pattern.size(); i++) {
    const size_t bin_position = i + param.intensity_pattern.size() / 2;
    vote_grid_msg.data[bin_position] = vote[i] * (max_vote_percentage / vote_max);
  }

  result.landmarks = std::move(detected_landmarks);
  result.center_intensity_grid = std::move(center_intensity_grid_msg);
  result.positive_grid = std::move(positive_grid_msg);
  result.negative_grid = std::move(negative_grid_msg);
  result.matched_grid = std::move(matched_grid_msg);
  result.vote_grid = std::move(vote_grid_msg);
  return result;
}

template DetectionResult detect_landmarks<autoware::point_types::PointXYZIRC>(
  const pcl::PointCloud<autoware::point_types::PointXYZIRC> & points,
  const std_msgs::msg::Header & header, const DetectionParam & param);
template DetectionResult detect_landmarks<autoware::point_types::PointXYZIRADRT>(
  const pcl::PointCloud<autoware::point_types::PointXYZIRADRT> & points,
  const std_msgs::msg::Header & header, const DetectionParam & param);

}  // namespace autoware::lidar_marker_localizer
