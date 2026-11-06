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

#ifndef MARKER_DETECTOR_HPP_
#define MARKER_DETECTOR_HPP_

#include <autoware/landmark_manager/landmark_manager.hpp>
#include <autoware/point_types/types.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/header.hpp>

#include <pcl/point_cloud.h>

#include <cstdint>
#include <vector>

namespace autoware::lidar_marker_localizer
{

struct DetectionParam
{
  bool road_surface_mode;
  double resolution;
  std::vector<int64_t> intensity_pattern;
  int64_t match_intensity_difference_threshold;
  int64_t positive_match_num_threshold;
  int64_t negative_match_num_threshold;
  int64_t vote_threshold_for_detect_marker;
  double marker_to_vehicle_offset_y;
  double marker_height_from_ground;
  int64_t reference_ring_number;
  int64_t lower_ring_id_init;
  int64_t upper_ring_id_init;
};

enum class DetectionStatus { ok, no_points, bin_num_too_small };

struct DetectionResult
{
  DetectionStatus status{DetectionStatus::ok};
  // poses of the detected markers on base_link
  std::vector<landmark_manager::Landmark> landmarks;

  // for debug
  nav_msgs::msg::OccupancyGrid center_intensity_grid;
  nav_msgs::msg::OccupancyGrid positive_grid;
  nav_msgs::msg::OccupancyGrid negative_grid;
  nav_msgs::msg::OccupancyGrid matched_grid;
  nav_msgs::msg::OccupancyGrid vote_grid;
};

// Helper functions to get ring/channel ID from different point types
inline uint16_t get_ring_id(const autoware::point_types::PointXYZIRC & point)
{
  return point.channel;
}

inline uint16_t get_ring_id(const autoware::point_types::PointXYZIRADRT & point)
{
  return point.ring;
}

// Detect markers by matching the intensity of each ring along the x-axis of base_link with
// param.intensity_pattern. The points are assumed to be on base_link.
// Instantiated for autoware::point_types::PointXYZIRC and PointXYZIRADRT.
template <typename PointT>
DetectionResult detect_landmarks(
  const pcl::PointCloud<PointT> & points, const std_msgs::msg::Header & header,
  const DetectionParam & param);

}  // namespace autoware::lidar_marker_localizer

#endif  // MARKER_DETECTOR_HPP_
