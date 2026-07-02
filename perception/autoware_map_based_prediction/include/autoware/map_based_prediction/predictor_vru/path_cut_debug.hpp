// Copyright 2026 TIER IV, inc.
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

#ifndef AUTOWARE__MAP_BASED_PREDICTION__PREDICTOR_VRU__PATH_CUT_DEBUG_HPP_
#define AUTOWARE__MAP_BASED_PREDICTION__PREDICTOR_VRU__PATH_CUT_DEBUG_HPP_

#include "autoware/map_based_prediction/path_generator/path_generator.hpp"

#include <rclcpp/time.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <cstdint>
#include <string>
#include <vector>

namespace autoware::map_based_prediction
{

enum class PathCutSource { Vegetation, Fence };

struct PathCutDebug
{
  bool cut{false};
  std::vector<geometry_msgs::msg::Point> display_points;
  std::vector<geometry_msgs::msg::Point> kept_points;
};

struct PathCutEvent
{
  std::string object_id;
  uint8_t label{0};
  geometry_msgs::msg::Pose object_pose{};
  PathCutDebug path_debug;
  geometry_msgs::msg::Point cut_pose{};
};

namespace debug
{

void append_path_cut_event_markers(
  visualization_msgs::msg::MarkerArray * debug_markers, const PredictedPath & original_path,
  const PredictedPath & cut_path, const autoware_perception_msgs::msg::PredictedObject & object,
  PathCutSource source, const rclcpp::Time & stamp);

void append_path_cut_event_markers(
  visualization_msgs::msg::MarkerArray * debug_markers,
  const std::vector<PredictedPath> & original_paths, const std::vector<PredictedPath> & cut_paths,
  const autoware_perception_msgs::msg::PredictedObject & object, PathCutSource source,
  const rclcpp::Time & stamp);

}  // namespace debug

}  // namespace autoware::map_based_prediction

#endif  // AUTOWARE__MAP_BASED_PREDICTION__PREDICTOR_VRU__PATH_CUT_DEBUG_HPP_
