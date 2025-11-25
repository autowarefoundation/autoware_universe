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

#include "autoware/diffusion_planner/preprocessing/lane_segments.hpp"

#include "autoware/diffusion_planner/constants.hpp"
#include "autoware/diffusion_planner/dimensions.hpp"

#include <autoware_lanelet2_extension/regulatory_elements/road_marking.hpp>  // for lanelet::autoware::RoadMarking
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>

#include <Eigen/src/Core/Matrix.h>
#include <lanelet2_core/Forward.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <tuple>
#include <utility>
#include <vector>

namespace autoware::diffusion_planner::preprocess
{

// Internal functions declaration
namespace
{
using autoware_perception_msgs::msg::TrafficLightElement;

std::map<lanelet::Id, size_t> create_lane_id_to_array_index_map(
  const std::vector<LaneSegment> & lane_segments);
bool is_segment_inside(const LaneSegment & segment, const double center_x, const double center_y);
uint8_t identify_current_light_status(
  const int64_t turn_direction, const std::vector<TrafficLightElement> & traffic_light_elements);
}  // namespace

// LaneSegmentContext implementation
LaneSegmentContext::LaneSegmentContext(const std::shared_ptr<lanelet::LaneletMap> & lanelet_map_ptr)
: lanelet_map_(convert_to_internal_lanelet_map(lanelet_map_ptr)),
  lanelet_id_to_array_index_(create_lane_id_to_array_index_map(lanelet_map_.lane_segments))
{
  if (lanelet_map_.lane_segments.empty()) {
    throw std::runtime_error("No lane segments found in the map");
  }
}

std::vector<int64_t> LaneSegmentContext::select_route_segment_indices(
  const LaneletRoute & route, const double center_x, const double center_y,
  const int64_t max_segments) const
{
  std::vector<int64_t> array_indices;
  double closest_distance = std::numeric_limits<double>::max();
  size_t closest_index = 0;
  for (size_t i = 0; i < route.segments.size(); ++i) {
    // add index
    const int64_t lanelet_id = route.segments[i].preferred_primitive.id;
    if (lanelet_id_to_array_index_.count(lanelet_id) == 0) {
      continue;
    }
    const int64_t array_index = lanelet_id_to_array_index_.at(lanelet_id);
    array_indices.push_back(array_index);

    // calculate closest index
    const LaneSegment & route_segment = lanelet_map_.lane_segments[array_index];
    double distance = std::numeric_limits<double>::max();
    for (const LanePoint & point : route_segment.centerline) {
      const double diff_x = point.x() - center_x;
      const double diff_y = point.y() - center_y;
      const double curr_distance = std::sqrt(diff_x * diff_x + diff_y * diff_y);
      distance = std::min(distance, curr_distance);
    }
    if (distance < closest_distance) {
      closest_distance = distance;
      closest_index = i;
    }
  }

  std::vector<int64_t> selected_indices;

  // Select route segment indices
  for (size_t i = closest_index; i < array_indices.size(); ++i) {
    const int64_t segment_idx = array_indices[i];

    if (!is_segment_inside(lanelet_map_.lane_segments[segment_idx], center_x, center_y)) {
      continue;
    }

    selected_indices.push_back(segment_idx);
    if (selected_indices.size() >= static_cast<size_t>(max_segments)) {
      break;
    }
  }

  return selected_indices;
}

std::vector<int64_t> LaneSegmentContext::select_lane_segment_indices(
  const Eigen::Matrix4d & transform_matrix, const double center_x, const double center_y,
  const int64_t max_segments) const
{
  struct ColWithDistance
  {
    int64_t index;           //!< Column index in the input matrix.
    float distance_squared;  //!< Squared distance from the center.
  };

  auto calc_distance = [&](const LanePoint & point) {
    const Eigen::Vector4d transformed_point =
      transform_matrix * Eigen::Vector4d(point.x(), point.y(), point.z(), 1.0);
    const float diff_x = transformed_point.x();
    const float diff_y = transformed_point.y();
    return std::sqrt(diff_x * diff_x + diff_y * diff_y);
  };

  // Step 1: Compute distances
  std::vector<ColWithDistance> distances;
  distances.reserve(lanelet_map_.lane_segments.size());

  for (size_t i = 0; i < lanelet_map_.lane_segments.size(); ++i) {
    const LaneSegment & segment = lanelet_map_.lane_segments[i];

    if (!is_segment_inside(segment, center_x, center_y)) {
      continue;
    }

    const std::vector<LanePoint> & centerline = segment.centerline;

    // -1 is the same as next first point, so use -2
    const float distance_squared =
      std::min(calc_distance(centerline.front()), calc_distance(centerline[centerline.size() - 2]));
    distances.push_back({static_cast<int64_t>(i), distance_squared});
  }

  // Step 2: Sort indices by distance
  std::sort(distances.begin(), distances.end(), [](const auto & a, const auto & b) {
    return a.distance_squared < b.distance_squared;
  });

  // Step 3: Select indices that are inside the mask
  std::vector<int64_t> selected_indices;
  for (const ColWithDistance & distance : distances) {
    selected_indices.push_back(distance.index);
    if (selected_indices.size() >= static_cast<size_t>(max_segments)) {
      break;
    }
  }

  return selected_indices;
}

std::pair<std::vector<float>, std::vector<float>>
LaneSegmentContext::create_tensor_data_from_indices(
  const Eigen::Matrix4d & transform_matrix,
  const std::map<lanelet::Id, TrafficSignalStamped> & traffic_light_id_map,
  const std::vector<int64_t> & segment_indices, const int64_t max_segments) const
{
  const auto total_points = max_segments * POINTS_PER_SEGMENT;
  Eigen::MatrixXd output_matrix(SEGMENT_POINT_DIM, total_points);
  output_matrix.setZero();

  std::vector<float> speed_limit_vector(max_segments, 0.0f);

  auto convert_to_vector4d = [](const LanePoint & point) {
    return Eigen::Vector4d(point.x(), point.y(), point.z(), 1.0);
  };

  auto encode = [](const int64_t line_type) {
    Eigen::Vector<double, LINE_TYPE_NUM> one_hot = Eigen::Vector<double, LINE_TYPE_NUM>::Zero();
    if (line_type >= 0 && line_type < LINE_TYPE_NUM) {
      one_hot[line_type] = 1.0;
    }
    return one_hot;
  };

  int64_t added_segments = 0;
  for (const int64_t segment_idx : segment_indices) {
    if (added_segments >= max_segments) {
      break;
    }

    const LaneSegment & lane_segment = lanelet_map_.lane_segments[segment_idx];

    // Check if segment has valid data
    if (
      lane_segment.centerline.empty() || lane_segment.left_boundary.empty() ||
      lane_segment.right_boundary.empty()) {
      continue;
    }

    const std::vector<LanePoint> & centerline = lane_segment.centerline;
    const std::vector<LanePoint> & left_boundary = lane_segment.left_boundary;
    const std::vector<LanePoint> & right_boundary = lane_segment.right_boundary;

    if (
      centerline.size() != POINTS_PER_SEGMENT || left_boundary.size() != POINTS_PER_SEGMENT ||
      right_boundary.size() != POINTS_PER_SEGMENT) {
      continue;
    }

    const Eigen::Vector<double, TRAFFIC_LIGHT_ONE_HOT_DIM> traffic_light_one_hot_encoding = [&]() {
      Eigen::Vector<double, TRAFFIC_LIGHT_ONE_HOT_DIM> encoding =
        Eigen::Vector<double, TRAFFIC_LIGHT_ONE_HOT_DIM>::Zero();
      if (lane_segment.traffic_light_id == LaneSegment::TRAFFIC_LIGHT_ID_NONE) {
        encoding[TRAFFIC_LIGHT_NO_TRAFFIC_LIGHT - TRAFFIC_LIGHT] = 1.0;
        return encoding;
      }

      const auto traffic_light_stamped_info_itr =
        traffic_light_id_map.find(lane_segment.traffic_light_id);
      if (traffic_light_stamped_info_itr == traffic_light_id_map.end()) {
        encoding[TRAFFIC_LIGHT_WHITE - TRAFFIC_LIGHT] = 1.0;
        return encoding;
      }

      const auto & signal = traffic_light_stamped_info_itr->second.signal;
      const uint8_t traffic_color =
        identify_current_light_status(lane_segment.turn_direction, signal.elements);
      return Eigen::Vector<double, TRAFFIC_LIGHT_ONE_HOT_DIM>{
        traffic_color == TrafficLightElement::GREEN,    // 3
        traffic_color == TrafficLightElement::AMBER,    // 2
        traffic_color == TrafficLightElement::RED,      // 1
        traffic_color == TrafficLightElement::UNKNOWN,  // 0
        traffic_color == TrafficLightElement::WHITE     // 4
      };
    }();

    const Eigen::Vector<double, LINE_TYPE_NUM> lt_left = encode(lane_segment.left_line_type);
    const Eigen::Vector<double, LINE_TYPE_NUM> lt_right = encode(lane_segment.right_line_type);

    // Process each point in the segment
    for (int64_t i = 0; i < POINTS_PER_SEGMENT; ++i) {
      const int64_t col_idx = added_segments * POINTS_PER_SEGMENT + i;

      // Center (0, 1)
      const Eigen::Vector4d center = transform_matrix * convert_to_vector4d(centerline[i]);
      output_matrix(X, col_idx) = center.x();
      output_matrix(Y, col_idx) = center.y();

      // Direction (2, 3)
      if (i > 0) {
        const int64_t col_idx_p = added_segments * POINTS_PER_SEGMENT + (i - 1);
        output_matrix(dX, col_idx_p) = output_matrix(X, col_idx) - output_matrix(X, col_idx_p);
        output_matrix(dY, col_idx_p) = output_matrix(Y, col_idx) - output_matrix(Y, col_idx_p);
      }

      // Left (4, 5)
      const Eigen::Vector4d left = transform_matrix * convert_to_vector4d(left_boundary[i]);
      output_matrix(LB_X, col_idx) = left.x() - center.x();
      output_matrix(LB_Y, col_idx) = left.y() - center.y();

      // Right (6, 7)
      const Eigen::Vector4d right = transform_matrix * convert_to_vector4d(right_boundary[i]);
      output_matrix(RB_X, col_idx) = right.x() - center.x();
      output_matrix(RB_Y, col_idx) = right.y() - center.y();

      // Traffic Light (8-13)
      output_matrix.block<TRAFFIC_LIGHT_ONE_HOT_DIM, 1>(TRAFFIC_LIGHT, col_idx) =
        traffic_light_one_hot_encoding;

      // Left LineType (14-23)
      output_matrix.block<LINE_TYPE_NUM, 1>(LINE_TYPE_LEFT_START, col_idx) = lt_left;

      // Right LineType (24-33)
      output_matrix.block<LINE_TYPE_NUM, 1>(LINE_TYPE_RIGHT_START, col_idx) = lt_right;
    }

    speed_limit_vector[added_segments] = lane_segment.speed_limit_mps.value_or(0.0f);
    ++added_segments;
  }

  // Convert to float vector
  const Eigen::MatrixXf output_matrix_f = output_matrix.cast<float>().eval();
  std::vector<float> tensor_data(
    output_matrix_f.data(), output_matrix_f.data() + output_matrix_f.size());

  return {tensor_data, speed_limit_vector};
}

std::vector<float> LaneSegmentContext::create_line_tensor(
  const std::vector<std::vector<LanePoint>> & polylines, const Eigen::Matrix4d & transform_matrix,
  const double center_x, const double center_y, const int64_t num_elements,
  const int64_t num_points) const
{
  using autoware::diffusion_planner::constants::LANE_MASK_RANGE_M;

  auto judge_inside = [&](const double x, const double y) -> bool {
    return (
      x > center_x - LANE_MASK_RANGE_M && x < center_x + LANE_MASK_RANGE_M &&
      y > center_y - LANE_MASK_RANGE_M && y < center_y + LANE_MASK_RANGE_M);
  };

  struct PolylineWithDistance
  {
    Polyline polyline;
    double min_distance;
  };

  std::vector<PolylineWithDistance> result_list;

  for (const auto & polyline : polylines) {
    bool inside_at_least_one = false;
    for (const auto & point : polyline) {
      if (judge_inside(point.x(), point.y())) {
        inside_at_least_one = true;
        break;
      }
    }
    if (!inside_at_least_one) {
      continue;
    }

    std::vector<LanePoint> transformed_polyline;
    for (const auto & point : polyline) {
      const Eigen::Vector4d transformed_point =
        transform_matrix * Eigen::Vector4d(point.x(), point.y(), point.z(), 1.0);
      transformed_polyline.push_back(
        Eigen::Vector3d(transformed_point.x(), transformed_point.y(), transformed_point.z()));
    }

    double min_distance = std::numeric_limits<double>::max();
    for (const auto & point : transformed_polyline) {
      const double distance = std::sqrt(point.x() * point.x() + point.y() * point.y());
      min_distance = std::min(min_distance, distance);
    }

    result_list.push_back({transformed_polyline, min_distance});
  }

  std::sort(
    result_list.begin(), result_list.end(),
    [](const PolylineWithDistance & a, const PolylineWithDistance & b) {
      return a.min_distance < b.min_distance;
    });

  // Create tensor data
  std::vector<float> tensor_data(num_elements * num_points * 2, 0.0f);
  const size_t max_elements_size = std::min(static_cast<size_t>(num_elements), result_list.size());
  for (size_t i = 0; i < max_elements_size; ++i) {
    const auto & polyline = result_list[i].polyline;
    const size_t max_points_size = std::min(static_cast<size_t>(num_points), polyline.size());

    for (size_t j = 0; j < max_points_size; ++j) {
      const auto & point = polyline[j];
      const size_t base_index = (i * num_points + j) * 2;
      tensor_data[base_index + 0] = static_cast<float>(point.x());
      tensor_data[base_index + 1] = static_cast<float>(point.y());
    }
  }

  return tensor_data;
}

// Internal functions implementation
namespace
{

std::map<lanelet::Id, size_t> create_lane_id_to_array_index_map(
  const std::vector<LaneSegment> & lane_segments)
{
  std::map<lanelet::Id, size_t> lane_id_to_index;
  for (size_t i = 0; i < lane_segments.size(); ++i) {
    lane_id_to_index[lane_segments[i].id] = i;
  }
  return lane_id_to_index;
}

bool is_segment_inside(const LaneSegment & segment, const double center_x, const double center_y)
{
  for (const auto & point : segment.centerline) {
    if (
      std::abs(point.x() - center_x) <= autoware::diffusion_planner::constants::LANE_MASK_RANGE_M &&
      std::abs(point.y() - center_y) <= autoware::diffusion_planner::constants::LANE_MASK_RANGE_M) {
      return true;
    }
  }

  return false;
}

uint8_t identify_current_light_status(
  const int64_t turn_direction, const std::vector<TrafficLightElement> & traffic_light_elements)
{
  // Filter out ineffective elements (color == 0 which is UNKNOWN)
  std::vector<TrafficLightElement> effective_elements;
  for (const auto & element : traffic_light_elements) {
    if (element.color != TrafficLightElement::UNKNOWN) {
      effective_elements.push_back(element);
    }
  }

  // If no effective elements, return UNKNOWN (0)
  if (effective_elements.empty()) {
    return TrafficLightElement::UNKNOWN;
  }

  // If only one effective element, return its color
  if (effective_elements.size() == 1) {
    return effective_elements[0].color;
  }

  // For multiple elements, find the one that matches the turn direction
  // Map turn direction to corresponding arrow shape
  const std::map<int64_t, uint8_t> direction_to_shape_map = {
    {LaneSegment::TURN_DIRECTION_NONE, TrafficLightElement::UNKNOWN},       // none
    {LaneSegment::TURN_DIRECTION_STRAIGHT, TrafficLightElement::UP_ARROW},  // straight
    {LaneSegment::TURN_DIRECTION_LEFT, TrafficLightElement::LEFT_ARROW},    // left
    {LaneSegment::TURN_DIRECTION_RIGHT, TrafficLightElement::RIGHT_ARROW}   // right
  };

  const auto target_shape_iter = direction_to_shape_map.find(turn_direction);
  const uint8_t target_shape = (target_shape_iter != direction_to_shape_map.end())
                                 ? target_shape_iter->second
                                 : TrafficLightElement::UNKNOWN;

  // If multiple matching elements, take the one with highest confidence
  auto get_max_confidence_color = [](const std::vector<TrafficLightElement> & elements) {
    return std::max_element(
             elements.begin(), elements.end(),
             [](const TrafficLightElement & a, const TrafficLightElement & b) {
               return a.confidence < b.confidence;
             })
      ->color;
  };

  // First priority: Find elements with exactly matching direction
  std::vector<TrafficLightElement> matching_elements;
  for (const TrafficLightElement & element : effective_elements) {
    if (element.shape == target_shape) {
      matching_elements.push_back(element);
    }
  }
  if (!matching_elements.empty()) {
    return get_max_confidence_color(matching_elements);
  }

  // Second priority: Find circle elements
  std::vector<TrafficLightElement> circle_elements;
  for (const TrafficLightElement & element : effective_elements) {
    if (element.shape == TrafficLightElement::CIRCLE) {
      circle_elements.push_back(element);
    }
  }
  if (!circle_elements.empty()) {
    return get_max_confidence_color(circle_elements);
  }

  // If no matching direction or circle, return the element with highest confidence
  return get_max_confidence_color(effective_elements);
}

}  // namespace

}  // namespace autoware::diffusion_planner::preprocess
