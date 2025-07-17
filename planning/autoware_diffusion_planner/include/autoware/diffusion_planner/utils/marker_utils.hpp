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

#ifndef AUTOWARE__DIFFUSION_PLANNER__UTILS__MARKER_UTILS_HPP_
#define AUTOWARE__DIFFUSION_PLANNER__UTILS__MARKER_UTILS_HPP_

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <std_msgs/msg/detail/color_rgba__struct.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace autoware::diffusion_planner::utils
{
using rclcpp::Duration;
using rclcpp::Time;
using std_msgs::msg::ColorRGBA;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

/**
 * @brief Determines the appropriate traffic light color based on the given green, yellow, and red
 * values.
 *
 * This function selects and returns a ColorRGBA value representing the traffic light color
 * according to the provided intensity values for green, yellow, and red. The original_color
 * parameter serves as a fallback or reference color.
 *
 * @param g The intensity value for green (typically in the range [0.0, 1.0]).
 * @param y The intensity value for yellow (typically in the range [0.0, 1.0]).
 * @param r The intensity value for red (typically in the range [0.0, 1.0]).
 * @param original_color The original ColorRGBA to use as a reference or fallback.
 * @return ColorRGBA The resulting color representing the traffic light state.
 */
ColorRGBA get_traffic_light_color(float g, float y, float r, const ColorRGBA & original_color);

/**
 * @brief Creates a visualization marker array representing a lane.
 *
 * This function generates a MarkerArray for visualizing a lane using the provided lane vector and
 * shape. The markers can be customized with color, frame, and lifetime, and optionally colored for
 * traffic lights.
 *
 * @param lane_vector A vector of floats representing the lane geometry or points.
 * @param shape A vector of longs specifying the shape or dimensions of the lane data.
 * @param stamp The timestamp to assign to the marker messages.
 * @param lifetime The duration for which the markers should remain visible.
 * @param colors An array of 4 floats specifying the RGBA color of the lane markers (default: green
 * with alpha 0.8).
 * @param frame_id The coordinate frame in which to publish the markers (default: "base_link").
 * @param set_traffic_light_color If true, sets the marker color based on traffic light state
 * (default: false).
 * @return MarkerArray containing the generated lane markers.
 */
MarkerArray create_lane_marker(
  const Eigen::Matrix4f & transform_ego_to_map, const std::vector<float> & lane_vector,
  const std::vector<int64_t> & shape, const Time & stamp, const rclcpp::Duration & lifetime,
  const std::array<float, 4> colors = {0.0f, 1.0f, 0.0f, 0.8f},
  const std::string & frame_id = "base_link", const bool set_traffic_light_color = false);
}  // namespace autoware::diffusion_planner::utils
#endif  // AUTOWARE__DIFFUSION_PLANNER__UTILS__MARKER_UTILS_HPP_
