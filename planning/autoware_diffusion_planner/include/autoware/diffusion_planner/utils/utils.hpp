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

#ifndef AUTOWARE__DIFFUSION_PLANNER__UTILS__UTILS_HPP_
#define AUTOWARE__DIFFUSION_PLANNER__UTILS__UTILS_HPP_

#include <Eigen/Dense>

#include "nav_msgs/msg/odometry.hpp"

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::diffusion_planner::utils
{

/**
 * @brief Generates transformation matrices from an odometry message.
 *
 * @param msg Odometry message containing position and orientation data.
 * @return A pair of 4x4 transformation matrices:
 *         - The first matrix represents the transformation from the map frame to the ego frame.
 *         - The second matrix represents the inverse transformation (ego frame to map frame).
 */
std::pair<Eigen::Matrix4f, Eigen::Matrix4f> get_transform_matrix(
  const nav_msgs::msg::Odometry & msg);

/**
 * @brief Creates a vector of floats initialized with a specific value.
 *
 * @param shape A vector specifying the dimensions of the data (e.g., rows, columns).
 * @param fill The value to initialize the vector with. Defaults to 1.0f.
 * @return A flattened vector of floats with the specified shape and initialized values.
 */
std::vector<float> create_float_data(const std::vector<int64_t> & shape, float fill = 1.0f);

/**
 * @brief Checks if the input map contains valid data.
 *
 * @param input_map An unordered_map with string keys and vector<float> values.
 * @return True if the input map is valid, false otherwise.
 */
bool check_input_map(const std::unordered_map<std::string, std::vector<float>> & input_map);

/**
 * @brief Converts a geometry_msgs::msg::Pose to a 4x4 transformation matrix.
 *
 * @param pose The pose containing position and orientation information.
 * @return A 4x4 transformation matrix representing the pose.
 */
Eigen::Matrix4f pose_to_matrix4f(const geometry_msgs::msg::Pose & pose);

/**
 * @brief Extracts yaw angle from rotation matrix and converts to cos/sin representation.
 *
 * @param rotation_matrix 3x3 rotation matrix.
 * @return A pair containing cos(yaw) and sin(yaw).
 */
std::pair<float, float> rotation_matrix_to_cos_sin(const Eigen::Matrix3f & rotation_matrix);

}  // namespace autoware::diffusion_planner::utils
#endif  // AUTOWARE__DIFFUSION_PLANNER__UTILS__UTILS_HPP_
