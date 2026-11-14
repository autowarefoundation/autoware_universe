// Copyright 2026 TIER IV, Inc.
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

#ifndef AUTOWARE__CAMERA_STREAMPETR__EGO_POSE_BINDINGS_HPP_
#define AUTOWARE__CAMERA_STREAMPETR__EGO_POSE_BINDINGS_HPP_

#include <tf2/LinearMath/Transform.hpp>

#include <optional>
#include <utility>
#include <vector>

namespace autoware::camera_streampetr
{

// Row-major 4x4 float matrix, the layout of the head's pose bindings.
inline std::vector<float> to_row_major(const tf2::Transform & transform)
{
  const tf2::Matrix3x3 & rotation = transform.getBasis();
  const tf2::Vector3 & translation = transform.getOrigin();
  std::vector<float> matrix;
  matrix.reserve(16);
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      matrix.push_back(static_cast<float>(rotation[row][col]));
    }
    matrix.push_back(static_cast<float>(translation[row]));
  }
  matrix.insert(matrix.end(), {0.0f, 0.0f, 0.0f, 1.0f});
  return matrix;
}

// The head moves its memory by `data_ego_pose_inv` and stores it by `data_ego_pose`. Binding the
// identity and the motion since the previous inference frame keeps the memory in the previous
// vehicle frame, so every number in the head stays within the perception range. Map poses would
// make the head cancel translations as large as the distance driven, which an FP16 engine cannot
// represent: the memory drifts by meters a few kilometers out. The motion is composed in double
// precision; at the start of a stream, without a previous frame, it is the identity.
// Returns `data_ego_pose` and `data_ego_pose_inv`.
inline std::pair<std::vector<float>, std::vector<float>> compute_ego_pose_bindings(
  const std::optional<tf2::Transform> & previous_map_to_base,
  const tf2::Transform & current_map_to_base)
{
  const tf2::Transform motion = previous_map_to_base.has_value()
                                  ? current_map_to_base.inverse() * previous_map_to_base.value()
                                  : tf2::Transform::getIdentity();
  return {to_row_major(tf2::Transform::getIdentity()), to_row_major(motion)};
}

}  // namespace autoware::camera_streampetr

#endif  // AUTOWARE__CAMERA_STREAMPETR__EGO_POSE_BINDINGS_HPP_
