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

#include "autoware/diffusion_planner/preprocessing/preprocessing_utils.hpp"

#include "autoware/diffusion_planner/dimensions.hpp"
#include "autoware/diffusion_planner/utils/utils.hpp"

#include <algorithm>
#include <deque>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::diffusion_planner::preprocess
{
void normalize_input_data(InputDataMap & input_data_map, const NormalizationMap & normalization_map)
{
  auto normalize_vector = [](
                            std::vector<float> & data, const std::vector<float> & mean,
                            const std::vector<float> & std_dev) -> void {
    assert(!data.empty() && "Data vector must not be empty");
    assert((mean.size() == std_dev.size()) && "Mean and std must be same size");
    assert((data.size() % std_dev.size() == 0) && "Data size must be divisible by std_dev size");
    auto cols = std_dev.size();
    auto rows = data.size() / cols;

    for (size_t row = 0; row < rows; ++row) {
      const auto offset = row * cols;
      const auto row_begin =
        data.begin() + static_cast<std::vector<float>::difference_type>(offset);

      bool is_zero_row = std::all_of(
        row_begin, row_begin + static_cast<std::vector<float>::difference_type>(cols),
        [](float x) { return std::abs(x) < std::numeric_limits<float>::epsilon(); });

      if (is_zero_row) continue;

      for (size_t col = 0; col < cols; ++col) {
        float m = (mean.size() == 1) ? mean[0] : mean[col];
        float s = (std_dev.size() == 1) ? std_dev[0] : std_dev[col];
        // Prevent division by zero
        if (std::abs(s) < std::numeric_limits<float>::epsilon()) {
          throw std::runtime_error("Standard deviation is zero, cannot normalize data");
        }
        data[offset + col] = (data[offset + col] - m) / s;
      }
    }
  };

  for (auto & [key, value] : input_data_map) {
    // Skip normalization for ego_shape and sampled_trajectories
    if (key == "ego_shape" || key == "sampled_trajectories") {
      continue;
    }

    if (normalization_map.find(key) == normalization_map.end()) {
      std::string err{"Missing key " + key + " from normalization map"};
      throw std::runtime_error(err.c_str());
    }

    const auto & [mean, std_dev] = normalization_map.at(key);
    normalize_vector(value, mean, std_dev);
  }
}

std::vector<float> create_ego_agent_past(
  const std::deque<geometry_msgs::msg::Pose> & pose_msgs, size_t num_timesteps,
  const Eigen::Matrix4d & map_to_ego_transform)
{
  const size_t features_per_timestep = 4;  // x, y, cos, sin
  const size_t total_size = num_timesteps * features_per_timestep;

  std::vector<float> ego_agent_past(total_size, 0.0f);

  const size_t start_idx =
    (pose_msgs.size() >= num_timesteps) ? pose_msgs.size() - num_timesteps : 0;

  for (size_t i = start_idx; i < pose_msgs.size(); ++i) {
    const auto & historical_pose = pose_msgs[i];

    // Convert pose to 4x4 matrix
    const Eigen::Matrix4d pose_map_4x4 = utils::pose_to_matrix4f(historical_pose);

    // Transform to ego frame
    const Eigen::Matrix4d pose_ego_4x4 = map_to_ego_transform * pose_map_4x4;

    // Extract position
    const float x = pose_ego_4x4(0, 3);
    const float y = pose_ego_4x4(1, 3);

    // Extract heading as cos/sin
    const auto [cos_yaw, sin_yaw] =
      utils::rotation_matrix_to_cos_sin(pose_ego_4x4.block<3, 3>(0, 0));

    // Store in flat array: [timestep, features]
    const size_t timestep_idx = i - start_idx;
    const size_t base_idx = timestep_idx * features_per_timestep;
    ego_agent_past[base_idx + EGO_AGENT_PAST_IDX_X] = x;
    ego_agent_past[base_idx + EGO_AGENT_PAST_IDX_Y] = y;
    ego_agent_past[base_idx + EGO_AGENT_PAST_IDX_COS] = cos_yaw;
    ego_agent_past[base_idx + EGO_AGENT_PAST_IDX_SIN] = sin_yaw;
  }

  return ego_agent_past;
}

std::vector<float> create_sampled_trajectories(const double temperature)
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<float> dist(0.0f, 1.0f);
  std::vector<float> sampled_trajectories((MAX_NUM_NEIGHBORS + 1) * (OUTPUT_T + 1) * POSE_DIM);
  for (float & val : sampled_trajectories) {
    val = dist(gen) * static_cast<float>(temperature);
  }
  return sampled_trajectories;
}

}  // namespace autoware::diffusion_planner::preprocess
