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

#include <algorithm>
#include <limits>
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
    // Skip normalization for ego_shape
    if (key == "ego_shape") {
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

}  // namespace autoware::diffusion_planner::preprocess
