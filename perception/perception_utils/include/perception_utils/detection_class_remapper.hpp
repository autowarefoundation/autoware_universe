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

#ifndef PERCEPTION_UTILS__DETECTION_CLASS_REMAPPER_HPP_
#define PERCEPTION_UTILS__DETECTION_CLASS_REMAPPER_HPP_

#include <autoware_perception_msgs/msg/detected_objects.hpp>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace perception_utils
{

class DetectionClassRemapper
{
public:
  void setParameters(
    const std::vector<std::int64_t> & allow_remapping_by_area_matrix,
    const std::vector<double> & min_area_matrix, const std::vector<double> & max_area_matrix);

  void mapClasses(autoware_perception_msgs::msg::DetectedObjects & msg) const;

private:
  std::size_t matrixIndex(std::size_t source_label, std::size_t destination_label) const;

  std::vector<bool> allow_remapping_by_area_matrix_;
  std::vector<double> min_area_matrix_;
  std::vector<double> max_area_matrix_;
  std::size_t num_labels_{};
};

}  // namespace perception_utils

#endif  // PERCEPTION_UTILS__DETECTION_CLASS_REMAPPER_HPP_
