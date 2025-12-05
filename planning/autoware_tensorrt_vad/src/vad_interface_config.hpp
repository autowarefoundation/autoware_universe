// Copyright 2025 TIER IV.
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

#ifndef VAD_INTERFACE_CONFIG_HPP_
#define VAD_INTERFACE_CONFIG_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <tf2_ros/buffer.h>

#include <array>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::tensorrt_vad
{

class VadInterfaceConfig
{
public:
  int32_t target_image_width;
  int32_t target_image_height;
  std::array<float, 6> detection_range;
  int32_t default_command;
  std::map<std::string, std::array<float, 3>> map_colors;  // Map type to RGB color
  std::vector<std::string> class_mapping;  // VAD class index to Autoware class name mapping (array
                                           // index = VAD class index)
  std::vector<std::string> bbox_class_names;  // Object class names from VAD model

  // ROS 2 parameter types: double and int64_t used for vector parameters
  // (ROS 2's declare_parameter does not accept std::vector<float> or std::vector<int32_t>)
  VadInterfaceConfig(
    int32_t target_image_width_, int32_t target_image_height_,
    const std::vector<double> & detection_range_, int32_t default_command_,
    const std::vector<std::string> & map_classes_, const std::vector<double> & map_colors_,
    const std::vector<std::string> & class_mapping_,
    const std::vector<std::string> & bbox_class_names_)
  : target_image_width(target_image_width_),
    target_image_height(target_image_height_),
    default_command(default_command_),
    class_mapping(class_mapping_),
    bbox_class_names(bbox_class_names_)
  {
    // detection_range: 6 elements
    for (int i = 0; i < 6; ++i) {
      detection_range[i] = static_cast<float>(detection_range_[i]);
    }
    // map_colors: convert from vector<string> (classes) and vector<double> (colors) to map of
    // array<float, 3>
    map_colors.clear();
    // Format: [class1_r, class1_g, class1_b, class2_r, class2_g, class2_b, ...]
    // Each class gets 3 consecutive color values (RGB)
    if (map_colors_.size() >= map_classes_.size() * 3) {
      for (size_t i = 0; i < map_classes_.size(); ++i) {
        const std::string & class_name = map_classes_[i];
        size_t color_idx = i * 3;
        map_colors[class_name] = {
          static_cast<float>(map_colors_[color_idx]),
          static_cast<float>(map_colors_[color_idx + 1]),
          static_cast<float>(map_colors_[color_idx + 2])};
      }
    }
  }
};

}  // namespace autoware::tensorrt_vad

#endif  // VAD_INTERFACE_CONFIG_HPP_
