#ifndef AUTOWARE_TENSORRT_VAD_VAD_INTERFACE_CONFIG_HPP_
#define AUTOWARE_TENSORRT_VAD_VAD_INTERFACE_CONFIG_HPP_

#include <vector>
#include <memory>
#include <unordered_map>
#include <array>
#include <map>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf2_ros/buffer.h>

namespace autoware::tensorrt_vad {

class VadInterfaceConfig {
public:
  int32_t target_image_width;
  int32_t target_image_height;
  std::array<float, 6> detection_range;
  int32_t default_command;
  Eigen::Matrix4f vad2base;
  Eigen::Matrix4f base2vad;
  std::unordered_map<int32_t, int32_t> autoware_to_vad_camera_mapping;
  std::map<std::string, std::array<float, 3>> map_colors;  // Map type to RGB color
  std::vector<std::string> class_mapping;  // VAD class index to Autoware class name mapping (array index = VAD class index)
  std::vector<std::string> bbox_class_names;  // Object class names from VAD model

  // NOTE: double and int64_t are used because ROS 2's declare_parameter cannot accept std::vector<float> or std::vector<int32_t>
  VadInterfaceConfig(
    int32_t target_image_width_, int32_t target_image_height_,
    const std::vector<double>& detection_range_,
    int32_t default_command_,
    const std::vector<double>& vad2base_,
    const std::vector<int64_t>& autoware_to_vad_camera_mapping_,
    const std::vector<std::string>& map_classes_,
    const std::vector<double>& map_colors_,
    const std::vector<std::string>& class_mapping_,
    const std::vector<std::string>& bbox_class_names_)
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
    // vad2base: 16 elements, row-major
    vad2base = Eigen::Matrix4f::Identity();
    for (int i = 0; i < 16; ++i) {
      vad2base(i/4, i%4) = static_cast<float>(vad2base_[i]);
    }
    // base2vad: inverse
    base2vad = vad2base.inverse();
    // camera mapping: convert from vector to map
    autoware_to_vad_camera_mapping.clear();
    for (size_t i = 0; i + 1 < autoware_to_vad_camera_mapping_.size(); i += 2) {
        int32_t key = static_cast<int32_t>(autoware_to_vad_camera_mapping_[i]);
        int32_t value = static_cast<int32_t>(autoware_to_vad_camera_mapping_[i + 1]);
        autoware_to_vad_camera_mapping[key] = value;
    }
    // map_colors: convert from vector<string> (classes) and vector<double> (colors) to map of array<float, 3>
    map_colors.clear();
    // Format: [class1_r, class1_g, class1_b, class2_r, class2_g, class2_b, ...]
    // Each class gets 3 consecutive color values (RGB)
    if (map_colors_.size() >= map_classes_.size() * 3) {
      for (size_t i = 0; i < map_classes_.size(); ++i) {
        const std::string& class_name = map_classes_[i];
        size_t color_idx = i * 3;
        map_colors[class_name] = {
          static_cast<float>(map_colors_[color_idx]),
          static_cast<float>(map_colors_[color_idx + 1]),
          static_cast<float>(map_colors_[color_idx + 2])
        };
      }
    }
  }
};

} // namespace autoware::tensorrt_vad

#endif // AUTOWARE_TENSORRT_VAD_VAD_INTERFACE_CONFIG_HPP_
