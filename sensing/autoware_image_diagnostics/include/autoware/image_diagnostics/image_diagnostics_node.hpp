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

#pragma once

#include <autoware_utils/ros/diagnostics_interface.hpp>
#include <autoware_utils/ros/polling_subscriber.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/float32_multi_array_stamped.hpp>
#include <autoware_internal_debug_msgs/msg/float32_stamped.hpp>
#include <autoware_internal_debug_msgs/msg/int32_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <deque>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::image_diagnostics
{

class ImageDiagNode : public rclcpp::Node
{
private:
  enum ImageState : uint8_t {
    NORMAL,
    SHADOW_CLIPPING,
    BLOCKAGE,
    LOW_VISIBILITY,
    HIGHLIGHT_CLIPPING
  };

  struct StateInfo
  {
    std::string label;
    std::string key_prefix;
    ImageState state_enum;
    int threshold;
    int count = 0;
    float ratio = 0.0f;

    StateInfo(std::string label, std::string key_prefix, ImageState state_enum, int threshold)
    : label(std::move(label)),
      key_prefix(std::move(key_prefix)),
      state_enum(state_enum),
      threshold(threshold)
    {
    }

    void update_count_and_ratio(const std::vector<ImageState> & states, int total_blocks)
    {
      count = static_cast<int>(std::count(states.begin(), states.end(), state_enum));
      ratio = static_cast<float>(count) / static_cast<float>(total_blocks);
    }

    [[nodiscard]] bool is_error() const { return count > threshold; }

    [[nodiscard]] std::string get_status_string() const { return is_error() ? "ERROR" : "OK"; }

    [[nodiscard]] std::string get_status_message() const
    {
      return label + ": ERROR (count = " + std::to_string(count) +
             ", error threshold = " + std::to_string(threshold) + ")";
    }

    void add_to_diagnostics(autoware_utils::DiagnosticsInterface & interface) const
    {
      interface.add_key_value(key_prefix + "_status", get_status_string());
      interface.add_key_value(key_prefix + "_number", std::to_string(count));
      interface.add_key_value(key_prefix + "_ratio", std::to_string(ratio));
    }
  };

  struct RegionFeatures
  {
    std::vector<int> avg_intensity;
    std::vector<float> blockage_ratio;
    std::vector<float> frequency_mean;
    cv::Mat frequency_map;
  };

  struct Parameters
  {
    // General settings
    bool debug;
    std::string hardware_id;
    int consecutive_error_frame_threshold;
    int image_resize_height;
    int num_blocks_horizontal;
    int num_blocks_vertical;

    // Blockage threshold
    int blockage_region_error_threshold;
    float blockage_ratio_threshold;
    int blockage_intensity_threshold;
    double blockage_frequency_ratio_threshold;

    // Shadow clipping threshold
    int shadow_region_error_threshold;
    int shadow_intensity_threshold;

    // Highlight clipping threshold
    int highlight_region_error_threshold;
    int highlight_intensity_threshold;

    // Low visibility threshold
    int low_visibility_region_error_threshold;
    double low_visibility_frequency_threshold;
  } params_;

  std::unordered_map<ImageState, cv::Scalar> state_color_map_ = {
    {ImageState::NORMAL, cv::Scalar(100, 100, 100)},
    {ImageState::SHADOW_CLIPPING, cv::Scalar(0, 0, 0)},
    {ImageState::BLOCKAGE, cv::Scalar(0, 0, 200)},
    {ImageState::LOW_VISIBILITY, cv::Scalar(0, 200, 200)},
    {ImageState::HIGHLIGHT_CLIPPING, cv::Scalar(200, 0, 200)},
  };

  cv::Scalar border_color_ = cv::Scalar(255, 255, 255);

  int total_blocks_{0};
  std::deque<bool> recent_error_flags_;
  std::vector<StateInfo> state_infos_;

  void check_parameters() const;
  void run_image_diagnostics(const sensor_msgs::msg::Image::ConstSharedPtr input_image_msg);
  cv::Mat preprocess_image(const sensor_msgs::msg::Image::ConstSharedPtr & msg) const;
  RegionFeatures compute_image_features(const cv::Mat & gray_image) const;
  std::vector<ImageState> classify_regions(const RegionFeatures & features) const;
  cv::Mat generate_diagnostic_image(const std::vector<ImageState> & states, const cv::Size & size);
  void publish_debug_images(
    const std_msgs::msg::Header & header, const cv::Mat & gray_image, const cv::Mat & dft_image,
    const cv::Mat & diagnostic_image);
  static void shift_image(cv::Mat & img);
  void update_image_diagnostics(
    const std::vector<ImageState> & states, const rclcpp::Time & timestamp);

  std::unique_ptr<autoware_utils::DiagnosticsInterface> diagnostics_interface_;

public:
  explicit ImageDiagNode(const rclcpp::NodeOptions & node_options);

protected:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  image_transport::Publisher diagnostic_image_pub_;
  image_transport::Publisher dft_image_pub_;
  image_transport::Publisher gray_image_pub_;
};

}  // namespace autoware::image_diagnostics
