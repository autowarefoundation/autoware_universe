// Copyright 2022 TIER IV, Inc.
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
#include <image_transport/image_transport.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/float32_multi_array_stamped.hpp>
#include <autoware_internal_debug_msgs/msg/float32_stamped.hpp>
#include <autoware_internal_debug_msgs/msg/int32_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::image_diagnostics
{

class ImageDiagNode : public rclcpp::Node
{
private:
  enum Image_State : uint8_t {
    NORMAL,
    SHADOW_CLIPPING,
    BLOCKAGE,
    LOW_VISIBILITY,
    HIGHLIGHT_CLIPPING
  };
  std::unordered_map<Image_State, cv::Scalar> state_color_map_ = {
    {Image_State::NORMAL, cv::Scalar(100, 100, 100)},
    {Image_State::SHADOW_CLIPPING, cv::Scalar(0, 0, 0)},
    {Image_State::BLOCKAGE, cv::Scalar(0, 0, 200)},
    {Image_State::LOW_VISIBILITY, cv::Scalar(0, 200, 200)},
    {Image_State::HIGHLIGHT_CLIPPING, cv::Scalar(200, 0, 200)},
  };

  cv::Scalar border_color_ = cv::Scalar(255, 255, 255);

  struct Parameters
  {
    // General settings
    int image_resize_height;
    int num_blocks_horizontal;
    int num_blocks_vertical;

    // Blockage threshold
    int blockage_region_warn_threshold;
    int blockage_region_error_threshold;
    float blockage_ratio_threshold;
    int blockage_intensity_threshold;
    float blockage_frequency_ratio_threshold;

    // Shadow clipping threshold
    int shadow_region_warn_threshold;
    int shadow_region_error_threshold;
    int shadow_intensity_threshold;

    // Highlight clipping threshold
    int highlight_region_warn_threshold;
    int highlight_region_error_threshold;
    int highlight_intensity_threshold;

    // Low visibility threshold
    int low_visibility_region_warn_threshold;
    int low_visibility_region_error_threshold;
    float low_visibility_frequency_threshold;
  } params_;

  struct RegionFeatures
  {
    std::vector<int> avg_intensity;
    std::vector<float> blockage_ratio;
    std::vector<float> frequency_mean;
    cv::Mat frequency_map;
  };

  void check_parameters() const;
  void run_image_diagnostics(const sensor_msgs::msg::Image::ConstSharedPtr input_image_msg);
  cv::Mat preprocess_image(const sensor_msgs::msg::Image::ConstSharedPtr & msg) const;
  RegionFeatures compute_image_features(const cv::Mat & gray_image) const;
  std::vector<ImageDiagNode::Image_State> classify_regions(const RegionFeatures & features) const;
  cv::Mat generate_diagnostic_image(const std::vector<Image_State> & states, const cv::Size & size);
  void publish_debug_images(
    const std_msgs::msg::Header & header, const cv::Mat & gray_image, const cv::Mat & dft_image,
    const cv::Mat & diagnostic_image);
  static void shift_image(cv::Mat & img);
  void update_image_diagnostics(const std::vector<Image_State> & states);

  std::unique_ptr<autoware_utils_diagnostics::DiagnosticsInterface> diagnostics_interface_;

public:
  explicit ImageDiagNode(const rclcpp::NodeOptions & node_options);

protected:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  image_transport::Publisher diagnostic_image_pub_;
  image_transport::Publisher dft_image_pub_;
  image_transport::Publisher gray_image_pub_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float32MultiArrayStamped>::SharedPtr
    average_pub_;
};

}  // namespace autoware::image_diagnostics
