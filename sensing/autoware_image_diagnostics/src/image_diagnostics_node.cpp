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

#include "autoware/image_diagnostics/image_diagnostics_node.hpp"

#include <rclcpp/time.hpp>

#include <std_msgs/msg/header.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace autoware::image_diagnostics
{

ImageDiagNode::ImageDiagNode(const rclcpp::NodeOptions & node_options)
: Node("image_diagnostics_node", node_options)
{
  // Parameters
  params_.debug = declare_parameter<bool>("debug");
  params_.hardware_id = declare_parameter<std::string>("hardware_id");
  params_.consecutive_error_frame_threshold =
    declare_parameter<int>("consecutive_error_frame_threshold");

  params_.image_resize_height = declare_parameter<int>("image_resize_height");
  params_.num_blocks_horizontal = declare_parameter<int>("num_blocks_horizontal");
  params_.num_blocks_vertical = declare_parameter<int>("num_blocks_vertical");

  // Blockage thresholds
  params_.blockage_region_error_threshold =
    declare_parameter<int>("blockage_region_error_threshold");
  params_.blockage_ratio_threshold = declare_parameter<float>("blockage_ratio_threshold");
  params_.blockage_intensity_threshold = declare_parameter<int>("blockage_intensity_threshold");
  params_.blockage_frequency_ratio_threshold =
    declare_parameter<double>("blockage_frequency_ratio_threshold");

  // Shadow clipping thresholds
  params_.shadow_region_error_threshold = declare_parameter<int>("shadow_region_error_threshold");
  params_.shadow_intensity_threshold = declare_parameter<int>("shadow_intensity_threshold");

  // Highlight clipping thresholds
  params_.highlight_region_error_threshold =
    declare_parameter<int>("highlight_region_error_threshold");
  params_.highlight_intensity_threshold = declare_parameter<int>("highlight_intensity_threshold");

  // Low visibility thresholds
  params_.low_visibility_region_error_threshold =
    declare_parameter<int>("low_visibility_region_error_threshold");
  params_.low_visibility_frequency_threshold =
    declare_parameter<double>("low_visibility_frequency_threshold");

  total_blocks_ = params_.num_blocks_horizontal * params_.num_blocks_vertical;
  check_parameters();

  state_infos_ = {
    {"Blockage", "blockage", ImageState::BLOCKAGE, params_.blockage_region_error_threshold},
    {"Highlight clipping", "highlight_clipping", ImageState::HIGHLIGHT_CLIPPING,
     params_.highlight_region_error_threshold},
    {"Shadow clipping", "shadow_clipping", ImageState::SHADOW_CLIPPING,
     params_.shadow_region_error_threshold},
    {"Low visibility", "low_visibility", ImageState::LOW_VISIBILITY,
     params_.low_visibility_region_error_threshold}};

  // Publisher and Subscriber
  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    "input/raw_image", rclcpp::SensorDataQoS(),
    std::bind(&ImageDiagNode::run_image_diagnostics, this, std::placeholders::_1));

  diagnostics_interface_ =
    std::make_unique<autoware_utils::DiagnosticsInterface>(this, params_.hardware_id);

  if (params_.debug) {
    diagnostic_image_pub_ = image_transport::create_publisher(
      this, "image_diag/debug/" + params_.hardware_id + "/diag_block_image");
    dft_image_pub_ = image_transport::create_publisher(
      this, "image_diag/debug/" + params_.hardware_id + "/dft_image");
    gray_image_pub_ = image_transport::create_publisher(
      this, "image_diag/debug/" + params_.hardware_id + "/gray_image");
  }
}

void ImageDiagNode::check_parameters() const
{
  auto validate_threshold = [&](const std::string & name, int err_thresh) {
    if (err_thresh > total_blocks_) {
      throw std::runtime_error(
        name + ": thresholds exceed total number of blocks (" + std::to_string(total_blocks_) +
        ").");
    }
  };

  // Validate thresholds that are per-block count
  validate_threshold("blockage", params_.blockage_region_error_threshold);
  validate_threshold("shadow", params_.shadow_region_error_threshold);
  validate_threshold("highlight", params_.highlight_region_error_threshold);
  validate_threshold("low_visibility", params_.low_visibility_region_error_threshold);
}

void ImageDiagNode::run_image_diagnostics(
  const sensor_msgs::msg::Image::ConstSharedPtr input_image_msg)
{
  const cv::Mat gray_img = preprocess_image(input_image_msg);
  const RegionFeatures features = compute_image_features(gray_img);
  const std::vector<ImageState> region_states = classify_regions(features);

  if (params_.debug) {
    const cv::Mat diagnostic_image = generate_diagnostic_image(region_states, gray_img.size());
    publish_debug_images(
      input_image_msg->header, gray_img, features.frequency_map, diagnostic_image);
  }
  update_image_diagnostics(region_states, input_image_msg->header.stamp);
}

cv::Mat ImageDiagNode::preprocess_image(const sensor_msgs::msg::Image::ConstSharedPtr & msg) const
{
  cv::Mat gray_img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8)->image;
  cv::Size new_size(
    params_.image_resize_height * gray_img.cols / gray_img.rows, params_.image_resize_height);
  cv::resize(gray_img, gray_img, new_size);
  return gray_img;
}

ImageDiagNode::RegionFeatures ImageDiagNode::compute_image_features(
  const cv::Mat & gray_image) const
{
  RegionFeatures features;

  // Compute the width and height of each block
  const int block_w = std::floor(gray_image.cols / params_.num_blocks_horizontal);
  const int block_h = std::floor(gray_image.rows / params_.num_blocks_vertical);
  const int num_region_pixels = block_w * block_h;
  // Get optimal sizes for DFT computation (padded sizes)
  const int dft_w = cv::getOptimalDFTSize(block_w);
  const int dft_h = cv::getOptimalDFTSize(block_h);

  /// Threshold the grayscale image: pixels >= threshold become 255, others become 0.
  // Used to identify bright (unblocked) areas for computing blockage ratio.
  cv::Mat binary_image;
  cv::threshold(
    gray_image, binary_image, params_.blockage_intensity_threshold, 255, cv::THRESH_BINARY);

  // Convert to 32-bit float for frequency domain processing
  cv::Mat gray_image_32f;
  gray_image.convertTo(gray_image_32f, CV_32FC1);
  features.frequency_map = cv::Mat(gray_image.size(), CV_32FC1, cv::Scalar(0));

  // Iterate over each image block
  for (int v = 0; v < params_.num_blocks_vertical; ++v) {
    for (int h = 0; h < params_.num_blocks_horizontal; ++h) {
      cv::Rect roi(h * block_w, v * block_h, block_w, block_h);

      // Compute average intensity and blockage ratio
      int avg_intensity = static_cast<int>(cv::mean(gray_image(roi))[0]);
      int num_blocked_pixels = cv::countNonZero(binary_image(roi));
      float ratio = static_cast<float>(num_region_pixels - num_blocked_pixels) /
                    static_cast<float>(num_region_pixels);

      // Extract and pad block for DFT
      cv::Mat gray_img_roi = gray_image_32f(roi);
      cv::Mat padded_roi;
      cv::copyMakeBorder(
        gray_img_roi, padded_roi, 0, dft_h - block_h, 0, dft_w - block_w, cv::BORDER_CONSTANT);

      // Perform DFT to obtain frequency components
      std::vector<cv::Mat> dft_channel_img = {
        padded_roi, cv::Mat::zeros(padded_roi.size(), CV_32FC1)};
      cv::Mat complex_img;
      cv::merge(dft_channel_img, complex_img);
      cv::dft(complex_img, complex_img);
      shift_image(complex_img);
      cv::split(complex_img, dft_channel_img);

      // Crop back to original block size and compute log frequency spectrum
      cv::Mat frequency_roi = dft_channel_img[0](cv::Rect(0, 0, block_w, block_h)).clone();
      cv::log(frequency_roi, frequency_roi);
      float freq_mean = static_cast<float>(cv::mean(frequency_roi)[0]);

      // Store features
      dft_channel_img[0](cv::Rect(0, 0, block_w, block_h)).copyTo(features.frequency_map(roi));
      features.avg_intensity.push_back(avg_intensity);
      features.blockage_ratio.push_back(ratio);
      features.frequency_mean.push_back(freq_mean);
    }
  }
  return features;
}

std::vector<ImageDiagNode::ImageState> ImageDiagNode::classify_regions(
  const RegionFeatures & features) const
{
  std::vector<ImageState> states;
  for (size_t i = 0; i < features.avg_intensity.size(); ++i) {
    ImageState state = ImageState::NORMAL;
    if (features.avg_intensity[i] < params_.shadow_intensity_threshold) {
      state = ImageState::SHADOW_CLIPPING;
    } else if (
      features.blockage_ratio[i] > params_.blockage_ratio_threshold &&
      features.frequency_mean[i] < params_.blockage_frequency_ratio_threshold) {
      state = ImageState::BLOCKAGE;
    } else if (
      features.frequency_mean[i] < params_.low_visibility_frequency_threshold &&
      features.avg_intensity[i] < params_.highlight_intensity_threshold) {
      state = ImageState::LOW_VISIBILITY;
    } else if (features.avg_intensity[i] > params_.highlight_intensity_threshold) {
      state = ImageState::HIGHLIGHT_CLIPPING;
    }
    states.push_back(state);
  }
  return states;
}

void ImageDiagNode::update_image_diagnostics(
  const std::vector<ImageState> & states, const rclcpp::Time & timestamp)
{
  diagnostics_interface_->clear();

  const int num_normal =
    static_cast<int>(std::count(states.begin(), states.end(), ImageState::NORMAL));
  const float ratio_normal = static_cast<float>(num_normal) / static_cast<float>(total_blocks_);

  diagnostics_interface_->add_key_value("image header timestamp", timestamp.seconds());
  diagnostics_interface_->add_key_value("normal_ratio", std::to_string(ratio_normal));

  // Diagnostic for blockage, shadow, highlight, low visibility
  std::vector<std::string> status_details;
  bool is_current_error = false;

  for (auto & info : state_infos_) {
    info.update_count_and_ratio(states, total_blocks_);
    if (info.is_error()) {
      is_current_error = true;
      status_details.emplace_back(info.get_status_message());
    }
    info.add_to_diagnostics(*diagnostics_interface_);
  }

  recent_error_flags_.push_back(is_current_error);
  if (recent_error_flags_.size() > static_cast<size_t>(params_.consecutive_error_frame_threshold)) {
    recent_error_flags_.pop_front();
  }

  const int recent_error_count =
    static_cast<int>(std::count(recent_error_flags_.begin(), recent_error_flags_.end(), true));

  // Set level based on recent error count
  int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::ostringstream status_msg;

  if (recent_error_count >= params_.consecutive_error_frame_threshold) {
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status_msg << "Image status: ERROR\n";
  } else if (recent_error_count > 0) {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status_msg << "Image status: WARNING\n";
  }

  if (!status_details.empty()) {
    status_msg << "Details:\n";
    for (const auto & msg : status_details) {
      status_msg << "- " << msg << "\n";
    }
  }

  diagnostics_interface_->update_level_and_message(level, status_msg.str());
  diagnostics_interface_->publish(this->get_clock()->now());
}

cv::Mat ImageDiagNode::generate_diagnostic_image(
  const std::vector<ImageState> & states, const cv::Size & size)
{
  cv::Mat diag_block_image(size, CV_8UC3);
  int block_w = std::floor(size.width / params_.num_blocks_horizontal);
  int block_h = std::floor(size.height / params_.num_blocks_vertical);

  int idx = 0;
  for (int v = 0; v < params_.num_blocks_vertical; ++v) {
    for (int h = 0; h < params_.num_blocks_horizontal; ++h, ++idx) {
      cv::Rect r(h * block_w, v * block_h, block_w, block_h);
      cv::rectangle(diag_block_image, r, state_color_map_.at(states[idx]), -1);
    }
  }

  // draw boundary of blocks
  for (int v = 1; v < params_.num_blocks_vertical; ++v)
    cv::line(
      diag_block_image, cv::Point(0, v * block_h), cv::Point(size.width, v * block_h),
      border_color_, 1, cv::LINE_AA, 0);
  for (int h = 1; h < params_.num_blocks_horizontal; ++h)
    cv::line(
      diag_block_image, cv::Point(h * block_w, 0), cv::Point(h * block_w, size.height),
      border_color_, 1, cv::LINE_AA, 0);
  return diag_block_image;
}

void ImageDiagNode::publish_debug_images(
  const std_msgs::msg::Header & header, const cv::Mat & gray_image, const cv::Mat & dft_image,
  const cv::Mat & diagnostic_image)
{
  auto gray_image_msg = cv_bridge::CvImage(header, "mono8", gray_image).toImageMsg();
  cv::Mat dft_image_8uc1;
  dft_image.convertTo(dft_image_8uc1, CV_8UC1);
  auto dft_image_msg = cv_bridge::CvImage(header, "mono8", dft_image_8uc1).toImageMsg();
  auto diagnostic_image_msg = cv_bridge::CvImage(header, "bgr8", diagnostic_image).toImageMsg();
  gray_image_pub_.publish(gray_image_msg);
  dft_image_pub_.publish(dft_image_msg);
  diagnostic_image_pub_.publish(diagnostic_image_msg);
}

void ImageDiagNode::shift_image(cv::Mat & img)
{
  int cx = img.cols / 2;
  int cy = img.rows / 2;

  cv::Mat left_top(img, cv::Rect(0, 0, cx, cy));
  cv::Mat right_top(img, cv::Rect(cx, 0, cx, cy));
  cv::Mat left_bottom(img, cv::Rect(0, cy, cx, cy));
  cv::Mat right_bottom(img, cv::Rect(cx, cy, cx, cy));

  cv::Mat tmp;

  left_top.copyTo(tmp);
  right_bottom.copyTo(left_top);
  tmp.copyTo(right_bottom);

  right_top.copyTo(tmp);
  left_bottom.copyTo(right_top);
  tmp.copyTo(left_bottom);
}

}  // namespace autoware::image_diagnostics

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::image_diagnostics::ImageDiagNode)
