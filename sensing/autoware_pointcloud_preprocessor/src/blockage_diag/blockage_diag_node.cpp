// Copyright 2024 TIER IV, Inc.
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

#include "autoware/pointcloud_preprocessor/blockage_diag/blockage_diag_node.hpp"

#include "autoware/point_types/types.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::pointcloud_preprocessor
{
using autoware::point_types::PointXYZIRCAEDT;
using diagnostic_msgs::msg::DiagnosticStatus;

BlockageDiagComponent::BlockageDiagComponent(const rclcpp::NodeOptions & options)
: Filter("BlockageDiag", options)
{
  {
    // LiDAR configuration
    // Horizontal FoV, expects two values: [min, max]
    angle_range_deg_ = declare_parameter<std::vector<double>>("angle_range");
    // Whether the channel order is top-down (true) or bottom-up (false)
    is_channel_order_top2down_ = declare_parameter<bool>("is_channel_order_top2down");

    // Blockage mask format configuration
    // The number of vertical bins in the mask. Has to equal the number of channels of the LiDAR.
    vertical_bins_ = declare_parameter<int>("vertical_bins");
    // The angular resolution of the mask, in degrees.
    horizontal_resolution_ = declare_parameter<double>("horizontal_resolution");

    // Dust detection configuration
    enable_dust_diag_ = declare_parameter<bool>("enable_dust_diag");
    dust_ratio_threshold_ = declare_parameter<float>("dust_ratio_threshold");
    dust_count_threshold_ = declare_parameter<int>("dust_count_threshold");
    dust_kernel_size_ = declare_parameter<int>("dust_kernel_size");
    dust_buffering_frames_ = declare_parameter<int>("dust_buffering_frames");
    dust_buffering_interval_ = declare_parameter<int>("dust_buffering_interval");

    // Blockage detection configuration
    blockage_ratio_threshold_ = declare_parameter<float>("blockage_ratio_threshold");
    blockage_count_threshold_ = declare_parameter<int>("blockage_count_threshold");
    blockage_kernel_ = declare_parameter<int>("blockage_kernel");
    blockage_buffering_frames_ = declare_parameter<int>("blockage_buffering_frames");
    blockage_buffering_interval_ = declare_parameter<int>("blockage_buffering_interval");

    // Debug configuration
    publish_debug_image_ = declare_parameter<bool>("publish_debug_image");

    // Depth map configuration
    // The maximum distance range of the LiDAR, in meters. The depth map is normalized to this
    // value.
    max_distance_range_ = declare_parameter<double>("max_distance_range");

    // Ground segmentation configuration
    // The ring ID that coincides with the horizon. Regions below are treated as ground,
    // regions above are treated as sky.
    horizontal_ring_id_ = declare_parameter<int>("horizontal_ring_id");
  }
  dust_mask_buffer.set_capacity(dust_buffering_frames_);
  no_return_mask_buffer.set_capacity(blockage_buffering_frames_);
  if (vertical_bins_ <= horizontal_ring_id_) {
    RCLCPP_ERROR(
      this->get_logger(),
      "The horizontal_ring_id should be smaller than vertical_bins. Skip blockage diag!");
    return;
  }

  updater_.setHardwareID("blockage_diag");
  updater_.add(std::string(this->get_namespace()) + ": blockage_validation", [this](auto & stat) {
    run_blockage_check(stat);
  });

  if (enable_dust_diag_) {
    updater_.add(std::string(this->get_namespace()) + ": dust_validation", [this](auto & stat) {
      run_dust_check(stat);
    });

    ground_dust_ratio_pub_ = create_publisher<autoware_internal_debug_msgs::msg::Float32Stamped>(
      "blockage_diag/debug/ground_dust_ratio", rclcpp::SensorDataQoS());
    if (publish_debug_image_) {
      single_frame_dust_mask_pub =
        image_transport::create_publisher(this, "blockage_diag/debug/single_frame_dust_mask_image");
      multi_frame_dust_mask_pub =
        image_transport::create_publisher(this, "blockage_diag/debug/multi_frame_dust_mask_image");
      blockage_dust_merged_pub =
        image_transport::create_publisher(this, "blockage_diag/debug/blockage_dust_merged_image");
    }
  }
  updater_.setPeriod(0.1);
  if (publish_debug_image_) {
    lidar_depth_map_pub_ =
      image_transport::create_publisher(this, "blockage_diag/debug/lidar_depth_map");
    blockage_mask_pub_ =
      image_transport::create_publisher(this, "blockage_diag/debug/blockage_mask_image");
  }
  ground_blockage_ratio_pub_ = create_publisher<autoware_internal_debug_msgs::msg::Float32Stamped>(
    "blockage_diag/debug/ground_blockage_ratio", rclcpp::SensorDataQoS());
  sky_blockage_ratio_pub_ = create_publisher<autoware_internal_debug_msgs::msg::Float32Stamped>(
    "blockage_diag/debug/sky_blockage_ratio", rclcpp::SensorDataQoS());
  using std::placeholders::_1;
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&BlockageDiagComponent::param_callback, this, _1));
}

void BlockageDiagComponent::run_blockage_check(DiagnosticStatusWrapper & stat) const
{
  stat.add("ground_blockage_ratio", std::to_string(ground_blockage_ratio_));
  stat.add("ground_blockage_count", std::to_string(ground_blockage_count_));
  stat.add(
    "ground_blockage_range_deg", "[" + std::to_string(ground_blockage_range_deg_[0]) + "," +
                                   std::to_string(ground_blockage_range_deg_[1]) + "]");
  stat.add("sky_blockage_ratio", std::to_string(sky_blockage_ratio_));
  stat.add("sky_blockage_count", std::to_string(sky_blockage_count_));
  stat.add(
    "sky_blockage_range_deg", "[" + std::to_string(sky_blockage_range_deg_[0]) + "," +
                                std::to_string(sky_blockage_range_deg_[1]) + "]");
  // TODO(badai-nguyen): consider sky_blockage_ratio_ for DiagnosticsStatus." [todo]

  auto level = DiagnosticStatus::OK;
  std::string msg = "OK";
  if (ground_blockage_ratio_ < 0) {
    level = DiagnosticStatus::STALE;
    msg = "STALE";
  } else if (
    (ground_blockage_ratio_ > blockage_ratio_threshold_) &&
    (ground_blockage_count_ > blockage_count_threshold_)) {
    level = DiagnosticStatus::ERROR;
    msg = "ERROR";
  } else if (ground_blockage_ratio_ > 0.0f) {
    level = DiagnosticStatus::WARN;
    msg = "WARN";
  }

  if ((ground_blockage_ratio_ > 0.0f) && (sky_blockage_ratio_ > 0.0f)) {
    msg = msg + ": LIDAR both blockage";
  } else if (ground_blockage_ratio_ > 0.0f) {
    msg = msg + ": LIDAR ground blockage";
  } else if (sky_blockage_ratio_ > 0.0f) {
    msg = msg + ": LIDAR sky blockage";
  }
  stat.summary(level, msg);
}

void BlockageDiagComponent::run_dust_check(diagnostic_updater::DiagnosticStatusWrapper & stat) const
{
  stat.add("ground_dust_ratio", std::to_string(ground_dust_ratio_));
  auto level = DiagnosticStatus::OK;
  std::string msg = "OK";
  if (ground_dust_ratio_ < 0.0f) {
    level = DiagnosticStatus::STALE;
    msg = "STALE";
  } else if (
    (ground_dust_ratio_ > dust_ratio_threshold_) && (dust_frame_count_ > dust_count_threshold_)) {
    level = DiagnosticStatus::ERROR;
    msg = "ERROR";
  } else if (ground_dust_ratio_ > 0.0f) {
    level = DiagnosticStatus::WARN;
    msg = "WARN";
  }

  if (ground_dust_ratio_ > 0.0f) {
    msg = msg + ": LIDAR ground dust";
  }
  stat.summary(level, msg);
}

cv::Size BlockageDiagComponent::get_mask_dimensions() const
{
  auto horizontal_bins = get_horizontal_bin(angle_range_deg_[1]);
  if (!horizontal_bins) {
    throw std::logic_error("Horizontal bin is not valid");
  }

  return {*horizontal_bins, vertical_bins_};
}

std::optional<int> BlockageDiagComponent::get_horizontal_bin(double azimuth_deg) const
{
  double min_deg = angle_range_deg_[0];
  double max_deg = angle_range_deg_[1];

  bool fov_wraps_around = (min_deg > max_deg);
  if (fov_wraps_around) {
    azimuth_deg += 360.0;
    max_deg += 360.0;
  }

  bool azimuth_is_in_fov = ((azimuth_deg > min_deg) && (azimuth_deg <= max_deg));
  if (!azimuth_is_in_fov) {
    return std::nullopt;
  }

  return {static_cast<int>((azimuth_deg - min_deg) / horizontal_resolution_)};
}

std::optional<int> BlockageDiagComponent::get_vertical_bin(uint16_t channel) const
{
  if (channel >= vertical_bins_) {
    return std::nullopt;
  }

  if (is_channel_order_top2down_) {
    return {channel};
  }

  return {vertical_bins_ - channel - 1};
}

cv::Mat BlockageDiagComponent::make_normalized_depth_image(const PCLCloudXYZIRCAEDT & input) const
{
  auto dimensions = get_mask_dimensions();
  cv::Mat depth_image(dimensions, CV_16UC1, cv::Scalar(0));

  for (const auto & p : input.points) {
    auto vertical_bin = get_vertical_bin(p.channel);
    if (!vertical_bin) {
      RCLCPP_ERROR(
        this->get_logger(),
        "p.channel: %d is larger than vertical_bins: %d. Please check the parameter "
        "'vertical_bins'.",
        p.channel, vertical_bins_);
      throw std::runtime_error("Parameter is not valid");
    }

    double azimuth_deg = p.azimuth * (180.0 / M_PI);
    auto horizontal_bin = get_horizontal_bin(azimuth_deg);
    if (!horizontal_bin) {
      continue;
    }

    // Max distance is mapped to 0, zero-distance is mapped to UINT16_MAX.
    uint16_t normalized_depth =
      UINT16_MAX * (1.0 - std::min(p.distance / max_distance_range_, 1.0));
    depth_image.at<uint16_t>(*vertical_bin, *horizontal_bin) = normalized_depth;
  }

  return depth_image;
}

cv::Mat BlockageDiagComponent::quantize_to_8u(const cv::Mat & image_16u) const
{
  auto dimensions = get_mask_dimensions();
  assert(dimensions == image_16u.size());
  assert(image_16u.type() == CV_16UC1);

  cv::Mat image_8u(dimensions, CV_8UC1, cv::Scalar(0));
  // UINT16_MAX = 65535, UINT8_MAX = 255, so downscale by ceil(65535 / 255) = 256.
  image_16u.convertTo(image_8u, CV_8UC1, 1.0 / 256);
  return image_8u;
}

cv::Mat BlockageDiagComponent::make_no_return_mask(const cv::Mat & depth_image) const
{
  auto dimensions = get_mask_dimensions();
  assert(dimensions == depth_image.size());
  assert(depth_image.type() == CV_8UC1);

  cv::Mat no_return_mask(dimensions, CV_8UC1, cv::Scalar(0));
  cv::inRange(depth_image, 0, 1, no_return_mask);

  return no_return_mask;
}

cv::Mat BlockageDiagComponent::make_blockage_mask(const cv::Mat & no_return_mask) const
{
  auto dimensions = get_mask_dimensions();
  assert(dimensions == no_return_mask.size());
  assert(no_return_mask.type() == CV_8UC1);

  int kernel_size = 2 * blockage_kernel_ + 1;
  int kernel_center = blockage_kernel_;
  cv::Mat kernel = cv::getStructuringElement(
    cv::MORPH_RECT, cv::Size(kernel_size, kernel_size), cv::Point(kernel_center, kernel_center));

  cv::Mat erosion_result(dimensions, CV_8UC1, cv::Scalar(0));
  cv::erode(no_return_mask, erosion_result, kernel);

  cv::Mat blockage_mask(dimensions, CV_8UC1, cv::Scalar(0));
  cv::dilate(erosion_result, blockage_mask, kernel);

  return blockage_mask;
}

cv::Mat BlockageDiagComponent::update_time_series_blockage_mask(const cv::Mat & blockage_mask)
{
  if (blockage_buffering_interval_ == 0) {
    return blockage_mask.clone();
  }

  auto dimensions = get_mask_dimensions();
  assert(dimensions == blockage_mask.size());
  assert(blockage_mask.type() == CV_8UC1);

  cv::Mat time_series_blockage_result(dimensions, CV_8UC1, cv::Scalar(0));
  cv::Mat time_series_blockage_mask(dimensions, CV_8UC1, cv::Scalar(0));
  cv::Mat no_return_mask_binarized(dimensions, CV_8UC1, cv::Scalar(0));

  no_return_mask_binarized = blockage_mask / 255;
  if (blockage_frame_count_ >= blockage_buffering_interval_) {
    no_return_mask_buffer.push_back(no_return_mask_binarized);
    blockage_frame_count_ = 0;
  } else {
    blockage_frame_count_++;
  }

  for (const auto & binary_mask : no_return_mask_buffer) {
    time_series_blockage_mask += binary_mask;
  }

  cv::inRange(
    time_series_blockage_mask, no_return_mask_buffer.size() - 1, no_return_mask_buffer.size(),
    time_series_blockage_result);

  return time_series_blockage_result;
}

std::pair<cv::Mat, cv::Mat> BlockageDiagComponent::segment_into_ground_and_sky(
  const cv::Mat & mask) const
{
  auto dimensions = get_mask_dimensions();
  assert(dimensions == mask.size());
  assert(mask.type() == CV_8UC1);

  cv::Mat sky_mask;
  mask(cv::Rect(0, 0, dimensions.width, horizontal_ring_id_)).copyTo(sky_mask);

  cv::Mat ground_mask;
  mask(cv::Rect(0, horizontal_ring_id_, dimensions.width, dimensions.height - horizontal_ring_id_))
    .copyTo(ground_mask);

  return {ground_mask, sky_mask};
}

float BlockageDiagComponent::get_nonzero_ratio(const cv::Mat & mask)
{
  size_t area = mask.cols * mask.rows;
  if (area == 0) {
    return 0.F;
  }

  return static_cast<float>(cv::countNonZero(mask)) / static_cast<float>(area);
}

void BlockageDiagComponent::update_ground_blockage_info(const cv::Mat & ground_blockage_mask)
{
  if (ground_blockage_ratio_ <= blockage_ratio_threshold_) {
    ground_blockage_count_ = 0;
    return;
  }

  cv::Rect blockage_bb = cv::boundingRect(ground_blockage_mask);
  double blockage_start_deg = blockage_bb.x * horizontal_resolution_ + angle_range_deg_[0];
  double blockage_end_deg =
    (blockage_bb.x + blockage_bb.width) * horizontal_resolution_ + angle_range_deg_[0];

  ground_blockage_range_deg_[0] = static_cast<float>(blockage_start_deg);
  ground_blockage_range_deg_[1] = static_cast<float>(blockage_end_deg);

  if (ground_blockage_count_ <= 2 * blockage_count_threshold_) {
    ground_blockage_count_ += 1;
  }
}

void BlockageDiagComponent::update_sky_blockage_info(const cv::Mat & sky_blockage_mask)
{
  if (sky_blockage_ratio_ <= blockage_ratio_threshold_) {
    sky_blockage_count_ = 0;
    return;
  }

  cv::Rect blockage_bb = cv::boundingRect(sky_blockage_mask);
  double blockage_start_deg = blockage_bb.x * horizontal_resolution_ + angle_range_deg_[0];
  double blockage_end_deg =
    (blockage_bb.x + blockage_bb.width) * horizontal_resolution_ + angle_range_deg_[0];

  sky_blockage_range_deg_[0] = static_cast<float>(blockage_start_deg);
  sky_blockage_range_deg_[1] = static_cast<float>(blockage_end_deg);

  if (sky_blockage_count_ <= 2 * blockage_count_threshold_) {
    sky_blockage_count_ += 1;
  }
}

void BlockageDiagComponent::compute_dust_diagnostics(
  const cv::Mat & no_return_mask, const DebugInfo & debug_info)
{
  auto dimensions = get_mask_dimensions();
  assert(dimensions == no_return_mask.size());
  assert(no_return_mask.type() == CV_8UC1);

  auto [single_dust_ground_img, sky_blank] = segment_into_ground_and_sky(no_return_mask);

  // It is normal for the sky region to be blank, therefore ignore it.
  sky_blank.setTo(cv::Scalar(0));

  int kernel_size = 2 * dust_kernel_size_ + 1;
  int kernel_center = dust_kernel_size_;
  cv::Mat kernel = cv::getStructuringElement(
    cv::MORPH_RECT, cv::Size(kernel_size, kernel_size), cv::Point(kernel_center, kernel_center));

  cv::dilate(single_dust_ground_img, single_dust_ground_img, kernel);
  cv::erode(single_dust_ground_img, single_dust_ground_img, kernel);
  cv::inRange(single_dust_ground_img, 254, 255, single_dust_ground_img);

  // Re-assemble the processed ground dust image and the sky blank.
  cv::Mat single_dust_img(dimensions, CV_8UC1, cv::Scalar(0));
  cv::vconcat(sky_blank, single_dust_ground_img, single_dust_img);

  autoware_internal_debug_msgs::msg::Float32Stamped ground_dust_ratio_msg;
  ground_dust_ratio_ = static_cast<float>(cv::countNonZero(single_dust_ground_img)) /
                       (single_dust_ground_img.cols * single_dust_ground_img.rows);
  ground_dust_ratio_msg.data = ground_dust_ratio_;
  ground_dust_ratio_msg.stamp = now();
  ground_dust_ratio_pub_->publish(ground_dust_ratio_msg);

  if (ground_dust_ratio_ > dust_ratio_threshold_) {
    if (dust_frame_count_ < 2 * dust_count_threshold_) {
      dust_frame_count_++;
    }
  } else {
    dust_frame_count_ = 0;
  }

  if (publish_debug_image_) {
    cv::Mat binarized_dust_mask_(dimensions, CV_8UC1, cv::Scalar(0));
    cv::Mat multi_frame_dust_mask(dimensions, CV_8UC1, cv::Scalar(0));
    cv::Mat multi_frame_ground_dust_result(dimensions, CV_8UC1, cv::Scalar(0));

    if (dust_buffering_interval_ == 0) {
      single_dust_img.copyTo(multi_frame_ground_dust_result);
      dust_buffering_frame_counter_ = 0;
    } else {
      binarized_dust_mask_ = single_dust_img / 255;
      if (dust_buffering_frame_counter_ >= dust_buffering_interval_) {
        dust_mask_buffer.push_back(binarized_dust_mask_);
        dust_buffering_frame_counter_ = 0;
      } else {
        dust_buffering_frame_counter_++;
      }
      for (const auto & binarized_dust_mask : dust_mask_buffer) {
        multi_frame_dust_mask += binarized_dust_mask;
      }
      cv::inRange(
        multi_frame_dust_mask, dust_mask_buffer.size() - 1, dust_mask_buffer.size(),
        multi_frame_ground_dust_result);
    }
    cv::Mat single_frame_ground_dust_colorized(dimensions, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::applyColorMap(single_dust_img, single_frame_ground_dust_colorized, cv::COLORMAP_JET);
    cv::Mat multi_frame_ground_dust_colorized;
    cv::Mat blockage_dust_merged_img(dimensions, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat blockage_dust_merged_mask(dimensions, CV_8UC1, cv::Scalar(0));
    blockage_dust_merged_img.setTo(
      cv::Vec3b(0, 0, 255), debug_info.blockage_mask_multi_frame);  // red:blockage
    blockage_dust_merged_img.setTo(
      cv::Vec3b(0, 255, 255), multi_frame_ground_dust_result);  // yellow:dust
    sensor_msgs::msg::Image::SharedPtr single_frame_dust_mask_msg =
      cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", single_frame_ground_dust_colorized)
        .toImageMsg();
    single_frame_dust_mask_pub.publish(single_frame_dust_mask_msg);
    sensor_msgs::msg::Image::SharedPtr multi_frame_dust_mask_msg =
      cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", multi_frame_ground_dust_colorized)
        .toImageMsg();
    multi_frame_dust_mask_pub.publish(multi_frame_dust_mask_msg);
    cv::Mat blockage_dust_merged_colorized(dimensions, CV_8UC3, cv::Scalar(0, 0, 0));
    blockage_dust_merged_img.copyTo(blockage_dust_merged_colorized);
    sensor_msgs::msg::Image::SharedPtr blockage_dust_merged_msg =
      cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", blockage_dust_merged_colorized)
        .toImageMsg();
    blockage_dust_merged_msg->header = debug_info.input_header;
    blockage_dust_merged_pub.publish(blockage_dust_merged_msg);
  }
}

void BlockageDiagComponent::publish_debug_info(const DebugInfo & debug_info) const
{
  autoware_internal_debug_msgs::msg::Float32Stamped ground_blockage_ratio_msg;
  ground_blockage_ratio_msg.data = ground_blockage_ratio_;
  ground_blockage_ratio_msg.stamp = now();
  ground_blockage_ratio_pub_->publish(ground_blockage_ratio_msg);

  autoware_internal_debug_msgs::msg::Float32Stamped sky_blockage_ratio_msg;
  sky_blockage_ratio_msg.data = sky_blockage_ratio_;
  sky_blockage_ratio_msg.stamp = now();
  sky_blockage_ratio_pub_->publish(sky_blockage_ratio_msg);

  if (publish_debug_image_) {
    sensor_msgs::msg::Image::SharedPtr lidar_depth_map_msg =
      cv_bridge::CvImage(std_msgs::msg::Header(), "mono16", debug_info.depth_image_16u)
        .toImageMsg();
    lidar_depth_map_msg->header = debug_info.input_header;
    lidar_depth_map_pub_.publish(lidar_depth_map_msg);
    cv::Mat blockage_mask_colorized;
    cv::applyColorMap(
      debug_info.blockage_mask_multi_frame, blockage_mask_colorized, cv::COLORMAP_JET);
    sensor_msgs::msg::Image::SharedPtr blockage_mask_msg =
      cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", blockage_mask_colorized).toImageMsg();
    blockage_mask_msg->header = debug_info.input_header;
    blockage_mask_pub_.publish(blockage_mask_msg);
  }
}

void BlockageDiagComponent::filter(
  const PointCloud2ConstPtr & input, [[maybe_unused]] const IndicesPtr & indices,
  PointCloud2 & output)
{
  std::scoped_lock lock(mutex_);

  PCLCloudXYZIRCAEDT pcl_input;
  pcl::fromROSMsg(*input, pcl_input);

  cv::Mat depth_image_16u = make_normalized_depth_image(pcl_input);
  cv::Mat depth_image_8u = quantize_to_8u(depth_image_16u);
  cv::Mat no_return_mask = make_no_return_mask(depth_image_8u);
  cv::Mat blockage_mask = make_blockage_mask(no_return_mask);
  cv::Mat time_series_blockage_result = update_time_series_blockage_mask(blockage_mask);

  auto [ground_blockage_mask, sky_blockage_mask] = segment_into_ground_and_sky(blockage_mask);

  ground_blockage_ratio_ = get_nonzero_ratio(ground_blockage_mask);
  sky_blockage_ratio_ = get_nonzero_ratio(sky_blockage_mask);

  update_ground_blockage_info(ground_blockage_mask);
  update_sky_blockage_info(sky_blockage_mask);

  const DebugInfo debug_info = {input->header, depth_image_16u, time_series_blockage_result};

  if (enable_dust_diag_) {
    compute_dust_diagnostics(no_return_mask, debug_info);
  }

  publish_debug_info(debug_info);

  pcl::toROSMsg(pcl_input, output);
  output.header = input->header;
}

rcl_interfaces::msg::SetParametersResult BlockageDiagComponent::param_callback(
  const std::vector<rclcpp::Parameter> & p)
{
  std::scoped_lock lock(mutex_);
  if (get_param(p, "blockage_ratio_threshold", blockage_ratio_threshold_)) {
    RCLCPP_DEBUG(
      get_logger(), "Setting new blockage_ratio_threshold to: %f.", blockage_ratio_threshold_);
  }
  if (get_param(p, "horizontal_ring_id", horizontal_ring_id_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new horizontal_ring_id to: %d.", horizontal_ring_id_);
  }
  if (get_param(p, "vertical_bins", vertical_bins_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new vertical_bins to: %d.", vertical_bins_);
  }
  if (get_param(p, "blockage_count_threshold", blockage_count_threshold_)) {
    RCLCPP_DEBUG(
      get_logger(), "Setting new blockage_count_threshold to: %d.", blockage_count_threshold_);
  }
  if (get_param(p, "is_channel_order_top2down", is_channel_order_top2down_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new lidar model to: %d. ", is_channel_order_top2down_);
  }
  if (get_param(p, "angle_range", angle_range_deg_)) {
    RCLCPP_DEBUG(
      get_logger(), " Setting new angle_range to: [%f , %f].", angle_range_deg_[0],
      angle_range_deg_[1]);
  }
  if (get_param(p, "blockage_buffering_frames", blockage_buffering_frames_)) {
    RCLCPP_DEBUG(
      get_logger(), "Setting new blockage_buffering_frames_ to: %d.", blockage_buffering_frames_);
  }
  if (get_param(p, "blockage_buffering_interval", blockage_buffering_interval_)) {
    RCLCPP_DEBUG(
      get_logger(), "Setting new blockage_buffering_interval_ to: %d.",
      blockage_buffering_interval_);
  }
  if (get_param(p, "dust_kernel_size", dust_kernel_size_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new dust_kernel_size_ to: %d.", dust_kernel_size_);
  }
  if (get_param(p, "dust_buffering_frames", dust_buffering_frames_)) {
    RCLCPP_DEBUG(
      get_logger(), "Setting new dust_buffering_frames_ to: %d.", dust_buffering_frames_);
    // note:NOT affects to actual variable.
    // if you want change this param/variable, change the parameter called at launch this
    // node(aip_launcher).
  }
  if (get_param(p, "dust_buffering_interval", dust_buffering_interval_)) {
    RCLCPP_DEBUG(
      get_logger(), "Setting new dust_buffering_interval_ to: %d.", dust_buffering_interval_);
    dust_buffering_frame_counter_ = 0;
  }
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}
}  // namespace autoware::pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::pointcloud_preprocessor::BlockageDiagComponent)
