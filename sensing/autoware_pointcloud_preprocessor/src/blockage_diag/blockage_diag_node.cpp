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

#include "autoware/pointcloud_preprocessor/blockage_diag/blockage_diag.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::pointcloud_preprocessor
{
using diagnostic_msgs::msg::DiagnosticStatus;

BlockageDiagComponent::BlockageDiagComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("BlockageDiag", rclcpp::NodeOptions(options).start_parameter_services(false))
{
  {
    // LiDAR configuration
    // Horizontal FoV, expects two values: [min, max]
    angle_range_deg_ = declare_parameter<std::vector<double>>("angle_range");
    // Whether the channel order is top-down (true) or bottom-up (false)
    bool is_channel_order_top2down = declare_parameter<bool>("is_channel_order_top2down");

    // Blockage mask format configuration
    // The number of vertical bins in the mask. Has to equal the number of channels of the LiDAR.
    int vertical_bins = declare_parameter<int>("vertical_bins");
    // The angular resolution of the mask, in degrees.
    horizontal_resolution_ = declare_parameter<double>("horizontal_resolution");

    // Dust detection configuration
    enable_dust_diag_ = declare_parameter<bool>("enable_dust_diag");
    dust_config_.dust_ratio_threshold = declare_parameter<float>("dust_ratio_threshold");
    dust_config_.dust_count_threshold = declare_parameter<int>("dust_count_threshold");
    dust_config_.dust_kernel_size = declare_parameter<int>("dust_kernel_size");
    // Multi-frame dust visualization configuration
    int dust_buffering_frames = declare_parameter<int>("dust_buffering_frames");
    dust_visualize_data_.buffering_interval = declare_parameter<int>("dust_buffering_interval");
    dust_visualize_data_.mask_buffer.set_capacity(dust_buffering_frames);

    // Blockage detection configuration
    blockage_config_.blockage_ratio_threshold =
      declare_parameter<float>("blockage_ratio_threshold");
    blockage_config_.blockage_count_threshold = declare_parameter<int>("blockage_count_threshold");
    blockage_config_.blockage_kernel = declare_parameter<int>("blockage_kernel");
    // Multi-frame blockage visualization configuration
    int blockage_buffering_frames = declare_parameter<int>("blockage_buffering_frames");
    blockage_visualize_data_.buffering_interval =
      declare_parameter<int>("blockage_buffering_interval");
    blockage_visualize_data_.mask_buffer.set_capacity(blockage_buffering_frames);

    // Debug configuration
    publish_debug_image_ = declare_parameter<bool>("publish_debug_image");

    // Depth map configuration
    // The maximum distance range of the LiDAR, in meters. The depth map is normalized to this
    // value.
    double max_distance_range = declare_parameter<double>("max_distance_range");

    // Ground segmentation configuration
    // The ring ID that coincides with the horizon. Regions below are treated as ground,
    // regions above are treated as sky.
    horizontal_ring_id_ = declare_parameter<int>("horizontal_ring_id");

    // Validate parameters
    if (vertical_bins <= horizontal_ring_id_) {
      RCLCPP_ERROR(
        this->get_logger(),
        "The horizontal_ring_id should be smaller than vertical_bins. Skip blockage diag!");
      return;
    }

    // Initialize PointCloud2ToDepthImage converter
    pointcloud2_to_depth_image::ConverterConfig depth_image_config;
    depth_image_config.horizontal.angle_range_min_deg = angle_range_deg_[0];
    depth_image_config.horizontal.angle_range_max_deg = angle_range_deg_[1];
    depth_image_config.horizontal.horizontal_resolution = horizontal_resolution_;
    depth_image_config.vertical.vertical_bins = vertical_bins;
    depth_image_config.vertical.is_channel_order_top2down = is_channel_order_top2down;
    depth_image_config.max_distance_range = max_distance_range;
    depth_image_converter_ =
      std::make_unique<pointcloud2_to_depth_image::PointCloud2ToDepthImage>(depth_image_config);
  }

  // Publishers setup
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

  if (enable_dust_diag_) {
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

  // Subscriber setup
  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "input", rclcpp::SensorDataQoS(),
    std::bind(&BlockageDiagComponent::update_diagnostics, this, std::placeholders::_1));

  // Diagnostic updater setup
  updater_.setHardwareID("blockage_diag");
  updater_.add(std::string(this->get_namespace()) + ": blockage_validation", [this](auto & stat) {
    run_blockage_check(stat);
  });
  if (enable_dust_diag_) {
    updater_.add(std::string(this->get_namespace()) + ": dust_validation", [this](auto & stat) {
      run_dust_check(stat);
    });
  }
  updater_.setPeriod(0.1);
}

void BlockageDiagComponent::run_blockage_check(DiagnosticStatusWrapper & stat) const
{
  BlockageDetectionResult res = blockage_result_;
  stat.add("ground_blockage_ratio", std::to_string(res.ground.blockage_ratio));
  stat.add("ground_blockage_count", std::to_string(res.ground.blockage_count));
  stat.add(
    "ground_blockage_range_deg", "[" + std::to_string(res.ground.blockage_start_deg) + "," +
                                   std::to_string(res.ground.blockage_end_deg) + "]");
  stat.add("sky_blockage_ratio", std::to_string(res.sky.blockage_ratio));
  stat.add("sky_blockage_count", std::to_string(res.sky.blockage_count));
  stat.add(
    "sky_blockage_range_deg", "[" + std::to_string(res.sky.blockage_start_deg) + "," +
                                std::to_string(res.sky.blockage_end_deg) + "]");
  // TODO(badai-nguyen): consider sky_blockage_ratio_ for DiagnosticsStatus." [todo]

  auto level = DiagnosticStatus::OK;
  std::string msg = "OK";
  if (res.ground.blockage_ratio < 0) {
    level = DiagnosticStatus::STALE;
    msg = "STALE";
  } else if (
    (res.ground.blockage_ratio > blockage_config_.blockage_ratio_threshold) &&
    (res.ground.blockage_count > blockage_config_.blockage_count_threshold)) {
    level = DiagnosticStatus::ERROR;
    msg = "ERROR";
  } else if (res.ground.blockage_ratio > 0.0f) {
    level = DiagnosticStatus::WARN;
    msg = "WARN";
  }

  if ((res.ground.blockage_ratio > 0.0f) && (res.sky.blockage_ratio > 0.0f)) {
    msg = msg + ": LIDAR both blockage";
  } else if (res.ground.blockage_ratio > 0.0f) {
    msg = msg + ": LIDAR ground blockage";
  } else if (res.sky.blockage_ratio > 0.0f) {
    msg = msg + ": LIDAR sky blockage";
  }
  stat.summary(level, msg);
}

void BlockageDiagComponent::run_dust_check(diagnostic_updater::DiagnosticStatusWrapper & stat) const
{
  stat.add("ground_dust_ratio", std::to_string(dust_result_.ground_dust_ratio));
  auto level = DiagnosticStatus::OK;
  std::string msg = "OK";
  if (dust_result_.ground_dust_ratio < 0.0f) {
    level = DiagnosticStatus::STALE;
    msg = "STALE";
  } else if (
    (dust_result_.ground_dust_ratio > dust_config_.dust_ratio_threshold) &&
    (dust_result_.dust_frame_count > dust_config_.dust_count_threshold)) {
    level = DiagnosticStatus::ERROR;
    msg = "ERROR";
  } else if (dust_result_.ground_dust_ratio > 0.0f) {
    level = DiagnosticStatus::WARN;
    msg = "WARN";
  }

  if (dust_result_.ground_dust_ratio > 0.0f) {
    msg = msg + ": LIDAR ground dust";
  }
  stat.summary(level, msg);
}

cv::Mat BlockageDiagComponent::quantize_to_8u(const cv::Mat & image_16u) const
{
  assert(image_16u.type() == CV_16UC1);
  auto dimensions = image_16u.size();

  cv::Mat image_8u(dimensions, CV_8UC1, cv::Scalar(0));
  // UINT16_MAX = 65535, UINT8_MAX = 255, so downscale by ceil(65535 / 255) = 256.
  image_16u.convertTo(image_8u, CV_8UC1, 1.0 / 256);
  return image_8u;
}

cv::Mat BlockageDiagComponent::make_no_return_mask(const cv::Mat & depth_image) const
{
  assert(depth_image.type() == CV_8UC1);
  auto dimensions = depth_image.size();

  cv::Mat no_return_mask(dimensions, CV_8UC1, cv::Scalar(0));
  cv::inRange(depth_image, 0, 1, no_return_mask);

  return no_return_mask;
}

cv::Mat BlockageDiagComponent::make_blockage_mask(const cv::Mat & no_return_mask) const
{
  assert(no_return_mask.type() == CV_8UC1);
  auto dimensions = no_return_mask.size();

  int kernel_size = 2 * blockage_config_.blockage_kernel + 1;
  int kernel_center = blockage_config_.blockage_kernel;
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
  if (blockage_visualize_data_.buffering_interval == 0) {
    return blockage_mask.clone();
  }

  assert(blockage_mask.type() == CV_8UC1);
  auto dimensions = blockage_mask.size();

  cv::Mat time_series_blockage_result(dimensions, CV_8UC1, cv::Scalar(0));
  cv::Mat time_series_blockage_mask(dimensions, CV_8UC1, cv::Scalar(0));
  cv::Mat no_return_mask_binarized(dimensions, CV_8UC1, cv::Scalar(0));

  no_return_mask_binarized = blockage_mask / 255;
  if (blockage_visualize_data_.frame_count >= blockage_visualize_data_.buffering_interval) {
    blockage_visualize_data_.mask_buffer.push_back(no_return_mask_binarized);
    blockage_visualize_data_.frame_count = 0;
  } else {
    blockage_visualize_data_.frame_count++;
  }

  for (const auto & binary_mask : blockage_visualize_data_.mask_buffer) {
    time_series_blockage_mask += binary_mask;
  }

  cv::inRange(
    time_series_blockage_mask, blockage_visualize_data_.mask_buffer.size() - 1,
    blockage_visualize_data_.mask_buffer.size(), time_series_blockage_result);

  return time_series_blockage_result;
}

std::pair<cv::Mat, cv::Mat> BlockageDiagComponent::segment_into_ground_and_sky(
  const cv::Mat & mask) const
{
  assert(mask.type() == CV_8UC1);
  auto dimensions = mask.size();

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

void BlockageDiagComponent::update_blockage_info(
  const cv::Mat & blockage_mask, BlockageAreaResult & area_result)
{
  if (area_result.blockage_ratio <= blockage_config_.blockage_ratio_threshold) {
    area_result.blockage_count = 0;
    return;
  }

  cv::Rect blockage_bb = cv::boundingRect(blockage_mask);
  double blockage_start_deg = blockage_bb.x * horizontal_resolution_ + angle_range_deg_[0];
  double blockage_end_deg =
    (blockage_bb.x + blockage_bb.width) * horizontal_resolution_ + angle_range_deg_[0];

  area_result.blockage_start_deg = static_cast<float>(blockage_start_deg);
  area_result.blockage_end_deg = static_cast<float>(blockage_end_deg);

  if (area_result.blockage_count <= 2 * blockage_config_.blockage_count_threshold) {
    area_result.blockage_count += 1;
  }
}

cv::Mat BlockageDiagComponent::compute_dust_diagnostics(const cv::Mat & depth_image_16u)
{
  cv::Mat depth_image_8u = quantize_to_8u(depth_image_16u);
  cv::Mat no_return_mask = make_no_return_mask(depth_image_8u);

  assert(no_return_mask.type() == CV_8UC1);
  auto dimensions = no_return_mask.size();

  auto [single_dust_ground_img, sky_blank] = segment_into_ground_and_sky(no_return_mask);

  // It is normal for the sky region to be blank, therefore ignore it.
  sky_blank.setTo(cv::Scalar(0));

  int kernel_size = 2 * dust_config_.dust_kernel_size + 1;
  int kernel_center = dust_config_.dust_kernel_size;
  cv::Mat kernel = cv::getStructuringElement(
    cv::MORPH_RECT, cv::Size(kernel_size, kernel_size), cv::Point(kernel_center, kernel_center));

  cv::dilate(single_dust_ground_img, single_dust_ground_img, kernel);
  cv::erode(single_dust_ground_img, single_dust_ground_img, kernel);
  cv::inRange(single_dust_ground_img, 254, 255, single_dust_ground_img);

  // Re-assemble the processed ground dust image and the sky blank.
  cv::Mat single_dust_img(dimensions, CV_8UC1, cv::Scalar(0));
  cv::vconcat(sky_blank, single_dust_ground_img, single_dust_img);

  dust_result_.ground_dust_ratio = static_cast<float>(cv::countNonZero(single_dust_ground_img)) /
                                   (single_dust_ground_img.cols * single_dust_ground_img.rows);

  if (dust_result_.ground_dust_ratio > dust_config_.dust_ratio_threshold) {
    if (dust_result_.dust_frame_count < 2 * dust_config_.dust_count_threshold) {
      dust_result_.dust_frame_count++;
    }
  } else {
    dust_result_.dust_frame_count = 0;
  }

  return single_dust_img;
}

void BlockageDiagComponent::publish_dust_debug_info(
  const DebugInfo & debug_info, const cv::Mat & single_dust_img)
{
  autoware_internal_debug_msgs::msg::Float32Stamped ground_dust_ratio_msg;
  ground_dust_ratio_msg.data = dust_result_.ground_dust_ratio;
  ground_dust_ratio_msg.stamp = now();
  ground_dust_ratio_pub_->publish(ground_dust_ratio_msg);

  if (publish_debug_image_) {
    auto dimensions = single_dust_img.size();
    cv::Mat binarized_dust_mask_(dimensions, CV_8UC1, cv::Scalar(0));
    cv::Mat multi_frame_dust_mask(dimensions, CV_8UC1, cv::Scalar(0));
    cv::Mat multi_frame_ground_dust_result(dimensions, CV_8UC1, cv::Scalar(0));

    if (dust_visualize_data_.buffering_interval == 0) {
      single_dust_img.copyTo(multi_frame_ground_dust_result);
      dust_visualize_data_.frame_count = 0;
    } else {
      binarized_dust_mask_ = single_dust_img / 255;
      if (dust_visualize_data_.frame_count >= dust_visualize_data_.buffering_interval) {
        dust_visualize_data_.mask_buffer.push_back(binarized_dust_mask_);
        dust_visualize_data_.frame_count = 0;
      } else {
        dust_visualize_data_.frame_count++;
      }
      for (const auto & binarized_dust_mask : dust_visualize_data_.mask_buffer) {
        multi_frame_dust_mask += binarized_dust_mask;
      }
      cv::inRange(
        multi_frame_dust_mask, dust_visualize_data_.mask_buffer.size() - 1,
        dust_visualize_data_.mask_buffer.size(), multi_frame_ground_dust_result);
    }

    // Publish single-frame dust mask image with color map
    cv::Mat single_frame_ground_dust_colorized(dimensions, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::applyColorMap(single_dust_img, single_frame_ground_dust_colorized, cv::COLORMAP_JET);
    sensor_msgs::msg::Image::SharedPtr single_frame_dust_mask_msg =
      cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", single_frame_ground_dust_colorized)
        .toImageMsg();
    single_frame_dust_mask_pub.publish(single_frame_dust_mask_msg);

    // Publish multi-frame dust mask image with color map
    cv::Mat multi_frame_ground_dust_colorized;
    cv::applyColorMap(
      multi_frame_ground_dust_result, multi_frame_ground_dust_colorized, cv::COLORMAP_JET);
    sensor_msgs::msg::Image::SharedPtr multi_frame_dust_mask_msg =
      cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", multi_frame_ground_dust_colorized)
        .toImageMsg();
    multi_frame_dust_mask_pub.publish(multi_frame_dust_mask_msg);

    // Publish blockage and dust merged image
    cv::Mat blockage_dust_merged_img(dimensions, CV_8UC3, cv::Scalar(0, 0, 0));
    blockage_dust_merged_img.setTo(
      cv::Vec3b(0, 0, 255), debug_info.blockage_mask_multi_frame);  // red:blockage
    blockage_dust_merged_img.setTo(
      cv::Vec3b(0, 255, 255), multi_frame_ground_dust_result);  // yellow:dust
    sensor_msgs::msg::Image::SharedPtr blockage_dust_merged_msg =
      cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", blockage_dust_merged_img).toImageMsg();
    blockage_dust_merged_msg->header = debug_info.input_header;
    blockage_dust_merged_pub.publish(blockage_dust_merged_msg);
  }
}

void BlockageDiagComponent::publish_blockage_debug_info(const DebugInfo & debug_info) const
{
  autoware_internal_debug_msgs::msg::Float32Stamped ground_blockage_ratio_msg;
  ground_blockage_ratio_msg.data = blockage_result_.ground.blockage_ratio;
  ground_blockage_ratio_msg.stamp = now();
  ground_blockage_ratio_pub_->publish(ground_blockage_ratio_msg);

  autoware_internal_debug_msgs::msg::Float32Stamped sky_blockage_ratio_msg;
  sky_blockage_ratio_msg.data = blockage_result_.sky.blockage_ratio;
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

cv::Mat BlockageDiagComponent::compute_blockage_diagnostics(const cv::Mat & depth_image_16u)
{
  cv::Mat depth_image_8u = quantize_to_8u(depth_image_16u);
  cv::Mat no_return_mask = make_no_return_mask(depth_image_8u);
  cv::Mat blockage_mask = make_blockage_mask(no_return_mask);
  cv::Mat time_series_blockage_result = update_time_series_blockage_mask(blockage_mask);

  auto [ground_blockage_mask, sky_blockage_mask] = segment_into_ground_and_sky(blockage_mask);

  blockage_result_.ground.blockage_ratio = get_nonzero_ratio(ground_blockage_mask);
  blockage_result_.sky.blockage_ratio = get_nonzero_ratio(sky_blockage_mask);

  update_blockage_info(ground_blockage_mask, blockage_result_.ground);
  update_blockage_info(sky_blockage_mask, blockage_result_.sky);

  return time_series_blockage_result;
}

void BlockageDiagComponent::update_diagnostics(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & input)
{
  try {
    validate_pointcloud_fields(*input);
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    return;
  }

  cv::Mat depth_image_16u = depth_image_converter_->make_normalized_depth_image(*input);

  // Blockage detection
  cv::Mat time_series_blockage_result = compute_blockage_diagnostics(depth_image_16u);
  const DebugInfo debug_info = {input->header, depth_image_16u, time_series_blockage_result};
  publish_blockage_debug_info(debug_info);

  // Dust detection
  if (enable_dust_diag_) {
    cv::Mat single_frame_dust_mask = compute_dust_diagnostics(depth_image_16u);
    publish_dust_debug_info(debug_info, single_frame_dust_mask);
  }
}
}  // namespace autoware::pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::pointcloud_preprocessor::BlockageDiagComponent)
