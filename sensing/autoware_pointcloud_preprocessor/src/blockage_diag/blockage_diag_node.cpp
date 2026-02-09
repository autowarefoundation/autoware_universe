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
    std::vector<double> angle_range_deg = declare_parameter<std::vector<double>>("angle_range");
    // Whether the channel order is top-down (true) or bottom-up (false)
    bool is_channel_order_top2down = declare_parameter<bool>("is_channel_order_top2down");

    // Blockage mask format configuration
    // The number of vertical bins in the mask. Has to equal the number of channels of the LiDAR.
    int vertical_bins = declare_parameter<int>("vertical_bins");
    // The angular resolution of the mask, in degrees.
    double horizontal_resolution = declare_parameter<double>("horizontal_resolution");

    // Multi-frame blockage aggregation configuration
    MultiFrameDetectionAggregatorConfig blockage_aggregator_config;
    blockage_aggregator_config.buffering_frames =
      declare_parameter<int>("blockage_buffering_frames");
    blockage_aggregator_config.buffering_interval =
      declare_parameter<int>("blockage_buffering_interval");
    blockage_aggregator_ =
      std::make_unique<MultiFrameDetectionAggregator>(blockage_aggregator_config);

    // Debug configuration
    publish_debug_image_ = declare_parameter<bool>("publish_debug_image");

    // Depth map configuration
    // The maximum distance range of the LiDAR, in meters. The depth map is normalized to this
    // value.
    double max_distance_range = declare_parameter<double>("max_distance_range");

    // Ground segmentation configuration
    // The ring ID that coincides with the horizon. Regions below are treated as ground,
    // regions above are treated as sky.
    int horizontal_ring_id = declare_parameter<int>("horizontal_ring_id");

    // Validate parameters
    if (vertical_bins <= horizontal_ring_id) {
      RCLCPP_ERROR(
        this->get_logger(),
        "The horizontal_ring_id should be smaller than vertical_bins. Skip blockage diag!");
      return;
    }

    // Initialize PointCloud2ToDepthImage converter
    pointcloud2_to_depth_image::ConverterConfig depth_image_config;
    depth_image_config.horizontal.angle_range_min_deg = angle_range_deg[0];
    depth_image_config.horizontal.angle_range_max_deg = angle_range_deg[1];
    depth_image_config.horizontal.horizontal_resolution = horizontal_resolution;
    depth_image_config.vertical.vertical_bins = vertical_bins;
    depth_image_config.vertical.is_channel_order_top2down = is_channel_order_top2down;
    depth_image_config.max_distance_range = max_distance_range;
    depth_image_converter_ =
      std::make_unique<pointcloud2_to_depth_image::PointCloud2ToDepthImage>(depth_image_config);

    // Initialize BlockageDetector
    BlockageDetectionConfig blockage_config;
    blockage_config.blockage_ratio_threshold = declare_parameter<float>("blockage_ratio_threshold");
    blockage_config.blockage_count_threshold = declare_parameter<int>("blockage_count_threshold");
    blockage_config.blockage_kernel = declare_parameter<int>("blockage_kernel");
    blockage_config.horizontal_ring_id = horizontal_ring_id;
    blockage_config.horizontal_resolution = horizontal_resolution;
    blockage_config.angle_range_min_deg = angle_range_deg[0];
    blockage_config.angle_range_max_deg = angle_range_deg[1];
    blockage_detector_ = std::make_unique<BlockageDetector>(blockage_config);

    // Initialize DustDetector
    enable_dust_diag_ = declare_parameter<bool>("enable_dust_diag");
    DustDetectionConfig dust_config;
    dust_config.dust_ratio_threshold = declare_parameter<float>("dust_ratio_threshold");
    dust_config.dust_count_threshold = declare_parameter<int>("dust_count_threshold");
    dust_config.dust_kernel_size = declare_parameter<int>("dust_kernel_size");
    dust_config.horizontal_ring_id = horizontal_ring_id;
    dust_detector_ = std::make_unique<DustDetector>(dust_config);
    // Multi-frame dust aggregation configuration
    MultiFrameDetectionAggregatorConfig dust_aggregator_config;
    dust_aggregator_config.buffering_frames = declare_parameter<int>("dust_buffering_frames");
    dust_aggregator_config.buffering_interval = declare_parameter<int>("dust_buffering_interval");
    dust_aggregator_ = std::make_unique<MultiFrameDetectionAggregator>(dust_aggregator_config);
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

void update_diagnostics_status(
  diagnostic_updater::DiagnosticStatusWrapper & stat, const DiagnosticOutput & output)
{
  stat.summary(static_cast<unsigned char>(output.level), output.message);
  for (const auto & data : output.additional_data) {
    stat.add(data.key, data.value);
  }
}

void BlockageDiagComponent::run_blockage_check(DiagnosticStatusWrapper & stat) const
{
  DiagnosticOutput blockage_diagnostic = blockage_detector_->get_blockage_diagnostics_output();
  update_diagnostics_status(stat, blockage_diagnostic);
}

void BlockageDiagComponent::run_dust_check(diagnostic_updater::DiagnosticStatusWrapper & stat) const
{
  DiagnosticOutput dust_diagnostic = dust_detector_->get_dust_diagnostics_output();
  update_diagnostics_status(stat, dust_diagnostic);
}

void BlockageDiagComponent::publish_dust_debug_info(
  const DustDetectionResult & dust_result, const std_msgs::msg::Header & input_header,
  const cv::Mat & blockage_mask_multi_frame)
{
  autoware_internal_debug_msgs::msg::Float32Stamped ground_dust_ratio_msg;
  ground_dust_ratio_msg.data = dust_result.ground_dust_ratio;
  ground_dust_ratio_msg.stamp = now();
  ground_dust_ratio_pub_->publish(ground_dust_ratio_msg);

  if (publish_debug_image_) {
    auto dimensions = dust_result.dust_mask.size();
    cv::Mat multi_frame_ground_dust_result = dust_aggregator_->update(dust_result.dust_mask);

    // Publish single-frame dust mask image with color map
    cv::Mat single_frame_ground_dust_colorized(dimensions, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::applyColorMap(dust_result.dust_mask, single_frame_ground_dust_colorized, cv::COLORMAP_JET);
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
      cv::Vec3b(0, 0, 255), blockage_mask_multi_frame);  // red:blockage
    blockage_dust_merged_img.setTo(
      cv::Vec3b(0, 255, 255), multi_frame_ground_dust_result);  // yellow:dust
    sensor_msgs::msg::Image::SharedPtr blockage_dust_merged_msg =
      cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", blockage_dust_merged_img).toImageMsg();
    blockage_dust_merged_msg->header = input_header;
    blockage_dust_merged_pub.publish(blockage_dust_merged_msg);
  }
}

void BlockageDiagComponent::publish_blockage_debug_info(
  const BlockageDetectionResult & blockage_result, const std_msgs::msg::Header & input_header,
  const cv::Mat & depth_image_16u, const cv::Mat & blockage_mask_multi_frame) const
{
  autoware_internal_debug_msgs::msg::Float32Stamped ground_blockage_ratio_msg;
  ground_blockage_ratio_msg.data = blockage_result.ground.blockage_ratio;
  ground_blockage_ratio_msg.stamp = now();
  ground_blockage_ratio_pub_->publish(ground_blockage_ratio_msg);

  autoware_internal_debug_msgs::msg::Float32Stamped sky_blockage_ratio_msg;
  sky_blockage_ratio_msg.data = blockage_result.sky.blockage_ratio;
  sky_blockage_ratio_msg.stamp = now();
  sky_blockage_ratio_pub_->publish(sky_blockage_ratio_msg);

  if (publish_debug_image_) {
    sensor_msgs::msg::Image::SharedPtr lidar_depth_map_msg =
      cv_bridge::CvImage(std_msgs::msg::Header(), "mono16", depth_image_16u).toImageMsg();
    lidar_depth_map_msg->header = input_header;
    lidar_depth_map_pub_.publish(lidar_depth_map_msg);

    cv::Mat blockage_mask_colorized;
    cv::applyColorMap(blockage_mask_multi_frame, blockage_mask_colorized, cv::COLORMAP_JET);
    sensor_msgs::msg::Image::SharedPtr blockage_mask_msg =
      cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", blockage_mask_colorized).toImageMsg();
    blockage_mask_msg->header = input_header;
    blockage_mask_pub_.publish(blockage_mask_msg);
  }
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
  BlockageDetectionResult blockage_result =
    blockage_detector_->compute_blockage_diagnostics(depth_image_16u);
  cv::Mat multi_frame_blockage_mask = blockage_aggregator_->update(blockage_result.blockage_mask);
  publish_blockage_debug_info(
    blockage_result, input->header, depth_image_16u, multi_frame_blockage_mask);

  // Dust detection
  if (enable_dust_diag_) {
    DustDetectionResult dust_result = dust_detector_->compute_dust_diagnostics(depth_image_16u);
    publish_dust_debug_info(dust_result, input->header, multi_frame_blockage_mask);
  }
}
}  // namespace autoware::pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::pointcloud_preprocessor::BlockageDiagComponent)
