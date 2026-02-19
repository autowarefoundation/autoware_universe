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

#ifndef AUTOWARE__POINTCLOUD_PREPROCESSOR__BLOCKAGE_DIAG__BLOCKAGE_DIAG_NODE_HPP_
#define AUTOWARE__POINTCLOUD_PREPROCESSOR__BLOCKAGE_DIAG__BLOCKAGE_DIAG_NODE_HPP_

#include "autoware/pointcloud_preprocessor/blockage_diag/blockage_detection.hpp"
#include "autoware/pointcloud_preprocessor/blockage_diag/dust_detection.hpp"
#include "autoware/pointcloud_preprocessor/blockage_diag/multi_frame_detection_aggregator.hpp"
#include "autoware/pointcloud_preprocessor/blockage_diag/pointcloud2_to_depth_image.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core/mat.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/float32_stamped.hpp>
#include <autoware_internal_debug_msgs/msg/string_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <memory>
#include <utility>
#include <vector>

namespace autoware::pointcloud_preprocessor
{
using diagnostic_updater::DiagnosticStatusWrapper;
using diagnostic_updater::Updater;

class BlockageDiagComponent : public rclcpp::Node
{
private:
  /** \brief Parameter service callback result : needed to be hold */
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  /** \brief Parameter service callback */
  rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter> & p);
  image_transport::Publisher lidar_depth_map_pub_;
  image_transport::Publisher blockage_mask_pub_;
  image_transport::Publisher single_frame_dust_mask_pub;
  image_transport::Publisher multi_frame_dust_mask_pub;
  image_transport::Publisher blockage_dust_merged_pub;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float32Stamped>::SharedPtr
    ground_blockage_ratio_pub_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float32Stamped>::SharedPtr
    sky_blockage_ratio_pub_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float32Stamped>::SharedPtr
    ground_dust_ratio_pub_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::StringStamped>::SharedPtr blockage_type_pub_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  void update_diagnostics(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & input);
  void run_blockage_check(DiagnosticStatusWrapper & stat) const;
  void run_dust_check(DiagnosticStatusWrapper & stat) const;

  /**
   * @brief Publish the debug info of blockage diagnostics if enabled.
   *
   * @param blockage_result The blockage detection result.
   * @param input_header The header of the input point cloud.
   * @param depth_image_16u The depth image converted from the input point cloud.
   * @param blockage_mask_multi_frame The multi-frame blockage mask.
   */
  void publish_blockage_debug_info(
    const BlockageDetectionResult & blockage_result, const std_msgs::msg::Header & input_header,
    const cv::Mat & depth_image_16u, const cv::Mat & blockage_mask_multi_frame) const;

  /**
   * @brief Publish the debug info of dust diagnostics if enabled.
   *
   * @param dust_result The dust detection result.
   * @param input_header The header of the input point cloud.
   * @param blockage_mask_multi_frame The multi-frame blockage mask.
   */
  void publish_dust_debug_info(
    const DustDetectionResult & dust_result, const std_msgs::msg::Header & input_header,
    const cv::Mat & blockage_mask_multi_frame);

  Updater updater_{this};

  // PointCloud2 to depth image converter
  std::unique_ptr<pointcloud2_to_depth_image::PointCloud2ToDepthImage> depth_image_converter_;

  // Debug parameters
  bool publish_debug_image_;

  // Blockage detection
  std::unique_ptr<BlockageDetector> blockage_detector_;
  std::unique_ptr<MultiFrameDetectionAggregator> blockage_aggregator_;

  // Dust detection
  bool enable_dust_diag_;
  std::unique_ptr<DustDetector> dust_detector_;
  std::unique_ptr<MultiFrameDetectionAggregator> dust_aggregator_;

public:
  explicit BlockageDiagComponent(const rclcpp::NodeOptions & options);
};

}  // namespace autoware::pointcloud_preprocessor

#endif  // AUTOWARE__POINTCLOUD_PREPROCESSOR__BLOCKAGE_DIAG__BLOCKAGE_DIAG_NODE_HPP_
