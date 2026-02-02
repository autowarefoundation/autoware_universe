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

#include "autoware/pointcloud_preprocessor/blockage_diag/blockage_diag.hpp"
#include "autoware/pointcloud_preprocessor/blockage_diag/pointcloud2_to_depth_image.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/float32_stamped.hpp>
#include <autoware_internal_debug_msgs/msg/string_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/header.hpp>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <boost/circular_buffer.hpp>

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
  struct DebugInfo
  {
    std_msgs::msg::Header input_header;
    cv::Mat depth_image_16u;
    cv::Mat blockage_mask_multi_frame;
  };

  void run_blockage_check(DiagnosticStatusWrapper & stat) const;
  void run_dust_check(DiagnosticStatusWrapper & stat) const;

  /**
   * @brief Make a binary, cleaned blockage mask from the input no-return mask.
   *
   * @param no_return_mask A mask where 255 is no-return and 0 is return.
   * @return cv::Mat The blockage mask. The data type is `CV_8UC1`.
   */
  cv::Mat make_blockage_mask(const cv::Mat & no_return_mask) const;

  /**
   * @brief Get the ratio of non-zero pixels in a given mask.
   *
   * @param mask The input mask. The data type is `CV_8UC1`.
   * @return float The ratio of non-zero pixels (e.g. 1.0 if all are non-zero, 0.0 if all are zero).
   */
  static float get_nonzero_ratio(const cv::Mat & mask);

  /**
   * @brief Update the blockage info for a specific area (ground or sky).
   *
   * @param blockage_mask The blockage mask. The data type is `CV_8UC1`.
   * @param area_result Reference to the BlockageAreaResult to update.
   */
  void update_blockage_info(const cv::Mat & blockage_mask, BlockageAreaResult & area_result);

  /**
   * @brief Compute blockage diagnostics and update the internal blockage info.
   *
   * @param depth_image_16u The input depth image. The data type is `CV_16UC1`.
   */
  cv::Mat compute_blockage_diagnostics(const cv::Mat & depth_image_16u);

  /**
   * @brief Publish the debug info of blockage diagnostics if enabled.
   *
   * @param debug_info The debug info to publish.
   */
  void publish_blockage_debug_info(const DebugInfo & debug_info) const;

  /**
   * @brief Publish the debug info of dust diagnostics if enabled.
   *
   * @param debug_info The debug info to publish.
   */
  void publish_dust_debug_info(const DebugInfo & debug_info, const cv::Mat & single_dust_img);

  Updater updater_{this};

  // PointCloud2 to depth image converter
  std::unique_ptr<pointcloud2_to_depth_image::PointCloud2ToDepthImage> depth_image_converter_;

  // Debug parameters
  bool publish_debug_image_;

  // Mask size parameters
  std::vector<double> angle_range_deg_;
  double horizontal_resolution_{0.4};

  // Ground/sky segmentation parameters
  int horizontal_ring_id_;

  // Blockage detection
  BlockageDetectionConfig blockage_config_;
  BlockageDetectionResult blockage_result_;
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
