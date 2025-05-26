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

#include "autoware/pointcloud_preprocessor/filter.hpp"
#include "autoware/point_types/types.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/float32_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <boost/circular_buffer.hpp>

#include <string>
#include <vector>

namespace autoware::pointcloud_preprocessor
{
using diagnostic_updater::DiagnosticStatusWrapper;
using diagnostic_updater::Updater;
using autoware::point_types::PointXYZIRCAEDT;

class BlockageDiagComponent : public autoware::pointcloud_preprocessor::Filter
{
protected:
  void filter(
    const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output) override;
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

private:
  void run_blockage_check(DiagnosticStatusWrapper & stat);
  void run_dust_check(DiagnosticStatusWrapper & stat);

  /**
   * @brief Get the horizontal bin index of the given azimuth, if within the FoV.
   *
   * If the FoV wraps around, the azimuth is adjusted to be within the FoV.
   * The bin is calculated as `(azimuth_deg - min_deg) / horizontal_resolution_` and any
   * azimuth for which `min_deg < azimuth_deg <= max_deg` is valid.
   *
   * @param azimuth_deg The azimuth to get the bin index for.
   * @return std::optional<int> The bin index if valid, otherwise `std::nullopt`.
   */
  std::optional<int> get_horizontal_bin(double azimuth_deg) const;

  /**
   * @brief Get the vertical bin index of the given channel, if within the FoV.
   *
   * Vertical bins and channels are usually equivalent, apart from the 0-based index of bins.
   * If `is_channel_order_top2down_` is `false`, the bin order is reversed compared to the channel
   * order.
   *
   * @param channel The channel to get the bin index for.
   * @return std::optional<int> The bin index if valid, otherwise `std::nullopt`.
   */
  std::optional<int> get_vertical_bin(uint16_t channel) const;

  /**
   * @brief Get the dimensions of the mask, i.e. the number of horizontal and vertical bins.
   *
   * @return cv::Size The dimensions of the mask.
   */
  cv::Size get_mask_dimensions() const;

  Updater updater_{this};
  int vertical_bins_;
  std::vector<double> angle_range_deg_;
  int horizontal_ring_id_;
  float blockage_ratio_threshold_;
  float dust_ratio_threshold_;
  float ground_blockage_ratio_ = -1.0f;
  float sky_blockage_ratio_ = -1.0f;
  float ground_dust_ratio_ = -1.0f;
  std::vector<float> ground_blockage_range_deg_ = {0.0f, 0.0f};
  std::vector<float> sky_blockage_range_deg_ = {0.0f, 0.0f};
  int blockage_kernel_ = 10;
  int blockage_frame_count_ = 0;
  int ground_blockage_count_ = 0;
  int sky_blockage_count_ = 0;
  int blockage_count_threshold_;
  bool is_channel_order_top2down_;
  int blockage_buffering_frames_;
  int blockage_buffering_interval_;
  bool enable_dust_diag_;
  bool publish_debug_image_;
  int dust_kernel_size_;
  int dust_buffering_frames_;
  int dust_buffering_interval_;
  int dust_buffering_frame_counter_ = 0;
  int dust_count_threshold_;
  int dust_frame_count_ = 0;
  double max_distance_range_{200.0};
  double horizontal_resolution_{0.4};
  boost::circular_buffer<cv::Mat> no_return_mask_buffer{1};
  boost::circular_buffer<cv::Mat> dust_mask_buffer{1};

public:
  PCL_MAKE_ALIGNED_OPERATOR_NEW
  explicit BlockageDiagComponent(const rclcpp::NodeOptions & options);
};

}  // namespace autoware::pointcloud_preprocessor

#endif  // AUTOWARE__POINTCLOUD_PREPROCESSOR__BLOCKAGE_DIAG__BLOCKAGE_DIAG_NODE_HPP_
