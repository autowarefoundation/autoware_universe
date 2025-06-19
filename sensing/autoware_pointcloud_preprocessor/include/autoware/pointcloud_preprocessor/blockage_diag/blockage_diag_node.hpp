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

#include "autoware/point_types/types.hpp"
#include "autoware/pointcloud_preprocessor/filter.hpp"

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

#include <pcl/PCLPointCloud2.h>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <boost/circular_buffer.hpp>

#include <utility>
#include <vector>

namespace autoware::pointcloud_preprocessor
{
using autoware::point_types::PointXYZIRCAEDT;
using diagnostic_updater::DiagnosticStatusWrapper;
using diagnostic_updater::Updater;
using PCLCloudXYZIRCAEDT = pcl::PointCloud<PointXYZIRCAEDT>;

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
  struct DebugInfo
  {
    std_msgs::msg::Header input_header;
    cv::Mat depth_image_16u;
    cv::Mat blockage_mask_multi_frame;
  };

  void run_blockage_check(DiagnosticStatusWrapper & stat) const;
  void run_dust_check(DiagnosticStatusWrapper & stat) const;

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

  /**
   * @brief Make a downsampled depth image from the input point cloud, normalized to 0-35565.
   *
   * The size of the output is given by `get_mask_dimensions()`.
   * Close depth values are mapped to higher values, far depth values are mapped to lower values.
   * The `max_distance_range_` is mapped to 0, and a LiDAR distance of 0 is mapped to UINT16_MAX.
   *
   * @param input The input point cloud.
   * @return cv::Mat The normalized depth image. The data type is `CV_16UC1`.
   */
  cv::Mat make_normalized_depth_image(const PCLCloudXYZIRCAEDT & input) const;

  /**
   * @brief Quantize a 16-bit image to 8-bit.
   *
   * The values are scaled by `1.0 / 300` to prevent overflow.
   *
   * @param image_16u The input 16-bit image.
   * @return cv::Mat The quantized 8-bit image. The data type is `CV_8UC1`.
   */
  cv::Mat quantize_to_8u(const cv::Mat & image_16u) const;

  /**
   * @brief Make a no-return mask from the input depth image.
   *
   * The mask is a binary image where 255 is no-return and 0 is return.
   *
   * @param depth_image The input depth image.
   * @return cv::Mat The no-return mask. The data type is `CV_8UC1`.
   */
  cv::Mat make_no_return_mask(const cv::Mat & depth_image) const;

  /**
   * @brief Make a binary, cleaned blockage mask from the input no-return mask.
   *
   * @param no_return_mask A mask where 255 is no-return and 0 is return.
   * @return cv::Mat The blockage mask. The data type is `CV_8UC1`.
   */
  cv::Mat make_blockage_mask(const cv::Mat & no_return_mask) const;

  /**
   * @brief Update the internal blockage mask buffer and return the updated mask.
   *
   * @param blockage_mask The current blockage mask. The data type is `CV_8UC1`.
   * @return cv::Mat The updated aggregated blockage mask. The data type is `CV_8UC1`.
   */
  cv::Mat update_time_series_blockage_mask(const cv::Mat & blockage_mask);

  /**
   * @brief Segments a given mask into two masks, according to the ground/sky segmentation
   * parameters.
   *
   * @param mask The input mask. The data type is `CV_8UC1`.
   * @return std::pair<cv::Mat, cv::Mat> The pair {ground_mask, sky_mask}. The data type is
   * `CV_8UC1`.
   */
  std::pair<cv::Mat, cv::Mat> segment_into_ground_and_sky(const cv::Mat & mask) const;

  /**
   * @brief Get the ratio of non-zero pixels in a given mask.
   *
   * @param mask The input mask. The data type is `CV_8UC1`.
   * @return float The ratio of non-zero pixels (e.g. 1.0 if all are non-zero, 0.0 if all are zero).
   */
  static float get_nonzero_ratio(const cv::Mat & mask);

  /**
   * @brief Update the internal ground blockage info.
   *
   * @param ground_blockage_mask The ground blockage mask. The data type is `CV_8UC1`.
   */
  void update_ground_blockage_info(const cv::Mat & ground_blockage_mask);

  /**
   * @brief Update the internal sky blockage info.
   *
   * @param sky_blockage_mask The sky blockage mask. The data type is `CV_8UC1`.
   */
  void update_sky_blockage_info(const cv::Mat & sky_blockage_mask);

  /**
   * @brief Compute dust diagnostics and update the internal dust info.
   *
   * @param no_return_mask The no-return mask. The data type is `CV_8UC1`.
   * @param debug_info The debug info to publish if enabled.
   */
  void compute_dust_diagnostics(const cv::Mat & no_return_mask, const DebugInfo & debug_info);

  /**
   * @brief Publish the debug info if enabled.
   *
   * @param debug_info The debug info to publish.
   */
  void publish_debug_info(const DebugInfo & debug_info) const;

  Updater updater_{this};

  // Debug parameters
  bool publish_debug_image_;

  // LiDAR parameters
  double max_distance_range_{200.0};

  // Mask size parameters
  int vertical_bins_;
  std::vector<double> angle_range_deg_;
  double horizontal_resolution_{0.4};

  // Ground/sky segmentation parameters
  bool is_channel_order_top2down_;
  int horizontal_ring_id_;

  // Blockage detection parameters
  float blockage_ratio_threshold_;
  int blockage_kernel_ = 10;
  int blockage_buffering_frames_;
  int blockage_buffering_interval_;
  int blockage_count_threshold_;

  // Blockage detection state
  float ground_blockage_ratio_ = -1.0f;
  float sky_blockage_ratio_ = -1.0f;
  int ground_blockage_count_ = 0;
  int sky_blockage_count_ = 0;
  std::vector<float> ground_blockage_range_deg_ = {0.0f, 0.0f};
  std::vector<float> sky_blockage_range_deg_ = {0.0f, 0.0f};

  // Multi-frame blockage detection state
  int blockage_frame_count_ = 0;
  boost::circular_buffer<cv::Mat> no_return_mask_buffer{1};

  // Dust detection parameters
  bool enable_dust_diag_;
  float dust_ratio_threshold_;
  int dust_kernel_size_;
  int dust_buffering_frames_;
  int dust_buffering_interval_;
  int dust_count_threshold_;

  // Dust detection state
  float ground_dust_ratio_ = -1.0f;

  // Multi-frame dust detection state
  int dust_buffering_frame_counter_ = 0;
  int dust_frame_count_ = 0;
  boost::circular_buffer<cv::Mat> dust_mask_buffer{1};

public:
  PCL_MAKE_ALIGNED_OPERATOR_NEW
  explicit BlockageDiagComponent(const rclcpp::NodeOptions & options);
};

}  // namespace autoware::pointcloud_preprocessor

#endif  // AUTOWARE__POINTCLOUD_PREPROCESSOR__BLOCKAGE_DIAG__BLOCKAGE_DIAG_NODE_HPP_
