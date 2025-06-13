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

#ifndef AUTOWARE__POINTCLOUD_PREPROCESSOR__OUTLIER_FILTER__RING_OUTLIER_FILTER_NODE_HPP_
#define AUTOWARE__POINTCLOUD_PREPROCESSOR__OUTLIER_FILTER__RING_OUTLIER_FILTER_NODE_HPP_

#include "autoware/point_types/types.hpp"
#include "autoware/pointcloud_preprocessor/diagnostics/diagnostics_base.hpp"
#include "autoware/pointcloud_preprocessor/filter.hpp"
#include "autoware/pointcloud_preprocessor/transform_info.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <image_transport/image_transport.hpp>
#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::pointcloud_preprocessor
{
using point_cloud_msg_wrapper::PointCloud2Modifier;

class RingOutlierFilterComponent : public autoware::pointcloud_preprocessor::Filter
{
protected:
  using InputPointIndex = autoware::point_types::PointXYZIRCAEDTIndex;
  using InputPointType = autoware::point_types::PointXYZIRCAEDT;
  using OutputPointType = autoware::point_types::PointXYZIRC;

  void filter(
    const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output) override;

  // TODO(sykwer): Temporary Implementation: Remove this interface when all the filter nodes conform
  // to new API
  void faster_filter(
    const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output,
    const TransformInfo & transform_info) override;

  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float32Stamped>::SharedPtr visibility_pub_;

private:
  /** \brief publisher of excluded pointcloud for debug reason. **/
  rclcpp::Publisher<PointCloud2>::SharedPtr outlier_pointcloud_publisher_;

  double distance_ratio_;
  double object_length_threshold_;
  uint16_t max_rings_num_;
  size_t max_points_num_per_ring_;
  bool publish_outlier_pointcloud_;
  double processing_time_threshold_sec_;

  // for visibility score
  int noise_threshold_;
  int vertical_bins_;
  int horizontal_bins_;

  float min_azimuth_deg_;
  float max_azimuth_deg_;
  float max_distance_;

  /** \brief Parameter service callback result : needed to be hold */
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  /** \brief Parameter service callback */
  rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter> & p);

  bool is_cluster(const PointCloud2ConstPtr & input, std::pair<int, int> data_idx_both_ends) const
  {
    auto first_point =
      reinterpret_cast<const InputPointType *>(&input->data[data_idx_both_ends.first]);
    auto last_point =
      reinterpret_cast<const InputPointType *>(&input->data[data_idx_both_ends.second]);

    const auto x = first_point->x - last_point->x;
    const auto y = first_point->y - last_point->y;
    const auto z = first_point->z - last_point->z;

    return x * x + y * y + z * z >= object_length_threshold_ * object_length_threshold_;
  }

  void set_up_pointcloud_format(
    const PointCloud2ConstPtr & input, PointCloud2 & formatted_points, size_t points_size);
  float calculate_visibility_score(const PointCloud2 & input) const;
  void publish_diagnostics(const std::vector<std::shared_ptr<const DiagnosticsBase>> & diagnostics);

public:
  PCL_MAKE_ALIGNED_OPERATOR_NEW
  explicit RingOutlierFilterComponent(const rclcpp::NodeOptions & options);
};

}  // namespace autoware::pointcloud_preprocessor
#endif  // AUTOWARE__POINTCLOUD_PREPROCESSOR__OUTLIER_FILTER__RING_OUTLIER_FILTER_NODE_HPP_
