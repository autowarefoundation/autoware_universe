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

#ifndef AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_OUTLIER_FILTER__CUDA_POLAR_VOXEL_OUTLIER_FILTER_NODE_HPP_
#define AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_OUTLIER_FILTER__CUDA_POLAR_VOXEL_OUTLIER_FILTER_NODE_HPP_

#include "autoware/cuda_pointcloud_preprocessor/cuda_outlier_filter/cuda_polar_voxel_outlier_filter.hpp"

#include <cuda_blackboard/cuda_adaptation.hpp>
#include <cuda_blackboard/cuda_blackboard_publisher.hpp>
#include <cuda_blackboard/cuda_blackboard_subscriber.hpp>
#include <cuda_blackboard/cuda_pointcloud2.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/float32_stamped.hpp>

#include <mutex>
#include <vector>

namespace autoware::cuda_pointcloud_preprocessor
{
class CudaPolarVoxelOutlierFilterNode : public rclcpp::Node
{
public:
  explicit CudaPolarVoxelOutlierFilterNode(const rclcpp::NodeOptions & node_options);

protected:
  void pointcloud_callback(const cuda_blackboard::CudaPointCloud2::ConstSharedPtr msg);

  /** \brief Parameter service callback result : needed to be hold */
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  /** \brief Parameter service callback */
  rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter> & p);

private:
  CudaPolarVoxelOutlierFilterParameters filter_params_;
  // rclcpp always returns integer array as std::vector<int64_t>, so declare the same type here
  std::vector<int64_t> primary_return_types_;    // Return types considered as primary returns
  std::vector<int64_t> secondary_return_types_;  // Return types considered as secondary returns
  std::mutex param_mutex_;

  // Diagnostics members
  double visibility_;    // Current visibility value
  double filter_ratio_;  // Current filter ratio
  diagnostic_updater::Updater updater_{this};

  // CUDA sub
  std::shared_ptr<cuda_blackboard::CudaBlackboardSubscriber<cuda_blackboard::CudaPointCloud2>>
    sub_{};

  // CUDA pub
  std::unique_ptr<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>
    filtered_cloud_pub_{};
  std::unique_ptr<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>
    noise_cloud_pub_{};
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float32Stamped>::SharedPtr visibility_pub_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float32Stamped>::SharedPtr ratio_pub_;

  std::unique_ptr<CudaPolarVoxelOutlierFilter> cuda_polar_voxel_outlier_filter_{};

  /** \brief Diagnostics callback for visibility validation
   * Visibility represents the percentage of voxels that pass the primary-to-secondary ratio test.
   * Statistics are efficiently collected during main filtering loop (single-pass optimization).
   * Only published when return type classification is enabled for PointXYZIRCAEDT input.
   */
  void onVisibilityChecker(diagnostic_updater::DiagnosticStatusWrapper & stat);

  /** \brief Diagnostics callback for filter ratio validation
   * Filter ratio represents the ratio of output points to input points.
   * Always published for both PointXYZ and PointXYZIRCAEDT input formats.
   */
  void onFilterRatioChecker(diagnostic_updater::DiagnosticStatusWrapper & stat);
};

}  // namespace autoware::cuda_pointcloud_preprocessor

#endif  // AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_OUTLIER_FILTER__CUDA_POLAR_VOXEL_OUTLIER_FILTER_NODE_HPP_
