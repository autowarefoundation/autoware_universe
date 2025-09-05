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

#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace autoware::cuda_pointcloud_preprocessor
{
class CudaPolarVoxelOutlierFilterNode : public rclcpp::Node
{
  static constexpr double diagnostics_update_period_sec = 0.1;

public:
  explicit CudaPolarVoxelOutlierFilterNode(const rclcpp::NodeOptions & node_options);

protected:
  /** \brief Parameter service callback result : needed to be hold */
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  /** \brief Parameter service callback */
  rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter> & p);

  /** \brief main callback for pointcloud processing
   */
  void pointcloud_callback(const cuda_blackboard::CudaPointCloud2::ConstSharedPtr msg);

  /** \brief Diagnostics callback for visibility validation
   * Visibility represents the percentage of voxels that pass the primary-to-secondary ratio test.
   * Statistics are efficiently collected during main filtering loop (single-pass optimization).
   * Only published when return type classification is enabled for PointXYZIRCAEDT input.
   */
  void on_visibility_check(diagnostic_updater::DiagnosticStatusWrapper & stat);

  /** \brief Diagnostics callback for filter ratio validation
   * Filter ratio represents the ratio of output points to input points.
   * Always published for both PointXYZ and PointXYZIRCAEDT input formats.
   */
  void on_filter_ratio_check(diagnostic_updater::DiagnosticStatusWrapper & stat);

  // Utility functions to validate inputs
  void validate_filter_inputs(const cuda_blackboard::CudaPointCloud2::ConstSharedPtr & input_cloud);
  void voidvalidate_return_type_field(
    const cuda_blackboard::CudaPointCloud2::ConstSharedPtr & input_cloud);
  void validate_intensity_field(
    const cuda_blackboard::CudaPointCloud2::ConstSharedPtr & input_cloud);
  void validate_return_type_field(
    const cuda_blackboard::CudaPointCloud2::ConstSharedPtr & input_cloud);
  bool has_field(
    const cuda_blackboard::CudaPointCloud2::ConstSharedPtr & input, const std::string & field_name);

  // Parameter validation helpers (static, private)
  static bool validate_positive_double(const rclcpp::Parameter & param, std::string & reason);
  static bool validate_non_negative_double(const rclcpp::Parameter & param, std::string & reason);
  static bool validate_positive_int(const rclcpp::Parameter & param, std::string & reason);
  static bool validate_non_negative_int(const rclcpp::Parameter & param, std::string & reason);
  static bool validate_intensity_threshold(const rclcpp::Parameter & param, std::string & reason);
  static bool validate_primary_return_types(const rclcpp::Parameter & param, std::string & reason);
  static bool validate_normalized(const rclcpp::Parameter & param, std::string & reason);

private:
  CudaPolarVoxelOutlierFilterParameters filter_params_;
  std::vector<int> primary_return_types_;  // Return types considered as primary returns
  std::mutex param_mutex_;

  // Diagnostics members
  std::optional<double> visibility_;    // Current visibility value
  std::optional<double> filter_ratio_;  // Current filter ratio
  diagnostic_updater::Updater updater_;

  // CUDA sub
  std::shared_ptr<cuda_blackboard::CudaBlackboardSubscriber<cuda_blackboard::CudaPointCloud2>>
    pointcloud_sub_{};

  // CUDA pub
  std::unique_ptr<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>
    filtered_cloud_pub_{};
  std::unique_ptr<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>
    noise_cloud_pub_{};
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float32Stamped>::SharedPtr visibility_pub_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float32Stamped>::SharedPtr ratio_pub_;

  std::unique_ptr<CudaPolarVoxelOutlierFilter> cuda_polar_voxel_outlier_filter_{};
};

}  // namespace autoware::cuda_pointcloud_preprocessor

#endif  // AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_OUTLIER_FILTER__CUDA_POLAR_VOXEL_OUTLIER_FILTER_NODE_HPP_
