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

#ifndef AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_OUTLIER_FILTER__CUDA_POLAR_VOXEL_NOISE_FILTER_NODE_HPP_
#define AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_OUTLIER_FILTER__CUDA_POLAR_VOXEL_NOISE_FILTER_NODE_HPP_

#include "autoware/cuda_pointcloud_preprocessor/cuda_outlier_filter/cuda_polar_voxel_noise_filter.hpp"

#include <cuda_blackboard/cuda_adaptation.hpp>
#include <cuda_blackboard/cuda_blackboard_publisher.hpp>
#include <cuda_blackboard/cuda_blackboard_subscriber.hpp>
#include <cuda_blackboard/cuda_pointcloud2.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace autoware::cuda_pointcloud_preprocessor
{
class CudaPolarVoxelNoiseFilterNode : public rclcpp::Node
{
public:
  explicit CudaPolarVoxelNoiseFilterNode(const rclcpp::NodeOptions & node_options);

protected:
  /** \brief Parameter service callback result : needed to be hold */
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  /** \brief Parameter service callback */
  rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter> & p);

  /** \brief main callback for pointcloud processing
   */
  void pointcloud_callback(const cuda_blackboard::CudaPointCloud2::ConstSharedPtr msg);

  // Utility functions to validate inputs
  void validate_filter_inputs(const cuda_blackboard::CudaPointCloud2::ConstSharedPtr & input_cloud);
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

private:
  CudaPolarVoxelNoiseFilterParameters filter_params_;
  std::vector<int> primary_return_types_;  // Return types considered as primary returns
  std::mutex param_mutex_;

  // CUDA sub
  std::shared_ptr<cuda_blackboard::CudaBlackboardSubscriber<cuda_blackboard::CudaPointCloud2>>
    pointcloud_sub_{};

  // CUDA pub
  std::unique_ptr<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>
    filtered_cloud_pub_{};
  std::unique_ptr<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>
    noise_cloud_pub_{};

  std::unique_ptr<CudaPolarVoxelNoiseFilter> cuda_polar_voxel_noise_filter_{};
};

}  // namespace autoware::cuda_pointcloud_preprocessor

#endif  // AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_OUTLIER_FILTER__CUDA_POLAR_VOXEL_NOISE_FILTER_NODE_HPP_
