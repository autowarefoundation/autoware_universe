// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_POINTCLOUD_PREPROCESSOR_HPP_
#define AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_POINTCLOUD_PREPROCESSOR_HPP_

#include "autoware/cuda_pointcloud_preprocessor/point_types.hpp"
#include "autoware/cuda_pointcloud_preprocessor/types.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <autoware/cuda_utils/thrust_utils.hpp>
#include <cuda_blackboard/cuda_pointcloud2.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <thrust/device_vector.h>

#include <cstdint>
#include <deque>
#include <memory>
#include <vector>

namespace autoware::cuda_pointcloud_preprocessor
{

struct ProcessingStats
{
  int mismatch_count{0};
  int num_crop_box_passed_points{0};
  int num_nan_points{0};
};

class CudaPointcloudPreprocessor
{
public:
  enum class UndistortionType { Invalid, Undistortion2D, Undistortion3D };

  CudaPointcloudPreprocessor();

  void setCropBoxParameters(const std::vector<CropBoxParameters> & crop_box_parameters);
  void setRingOutlierFilterParameters(const RingOutlierFilterParameters & ring_outlier_parameters);
  void setUndistortionType(const UndistortionType & undistortion_type);

  void preallocateOutput();
  [[nodiscard]] ProcessingStats getProcessingStats() const { return stats_; }

  std::unique_ptr<cuda_blackboard::CudaPointCloud2> process(
    const sensor_msgs::msg::PointCloud2 & input_pointcloud_msg,
    const geometry_msgs::msg::TransformStamped & transform_msg,
    const std::deque<geometry_msgs::msg::TwistWithCovarianceStamped> & twist_queue,
    const std::deque<geometry_msgs::msg::Vector3Stamped> & angular_velocity_queue,
    const std::uint32_t first_point_rel_stamp_nsec);

private:
  static cudaStream_t initialize_stream();

  void organizePointcloud();

  CropBoxParameters self_crop_box_parameters_{};
  CropBoxParameters mirror_crop_box_parameters_{};
  RingOutlierFilterParameters ring_outlier_parameters_{};
  UndistortionType undistortion_type_{UndistortionType::Invalid};

  int num_rings_{};
  int max_points_per_ring_{};
  size_t num_raw_points_{};
  size_t num_organized_points_{};

  std::vector<sensor_msgs::msg::PointField> point_fields_;
  std::unique_ptr<cuda_blackboard::CudaPointCloud2> output_pointcloud_ptr_;

  cudaStream_t stream_{};
  int max_blocks_per_grid_{};
  const int threads_per_block_{256};
  cudaMemPool_t device_memory_pool_{};

  ProcessingStats stats_;

  // Organizing buffers
  thrust::device_vector<InputPointType> device_input_points_;
  thrust::device_vector<InputPointType> device_organized_points_;
  thrust::device_vector<std::int32_t> device_ring_index_;
  thrust::device_vector<std::uint32_t> device_indexes_tensor_;
  thrust::device_vector<std::uint32_t> device_sorted_indexes_tensor_;
  thrust::device_vector<std::int32_t> device_segment_offsets_;
  thrust::device_vector<std::int32_t> device_max_ring_;
  thrust::device_vector<std::int32_t> device_max_points_per_ring_;

  thrust::device_vector<std::uint8_t> device_sort_workspace_;
  std::size_t sort_workspace_bytes_{0};

  // Pointcloud preprocessing buffers
  thrust::device_vector<InputPointType> device_transformed_points_;
  thrust::device_vector<OutputPointType> device_output_points_;
  thrust::device_vector<std::uint32_t> device_crop_mask_;
  thrust::device_vector<std::uint8_t> device_nan_mask_;
  thrust::device_vector<std::uint8_t> device_mismatch_mask_;
  thrust::device_vector<std::uint32_t> device_ring_outlier_mask_;
  thrust::device_vector<std::uint32_t> device_indices_;
  thrust::device_vector<TwistStruct2D> device_twist_2d_structs_;
  thrust::device_vector<TwistStruct3D> device_twist_3d_structs_;
  thrust::device_vector<CropBoxParameters> device_crop_box_structs_;
};

}  // namespace autoware::cuda_pointcloud_preprocessor

#endif  // AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_POINTCLOUD_PREPROCESSOR_HPP_
