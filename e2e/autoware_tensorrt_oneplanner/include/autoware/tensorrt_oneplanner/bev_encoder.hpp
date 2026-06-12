// Copyright 2026 TIER IV, Inc.
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

#ifndef AUTOWARE__TENSORRT_ONEPLANNER__BEV_ENCODER_HPP_
#define AUTOWARE__TENSORRT_ONEPLANNER__BEV_ENCODER_HPP_

#include <autoware/bevfusion/bevfusion_config.hpp>
#include <autoware/bevfusion/preprocess/pointcloud_densification.hpp>
#include <autoware/bevfusion/preprocess/preprocess_kernel.hpp>
#include <autoware/bevfusion/preprocess/voxel_generator.hpp>
#include <autoware/cuda_utils/cuda_unique_ptr.hpp>
#include <autoware/tensorrt_common/tensorrt_common.hpp>
#include <cuda_blackboard/cuda_pointcloud2.hpp>

#include <tf2_ros/buffer.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace autoware::tensorrt_oneplanner
{

using autoware::cuda_utils::CudaUniquePtr;

struct BevEncoderParams
{
  std::string onnx_path;
  std::string engine_path;
  std::string trt_precision;
  std::string plugins_path;
  std::string densification_world_frame_id;
  int64_t densification_num_past_frames;
  int64_t cloud_capacity;
  int64_t max_points_per_voxel;
  std::vector<int64_t> voxels_num;       // [min, opt, max]
  std::vector<float> point_cloud_range;  // [x_min, y_min, z_min, x_max, y_max, z_max]
  std::vector<float> voxel_size;         // [x, y, z]
  int64_t out_size_factor;
  int64_t bev_feature_channels;
};

/**
 * @class BevEncoder
 * @brief LiDAR-only BEVFusion encoder producing a BEV feature map for the OnePlanner head.
 *
 * Reuses the autoware_bevfusion preprocessing stack (densification, voxelization CUDA
 * kernels) and runs a TensorRT engine whose graph is the BEVFusion lidar branch up to and
 * including the SECOND-FPN neck. The engine consumes the same inputs as the
 * autoware_bevfusion lidar-only model (voxels / num_points_per_voxel / coors) and outputs
 * a dense BEV feature map instead of detections.
 */
class BevEncoder
{
public:
  BevEncoder(const BevEncoderParams & params, cudaStream_t stream);

  /**
   * @brief Voxelize the pointcloud and run the encoder engine.
   *
   * The resulting feature map stays on the device; access it via bev_feature_d().
   *
   * @return true on success
   */
  bool encode(
    const std::shared_ptr<const cuda_blackboard::CudaPointCloud2> & pointcloud_msg_ptr,
    const tf2_ros::Buffer & tf_buffer);

  const float * bev_feature_d() const { return bev_feature_d_.get(); }
  size_t bev_feature_num_elements() const { return bev_feature_size_; }
  int64_t bev_size() const { return bev_size_; }
  int64_t bev_channels() const { return params_.bev_feature_channels; }

private:
  void init_config();
  void init_trt();

  BevEncoderParams params_;
  cudaStream_t stream_;

  std::unique_ptr<autoware::bevfusion::BEVFusionConfig> config_;
  std::unique_ptr<autoware::bevfusion::VoxelGenerator> voxel_generator_;
  std::unique_ptr<autoware::bevfusion::PreprocessCuda> preprocess_;
  std::unique_ptr<autoware::tensorrt_common::TrtCommon> network_trt_ptr_;

  int64_t bev_size_{0};
  size_t bev_feature_size_{0};

  CudaUniquePtr<float[]> points_d_;
  CudaUniquePtr<float[]> voxel_features_d_;
  CudaUniquePtr<std::int32_t[]> voxel_coords_d_;
  CudaUniquePtr<std::int32_t[]> num_points_per_voxel_d_;
  CudaUniquePtr<float[]> bev_feature_d_;
};

}  // namespace autoware::tensorrt_oneplanner

#endif  // AUTOWARE__TENSORRT_ONEPLANNER__BEV_ENCODER_HPP_
