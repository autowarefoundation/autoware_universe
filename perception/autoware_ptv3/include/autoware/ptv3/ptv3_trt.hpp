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

#ifndef AUTOWARE__PTV3__PTV3_TRT_HPP_
#define AUTOWARE__PTV3__PTV3_TRT_HPP_

#include "autoware/ptv3/postprocess/postprocess_kernel.hpp"
#include "autoware/ptv3/preprocess/preprocess_kernel.hpp"
#include "autoware/ptv3/visibility_control.hpp"

#include <autoware/cuda_utils/cuda_unique_ptr.hpp>
#include <autoware/tensorrt_common/tensorrt_common.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <cuda_blackboard/cuda_pointcloud2.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::ptv3
{

using autoware::cuda_utils::CudaUniquePtr;

class PTV3_PUBLIC PTv3TRT
{
public:
  explicit PTv3TRT(const tensorrt_common::TrtCommonConfig & trt_config, const PTv3Config & config);
  virtual ~PTv3TRT();

  bool fake_segment(sensor_msgs::msg::PointCloud2 & out_msg);

  // cSpell:ignore probs
  bool segment(
    const std::shared_ptr<const cuda_blackboard::CudaPointCloud2> & msg_ptr,
    bool should_publish_segmented_pointcloud, bool should_publish_ground_segmented_pointcloud,
    bool should_publish_probs_pointcloud, std::unordered_map<std::string, double> & proc_timing);

  void setPublishSegmentedPointcloud(
    std::function<void(std::unique_ptr<const cuda_blackboard::CudaPointCloud2>)> func);
  void setPublishGroundSegmentedPointcloud(
    std::function<void(std::unique_ptr<const cuda_blackboard::CudaPointCloud2>)> func);
  // cSpell:ignore Probs
  void setPublishProbsPointcloud(
    std::function<void(std::unique_ptr<const cuda_blackboard::CudaPointCloud2>)> func);

protected:
  void initPtr();
  void initTrt(const tensorrt_common::TrtCommonConfig & trt_config);
  void createPointFields();
  void allocateMessages();

  bool preProcess(const std::shared_ptr<const cuda_blackboard::CudaPointCloud2> & msg_ptr);

  bool inference();

  bool postProcess(
    const std_msgs::msg::Header & header, bool should_publish_segmented_pointcloud,
    bool should_publish_ground_segmented_pointcloud, bool should_publish_probs_pointcloud);

  std::unique_ptr<autoware::tensorrt_common::TrtCommon> network_trt_ptr_{nullptr};
  std::unique_ptr<autoware_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_{nullptr};
  std::unique_ptr<PreprocessCuda> pre_ptr_{nullptr};
  std::unique_ptr<PostprocessCuda> post_ptr_{nullptr};
  cudaStream_t stream_{nullptr};

  std::function<void(std::unique_ptr<const cuda_blackboard::CudaPointCloud2>)>
    publish_segmented_pointcloud_{nullptr};
  std::function<void(std::unique_ptr<const cuda_blackboard::CudaPointCloud2>)>
    publish_ground_segmented_pointcloud_{nullptr};
  std::function<void(std::unique_ptr<const cuda_blackboard::CudaPointCloud2>)>
    publish_probs_pointcloud_{nullptr};

  std::vector<sensor_msgs::msg::PointField> segmented_pointcloud_fields_;
  std::vector<sensor_msgs::msg::PointField> ground_segmented_pointcloud_fields_;
  std::vector<sensor_msgs::msg::PointField> probs_pointcloud_fields_;

  std::unique_ptr<cuda_blackboard::CudaPointCloud2> segmented_points_msg_ptr_{nullptr};
  std::unique_ptr<cuda_blackboard::CudaPointCloud2> ground_segmented_points_msg_ptr_{nullptr};
  std::unique_ptr<cuda_blackboard::CudaPointCloud2> probs_points_msg_ptr_{nullptr};

  PTv3Config config_;

  // pre-process inputs
  std::int64_t num_voxels_{0};

  CudaUniquePtr<std::int64_t[]> grid_coord_d_{nullptr};
  CudaUniquePtr<float[]> feat_d_{nullptr};
  CudaUniquePtr<std::int64_t[]> serialized_code_d_{nullptr};

  CudaUniquePtr<std::int64_t[]> pred_labels_d_{nullptr};
  CudaUniquePtr<float[]> pred_probs_d_{nullptr};
};

}  // namespace autoware::ptv3

#endif  // AUTOWARE__PTV3__PTV3_TRT_HPP_
