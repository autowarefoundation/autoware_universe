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

#ifndef AUTOWARE__PTV3__SEMSEG_MODULE_HPP_
#define AUTOWARE__PTV3__SEMSEG_MODULE_HPP_

#include "autoware/ptv3/execution_context.hpp"
#include "autoware/ptv3/postprocess/postprocess_kernel.hpp"
#include "autoware/ptv3/preprocess/preprocess_kernel.hpp"
#include "autoware/ptv3/preprocess/semseg_preprocess_kernel.hpp"
#include "autoware/ptv3/ptv3_config.hpp"
#include "autoware/ptv3/utils.hpp"
#include "autoware/ptv3/visibility_control.hpp"

#include <autoware/cuda_utils/cuda_unique_ptr.hpp>
#include <autoware/tensorrt_common/tensorrt_common.hpp>
#include <cuda_blackboard/cuda_pointcloud2.hpp>

#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/header.hpp>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace autoware::ptv3
{

class PTV3_PUBLIC SemsegModule
{
public:
  SemsegModule(
    const tensorrt_common::TrtCommonConfig & trt_config, const PTv3SemsegConfig & config,
    const float * backbone_point_feat, const float * backbone_input_features,
    const void * compact_points);

  [[nodiscard]] bool shouldRun(
    bool should_publish_segmented_pointcloud, bool should_publish_visualization_pointcloud,
    bool should_publish_filtered_pointcloud) const;

  void setPublishSegmentedPointcloud(
    std::function<void(std::unique_ptr<const cuda_blackboard::CudaPointCloud2>)> func);
  void setPublishVisualizationPointcloud(
    std::function<void(std::unique_ptr<const cuda_blackboard::CudaPointCloud2>)> func);
  void setPublishFilteredPointcloud(
    std::function<void(std::unique_ptr<const cuda_blackboard::CudaPointCloud2>)> func);

  void preparePreprocess(
    const cuda_blackboard::CudaPointCloud2 & msg, const PTv3ExecutionContext & context,
    CloudFormat input_format, bool should_run);
  void preprocessSourceReconstruction(
    const cuda_blackboard::CudaPointCloud2 & msg, const PTv3ExecutionContext & context,
    const PreprocessCuda & backbone_preprocessor, std::size_t num_cropped_points, bool should_run);
  void setInputShape(std::int64_t num_voxels);
  bool enqueue(const PTv3ExecutionContext & context);
  bool postProcess(
    const std_msgs::msg::Header & header, const PTv3ExecutionContext & context,
    const PreprocessCuda & preprocessor, CloudFormat input_format,
    bool should_publish_segmented_pointcloud, bool should_publish_visualization_pointcloud,
    bool should_publish_filtered_pointcloud, std::int64_t num_voxels);

private:
  void initTrt(const tensorrt_common::TrtCommonConfig & trt_config);
  void createPointFields();
  void prepareFilteredPointcloudFormat(CloudFormat input_format);
  void allocateOutputMessages();
  [[nodiscard]] std::int64_t outputCapacity() const;

  PTv3SemsegConfig config_;
  const float * backbone_point_feat_;
  const float * backbone_input_features_;
  const void * compact_points_;

  std::unique_ptr<autoware::tensorrt_common::TrtCommon> trt_ptr_{nullptr};
  std::unique_ptr<SemsegPreprocessCuda> preprocess_ptr_{nullptr};
  std::unique_ptr<PostprocessCuda> post_ptr_{nullptr};

  std::function<void(std::unique_ptr<const cuda_blackboard::CudaPointCloud2>)>
    publish_segmented_pointcloud_{nullptr};
  std::function<void(std::unique_ptr<const cuda_blackboard::CudaPointCloud2>)>
    publish_visualization_pointcloud_{nullptr};
  std::function<void(std::unique_ptr<const cuda_blackboard::CudaPointCloud2>)>
    publish_filtered_pointcloud_{nullptr};

  std::vector<sensor_msgs::msg::PointField> segmented_pointcloud_fields_;
  std::vector<sensor_msgs::msg::PointField> visualization_pointcloud_fields_;
  std::vector<sensor_msgs::msg::PointField> filtered_pointcloud_fields_;

  std::unique_ptr<cuda_blackboard::CudaPointCloud2> segmented_points_msg_ptr_{nullptr};
  std::unique_ptr<cuda_blackboard::CudaPointCloud2> visualization_points_msg_ptr_{nullptr};
  std::unique_ptr<cuda_blackboard::CudaPointCloud2> filtered_points_msg_ptr_{nullptr};

  std::size_t num_source_points_{0};
  std::size_t num_cropped_points_{0};
  const void * current_input_data_{nullptr};
  std::once_flag init_filtered_cloud_;
  CloudFormat input_format_{CloudFormat::UNKNOWN};
  CloudFormat filtered_output_format_{CloudFormat::UNKNOWN};

  autoware::cuda_utils::CudaUniquePtr<std::uint8_t[]> cropped_source_points_d_{nullptr};
  autoware::cuda_utils::CudaUniquePtr<float[]> reconstructed_features_d_{nullptr};
  autoware::cuda_utils::CudaUniquePtr<std::int64_t[]> inverse_map_d_{nullptr};
  autoware::cuda_utils::CudaUniquePtr<std::int64_t[]> reconstructed_labels_d_{nullptr};
  autoware::cuda_utils::CudaUniquePtr<float[]> reconstructed_probs_d_{nullptr};
  autoware::cuda_utils::CudaUniquePtr<std::int64_t[]> pred_labels_d_{nullptr};
  autoware::cuda_utils::CudaUniquePtr<float[]> pred_probs_d_{nullptr};
};

}  // namespace autoware::ptv3

#endif  // AUTOWARE__PTV3__SEMSEG_MODULE_HPP_
