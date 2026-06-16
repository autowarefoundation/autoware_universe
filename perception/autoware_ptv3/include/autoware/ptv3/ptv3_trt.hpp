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

#include "autoware/ptv3/postprocess/detection3d_postprocess.hpp"
#include "autoware/ptv3/postprocess/postprocess_kernel.hpp"
#include "autoware/ptv3/preprocess/preprocess_kernel.hpp"
#include "autoware/ptv3/utils.hpp"
#include "autoware/ptv3/visibility_control.hpp"

#include <autoware/cuda_utils/cuda_unique_ptr.hpp>
#include <autoware/tensorrt_common/tensorrt_common.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <cuda_blackboard/cuda_pointcloud2.hpp>

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::ptv3
{

using autoware::cuda_utils::CudaUniquePtr;

class PTV3_PUBLIC PTv3TRT
{
public:
  explicit PTv3TRT(
    const tensorrt_common::TrtCommonConfig & backbone_trt_config,
    const std::optional<tensorrt_common::TrtCommonConfig> & seg3d_head_trt_config,
    const std::optional<tensorrt_common::TrtCommonConfig> & det3d_head_trt_config,
    const PTv3Config & config);
  virtual ~PTv3TRT();

  // cSpell:ignore probs
  bool infer(
    const std::shared_ptr<const cuda_blackboard::CudaPointCloud2> & msg_ptr,
    bool should_publish_segmented_pointcloud, bool should_publish_visualization_pointcloud,
    bool should_publish_filtered_pointcloud, bool should_detect_objects,
    std::optional<std::vector<Box3D>> & det_boxes3d,
    std::unordered_map<std::string, double> & proc_timing);

  void setPublishSegmentedPointcloud(
    std::function<void(std::unique_ptr<const cuda_blackboard::CudaPointCloud2>)> func);
  void setPublishVisualizationPointcloud(
    std::function<void(std::unique_ptr<const cuda_blackboard::CudaPointCloud2>)> func);
  void setPublishFilteredPointcloud(
    std::function<void(std::unique_ptr<const cuda_blackboard::CudaPointCloud2>)> func);

  /// CUDA stream used for pointcloud processing and inference.
  /// Pass this to CudaBlackboardSubscriber for stream-level producer/consumer ordering.
  cudaStream_t stream() const { return stream_; }

protected:
  void initPtr();
  void initBackboneTrt(const tensorrt_common::TrtCommonConfig & trt_config);
  void initSeg3dHeadTrt(const tensorrt_common::TrtCommonConfig & trt_config);
  void initDetection3DHeadTrt(const tensorrt_common::TrtCommonConfig & trt_config);
  void createPointFields();
  void allocateSegOutputMessages();
  void allocateSerializedPoolingBuffers();
  void bindSerializedPoolingAddresses();
  void precomputeSerializedPoolingMetadata();
  bool setSerializedPoolingInputShapes();
  [[nodiscard]] CloudFormat detectCloudFormat(const cuda_blackboard::CudaPointCloud2 & cloud) const;

  bool preProcess(
    const std::shared_ptr<const cuda_blackboard::CudaPointCloud2> & msg_ptr, bool should_run_seg3d);

  bool inferenceBackbone();
  bool inferenceSeg3dHead();
  bool inferenceDetection3DHead();

  bool postProcess(
    const std_msgs::msg::Header & header, bool should_publish_segmented_pointcloud,
    bool should_publish_visualization_pointcloud, bool should_publish_filtered_pointcloud);

  bool postProcessDetection3D(std::vector<Box3D> & detection_boxes);

  // The backbone is always present. The heads are loaded only when enabled.
  std::unique_ptr<autoware::tensorrt_common::TrtCommon> backbone_trt_ptr_{nullptr};
  std::unique_ptr<autoware::tensorrt_common::TrtCommon> seg3d_head_trt_ptr_{nullptr};
  std::unique_ptr<autoware::tensorrt_common::TrtCommon> detection3d_head_trt_ptr_{nullptr};
  std::unique_ptr<autoware_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_{nullptr};
  std::unique_ptr<PreprocessCuda> pre_ptr_{nullptr};
  std::unique_ptr<PostprocessCuda> post_ptr_{nullptr};
  std::unique_ptr<Detection3DPostprocess> detection3d_post_ptr_{nullptr};
  cudaStream_t stream_{nullptr};

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

  PTv3Config config_;
  std::once_flag init_cloud_;
  CloudFormat input_format_{CloudFormat::UNKNOWN};
  CloudFormat filtered_output_format_{CloudFormat::UNKNOWN};

  struct SerializedPoolingDeviceStage
  {
    CudaUniquePtr<std::int64_t[]> indices{nullptr};
    CudaUniquePtr<std::int64_t[]> indptr{nullptr};
    CudaUniquePtr<std::int64_t[]> head_indices{nullptr};
    CudaUniquePtr<std::int64_t[]> cluster{nullptr};
    CudaUniquePtr<std::int32_t[]> grid_coord{nullptr};
    CudaUniquePtr<std::int64_t[]> serialized_code{nullptr};
    CudaUniquePtr<std::int64_t[]> serialized_order{nullptr};
    CudaUniquePtr<std::int64_t[]> serialized_inverse{nullptr};
  };

  std::vector<SerializedPoolingDeviceStage> serialized_pooling_stages_d_;
  CudaUniquePtr<std::int64_t[]> serialized_pooling_num_voxels_d_{nullptr};
  std::vector<std::int64_t> serialized_pooling_num_voxels_;
  std::vector<std::int64_t> serialized_pooling_depths_;

  // Preprocess outputs
  std::int64_t num_voxels_{0};
  std::int64_t num_cropped_points_{0};        // only for partial
  std::int64_t num_source_points_{0};         // only for full
  const void * current_input_data_{nullptr};  // only for full

  CudaUniquePtr<std::uint8_t[]> compact_points_d_{nullptr};
  CudaUniquePtr<std::uint8_t[]> cropped_source_points_d_{nullptr};  // only for partial
  CudaUniquePtr<float[]> reconstructed_features_d_{nullptr};        // only for partial and full
  CudaUniquePtr<std::int64_t[]> inverse_map_d_{nullptr};            // only for partial and full
  CudaUniquePtr<std::int64_t[]> reconstructed_labels_d_{nullptr};   // only for partial and full
  CudaUniquePtr<float[]> reconstructed_probs_d_{nullptr};           // only for partial and full
  CudaUniquePtr<std::int32_t[]> grid_coord_d_{nullptr};
  CudaUniquePtr<float[]> feat_d_{nullptr};
  CudaUniquePtr<std::int64_t[]> serialized_code_d_{nullptr};

  // Backbone outputs shared with all the heads
  CudaUniquePtr<float[]> bb_point_feat_d_{nullptr};
  CudaUniquePtr<std::int32_t[]> bb_point_grid_coord_d_{nullptr};
  CudaUniquePtr<std::int64_t[]> bb_point_offset_d_{nullptr};

  // Segmentation head outputs
  CudaUniquePtr<std::int64_t[]> pred_labels_d_{nullptr};
  CudaUniquePtr<float[]> pred_probs_d_{nullptr};

  // Detection3D head outputs
  CudaUniquePtr<float[]> dense_heatmap_d_{nullptr};
  CudaUniquePtr<float[]> query_heatmap_score_d_{nullptr};
  CudaUniquePtr<std::int64_t[]> query_labels_d_{nullptr};
  CudaUniquePtr<float[]> heatmap_d_{nullptr};
  CudaUniquePtr<float[]> center_d_{nullptr};
  CudaUniquePtr<float[]> height_d_{nullptr};
  CudaUniquePtr<float[]> dim_d_{nullptr};
  CudaUniquePtr<float[]> rot_d_{nullptr};
  CudaUniquePtr<float[]> vel_d_{nullptr};
};

}  // namespace autoware::ptv3

#endif  // AUTOWARE__PTV3__PTV3_TRT_HPP_
