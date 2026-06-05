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

#include "autoware/ptv3/postprocess/det3d_trans_head_postprocess.hpp"
#include "autoware/ptv3/postprocess/seg3d_postprocess.hpp"
#include "autoware/ptv3/preprocess/backbone_preprocess.hpp"
#include "autoware/ptv3/utils.hpp"
#include "autoware/ptv3/visibility_control.hpp"

#include <autoware/cuda_utils/cuda_unique_ptr.hpp>
#include <autoware/tensorrt_common/tensorrt_common.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <cuda_blackboard/cuda_pointcloud2.hpp>

#include <cuda_fp16.h>

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

/**
 * @brief Owns the PTv3 TensorRT engines and CUDA buffers used by the node.
 *
 * The backbone is always loaded. Segmentation and detection heads are loaded only when enabled.
 */
class PTV3_PUBLIC PTv3TRT
{
public:
  /**
   * @brief Load the backbone engine and any enabled heads.
   *
   * @param backbone_trt_config TensorRT configuration for the backbone engine.
   * @param seg3d_head_trt_config TensorRT configuration for the segmentation head if enabled.
   * @param det3d_head_trt_config TensorRT configuration for the detection head if enabled.
   * @param config Runtime configuration shared by preprocessing, inference, and postprocessing.
   */
  explicit PTv3TRT(
    const tensorrt_common::TrtCommonConfig & backbone_trt_config,
    const std::optional<tensorrt_common::TrtCommonConfig> & seg3d_head_trt_config,
    const std::optional<tensorrt_common::TrtCommonConfig> & det3d_head_trt_config,
    const PTv3Config & config);

  /** @brief Wait for CUDA work and release the stream. */
  virtual ~PTv3TRT();

  /**
   * @brief Run the segmentation head for the requested point cloud outputs.
   *
   * @param msg_ptr Input GPU point cloud message.
   * @param should_publish_segmented_pointcloud Publish the labeled point cloud.
   * @param should_publish_visualization_pointcloud Publish the RGB visualization cloud.
   * @param should_publish_filtered_pointcloud Publish the probability filtered cloud.
   * @param proc_timing Per-stage timings in milliseconds.
   * @return true when segmentation finished and requested outputs were published.
   */
  bool segment(
    const std::shared_ptr<const cuda_blackboard::CudaPointCloud2> & msg_ptr,
    bool should_publish_segmented_pointcloud, bool should_publish_visualization_pointcloud,
    bool should_publish_filtered_pointcloud, std::unordered_map<std::string, double> & proc_timing);

  /**
   * @brief Run the detection head and copy decoded boxes to host memory.
   *
   * @param msg_ptr Input GPU point cloud message.
   * @param det_boxes3d Output boxes after score filtering and GPU sorting.
   * @param proc_timing Per-stage timings in milliseconds.
   * @return true when detection finished.
   */
  bool detect(
    const std::shared_ptr<const cuda_blackboard::CudaPointCloud2> & msg_ptr,
    std::vector<Box3D> & det_boxes3d, std::unordered_map<std::string, double> & proc_timing);

  /**
   * @brief Run the shared backbone, then only the requested heads.
   *
   * @param msg_ptr Input GPU point cloud message.
   * @param should_publish_segmented_pointcloud Publish the labeled point cloud.
   * @param should_publish_visualization_pointcloud Publish the RGB visualization cloud.
   * @param should_publish_filtered_pointcloud Publish the probability filtered cloud.
   * @param should_detect_objects Run the detection head and fill det_boxes3d on success.
   * @param det_boxes3d Set only when detection was requested and completed.
   * @param proc_timing Per-stage timings in milliseconds.
   * @return true when at least one requested head finished.
   */
  bool infer(
    const std::shared_ptr<const cuda_blackboard::CudaPointCloud2> & msg_ptr,
    bool should_publish_segmented_pointcloud, bool should_publish_visualization_pointcloud,
    bool should_publish_filtered_pointcloud, bool should_detect_objects,
    std::optional<std::vector<Box3D>> & det_boxes3d,
    std::unordered_map<std::string, double> & proc_timing);

  /**
   * @brief Set the callback used for segmented point cloud publication.
   *
   * @param func Callback that takes ownership of a CudaPointCloud2.
   */
  void set_publish_segmented_pointcloud(
    std::function<void(std::unique_ptr<const cuda_blackboard::CudaPointCloud2>)> func);

  /**
   * @brief Set the callback used for visualization point cloud publication.
   *
   * @param func Callback that takes ownership of a CudaPointCloud2.
   */
  void set_publish_visualization_pointcloud(
    std::function<void(std::unique_ptr<const cuda_blackboard::CudaPointCloud2>)> func);

  /**
   * @brief Set the callback used for filtered point cloud publication.
   *
   * @param func Callback that takes ownership of a CudaPointCloud2.
   */
  void set_publish_filtered_pointcloud(
    std::function<void(std::unique_ptr<const cuda_blackboard::CudaPointCloud2>)> func);

protected:
  void init_ptr();
  void init_backbone_trt(const tensorrt_common::TrtCommonConfig & trt_config);
  void init_seg3d_head_trt(const tensorrt_common::TrtCommonConfig & trt_config);
  void init_det3d_head_trt(const tensorrt_common::TrtCommonConfig & trt_config);
  void create_point_fields();
  void allocate_messages();
  [[nodiscard]] static CloudFormat detect_cloud_format(
    const cuda_blackboard::CudaPointCloud2 & cloud);

  bool pre_process(const std::shared_ptr<const cuda_blackboard::CudaPointCloud2> & msg_ptr);
  bool infer_backbone();
  bool infer_seg3d_head();
  bool infer_det3d_head();

  bool post_process_seg3d(
    const std_msgs::msg::Header & header, bool should_publish_segmented_pointcloud,
    bool should_publish_visualization_pointcloud, bool should_publish_filtered_pointcloud);

  bool post_process_det3d(std::vector<Box3D> & det_boxes3d);

  static nvinfer1::DataType bind_float_output(
    autoware::tensorrt_common::TrtCommon * trt_ptr, const char * tensor_name, float * fp32_buffer,
    CudaUniquePtr<__half[]> & fp16_buffer, std::size_t num_elements);

  // The backbone is always present. Heads are loaded only when enabled.
  std::unique_ptr<autoware::tensorrt_common::TrtCommon> backbone_trt_ptr_{nullptr};
  std::unique_ptr<autoware::tensorrt_common::TrtCommon> seg3d_head_trt_ptr_{nullptr};
  std::unique_ptr<autoware::tensorrt_common::TrtCommon> det3d_head_trt_ptr_{nullptr};

  std::unique_ptr<autoware_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_{nullptr};
  std::unique_ptr<BackbonePreprocess> pre_ptr_{nullptr};
  std::unique_ptr<Seg3dPostprocess> post_ptr_{nullptr};
  std::unique_ptr<Det3dTransHeadPostprocess> trans_head_post_ptr_{nullptr};
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

  // Preprocess state.
  std::int64_t num_voxels_{0};
  std::int64_t num_cropped_points_{0};
  std::int64_t num_source_points_{0};
  const void * current_input_data_{nullptr};

  // Backbone inputs from preprocessing.
  CudaUniquePtr<std::uint8_t[]> compact_points_d_{nullptr};
  CudaUniquePtr<std::uint8_t[]> cropped_source_points_d_{nullptr};
  CudaUniquePtr<float[]> reconstructed_features_d_{nullptr};
  CudaUniquePtr<std::int64_t[]> inverse_map_d_{nullptr};
  CudaUniquePtr<std::int64_t[]> reconstructed_labels_d_{nullptr};
  CudaUniquePtr<float[]> reconstructed_probs_d_{nullptr};
  CudaUniquePtr<std::int32_t[]> grid_coord_d_{nullptr};
  CudaUniquePtr<float[]> feat_d_{nullptr};
  CudaUniquePtr<std::int64_t[]> serialized_code_d_{nullptr};

  // Backbone outputs shared by enabled heads.
  CudaUniquePtr<float[]> bb_point_feat_d_{nullptr};
  CudaUniquePtr<std::int32_t[]> bb_point_grid_coord_d_{nullptr};
  CudaUniquePtr<std::int64_t[]> bb_point_offset_d_{nullptr};

  // Segmentation head outputs.
  CudaUniquePtr<std::int64_t[]> pred_labels_d_{nullptr};
  CudaUniquePtr<float[]> pred_probs_d_{nullptr};
  CudaUniquePtr<__half[]> pred_probs_fp16_d_{nullptr};
  nvinfer1::DataType pred_probs_dtype_{nvinfer1::DataType::kFLOAT};

  // Full and partial reconstruction buffers for FP16 probabilities.
  CudaUniquePtr<__half[]> reconstructed_probs_fp16_d_{nullptr};

  // Detection head outputs.
  CudaUniquePtr<float[]> heatmap_d_{nullptr};
  CudaUniquePtr<__half[]> heatmap_fp16_d_{nullptr};
  nvinfer1::DataType heatmap_dtype_{nvinfer1::DataType::kFLOAT};
  CudaUniquePtr<float[]> height_d_{nullptr};
  CudaUniquePtr<__half[]> height_fp16_d_{nullptr};
  nvinfer1::DataType height_dtype_{nvinfer1::DataType::kFLOAT};
  CudaUniquePtr<float[]> dim_d_{nullptr};
  CudaUniquePtr<__half[]> dim_fp16_d_{nullptr};
  nvinfer1::DataType dim_dtype_{nvinfer1::DataType::kFLOAT};
  CudaUniquePtr<float[]> rot_d_{nullptr};
  CudaUniquePtr<__half[]> rot_fp16_d_{nullptr};
  nvinfer1::DataType rot_dtype_{nvinfer1::DataType::kFLOAT};
  CudaUniquePtr<float[]> vel_d_{nullptr};
  CudaUniquePtr<__half[]> vel_fp16_d_{nullptr};
  nvinfer1::DataType vel_dtype_{nvinfer1::DataType::kFLOAT};

  // TransHead outputs.
  CudaUniquePtr<float[]> dense_heatmap_d_{nullptr};
  CudaUniquePtr<__half[]> dense_heatmap_fp16_d_{nullptr};
  nvinfer1::DataType dense_heatmap_dtype_{nvinfer1::DataType::kFLOAT};
  CudaUniquePtr<float[]> query_heatmap_score_d_{nullptr};
  CudaUniquePtr<__half[]> query_heatmap_score_fp16_d_{nullptr};
  nvinfer1::DataType query_heatmap_score_dtype_{nvinfer1::DataType::kFLOAT};
  CudaUniquePtr<std::int32_t[]> query_labels_i32_d_{nullptr};
  CudaUniquePtr<std::int64_t[]> query_labels_i64_d_{nullptr};
  nvinfer1::DataType query_labels_dtype_{nvinfer1::DataType::kINT64};
  CudaUniquePtr<float[]> center_d_{nullptr};
  CudaUniquePtr<__half[]> center_fp16_d_{nullptr};
  nvinfer1::DataType center_dtype_{nvinfer1::DataType::kFLOAT};
};

}  // namespace autoware::ptv3

#endif  // AUTOWARE__PTV3__PTV3_TRT_HPP_
