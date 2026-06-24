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

#include "autoware/ptv3/backbone_engine.hpp"
#include "autoware/ptv3/backbone_preprocessor.hpp"
#include "autoware/ptv3/detection3d_module.hpp"
#include "autoware/ptv3/semseg_module.hpp"
#include "autoware/ptv3/utils.hpp"
#include "autoware/ptv3/visibility_control.hpp"

#include <autoware/tensorrt_common/tensorrt_common.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <cuda_blackboard/cuda_pointcloud2.hpp>

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::ptv3
{

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

protected:
  bool prepareInferenceShapes(bool should_run_seg3d, bool should_run_det3d);

  // The backbone is always present. The heads are loaded only when enabled.
  std::unique_ptr<BackboneEngine> backbone_engine_{nullptr};
  std::unique_ptr<BackbonePreprocessor> backbone_preprocessor_{nullptr};
  std::unique_ptr<SemsegModule> semseg_module_{nullptr};
  std::unique_ptr<Detection3DModule> detection3d_module_{nullptr};
  std::unique_ptr<autoware_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_{nullptr};
  cudaStream_t stream_{nullptr};

  PTv3Config config_;
};

}  // namespace autoware::ptv3

#endif  // AUTOWARE__PTV3__PTV3_TRT_HPP_
