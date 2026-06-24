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

#ifndef AUTOWARE__PTV3__DETECTION3D_MODULE_HPP_
#define AUTOWARE__PTV3__DETECTION3D_MODULE_HPP_

#include "autoware/ptv3/execution_context.hpp"
#include "autoware/ptv3/postprocess/detection3d_postprocess.hpp"
#include "autoware/ptv3/ptv3_config.hpp"
#include "autoware/ptv3/utils.hpp"
#include "autoware/ptv3/visibility_control.hpp"

#include <autoware/cuda_utils/cuda_unique_ptr.hpp>
#include <autoware/tensorrt_common/tensorrt_common.hpp>

#include <cstdint>
#include <memory>
#include <vector>

namespace autoware::ptv3
{

class PTV3_PUBLIC Detection3DModule
{
public:
  Detection3DModule(
    const tensorrt_common::TrtCommonConfig & trt_config, const PTv3Detection3DConfig & config,
    const float * backbone_point_feat, const std::int32_t * backbone_point_grid_coord);

  void preparePreprocess();
  void setInputShapes(std::int64_t num_voxels);
  bool enqueue(const PTv3ExecutionContext & context);
  bool postProcess(const PTv3ExecutionContext & context, std::vector<Box3D> & detection_boxes);

private:
  void initTrt(const tensorrt_common::TrtCommonConfig & trt_config);

  PTv3Detection3DConfig config_;
  const float * backbone_point_feat_;
  const std::int32_t * backbone_point_grid_coord_;

  std::unique_ptr<autoware::tensorrt_common::TrtCommon> trt_ptr_{nullptr};
  std::unique_ptr<Detection3DPostprocess> post_ptr_{nullptr};

  autoware::cuda_utils::CudaUniquePtr<float[]> dense_heatmap_d_{nullptr};
  autoware::cuda_utils::CudaUniquePtr<float[]> query_heatmap_score_d_{nullptr};
  autoware::cuda_utils::CudaUniquePtr<std::int64_t[]> query_labels_d_{nullptr};
  autoware::cuda_utils::CudaUniquePtr<float[]> heatmap_d_{nullptr};
  autoware::cuda_utils::CudaUniquePtr<float[]> center_d_{nullptr};
  autoware::cuda_utils::CudaUniquePtr<float[]> height_d_{nullptr};
  autoware::cuda_utils::CudaUniquePtr<float[]> dim_d_{nullptr};
  autoware::cuda_utils::CudaUniquePtr<float[]> rot_d_{nullptr};
  autoware::cuda_utils::CudaUniquePtr<float[]> vel_d_{nullptr};
};

}  // namespace autoware::ptv3

#endif  // AUTOWARE__PTV3__DETECTION3D_MODULE_HPP_
