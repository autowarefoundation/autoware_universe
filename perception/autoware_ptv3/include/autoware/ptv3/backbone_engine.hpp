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

#ifndef AUTOWARE__PTV3__BACKBONE_ENGINE_HPP_
#define AUTOWARE__PTV3__BACKBONE_ENGINE_HPP_

#include "autoware/ptv3/execution_context.hpp"
#include "autoware/ptv3/preprocess/preprocess_kernel.hpp"
#include "autoware/ptv3/ptv3_config.hpp"
#include "autoware/ptv3/visibility_control.hpp"

#include <autoware/cuda_utils/cuda_unique_ptr.hpp>
#include <autoware/tensorrt_common/tensorrt_common.hpp>

#include <cstdint>
#include <memory>
#include <vector>

namespace autoware::ptv3
{

struct BackboneOutputView
{
  float * point_feat{};
  std::int32_t * point_grid_coord{};
  std::int64_t * point_offset{};
};

class PTV3_PUBLIC BackboneEngine
{
public:
  BackboneEngine(
    const tensorrt_common::TrtCommonConfig & trt_config, const PTv3BackboneConfig & config,
    std::int32_t * grid_coord, float * feat, std::int64_t * serialized_code);

  [[nodiscard]] BackboneOutputView output() const;
  void bindSerializedPoolingAddresses(const std::vector<SerializedPoolingDeviceStageView> & stages);
  bool setInputShapes(
    std::int64_t num_voxels, const std::vector<std::int64_t> & serialized_pooling_num_voxels);
  bool enqueue(const PTv3ExecutionContext & context);

private:
  PTv3BackboneConfig config_;

  std::unique_ptr<autoware::tensorrt_common::TrtCommon> trt_ptr_{nullptr};
  autoware::cuda_utils::CudaUniquePtr<float[]> point_feat_d_{nullptr};
  autoware::cuda_utils::CudaUniquePtr<std::int32_t[]> point_grid_coord_d_{nullptr};
  autoware::cuda_utils::CudaUniquePtr<std::int64_t[]> point_offset_d_{nullptr};
};

}  // namespace autoware::ptv3

#endif  // AUTOWARE__PTV3__BACKBONE_ENGINE_HPP_
