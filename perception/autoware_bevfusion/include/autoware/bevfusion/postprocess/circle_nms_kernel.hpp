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

#ifndef AUTOWARE__BEVFUSION__POSTPROCESS__CIRCLE_NMS_KERNEL_HPP_
#define AUTOWARE__BEVFUSION__POSTPROCESS__CIRCLE_NMS_KERNEL_HPP_

#include "autoware/bevfusion/bevfusion_config.hpp"
#include "autoware/bevfusion/utils.hpp"

#include <autoware/cuda_utils/cuda_unique_ptr.hpp>

#include <cuda.h>
#include <cuda_runtime_api.h>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace autoware::bevfusion
{
using autoware::cuda_utils::CudaUniquePtr;

class CircleNMS
{
public:
  explicit CircleNMS(const BEVFusionConfig & config, cudaStream_t stream);
  // Non-maximum suppression (NMS) uses the distance on the xy plane instead of
  // intersection over union (IoU) to suppress overlapped objects.
  // Note: the mask uses std::uint8_t (not bool) because std::vector<bool> is bit-packed and
  // cannot be copied to the device as a contiguous byte array.
  std::vector<std::uint8_t> circleNMS(Box3D * descending_sorted_bboxes, cudaStream_t stream);

private:
  BEVFusionConfig config_;
  cudaStream_t stream_;

  // Memory for NMS
  std::vector<std::uint64_t> final_keep_mask_h_;
  CudaUniquePtr<std::uint64_t[]> final_keep_mask_d_{nullptr};
  std::size_t col_blocks_ = 0;
};
}  // namespace autoware::bevfusion

#endif  // AUTOWARE__BEVFUSION__POSTPROCESS__CIRCLE_NMS_KERNEL_HPP_
