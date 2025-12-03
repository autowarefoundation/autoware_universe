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

#ifndef AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_FILTER__CUDA_CROP_BOX_FILTER_HPP_
#define AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_FILTER__CUDA_CROP_BOX_FILTER_HPP_

#include "autoware/cuda_pointcloud_preprocessor/common_kernels.hpp"
#include "autoware/cuda_pointcloud_preprocessor/point_types.hpp"
#include "autoware/cuda_pointcloud_preprocessor/types.hpp"
#include "autoware/cuda_utils/cuda_check_error.hpp"

#include <cuda_blackboard/cuda_pointcloud2.hpp>
#include <cuda_blackboard/cuda_unique_ptr.hpp>

#include <cuda_runtime.h>
#include <thrust/device_vector.h>
#include <thrust/execution_policy.h>
#include <thrust/fill.h>
#include <thrust/scan.h>

#include <cstdint>
#include <memory>
#include <vector>

namespace autoware::cuda_pointcloud_preprocessor
{

class CudaCropBoxFilter
{
public:
  explicit CudaCropBoxFilter(
    const CropBoxParameters crop_box_parameters, int64_t max_mem_pool_size_in_byte = 1e9,
    bool output_point_xyzircaedt = false);
  ~CudaCropBoxFilter() = default;

  std::unique_ptr<cuda_blackboard::CudaPointCloud2> filter(
    const cuda_blackboard::CudaPointCloud2::ConstSharedPtr & input_points);

private:
  template <typename T>
  T * allocateBufferFromPool(size_t num_elements);

  template <typename T>
  void returnBufferToPool(T * buffer);

  CropBoxParameters crop_box_parameters_;
  bool output_point_xyzircaedt_;

  cudaStream_t stream_{};
  cudaMemPool_t mem_pool_{};

  static constexpr int threads_per_block_ = 512;
};

}  // namespace autoware::cuda_pointcloud_preprocessor

#endif  // AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_FILTER__CUDA_CROP_BOX_FILTER_HPP_
