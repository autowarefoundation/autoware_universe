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

#ifndef AUTOWARE__PTV3__POSTPROCESS__POSTPROCESS_KERNEL_HPP_
#define AUTOWARE__PTV3__POSTPROCESS__POSTPROCESS_KERNEL_HPP_

#include "autoware/ptv3/ptv3_config.hpp"

#include <autoware/cuda_utils/cuda_check_error.hpp>
#include <autoware/cuda_utils/cuda_unique_ptr.hpp>

#include <cuda_runtime_api.h>

namespace autoware::ptv3
{

using autoware::cuda_utils::CudaUniquePtr;

class PostprocessCuda
{
public:
  explicit PostprocessCuda(const PTv3Config & config, cudaStream_t stream);

  void paintPointcloud(
    const float * input_features, const std::int64_t * pred_labels, float * output_points,
    std::size_t num_points);

  // cSpell:ignore Probs probs
  void createProbsPointcloud(
    const float * input_features, const float * pred_probs, float * output_points,
    std::size_t num_classes, std::size_t num_points);

  std::size_t createGroundSegmentedPointcloud(
    const float * input_features, const std::int64_t * pred_labels, const float * pred_probs,
    float * output_points, const int ground_label, const float ground_prob_threshold,
    std::size_t num_classes, std::size_t num_points);

private:
  PTv3Config config_;

  CudaUniquePtr<std::uint32_t[]> ground_mask_d_{nullptr};
  CudaUniquePtr<float[]> color_map_d_{nullptr};
  cudaStream_t stream_;
};

}  // namespace autoware::ptv3

#endif  // AUTOWARE__PTV3__POSTPROCESS__POSTPROCESS_KERNEL_HPP_
