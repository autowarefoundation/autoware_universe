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

#ifndef AUTOWARE__PTV3__PREPROCESS__SEMSEG_PREPROCESS_KERNEL_HPP_
#define AUTOWARE__PTV3__PREPROCESS__SEMSEG_PREPROCESS_KERNEL_HPP_

#include "autoware/ptv3/preprocess/point_type.hpp"
#include "autoware/ptv3/preprocess/preprocess_kernel.hpp"
#include "autoware/ptv3/ptv3_config.hpp"

#include <cuda_runtime_api.h>

#include <cstddef>
#include <cstdint>

namespace autoware::ptv3
{

/**
 * Device buffers requested by the semantic-segmentation module during preprocessing.
 *
 * The backbone preprocessor always produces voxelized backbone inputs. When semseg output is
 * requested, this view points at semseg-owned buffers that preserve enough source-point information
 * to reconstruct voxel predictions back into the selected source point domain.
 */
struct SemsegSourceReconstructionView
{
  float * reconstruction_features{};
  void * cropped_source_points{};
  std::int64_t * inverse_map{};
  std::size_t * num_cropped_points{};
};

class SemsegPreprocessCuda
{
public:
  explicit SemsegPreprocessCuda(const PTv3SemsegConfig & config);

  /**
   * Populate semseg-owned source reconstruction buffers from backbone preprocessing outputs.
   *
   * The caller passes the same execution stream used by the surrounding inference context. This
   * helper does not store the stream because it is a leaf CUDA wrapper, not a pipeline phase owner.
   */
  void reconstructSource(
    const PreprocessCuda & backbone_preprocessor, const void * input_data, CloudFormat input_format,
    std::size_t num_points, std::size_t num_cropped_points, SemsegSourceReconstructionView view,
    cudaStream_t stream) const;

private:
  PTv3SemsegConfig config_;
};

}  // namespace autoware::ptv3

#endif  // AUTOWARE__PTV3__PREPROCESS__SEMSEG_PREPROCESS_KERNEL_HPP_
