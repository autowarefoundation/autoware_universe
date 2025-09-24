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

#ifndef AUTOWARE__SIMPL_PREDICTION__TRT_SIMPL_HPP_
#define AUTOWARE__SIMPL_PREDICTION__TRT_SIMPL_HPP_

#include "autoware/simpl_prediction/archetype/result.hpp"
#include "autoware/simpl_prediction/archetype/tensor.hpp"

#include <autoware/cuda_utils/cuda_unique_ptr.hpp>
#include <autoware/tensorrt_common/tensorrt_common.hpp>
#include <autoware/tensorrt_common/utils.hpp>

#include <memory>
#include <tuple>
#include <vector>

namespace autoware::simpl_prediction
{
/**
 * @brief A class of SIMPL for inference with TensorRT.
 */
class TrtSimpl
{
public:
  using score_type = std::vector<float>;
  using trajectory_type = std::vector<float>;
  using output_type = std::tuple<score_type, trajectory_type>;

  /**
   * @brief Construct a new TrtSimpl object.
   *
   * @param config Configuration of TensorRT engine.
   */
  explicit TrtSimpl(const tensorrt_common::TrtCommonConfig & config);

  /**
   * @brief Execute inference.
   *
   * @param agent_tensor Agent tensor.
   * @param map_tensor Map tensor.
   * @param rpe_tensor RPE (Relative Pose Encoding) tensor.
   * @return archetype::Result<output_type>
   */
  archetype::Result<output_type> do_inference(
    const archetype::AgentTensor & agent_tensor, const archetype::MapTensor & map_tensor,
    const std::vector<float> & rpe_tensor) noexcept;

private:
  /**
   * @brief Initialize and setup cuda pointers.
   */
  void init_cuda_ptr(
    const archetype::AgentTensor & agent_tensor, const archetype::MapTensor & map_tensor,
    const std::vector<float> & rpe_tensor);

  std::unique_ptr<tensorrt_common::TrtCommon> trt_common_;  //!< TensorRT common.
  cudaStream_t stream_;                                     //!< CUDA stream.

  cuda_utils::CudaUniquePtr<float[]> in_agent_d_;  //!< Input agent tensor on device.
  cuda_utils::CudaUniquePtr<float[]> in_map_d_;    //!< Input map tensor on device.
  cuda_utils::CudaUniquePtr<float[]> in_rpe_d_;    //!< Input RPE tensor on device.

  cuda_utils::CudaUniquePtr<float[]> out_score_d_;       //!< Output score tensor on device.
  cuda_utils::CudaUniquePtr<float[]> out_trajectory_d_;  //!< Output trajectory tensor on device.
};
}  // namespace autoware::simpl_prediction
#endif  // AUTOWARE__SIMPL_PREDICTION__TRT_SIMPL_HPP_
