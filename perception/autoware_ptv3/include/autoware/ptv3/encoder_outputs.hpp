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

#ifndef AUTOWARE__PTV3__ENCODER_OUTPUTS_HPP_
#define AUTOWARE__PTV3__ENCODER_OUTPUTS_HPP_

#include <autoware/cuda_utils/cuda_unique_ptr.hpp>
#include <autoware/tensorrt_common/tensorrt_common.hpp>
#include <autoware/tensorrt_common/utils.hpp>

#include <NvInfer.h>

#include <cstdint>
#include <string>
#include <vector>

namespace autoware::ptv3
{

/// Owns a device buffer for every encoder output and binds it to the head inputs of the same name.
/// A head input takes the dtype and the shapes of the encoder output, so the encoder and the heads
/// agree on names only.
class EncoderOutputs
{
public:
  /// Before the encoder's setup(): offer every output of the parsed encoder network.
  static void offerOutputs(
    tensorrt_common::TrtCommon & encoder, std::vector<tensorrt_common::NetworkIO> & network_io);

  /// After the encoder's setup(): resolve each output's shape at the min, opt and max shapes of
  /// the encoder profile, then allocate each output at its max shape and bind it.
  /// @throw std::runtime_error If an output's shape depends on the input data.
  void allocate(
    tensorrt_common::TrtCommon & encoder,
    const std::vector<tensorrt_common::ProfileDims> & encoder_profile);

  /// Before a head's setup(): offer the head inputs that are encoder outputs.
  /// @return The names of the offered inputs.
  [[nodiscard]] std::vector<std::string> offerInputs(
    tensorrt_common::TrtCommon & head, std::vector<tensorrt_common::NetworkIO> & network_io,
    std::vector<tensorrt_common::ProfileDims> & profile_dims) const;

  /// After a head's setup(): bind the offered inputs to the encoder's buffers.
  void bind(tensorrt_common::TrtCommon & head, const std::vector<std::string> & names) const;

  /// Per frame, after the encoder's input shapes are set: give the offered head inputs the shapes
  /// of the encoder outputs.
  bool setInputShapes(
    const tensorrt_common::TrtCommon & encoder, tensorrt_common::TrtCommon & head,
    const std::vector<std::string> & names) const;

private:
  struct Tensor
  {
    std::string name;
    nvinfer1::DataType dtype;
    nvinfer1::Dims min_dims;
    nvinfer1::Dims opt_dims;
    nvinfer1::Dims max_dims;
    cuda_utils::CudaUniquePtr<std::uint8_t[]> data;
  };

  [[nodiscard]] const Tensor * find(const std::string & name) const;

  std::vector<Tensor> tensors_;
};

}  // namespace autoware::ptv3

#endif  // AUTOWARE__PTV3__ENCODER_OUTPUTS_HPP_
