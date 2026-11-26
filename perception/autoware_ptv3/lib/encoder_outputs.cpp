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

#include "autoware/ptv3/encoder_outputs.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::ptv3
{
namespace
{

using tensorrt_common::NetworkIO;
using tensorrt_common::ProfileDims;
using tensorrt_common::TensorInfo;
using tensorrt_common::TrtCommon;

std::size_t elementSize(const nvinfer1::DataType dtype)
{
  switch (dtype) {
    case nvinfer1::DataType::kINT64:
      return 8;
    case nvinfer1::DataType::kFLOAT:
    case nvinfer1::DataType::kINT32:
      return 4;
    case nvinfer1::DataType::kHALF:
    case nvinfer1::DataType::kBF16:
      return 2;
    case nvinfer1::DataType::kINT8:
    case nvinfer1::DataType::kUINT8:
    case nvinfer1::DataType::kBOOL:
    case nvinfer1::DataType::kFP8:
      return 1;
    default:
      throw std::runtime_error(
        std::string("Unsupported encoder output dtype: ") + tensorrt_common::dtype_name(dtype));
  }
}

bool isResolved(const nvinfer1::Dims & dims)
{
  return dims.nbDims >= 0 &&
         std::all_of(dims.d, dims.d + dims.nbDims, [](const auto extent) { return extent >= 0; });
}

bool isDynamic(const nvinfer1::Dims & dims)
{
  return std::any_of(dims.d, dims.d + dims.nbDims, [](const auto extent) { return extent < 0; });
}

std::size_t volume(const nvinfer1::Dims & dims)
{
  std::size_t count = 1;
  for (std::int32_t i = 0; i < dims.nbDims; ++i) {
    count *= static_cast<std::size_t>(dims.d[i]);
  }
  return count;
}

}  // namespace

void EncoderOutputs::offerOutputs(TrtCommon & encoder, std::vector<NetworkIO> & network_io)
{
  const auto network = encoder.getNetwork();
  for (std::int32_t i = 0; i < network->getNbOutputs(); ++i) {
    const auto * output = network->getOutput(i);
    network_io.emplace_back(output->getName(), output->getDimensions());
  }
}

void EncoderOutputs::allocate(TrtCommon & encoder, const std::vector<ProfileDims> & encoder_profile)
{
  tensors_.clear();
  for (std::int32_t i = 0; i < encoder.getNbIOTensors(); ++i) {
    const char * name = encoder.getIOTensorName(i);
    if (encoder.getTensorIOMode(name) == nvinfer1::TensorIOMode::kOUTPUT) {
      tensors_.push_back(Tensor{name, *encoder.getTensorDataType(name), {}, {}, {}, nullptr});
    }
  }

  const auto resolve = [&encoder, &encoder_profile, this](
                         nvinfer1::Dims ProfileDims::* profile_field,
                         nvinfer1::Dims Tensor::* tensor_field) {
    for (const auto & input : encoder_profile) {
      encoder.setInputShape(input.tensor_name.c_str(), input.*profile_field);
    }
    for (auto & tensor : tensors_) {
      tensor.*tensor_field = encoder.getResolvedTensorShape(tensor.name.c_str());
    }
  };
  resolve(&ProfileDims::min_dims, &Tensor::min_dims);
  resolve(&ProfileDims::opt_dims, &Tensor::opt_dims);
  resolve(&ProfileDims::max_dims, &Tensor::max_dims);

  for (auto & tensor : tensors_) {
    if (!isResolved(tensor.max_dims)) {
      throw std::runtime_error(
        "Encoder output '" + tensor.name + "' has a data-dependent shape " +
        TensorInfo::dimsRepr(tensor.max_dims) + ", which cannot be forwarded to the heads.");
    }
    tensor.data =
      cuda_utils::make_unique<std::uint8_t[]>(volume(tensor.max_dims) * elementSize(tensor.dtype));
    encoder.setTensorAddress(tensor.name.c_str(), tensor.data.get());
  }
}

std::vector<std::string> EncoderOutputs::offerInputs(
  TrtCommon & head, std::vector<NetworkIO> & network_io,
  std::vector<ProfileDims> & profile_dims) const
{
  std::vector<std::string> names;
  const auto network = head.getNetwork();
  for (std::int32_t i = 0; i < network->getNbInputs(); ++i) {
    const auto * input = network->getInput(i);
    const auto * tensor = find(input->getName());
    if (tensor == nullptr) {
      continue;
    }
    const auto dims = input->getDimensions();
    if (dims.nbDims != tensor->max_dims.nbDims) {
      throw std::runtime_error(
        "Head input '" + tensor->name + "' has rank " + std::to_string(dims.nbDims) +
        ", but the encoder output has rank " + std::to_string(tensor->max_dims.nbDims) + ".");
    }
    // The head reads the encoder's buffer as it is, so it must take the encoder's dtype.
    network_io.emplace_back(tensor->name, dims, tensor->dtype);
    if (isDynamic(dims)) {
      profile_dims.emplace_back(tensor->name, tensor->min_dims, tensor->opt_dims, tensor->max_dims);
    }
    names.push_back(tensor->name);
  }
  return names;
}

void EncoderOutputs::bind(TrtCommon & head, const std::vector<std::string> & names) const
{
  for (const auto & name : names) {
    head.setTensorAddress(name.c_str(), find(name)->data.get());
  }
}

bool EncoderOutputs::setInputShapes(
  const TrtCommon & encoder, TrtCommon & head, const std::vector<std::string> & names) const
{
  bool success = true;
  for (const auto & name : names) {
    success &= head.setInputShape(name.c_str(), encoder.getResolvedTensorShape(name.c_str()));
  }
  return success;
}

const EncoderOutputs::Tensor * EncoderOutputs::find(const std::string & name) const
{
  const auto it = std::find_if(
    tensors_.begin(), tensors_.end(), [&name](const auto & tensor) { return tensor.name == name; });
  return it == tensors_.end() ? nullptr : &*it;
}

}  // namespace autoware::ptv3
