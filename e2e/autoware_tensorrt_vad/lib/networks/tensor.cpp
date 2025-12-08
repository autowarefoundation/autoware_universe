// Copyright 2023-2025 NVIDIA CORPORATION & AFFILIATES.
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

#include "../src/networks/tensor.hpp"

#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::tensorrt_vad
{

unsigned int getElementSize(nvinfer1::DataType t)
{
  // Static map for data type sizes
  static const std::unordered_map<nvinfer1::DataType, unsigned int> size_map = {
    {nvinfer1::DataType::kFLOAT, sizeof(float)},  {nvinfer1::DataType::kHALF, sizeof(float) / 2},
    {nvinfer1::DataType::kINT8, sizeof(int8_t)},  {nvinfer1::DataType::kINT32, sizeof(int32_t)},
    {nvinfer1::DataType::kBOOL, sizeof(bool)},    {nvinfer1::DataType::kUINT8, sizeof(uint8_t)},
    {nvinfer1::DataType::kFP8, sizeof(float) / 4}};

  auto it = size_map.find(t);
  return (it != size_map.end()) ? it->second : 0;
}

Tensor::Tensor(
  std::string name, nvinfer1::Dims dim, nvinfer1::DataType dtype, std::shared_ptr<VadLogger> logger)
: name(name), dim(dim), dtype(dtype), logger_(logger)
{
  if (dim.nbDims == 0) {
    volume = 0;
  } else {
    volume = 1;
    for (int i = 0; i < dim.nbDims; i++) {
      volume *= dim.d[i];
    }
  }
  cudaMalloc(&ptr, volume * getElementSize(dtype));
}

int32_t Tensor::nbytes()
{
  return volume * getElementSize(dtype);
}

template <class Dtype>
void Tensor::load(const std::vector<float> & data, cudaStream_t stream)
{
  if (static_cast<int32_t>(data.size()) != volume) {
    logger_->error(
      "Data size mismatch: expected " + std::to_string(volume) + ", got " +
      std::to_string(data.size()));
    return;
  }

  size_t dsize = volume * getElementSize(dtype);

  if (dtype == nvinfer1::DataType::kFLOAT) {
    // Direct copy for float data
    cudaMemcpyAsync(ptr, data.data(), dsize, cudaMemcpyHostToDevice, stream);
  } else {
    // Type conversion needed
    std::vector<Dtype> buffer(volume);

    for (int i = 0; i < volume; i++) {
      buffer[i] = static_cast<Dtype>(data[i]);
    }

    cudaMemcpyAsync(ptr, buffer.data(), dsize, cudaMemcpyHostToDevice, stream);
  }
}

template <class T>
std::vector<T> Tensor::cpu()
{
  std::vector<T> buffer(volume);
  cudaMemcpy(buffer.data(), ptr, volume * sizeof(T), cudaMemcpyDeviceToHost);
  return buffer;
}

// Explicit template instantiations
template void Tensor::load<float>(const std::vector<float> & data, cudaStream_t stream);

template std::vector<float> Tensor::cpu<float>();

}  // namespace autoware::tensorrt_vad
