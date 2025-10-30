/*
 * SPDX-FileCopyrightText: Copyright (c) 2023-2025 NVIDIA CORPORATION & AFFILIATES. All rights
 * reserved. SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "autoware/tensorrt_vad/networks/tensor.hpp"

#include <iostream>

namespace autoware::tensorrt_vad
{

unsigned int getElementSize(nvinfer1::DataType t)
{
  switch (t) {
    case nvinfer1::DataType::kFLOAT:
      return sizeof(float);
    case nvinfer1::DataType::kHALF:
      return sizeof(float) / 2;
    case nvinfer1::DataType::kINT8:
      return sizeof(int8_t);
    case nvinfer1::DataType::kINT32:
      return sizeof(int32_t);
    case nvinfer1::DataType::kBOOL:
      return sizeof(bool);
    case nvinfer1::DataType::kUINT8:
      return sizeof(uint8_t);
    case nvinfer1::DataType::kFP8:
      return sizeof(float) / 4;
    default:
      return 0;
  }
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
    std::vector<char> buffer(dsize);
    Dtype * dbuffer = reinterpret_cast<Dtype *>(buffer.data());

    for (int i = 0; i < volume; i++) {
      dbuffer[i] = static_cast<Dtype>(data[i]);
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
