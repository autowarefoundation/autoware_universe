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

#ifndef AUTOWARE__TENSORRT_VAD__NETWORKS__TENSOR_HPP_
#define AUTOWARE__TENSORRT_VAD__NETWORKS__TENSOR_HPP_

#include "autoware/tensorrt_vad/ros_vad_logger.hpp"

#include <NvInfer.h>
#include <NvInferRuntime.h>
#include <cuda_fp16.h>
#include <cuda_runtime_api.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::tensorrt_vad
{

// Function to get element size for different data types
unsigned int getElementSize(nvinfer1::DataType t);

struct Tensor
{
  std::string name;
  void * ptr;
  nvinfer1::Dims dim;
  int32_t volume = 1;
  nvinfer1::DataType dtype;
  std::shared_ptr<VadLogger> logger_;

  // Constructor
  Tensor(
    std::string name, nvinfer1::Dims dim, nvinfer1::DataType dtype,
    std::shared_ptr<VadLogger> logger = nullptr);

  // Get number of bytes
  int32_t nbytes();

  // Load data from host to device
  template <class Dtype = float>
  void load(const std::vector<float> & data, cudaStream_t stream = 0);

  // Copy data from device to host
  template <class T>
  std::vector<T> cpu();

};  // struct Tensor

using TensorMap = std::unordered_map<std::string, std::shared_ptr<Tensor>>;

}  // namespace autoware::tensorrt_vad

#endif  // AUTOWARE__TENSORRT_VAD__NETWORKS__TENSOR_HPP_
