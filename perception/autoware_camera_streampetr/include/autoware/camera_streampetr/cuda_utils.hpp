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
/*
 * Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

/*
 * This code is licensed under CC0 1.0 Universal (Public Domain).
 * You can use this without any limitation.
 * https://creativecommons.org/publicdomain/zero/1.0/deed.en
 */

#ifndef AUTOWARE__CAMERA_STREAMPETR__CUDA_UTILS_HPP_
#define AUTOWARE__CAMERA_STREAMPETR__CUDA_UTILS_HPP_

#include <autoware/cuda_utils/cuda_check_error.hpp>
#include <autoware/cuda_utils/cuda_unique_ptr.hpp>
#include <autoware/cuda_utils/cuda_utils.hpp>

#include <NvInferRuntime.h>
#include <cuda_runtime_api.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <type_traits>
#include <vector>

namespace cuda
{

using namespace nvinfer1;

inline unsigned int getElementSize(DataType t)
{
  switch (t) {
    case DataType::kINT32:
      return 4;
    case DataType::kFLOAT:
      return 4;
    case DataType::kHALF:
      return 2;
    case DataType::kINT8:
      return 1;
    case DataType::kUINT8:
      return 1;
    default:
      throw std::runtime_error("Invalid DataType.");
  }
  return 0;
}

struct Tensor
{
  std::string name;
  void * ptr;
  Dims dim;
  int32_t volume = 1;
  DataType dtype;
  TensorIOMode iomode;

  Tensor(std::string name, Dims dim, DataType dtype) : name(name), dim(dim), dtype(dtype)
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

  int32_t nbytes() const { return volume * getElementSize(dtype); }

  ~Tensor()
  {
    if (ptr) {
      cudaFree(ptr);
      ptr = nullptr;
    }
  }

  void copy(std::shared_ptr<Tensor> other, cudaStream_t stream)
  {
    // copy from 'other'
    cudaMemcpyAsync(ptr, other->ptr, nbytes(), cudaMemcpyDeviceToDevice, stream);
  }

  void initialize_to_zeros(cudaStream_t stream) { cudaMemsetAsync(ptr, 0, nbytes(), stream); }

  // template<class Htype=float, class Dtype=float>
  template <class Htype = float>
  void load_from_vector(const std::vector<Htype> & data)
  {
    if (data.size() != static_cast<std::size_t>(volume)) {
      std::cerr << "Data size mismatch! Expected " << volume << " elements." << std::endl;
      return;
    }

    std::size_t dsize = volume * getElementSize(dtype);
    cudaMemcpy(ptr, data.data(), dsize, cudaMemcpyHostToDevice);
  }

  std::vector<float> copy_tensor_to_host_buffer() const
  {
    std::vector<float> buffer(volume);
    cudaMemcpy(buffer.data(), ptr, volume * sizeof(float), cudaMemcpyDeviceToHost);
    return buffer;
  }

  std::vector<char> load_ref(std::string fname)
  {
    std::size_t bsize = volume * sizeof(float);
    std::vector<char> buffer(bsize);
    std::ifstream file_(fname, std::ios::binary);
    file_.read(buffer.data(), bsize);
    return buffer;
  }

  /**
   * @brief Save tensor data as a numpy array file (.npy). Useful for debugging.
   * @param filepath Path where to save the numpy file
   * @return true if successful, false otherwise
   */
  bool save_as_numpy(const std::string & filepath) const
  {
    // Check if file already exists
    if (std::filesystem::exists(filepath)) {
      std::cout << "File already exists: " << filepath << std::endl;
      return false;
    }

    // Create directory if it doesn't exist
    std::filesystem::path path(filepath);
    std::filesystem::path dir = path.parent_path();
    if (!dir.empty() && !std::filesystem::exists(dir)) {
      try {
        std::filesystem::create_directories(dir);
      } catch (const std::exception & e) {
        std::cerr << "Failed to create directory: " << e.what() << std::endl;
        return false;
      }
    }

    // Copy data from GPU to CPU
    std::vector<float> cpu_data = copy_tensor_to_host_buffer();

    // Open file for writing
    std::ofstream writer_stream(filepath, std::ios::out | std::ios::binary);
    if (!writer_stream) {
      std::cerr << "Cannot open file for write: " << filepath << std::endl;
      return false;
    }

    // Write numpy magic string and version
    char magic[] = {'\x93', 'N', 'U', 'M', 'P', 'Y'};
    char version[] = {'\x01', '\x00'};
    writer_stream.write(magic, sizeof(magic));
    writer_stream.write(version, sizeof(version));

    // Determine numpy dtype string based on TensorRT DataType
    std::string dtype_str;
    switch (dtype) {
      case DataType::kFLOAT:
        dtype_str = "<f4";
        break;
      case DataType::kHALF:
        dtype_str = "<f2";
        break;
      case DataType::kINT32:
        dtype_str = "<i4";
        break;
      case DataType::kINT8:
        dtype_str = "<i1";
        break;
      case DataType::kUINT8:
        dtype_str = "|u1";
        break;
      case DataType::kBOOL:
        dtype_str = "|b1";
        break;
      case DataType::kFP8:
        std::cerr << "FP8 not directly supported, converting to float" << std::endl;
        dtype_str = "<f4";
        break;
      default:
        std::cerr << "Unsupported data type for numpy save" << std::endl;
        return false;
    }

    // Construct header
    std::stringstream header;
    header << "{'descr': '" << dtype_str << "', 'fortran_order': False, 'shape': (";

    for (int i = 0; i < dim.nbDims; i++) {
      if (i > 0) header << ", ";
      header << dim.d[i];
    }
    header << "), }";

    // Pad header to 16 bytes alignment
    std::string headerStr = header.str();
    int32_t headerLen = 10 + headerStr.length();
    int32_t padding = 16 - ((headerLen + 1) % 16);
    headerStr.append(padding, ' ');
    headerStr += '\n';

    // Write header length and header
    uint16_t headerSize = headerStr.length();
    writer_stream.write(reinterpret_cast<char *>(&headerSize), sizeof(uint16_t));
    writer_stream.write(headerStr.c_str(), headerSize);

    // Write data
    writer_stream.write(
      reinterpret_cast<const char *>(cpu_data.data()), cpu_data.size() * sizeof(float));
    writer_stream.close();

    std::cout << "Tensor '" << name << "' saved as numpy array to: " << filepath << std::endl;
    return true;
  }
};  // struct Tensor

inline std::ostream & operator<<(std::ostream & os, Tensor & t)
{
  os << "[" << (int)(t.iomode) << "] ";
  os << t.name << ", [";

  for (int nd = 0; nd < t.dim.nbDims; nd++) {
    if (nd == 0) {
      os << t.dim.d[nd];
    } else {
      os << ", " << t.dim.d[nd];
    }
  }
  os << "]";

  // Convert DataType to string
  std::string dtype_str;
  switch (t.dtype) {
    case DataType::kFLOAT:
      dtype_str = "FLOAT32";
      break;
    case DataType::kHALF:
      dtype_str = "FLOAT16";
      break;
    case DataType::kINT32:
      dtype_str = "INT32";
      break;
    case DataType::kINT8:
      dtype_str = "INT8";
      break;
    case DataType::kBOOL:
      dtype_str = "BOOL";
      break;
    case DataType::kUINT8:
      dtype_str = "UINT8";
      break;
    case DataType::kFP8:
      dtype_str = "FP8";
      break;
    default:
      dtype_str = "UNKNOWN";
      break;
  }
  os << ", type = " << dtype_str;
  return os;
}

}  // namespace cuda

#endif  // AUTOWARE__CAMERA_STREAMPETR__CUDA_UTILS_HPP_
