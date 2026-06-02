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

#ifndef AUTOWARE__TENSORRT_PLUGINS__PINNED_HOST_BUFFER_HPP_
#define AUTOWARE__TENSORRT_PLUGINS__PINNED_HOST_BUFFER_HPP_

#include "autoware/tensorrt_plugins/plugin_utils.hpp"

#include <cuda_runtime_api.h>

#include <cstddef>
#include <utility>

namespace autoware::tensorrt_plugins
{

/// RAII owner of a page-locked (pinned) host allocation obtained via `cudaMallocHost`.
///
/// `cudaMemcpyAsync` is only asynchronous if the host memory is pinned, so performance can be
/// improved by using pinned host memory in some cases.
///
/// This class makes lifetime management of such pinned host buffers easy and safe.
///
/// Read more:
/// https://docs.nvidia.com/cuda/cuda-programming-guide/02-basics/asynchronous-execution.html#launching-memory-transfers-in-cuda-streams
///
/// Move-only: ownership is unique, so the buffer is freed exactly once.
template <typename T>
class PinnedHostBuffer
{
public:
  /// Allocates `count` elements of pinned host memory. Aborts (via `PLUGIN_ASSERT`) on failure.
  explicit PinnedHostBuffer(std::size_t count = 1U)
  {
    PLUGIN_ASSERT(
      cudaMallocHost(reinterpret_cast<void **>(&data_), count * sizeof(T)) == cudaSuccess);
  }

  ~PinnedHostBuffer() { reset(); }

  PinnedHostBuffer(const PinnedHostBuffer &) = delete;
  PinnedHostBuffer & operator=(const PinnedHostBuffer &) = delete;

  PinnedHostBuffer(PinnedHostBuffer && other) noexcept : data_{std::exchange(other.data_, nullptr)}
  {
  }

  PinnedHostBuffer & operator=(PinnedHostBuffer && other) noexcept
  {
    if (this != &other) {
      reset();
      data_ = std::exchange(other.data_, nullptr);
    }
    return *this;
  }

  [[nodiscard]] T * get() const noexcept { return data_; }
  T & operator*() const noexcept { return *data_; }
  T * operator->() const noexcept { return data_; }
  T & operator[](std::size_t index) const noexcept { return data_[index]; }

private:
  void reset() noexcept
  {
    if (data_ != nullptr) {
      cudaFreeHost(data_);
      data_ = nullptr;
    }
  }

  T * data_{nullptr};
};

}  // namespace autoware::tensorrt_plugins

#endif  // AUTOWARE__TENSORRT_PLUGINS__PINNED_HOST_BUFFER_HPP_
