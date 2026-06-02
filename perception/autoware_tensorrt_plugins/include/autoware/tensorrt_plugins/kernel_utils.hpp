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

#ifndef AUTOWARE__TENSORRT_PLUGINS__KERNEL_UTILS_HPP_
#define AUTOWARE__TENSORRT_PLUGINS__KERNEL_UTILS_HPP_

#include <cstddef>
#include <cstdint>

namespace autoware::tensorrt_plugins
{

inline std::size_t align_up(const std::size_t size, const std::size_t alignment)
{
  return ((size + alignment - 1U) / alignment) * alignment;
}

static __global__ void fill_iota(std::int64_t * output_out, const std::size_t num_elements_in)
{
  const auto index = static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (index >= num_elements_in) {
    return;
  }

  output_out[index] = static_cast<std::int64_t>(index);
}

}  // namespace autoware::tensorrt_plugins

#endif  // AUTOWARE__TENSORRT_PLUGINS__KERNEL_UTILS_HPP_
