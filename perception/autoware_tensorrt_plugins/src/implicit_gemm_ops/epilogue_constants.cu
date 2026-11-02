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

#include "autoware/implicit_gemm_ops/epilogue_constants.hpp"

namespace
{
constexpr int kBlockSize = 256;

__global__ void scale_per_channel_kernel(
  const float * __restrict__ weight_scale, float input_scale, float * __restrict__ out,
  std::int64_t c_out)
{
  std::int64_t idx = static_cast<std::int64_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (idx < c_out) {
    out[idx] = input_scale * weight_scale[idx];
  }
}
}  // namespace

void launch_scale_per_channel(
  const float * weight_scale, float input_scale, float * out, std::int64_t c_out,
  cudaStream_t stream)
{
  int grid = static_cast<int>((c_out + kBlockSize - 1) / kBlockSize);
  scale_per_channel_kernel<<<grid, kBlockSize, 0, stream>>>(weight_scale, input_scale, out, c_out);
}
