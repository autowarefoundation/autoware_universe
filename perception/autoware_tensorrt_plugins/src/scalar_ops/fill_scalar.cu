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

#include "autoware/scalar_ops/fill_scalar.hpp"

namespace
{

__global__ void fill_scalar_int32_kernel(std::int32_t * output_out, const std::int32_t value_in)
{
  *output_out = value_in;
}

}  // namespace

cudaError_t fill_scalar_int32(std::int32_t * output, std::int32_t value, cudaStream_t stream)
{
  fill_scalar_int32_kernel<<<1, 1, 0, stream>>>(output, value);
  return cudaPeekAtLastError();
}
