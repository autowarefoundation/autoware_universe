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

#ifndef AUTOWARE__IMPLICIT_GEMM_OPS__EPILOGUE_CONSTANTS_HPP_
#define AUTOWARE__IMPLICIT_GEMM_OPS__EPILOGUE_CONSTANTS_HPP_

#include <cuda_runtime.h>

#include <cstdint>

// Per-output-channel constants the ImplicitGemm plugin derives once from its FP32 inputs.

/// INT8 GEMM scale: out[c] = input_scale * weight_scale[c].
void launch_scale_per_channel(
  const float * weight_scale, float input_scale, float * out, std::int64_t c_out,
  cudaStream_t stream);

#endif  // AUTOWARE__IMPLICIT_GEMM_OPS__EPILOGUE_CONSTANTS_HPP_
