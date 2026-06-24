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

#ifndef AUTOWARE__PTV3__EXECUTION_CONTEXT_HPP_
#define AUTOWARE__PTV3__EXECUTION_CONTEXT_HPP_

#include <cuda_runtime_api.h>

namespace autoware::ptv3
{

/**
 * Non-owning state shared by every phase of one PTv3 inference.
 *
 * PTv3TRT owns the CUDA stream lifetime. Modules receive this context instead of a raw stream so a
 * caller cannot accidentally pass different streams to preprocess, enqueue, and postprocess phases
 * of the same inference call. Leaf CUDA helpers still take cudaStream_t because they only enqueue
 * immediate work and do not model pipeline phase ordering.
 */
class PTv3ExecutionContext
{
public:
  explicit PTv3ExecutionContext(cudaStream_t stream) : stream_(stream) {}

  PTv3ExecutionContext(const PTv3ExecutionContext &) = delete;
  PTv3ExecutionContext & operator=(const PTv3ExecutionContext &) = delete;

  [[nodiscard]] cudaStream_t stream() const { return stream_; }

private:
  cudaStream_t stream_;
};

}  // namespace autoware::ptv3

#endif  // AUTOWARE__PTV3__EXECUTION_CONTEXT_HPP_
