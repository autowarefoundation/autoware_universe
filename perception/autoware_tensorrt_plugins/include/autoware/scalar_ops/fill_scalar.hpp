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

#ifndef AUTOWARE__SCALAR_OPS__FILL_SCALAR_HPP_
#define AUTOWARE__SCALAR_OPS__FILL_SCALAR_HPP_

#include <cuda_runtime_api.h>

#include <cstdint>

/// \brief Write a single host-known ``std::int32_t`` into device memory.
///
/// This exists so that plugins can publish a scalar they computed on the host without issuing a
/// host-to-device ``cudaMemcpyAsync`` from pageable memory. Such a copy has synchronous behaviour
/// and is rejected with ``cudaErrorStreamCaptureUnsupported`` while a stream is capturing, which
/// TensorRT does when it times tactics during engine build. A kernel launch carries its arguments
/// by value, so it is legal both during capture and during ordinary execution.
cudaError_t fill_scalar_int32(std::int32_t * output, std::int32_t value, cudaStream_t stream);

#endif  // AUTOWARE__SCALAR_OPS__FILL_SCALAR_HPP_
