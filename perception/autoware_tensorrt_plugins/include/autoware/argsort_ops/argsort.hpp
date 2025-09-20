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

#ifndef AUTOWARE__ARGSORT_OPS__ARGSORT_HPP_
#define AUTOWARE__ARGSORT_OPS__ARGSORT_HPP_

#include <cuda_runtime_api.h>

#include <cstdint>

cudaError_t argsort(
  const std::int64_t * input, std::int64_t * output, void * workspace, std::size_t num_elements,
  std::size_t argsort_workspace_size, cudaStream_t stream);

std::size_t get_argsort_workspace_size(std::size_t num_elements);

#endif  // AUTOWARE__ARGSORT_OPS__ARGSORT_HPP_
