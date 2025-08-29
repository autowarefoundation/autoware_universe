// Copyright 2025 Tier IV, Inc.
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

#ifndef AUTOWARE__CUDA_UTILS__THRUST_UTILS_HPP_
#define AUTOWARE__CUDA_UTILS__THRUST_UTILS_HPP_

#include <cuda_runtime_api.h>
#include <thrust/count.h>
#include <thrust/device_vector.h>
#include <thrust/execution_policy.h>
#include <thrust/fill.h>

namespace autoware::cuda_utils
{

[[nodiscard]] inline auto thrust_on_stream(cudaStream_t stream)
{
  return thrust::cuda::par.on(stream);
}

namespace thrust_stream
{

template <typename T, typename AllocT>
void fill(
  typename thrust::device_vector<T, AllocT>::iterator first,
  typename thrust::device_vector<T, AllocT>::iterator last, const T & value, cudaStream_t stream)
{
  thrust::fill(thrust_on_stream(stream), first, last, value);
}

template <typename T, typename AllocT>
void fill(thrust::device_vector<T, AllocT> & vec, const T & value, cudaStream_t stream)
{
  fill<T, AllocT>(vec.begin(), vec.end(), value, stream);
}

template <typename T, typename AllocT>
void fill_n(
  typename thrust::device_vector<T, AllocT>::iterator first, std::size_t n, const T & value,
  cudaStream_t stream)
{
  thrust::fill_n(thrust_on_stream(stream), first, n, value);
}

template <typename T, typename AllocT>
void fill_n(
  thrust::device_vector<T, AllocT> & vec, std::size_t n, const T & value, cudaStream_t stream)
{
  fill_n<T, AllocT>(vec.begin(), n, value, stream);
}

template <typename T, typename AllocT>
auto count(
  typename thrust::device_vector<T, AllocT>::iterator first,
  typename thrust::device_vector<T, AllocT>::iterator last, const T & value, cudaStream_t stream)
{
  return thrust::count(thrust_on_stream(stream), first, last, value);
}

template <typename T, typename AllocT>
auto count(thrust::device_vector<T, AllocT> & vec, const T & value, cudaStream_t stream)
{
  return count<T, AllocT>(vec.begin(), vec.end(), value, stream);
}

}  // namespace thrust_stream

}  // namespace autoware::cuda_utils

#endif  // AUTOWARE__CUDA_UTILS__THRUST_UTILS_HPP_
