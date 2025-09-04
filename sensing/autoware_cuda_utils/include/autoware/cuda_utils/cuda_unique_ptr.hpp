// Copyright 2022 Tier IV, Inc.
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

// This code is licensed under CC0 1.0 Universal (Public Domain).
// You can use this without any limitation.
// https://creativecommons.org/publicdomain/zero/1.0/deed.en
// borrowed from https://proc-cpuinfo.fixstars.com/2019/02/cuda_smart_pointer/

#ifndef AUTOWARE__CUDA_UTILS__CUDA_UNIQUE_PTR_HPP_
#define AUTOWARE__CUDA_UTILS__CUDA_UNIQUE_PTR_HPP_

#include "autoware/cuda_utils/cuda_check_error.hpp"

#include <functional>
#include <memory>
#include <type_traits>

namespace autoware::cuda_utils
{
struct CudaDeleter
{
  void operator()(void * p) const { CHECK_CUDA_ERROR(::cudaFree(p)); }
};
template <typename T>
using CudaUniquePtr = std::unique_ptr<T, CudaDeleter>;

template <typename T>
typename std::enable_if_t<std::is_array<T>::value, CudaUniquePtr<T>> make_unique(
  const std::size_t n)
{
  using U = typename std::remove_extent_t<T>;
  U * p;
  CHECK_CUDA_ERROR(::cudaMalloc(reinterpret_cast<void **>(&p), sizeof(U) * n));
  return CudaUniquePtr<T>{p};
}

template <typename T>
CudaUniquePtr<T> make_unique()
{
  T * p;
  CHECK_CUDA_ERROR(::cudaMalloc(reinterpret_cast<void **>(&p), sizeof(T)));
  return CudaUniquePtr<T>{p};
}

template <typename T>
using CudaPooledUniquePtr = std::unique_ptr<T, std::function<void(T *)>>;

template <typename T>
CudaPooledUniquePtr<T> make_unique(const std::size_t n, cudaStream_t stream, cudaMemPool_t pool)
{
  using U = typename std::remove_extent_t<T>;
  T * ptr = nullptr;

  CHECK_CUDA_ERROR(
    cudaMallocFromPoolAsync(reinterpret_cast<void **>(&ptr), sizeof(U) * n, pool, stream));
  // Custom deleter calls cudaFreeAsync with the same stream
  auto deleter = [stream](T * p) {
    if (p) {
      CHECK_CUDA_ERROR(cudaFreeAsync(p, stream));
    }
  };

  // To prevent unexpected behavior caused by dirty region allocated by the pool,
  // zero clear the taken region
  CHECK_CUDA_ERROR(cudaMemsetAsync(ptr, 0, n * sizeof(T), stream));

  return CudaPooledUniquePtr<T>(ptr, deleter);
}

template <typename T>
CudaPooledUniquePtr<T> make_unique(cudaStream_t stream, cudaMemPool_t pool)
{
  return make_unique<T>(1, stream, pool);
}

struct CudaDeleterHost
{
  void operator()(void * p) const { CHECK_CUDA_ERROR(::cudaFreeHost(p)); }
};
template <typename T>
using CudaUniquePtrHost = std::unique_ptr<T, CudaDeleterHost>;

template <typename T>
typename std::enable_if_t<std::is_array<T>::value, CudaUniquePtrHost<T>> make_unique_host(
  const std::size_t n, unsigned int flag)
{
  using U = typename std::remove_extent_t<T>;
  U * p;
  CHECK_CUDA_ERROR(::cudaHostAlloc(reinterpret_cast<void **>(&p), sizeof(U) * n, flag));
  return CudaUniquePtrHost<T>{p};
}

template <typename T>
CudaUniquePtrHost<T> make_unique_host(unsigned int flag = cudaHostAllocDefault)
{
  T * p;
  CHECK_CUDA_ERROR(::cudaHostAlloc(reinterpret_cast<void **>(&p), sizeof(T), flag));
  return CudaUniquePtrHost<T>{p};
}
}  // namespace autoware::cuda_utils

#endif  // AUTOWARE__CUDA_UTILS__CUDA_UNIQUE_PTR_HPP_
