// Copyright 2025 The Autoware Contributors
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
//
/*
 * This file includes portions of code directly from the BEVFormer TensorRT implementation
 * by Derry Lin, available at:
 *   https://github.com/DerryHub/BEVFormer_tensorrt
 *
 * The included code is used under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * The Autoware Contributors have reused this code as-is in 2025, with no modifications.
 * Original creation by Derry Lin on 2022/10/22.
 */

// cspell:ignore BEVFORMER, transa, transb, DFALT

#ifndef PERCEPTION__AUTOWARE_TENSORRT_BEVFORMER__TENSORRT__COMMON__CUDA_HELPER_H_
#define PERCEPTION__AUTOWARE_TENSORRT_BEVFORMER__TENSORRT__COMMON__CUDA_HELPER_H_

#include <cublas_v2.h>
#include <cuda.h>

#include <algorithm>
#include <cstdio>
#include <iostream>

#define CUDA_1D_KERNEL_LOOP(i, n) \
  for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < (n); i += blockDim.x * gridDim.x)

#define THREADS_PER_BLOCK 512

#define DIVUP(m, n) ((m) / (n) + ((m) % (n) > 0))

#define GET_BLOCKS(x) DIVUP(x, THREADS_PER_BLOCK)
// inline int GET_BLOCKS(const int N) {
//  int optimal_block_num = DIVUP(N, THREADS_PER_BLOCK);
//    if (optimal_block_num > MAX_BLOCK_NUM) { \
//      std::cerr << "#assertion" << __FILE__ << "," << __LINE__ << std::endl; \
//      abort(); \
//    }
//    return optimal_block_num;
//}

#define cudaCheckError()                                                               \
  {                                                                                    \
    cudaError_t e = cudaGetLastError();                                                \
    if (e != cudaSuccess) {                                                            \
      printf("Cuda failure %s:%d: '%s'\n", __FILE__, __LINE__, cudaGetErrorString(e)); \
      exit(0);                                                                         \
    }                                                                                  \
  }

/**
 * Returns a view of the original tensor with its dimensions permuted.
 *
 * @param[out] dst pointer to the destination tensor
 * @param[in] src pointer to the source tensor
 * @param[in] src_size shape of the src tensor
 * @param[in] permute The desired ordering of dimensions
 * @param[in] src_dim dim of src tensor
 * @param[in] stream cuda stream handle
 */
template <class scalar_t>
void memcpyPermute(
  scalar_t * dst, const scalar_t * src, int * src_size, int * permute, int src_dim,
  cudaStream_t stream = 0);

template <typename scalar_t>
cublasStatus_t cublasGemmWrap(
  cublasHandle_t handle, cublasOperation_t transa, cublasOperation_t transb, int m, int n, int k,
  const scalar_t * alpha, const scalar_t * A, int lda, const scalar_t * B, int ldb,
  const scalar_t * beta, scalar_t * C, int ldc,
  cublasGemmAlgo_t algo = CUBLAS_GEMM_DFALT_TENSOR_OP);

cublasStatus_t cublasGemmWrap_int8(
  cublasHandle_t handle, cublasOperation_t transa, cublasOperation_t transb, int m, int n, int k,
  const int32_t * alpha, const int8_t * A, int lda, const int8_t * B, int ldb, const int32_t * beta,
  int32_t * C, int ldc, cublasGemmAlgo_t algo = CUBLAS_GEMM_DFALT_TENSOR_OP);

template <typename scalar_t>
cublasStatus_t cublasGemmBatchedWrap(
  cublasHandle_t handle, cublasOperation_t transa, cublasOperation_t transb, int m, int n, int k,
  const scalar_t * alpha, const scalar_t * const Aarray[], int lda, const scalar_t * const Barray[],
  int ldb, const scalar_t * beta, scalar_t * const Carray[], int ldc, int batchCount,
  cublasGemmAlgo_t algo = CUBLAS_GEMM_DFALT_TENSOR_OP);

cublasStatus_t cublasGemmBatchedWrap_int8(
  cublasHandle_t handle, cublasOperation_t transa, cublasOperation_t transb, int m, int n, int k,
  const int32_t * alpha, const int8_t * const Aarray[], int lda, const int8_t * const Barray[],
  int ldb, const int32_t * beta, const int32_t * const Carray[], int ldc, int batchCount,
  cublasGemmAlgo_t algo = CUBLAS_GEMM_DFALT_TENSOR_OP);

template <typename scalar_t>
cublasStatus_t cublasGemmStridedBatchedWrap(
  cublasHandle_t handle, cublasOperation_t transa, cublasOperation_t transb, int m, int n, int k,
  const scalar_t * alpha, const scalar_t * A, int lda,
  long long int strideA,                                                // NOLINT(runtime/int)
  const scalar_t * B, int ldb, long long int strideB,                   // NOLINT(runtime/int)
  const scalar_t * beta, scalar_t * C, int ldc, long long int strideC,  // NOLINT(runtime/int)
  int batchCount, cublasGemmAlgo_t algo = CUBLAS_GEMM_DFALT_TENSOR_OP);

cublasStatus_t cublasGemmStridedBatchedWrap_int8(
  cublasHandle_t handle, cublasOperation_t transa, cublasOperation_t transb, int m, int n, int k,
  const int32_t * alpha, const int8_t * A, int lda, long long int strideA,  // NOLINT(runtime/int)
  const int8_t * B, int ldb, long long int strideB,                         // NOLINT(runtime/int)
  const int32_t * beta, const int32_t * C, int ldc, long long int strideC,  // NOLINT(runtime/int)
  int batchCount, cublasGemmAlgo_t algo = CUBLAS_GEMM_DFALT_TENSOR_OP);     // NOLINT(runtime/int)

template <typename scalar_t>
__device__ __forceinline__ scalar_t bilinear_interpolate(
  const scalar_t * __restrict__ input, const int height, const int width, scalar_t y, scalar_t x)
{
  // deal with cases that inverse elements are out of feature map boundary
  if (y < -1.0 || y > height || x < -1.0 || x > width) {
    return 0;
  }

  y = min(scalar_t(height - 1), max(scalar_t(0), y));
  x = min(scalar_t(width - 1), max(scalar_t(0), x));

  const int y_low = floor(y);
  const int x_low = floor(x);
  const int y_high = ceil(y);
  const int x_high = ceil(x);

  const scalar_t v1 = input[y_low * width + x_low];
  const scalar_t v2 = input[y_low * width + x_high];
  const scalar_t v3 = input[y_high * width + x_low];
  const scalar_t v4 = input[y_high * width + x_high];

  // lerp can be performed by fma
  const scalar_t ly = y - y_low;
  const scalar_t lx = x - x_low;
  const scalar_t v_low = fma(v2 - v1, lx, v1);
  const scalar_t v_high = fma(v4 - v3, lx, v3);
  const scalar_t val = fma(v_high - v_low, ly, v_low);

  return val;
}

#endif  // PERCEPTION__AUTOWARE_TENSORRT_BEVFORMER__TENSORRT__COMMON__CUDA_HELPER_H_
