// Copyright 2025 The Autoware Contributors
//
// Portions of this code are derived from the BEVFormer TensorRT implementation by Derry Lin:
// https://github.com/DerryHub/BEVFormer_tensorrt
//
// Original code licensed under the Apache License, Version 2.0 (the "License");
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

#include "cuda_helper.h"
#include "helper.h"

using helper::TensorDesc;

template <class scalar_t>
__global__ void copy_permute_kernel(
  scalar_t * __restrict__ dst, const scalar_t * __restrict__ src, int n, TensorDesc ts_src_stride,
  TensorDesc ts_dst_stride, TensorDesc ts_permute)
{
  const int src_dim = ts_src_stride.dim;
  const auto src_stride = ts_src_stride.stride;
  const auto dst_stride = ts_dst_stride.stride;
  const auto permute = ts_permute.shape;
  CUDA_1D_KERNEL_LOOP(index, n)
  {
    size_t dst_index = index;
    size_t src_index = 0;
    for (int i = 0; i < src_dim; ++i) {
      int dim_index = dst_index / dst_stride[i];
      dst_index = dst_index % dst_stride[i];
      src_index += dim_index * src_stride[permute[i]];
    }
    dst[index] = src[src_index];
  }
}

template <class scalar_t>
void memcpyPermute(
  scalar_t * dst, const scalar_t * src, int * src_size, int * permute, int src_dim,
  cudaStream_t stream)
{
  size_t copy_size = 1;
  TensorDesc ts_permute;
  memcpy(&(ts_permute.shape[0]), permute, src_dim * sizeof(int));

  TensorDesc ts_src_stride;
  TensorDesc ts_dst_stride;
  ts_src_stride.dim = src_dim;
  ts_dst_stride.dim = src_dim;
  int * src_stride = &(ts_src_stride.stride[0]);
  int * dst_stride = &(ts_dst_stride.stride[0]);
  int * dst_size = &(ts_dst_stride.shape[0]);
  src_stride[src_dim - 1] = 1;
  dst_stride[src_dim - 1] = 1;

  for (int i = src_dim - 1; i >= 0; --i) {
    dst_size[i] = src_size[permute[i]];
    if (i < src_dim - 1) {
      src_stride[i] = src_stride[i + 1] * src_size[i + 1];
    }
  }

  for (int i = src_dim - 1; i >= 0; --i) {
    copy_size *= dst_size[i];
    if (i < src_dim - 1) {
      dst_stride[i] = dst_stride[i + 1] * dst_size[i + 1];
    }
  }

  copy_permute_kernel<scalar_t><<<GET_BLOCKS(copy_size), THREADS_PER_BLOCK, 0, stream>>>(
    dst, src, copy_size, ts_src_stride, ts_dst_stride, ts_permute);
}

template void memcpyPermute<float>(
  float * dst, const float * src, int * src_size, int * permute, int src_dim, cudaStream_t stream);
template void memcpyPermute<half>(
  half * dst, const half * src, int * src_size, int * permute, int src_dim, cudaStream_t stream);

cudnnStatus_t convert_trt2cudnn_dtype(nvinfer1::DataType trt_dtype, cudnnDataType_t * cudnn_dtype)
{
  switch (trt_dtype) {
    case nvinfer1::DataType::kFLOAT:
      *cudnn_dtype = CUDNN_DATA_FLOAT;
      break;
    case nvinfer1::DataType::kHALF:
      *cudnn_dtype = CUDNN_DATA_HALF;
      break;
    default:
      return CUDNN_STATUS_BAD_PARAM;
  }
  return CUDNN_STATUS_SUCCESS;
}

template <>
cublasStatus_t cublasGemmWrap<float>(
  cublasHandle_t handle, cublasOperation_t transa, cublasOperation_t transb, int m, int n, int k,
  const float * alpha, const float * A, int lda, const float * B, int ldb, const float * beta,
  float * C, int ldc, cublasGemmAlgo_t algo)
{
  return cublasGemmEx(
    handle, transa, transb, m, n, k, alpha, A, CUDA_R_32F, lda, B, CUDA_R_32F, ldb, beta, C,
    CUDA_R_32F, ldc, CUDA_R_32F, algo);
}

template <>
cublasStatus_t cublasGemmWrap<half>(
  cublasHandle_t handle, cublasOperation_t transa, cublasOperation_t transb, int m, int n, int k,
  const half * alpha, const half * A, int lda, const half * B, int ldb, const half * beta, half * C,
  int ldc, cublasGemmAlgo_t algo)
{
  return cublasGemmEx(
    handle, transa, transb, m, n, k, alpha, A, CUDA_R_16F, lda, B, CUDA_R_16F, ldb, beta, C,
    CUDA_R_16F, ldc, CUDA_R_16F, algo);
}

cublasStatus_t cublasGemmWrap_int8(
  cublasHandle_t handle, cublasOperation_t transa, cublasOperation_t transb, int m, int n, int k,
  const int32_t * alpha, const int8_t * A, int lda, const int8_t * B, int ldb, const int32_t * beta,
  int32_t * C, int ldc, cublasGemmAlgo_t algo)
{
  return cublasGemmEx(
    handle, transa, transb, m, n, k, alpha, A, CUDA_R_8I, lda, B, CUDA_R_8I, ldb, beta, C,
    CUDA_R_32I, ldc, CUDA_R_32I, algo);
}

template <>
cublasStatus_t cublasGemmBatchedWrap<float>(
  cublasHandle_t handle, cublasOperation_t transa, cublasOperation_t transb, int m, int n, int k,
  const float * alpha, const float * const Aarray[], int lda, const float * const Barray[], int ldb,
  const float * beta, float * const Carray[], int ldc, int batchCount, cublasGemmAlgo_t algo)
{
  return cublasGemmBatchedEx(
    handle, transa, transb, m, n, k, alpha, (const void * const *)Aarray, CUDA_R_32F, lda,
    (const void * const *)Barray, CUDA_R_32F, ldb, beta, (void * const *)Carray, CUDA_R_32F, ldc,
    batchCount, CUDA_R_32F, algo);
}

template <>
cublasStatus_t cublasGemmBatchedWrap<half>(
  cublasHandle_t handle, cublasOperation_t transa, cublasOperation_t transb, int m, int n, int k,
  const half * alpha, const half * const Aarray[], int lda, const half * const Barray[], int ldb,
  const half * beta, half * const Carray[], int ldc, int batchCount, cublasGemmAlgo_t algo)
{
  return cublasGemmBatchedEx(
    handle, transa, transb, m, n, k, alpha, (const void * const *)Aarray, CUDA_R_16F, lda,
    (const void * const *)Barray, CUDA_R_16F, ldb, beta, (void * const *)Carray, CUDA_R_16F, ldc,
    batchCount, CUDA_R_16F, algo);
}

cublasStatus_t cublasGemmBatchedWrap_int8(
  cublasHandle_t handle, cublasOperation_t transa, cublasOperation_t transb, int m, int n, int k,
  const int32_t * alpha, const int8_t * const Aarray[], int lda, const int8_t * const Barray[],
  int ldb, const int32_t * beta, int32_t * const Carray[], int ldc, int batchCount,
  cublasGemmAlgo_t algo)
{
  return cublasGemmBatchedEx(
    handle, transa, transb, m, n, k, alpha, (const void * const *)Aarray, CUDA_R_8I, lda,
    (const void * const *)Barray, CUDA_R_8I, ldb, beta, (void * const *)Carray, CUDA_R_32I, ldc,
    batchCount, CUDA_R_32I, algo);
}

template <>
cublasStatus_t cublasGemmStridedBatchedWrap<float>(
  cublasHandle_t handle, cublasOperation_t transa, cublasOperation_t transb, int m, int n, int k,
  const float * alpha, const float * A, int lda, long long int strideA, const float * B, int ldb,
  long long int strideB, const float * beta, float * C, int ldc, long long int strideC,
  int batchCount, cublasGemmAlgo_t algo)
{
  return cublasGemmStridedBatchedEx(
    handle, transa, transb, m, n, k, alpha, A, CUDA_R_32F, lda, strideA, B, CUDA_R_32F, ldb,
    strideB, beta, C, CUDA_R_32F, ldc, strideC, batchCount, CUDA_R_32F, algo);
}

template <>
cublasStatus_t cublasGemmStridedBatchedWrap<half>(
  cublasHandle_t handle, cublasOperation_t transa, cublasOperation_t transb, int m, int n, int k,
  const half * alpha, const half * A, int lda, long long int strideA, const half * B, int ldb,
  long long int strideB, const half * beta, half * C, int ldc, long long int strideC,
  int batchCount, cublasGemmAlgo_t algo)
{
  return cublasGemmStridedBatchedEx(
    handle, transa, transb, m, n, k, alpha, A, CUDA_R_16F, lda, strideA, B, CUDA_R_16F, ldb,
    strideB, beta, C, CUDA_R_16F, ldc, strideC, batchCount, CUDA_R_16F, algo);
}

cublasStatus_t cublasGemmStridedBatchedWrap_int8(
  cublasHandle_t handle, cublasOperation_t transa, cublasOperation_t transb, int m, int n, int k,
  const int32_t * alpha, const int8_t * A, int lda, long long int strideA, const int8_t * B,
  int ldb, long long int strideB, const int32_t * beta, const int32_t * C, int ldc,
  long long int strideC, int batchCount, cublasGemmAlgo_t algo)
{
  return cublasGemmStridedBatchedEx(
    handle, transa, transb, m, n, k, alpha, A, CUDA_R_8I, lda, strideA, B, CUDA_R_8I, ldb, strideB,
    beta, (void *)C, CUDA_R_32I, ldc, strideC, batchCount, CUDA_R_32I, algo);
}
