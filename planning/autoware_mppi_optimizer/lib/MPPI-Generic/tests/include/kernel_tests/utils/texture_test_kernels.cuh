//
// Created by jason on 1/9/22.
//

#ifndef MPPIGENERIC_TEXTURE_TEST_KERNELS_CUH
#define MPPIGENERIC_TEXTURE_TEST_KERNELS_CUH

#include <mppi/utils/texture_helpers/two_d_texture_helper.cuh>

template <class TEX_T, class DATA_T>
__global__ void textureTestKernel(TEX_T& tex, DATA_T* test_results, float4* test_indexes, int num_points)
{
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  // printf("\nEntering the kernel!\n");
  // printf("The thread id is: %i\n", tid);
  if (tid < num_points)
  {
    // printf("thread ia: %i went to check texture at index %i, %i\n", tid, test_indexes[tid].x, test_indexes[tid].y);

    // query texture
    float3 query_point = make_float3(test_indexes[tid].x, test_indexes[tid].y, test_indexes[tid].z);
    int index = round(test_indexes[tid].w);
    test_results[tid] = tex.queryTexture(index, query_point);
    // printf("query at %d %f,%f,%f = %f, %f, %f, %f\n", index, query_point.x, query_point.y, query_point.z,
    //       test_results[tid].x, test_results[tid].y, test_results[tid].z, test_results[tid].w);
  }
}

template <class TEX_T, class DATA_T>
std::vector<DATA_T> getTextureAtPointsKernel(const TEX_T& helper, std::vector<float4>& query_points)
{
  std::vector<DATA_T> results;
  int num_test_points = query_points.size();
  results.resize(num_test_points);

  float4* tex_results_d;
  float4* tex_query_points_d;
  HANDLE_ERROR(cudaMalloc((void**)&tex_results_d, sizeof(DATA_T) * num_test_points))
  HANDLE_ERROR(cudaMalloc((void**)&tex_query_points_d, sizeof(float4) * num_test_points))

  HANDLE_ERROR(
      cudaMemcpy(tex_query_points_d, query_points.data(), sizeof(float4) * num_test_points, cudaMemcpyHostToDevice));

  dim3 threadsPerBlock(num_test_points, 1);
  dim3 numBlocks(1, 1);
  textureTestKernel<<<numBlocks, threadsPerBlock>>>(*helper.ptr_d_, tex_results_d, tex_query_points_d, num_test_points);
  CudaCheckError();
  cudaDeviceSynchronize();

  HANDLE_ERROR(cudaMemcpy(results.data(), tex_results_d, sizeof(DATA_T) * num_test_points, cudaMemcpyDeviceToHost));

  cudaDeviceSynchronize();

  cudaFree(tex_results_d);
  cudaFree(tex_query_points_d);
  return results;
}

template <class TEX_T, class DATA_T>
__global__ void textureAtMapPoseTestKernel(TEX_T& tex, DATA_T* test_results, float4* test_indexes, int num_points)
{
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid < num_points)
  {
    // query texture
    float3 query_point = make_float3(test_indexes[tid].x, test_indexes[tid].y, test_indexes[tid].z);
    int index = round(test_indexes[tid].w);
    test_results[tid] = tex.queryTextureAtMapPose(index, query_point);
    // printf("query at %d %f,%f,%f = %f, %f, %f, %f\n", index, query_point.x, query_point.y, query_point.z,
    //       test_results[tid].x, test_results[tid].y, test_results[tid].z, test_results[tid].w);
  }
}

template <class TEX_T, class DATA_T>
std::vector<DATA_T> getTextureAtMapPointsKernel(const TEX_T& helper, std::vector<float4>& query_points)
{
  std::vector<DATA_T> results;
  int num_test_points = query_points.size();
  results.resize(num_test_points);

  float4* tex_results_d;
  float4* tex_query_points_d;
  HANDLE_ERROR(cudaMalloc((void**)&tex_results_d, sizeof(DATA_T) * num_test_points))
  HANDLE_ERROR(cudaMalloc((void**)&tex_query_points_d, sizeof(float4) * num_test_points))

  HANDLE_ERROR(
      cudaMemcpy(tex_query_points_d, query_points.data(), sizeof(float4) * num_test_points, cudaMemcpyHostToDevice));

  dim3 threadsPerBlock(num_test_points, 1);
  dim3 numBlocks(1, 1);
  textureAtMapPoseTestKernel<<<numBlocks, threadsPerBlock>>>(*helper.ptr_d_, tex_results_d, tex_query_points_d,
                                                             num_test_points);
  CudaCheckError();
  cudaDeviceSynchronize();

  HANDLE_ERROR(cudaMemcpy(results.data(), tex_results_d, sizeof(DATA_T) * num_test_points, cudaMemcpyDeviceToHost));

  cudaDeviceSynchronize();

  cudaFree(tex_results_d);
  cudaFree(tex_query_points_d);
  return results;
}

template <class TEX_T, class DATA_T>
__global__ void textureAtWorldPoseTestKernel(TEX_T& tex, DATA_T* test_results, float4* test_indexes, int num_points)
{
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid < num_points)
  {
    // query texture
    float3 query_point = make_float3(test_indexes[tid].x, test_indexes[tid].y, test_indexes[tid].z);
    int index = round(test_indexes[tid].w);
    test_results[tid] = tex.queryTextureAtWorldPose(index, query_point);
    // printf("query at %d %f,%f,%f = %f, %f, %f, %f\n", index, query_point.x, query_point.y, query_point.z,
    //       test_results[tid].x, test_results[tid].y, test_results[tid].z, test_results[tid].w);
  }
}

template <class TEX_T, class DATA_T>
std::vector<DATA_T> getTextureAtWorldPointsKernel(const TEX_T& helper, std::vector<float4>& query_points)
{
  std::vector<DATA_T> results;
  int num_test_points = query_points.size();
  results.resize(num_test_points);

  float4* tex_results_d;
  float4* tex_query_points_d;
  HANDLE_ERROR(cudaMalloc((void**)&tex_results_d, sizeof(DATA_T) * num_test_points))
  HANDLE_ERROR(cudaMalloc((void**)&tex_query_points_d, sizeof(float4) * num_test_points))

  HANDLE_ERROR(
      cudaMemcpy(tex_query_points_d, query_points.data(), sizeof(float4) * num_test_points, cudaMemcpyHostToDevice));

  dim3 threadsPerBlock(num_test_points, 1);
  dim3 numBlocks(1, 1);
  textureAtWorldPoseTestKernel<<<numBlocks, threadsPerBlock>>>(*helper.ptr_d_, tex_results_d, tex_query_points_d,
                                                               num_test_points);
  CudaCheckError();
  cudaDeviceSynchronize();

  HANDLE_ERROR(cudaMemcpy(results.data(), tex_results_d, sizeof(DATA_T) * num_test_points, cudaMemcpyDeviceToHost));

  cudaDeviceSynchronize();

  cudaFree(tex_results_d);
  cudaFree(tex_query_points_d);
  return results;
}

#if __CUDACC__
#include "texture_test_kernels.cuh"
#endif

#endif  // MPPIGENERIC_TEXTURE_TEST_KERNELS_CUH
