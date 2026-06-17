//
// Created by jason on 1/8/20.
//

#ifndef MPPIGENERIC_AR_STANDARD_COST_KERNEL_TEST_CUH
#define MPPIGENERIC_AR_STANDARD_COST_KERNEL_TEST_CUH

#include <mppi/cost_functions/autorally/ar_standard_cost.cuh>

template <typename COST_T, typename PARAMS_T>
__global__ void parameterTestKernel(const COST_T* cost, PARAMS_T& params, int& width, int& height);

template <typename COST_T, typename PARAMS_T>
void launchParameterTestKernel(const COST_T& cost, PARAMS_T& params, int& width, int& height);

void launchCheckCudaArray(std::vector<float4>& result_arr, cudaArray* array, int number);

__global__ void checkCudaArrayKernel(float4* result_arr, cudaArray* array, int number);

template <typename COST_T>
void launchTransformTestKernel(std::vector<float3>& result, const COST_T& cost);

template <typename COST_T>
__global__ void transformTestKernel(float3* results, COST_T* cost);

template <typename COST_T>
void launchTextureTestKernel(const COST_T& cost, std::vector<float4>& test_results, std::vector<float2>& test_indexes);

template <typename COST_T>
__global__ void textureTestKernel(const COST_T& cost, float4* test_results, float2* test_indexes, int num_points);

template <typename COST_T>
void launchTextureTransformTestKernel(const COST_T& cost, std::vector<float4>& test_results,
                                      std::vector<float2>& test_indexes);

template <typename COST_T>
__global__ void textureTransformTestKernel(COST_T& cost, float4* test_results, float2* test_indexes, int num_points);

template <typename COST_T>
void launchTrackCostTestKernel(const COST_T& cost, std::vector<float3>& test_indexes, std::vector<float>& cost_results,
                               std::vector<int>& crash_results);

template <typename COST_T>
__global__ void trackCostTestKernel(COST_T* cost, float3* test_indexes, int num_points, float* cost_results,
                                    int* crash_results);

template <typename COST_T>
void launchComputeCostTestKernel(const COST_T& cost, std::vector<std::array<float, 9>>& test_xu,
                                 std::vector<float>& cost_results, std::vector<int>& timesteps);

template <typename COST_T>
__global__ void computeCostTestKernel(COST_T& cost, float* test_xu, float* cost_results, int* timesteps,
                                      int num_points);

#include "ar_standard_cost_kernel_test.cu"

#endif  // MPPIGENERIC_AR_STANDARD_COST_KERNEL_TEST_CUH
