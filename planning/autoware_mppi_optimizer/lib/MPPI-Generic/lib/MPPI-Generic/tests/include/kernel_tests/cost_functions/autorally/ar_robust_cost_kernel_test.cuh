//
// Created by jgibson37 on 2/7/20.
//

#ifndef MPPIGENERIC_AR_ROBUST_COST_KERNEL_TEST_CUH
#define MPPIGENERIC_AR_ROBUST_COST_KERNEL_TEST_CUH

#include "ar_standard_cost_kernel_test.cuh"

template <class CLASS_T>
__global__ void getCostmapCostTestKernel(CLASS_T* cost, float* test_xu, float* cost_results, int num_points);

template <class CLASS_T>
void launchGetCostmapCostTestKernel(CLASS_T& cost, std::vector<std::array<float, 9>>& test_xu,
                                    std::vector<float>& cost_results);

#include "ar_robust_cost_kernel_test.cu"

#endif  // MPPIGENERIC_AR_ROBUST_COST_KERNEL_TEST_CUH
