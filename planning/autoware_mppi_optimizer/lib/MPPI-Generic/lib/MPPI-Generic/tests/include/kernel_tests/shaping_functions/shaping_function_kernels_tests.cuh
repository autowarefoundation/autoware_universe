#ifndef MPPIGENERIC_SHAPING_FUNCTION_KERNEL_TEST_CUH
#define MPPIGENERIC_SHAPING_FUNCTION_KERNEL_TEST_CUH

#include <mppi/core/mppi_common.cuh>
#include <mppi/utils/managed.cuh>

template <class CLASS_T, int NUM_ROLLOUTS>
void launchShapingFunction_KernelTest(typename CLASS_T::cost_traj& trajectory_costs_host, CLASS_T& shape_function,
                                      float baseline, std::array<float, NUM_ROLLOUTS>& normalized_compute,
                                      cudaStream_t stream = nullptr);

template <class CLASS_T, int NUM_ROLLOUTS, int BDIM_X>
void launchShapingFunction_KernelTest(std::array<float, NUM_ROLLOUTS>& trajectory_costs_host, CLASS_T& shape_function,
                                      float baseline, std::array<float, NUM_ROLLOUTS>& normalized_compute);

#if __CUDACC__
#include "shaping_function_kernels_tests.cu"
#endif

#endif  // MPPIGENERIC_SHAPING_FUNCTION_KERNEL_TEST_CUH
