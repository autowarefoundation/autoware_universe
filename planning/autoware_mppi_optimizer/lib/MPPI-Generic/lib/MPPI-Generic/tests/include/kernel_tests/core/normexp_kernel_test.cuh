#pragma once

#ifndef MPPI_GENERIC_KERNEL_TESTS_MPPI_CORE_NORMEXP_KERNEL_TEST_CUH_
#define MPPI_GENERIC_KERNEL_TESTS_MPPI_CORE_NORMEXP_KERNEL_TEST_CUH_

#include <mppi/core/mppi_common.cuh>
#include <array>

template <int NUM_ROLLOUTS>
void launchNormExp_KernelTest(std::array<float, NUM_ROLLOUTS>& trajectory_costs_host, float gamma, float baseline,
                              std::array<float, NUM_ROLLOUTS>& normalized_compute);

template <int NUM_ROLLOUTS, int BLOCKSIZE_X>
void launchGenericNormExpKernelTest(std::array<float, NUM_ROLLOUTS> trajectory_costs_host, float gamma, float baseline,
                                    std::array<float, NUM_ROLLOUTS>& normalized_compute);

template <int NUM_ROLLOUTS, int BLOCKSIZE_X, int BLOCKSIZE_Y>
void launchAutorallyNormExpKernelTest(std::array<float, NUM_ROLLOUTS> trajectory_costs_host, float gamma,
                                      float baseline, std::array<float, NUM_ROLLOUTS>& normalized_compute);

#if __CUDACC__
#include "normexp_kernel_test.cu"
#endif

#endif  //! MPPI_GENERIC_KERNEL_TESTS_MPPI_CORE_NORMEXP_KERNEL_TEST_CUH_