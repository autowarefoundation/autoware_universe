#pragma once

#include <mppi/utils/math_utils.h>
#include <mppi/core/mppi_common.cuh>
#include <mppi/sampling_distributions/gaussian/gaussian.cuh>

#include <array>
#include <iostream>
#include <vector>

namespace mp1 = ::mppi::p1;
#define USE_ROLLOUT_COST_KERNEL

template <class COST_T>
__global__ void computeRunningCostTestKernel(COST_T* __restrict__ cost, const float* __restrict__ y_d,
                                             const float* __restrict__ u_d, int num_rollouts, int num_timesteps,
                                             float dt, float* __restrict__ output_cost);

template <class COST_T>
__global__ void computeTerminalCostTestKernel(COST_T* __restrict__ cost, const float* __restrict__ y_d,
                                              int num_rollouts, float dt, float* __restrict__ output_cost);

template <class COST_T>
void launchRunningCostTestKernel(COST_T& cost, std::vector<std::array<float, COST_T::OUTPUT_DIM>>& y,
                                 std::vector<std::array<float, COST_T::CONTROL_DIM>>& u, int num_rollouts,
                                 int num_timesteps, float dt, int dim_y, std::vector<float>& output_costs);

template <class COST_T, class SAMPLING_T>
void launchRolloutCostKernel(COST_T& cost, SAMPLING_T& sampler, std::vector<std::array<float, COST_T::OUTPUT_DIM>>& y,
                             int num_rollouts, int num_timesteps, float dt, int dim_y,
                             std::vector<float>& output_costs);

template <class COST_T>
void launchTerminalCostTestKernel(COST_T& cost, std::vector<std::array<float, COST_T::OUTPUT_DIM>>& y, float dt,
                                  int dim_x, int dim_y, std::vector<float>& output_costs);

template <class COST_T>
void checkGPURolloutCost(COST_T& cost, float dt);

#ifdef __CUDACC__
#include "cost_generic_kernel_tests.cu"
#endif
