/**
 * Created by Bogdan Vlahov on 3/25/2023
 **/
#pragma once
#include <mppi/utils/math_utils.h>
#include <mppi/dynamics/dynamics.cuh>
#include <mppi/cost_functions/cost.cuh>

namespace mppi
{
namespace kernels
{
namespace rmppi
{
/**
 * Kernels Methods
 **/
template <class DYN_T, class SAMPLING_T>
__global__ void initEvalDynKernel(DYN_T* __restrict__ dynamics, SAMPLING_T* __restrict__ sampling, float dt,
                                  const int num_timesteps, const int num_rollouts, int samples_per_condition,
                                  const int* __restrict__ strides_d, const float* __restrict__ states_d,
                                  float* __restrict__ y_d);

template <class COST_T, class SAMPLING_T, bool COALESCE = false>
__global__ void initEvalCostKernel(COST_T* __restrict__ costs, SAMPLING_T* __restrict__ sampling, float dt,
                                   const int num_timesteps, const int num_rollouts, float lambda, float alpha,
                                   const int samples_per_condition, const int* __restrict__ strides_d,
                                   const float* __restrict__ y_d, float* __restrict__ trajectory_costs_d);

template <class DYN_T, class FB_T, class SAMPLING_T, int NOMINAL_STATE_IDX = 0>
__global__ void rolloutRMPPIDynamicsKernel(DYN_T* __restrict__ dynamics, FB_T* __restrict__ fb_controller,
                                           SAMPLING_T* __restrict__ sampling, float dt, const int num_timesteps,
                                           const int num_rollouts, const float* __restrict__ init_x_d,
                                           float* __restrict__ y_d);

template <class COST_T, class DYN_T, class SAMPLING_T, class FB_T, int NOMINAL_STATE_IDX = 0, bool COALESCE = false>
__global__ void rolloutRMPPICostKernel(COST_T* __restrict__ costs, DYN_T* __restrict__ dynamics,
                                       FB_T* __restrict__ fb_controller, SAMPLING_T* __restrict__ sampling, float dt,
                                       const int num_timesteps, const int num_rollouts, float lambda, float alpha,
                                       float value_func_threshold, const float* __restrict__ init_x_d,
                                       const float* __restrict__ y_d, float* __restrict__ trajectory_costs_d);

template <class DYN_T, class COST_T, class SAMPLING_T>
__global__ void initEvalKernel(DYN_T* __restrict__ dynamics, COST_T* __restrict__ costs,
                               SAMPLING_T* __restrict__ sampling, float dt, const int num_timesteps,
                               const int num_rollouts, int samples_per_condition, float lambda, float alpha,
                               const int* __restrict__ strides_d, const float* __restrict__ states_d,
                               float* __restrict__ trajectory_costs_d);

template <class DYN_T, class COST_T, class FB_T, class SAMPLING_T, int NOMINAL_STATE_IDX = 0>
__global__ void rolloutRMPPIKernel(DYN_T* __restrict__ dynamics, COST_T* __restrict__ costs,
                                   FB_T* __restrict__ fb_controller, SAMPLING_T* __restrict__ sampling, float dt,
                                   const int num_timesteps, const int num_rollouts, float lambda, float alpha,
                                   float value_func_threshold, const float* __restrict__ init_x_d,
                                   float* __restrict__ trajectory_costs_d);
/**
 * Launch Methods
 **/
template <class DYN_T, class COST_T, typename SAMPLING_T, bool COALESCE = false>
void launchSplitInitEvalKernel(DYN_T* __restrict__ dynamics, COST_T* __restrict__ costs,
                               SAMPLING_T* __restrict__ sampling, float dt, const int num_timesteps,
                               const int num_rollouts, float lambda, float alpha, int samples_per_condition,
                               int* __restrict__ strides_d, float* __restrict__ init_x_d, float* __restrict__ y_d,
                               float* __restrict__ trajectory_costs, dim3 dimDynBlock, dim3 dimCostBlock,
                               cudaStream_t stream, bool synchronize = true);

template <class DYN_T, class COST_T, class SAMPLING_T, class FB_T, int NOMINAL_STATE_IDX = 0, bool COALESCE = false>
void launchSplitRMPPIRolloutKernel(DYN_T* __restrict__ dynamics, COST_T* __restrict__ costs,
                                   SAMPLING_T* __restrict__ sampling, FB_T* __restrict__ fb_controller, float dt,
                                   const int num_timesteps, const int num_rollouts, float lambda, float alpha,
                                   float value_func_threshold, float* __restrict__ init_x_d, float* __restrict__ y_d,
                                   float* __restrict__ trajectory_costs, dim3 dimDynBlock, dim3 dimCostBlock,
                                   cudaStream_t stream, bool synchronize = true);

template <class DYN_T, class COST_T, typename SAMPLING_T>
void launchInitEvalKernel(DYN_T* __restrict__ dynamics, COST_T* __restrict__ costs, SAMPLING_T* __restrict__ sampling,
                          float dt, const int num_timesteps, const int num_rollouts, float lambda, float alpha,
                          int samples_per_condition, int* __restrict__ strides_d, float* __restrict__ init_x_d,
                          float* __restrict__ trajectory_costs, dim3 dimBlock, cudaStream_t stream, bool synchronize = true);

template <class DYN_T, class COST_T, class SAMPLING_T, class FB_T, int NOMINAL_STATE_IDX = 0>
void launchRMPPIRolloutKernel(DYN_T* __restrict__ dynamics, COST_T* __restrict__ costs,
                              SAMPLING_T* __restrict__ sampling, FB_T* __restrict__ fb_controller, float dt,
                              const int num_timesteps, const int num_rollouts, float lambda, float alpha,
                              float value_func_threshold, float* __restrict__ init_x_d,
                              float* __restrict__ trajectory_costs, dim3 dimBlock, cudaStream_t stream,
                              bool synchronize = true);

/**
 * Device-only Kernel Helper Methods
 **/
__device__ void multiCostArrayReduction(float* running_cost, float* running_cost_extra, const int start_size,
                                        const int index, const int step, const bool catch_condition,
                                        const int stride = 1);

/**
 * Shared Memory Calculators for various kernels
 */
template <class DYN_T, class SAMPLER_T>
unsigned calcEvalDynKernelSharedMemSize(const DYN_T* dynamics, const SAMPLER_T* sampler, dim3& dimBlock);

template <class COST_T, class SAMPLER_T>
unsigned calcEvalCostKernelSharedMemSize(const COST_T* cost, const SAMPLER_T* sampler, const int num_rollouts,
                                         const int samples_per_condition, dim3& dimBlock);

template <class DYN_T, class COST_T, class SAMPLER_T>
unsigned calcEvalCombinedKernelSharedMemSize(const DYN_T* dynamics, const COST_T* cost, const SAMPLER_T* sampler,
                                             const int num_rollouts, const int samples_per_condition, dim3& dimBlock);

template <class DYN_T, class SAMPLER_T, class FB_T>
unsigned calcRMPPIDynKernelSharedMemSize(const DYN_T* dynamics, const SAMPLER_T* sampler, const FB_T* fb_controller,
                                         dim3& dimBlock);

template <class COST_T, class SAMPLER_T, class FB_T>
unsigned calcRMPPICostKernelSharedMemSize(const COST_T* cost, const SAMPLER_T* sampler, const FB_T* fb_controller,
                                          dim3& dimBlock);

template <class DYN_T, class COST_T, class SAMPLER_T, class FB_T>
unsigned calcRMPPICombinedKernelSharedMemSize(const DYN_T* dynamics, const COST_T* cost, const SAMPLER_T* sampler,
                                              const FB_T* fb_controller, dim3& dimBlock);
}  // namespace rmppi
}  // namespace kernels
}  // namespace mppi

#if __CUDACC__
#include "rmppi_kernels.cu"
#endif
