//
// Created by Manan Gandhi on 12/2/19.
//

#ifndef MPPIGENERIC_MPPI_COMMON_CUH
#define MPPIGENERIC_MPPI_COMMON_CUH

#include <mppi/utils/math_utils.h>

namespace mppi
{
namespace kernels
{
/*******************************************************************************************************************
 * Kernel functions
 *******************************************************************************************************************/
template <class COST_T, class SAMPLING_T, bool COALESCE = true>
__global__ void rolloutCostKernel(COST_T* __restrict__ costs, SAMPLING_T* __restrict__ sampling, float dt,
                                  const int num_timesteps, const int num_rollouts, float lambda, float alpha,
                                  const float* __restrict__ y_d, float* __restrict__ trajectory_costs_d);

template <class DYN_T, class COST_T, class SAMPLING_T>
__global__ void rolloutKernel(DYN_T* __restrict__ dynamics, SAMPLING_T* __restrict__ sampling,
                              COST_T* __restrict__ costs, float dt, const int num_timesteps, const int num_rollouts,
                              const float* __restrict__ init_x_d, float lambda, float alpha,
                              float* __restrict__ trajectory_costs_d);

template <class DYN_T, class SAMPLING_T>
__global__ void rolloutDynamicsKernel(DYN_T* __restrict__ dynamics, SAMPLING_T* __restrict__ sampling, float dt,
                                      const int num_timesteps, const int num_rollouts,
                                      const float* __restrict__ init_x_d, float* __restrict__ y_d);

template <class DYN_T, class COST_T, class SAMPLING_T>
__global__ void visualizeKernel(DYN_T* __restrict__ dynamics, SAMPLING_T* __restrict__ sampling,
                                COST_T* __restrict__ costs, float dt, const int num_timesteps, const int num_rollouts,
                                const float* __restrict__ init_x_d, float lambda, float alpha, float* __restrict__ y_d,
                                float* __restrict__ cost_traj_d, int* __restrict__ crash_status_d);

template <class COST_T, class SAMPLING_T, bool COALESCE = true>
__global__ void visualizeCostKernel(COST_T* __restrict__ costs, SAMPLING_T* __restrict__ sampling, float dt,
                                    const int num_timesteps, const int num_rollouts, const float lambda, float alpha,
                                    const float* __restrict__ y_d, float* __restrict__ cost_traj_d,
                                    int* __restrict__ crash_status_d);

template <int CONTROL_DIM>
__global__ void weightedReductionKernel(const float* __restrict__ exp_costs_d, const float* __restrict__ du_d,
                                        float* __restrict__ new_u_d, const float normalizer, const int num_timesteps,
                                        const int num_rollouts, const int sum_stride);

// Norm Exponential Kernel
__global__ void normExpKernel(int num_rollouts, float* trajectory_costs_d, float gamma, float baseline);
// Tsallis Kernel
__global__ void TsallisKernel(int num_rollouts, float* trajectory_costs_d, float gamma, float r, float baseline);

/*******************************************************************************************************************
 * RolloutKernel Helpers
 *******************************************************************************************************************/
/*
 * loadGlobalToShared
 * Copy global memory into shared memory
 *
 * Args:
 * state_dim: Number of states, defined in DYN_T
 * control_dim: Number of controls, defined in DYN_T
 * num_rollouts: Total number of rollouts
 * blocksize_y: Y dimension of each block of threads
 * global_idx: Current rollout index.
 * thread_idy: Current y index of block dimension.
 * thread_idz: Current z index of block dimension.
 * x0_device: initial condition in device memory
 * x_thread: state in shared memory
 * xdot_thread: state_dot in shared memory
 * u_thread: control / perturbed control in shared memory
 *
 */
template <int STATE_DIM, int CONTROL_DIM>
__device__ void loadGlobalToShared(const int num_rollouts, const int blocksize_y, const int global_idx,
                                   const int thread_idy, const int thread_idz, const float* __restrict__ x_device,
                                   float* __restrict__ x_thread, float* __restrict__ xdot_thread,
                                   float* __restrict__ u_thread);

/**
 * @brief Calculate the terminal cost and add it to the trajectories' overall cost
 *
 * @tparam COST_T - Cost Function class
 * @param num_rollouts
 * @param num_timesteps
 * @param global_idx - sample trajectory index
 * @param costs - GPU version of cost function
 * @param x_thread - terminal state x
 * @param running_cost - current cost of the sample trajectory
 * @param theta_c - shared memory for the cost function
 * @param cost_rollouts_device - global memory array storing the cost of each sample
 *
 * @return
 */
template <class COST_T>
__device__ void computeAndSaveCost(int num_rollouts, int num_timesteps, int global_idx, COST_T* costs, float* x_thread,
                                   float running_cost, float* theta_c, float* cost_rollouts_device);

/**
 * @brief conduct a warp reduction of addition s[tid * stride] += s[(tid + BLOCKSIZE) * stride]
 *
 * @tparam BLOCKSIZE - how many threads are doing the reduction
 * @param sdata - float array to do the reduction on
 * @param tid - current thread index
 * @param stride - how spaced out the summations should be
 *
 * @return
 */
template <int BLOCKSIZE>
__device__ void warpReduceAdd(volatile float* sdata, const int tid, const int stride = 1);

/**
 * @brief conduct a sum of floats in array through a GPU reduction algorithm
 *
 * @param running_cost - array of floats to be summed
 * @param start_size - number of items to be summed
 * @param index - GPU thread index
 * @param step - GPU step to avoid overlap of threads
 * @param catch_condition - when to stop summation
 * @param stride - how far apart the desired floats are in the array
 *
 * @return
 */
__device__ inline void costArrayReduction(float* running_cost, const int start_size, const int index, const int step,
                                          const bool catch_condition, const int stride = 1);

// Norm Exp Kernel Helpers
__device__ __host__ inline void normExpTransform(const int num_rollouts, float* __restrict__ trajectory_costs_d,
                                                 const float lambda_inv, const float baseline, const int global_idx,
                                                 const int rollout_idx_step);
// Tsallis Kernel Helpers
__device__ __host__ inline void TsallisTransform(const int num_rollouts, float* __restrict__ trajectory_costs_d,
                                                 const float gamma, float r, const float baseline, const int global_idx,
                                                 const int rollout_idx_step);
float computeBaselineCost(float* cost_rollouts_host, int num_rollouts);

float computeNormalizer(float* cost_rollouts_host, int num_rollouts);

float constructBestWeights(float* cost_rollouts_host, int num_rollouts);

int computeBestIndex(float* cost_rollouts_host, int num_rollouts);

/**
 * Calculates the free energy mean and variance from the different
 * cost trajectories after normExpKernel
 * Inputs:
 *  cost_rollouts_host - sampled cost trajectories
 *  num_rollouts - the number of sampled cost trajectories
 *  lambda - the lambda term from the definition of free energy
 *  baseline - minimum cost trajectory
 * Outputs:
 *  free_energy - the free energy of the samples
 *  free_energy_var - the variance of the free energy calculation
 */
void computeFreeEnergy(float& free_energy, float& free_energy_var, float& free_energy_modified,
                       float* cost_rollouts_host, int num_rollouts, float baseline, float lambda = 1.0);

/*******************************************************************************************************************
 * Weighted Reduction Kernel Helpers
 *******************************************************************************************************************/
/**
 * @brief set controls to zero
 *
 * @param control_dim
 * @param thread_idx - threadIdx.x
 * @param u - memory for control array
 * @param u_intermediate - shared memory for control array
 *
 * @return
 */
__device__ void setInitialControlToZero(int control_dim, int thread_idx, float* __restrict__ u,
                                        float* __restrict__ u_intermediate);

/**
 * @brief calculated the weighted sum of the controls
 *
 * @param num_rollouts
 * @param num_timesteps
 * @param sum_stride - how many summations to do in a single thread
 * @param thread_idx - threadIdx.x
 * @param block_idx - blockIdx.x
 * @param control_dim
 * @param exp_costs_d - global memory of the weights
 * @param normalizer - sum of all the weights to use as a normalizing term
 * @param du_d - global memory of all sampled controls
 * @param u - local memory to store a control from a single time
 * @param u_intermediate - shared memory containing the weighted sum
 *
 * @return
 */
__device__ void strideControlWeightReduction(const int num_rollouts, const int num_timesteps, const int sum_stride,
                                             const int thread_idx, const int block_idx, const int control_dim,
                                             const float* __restrict__ exp_costs_d, const float normalizer,
                                             const float* __restrict__ du_d, float* __restrict__ u,
                                             float* __restrict__ u_intermediate);

__device__ void rolloutWeightReductionAndSaveControl(int thread_idx, int block_idx, int num_rollouts, int num_timesteps,
                                                     int control_dim, int sum_stride, float* u, float* u_intermediate,
                                                     float* du_new_d);

/**
 * Launch Kernel Methods
 **/
template <class DYN_T, class COST_T, typename SAMPLING_T, bool COALESCE = true>
void launchSplitRolloutKernel(DYN_T* __restrict__ dynamics, COST_T* __restrict__ costs,
                              SAMPLING_T* __restrict__ sampling, float dt, const int num_timesteps,
                              const int num_rollouts, float lambda, float alpha, float* __restrict__ init_x_d,
                              float* __restrict__ y_d, float* __restrict__ trajectory_costs, dim3 dimDynBlock,
                              dim3 dimCostBlock, cudaStream_t stream, bool synchronize = true);

template <class DYN_T, class COST_T, typename SAMPLING_T>
void launchRolloutKernel(DYN_T* __restrict__ dynamics, COST_T* __restrict__ costs, SAMPLING_T* __restrict__ sampling,
                         float dt, const int num_timesteps, const int num_rollouts, float lambda, float alpha,
                         float* __restrict__ init_x_d, float* __restrict__ trajectory_costs, dim3 dimBlock,
                         cudaStream_t stream, bool synchronize = true);

template <class COST_T, class SAMPLING_T, bool COALESCE = true>
void launchVisualizeCostKernel(COST_T* __restrict__ costs, SAMPLING_T* __restrict__ sampling, float dt,
                               const int num_timesteps, const int num_rollouts, float lambda, float alpha,
                               float* __restrict__ y_d, int* __restrict__ sampled_crash_status_d,
                               float* __restrict__ cost_traj_result, dim3 dimBlock, cudaStream_t stream,
                               bool synchronize = true);

template <class DYN_T, class COST_T, typename SAMPLING_T>
void launchVisualizeKernel(DYN_T* __restrict__ dynamics, COST_T* __restrict__ costs, SAMPLING_T* __restrict__ sampling,
                           float dt, const int num_timesteps, const int num_rollouts, float lambda, float alpha,
                           float* __restrict__ init_x_d, float* __restrict__ y_d, float* __restrict__ trajectory_costs,
                           int* __restrict__ crash_status_d, dim3 dimVisBlock, cudaStream_t stream,
                           bool synchronize = true);

template <int CONTROL_DIM>
void launchWeightedReductionKernel(const float* __restrict__ exp_costs_d, const float* __restrict__ du_d,
                                   float* __restrict__ new_u_d, const float normalizer, const int num_timesteps,
                                   const int num_rollouts, const int sum_stride, cudaStream_t stream,
                                   bool synchronize = true);

void launchNormExpKernel(int num_rollouts, int blocksize_x, float* trajectory_costs_d, float lambda_inv, float baseline,
                         cudaStream_t stream, bool synchronize = true);

void launchTsallisKernel(int num_rollouts, int blocksize_x, float* trajectory_costs_d, float gamma, float r,
                         float baseline, cudaStream_t stream, bool synchronize = true);

template <class DYN_T, int NUM_ROLLOUTS, int SUM_STRIDE>
void launchWeightedReductionKernel(float* exp_costs_d, float* du_d, float* du_new_d, float normalizer,
                                   int num_timesteps, cudaStream_t stream, bool synchronize = true);

/*******************************************************************************************************************
 * Shared Memory Calculators for various kernels
 *******************************************************************************************************************/
template <class DYN_T, class SAMPLER_T>
unsigned calcRolloutDynamicsKernelSharedMemSize(const DYN_T* dynamics, const SAMPLER_T* sampler, dim3& dimBlock);

template <class COST_T, class SAMPLER_T>
unsigned calcRolloutCostKernelSharedMemSize(const COST_T* cost, const SAMPLER_T* sampler, dim3& dimBlock);

template <class DYN_T, class COST_T, class SAMPLER_T>
unsigned calcRolloutCombinedKernelSharedMemSize(const DYN_T* dynamics, const COST_T* cost, const SAMPLER_T* sampler,
                                                dim3& dimBlock);

template <class DYN_T, class COST_T, class SAMPLER_T>
unsigned calcVisualizeKernelSharedMemSize(const DYN_T* dynamics, const COST_T* cost, const SAMPLER_T* sampler,
                                          const int& num_timesteps, dim3& dimBlock);

template <class COST_T, class SAMPLER_T>
unsigned calcVisCostKernelSharedMemSize(const COST_T* cost, const SAMPLER_T* sampler, const int& num_timesteps,
                                        dim3& dimBlock);

template <class T>
__host__ __device__ inline unsigned calcClassSharedMemSize(const T* class_ptr, const dim3& dimBlock);
}  // namespace kernels
}  // namespace mppi

#if __CUDACC__
#include "mppi_common.cu"
#endif

#endif  // MPPIGENERIC_MPPI_COMMON_CUH
