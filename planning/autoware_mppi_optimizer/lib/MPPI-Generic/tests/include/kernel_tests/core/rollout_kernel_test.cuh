#pragma once

#ifndef KERNEL_TESTS_MPPI_CORE_MPPI_CORE_KERNEL_TEST_CUH_
#define KERNEL_TESTS_MPPI_CORE_MPPI_CORE_KERNEL_TEST_CUH_

#include <mppi/core/mppi_common.cuh>
#include <curand.h>
#include <vector>
#include <array>

// Declare some sizes for the kernel parameters

template <int BLOCKSIZE_Z = 1>
__global__ void loadGlobalToShared_KernelTest(float* x0_device, float* x_thread, float* xdot_thread, float* u_thread);

void launchGlobalToShared_KernelTest(const std::vector<float>& x0_host, std::vector<float>& x_thread_host,
                                     std::vector<float>& xdot_thread_host, std::vector<float>& u_thread_host);

void launchGlobalToShared_KernelTest_nom_act(
    const std::vector<float>& x0_host_act, std::vector<float>& x_thread_host_act,
    std::vector<float>& xdot_thread_host_act, std::vector<float>& u_thread_host_act,
    const std::vector<float>& x0_host_nom, std::vector<float>& x_thread_host_nom,
    std::vector<float>& xdot_thread_host_nom, std::vector<float>& u_thread_host_nom);

template <class DYN_T, class COST_T, class SAMPLER_T>
void launchRolloutKernel_nom_act(DYN_T* dynamics, COST_T* costs, SAMPLER_T* sampler, float dt, const int num_timesteps,
                                 const int num_rollouts, float lambda, float alpha, const std::vector<float>& x0,
                                 const std::vector<float>& nom_control_seq, std::vector<float>& trajectory_costs_act,
                                 std::vector<float>& trajectory_costs_nom, cudaStream_t stream = 0);

template <class COST_T>
__global__ void computeAndSaveCostAllRollouts_KernelTest(COST_T* cost, int state_dim, int num_rollouts,
                                                         float* running_costs, float* terminal_state,
                                                         float* cost_rollout_device);

template <class COST_T, int STATE_DIM, int NUM_ROLLOUTS>
void launchComputeAndSaveCostAllRollouts_KernelTest(COST_T& cost,
                                                    const std::array<float, NUM_ROLLOUTS>& cost_all_rollouts,
                                                    const std::array<float, STATE_DIM * NUM_ROLLOUTS>& terminal_states,
                                                    std::array<float, NUM_ROLLOUTS>& cost_compute);

template <class DYNAMICS_T, class COSTS_T, int NUM_ROLLOUTS, int BLOCKSIZE_X, int BLOCKSIZE_Y>
__global__ void autorallyRolloutKernel(int num_timesteps, float* state_d, float* U_d, float* du_d, float* nu_d,
                                       float* costs_d, DYNAMICS_T* dynamics_model, COSTS_T* mppi_costs, int opt_delay,
                                       float lambda, float alpha, float dt);

template <class DYNAMICS_T, class COSTS_T, int NUM_ROLLOUTS, int NUM_TIMESTEPS, int BLOCKSIZE_X, int BLOCKSIZE_Y>
void launchAutorallyRolloutKernelTest(
    DYNAMICS_T* dynamics, COSTS_T* costs, float dt, float lambda, float alpha,
    std::array<float, DYNAMICS_T::STATE_DIM> state_array,
    std::array<float, NUM_TIMESTEPS * DYNAMICS_T::CONTROL_DIM> control_array,
    std::array<float, NUM_TIMESTEPS * NUM_ROLLOUTS * DYNAMICS_T::CONTROL_DIM> control_noise_array,
    std::array<float, DYNAMICS_T::CONTROL_DIM> sigma_u, std::array<float, NUM_ROLLOUTS>& costs_out,
    std::array<float, NUM_TIMESTEPS * NUM_ROLLOUTS * DYNAMICS_T::CONTROL_DIM>& control_noise_out, int opt_delay,
    cudaStream_t stream);

#if __CUDACC__
#include "rollout_kernel_test.cu"
#endif

#endif  // !KERNEL_TESTS_MPPI_CORE_MPPI_CORE_KERNEL_TEST_CUH_
