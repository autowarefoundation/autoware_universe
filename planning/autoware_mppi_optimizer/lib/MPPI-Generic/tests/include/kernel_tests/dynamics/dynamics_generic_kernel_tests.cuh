#ifndef DYNAMICS_GENERIC_KERNEL_TESTS_CUH_
#define DYNAMICS_GENERIC_KERNEL_TESTS_CUH_

#include <array>
#include <vector>
#include <mppi/utils/math_utils.h>

template <typename CLASS_T, typename PARAMS_T>
__global__ void parameterTestKernel(CLASS_T* class_t, PARAMS_T& params);
template <typename CLASS_T, typename PARAMS_T>
void launchParameterTestKernel(CLASS_T& class_t, PARAMS_T& params);

template <typename DYNAMICS_T, int C_DIM>
__global__ void controlRangesTestKernel(DYNAMICS_T* dynamics, float2* control_rngs);
template <typename DYNAMICS_T, int C_DIM>
void launchControlRangesTestKernel(DYNAMICS_T& dynamics, std::array<float2, C_DIM>& control_rngs);

template <typename DYNAMICS_T, int S_DIM, int C_DIM>
__global__ void enforceConstraintTestKernel(DYNAMICS_T* dynamics, float* state, float* control, int num);
template <typename DYNAMICS_T, int S_DIM, int C_DIM>
void launchEnforceConstraintTestKernel(DYNAMICS_T& dynamics, std::vector<std::array<float, S_DIM>>& state,
                                       std::vector<std::array<float, C_DIM>>& control, int dim_y);

template <class DYNAMICS_T, int S_DIM>
__global__ void computeKinematicsTestKernel(DYNAMICS_T* model, float* state, float* state_der, int num);
template <class DYNAMICS_T, int S_DIM>
void launchComputeKinematicsTestKernel(DYNAMICS_T& dynamics, std::vector<std::array<float, S_DIM>>& state,
                                       std::vector<std::array<float, S_DIM>>& state_der, int dim_y);

template <class DYNAMICS_T, int S_DIM, int C_DIM, int BLOCKDIM_X>
__global__ void computeDynamicsTestKernel(DYNAMICS_T* model, float* state, float* control, float* state_der, int count);
template <class DYNAMICS_T, int S_DIM, int C_DIM, int BLOCKDIM_X = 32>
void launchComputeDynamicsTestKernel(DYNAMICS_T& dynamics, std::vector<std::array<float, S_DIM>>& state,
                                     std::vector<std::array<float, C_DIM>>& control,
                                     std::vector<std::array<float, S_DIM>>& state_der, int dim_y);

template <typename DYNAMICS_T, int S_DIM>
__global__ void updateStateTestKernel(DYNAMICS_T* dynamics, float* state, float* state_der, float dt, int num);
template <typename DYNAMICS_T, int S_DIM>
void launchUpdateStateTestKernel(DYNAMICS_T& dynamics, std::vector<std::array<float, S_DIM>>& state,
                                 std::vector<std::array<float, S_DIM>>& state_der, float dt, int dim_y);

template <typename DYNAMICS_T, int S_DIM, int C_DIM, int BLOCKDIM_X>
__global__ void computeStateDerivTestKernel(DYNAMICS_T* dynamics, float* state, float* control, float* state_der,
                                            int num);
template <typename DYNAMICS_T, int S_DIM, int C_DIM, int BLOCKDIM_X = 32>
void launchComputeStateDerivTestKernel(DYNAMICS_T& dynamics, std::vector<std::array<float, S_DIM>>& state,
                                       std::vector<std::array<float, C_DIM>>& control,
                                       std::vector<std::array<float, S_DIM>>& state_der, int dim_y);

template <class DYN_T>
__global__ void getSharedMemorySizesKernel(DYN_T* __restrict__ dynamics, int* __restrict__ output_d);
template <typename DYN_T>
void launchGetSharedMemorySizesKernel(DYN_T& dynamics, int shared_mem_sizes[2]);

template <class DYN_T>
void checkGPUComputationStep(DYN_T& dynamics, float dt, int max_y_dim, int x_dim,
                             typename DYN_T::buffer_trajectory buffer, double tol = 1.0e-5);

#if __CUDACC__
#include "dynamics_generic_kernel_tests.cu"
#endif

#endif  // DYNAMICS_GENERIC_KERNEL_TESTS_CUH_
