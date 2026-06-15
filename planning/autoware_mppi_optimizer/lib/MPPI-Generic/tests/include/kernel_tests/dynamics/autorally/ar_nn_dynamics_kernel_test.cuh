#ifndef AR_NN_DYNAMICS_KERNEL_TEST_CUH
#define AR_NN_DYNAMICS_KERNEL_TEST_CUH

#include <array>
#include <mppi/dynamics/autorally/ar_nn_model.cuh>
#include "../dynamics_generic_kernel_tests.cuh"
#include <cuda_runtime.h>

template <class NETWORK_T, int THETA_SIZE, int STRIDE_SIZE, int NUM_LAYERS>
void launchParameterCheckTestKernel(NETWORK_T& model, std::array<float, THETA_SIZE>& theta,
                                    std::array<int, STRIDE_SIZE>& stride, std::array<int, NUM_LAYERS>& net_structure);
template <class NETWORK_T, int THETA_SIZE, int STRIDE_SIZE, int NUM_LAYERS>
__global__ void parameterCheckTestKernel(NETWORK_T* model, float* theta, int* stride, int* net_structure);

template <class NETWORK_T, int S_DIM, int C_DIM>
void launchFullARNNTestKernel(NETWORK_T& model, std::vector<std::array<float, S_DIM>>& state,
                              std::vector<std::array<float, C_DIM>>& control,
                              std::vector<std::array<float, S_DIM>>& state_der, float dt, int dim_y);
template <class NETWORK_T, int S_DIM, int C_DIM>
__global__ void fullARNNTestKernel(NETWORK_T* model, float* state, float* control, float* state_der, float dt);

#include "ar_nn_dynamics_kernel_test.cu"

#endif  // AR_NN_DYNAMICS_KERNEL_TEST_CUH
