#pragma once

#ifndef KERNEL_TESTS_DYNAMICS_CARTPOLE_CARTPOLE_KERNEL_TEST_CUH_
#define KERNEL_TESTS_DYNAMICS_CARTPOLE_CARTPOLE_KERNEL_TEST_CUH_

#include <mppi/dynamics/cartpole/cartpole_dynamics.cuh>

__global__ void CartMassTestKernel(CartpoleDynamics* CP, float& mass_check);
__global__ void PoleMassTestKernel(CartpoleDynamics* CP, float& mass_check);
__global__ void PoleLengthTestKernel(CartpoleDynamics* CP, float& length_check);
__global__ void GravityTestKernel(CartpoleDynamics* CP, float& gravity_check);
__global__ void DynamicsTestKernel(CartpoleDynamics* CP, float* state, float* control, float* state_der);

void launchCartMassTestKernel(const CartpoleDynamics&, float& mass_check);
void launchPoleMassTestKernel(const CartpoleDynamics&, float& mass_check);
void launchPoleLengthTestKernel(const CartpoleDynamics&, float& length_check);
void launchGravityTestKernel(const CartpoleDynamics&, float& gravity_check);
void launchDynamicsTestKernel(const CartpoleDynamics&, float* state_cpu, float* control_cpu, float* state_der_cpu);

#ifdef __CUDACC__
#include "cartpole_dynamics_kernel_test.cu"
#endif

#endif  //! KERNEL_TESTS_DYNAMICS_CARTPOLE_CARTPOLE_KERNEL_TEST_CUH_
