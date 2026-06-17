#pragma once

#ifndef KERNEL_TESTS_DYNAMICS_DOUBLE_INTEGRATOR_DI_KERNEL_TEST_CUH_
#define KERNEL_TESTS_DYNAMICS_DOUBLE_INTEGRATOR_DI_KERNEL_TEST_CUH_

#include <mppi/dynamics/double_integrator/di_dynamics.cuh>

__global__ void CheckModelSize(DoubleIntegratorDynamics* DI, long* model_size_check);

#include "di_dynamics_kernel_tests.cu"

#endif  //! KERNEL_TESTS_DYNAMICS_CARTPOLE_CARTPOLE_KERNEL_TEST_CUH_