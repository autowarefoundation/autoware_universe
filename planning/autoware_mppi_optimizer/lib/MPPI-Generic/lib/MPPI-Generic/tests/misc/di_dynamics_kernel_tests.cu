#include "di_dynamics_kernel_tests.cuh"

__global__ void CheckModelSize(DoubleIntegratorDynamics* DI, long* model_size_check)
{
  model_size_check[0] = sizeof(*DI);
}
