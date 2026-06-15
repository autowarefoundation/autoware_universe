#include "shaping_function_kernels_tests.cuh"

template <class CLASS_T, int NUM_ROLLOUTS, int BDIM_X>
void launchShapingFunction_KernelTest(std::array<float, NUM_ROLLOUTS>& trajectory_costs_host, CLASS_T& shape_function,
                                      float baseline, std::array<float, NUM_ROLLOUTS>& normalized_compute)
{
  // Allocate CUDA memory
  float* trajectory_costs_d;
  HANDLE_ERROR(cudaMalloc((void**)&trajectory_costs_d, sizeof(float) * trajectory_costs_host.size()))

  HANDLE_ERROR(cudaMemcpy(trajectory_costs_d, trajectory_costs_host.data(),
                          sizeof(float) * trajectory_costs_host.size(), cudaMemcpyHostToDevice))

  mppi_common::weightKernel<CLASS_T, NUM_ROLLOUTS>
      <<<1, NUM_ROLLOUTS>>>(trajectory_costs_d, baseline, shape_function.shaping_function_d_);
  CudaCheckError();

  HANDLE_ERROR(cudaMemcpy(normalized_compute.data(), trajectory_costs_d, sizeof(float) * trajectory_costs_host.size(),
                          cudaMemcpyDeviceToHost))

  cudaFree(trajectory_costs_d);
}

template <class CLASS_T, int NUM_ROLLOUTS>
void launchShapingFunction_KernelTest(typename CLASS_T::cost_traj& trajectory_costs_host, CLASS_T& shape_function,
                                      float baseline, std::array<float, NUM_ROLLOUTS>& normalized_compute,
                                      cudaStream_t stream)
{
  // Allocate CUDA memory
  float* trajectory_costs_d;
  HANDLE_ERROR(cudaMalloc((void**)&trajectory_costs_d, sizeof(float) * trajectory_costs_host.size()))

  HANDLE_ERROR(cudaMemcpy(trajectory_costs_d, trajectory_costs_host.data(),
                          sizeof(float) * trajectory_costs_host.size(), cudaMemcpyHostToDevice))

  shape_function.launchWeightKernel(trajectory_costs_d, baseline, stream);
  CudaCheckError();

  HANDLE_ERROR(cudaMemcpy(normalized_compute.data(), trajectory_costs_d, sizeof(float) * trajectory_costs_host.size(),
                          cudaMemcpyDeviceToHost))

  cudaFree(trajectory_costs_d);
}
