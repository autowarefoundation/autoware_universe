#include "normexp_kernel_test.cuh"

template <int NUM_ROLLOUTS>
void launchNormExp_KernelTest(std::array<float, NUM_ROLLOUTS>& trajectory_costs_host, float gamma, float baseline,
                              std::array<float, NUM_ROLLOUTS>& normalized_compute)
{
  // Allocate CUDA memory
  float* trajectory_costs_d;
  HANDLE_ERROR(cudaMalloc((void**)&trajectory_costs_d, sizeof(float) * trajectory_costs_host.size()))

  HANDLE_ERROR(cudaMemcpy(trajectory_costs_d, trajectory_costs_host.data(),
                          sizeof(float) * trajectory_costs_host.size(), cudaMemcpyHostToDevice))

  mppi::kernels::normExpKernel<<<1, NUM_ROLLOUTS>>>(NUM_ROLLOUTS, trajectory_costs_d, gamma, baseline);
  CudaCheckError();

  HANDLE_ERROR(cudaMemcpy(normalized_compute.data(), trajectory_costs_d, sizeof(float) * trajectory_costs_host.size(),
                          cudaMemcpyDeviceToHost))

  cudaFree(trajectory_costs_d);
}

template <int NUM_ROLLOUTS, int BLOCKSIZE_X>
void launchGenericNormExpKernelTest(std::array<float, NUM_ROLLOUTS> trajectory_costs_host, float gamma, float baseline,
                                    std::array<float, NUM_ROLLOUTS>& normalized_compute)
{
  // Allocate CUDA memory
  float* trajectory_costs_d;
  HANDLE_ERROR(cudaMalloc((void**)&trajectory_costs_d, sizeof(float) * trajectory_costs_host.size()));

  HANDLE_ERROR(cudaMemcpy(trajectory_costs_d, trajectory_costs_host.data(),
                          sizeof(float) * trajectory_costs_host.size(), cudaMemcpyHostToDevice));

  dim3 dimBlock(BLOCKSIZE_X, 1, 1);
  dim3 dimGrid((NUM_ROLLOUTS - 1) / BLOCKSIZE_X + 1, 1, 1);

  mppi::kernels::normExpKernel<<<dimGrid, dimBlock>>>(NUM_ROLLOUTS, trajectory_costs_d, gamma, baseline);
  CudaCheckError();

  HANDLE_ERROR(cudaMemcpy(normalized_compute.data(), trajectory_costs_d, sizeof(float) * trajectory_costs_host.size(),
                          cudaMemcpyDeviceToHost));

  cudaFree(trajectory_costs_d);
}
