#include <gtest/gtest.h>
#include <mppi/cost_functions/quadrotor/quadrotor_quadratic_cost.cuh>
#include <mppi/utils/test_helper.h>

#include <array>

TEST(QuadrotorQuadraticCost, Constructor)
{
  QuadrotorQuadraticCost();
}

template <class COST_T>
void __global__ StateCostKernel(COST_T* costs, float* s_d, float* result)
{
  result[0] = costs->computeStateCost(s_d);
  result[1] = costs->terminalCost(s_d);
}

TEST(QuadrotorQuadraticCost, ControlCost)
{
  using COST = QuadrotorQuadraticCost;
  const int num_results = 2;

  /**
   * CPU Setup
   */
  COST cost;
  COST::output_array s = COST::output_array::Random();
  Eigen::Quaternionf q_test(s[6], s[7], s[8], s[9]);
  q_test.normalize();
  s[6] = q_test.w();
  s[7] = q_test.x();
  s[8] = q_test.y();
  s[9] = q_test.z();

  // COST::control_array std_dev = COST::control_array::Constant(0.5);
  // float lambda = 0.8;
  QuadrotorQuadraticCostParams new_params;
  new_params.x_coeff = 5;
  cost.setParams(new_params);
  /**
   * GPU Setup
   */
  cost.GPUSetup();
  cudaStream_t stream_t;
  cudaStreamCreate(&stream_t);
  float* s_d;
  // float* eps_d;
  // float* std_dev_d;
  float* GPU_result_d;
  // Allocate Memory on GPU
  size_t state_size = sizeof(float) * COST::OUTPUT_DIM;
  HANDLE_ERROR(cudaMalloc((void**)&s_d, state_size));
  // HANDLE_ERROR(cudaMalloc((void**)&eps_d, state_size));
  // HANDLE_ERROR(cudaMalloc((void**)&std_dev_d, state_size));
  HANDLE_ERROR(cudaMalloc((void**)&GPU_result_d, sizeof(float) * num_results));

  // Copy data to GPU
  HANDLE_ERROR(cudaMemcpyAsync(s_d, s.data(), state_size, cudaMemcpyHostToDevice, stream_t));
  // HANDLE_ERROR(cudaMemcpyAsync(eps_d, eps.data(), state_size,
  //                              cudaMemcpyHostToDevice, stream_t));
  // HANDLE_ERROR(cudaMemcpyAsync(std_dev_d, std_dev.data(), state_size,
  //                              cudaMemcpyHostToDevice, stream_t));
  HANDLE_ERROR(cudaStreamSynchronize(stream_t));

  std::array<float, num_results> CPU_result;
  CPU_result[0] = cost.computeStateCost(s);
  CPU_result[1] = cost.terminalCost(s);

  // call GPU Kernel
  std::array<float, num_results> GPU_result;
  dim3 dimBlock(1, 1, 1);
  dim3 dimGrid(1, 1, 1);
  StateCostKernel<COST><<<dimGrid, dimBlock, 0, stream_t>>>(cost.cost_d_, s_d, GPU_result_d);
  CudaCheckError();
  HANDLE_ERROR(cudaStreamSynchronize(stream_t));
  // Copy GPU results back to Host
  HANDLE_ERROR(
      cudaMemcpyAsync(GPU_result.data(), GPU_result_d, sizeof(float) * num_results, cudaMemcpyDeviceToHost, stream_t));
  HANDLE_ERROR(cudaStreamSynchronize(stream_t));
  std::cout << "State Cost: " << CPU_result[0] << std::endl;
  array_assert_float_eq<num_results>(CPU_result, GPU_result);
}
