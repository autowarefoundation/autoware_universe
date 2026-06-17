#include <gtest/gtest.h>
#include <mppi/dynamics/quadrotor/quadrotor_dynamics.cuh>
#include <mppi/utils/test_helper.h>

template <class DYN_T>
void __global__ DynamicsDerivKernel(DYN_T* model, float* state_d, float* u_d, float* state_deriv_d)
{
  model->computeDynamics(state_d, u_d, state_deriv_d);
}

template <class DYN_T>
void __global__ UpdateStateKernel(DYN_T* model, float* state_d, float* u_d, float* state_deriv_d, float dt)
{
  model->computeDynamics(state_d, u_d, state_deriv_d);
  model->updateState(state_d, state_deriv_d, dt);
}

TEST(QuadrotorDynamics, Constructor)
{
  QuadrotorDynamics();
}

TEST(QuadrotorDynamics, CompareDynamics_CPU_GPU)
{
  using DYN = QuadrotorDynamics;
  DYN model;

  DYN::state_array s = DYN::state_array::Random();
  DYN::control_array u = DYN::control_array::Random();
  DYN::state_array state_deriv_cpu, state_deriv_gpu;

  Eigen::Quaternionf q_test(s[6], s[7], s[8], s[9]);
  q_test.normalize();
  s[6] = q_test.w();
  s[7] = q_test.x();
  s[8] = q_test.y();
  s[9] = q_test.z();

  /**
   * GPU Setup
   */
  model.GPUSetup();
  cudaStream_t s1;
  cudaStreamCreate(&s1);
  float* s_d;
  float* u_d;
  float* state_deriv_GPU;
  // Allocate GPU Memory
  // size_t control_size = sizeof(float) * DYN::CONTROL_DIM;
  HANDLE_ERROR(cudaMalloc((void**)&u_d, sizeof(float) * DYN::CONTROL_DIM));
  HANDLE_ERROR(cudaMalloc((void**)&s_d, sizeof(float) * DYN::STATE_DIM));
  HANDLE_ERROR(cudaMalloc((void**)&state_deriv_GPU, sizeof(float) * DYN::STATE_DIM));

  // Copy data to GPU
  HANDLE_ERROR(cudaMemcpyAsync(u_d, u.data(), sizeof(float) * DYN::CONTROL_DIM, cudaMemcpyHostToDevice, s1));
  HANDLE_ERROR(cudaMemcpyAsync(s_d, s.data(), sizeof(float) * DYN::STATE_DIM, cudaMemcpyHostToDevice, s1));
  HANDLE_ERROR(cudaStreamSynchronize(s1));

  model.computeDynamics(s, u, state_deriv_cpu);

  // Call GPU Kernel
  dim3 dimBlock(1, 5, 1);
  dim3 dimGrid(1, 1, 1);
  DynamicsDerivKernel<DYN><<<dimGrid, dimBlock, 0, s1>>>(model.model_d_, s_d, u_d, state_deriv_GPU);
  CudaCheckError();
  HANDLE_ERROR(cudaStreamSynchronize(s1));
  // Copy GPU results back to Host
  HANDLE_ERROR(cudaMemcpyAsync(state_deriv_gpu.data(), state_deriv_GPU, sizeof(float) * DYN::STATE_DIM,
                               cudaMemcpyDeviceToHost, s1));
  HANDLE_ERROR(cudaStreamSynchronize(s1));
  eigen_assert_float_eq<DYN::state_array>(state_deriv_cpu, state_deriv_gpu);
}

TEST(QuadrotorDynamics, UpdateState_CPU_GPU)
{
  using DYN = QuadrotorDynamics;
  DYN model;
  QuadrotorDynamicsParams params = QuadrotorDynamicsParams(2.5);

  DYN::state_array s_cpu = DYN::state_array::Random();
  DYN::control_array u = DYN::control_array::Random();
  DYN::state_array s_gpu;
  DYN::state_array state_deriv_cpu, state_deriv_gpu;
  float dt = 0.01;

  Eigen::Quaternionf q_test(s_cpu[6], s_cpu[7], s_cpu[8], s_cpu[9]);
  q_test.normalize();
  s_cpu[6] = q_test.w();
  s_cpu[7] = q_test.x();
  s_cpu[8] = q_test.y();
  s_cpu[9] = q_test.z();

  s_gpu = s_cpu;

  /**
   * GPU Setup
   */
  model.GPUSetup();
  model.setParams(params);
  cudaStream_t s1;
  cudaStreamCreate(&s1);
  float* s_d;
  float* u_d;
  float* state_deriv_GPU;
  // Allocate GPU Memory
  HANDLE_ERROR(cudaMalloc((void**)&u_d, sizeof(float) * DYN::CONTROL_DIM));
  HANDLE_ERROR(cudaMalloc((void**)&s_d, sizeof(float) * DYN::STATE_DIM));
  HANDLE_ERROR(cudaMalloc((void**)&state_deriv_GPU, sizeof(float) * DYN::STATE_DIM));

  // Copy data to GPU
  HANDLE_ERROR(cudaMemcpyAsync(u_d, u.data(), sizeof(float) * DYN::CONTROL_DIM, cudaMemcpyHostToDevice, s1));
  HANDLE_ERROR(cudaMemcpyAsync(s_d, s_gpu.data(), sizeof(float) * DYN::STATE_DIM, cudaMemcpyHostToDevice, s1));
  HANDLE_ERROR(cudaStreamSynchronize(s1));

  model.computeDynamics(s_cpu, u, state_deriv_cpu);
  model.updateState(s_cpu, state_deriv_cpu, dt);

  // Call GPU Kernel
  dim3 dimBlock(1, 5, 1);
  dim3 dimGrid(1, 1, 1);
  UpdateStateKernel<DYN><<<dimGrid, dimBlock, 0, s1>>>(model.model_d_, s_d, u_d, state_deriv_GPU, dt);
  CudaCheckError();
  HANDLE_ERROR(cudaStreamSynchronize(s1));
  // Copy GPU results back to Host
  HANDLE_ERROR(cudaMemcpyAsync(s_gpu.data(), s_d, sizeof(float) * DYN::STATE_DIM, cudaMemcpyDeviceToHost, s1));
  HANDLE_ERROR(cudaMemcpyAsync(state_deriv_gpu.data(), state_deriv_GPU, sizeof(float) * DYN::STATE_DIM,
                               cudaMemcpyDeviceToHost, s1));
  HANDLE_ERROR(cudaStreamSynchronize(s1));
  eigen_assert_float_eq<DYN::state_array>(s_cpu, s_gpu);
}