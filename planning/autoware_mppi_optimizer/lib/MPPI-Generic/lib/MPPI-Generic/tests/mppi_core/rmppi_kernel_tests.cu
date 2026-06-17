//
// Created by mgandhi on 5/23/20.
//
#include <gtest/gtest.h>
#include <kernel_tests/core/rmppi_kernel_test.cuh>
#include <mppi/controllers/MPPI/mppi_controller.cuh>
#include <mppi/core/rmppi_kernels.cuh>
#include <mppi/cost_functions/double_integrator/double_integrator_circle_cost.cuh>
#include <mppi/cost_functions/quadratic_cost/quadratic_cost.cuh>
#include <mppi/dynamics/double_integrator/di_dynamics.cuh>
#include <mppi/feedback_controllers/DDP/ddp.cuh>
#include <mppi/utils/test_helper.h>
#include <mppi/utils/logger.hpp>

#include <iostream>
#include <vector>

class RMPPIKernels : public ::testing::Test
{
public:
  static const int num_timesteps = 50;
  static const int nominal_idx = 0;
  static const int real_idx = 1;
  using DYN_T = DoubleIntegratorDynamics;
  using COST_T = DoubleIntegratorCircleCost;
  // using COST_T = QuadraticCost<DYN_T>;
  using SAMPLER_T = mppi::sampling_distributions::GaussianDistribution<typename DYN_T::DYN_PARAMS_T>;
  using FB_T = DDPFeedback<DYN_T, num_timesteps>;
  using control_trajectory = Eigen::Matrix<float, DYN_T::CONTROL_DIM, num_timesteps>;
  using state_array = DYN_T::state_array;
  using output_array = DYN_T::output_array;
  using control_array = DYN_T::control_array;

  float dt = 0.01;
  float lambda = 1.1f;
  float alpha = 0.0;
  float std_dev = 2.0f;
  float value_func_threshold = 20;
  float* initial_x_d = nullptr;
  float* cost_trajectories_d = nullptr;
  curandGenerator_t gen;
  cudaStream_t stream;
  mppi::util::MPPILoggerPtr logger = nullptr;

  void SetUp() override
  {
    model = new DYN_T(10);  // Initialize the double integrator DYN_T
    cost = new COST_T;      // Initialize the cost function
    auto sampler_params = SAMPLER_T::SAMPLING_PARAMS_T();
    for (int i = 0; i < DYN_T::CONTROL_DIM; i++)
    {
      sampler_params.std_dev[i] = std_dev;
    }
    sampler_params.num_timesteps = num_timesteps;
    sampler = new SAMPLER_T(sampler_params);
    fb_controller = new FB_T(model, dt);

    HANDLE_CURAND_ERROR(curandCreateGenerator(&gen, CURAND_RNG_PSEUDO_DEFAULT));
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    HANDLE_CURAND_ERROR(curandSetPseudoRandomGeneratorSeed(gen, seed));

    logger = std::make_shared<mppi::util::MPPILogger>();
    model->setLogger(logger);
    cost->setLogger(logger);
    sampler->setLogger(logger);
    fb_controller->setLogger(logger);

    HANDLE_ERROR(cudaStreamCreate(&stream));
    model->bindToStream(stream);
    cost->bindToStream(stream);
    sampler->bindToStream(stream);
    fb_controller->bindToStream(stream);
    curandSetStream(gen, stream);

    model->GPUSetup();
    cost->GPUSetup();
    sampler->GPUSetup();
    fb_controller->GPUSetup();
  }

  void TearDown() override
  {
    delete model;
    delete cost;
    delete sampler;
    delete fb_controller;
    if (initial_x_d)
    {
      HANDLE_ERROR(cudaFree(initial_x_d));
      initial_x_d = nullptr;
    }
    if (cost_trajectories_d)
    {
      HANDLE_ERROR(cudaFree(cost_trajectories_d));
      cost_trajectories_d = nullptr;
    }
  }

  DYN_T* model;
  COST_T* cost;
  SAMPLER_T* sampler;
  FB_T* fb_controller;
};

// Declare the static variables
const int RMPPIKernels::nominal_idx;
const int RMPPIKernels::real_idx;

/**
 * @brief Runs the combined init eval kernel and compares to one created on the CPU
 * to ensure they produce the same cost trajectories. Multiple runs of the init eval kernel
 * will be done with different parallelizations to ensure that the kernel calculates the same
 * regardless of the parallelization technique.
 */
TEST_F(RMPPIKernels, ValidateCombinedInitEvalKernelAgainstCPU)
{
  /**
   * Set up the problem
   */
  int num_candidates = 9;
  int num_samples = 64;
  int num_rollouts = num_candidates * num_samples;

  /**
   * Setup GPU
   */
  int* strides_d;
  sampler->setNumRollouts(num_rollouts);
  HANDLE_ERROR(cudaMalloc((void**)&strides_d, sizeof(int) * num_candidates));
  HANDLE_ERROR(cudaMalloc((void**)&initial_x_d, sizeof(float) * num_candidates * DYN_T::STATE_DIM));
  HANDLE_ERROR(cudaMalloc((void**)&cost_trajectories_d, sizeof(float) * num_rollouts));

  // Setup inputs on both CPU and GPU
  Eigen::MatrixXf candidates = Eigen::MatrixXf::Random(DYN_T::STATE_DIM, num_candidates);
  Eigen::MatrixXi strides = Eigen::MatrixXi::Zero(num_candidates, 1);
  for (int i = 0; i < num_candidates; i++)
  {
    strides(i) = i + 1;
  }
  Eigen::MatrixXf trajectory_costs_cpu = Eigen::MatrixXf::Zero(num_rollouts, 1);
  Eigen::MatrixXf trajectory_costs_gpu = Eigen::MatrixXf::Zero(num_rollouts, 1);

  control_trajectory nominal_trajectory = control_trajectory::Random();
  sampler->copyImportanceSamplerToDevice(nominal_trajectory.data(), 0, false);
  sampler->generateSamples(1, 0, gen, false);
  HANDLE_ERROR(cudaMemcpyAsync(initial_x_d, candidates.data(), sizeof(float) * DYN_T::STATE_DIM * num_candidates,
                               cudaMemcpyHostToDevice, stream));
  HANDLE_ERROR(
      cudaMemcpyAsync(strides_d, strides.data(), sizeof(int) * num_candidates, cudaMemcpyHostToDevice, stream));
  HANDLE_ERROR(cudaStreamSynchronize(stream));

  /**
   * Do CPU calculation
   */
  launchCPUInitEvalKernel<DYN_T, COST_T, SAMPLER_T>(model, cost, sampler, dt, num_timesteps, num_candidates,
                                                    num_samples, lambda, alpha, candidates, strides,
                                                    trajectory_costs_cpu);

  /**
   * Do GPU Calculation on various thread dimensions
   */
  std::vector<int> possible_thread_x;
  for (int size = num_samples; size > 0; size /= 2)
  {
    possible_thread_x.push_back(size);
  }
  std::vector<int> possible_thread_y{ 1, 2, 3, 8 };
  for (const auto& thread_x : possible_thread_x)
  {
    for (const auto& thread_y : possible_thread_y)
    {
      dim3 threadsPerBlock(thread_x, thread_y, 1);
      logger->info("Testing Combined Eval Kernel on (%d, %d, 1)\n", thread_x, thread_y);
      mppi::kernels::rmppi::launchInitEvalKernel<DYN_T, COST_T, SAMPLER_T>(
          model, cost, sampler, dt, num_timesteps, num_rollouts, lambda, alpha, num_samples, strides_d, initial_x_d,
          cost_trajectories_d, threadsPerBlock, stream, false);
      HANDLE_ERROR(cudaMemcpyAsync(trajectory_costs_gpu.data(), cost_trajectories_d, sizeof(float) * num_rollouts,
                                   cudaMemcpyDeviceToHost, stream));
      HANDLE_ERROR(cudaStreamSynchronize(stream));
      for (int i = 0; i < num_rollouts; i++)
      {
        float cost_diff = trajectory_costs_cpu(i) - trajectory_costs_gpu(i);
        std::string error_prefix = "Eval sample " + std::to_string(i) + " (" + std::to_string(threadsPerBlock.x) +
                                   ", " + std::to_string(threadsPerBlock.y) + ", " + std::to_string(threadsPerBlock.z) +
                                   ")";
        EXPECT_LT(fabsf(cost_diff), 1e-3 * trajectory_costs_cpu(i))
            << error_prefix << ": CPU = " << trajectory_costs_cpu(i) << ", GPU = " << trajectory_costs_gpu(i)
            << std::endl;
      }
    }
  }
  HANDLE_ERROR(cudaFree(strides_d));
}

TEST_F(RMPPIKernels, ValidateSplitInitEvalKernelAgainstCPU)
{
  /**
   * Set up the problem
   */
  int num_candidates = 12;
  int num_samples = 64;
  int num_rollouts = num_candidates * num_samples;

  /**
   * Setup GPU
   */
  int* strides_d;
  float* initial_x_d;
  float* cost_trajectories_d;
  float* output_d;
  sampler->setNumRollouts(num_rollouts);
  HANDLE_ERROR(cudaMalloc((void**)&strides_d, sizeof(int) * num_candidates));
  HANDLE_ERROR(cudaMalloc((void**)&initial_x_d, sizeof(float) * num_candidates * DYN_T::STATE_DIM));
  HANDLE_ERROR(cudaMalloc((void**)&cost_trajectories_d, sizeof(float) * num_rollouts));
  HANDLE_ERROR(cudaMalloc((void**)&output_d, sizeof(float) * num_rollouts * num_timesteps * DYN_T::OUTPUT_DIM));

  // Setup inputs on both CPU and GPU
  Eigen::MatrixXf candidates = Eigen::MatrixXf::Random(DYN_T::STATE_DIM, num_candidates) * 4;
  Eigen::MatrixXi strides = Eigen::MatrixXi::Zero(num_candidates, 1);
  for (int i = 0; i < num_candidates; i++)
  {
    strides(i) = i + 1;
  }
  Eigen::MatrixXf trajectory_costs_cpu = Eigen::MatrixXf::Zero(num_rollouts, 1);
  Eigen::MatrixXf trajectory_costs_gpu = Eigen::MatrixXf::Zero(num_rollouts, 1);

  control_trajectory nominal_trajectory = control_trajectory::Random();
  sampler->copyImportanceSamplerToDevice(nominal_trajectory.data(), 0, false);
  sampler->generateSamples(1, 0, gen, false);
  HANDLE_ERROR(cudaMemcpyAsync(initial_x_d, candidates.data(), sizeof(float) * DYN_T::STATE_DIM * num_candidates,
                               cudaMemcpyHostToDevice, stream));
  HANDLE_ERROR(
      cudaMemcpyAsync(strides_d, strides.data(), sizeof(int) * num_candidates, cudaMemcpyHostToDevice, stream));
  HANDLE_ERROR(cudaStreamSynchronize(stream));

  /**
   * Do CPU calculation
   */
  launchCPUInitEvalKernel<DYN_T, COST_T, SAMPLER_T>(model, cost, sampler, dt, num_timesteps, num_candidates,
                                                    num_samples, lambda, alpha, candidates, strides,
                                                    trajectory_costs_cpu);

  /**
   * Do GPU Calculation on various thread dimensions
   */
  std::vector<int> possible_dyn_thread_x;
  for (int size = num_samples; size > 0; size /= 2)
  {
    possible_dyn_thread_x.push_back(size);
  }
  std::vector<int> possible_cost_thread_x;
  for (int size = num_timesteps; size > 0; size /= 2)
  {
    possible_cost_thread_x.push_back(size);
  }
  std::vector<int> possible_thread_y{ 1, 2, 3, 8 };
  for (const auto& dyn_thread_x : possible_dyn_thread_x)
  {
    for (const auto& dyn_thread_y : possible_thread_y)
    {
      dim3 dynThreadsPerBlock(dyn_thread_x, dyn_thread_y, 1);
      for (const auto& cost_thread_x : possible_cost_thread_x)
      {
        for (const auto& cost_thread_y : possible_thread_y)
        {
          dim3 costThreadsPerBlock(cost_thread_x, cost_thread_y, 1);
          logger->info("Testing coalesced Split Eval Kernel with dyn(%d, %d, %d), cost(%d %d, %d)\n",
                       dynThreadsPerBlock.x, dynThreadsPerBlock.y, dynThreadsPerBlock.z, costThreadsPerBlock.x,
                       costThreadsPerBlock.y, costThreadsPerBlock.z);
          mppi::kernels::rmppi::launchSplitInitEvalKernel<DYN_T, COST_T, SAMPLER_T, true>(
              model, cost, sampler, dt, num_timesteps, num_rollouts, lambda, alpha, num_samples, strides_d, initial_x_d,
              output_d, cost_trajectories_d, dynThreadsPerBlock, costThreadsPerBlock, stream, false);
          HANDLE_ERROR(cudaMemcpyAsync(trajectory_costs_gpu.data(), cost_trajectories_d, sizeof(float) * num_rollouts,
                                       cudaMemcpyDeviceToHost, stream));
          HANDLE_ERROR(cudaStreamSynchronize(stream));
          for (int i = 0; i < num_rollouts; i++)
          {
            float cost_diff = trajectory_costs_cpu(i) - trajectory_costs_gpu(i);
            std::string error_prefix =
                "Coalesced eval sample " + std::to_string(i) + "/" + std::to_string(num_rollouts) + " dyn(" +
                std::to_string(dynThreadsPerBlock.x) + ", " + std::to_string(dynThreadsPerBlock.y) + ", " +
                std::to_string(dynThreadsPerBlock.z) + ") cost(" + std::to_string(costThreadsPerBlock.x) + ", " +
                std::to_string(costThreadsPerBlock.y) + ", " + std::to_string(costThreadsPerBlock.z) + ")";
            ASSERT_LT(fabsf(cost_diff), 1e-3 * trajectory_costs_cpu(i))
                << error_prefix << ": CPU = " << trajectory_costs_cpu(i) << ", GPU = " << trajectory_costs_gpu(i)
                << std::endl;
          }
          logger->info("Testing non-coalesced Split Eval Kernel with dyn(%d, %d, %d), cost(%d %d, %d)\n",
                       dynThreadsPerBlock.x, dynThreadsPerBlock.y, dynThreadsPerBlock.z, costThreadsPerBlock.x,
                       costThreadsPerBlock.y, costThreadsPerBlock.z);
          mppi::kernels::rmppi::launchSplitInitEvalKernel<DYN_T, COST_T, SAMPLER_T, false>(
              model, cost, sampler, dt, num_timesteps, num_rollouts, lambda, alpha, num_samples, strides_d, initial_x_d,
              output_d, cost_trajectories_d, dynThreadsPerBlock, costThreadsPerBlock, stream, false);
          HANDLE_ERROR(cudaMemcpyAsync(trajectory_costs_gpu.data(), cost_trajectories_d, sizeof(float) * num_rollouts,
                                       cudaMemcpyDeviceToHost, stream));
          HANDLE_ERROR(cudaStreamSynchronize(stream));
          for (int i = 0; i < num_rollouts; i++)
          {
            float cost_diff = trajectory_costs_cpu(i) - trajectory_costs_gpu(i);
            std::string error_prefix =
                "Non-coalesced eval sample " + std::to_string(i) + "/" + std::to_string(num_rollouts) + " dyn(" +
                std::to_string(dynThreadsPerBlock.x) + ", " + std::to_string(dynThreadsPerBlock.y) + ", " +
                std::to_string(dynThreadsPerBlock.z) + ") cost(" + std::to_string(costThreadsPerBlock.x) + ", " +
                std::to_string(costThreadsPerBlock.y) + ", " + std::to_string(costThreadsPerBlock.z) + ")";
            ASSERT_LT(fabsf(cost_diff), 1e-3 * trajectory_costs_cpu(i))
                << error_prefix << ": CPU = " << trajectory_costs_cpu(i) << ", GPU = " << trajectory_costs_gpu(i)
                << std::endl;
          }
        }
      }
    }
  }
  HANDLE_ERROR(cudaFree(strides_d));
  HANDLE_ERROR(cudaFree(output_d));
}

TEST_F(RMPPIKernels, ValidateCombinedRMPPIRolloutKernelAgainstCPU)
{
  int num_rollouts = 2048;

  sampler->setNumRollouts(num_rollouts);
  sampler->setNumDistributions(2);

  /**
   * Setup GPU
   */
  HANDLE_ERROR(cudaMalloc((void**)&initial_x_d, sizeof(float) * 2 * DYN_T::STATE_DIM));
  HANDLE_ERROR(cudaMalloc((void**)&cost_trajectories_d, sizeof(float) * 2 * num_rollouts));

  state_array initial_real_state = state_array::Random();
  state_array initial_nominal_state = state_array::Random();

  control_trajectory nominal_trajectory = control_trajectory::Random();
  sampler->copyImportanceSamplerToDevice(nominal_trajectory.data(), nominal_idx, false);
  sampler->copyImportanceSamplerToDevice(nominal_trajectory.data(), real_idx, false);
  fb_controller->copyToDevice(false);
  sampler->generateSamples(1, 0, gen, false);
  HANDLE_ERROR(cudaMemcpyAsync(initial_x_d + nominal_idx * DYN_T::STATE_DIM, initial_nominal_state.data(),
                               sizeof(float) * DYN_T::STATE_DIM, cudaMemcpyHostToDevice, stream));
  HANDLE_ERROR(cudaMemcpyAsync(initial_x_d + real_idx * DYN_T::STATE_DIM, initial_real_state.data(),
                               sizeof(float) * DYN_T::STATE_DIM, cudaMemcpyHostToDevice, stream));
  HANDLE_ERROR(cudaStreamSynchronize(stream));

  Eigen::MatrixXf trajectory_costs_cpu = Eigen::MatrixXf::Zero(2 * num_rollouts, 1);
  Eigen::MatrixXf trajectory_costs_gpu = Eigen::MatrixXf::Zero(2 * num_rollouts, 1);

  /**
   * CPU Calculation
   **/
  launchCPURMPPIRolloutKernel<DYN_T, COST_T, SAMPLER_T, FB_T>(
      model, cost, sampler, fb_controller, dt, num_timesteps, num_rollouts, lambda, alpha, value_func_threshold,
      nominal_idx, real_idx, initial_real_state, initial_nominal_state, trajectory_costs_cpu);
  /**
   * GPU Calculation
   **/
  std::vector<int> possible_thread_x;
  for (int size = 64; size > 0; size /= 2)
  {
    possible_thread_x.push_back(size);
  }
  std::vector<int> possible_thread_y{ 1, 2, 3 };
  for (const auto& thread_x : possible_thread_x)
  {
    for (const auto& thread_y : possible_thread_y)
    {
      dim3 threadsPerBlock(thread_x, thread_y, 2);
      logger->info("Testing RMPPI Rollout Kernel with (%d, %d, %d)\n", threadsPerBlock.x, threadsPerBlock.y,
                   threadsPerBlock.z);
      mppi::kernels::rmppi::launchRMPPIRolloutKernel<DYN_T, COST_T, SAMPLER_T, FB_T::TEMPLATED_GPU_FEEDBACK,
                                                     nominal_idx>(
          model, cost, sampler, fb_controller->getHostPointer().get(), dt, num_timesteps, num_rollouts, lambda, alpha,
          value_func_threshold, initial_x_d, cost_trajectories_d, threadsPerBlock, stream, false);
      HANDLE_ERROR(cudaMemcpyAsync(trajectory_costs_gpu.data(), cost_trajectories_d, sizeof(float) * 2 * num_rollouts,
                                   cudaMemcpyDeviceToHost, stream));
      HANDLE_ERROR(cudaStreamSynchronize(stream));
      for (int i = 0; i < 2 * num_rollouts; i++)
      {
        float cost_diff = trajectory_costs_cpu(i) - trajectory_costs_gpu(i);
        std::string error_prefix = "Rollout sample " + std::to_string(i) + " (" + std::to_string(threadsPerBlock.x) +
                                   ", " + std::to_string(threadsPerBlock.y) + ", " + std::to_string(threadsPerBlock.z) +
                                   ")";
        EXPECT_LT(fabsf(cost_diff), 1e-3 * trajectory_costs_cpu(i))
            << error_prefix << ": CPU = " << trajectory_costs_cpu(i) << ", GPU = " << trajectory_costs_gpu(i)
            << std::endl;
      }
    }
  }
}

TEST_F(RMPPIKernels, ValidateSplitRMPPIRolloutKernelAgainstCPU)
{
  int num_rollouts = 2048;

  sampler->setNumRollouts(num_rollouts);
  sampler->setNumDistributions(2);

  /**
   * Setup GPU
   */
  float* output_d = nullptr;
  HANDLE_ERROR(cudaMalloc((void**)&initial_x_d, sizeof(float) * 2 * DYN_T::STATE_DIM));
  HANDLE_ERROR(cudaMalloc((void**)&cost_trajectories_d, sizeof(float) * 2 * num_rollouts));
  HANDLE_ERROR(cudaMalloc((void**)&output_d, sizeof(float) * 2 * num_rollouts * num_timesteps * DYN_T::OUTPUT_DIM));

  state_array initial_real_state = state_array::Random();
  state_array initial_nominal_state = state_array::Random();

  control_trajectory nominal_trajectory = control_trajectory::Random();
  sampler->copyImportanceSamplerToDevice(nominal_trajectory.data(), nominal_idx, false);
  sampler->copyImportanceSamplerToDevice(nominal_trajectory.data(), real_idx, false);
  fb_controller->copyToDevice(false);
  sampler->generateSamples(1, 0, gen, false);
  HANDLE_ERROR(cudaMemcpyAsync(initial_x_d + nominal_idx * DYN_T::STATE_DIM, initial_nominal_state.data(),
                               sizeof(float) * DYN_T::STATE_DIM, cudaMemcpyHostToDevice, stream));
  HANDLE_ERROR(cudaMemcpyAsync(initial_x_d + real_idx * DYN_T::STATE_DIM, initial_real_state.data(),
                               sizeof(float) * DYN_T::STATE_DIM, cudaMemcpyHostToDevice, stream));
  HANDLE_ERROR(cudaStreamSynchronize(stream));

  Eigen::MatrixXf trajectory_costs_cpu = Eigen::MatrixXf::Zero(2 * num_rollouts, 1);
  Eigen::MatrixXf trajectory_costs_gpu = Eigen::MatrixXf::Zero(2 * num_rollouts, 1);

  /**
   * CPU Calculation
   **/
  launchCPURMPPIRolloutKernel<DYN_T, COST_T, SAMPLER_T, FB_T>(
      model, cost, sampler, fb_controller, dt, num_timesteps, num_rollouts, lambda, alpha, value_func_threshold,
      nominal_idx, real_idx, initial_real_state, initial_nominal_state, trajectory_costs_cpu);
  /**
   * GPU Calculation
   **/
  std::vector<int> possible_dyn_thread_x;
  for (int size = 64; size > 0; size /= 2)
  {
    possible_dyn_thread_x.push_back(size);
  }
  std::vector<int> possible_cost_thread_x;
  for (int size = num_timesteps; size > 0; size /= 2)
  {
    possible_cost_thread_x.push_back(size);
  }
  std::vector<int> possible_thread_y{ 1, 2, 3 };
  for (const auto& dyn_thread_x : possible_dyn_thread_x)
  {
    for (const auto& dyn_thread_y : possible_thread_y)
    {
      dim3 dynThreadsPerBlock(dyn_thread_x, dyn_thread_y, 2);
      for (const auto& cost_thread_x : possible_cost_thread_x)
      {
        for (const auto& cost_thread_y : possible_thread_y)
        {
          dim3 costThreadsPerBlock(cost_thread_x, cost_thread_y, 2);
          logger->info("Testing coalesced RMPPI Rollout Kernel with dyn(%d, %d, %d), cost(%d %d, %d)\n",
                       dynThreadsPerBlock.x, dynThreadsPerBlock.y, dynThreadsPerBlock.z, costThreadsPerBlock.x,
                       costThreadsPerBlock.y, costThreadsPerBlock.z);
          mppi::kernels::rmppi::launchSplitRMPPIRolloutKernel<DYN_T, COST_T, SAMPLER_T, FB_T::TEMPLATED_GPU_FEEDBACK,
                                                              nominal_idx, true>(
              model, cost, sampler, fb_controller->getHostPointer().get(), dt, num_timesteps, num_rollouts, lambda,
              alpha, value_func_threshold, initial_x_d, output_d, cost_trajectories_d, dynThreadsPerBlock,
              costThreadsPerBlock, stream, false);
          HANDLE_ERROR(cudaMemcpyAsync(trajectory_costs_gpu.data(), cost_trajectories_d,
                                       sizeof(float) * 2 * num_rollouts, cudaMemcpyDeviceToHost, stream));
          HANDLE_ERROR(cudaStreamSynchronize(stream));
          for (int i = 0; i < 2 * num_rollouts; i++)
          {
            float cost_diff = trajectory_costs_cpu(i) - trajectory_costs_gpu(i);
            std::string error_prefix =
                "Coalesced rollout sample " + std::to_string(i) + "/" + std::to_string(num_rollouts) + " dyn(" +
                std::to_string(dynThreadsPerBlock.x) + ", " + std::to_string(dynThreadsPerBlock.y) + ", " +
                std::to_string(dynThreadsPerBlock.z) + ") cost(" + std::to_string(costThreadsPerBlock.x) + ", " +
                std::to_string(costThreadsPerBlock.y) + ", " + std::to_string(costThreadsPerBlock.z) + ")";
            ASSERT_LT(fabsf(cost_diff), 1e-3 * trajectory_costs_cpu(i))
                << error_prefix << ": CPU = " << trajectory_costs_cpu(i) << ", GPU = " << trajectory_costs_gpu(i)
                << std::endl;
          }
          logger->info("Testing non-coalesced RMPPI Rollout Kernel with dyn(%d, %d, %d), cost(%d %d, %d)\n",
                       dynThreadsPerBlock.x, dynThreadsPerBlock.y, dynThreadsPerBlock.z, costThreadsPerBlock.x,
                       costThreadsPerBlock.y, costThreadsPerBlock.z);
          mppi::kernels::rmppi::launchSplitRMPPIRolloutKernel<DYN_T, COST_T, SAMPLER_T, FB_T::TEMPLATED_GPU_FEEDBACK,
                                                              nominal_idx, false>(
              model, cost, sampler, fb_controller->getHostPointer().get(), dt, num_timesteps, num_rollouts, lambda,
              alpha, value_func_threshold, initial_x_d, output_d, cost_trajectories_d, dynThreadsPerBlock,
              costThreadsPerBlock, stream, false);
          HANDLE_ERROR(cudaMemcpyAsync(trajectory_costs_gpu.data(), cost_trajectories_d,
                                       sizeof(float) * 2 * num_rollouts, cudaMemcpyDeviceToHost, stream));
          HANDLE_ERROR(cudaStreamSynchronize(stream));
          eigen_assert_float_near<Eigen::MatrixXf>(trajectory_costs_cpu, trajectory_costs_gpu, 1e-3);
          for (int i = 0; i < 2 * num_rollouts; i++)
          {
            float cost_diff = trajectory_costs_cpu(i) - trajectory_costs_gpu(i);
            std::string error_prefix =
                "Non-coalesced rollout sample " + std::to_string(i) + "/" + std::to_string(num_rollouts) + " dyn(" +
                std::to_string(dynThreadsPerBlock.x) + ", " + std::to_string(dynThreadsPerBlock.y) + ", " +
                std::to_string(dynThreadsPerBlock.z) + ") cost(" + std::to_string(costThreadsPerBlock.x) + ", " +
                std::to_string(costThreadsPerBlock.y) + ", " + std::to_string(costThreadsPerBlock.z) + ")";
            ASSERT_LT(fabsf(cost_diff), 1e-3 * trajectory_costs_cpu(i))
                << error_prefix << ": CPU = " << trajectory_costs_cpu(i) << ", GPU = " << trajectory_costs_gpu(i)
                << std::endl;
          }
        }
      }
    }
  }
  HANDLE_ERROR(cudaFree(output_d));
}

TEST_F(RMPPIKernels, ValidateCombinedRMPPIRolloutKernelAgainstMPPIRollout)
{
  int num_rollouts = 2048;

  sampler->setNumRollouts(num_rollouts);
  sampler->setNumDistributions(2);

  // Used to reset control samples between calls to rollout kernels
  Eigen::MatrixXf control_noise = Eigen::MatrixXf::Zero(DYN_T::CONTROL_DIM, 2 * num_rollouts * num_timesteps);

  /**
   * Setup GPU
   */
  HANDLE_ERROR(cudaMalloc((void**)&initial_x_d, sizeof(float) * 2 * DYN_T::STATE_DIM));
  HANDLE_ERROR(cudaMalloc((void**)&cost_trajectories_d, sizeof(float) * 2 * num_rollouts));

  state_array initial_real_state = state_array::Random();

  control_trajectory nominal_trajectory = control_trajectory::Random();
  sampler->copyImportanceSamplerToDevice(nominal_trajectory.data(), nominal_idx, false);
  sampler->copyImportanceSamplerToDevice(nominal_trajectory.data(), real_idx, false);
  fb_controller->copyToDevice(false);
  sampler->generateSamples(1, 0, gen, false);

  // Get unaltered control noise for resetting purposes
  HANDLE_ERROR(cudaMemcpyAsync(control_noise.data(), sampler->getControlSample(0, 0, 0),
                               sizeof(float) * 2 * num_rollouts * num_timesteps * DYN_T::CONTROL_DIM,
                               cudaMemcpyDeviceToHost, stream));
  // Start both nominal and real states at the same point
  HANDLE_ERROR(cudaMemcpyAsync(initial_x_d + nominal_idx * DYN_T::STATE_DIM, initial_real_state.data(),
                               sizeof(float) * DYN_T::STATE_DIM, cudaMemcpyHostToDevice, stream));
  HANDLE_ERROR(cudaMemcpyAsync(initial_x_d + real_idx * DYN_T::STATE_DIM, initial_real_state.data(),
                               sizeof(float) * DYN_T::STATE_DIM, cudaMemcpyHostToDevice, stream));
  HANDLE_ERROR(cudaStreamSynchronize(stream));

  Eigen::MatrixXf trajectory_costs_rmppi = Eigen::MatrixXf::Zero(2 * num_rollouts, 1);
  Eigen::MatrixXf trajectory_costs_mppi = Eigen::MatrixXf::Zero(2 * num_rollouts, 1);

  /**
   * GPU Calculation
   **/
  std::vector<int> possible_thread_x;
  for (int size = 64; size > 0; size /= 2)
  {
    possible_thread_x.push_back(size);
  }
  std::vector<int> possible_thread_y{ 1, 2, 3 };
  for (const auto& thread_x : possible_thread_x)
  {
    for (const auto& thread_y : possible_thread_y)
    {
      dim3 threadsPerBlock(thread_x, thread_y, 2);
      mppi::kernels::rmppi::launchRMPPIRolloutKernel<DYN_T, COST_T, SAMPLER_T, FB_T::TEMPLATED_GPU_FEEDBACK,
                                                     nominal_idx>(
          model, cost, sampler, fb_controller->getHostPointer().get(), dt, num_timesteps, num_rollouts, lambda, alpha,
          value_func_threshold, initial_x_d, cost_trajectories_d, threadsPerBlock, stream, false);
      HANDLE_ERROR(cudaMemcpyAsync(trajectory_costs_rmppi.data(), cost_trajectories_d, sizeof(float) * 2 * num_rollouts,
                                   cudaMemcpyDeviceToHost, stream));
      HANDLE_ERROR(cudaStreamSynchronize(stream));
      // Reset control samples
      HANDLE_ERROR(cudaMemcpyAsync(sampler->getControlSample(0, 0, 0), control_noise.data(),
                                   sizeof(float) * 2 * num_rollouts * num_timesteps * DYN_T::CONTROL_DIM,
                                   cudaMemcpyHostToDevice, stream));
      mppi::kernels::launchRolloutKernel<DYN_T, COST_T, SAMPLER_T>(model, cost, sampler, dt, num_timesteps,
                                                                   num_rollouts, lambda, alpha, initial_x_d,
                                                                   cost_trajectories_d, threadsPerBlock, stream, false);
      // Reset control samples
      HANDLE_ERROR(cudaMemcpyAsync(sampler->getControlSample(0, 0, 0), control_noise.data(),
                                   sizeof(float) * 2 * num_rollouts * num_timesteps * DYN_T::CONTROL_DIM,
                                   cudaMemcpyHostToDevice, stream));
      HANDLE_ERROR(cudaMemcpyAsync(trajectory_costs_mppi.data(), cost_trajectories_d, sizeof(float) * 2 * num_rollouts,
                                   cudaMemcpyDeviceToHost, stream));
      HANDLE_ERROR(cudaStreamSynchronize(stream));
      for (int i = 0; i < 2 * num_rollouts; i++)
      {
        float cost_diff = trajectory_costs_mppi(i) - trajectory_costs_rmppi(i);
        std::string error_prefix = "Rollout sample " + std::to_string(i) + " (" + std::to_string(threadsPerBlock.x) +
                                   ", " + std::to_string(threadsPerBlock.y) + ", " + std::to_string(threadsPerBlock.z) +
                                   ")";
        EXPECT_LT(fabsf(cost_diff), 1e-3 * trajectory_costs_mppi(i))
            << error_prefix << ": MPPI = " << trajectory_costs_mppi(i) << ", R-MPPI = " << trajectory_costs_rmppi(i)
            << std::endl;
      }
    }
  }
}
