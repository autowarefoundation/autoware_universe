#include <gtest/gtest.h>
#include <kernel_tests/core/rollout_kernel_test.cuh>
#include <mppi/cost_functions/autorally/ar_standard_cost.cuh>
#include <mppi/cost_functions/cartpole/cartpole_quadratic_cost.cuh>
#include <mppi/dynamics/autorally/ar_nn_model.cuh>
#include <mppi/dynamics/cartpole/cartpole_dynamics.cuh>
#include <mppi/sampling_distributions/gaussian/gaussian.cuh>
#include <mppi/utils/test_helper.h>

#include <autorally_test_map.h>
#include <autorally_test_network.h>
#include <random>

#include <autorally_test_network.h>
#include <autorally_test_map.h>
/*
 * Here we will test various device functions that are related to cuda kernel things.
 */

TEST(RolloutKernel, loadGlobalToShared)
{
  const int STATE_DIM = 12;
  const int CONTROL_DIM = 3;
  std::vector<float> x0_host = { 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2 };

  std::vector<float> x_thread_host(STATE_DIM, 0.f);
  std::vector<float> xdot_thread_host(STATE_DIM, 2.f);
  std::vector<float> u_thread_host(CONTROL_DIM, 3.f);

  launchGlobalToShared_KernelTest(x0_host, x_thread_host, xdot_thread_host, u_thread_host);

  array_assert_float_eq(x0_host, x_thread_host, STATE_DIM);
  array_assert_float_eq(0.f, xdot_thread_host, STATE_DIM);
  array_assert_float_eq(0.f, u_thread_host, CONTROL_DIM);
}

TEST(RolloutKernel, loadGlobalToSharedNominalAndActualState)
{
  const int STATE_DIM = 12;
  const int CONTROL_DIM = 3;
  std::vector<float> x0_host_act = { 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2 };

  std::vector<float> x0_host_nom = { 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, 2.0, 2.1, 2.2 };

  std::vector<float> x_thread_host_act(STATE_DIM, 0.f);
  std::vector<float> x_thread_host_nom(STATE_DIM, 0.f);
  std::vector<float> xdot_thread_host_act(STATE_DIM, 2.f);
  std::vector<float> xdot_thread_host_nom(STATE_DIM, 2.f);

  std::vector<float> u_thread_host_act(CONTROL_DIM, 3.f);
  std::vector<float> u_thread_host_nom(CONTROL_DIM, 3.f);

  launchGlobalToShared_KernelTest_nom_act(x0_host_act, x_thread_host_act, xdot_thread_host_act, u_thread_host_act,
                                          x0_host_nom, x_thread_host_nom, xdot_thread_host_nom, u_thread_host_nom);

  // std::cout << "Testing actual x0" << std::endl;
  array_assert_float_eq(x0_host_act, x_thread_host_act, STATE_DIM);
  // std::cout << "Testing nom x0" << std::endl;
  array_assert_float_eq(x0_host_nom, x_thread_host_nom, STATE_DIM);
  // std::cout << "Testing empty" << std::endl;
  array_assert_float_eq(0.f, xdot_thread_host_act, STATE_DIM);
  array_assert_float_eq(0.f, xdot_thread_host_nom, STATE_DIM);
  array_assert_float_eq(0.f, u_thread_host_act, CONTROL_DIM);
  array_assert_float_eq(0.f, u_thread_host_nom, CONTROL_DIM);
}

TEST(RolloutKernel, computeAndSaveCostAllRollouts)
{
  // Define an assortment of costs for a given number of rollouts
  CartpoleQuadraticCost cost;
  cost.GPUSetup();

  const int num_rollouts = 1234;
  std::array<float, num_rollouts> cost_all_rollouts = { 0 };
  std::array<float, CartpoleDynamics::STATE_DIM* num_rollouts> x_traj_terminal = { 0 };
  std::array<float, num_rollouts> cost_known = { 0 };
  std::array<float, num_rollouts> cost_compute = { 0 };

  std::default_random_engine generator(7.0);
  std::normal_distribution<float> distribution(1.0, 2.0);

  for (auto& costs : cost_all_rollouts)
  {
    costs = 10 * distribution(generator);
  }

  for (auto& state : x_traj_terminal)
  {
    state = distribution(generator);
  }
  // Compute terminal cost on CPU
  for (int i = 0; i < num_rollouts; ++i)
  {
    cost_known[i] =
        cost_all_rollouts[i] +
        (x_traj_terminal[CartpoleDynamics::STATE_DIM * i] * x_traj_terminal[CartpoleDynamics::STATE_DIM * i] *
             cost.getParams().cart_position_coeff +
         x_traj_terminal[CartpoleDynamics::STATE_DIM * i + 1] * x_traj_terminal[CartpoleDynamics::STATE_DIM * i + 1] *
             cost.getParams().cart_velocity_coeff +
         x_traj_terminal[CartpoleDynamics::STATE_DIM * i + 2] * x_traj_terminal[CartpoleDynamics::STATE_DIM * i + 2] *
             cost.getParams().pole_angle_coeff +
         x_traj_terminal[CartpoleDynamics::STATE_DIM * i + 3] * x_traj_terminal[CartpoleDynamics::STATE_DIM * i + 3] *
             cost.getParams().pole_angular_velocity_coeff) *
            cost.getParams().terminal_cost_coeff;
  }

  // Compute the dynamics on the GPU
  launchComputeAndSaveCostAllRollouts_KernelTest<CartpoleQuadraticCost, CartpoleDynamics::STATE_DIM, num_rollouts>(
      cost, cost_all_rollouts, x_traj_terminal, cost_compute);

  array_assert_float_eq<num_rollouts>(cost_known, cost_compute);
}

class RolloutKernelTests : public ::testing::Test
{
public:
  using DYN_T = CartpoleDynamics;
  using COST_T = CartpoleQuadraticCost;
  using DYN_PARAMS_T = typename DYN_T::DYN_PARAMS_T;
  using COST_PARAMS_T = typename COST_T::COST_PARAMS_T;
  using SAMPLER_T = mppi::sampling_distributions::GaussianDistribution<DYN_PARAMS_T>;
  using SAMPLER_PARAMS_T = typename SAMPLER_T::SAMPLING_PARAMS_T;
  using state_array = DYN_T::state_array;

  float dt = 0.01;
  float lambda = 0.5f;
  float alpha = 0.001f;
  float control_std_dev = 0.4f;
  int num_timesteps = 100;
  int num_rollouts = 2048;

  cudaStream_t stream;
  DYN_T* model;
  COST_T* cost;
  SAMPLER_T* sampler;
  mppi::util::MPPILoggerPtr logger;

  void SetUp() override
  {
    model = new DYN_T();
    cost = new COST_T();
    sampler = new SAMPLER_T();
    logger = std::make_shared<mppi::util::MPPILogger>();
    model->setLogger(logger);
    cost->setLogger(logger);
    sampler->setLogger(logger);

    SAMPLER_PARAMS_T sampler_params;
    for (int i = 0; i < DYN_T::CONTROL_DIM; i++)
    {
      sampler_params.std_dev[i] = control_std_dev;
    }
    sampler_params.num_rollouts = num_rollouts;
    sampler_params.num_timesteps = num_timesteps;
    sampler->setParams(sampler_params);

    COST_PARAMS_T cost_params;
    cost_params.cart_position_coeff = 100;
    cost_params.pole_angle_coeff = 200;
    cost_params.cart_velocity_coeff = 10;
    cost_params.pole_angular_velocity_coeff = 20;
    cost_params.control_cost_coeff[0] = 1;
    cost_params.terminal_cost_coeff = 0;
    cost_params.desired_terminal_state[0] = -20;
    cost_params.desired_terminal_state[1] = 0;
    cost_params.desired_terminal_state[2] = M_PI;
    cost_params.desired_terminal_state[3] = 0;
    cost->setParams(cost_params);

    HANDLE_ERROR(cudaStreamCreate(&stream));
  }

  void TearDown() override
  {
    delete model;
    delete cost;
    delete sampler;
  }
};

TEST_F(RolloutKernelTests, runRolloutKernelOnMultipleSystems)
{
  std::vector<float> x0(DYN_T::STATE_DIM);
  Eigen::MatrixXf nom_control = Eigen::MatrixXf::Random(DYN_T::CONTROL_DIM, num_timesteps);
  std::vector<float> nominal_control_seq(nom_control.data(), nom_control.data() + nom_control.size());
  std::vector<float> trajectory_costs_act(num_rollouts);
  std::vector<float> trajectory_costs_nom(num_rollouts);

  // set initial state
  for (size_t i = 0; i < x0.size(); i++)
  {
    x0[i] = i * 0.1 + 0.2;
  }
  launchRolloutKernel_nom_act<DYN_T, COST_T, SAMPLER_T>(model, cost, sampler, dt, num_timesteps, num_rollouts, lambda,
                                                        alpha, x0, nominal_control_seq, trajectory_costs_act,
                                                        trajectory_costs_nom, stream);
  array_assert_float_eq(trajectory_costs_act, trajectory_costs_nom, num_rollouts);
}

TEST_F(RolloutKernelTests, CombinedRolloutKernelGPUvsCPU)
{
  state_array x0 = state_array::Random();

  /**
   * GPU Setup
   **/
  model->GPUSetup();
  cost->GPUSetup();
  sampler->GPUSetup();

  Eigen::MatrixXf control_seq = Eigen::MatrixXf::Random(DYN_T::CONTROL_DIM, num_timesteps * num_rollouts);
  sampler->copyImportanceSamplerToDevice(control_seq.data(), 0, false);

  // Generate samples and do stream synchronize
  logger->debug("Generating samples\n");
  curandGenerator_t gen;
  curandCreateGenerator(&gen, CURAND_RNG_PSEUDO_DEFAULT);
  curandSetStream(gen, stream);
  sampler->generateSamples(1, 0, gen, true);

  Eigen::MatrixXf trajectory_costs_cpu = Eigen::MatrixXf::Zero(num_rollouts, 1);
  Eigen::MatrixXf trajectory_costs_gpu = Eigen::MatrixXf::Zero(num_rollouts, 1);

  logger->debug("Running CPU Rollout\n");
  launchCPURolloutKernel<DYN_T, COST_T, SAMPLER_T>(model, cost, sampler, dt, num_timesteps, num_rollouts, lambda, alpha,
                                                   x0, trajectory_costs_cpu, stream);

  /**
    GPU Computations
    **/
  float* initial_x_d;
  float* trajectory_costs_d;
  HANDLE_ERROR(cudaMalloc((void**)&initial_x_d, sizeof(float) * DYN_T::STATE_DIM));
  HANDLE_ERROR(cudaMalloc((void**)&trajectory_costs_d, sizeof(float) * num_rollouts));
  HANDLE_ERROR(
      cudaMemcpyAsync(initial_x_d, x0.data(), sizeof(float) * DYN_T::STATE_DIM, cudaMemcpyHostToDevice, stream));
  std::vector<int> possible_thread_x;
  for (int i = 64; i > 0; i /= 2)
  {
    possible_thread_x.push_back(i);
  }
  std::vector<int> possible_thread_y = { 1, 2, 3, 4 };

  for (const auto& thread_x : possible_thread_x)
  {
    for (const auto& thread_y : possible_thread_y)
    {
      dim3 threadsPerBlock(thread_x, thread_y, 1);
      logger->debug("Running GPU Combined Rollout on (%d, %d, %d)\n", threadsPerBlock.x, threadsPerBlock.y,
                    threadsPerBlock.z);
      mppi::kernels::launchRolloutKernel<DYN_T, COST_T, SAMPLER_T>(model, cost, sampler, dt, num_timesteps,
                                                                   num_rollouts, lambda, alpha, initial_x_d,
                                                                   trajectory_costs_d, threadsPerBlock, stream, false);
      HANDLE_ERROR(cudaMemcpyAsync(trajectory_costs_gpu.data(), trajectory_costs_d, sizeof(float) * num_rollouts,
                                   cudaMemcpyDeviceToHost, stream));
      HANDLE_ERROR(cudaStreamSynchronize(stream));

      eigen_assert_float_near<Eigen::MatrixXf>(trajectory_costs_cpu, trajectory_costs_gpu, 1e-4f);
    }
  }
}

TEST_F(RolloutKernelTests, SplitRolloutKernelGPUvsCPU)
{
  state_array x0 = state_array::Random();

  /**
   * GPU Setup
   **/
  model->GPUSetup();
  cost->GPUSetup();
  sampler->GPUSetup();

  Eigen::MatrixXf control_seq = Eigen::MatrixXf::Random(DYN_T::CONTROL_DIM, num_timesteps * num_rollouts);
  sampler->copyImportanceSamplerToDevice(control_seq.data(), 0, false);

  // Generate samples and do stream synchronize
  logger->debug("Generating samples\n");
  curandGenerator_t gen;
  curandCreateGenerator(&gen, CURAND_RNG_PSEUDO_DEFAULT);
  curandSetStream(gen, stream);
  sampler->generateSamples(1, 0, gen, true);

  Eigen::MatrixXf trajectory_costs_cpu = Eigen::MatrixXf::Zero(num_rollouts, 1);
  Eigen::MatrixXf trajectory_costs_gpu = Eigen::MatrixXf::Zero(num_rollouts, 1);

  logger->debug("Running CPU Rollout\n");
  launchCPURolloutKernel<DYN_T, COST_T, SAMPLER_T>(model, cost, sampler, dt, num_timesteps, num_rollouts, lambda, alpha,
                                                   x0, trajectory_costs_cpu, stream);

  /**
    GPU Computations
    **/
  float* initial_x_d;
  float* output_d;
  float* trajectory_costs_d;
  HANDLE_ERROR(cudaMalloc((void**)&initial_x_d, sizeof(float) * DYN_T::STATE_DIM));
  HANDLE_ERROR(cudaMalloc((void**)&output_d, sizeof(float) * DYN_T::OUTPUT_DIM * num_rollouts * num_timesteps));
  HANDLE_ERROR(cudaMalloc((void**)&trajectory_costs_d, sizeof(float) * num_rollouts));
  HANDLE_ERROR(
      cudaMemcpyAsync(initial_x_d, x0.data(), sizeof(float) * DYN_T::STATE_DIM, cudaMemcpyHostToDevice, stream));
  std::vector<int> possible_dyn_thread_x;
  for (int i = 128; i > 0; i /= 2)
  {
    possible_dyn_thread_x.push_back(i);
  }
  std::vector<int> possible_dyn_thread_y = { 1, 2, 3, 4 };

  std::vector<int> possible_cost_thread_x;
  for (int i = num_timesteps; i > 0; i /= 2)
  {
    possible_cost_thread_x.push_back(i);
  }
  std::vector<int> possible_cost_thread_y = { 1, 2, 3, 4 };

  for (const auto& dyn_thread_x : possible_dyn_thread_x)
  {
    for (const auto& dyn_thread_y : possible_dyn_thread_y)
    {
      for (const auto& cost_thread_x : possible_cost_thread_x)
      {
        for (const auto& cost_thread_y : possible_cost_thread_y)
        {
          dim3 dynThreadsPerBlock(dyn_thread_x, dyn_thread_y, 1);
          dim3 costThreadsPerBlock(cost_thread_x, cost_thread_y, 1);
          logger->debug("Running coalesced GPU Split Rollout with dyn(%d, %d, %d), cost(%d, %d, %d)\n",
                        dynThreadsPerBlock.x, dynThreadsPerBlock.y, dynThreadsPerBlock.z, costThreadsPerBlock.x,
                        costThreadsPerBlock.y, costThreadsPerBlock.z);
          mppi::kernels::launchSplitRolloutKernel<DYN_T, COST_T, SAMPLER_T, true>(
              model, cost, sampler, dt, num_timesteps, num_rollouts, lambda, alpha, initial_x_d, output_d,
              trajectory_costs_d, dynThreadsPerBlock, costThreadsPerBlock, stream, false);
          HANDLE_ERROR(cudaMemcpyAsync(trajectory_costs_gpu.data(), trajectory_costs_d, sizeof(float) * num_rollouts,
                                       cudaMemcpyDeviceToHost, stream));
          HANDLE_ERROR(cudaStreamSynchronize(stream));
          // eigen_assert_float_near<Eigen::MatrixXf>(trajectory_costs_cpu, trajectory_costs_gpu, 1e-4f);
          for (int i = 0; i < num_rollouts; i++)
          {
            float cost_diff = trajectory_costs_cpu(i) - trajectory_costs_gpu(i);
            std::string error_prefix =
                "Split Rollout sample " + std::to_string(i) + " dyn(" + std::to_string(dynThreadsPerBlock.x) + ", " +
                std::to_string(dynThreadsPerBlock.y) + ", " + std::to_string(dynThreadsPerBlock.z) + +" cost(" +
                std::to_string(costThreadsPerBlock.x) + ", " + std::to_string(costThreadsPerBlock.y) + ", " +
                std::to_string(costThreadsPerBlock.z) + ")";
            ASSERT_LT(fabsf(cost_diff), 1e-3 * trajectory_costs_cpu(i))
                << error_prefix << ": CPU = " << trajectory_costs_cpu(i) << ", GPU = " << trajectory_costs_gpu(i)
                << std::endl;
          }

          logger->debug("Running non-coalesced GPU Split Rollout with dyn(%d, %d, %d), cost(%d, %d, %d)\n",
                        dynThreadsPerBlock.x, dynThreadsPerBlock.y, dynThreadsPerBlock.z, costThreadsPerBlock.x,
                        costThreadsPerBlock.y, costThreadsPerBlock.z);
          mppi::kernels::launchSplitRolloutKernel<DYN_T, COST_T, SAMPLER_T, false>(
              model, cost, sampler, dt, num_timesteps, num_rollouts, lambda, alpha, initial_x_d, output_d,
              trajectory_costs_d, dynThreadsPerBlock, costThreadsPerBlock, stream, false);
          HANDLE_ERROR(cudaMemcpyAsync(trajectory_costs_gpu.data(), trajectory_costs_d, sizeof(float) * num_rollouts,
                                       cudaMemcpyDeviceToHost, stream));
          HANDLE_ERROR(cudaStreamSynchronize(stream));
          for (int i = 0; i < num_rollouts; i++)
          {
            float cost_diff = trajectory_costs_cpu(i) - trajectory_costs_gpu(i);
            std::string error_prefix =
                "Split Rollout sample " + std::to_string(i) + " dyn(" + std::to_string(dynThreadsPerBlock.x) + ", " +
                std::to_string(dynThreadsPerBlock.y) + ", " + std::to_string(dynThreadsPerBlock.z) + +" cost(" +
                std::to_string(costThreadsPerBlock.x) + ", " + std::to_string(costThreadsPerBlock.y) + ", " +
                std::to_string(costThreadsPerBlock.z) + ")";
            ASSERT_LT(fabsf(cost_diff), 1e-3 * trajectory_costs_cpu(i))
                << error_prefix << ": CPU = " << trajectory_costs_cpu(i) << ", GPU = " << trajectory_costs_gpu(i)
                << std::endl;
          }
          // eigen_assert_float_near<Eigen::MatrixXf>(trajectory_costs_cpu, trajectory_costs_gpu, 1e-4f);
        }
      }
    }
  }
}
