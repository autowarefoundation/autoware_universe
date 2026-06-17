#include <gtest/gtest.h>
#include <kernel_tests/core/weightedreduction_kernel_test.cuh>
#include <mppi/utils/test_helper.h>
#include <random>

TEST(WeightedReductionKernel, setInitialControlToZero)
{
  const int num_threads = 100;
  const int control_dim = 5;

  std::array<float, control_dim> u_host = { 1 };
  std::array<float, num_threads* control_dim> u_intermediate_host = { 1 };

  launchSetInitialControlToZero_KernelTest<num_threads, control_dim>(u_host, u_intermediate_host);

  array_assert_float_eq<control_dim>(0.f, u_host);
  array_assert_float_eq<control_dim * num_threads>(0.f, u_intermediate_host);
}

TEST(WeightedReductionKernel, strideControlWeightReduction)
{
  auto generator = std::default_random_engine(7.0);
  auto distribution = std::normal_distribution<float>(1.0, 0.2);
  const int sum_stride = 64;
  float normalizer = 1000;

  const int num_rollouts = 1024;
  const int control_dim = 6;
  const int num_timesteps = 100;

  std::array<float, num_rollouts> exp_costs_host = { 1 };
  std::array<float, num_rollouts* num_timesteps* control_dim> v_host = { 1 };  // Disturbed control

  for (size_t i = 1; i < exp_costs_host.size(); ++i)
  {
    exp_costs_host[i] = 0.001 * distribution(generator);
  }

  for (size_t i = 1; i < v_host.size(); ++i)
  {
    v_host[i] = distribution(generator);
  }

  const int cell_size = ((num_rollouts - 1) / sum_stride + 1);
  std::array<float, cell_size* num_timesteps* control_dim> u_intermediate_host = { 0 };

  launchStrideControlWeightReduction_KernelTest<control_dim, num_rollouts, num_timesteps, sum_stride>(
      normalizer, exp_costs_host, v_host, u_intermediate_host);
  CudaCheckError();

  std::array<float, num_timesteps*((num_rollouts - 1) / sum_stride + 1)* control_dim> u_intermediate_known = { 0 };

  // Compute weights per rollouts
  std::array<float, num_rollouts> rollout_weights = { 0 };
  for (size_t i = 0; i < rollout_weights.size(); ++i)
  {
    rollout_weights[i] = exp_costs_host[i] / normalizer;
  }

  // Weight all the controls with the appropriate cost
  std::array<float, num_timesteps* num_rollouts* control_dim> weighted_controls = { 0 };
  for (int i = 0; i < num_rollouts; ++i)
  {
    for (int j = 0; j < num_timesteps; ++j)
    {
      for (int k = 0; k < control_dim; ++k)
      {
        int index = i * num_timesteps * control_dim + j * control_dim + k;
        weighted_controls[index] = rollout_weights[i] * v_host[index];
      }
    }
  }

  // Iterate through each stride
  for (int i = 0; i < ((num_rollouts - 1) / sum_stride + 1); ++i)
  {  // i is the cell index
    for (int j = 0; j < sum_stride; ++j)
    {  // j is the stride index
      for (int k = 0; k < num_timesteps; ++k)
      {  // k specifies the current timestep
        for (int w = 0; w < control_dim; ++w)
        {
          int rollout_index = (i * sum_stride + j);
          if (rollout_index < num_rollouts)
          {
            u_intermediate_known[k * cell_size * control_dim + i * control_dim + w] +=
                weighted_controls[rollout_index * num_timesteps * control_dim + k * control_dim + w];
          }
        }
      }
    }
  }

  array_assert_float_eq<cell_size * num_timesteps * control_dim>(u_intermediate_known, u_intermediate_host);
}

TEST(WeightedReductionKernel, rolloutWeightReductionAndSaveControl)
{
  auto generator = std::default_random_engine(7.0);
  auto distribution = std::normal_distribution<float>(1.0, 0.2);
  const int sum_stride = 64;

  const int num_rollouts = 1024;
  const int control_dim = 6;
  const int num_timesteps = 100;
  const int cell_size = ((num_rollouts - 1) / sum_stride + 1);
  std::array<float, cell_size* num_timesteps* control_dim> u_intermediate_host = { 0 };
  std::array<float, num_timesteps* control_dim> du_new_compute = { 0 };  // Update control
  std::array<float, num_timesteps* control_dim> du_new_known = { 0 };

  for (size_t i = 0; i < u_intermediate_host.size(); ++i)
  {
    u_intermediate_host[i] = distribution(generator);
  }

  // Compute the control update
  for (int i = 0; i < ((num_rollouts - 1) / sum_stride + 1); ++i)
  {  // i is the cell index
    for (int k = 0; k < num_timesteps; ++k)
    {  // k specifies the current timestep
      for (int w = 0; w < control_dim; ++w)
      {
        du_new_known[k * control_dim + w] += u_intermediate_host[k * cell_size * control_dim + i * control_dim + w];
      }
    }
  }

  // Launch the test kernel
  launchRolloutWeightReductionAndSaveControl_KernelTest<control_dim, num_rollouts, num_timesteps, sum_stride>(
      u_intermediate_host, du_new_compute);

  array_assert_float_eq<num_timesteps * control_dim>(du_new_known, du_new_compute);
}

TEST(WeightedReductionKernel, comparisonTestAutorallyMPPI_Generic)
{
  auto generator = std::default_random_engine(7.0);
  auto distribution = std::normal_distribution<float>(5.0, 1.2);
  const int sum_stride = 64;

  const int num_rollouts = 1024;
  const int control_dim = 4;
  const int num_timesteps = 100;

  std::array<float, num_rollouts> exp_costs;
  std::array<float, control_dim * num_rollouts * num_timesteps> perturbed_controls;
  std::array<float, control_dim * num_timesteps> controls_out_autorally;
  std::array<float, control_dim * num_timesteps> controls_out_mppi_generic;

  // Initialize the exp costs with positive numbers
  for (float& exp_cost : exp_costs)
  {
    exp_cost = expf(-1 * distribution(generator));
  }
  exp_costs[0] = 0;  // Minimum cost

  // Initialize the control perturbations with random numbers
  for (float& control : perturbed_controls)
  {
    control = distribution(generator);
  }

  // The normalizer is the sum of all exponential costs;
  float normalizer = std::accumulate(exp_costs.begin(), exp_costs.end(), 0.0);

  launchWeightedReductionKernelTest<control_dim, num_rollouts, sum_stride, num_timesteps>(
      exp_costs, perturbed_controls, normalizer, controls_out_mppi_generic, 0);

  launchAutoRallyWeightedReductionKernelTest<control_dim, num_rollouts, sum_stride, num_timesteps>(
      exp_costs, perturbed_controls, normalizer, controls_out_autorally, 0);

  array_expect_float_eq<num_timesteps * control_dim>(controls_out_mppi_generic, controls_out_autorally);
}
