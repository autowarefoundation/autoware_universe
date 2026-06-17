#include <gtest/gtest.h>
#include <mppi/dynamics/cartpole/cartpole_dynamics.cuh>
#include <mppi/cost_functions/cartpole/cartpole_quadratic_cost.cuh>

#include <mppi/utils/test_helper.h>
#include <random>

#include <mppi/shaping_functions/shaping_function.cuh>

#include <kernel_tests/shaping_functions/shaping_function_kernels_tests.cuh>

class ShapingFunctionTest : public testing::Test
{
protected:
  void SetUp() override
  {
    generator = std::default_random_engine(7.0);
    distribution = std::normal_distribution<float>(100.0, 2.0);
  }

  void TearDown() override
  {
  }

  std::default_random_engine generator;
  std::normal_distribution<float> distribution;
};

TEST_F(ShapingFunctionTest, computeWeightTest)
{
  const int num_rollouts = 500;
  std::array<float, num_rollouts> cost_vec = { 0 };
  ShapingFunctionParams params;
  ShapingFunction<num_rollouts, 1> shaping_function;

  // Use a range based for loop to set the cost
  for (auto& cost : cost_vec)
  {
    cost = distribution(generator);
  }

  float min_cost_known = *std::min_element(cost_vec.begin(), cost_vec.end());

  for (float lambda_inv = 0.0; lambda_inv < 3.0; lambda_inv += 0.1)
  {
    params.lambda_inv = lambda_inv;
    shaping_function.setParams(params);
    for (int i = 0; i < cost_vec.size(); i++)
    {
      float weight = shaping_function.computeWeight(cost_vec.data(), min_cost_known, i);
      EXPECT_FLOAT_EQ(weight, expf(-lambda_inv * (cost_vec[i] - min_cost_known)));
    }
  }
}

TEST_F(ShapingFunctionTest, weightKernelTest)
{
  const int num_rollouts = 500;
  std::array<float, num_rollouts> cost_vec = { 0 };
  std::array<float, num_rollouts> result_cost_vec = { 0 };
  ShapingFunctionParams params;
  ShapingFunction<num_rollouts, 1> shaping_function;
  shaping_function.GPUSetup();

  // Use a range based for loop to set the cost
  for (auto& cost : cost_vec)
  {
    cost = distribution(generator);
  }

  float min_cost_known = *std::min_element(cost_vec.begin(), cost_vec.end());

  for (float lambda_inv = 0.01; lambda_inv < 3.0; lambda_inv += 0.1)
  {
    params.lambda_inv = lambda_inv;
    shaping_function.setParams(params);
    launchShapingFunction_KernelTest<ShapingFunction<num_rollouts, 1>, num_rollouts, 1>(
        cost_vec, shaping_function, min_cost_known, result_cost_vec);
    for (int i = 0; i < cost_vec.size(); i++)
    {
      EXPECT_FLOAT_EQ(result_cost_vec[i], expf(-lambda_inv * (cost_vec[i] - min_cost_known)));
    }
  }
}

TEST_F(ShapingFunctionTest, launchWeightKernelTest)
{
  const int num_rollouts = 500;
  ShapingFunction<num_rollouts, 1>::cost_traj cost_traj;
  std::array<float, num_rollouts> result_cost_vec = { 0 };

  ShapingFunctionParams params;
  ShapingFunction<num_rollouts, 1> shaping_function;
  shaping_function.GPUSetup();

  // Use a range based for loop to set the cost
  cost_traj = ShapingFunction<num_rollouts, 1>::cost_traj::Zero();
  for (int i = 0; i < num_rollouts; i++)
  {
    cost_traj(i) = distribution(generator);
  }

  float min_cost_known = cost_traj.minCoeff();

  for (float lambda_inv = 0.01; lambda_inv < 3.0; lambda_inv += 0.1)
  {
    params.lambda_inv = lambda_inv;
    shaping_function.setParams(params);
    launchShapingFunction_KernelTest<ShapingFunction<num_rollouts, 1>, num_rollouts>(cost_traj, shaping_function,
                                                                                     min_cost_known, result_cost_vec);
    for (int i = 0; i < result_cost_vec.size(); i++)
    {
      EXPECT_FLOAT_EQ(result_cost_vec[i], expf(-lambda_inv * (cost_traj(i) - min_cost_known)));
    }
  }
}

TEST_F(ShapingFunctionTest, computeWeightsTest)
{
  const int num_rollouts = 500;
  ShapingFunction<num_rollouts, 1>::cost_traj cost_traj;
  ShapingFunction<num_rollouts, 1>::cost_traj result_cost_traj;

  ShapingFunctionParams params;
  ShapingFunction<num_rollouts, 1> shaping_function;
  shaping_function.GPUSetup();

  // Use a range based for loop to set the cost
  cost_traj = ShapingFunction<num_rollouts, 1>::cost_traj::Zero();
  for (int i = 0; i < num_rollouts; i++)
  {
    cost_traj(i) = distribution(generator);
    result_cost_traj(i) = cost_traj(i);
  }

  float* trajectory_costs_d;
  HANDLE_ERROR(cudaMalloc((void**)&trajectory_costs_d, sizeof(float) * num_rollouts));
  HANDLE_ERROR(
      cudaMemcpy(trajectory_costs_d, cost_traj.data(), sizeof(float) * cost_traj.size(), cudaMemcpyHostToDevice))

  for (float lambda_inv = 0.01; lambda_inv < 3.0; lambda_inv += 0.1)
  {
    float min_cost_known = cost_traj.minCoeff();

    params.lambda_inv = lambda_inv;
    shaping_function.setParams(params);
    shaping_function.computeWeights(cost_traj, trajectory_costs_d);
    float normalizer = cost_traj.sum();

    EXPECT_EQ(shaping_function.getBaseline(), min_cost_known);
    EXPECT_EQ(shaping_function.getNormalizer(), normalizer);
    for (int i = 0; i < cost_traj.size(); i++)
    {
      EXPECT_FLOAT_EQ(cost_traj(i), expf(-lambda_inv * (result_cost_traj(i) - min_cost_known)));
      result_cost_traj(i) = cost_traj(i);
    }
  }
}
