#include <gtest/gtest.h>
#include <mppi/dynamics/cartpole/cartpole_dynamics.cuh>
#include <mppi/cost_functions/cartpole/cartpole_quadratic_cost.cuh>

#include <mppi/utils/test_helper.h>
#include <random>
#include <algorithm>

#include <mppi/shaping_functions/CEM/cem_shaping_function.cuh>
#include <kernel_tests/shaping_functions/shaping_function_kernels_tests.cuh>

class CEMShapingFunctionTest : public testing::Test
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

TEST_F(CEMShapingFunctionTest, computeWeightTest)
{
  const int num_rollouts = 500;
  std::array<float, num_rollouts> cost_vec = { 0 };
  CEMShapingFunctionParams params;
  CEMShapingFunction<num_rollouts, 1> shaping_function;

  // Use a range based for loop to set the cost
  for (auto& cost : cost_vec)
  {
    cost = distribution(generator);
  }

  float target_cost = 99;

  EXPECT_FLOAT_EQ(shaping_function.getNormalizer(), 1.0);

  for (float gamma = 0.0; gamma <= 1.0; gamma += 0.1)
  {
    params.gamma = gamma;
    shaping_function.setParams(params);

    // int index = (int) num_rollouts*gamma;
    for (int i = 0; i < cost_vec.size(); i++)
    {
      float weight = shaping_function.computeWeight(cost_vec.data(), target_cost, i);
      if (cost_vec[i] > target_cost)
      {
        EXPECT_FLOAT_EQ(weight, 1.0 / ((int)num_rollouts * params.gamma + 1));
      }
      else
      {
        EXPECT_FLOAT_EQ(weight, 0.0);
      }
    }
  }
}

TEST_F(CEMShapingFunctionTest, weightKernelTest)
{
  const int num_rollouts = 500;
  std::array<float, num_rollouts> cost_vec = { 0 };
  std::array<float, num_rollouts> result_cost_vec = { 0 };
  CEMShapingFunctionParams params;
  CEMShapingFunction<num_rollouts, 1> shaping_function;
  shaping_function.GPUSetup();

  // Use a range based for loop to set the cost
  for (auto& cost : cost_vec)
  {
    cost = distribution(generator);
  }

  float baseline = 99;

  for (float gamma = 0.0; gamma <= 1.0; gamma += 0.1)
  {
    params.gamma = gamma;
    shaping_function.setParams(params);
    launchShapingFunction_KernelTest<CEMShapingFunction<num_rollouts, 1>, num_rollouts, 1>(cost_vec, shaping_function,
                                                                                           baseline, result_cost_vec);
    for (int i = 0; i < cost_vec.size(); i++)
    {
      if (cost_vec[i] > baseline)
      {
        EXPECT_FLOAT_EQ(result_cost_vec[i], 1.0 / ((int)num_rollouts * params.gamma + 1));
      }
      else
      {
        EXPECT_FLOAT_EQ(result_cost_vec[i], 0.0);
      }
    }
  }
}

/*
TEST_F(ShapingFunctionTest, launchWeightKernelTest) {
const int num_rollouts = 500;
ShapingFunction<num_rollouts, 1>::cost_traj cost_traj;
std::array<float, num_rollouts> result_cost_vec = {0};

ShapingFunctionParams params;
ShapingFunction<num_rollouts, 1> shaping_function;
shaping_function.GPUSetup();

// Use a range based for loop to set the cost
cost_traj = ShapingFunction<num_rollouts, 1>::cost_traj::Zero();
for (int i = 0; i < num_rollouts; i++) {
cost_traj(i) = distribution(generator);
}

float min_cost_known = cost_traj.minCoeff();

for (float lambda_inv = 0.01; lambda_inv < 3.0; lambda_inv += 0.1) {
params.lambda_inv = lambda_inv;
shaping_function.setParams(params);
launchShapingFunction_KernelTest<ShapingFunction<num_rollouts, 1>, num_rollouts>(cost_traj, shaping_function,
min_cost_known, result_cost_vec); for (int i = 0; i < result_cost_vec.size(); i++) { EXPECT_FLOAT_EQ(result_cost_vec[i],
expf(-lambda_inv * (cost_traj(i) - min_cost_known)));
}
}
}
 */

TEST_F(CEMShapingFunctionTest, computeWeightsTest)
{
  const int num_rollouts = 500;
  CEMShapingFunction<num_rollouts, 1>::cost_traj cost_traj;
  CEMShapingFunction<num_rollouts, 1>::cost_traj cost_traj_copy;

  CEMShapingFunctionParams params;
  CEMShapingFunction<num_rollouts, 1> shaping_function;
  shaping_function.GPUSetup();

  // Use a range based for loop to set the cost
  cost_traj = ShapingFunction<num_rollouts, 1>::cost_traj::Zero();
  cost_traj_copy = ShapingFunction<num_rollouts, 1>::cost_traj::Zero();
  const float gamma_total = 1.0;
  const float gamma_inc = 0.1;
  const float gamma_start = 0.0;

  int index = 0;
  for (float gamma = gamma_start; gamma <= gamma_total; gamma += gamma_inc)
  {
    // create the random vector
    for (int i = 0; i < num_rollouts; i++)
    {
      cost_traj(i) = distribution(generator);
    }

    // find the correct costs for the baseline
    std::sort(cost_traj.data(), cost_traj.data() + num_rollouts, std::greater<float>());
    float pivot = cost_traj[gamma * num_rollouts];
    std::random_shuffle(cost_traj.data(), cost_traj.data() + num_rollouts);
    for (int i = 0; i < num_rollouts; i++)
    {
      cost_traj_copy(i) = cost_traj(i);
    }

    float* trajectory_costs_d;
    HANDLE_ERROR(cudaMalloc((void**)&trajectory_costs_d, sizeof(float) * num_rollouts));
    HANDLE_ERROR(
        cudaMemcpy(trajectory_costs_d, cost_traj.data(), sizeof(float) * cost_traj.size(), cudaMemcpyHostToDevice))

    params.gamma = gamma;
    shaping_function.setParams(params);

    shaping_function.computeWeights(cost_traj, trajectory_costs_d);

    EXPECT_FLOAT_EQ(shaping_function.getBaseline(), pivot);
    if (gamma == 0)
    {
      EXPECT_FLOAT_EQ(shaping_function.getBaseline(), cost_traj_copy.maxCoeff());
    }
    if (gamma == 1.0)
    {
      EXPECT_FLOAT_EQ(shaping_function.getBaseline(), cost_traj_copy.minCoeff());
    }
    EXPECT_EQ(shaping_function.getNormalizer(), 1.0);
    EXPECT_NEAR(cost_traj.sum(), 1.0, 1e-5) << cost_traj.sum();
    for (int i = 0; i < cost_traj.size(); i++)
    {
      if (cost_traj_copy(i) >= pivot)
      {
        EXPECT_FLOAT_EQ(cost_traj(i), 1.0 / ((int)num_rollouts * params.gamma + 1));
      }
      else
      {
        EXPECT_FLOAT_EQ(cost_traj(i), 0.0);
      }
    }
    index++;
  }
}
