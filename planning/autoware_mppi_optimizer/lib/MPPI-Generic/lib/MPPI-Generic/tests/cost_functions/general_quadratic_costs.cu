#include <gtest/gtest.h>
#include <mppi/cost_functions/quadratic_cost/quadratic_cost.cuh>
#include <mppi/dynamics/double_integrator/di_dynamics.cuh>

typedef QuadraticCost<DoubleIntegratorDynamics> DIQuadCost;

TEST(DIQuadraticCost, Constructor)
{
  DIQuadCost();
}

template <class COST_T>
__global__ void computeCostKernel(COST_T* cost, float* s, int* t, float* result)
{
  int crash = 0;
  *result = cost->computeStateCost(s, *t, nullptr, &crash);
}

class DITargetQuadraticCost : public testing::Test
{
public:
  using DYN = DoubleIntegratorDynamics;
  using COST = QuadraticCost<DYN>;
  DIQuadCost cost;
  DIQuadCost::COST_PARAMS_T cost_params;
  DYN::output_array s;
  dim3 dimBlock;
  dim3 dimGrid;
  void SetUp() override
  {
    cost_params = cost.getParams();
    cost_params.s_goal[0] = 1;
    cost_params.s_goal[1] = -5;
    cost_params.s_goal[2] = 0;
    cost_params.s_goal[3] = 0.5;
    cost.setParams(cost_params);

    s << 0, 0, 0, 0;
    dimBlock = dim3(10, 11, 5);
    dimGrid = dim3(1, 1, 1);
  }
};

class DITrajQuadraticCost : public testing::Test
{
public:
  using DYN = DoubleIntegratorDynamics;
  static const int TIME_HORIZON = 5;
  using COST = QuadraticCostTrajectory<DYN, TIME_HORIZON>;
  COST cost;
  COST::COST_PARAMS_T cost_params;
  float s_traj[28] = { 1, -5, 0, 0.5, 2, -5.5, 0, 1, 5, 5, 1, 10, 1, 1, 1, 1, 2, 3, 4, 5, 11, -5, 3, 8, 8, 8, 8, 8 };
  DYN::output_array s;
  // GPU Variables
  dim3 dimBlock;
  dim3 dimGrid;
  float* s_dev;
  float* resulting_cost_d;
  int* time_d;
  cudaStream_t stream;

  void SetUp() override
  {
    cost_params = cost.getParams();
    for (int i = 0; i < TIME_HORIZON * DYN::OUTPUT_DIM; i++)
    {
      cost_params.s_goal[i] = s_traj[i];
    }
    // cost_params.s_goal[0] = 1;
    // cost_params.s_goal[1] = -5;
    // cost_params.s_goal[2] = 0;
    // cost_params.s_goal[3] = 0.5;
    cost.setParams(cost_params);

    s << 0, 0, 0, 0;

    dimBlock = dim3(10, 11, 5);
    dimGrid = dim3(1, 1, 1);
    HANDLE_ERROR(cudaMalloc((void**)&s_dev, sizeof(float) * DYN::OUTPUT_DIM));
    HANDLE_ERROR(cudaMalloc((void**)&resulting_cost_d, sizeof(float)));
    HANDLE_ERROR(cudaMalloc((void**)&time_d, sizeof(int)));

    HANDLE_ERROR(cudaStreamCreate(&stream));
    cost.bindToStream(stream);
    cost.GPUSetup();
  }

  void TearDown() override
  {
    HANDLE_ERROR(cudaFree(s_dev));
    HANDLE_ERROR(cudaFree(resulting_cost_d));
    HANDLE_ERROR(cudaFree(time_d));
  }
};

TEST_F(DITargetQuadraticCost, SimpleStateCostCPU)
{
  float state_cost = cost.computeStateCost(s);
  ASSERT_FLOAT_EQ(state_cost, 26.25);
}

TEST_F(DITargetQuadraticCost, LateStateCostCPU)
{
  float state_cost = cost.computeStateCost(s, 10);
  ASSERT_FLOAT_EQ(state_cost, 26.25);
}

TEST_F(DITargetQuadraticCost, WeightStateCostCPU)
{
  cost_params.s_coeffs[0] = 10;
  cost_params.s_coeffs[1] = 5;
  cost_params.s_coeffs[2] = 7;
  cost_params.s_coeffs[3] = .1;
  cost.setParams(cost_params);
  float state_cost = cost.computeStateCost(s);
  ASSERT_FLOAT_EQ(state_cost, 10 + 5 * 5 * 5 + 0 + 0.025);
}

TEST_F(DITargetQuadraticCost, LipschitzConstant)
{
  cost_params.s_coeffs[0] = -10;
  cost_params.s_coeffs[1] = 5;
  cost_params.s_coeffs[2] = 7;
  cost_params.s_coeffs[3] = .1;
  cost.setParams(cost_params);

  float lipschitz_constant = cost.getLipshitzConstantCost();
  ASSERT_FLOAT_EQ(lipschitz_constant, 20);
}

TEST_F(DITargetQuadraticCost, StateCostGPU)
{
  cudaStream_t stream;
  HANDLE_ERROR(cudaStreamCreate(&stream));
  cost.bindToStream(stream);
  cost.GPUSetup();
  int time = 0;
  float result_cost_h = -1;

  // Setup GPU params
  float* s_dev;
  float* resulting_cost_d;
  int* time_d;
  HANDLE_ERROR(cudaMalloc((void**)&s_dev, sizeof(float) * DYN::OUTPUT_DIM));
  HANDLE_ERROR(cudaMalloc((void**)&resulting_cost_d, sizeof(float)));
  HANDLE_ERROR(cudaMalloc((void**)&time_d, sizeof(int)));

  HANDLE_ERROR(cudaMemcpyAsync(s_dev, s.data(), sizeof(float) * DYN::OUTPUT_DIM, cudaMemcpyHostToDevice, stream));
  HANDLE_ERROR(cudaMemcpyAsync(time_d, &time, sizeof(int), cudaMemcpyHostToDevice, stream));
  computeCostKernel<COST><<<dimGrid, dimBlock, 0, stream>>>(cost.cost_d_, s_dev, time_d, resulting_cost_d);

  HANDLE_ERROR(cudaMemcpyAsync(&result_cost_h, resulting_cost_d, sizeof(float), cudaMemcpyDeviceToHost, stream));
  HANDLE_ERROR(cudaStreamSynchronize(stream));
  ASSERT_FLOAT_EQ(result_cost_h, 26.25);
}

TEST_F(DITargetQuadraticCost, LateStateCostGPU)
{
  cudaStream_t stream;
  HANDLE_ERROR(cudaStreamCreate(&stream));
  cost.bindToStream(stream);
  cost.GPUSetup();
  int time = 10;
  float result_cost_h = -1;

  // Setup GPU params
  float* s_dev;
  float* resulting_cost_d;
  int* time_d;
  HANDLE_ERROR(cudaMalloc((void**)&s_dev, sizeof(float) * DYN::OUTPUT_DIM));
  HANDLE_ERROR(cudaMalloc((void**)&resulting_cost_d, sizeof(float)));
  HANDLE_ERROR(cudaMalloc((void**)&time_d, sizeof(int)));

  HANDLE_ERROR(cudaMemcpyAsync(s_dev, s.data(), sizeof(float) * DYN::OUTPUT_DIM, cudaMemcpyHostToDevice, stream));
  HANDLE_ERROR(cudaMemcpyAsync(time_d, &time, sizeof(int), cudaMemcpyHostToDevice, stream));
  computeCostKernel<COST><<<dimGrid, dimBlock, 0, stream>>>(cost.cost_d_, s_dev, time_d, resulting_cost_d);

  HANDLE_ERROR(cudaMemcpyAsync(&result_cost_h, resulting_cost_d, sizeof(float), cudaMemcpyDeviceToHost, stream));
  HANDLE_ERROR(cudaStreamSynchronize(stream));
  ASSERT_FLOAT_EQ(result_cost_h, 26.25);
}

TEST_F(DITrajQuadraticCost, MidTrajStateCostGPU)
{
  int time = 2;
  float result_cost_h = -1;

  // Setup GPU params
  HANDLE_ERROR(cudaMemcpyAsync(s_dev, s.data(), sizeof(float) * DYN::OUTPUT_DIM, cudaMemcpyHostToDevice, stream));
  HANDLE_ERROR(cudaMemcpyAsync(time_d, &time, sizeof(int), cudaMemcpyHostToDevice, stream));
  computeCostKernel<COST><<<dimGrid, dimBlock, 0, stream>>>(cost.cost_d_, s_dev, time_d, resulting_cost_d);

  HANDLE_ERROR(cudaMemcpyAsync(&result_cost_h, resulting_cost_d, sizeof(float), cudaMemcpyDeviceToHost, stream));
  HANDLE_ERROR(cudaStreamSynchronize(stream));
  float ground_truth = powf(cost_params.s_goal[8], 2) + powf(cost_params.s_goal[9], 2) +
                       powf(cost_params.s_goal[10], 2) + powf(cost_params.s_goal[11], 2);

  ASSERT_FLOAT_EQ(result_cost_h, ground_truth);
}
