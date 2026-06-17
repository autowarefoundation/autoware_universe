//
// Created by jgibson37 on 2/7/20.
//

#include <gtest/gtest.h>
#include <mppi/cost_functions/autorally/ar_robust_cost.cuh>
#include <kernel_tests/cost_functions/autorally/ar_robust_cost_kernel_test.cuh>
#include <kernel_tests/cost_functions/cost_generic_kernel_tests.cuh>

// Auto-generated header file
#include <autorally_test_map.h>

class ARRobustCostTest : public ::testing::Test
{
public:
  ARRobustCost cost;

protected:
  void SetUp() override
  {
    ARRobustCostParams params;
    params.boundary_threshold = 0.0;
    params.crash_coeff = 10000;
    params.track_slop = 0.0;
    params.desired_speed = 10;
    params.speed_coeff = 10;
    params.heading_coeff = 20;
    cost.setParams(params);
  }
  void TearDown() override
  {
  }
};

TEST_F(ARRobustCostTest, Constructor)
{
  // checks for CRTP
  ARRobustCost* robust = cost.cost_d_;
  EXPECT_EQ(robust, nullptr);
}

TEST_F(ARRobustCostTest, GPUSetup)
{
  cudaStream_t stream;
  HANDLE_ERROR(cudaStreamCreate(&stream));

  cost.bindToStream(stream);

  EXPECT_EQ(cost.stream_, stream) << "Stream binding failure.";

  EXPECT_EQ(cost.GPUMemStatus_, false);
  EXPECT_EQ(cost.cost_d_, nullptr);

  cost.GPUSetup();

  EXPECT_EQ(cost.GPUMemStatus_, true);
  EXPECT_NE(cost.cost_d_, nullptr);

  HANDLE_ERROR(cudaStreamDestroy(stream));
}

void checkParameters(ARRobustCostParams& params, ARRobustCostParams& result)
{
  EXPECT_FLOAT_EQ(params.speed_coeff, result.speed_coeff);
  EXPECT_FLOAT_EQ(params.track_coeff, result.track_coeff);
  EXPECT_FLOAT_EQ(params.heading_coeff, result.heading_coeff);
  EXPECT_FLOAT_EQ(params.control_cost_coeff[0], result.control_cost_coeff[0]);
  EXPECT_FLOAT_EQ(params.control_cost_coeff[1], result.control_cost_coeff[1]);
  EXPECT_FLOAT_EQ(params.slip_coeff, result.slip_coeff);
  EXPECT_FLOAT_EQ(params.crash_coeff, result.crash_coeff);
  EXPECT_FLOAT_EQ(params.boundary_threshold, result.boundary_threshold);
  EXPECT_FLOAT_EQ(params.max_slip_ang, result.max_slip_ang);
  EXPECT_FLOAT_EQ(params.track_slop, result.track_slop);
  EXPECT_FLOAT_EQ(params.r_c1.x, result.r_c1.x);
  EXPECT_FLOAT_EQ(params.r_c1.y, result.r_c1.y);
  EXPECT_FLOAT_EQ(params.r_c1.z, result.r_c1.z);
  EXPECT_FLOAT_EQ(params.r_c2.x, result.r_c2.x);
  EXPECT_FLOAT_EQ(params.r_c2.y, result.r_c2.y);
  EXPECT_FLOAT_EQ(params.r_c2.z, result.r_c2.z);
  EXPECT_FLOAT_EQ(params.trs.x, result.trs.x);
  EXPECT_FLOAT_EQ(params.trs.y, result.trs.y);
  EXPECT_FLOAT_EQ(params.trs.z, result.trs.z);
}

TEST_F(ARRobustCostTest, setParams)
{
  ARRobustCostParams params;

  params.speed_coeff = 1.0;
  params.track_coeff = 2.0;
  params.heading_coeff = 3.0;
  params.control_cost_coeff[0] = 4.0;
  params.control_cost_coeff[1] = 5.0;
  params.slip_coeff = 6.0;
  params.crash_coeff = 7.0;
  params.boundary_threshold = 8.0;
  params.max_slip_ang = 9.0;
  params.track_slop = 10.0;
  params.r_c1.x = 12;
  params.r_c1.y = 13;
  params.r_c1.z = 14;
  params.r_c2.x = 15;
  params.r_c2.y = 16;
  params.r_c2.z = 17;
  params.trs.x = 18;
  params.trs.y = 19;
  params.trs.z = 20;

  cost.setParams(params);
  ARRobustCostParams result = cost.getParams();
  checkParameters(params, result);

  cost.GPUSetup();
  int width, height;
  launchParameterTestKernel<>(cost, params, width, height);
  checkParameters(params, result);
}

TEST_F(ARRobustCostTest, getStabilizingCostTest)
{
  ARRobustCostParams params;
  params.max_slip_ang = 1.25;
  params.crash_coeff = 10000;
  params.slip_coeff = 10;
  cost.setParams(params);

  float s[7];
  for (int i = 0; i < 7; i++)
  {
    s[i] = 0;
  }
  s[4] = 0.24;
  s[5] = 0.0;
  float result = cost.getStabilizingCost(s);
  EXPECT_FLOAT_EQ(result, 0.0);

  s[4] = 1.0;
  s[5] = 1.0;
  result = cost.getStabilizingCost(s);
  EXPECT_FLOAT_EQ(result, 0.785398 * 10);
  // EXPECT_FLOAT_EQ(result, fabs(atan(s[5]/s[4])) * params.slip_coeff + params.crash_coeff);

  s[4] = 1.0;
  s[5] = 10.0;
  result = cost.getStabilizingCost(s);
  EXPECT_FLOAT_EQ(result, 1.4711 * 10 + params.crash_coeff);

  s[3] = 1.5;
  s[4] = 1.0;
  s[5] = 10.0;
  result = cost.getStabilizingCost(s);
  EXPECT_FLOAT_EQ(result, 1.4711 * 10 + params.crash_coeff);
}

float calculateRobustCostmapValue(ARRobustCost& cost, float3 state, int width, int height, float x_min, float x_max,
                                  float y_min, float y_max, int ppm)
{
  float x_front = state.x + cost.FRONT_D * cosf(state.z);
  float y_front = state.y + cost.FRONT_D * sinf(state.z);
  float x_back = state.x + cost.BACK_D * cosf(state.z);
  float y_back = state.y + cost.BACK_D * sinf(state.z);

  // check for overflow
  float new_x = max(min(x_front - x_min, x_max - x_min), 0.0f);
  float new_y = max(min(y_front - y_min, y_max - y_min), 0.0f);

  // calculate the track value
  float front_track = fabs(height / 2.0f - (new_y)) + (new_x) / width;
  std::cout << "front point = " << new_x << ", " << new_y << " = " << front_track << std::endl;

  new_x = max(min(x_back - x_min + 1.0 / (width * ppm), x_max - x_min), 0.0f);
  new_y = max(min(y_back - y_min + 1.0 / (height * ppm), y_max - y_min), 0.0f);

  float back_track = fabs(height / 2.0f - (new_y)) + (new_x) / width;
  std::cout << "back point = " << new_x << ", " << new_y << " = " << back_track << std::endl;
  return (front_track + back_track) / 2.0f;
}

TEST_F(ARRobustCostTest, getCostmapCostSpeedMapTest)
{
  ARRobustCostParams params = cost.getParams();
  params.desired_speed = -1;
  cost.setParams(params);

  cost.GPUSetup();
  cost.loadTrackData(mppi::tests::robust_test_map_file);

  std::vector<std::array<float, 9>> states;

  std::array<float, 9> array = { 0.0 };
  array[0] = 3.0;     // X
  array[1] = 0.0;     // Y
  array[2] = M_PI_2;  // Theta
  array[3] = 0.0;     // Roll
  array[4] = 2.0;     // Vx
  array[5] = 1.0;     // Vy
  array[6] = 0.1;     // Yaw dot
  array[7] = 0.5;     // steering
  array[8] = 0.3;     // throttle
  states.push_back(array);

  std::vector<float> cost_results;

  launchGetCostmapCostTestKernel(cost, states, cost_results);

  EXPECT_FLOAT_EQ(cost_results[0], 11629.229);
}

TEST_F(ARRobustCostTest, getCostmapCostSpeedNoMapTest)
{
  cost.GPUSetup();
  cost.loadTrackData(mppi::tests::robust_test_map_file);

  std::vector<std::array<float, 9>> states;

  std::array<float, 9> array = { 0.0 };
  array[0] = 3.0;     // X
  array[1] = 0.0;     // Y
  array[2] = M_PI_2;  // Theta
  array[3] = 0.0;     // Roll
  array[4] = 2.0;     // Vx
  array[5] = 1.0;     // Vy
  array[6] = 0.1;     // Yaw dot
  array[7] = 0.5;     // steering
  array[8] = 0.3;     // throttle
  states.push_back(array);

  std::vector<float> cost_results;

  launchGetCostmapCostTestKernel(cost, states, cost_results);

  EXPECT_FLOAT_EQ(cost_results[0], 11349.729);
}

TEST_F(ARRobustCostTest, computeCostTest)
{
  cost.GPUSetup();
  cost.loadTrackData(mppi::tests::robust_test_map_file);

  checkGPURolloutCost(cost, 0.02);

  // std::vector<std::array<float, 9>> states;

  // std::array<float, 9> array = { 0.0 };
  // array[0] = 3.0;     // X
  // array[1] = 0.0;     // Y
  // array[2] = M_PI_2;  // Theta
  // array[3] = 0.0;     // Roll
  // array[4] = 2.0;     // Vx
  // array[5] = 1.0;     // Vy
  // array[6] = 0.1;     // Yaw dot
  // array[7] = 0.5;     // steering
  // array[8] = 0.3;     // throttle
  // states.push_back(array);

  // std::vector<float> cost_results;

  // launchComputeCostTestKernel<>(cost, states, cost_results);

  // EXPECT_FLOAT_EQ(cost_results[0], 11349.729);
}
