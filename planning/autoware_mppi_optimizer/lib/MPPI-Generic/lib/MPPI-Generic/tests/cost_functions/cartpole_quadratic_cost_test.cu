#include <gtest/gtest.h>
#include <mppi/cost_functions/cartpole/cartpole_quadratic_cost.cuh>
#include <kernel_tests/cost_functions/cartpole/cartpole_quadratic_cost_kernel_test.cuh>
#include <array>

TEST(CartpoleQuadraticCost, Constructor)
{
  CartpoleQuadraticCost cost;

  // Test passes if the object is constructed without runetime error.
}

TEST(CartpoleQuadraticCost, BindStream)
{
  cudaStream_t stream;
  HANDLE_ERROR(cudaStreamCreate(&stream));
  CartpoleQuadraticCost cost(stream);
  EXPECT_EQ(cost.stream_, stream) << "Stream binding failure.";
  HANDLE_ERROR(cudaStreamDestroy(stream));
}

TEST(CartpoleQuadraticCost, GPUMemoryNull)
{
  CartpoleQuadraticCost cost;
  ASSERT_EQ(cost.cost_d_, nullptr);
}

TEST(CartpoleQuadraticCost, GPUSetup)
{
  CartpoleQuadraticCost cost;
  cost.GPUSetup();

  ASSERT_FALSE(cost.cost_d_ == nullptr);
}

TEST(CartpoleQuadraticCost, SetParamsCPU)
{
  CartpoleQuadraticCostParams new_params;
  new_params.cart_position_coeff = 3;
  new_params.pole_angle_coeff = 3;
  new_params.cart_velocity_coeff = 3;
  new_params.pole_angular_velocity_coeff = 3;
  new_params.control_cost_coeff[0] = 5;
  new_params.terminal_cost_coeff = 20;
  new_params.desired_terminal_state[0] = 3;
  new_params.desired_terminal_state[1] = 2;
  new_params.desired_terminal_state[2] = 3.14;
  new_params.desired_terminal_state[3] = 1;

  CartpoleQuadraticCost cost;

  cost.setParams(new_params);
  auto current_params = cost.getParams();

  EXPECT_FLOAT_EQ(new_params.cart_position_coeff, current_params.cart_position_coeff);
  EXPECT_FLOAT_EQ(new_params.pole_angle_coeff, current_params.pole_angle_coeff);
  EXPECT_FLOAT_EQ(new_params.cart_velocity_coeff, current_params.cart_velocity_coeff);
  EXPECT_FLOAT_EQ(new_params.pole_angular_velocity_coeff, current_params.pole_angular_velocity_coeff);
  EXPECT_FLOAT_EQ(new_params.control_cost_coeff[0], current_params.control_cost_coeff[0]);
  EXPECT_FLOAT_EQ(new_params.terminal_cost_coeff, current_params.terminal_cost_coeff);
  EXPECT_FLOAT_EQ(new_params.desired_terminal_state[0], current_params.desired_terminal_state[0]);
  EXPECT_FLOAT_EQ(new_params.desired_terminal_state[1], current_params.desired_terminal_state[1]);
  EXPECT_FLOAT_EQ(new_params.desired_terminal_state[2], current_params.desired_terminal_state[2]);
  EXPECT_FLOAT_EQ(new_params.desired_terminal_state[3], current_params.desired_terminal_state[3]);
}

TEST(CartpoleQuadraticCost, SetParamsGPU)
{
  CartpoleQuadraticCost cost;
  cost.GPUSetup();

  CartpoleQuadraticCostParams new_params;
  new_params.cart_position_coeff = 5;
  new_params.pole_angle_coeff = 6;
  new_params.cart_velocity_coeff = 7;
  new_params.pole_angular_velocity_coeff = 8;
  new_params.control_cost_coeff[0] = 9;
  new_params.terminal_cost_coeff = 2000;
  new_params.desired_terminal_state[0] = 3;
  new_params.desired_terminal_state[1] = 2;
  new_params.desired_terminal_state[2] = 3.14;
  new_params.desired_terminal_state[3] = 1;

  CartpoleQuadraticCostParams gpu_params;

  cost.setParams(new_params);

  if (cost.GPUMemStatus_)
  {
    // Launch kernel to grab parameters from the GPU
    launchParameterTestKernel(cost, gpu_params);
  }
  else
  {
    FAIL() << "GPU Setup has not been called or is not functioning.";
  }

  EXPECT_FLOAT_EQ(new_params.cart_position_coeff, gpu_params.cart_position_coeff);
  EXPECT_FLOAT_EQ(new_params.pole_angle_coeff, gpu_params.pole_angle_coeff);
  EXPECT_FLOAT_EQ(new_params.cart_velocity_coeff, gpu_params.cart_velocity_coeff);
  EXPECT_FLOAT_EQ(new_params.pole_angular_velocity_coeff, gpu_params.pole_angular_velocity_coeff);
  EXPECT_FLOAT_EQ(new_params.control_cost_coeff[0], gpu_params.control_cost_coeff[0]);
  EXPECT_FLOAT_EQ(new_params.terminal_cost_coeff, gpu_params.terminal_cost_coeff);
  EXPECT_FLOAT_EQ(new_params.desired_terminal_state[0], gpu_params.desired_terminal_state[0]);
  EXPECT_FLOAT_EQ(new_params.desired_terminal_state[1], gpu_params.desired_terminal_state[1]);
  EXPECT_FLOAT_EQ(new_params.desired_terminal_state[2], gpu_params.desired_terminal_state[2]);
  EXPECT_FLOAT_EQ(new_params.desired_terminal_state[3], gpu_params.desired_terminal_state[3]);
}
TEST(CartpoleQuadraticCost, ComputeStateCost)
{
  CartpoleQuadraticCost cost;

  CartpoleQuadraticCost::output_array state;
  state << 1, 2, 3, 4;

  float cost_compute = cost.computeStateCost(state);
  float cost_known =
      (state[0] - cost.getParams().desired_terminal_state[0]) *
          (state(0) - cost.getParams().desired_terminal_state[0]) * cost.getParams().cart_position_coeff +
      (state[1] - cost.getParams().desired_terminal_state[1]) *
          (state(1) - cost.getParams().desired_terminal_state[1]) * cost.getParams().cart_velocity_coeff +
      (state[2] - cost.getParams().desired_terminal_state[2]) *
          (state(2) - cost.getParams().desired_terminal_state[2]) * cost.getParams().pole_angle_coeff +
      (state[3] - cost.getParams().desired_terminal_state[3]) *
          (state(3) - cost.getParams().desired_terminal_state[3]) * cost.getParams().pole_angular_velocity_coeff;

  ASSERT_EQ(cost_known, cost_compute);
}

TEST(CartpoleQuadraticCost, ComputeRunningCost)
{
  CartpoleQuadraticCost cost;

  CartpoleQuadraticCost::output_array state;
  CartpoleQuadraticCost::control_array control, noise, std_dev;
  state << 5, 3, 2, 4;
  control << 6;
  noise << 0.3;
  std_dev << 2;
  int timestep = 0;
  int crash_status[1] = { 0 };

  float cost_compute = cost.computeRunningCost(state, control, timestep, crash_status);
  float cost_known =
      (state[0] - cost.getParams().desired_terminal_state[0]) *
          (state(0) - cost.getParams().desired_terminal_state[0]) * cost.getParams().cart_position_coeff +
      (state[1] - cost.getParams().desired_terminal_state[1]) *
          (state(1) - cost.getParams().desired_terminal_state[1]) * cost.getParams().cart_velocity_coeff +
      (state[2] - cost.getParams().desired_terminal_state[2]) *
          (state(2) - cost.getParams().desired_terminal_state[2]) * cost.getParams().pole_angle_coeff +
      (state[3] - cost.getParams().desired_terminal_state[3]) *
          (state(3) - cost.getParams().desired_terminal_state[3]) * cost.getParams().pole_angular_velocity_coeff;

  cost_known = cost_known;
  ASSERT_EQ(cost_known, cost_compute);
}

TEST(CartpoleQuadraticCost, ComputeTerminalCost)
{
  CartpoleQuadraticCost cost;

  std::array<float, 4> state = { 2.f, 3.f, 7.f, 43.f };

  // float cost_compute = cost.terminalCost(state);
  float cost_compute = 0.0f;
  float cost_known =
      ((state[0] - cost.getParams().desired_terminal_state[0]) *
           (state[0] - cost.getParams().desired_terminal_state[0]) * cost.getParams().cart_position_coeff +
       (state[1] - cost.getParams().desired_terminal_state[1]) *
           (state[1] - cost.getParams().desired_terminal_state[1]) * cost.getParams().cart_velocity_coeff +
       (state[2] - cost.getParams().desired_terminal_state[2]) *
           (state[2] - cost.getParams().desired_terminal_state[2]) * cost.getParams().pole_angle_coeff +
       (state[3] - cost.getParams().desired_terminal_state[3]) *
           (state[3] - cost.getParams().desired_terminal_state[3]) * cost.getParams().pole_angular_velocity_coeff) *
      cost.getParams().terminal_cost_coeff;
  ASSERT_EQ(cost_known, cost_compute);
}
