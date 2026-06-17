#include <gtest/gtest.h>
#include <mppi/cost_functions/quadrotor/quadrotor_map_cost.cuh>
#include <mppi/utils/test_helper.h>

TEST(QuadrotorMapCost, checkHeadingCost)
{
  GTEST_SKIP_("The calculation has fundamentally changed, uses velocity vector and not orientation");
  using COST = QuadrotorMapCost;
  COST cost;
  COST::output_array curr_state = COST::output_array::Zero();
  Eigen::Quaternionf temp_quat;
  float deg2rad = M_PI / 180;
  // Have velocity in the y direction
  curr_state[4] = 1;
  // Get quaternion for yaw of 30 degrees
  mppi::math::Euler2QuatNWU(0, 0, 30.0 * deg2rad, temp_quat);
  temp_quat.normalize();
  curr_state[6] = temp_quat.w();
  curr_state[7] = temp_quat.x();
  curr_state[8] = temp_quat.y();
  curr_state[9] = temp_quat.z();
  // Current state has yaw of 30 degrees
  auto params = cost.getParams();
  params.attitude_coeff = 0;
  params.heading_coeff = 10;
  params.curr_waypoint = make_float4(0, 1, 0, 0);
  cost.setParams(params);

  float expected_cost = params.heading_coeff * powf(M_PI_2 - 30 * deg2rad, 2);
  float calculated_cost = cost.computeHeadingCost(curr_state.data());
  EXPECT_FLOAT_EQ(expected_cost, calculated_cost);
}

TEST(QuadrotorMapCost, checkSpeedCost)
{
  using COST = QuadrotorMapCost;
  COST cost;
  COST::output_array curr_state = COST::output_array::Zero();
  Eigen::Quaternionf temp_quat;
  // Get quaternion for yaw of 30 degrees
  curr_state[3] = 3;
  curr_state[4] = 4;
  // Current state has yaw of 30 degrees
  auto params = cost.getParams();
  params.speed_coeff = 10;
  params.desired_speed = 10;
  cost.setParams(params);

  float expected_cost = params.speed_coeff * powf(10 - 5, 2);
  float calculated_cost = cost.computeSpeedCost(curr_state.data());
  EXPECT_FLOAT_EQ(expected_cost, calculated_cost);
}
