#include <gtest/gtest.h>
#include <kernel_tests/dynamics/dynamics_generic_kernel_tests.cuh>
#include <mppi/dynamics/linear/linear.cuh>

TEST(Linear, Dimensionality)
{
  using DYN_T = LinearDynamics<4, 12>;
  ASSERT_EQ(4, DYN_T::STATE_DIM);
  ASSERT_EQ(4, DYN_T::OUTPUT_DIM);
  ASSERT_EQ(12, DYN_T::CONTROL_DIM);
}

TEST(Linear, SetParamsA)
{
  using DYN_T = LinearDynamics<4, 12>;
  using DFDX = typename DYN_T::dfdx;
  // Set A from params
  DFDX A = DFDX::Random();
  typename DYN_T::DYN_PARAMS_T params;
  params.setA(A);
  auto dynamics = DYN_T(params);
  auto dyn_params = dynamics.getParams();
  Eigen::Map<DFDX> A_params_result(dyn_params.A);
  float diff = (A - A_params_result).sum();
  EXPECT_FLOAT_EQ(diff, 0);
  // Set A from dynamics
  A = DFDX::Random();
  dynamics.setA(A);
  dyn_params = dynamics.getParams();
  Eigen::Map<DFDX> A_class_result(dyn_params.A);
  diff = (A - A_class_result).sum();
  EXPECT_FLOAT_EQ(diff, 0);
}

TEST(Linear, SetParamsB)
{
  using DYN_T = LinearDynamics<4, 12>;
  using DFDU = typename DYN_T::dfdu;
  // Set B from params
  DFDU B = DFDU::Random();
  typename DYN_T::DYN_PARAMS_T params;
  params.setB(B);
  auto dynamics = DYN_T(params);
  auto dyn_params = dynamics.getParams();
  Eigen::Map<DFDU> B_result(dyn_params.B);
  float diff = (B - B_result).sum();
  EXPECT_FLOAT_EQ(diff, 0);
  // Set A from dynamics
  B = DFDU::Random();
  dynamics.setB(B);
  dyn_params = dynamics.getParams();
  Eigen::Map<DFDU> B_class_result(dyn_params.B);
  diff = (B - B_class_result).sum();
  EXPECT_FLOAT_EQ(diff, 0);
}

TEST(Linear, CheckSharedMemorySizes)
{
  using DYN_T = LinearDynamics<4, 12>;
  auto dynamics = DYN_T();
  dynamics.GPUSetup();
  int output_gpu[2] = { 0 };
  int output_cpu[2] = { 0 };
  launchGetSharedMemorySizesKernel<DYN_T>(dynamics, output_gpu);
  output_cpu[0] = dynamics.getGrdSharedSizeBytes();
  output_cpu[1] = dynamics.getBlkSharedSizeBytes();
  ASSERT_EQ(output_cpu[0], output_gpu[0]);
  ASSERT_EQ(output_cpu[1], output_gpu[1]);
}

TEST(Linear, StepCPUGPUComparison)
{
  using DYN_T = LinearDynamics<10, 12>;
  using DFDU = typename DYN_T::dfdu;
  using DFDX = typename DYN_T::dfdx;
  DFDU B = DFDU::Random();
  DFDX A = DFDX::Random();
  auto dynamics = DYN_T();
  dynamics.setA(A);
  dynamics.setB(B);
  typename DYN_T::buffer_trajectory buffer;

  std::vector<int> x_sizes = { 1, 2, 4, 8, 16, 32 };
  for (const auto& x_dim : x_sizes)
  {
    checkGPUComputationStep<DYN_T>(dynamics, 0.01, 32, x_dim, buffer);
  }
}

TEST(Linear, JacobianCheck)
{
  using DYN_T = LinearDynamics<10, 6>;
  using DFDU = typename DYN_T::dfdu;
  using DFDX = typename DYN_T::dfdx;
  using state_array = typename DYN_T::state_array;
  using control_array = typename DYN_T::control_array;
  auto dynamics = DYN_T();
  DFDX A = DFDX::Identity();
  DFDU B = DFDU::Random();
  dynamics.setA(A);
  dynamics.setB(B);

  DFDX Jacobian_A;
  DFDU Jacobian_B;
  state_array x = dynamics.getZeroState();
  control_array u = dynamics.getZeroControl();
  dynamics.computeGrad(x, u, Jacobian_A, Jacobian_B);
  float a_diff = (Jacobian_A - A).array().abs().sum();
  float b_diff = (Jacobian_B - B).array().abs().sum();
  ASSERT_EQ(a_diff, 0);
  ASSERT_EQ(b_diff, 0);
}

TEST(Linear, HardCodeCPUTest)
{
  using DYN_T = LinearDynamics<3, 1>;
  using DFDU = typename DYN_T::dfdu;
  using DFDX = typename DYN_T::dfdx;
  using state_array = typename DYN_T::state_array;
  using control_array = typename DYN_T::control_array;
  using output_array = typename DYN_T::output_array;

  auto dynamics = DYN_T();
  DFDX A = DFDX::Identity();
  DFDU B = DFDU::Zero();
  B(2, 0) = 0.5f;
  dynamics.setA(A);
  dynamics.setB(B);

  state_array x = dynamics.getZeroState();
  control_array u = dynamics.getZeroControl();
  x << 1, 5, 10;
  u << 3;
  float dt = 0.1;
  output_array y;
  state_array x_der, x_next;
  dynamics.step(x, x_next, x_der, u, y, 0, dt);
  // Check derivative
  ASSERT_FLOAT_EQ(x_der[0], 1.0f);
  ASSERT_FLOAT_EQ(x_der[1], 5.0f);
  ASSERT_FLOAT_EQ(x_der[2], 10.0f + 1.5f);

  // Check next state
  ASSERT_FLOAT_EQ(x_next[0], 1.0f + 1.0f * 0.1f);
  ASSERT_FLOAT_EQ(x_next[1], 5.0f + 5.0f * 0.1f);
  ASSERT_FLOAT_EQ(x_next[2], 10.0f + (10.0f + 1.5f) * 0.1f);

  // Check output
  ASSERT_FLOAT_EQ(y[0], 1.0f + 1.0f * 0.1f);
  ASSERT_FLOAT_EQ(y[1], 5.0f + 5.0f * 0.1f);
  ASSERT_FLOAT_EQ(y[2], 10.0f + (10.0f + 1.5f) * 0.1f);
}
