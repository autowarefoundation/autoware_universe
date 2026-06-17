//
// Created by mgandhi3 on 10/4/19.
//

#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <kernel_tests/dynamics/cartpole/cartpole_dynamics_kernel_test.cuh>
#include <cuda_runtime.h>
#include <mppi/ddp/ddp_model_wrapper.h>
#include <memory>

TEST(CartPole, StateDim)
{
  auto CP = CartpoleDynamics(1, 1, 1);
  EXPECT_EQ(4, CartpoleDynamics::STATE_DIM);
}

TEST(CartPole, ControlDim)
{
  auto CP = CartpoleDynamics(1, 1, 1);
  EXPECT_EQ(1, CartpoleDynamics::CONTROL_DIM);
}

TEST(CartPole, Equilibrium)
{
  auto CP = CartpoleDynamics(1, 1, 1);

  CartpoleDynamics::state_array state;
  state << 0, 0, 0, 0;

  CartpoleDynamics::control_array control;
  control << 0;

  CartpoleDynamics::state_array state_dot_compute;
  state_dot_compute << 1, 1, 1, 1;

  CartpoleDynamics::state_array state_dot_known;
  state_dot_known << 0, 0, 0, 0;

  CP.computeDynamics(state, control, state_dot_compute);
  for (int i = 0; i < CartpoleDynamics::STATE_DIM; i++)
  {
    EXPECT_NEAR(state_dot_known(i), state_dot_compute(i), 1e-4) << "Failed at index: " << i;
  }
}

TEST(CartPole, BindStream)
{
  cudaStream_t stream;

  HANDLE_ERROR(cudaStreamCreate(&stream));

  auto CP = CartpoleDynamics(1, 1, 2, stream);

  EXPECT_EQ(CP.stream_, stream) << "Stream binding failure.";

  HANDLE_ERROR(cudaStreamDestroy(stream));
}

TEST(CartPole, SetGetParamsHost)
{
  auto params = CartpoleDynamicsParams(2.0, 3.0, 4.0);
  auto CP = CartpoleDynamics(1, 1, 2);

  CP.setParams(params);
  auto CP_params = CP.getParams();

  EXPECT_FLOAT_EQ(params.cart_mass, CP_params.cart_mass);
  EXPECT_FLOAT_EQ(params.pole_mass, CP_params.pole_mass);
  EXPECT_FLOAT_EQ(params.pole_length, CP_params.pole_length);
}

TEST(CartPole, CartPole_GPUSetup_Test)
{
  auto CP_host = new CartpoleDynamics(1, 2, 2);
  CP_host->GPUSetup();
  // float mass;
  // launchParameterTestKernel(*CP_host, mass);

  EXPECT_TRUE(CP_host->GPUMemStatus_);

  delete (CP_host);
}

TEST(CartPole, GetCartMassFromGPU)
{
  auto CP_host = new CartpoleDynamics(1, 1, 2);
  CP_host->GPUSetup();

  auto params = CartpoleDynamicsParams(2.0, 3.0, 4.0);
  CP_host->setParams(params);
  float mass;

  launchCartMassTestKernel(*CP_host, mass);

  EXPECT_FLOAT_EQ(params.cart_mass, mass);

  CP_host->freeCudaMem();
  delete (CP_host);
}

TEST(CartPole, GetPoleMassFromGPU)
{
  auto CP_host = new CartpoleDynamics(1, 1, 2);
  CP_host->GPUSetup();

  auto params = CartpoleDynamicsParams(2.0, 3.0, 4.0);
  CP_host->setParams(params);
  float mass;

  launchPoleMassTestKernel(*CP_host, mass);

  EXPECT_FLOAT_EQ(params.pole_mass, mass);

  CP_host->freeCudaMem();
  delete (CP_host);
}

TEST(CartPole, GetPoleLengthFromGPU)
{
  auto CP_host = new CartpoleDynamics(1, 1, 2);
  CP_host->GPUSetup();

  auto params = CartpoleDynamicsParams(2.0, 3.0, 4.0);
  CP_host->setParams(params);
  float length;

  launchPoleLengthTestKernel(*CP_host, length);

  EXPECT_FLOAT_EQ(params.pole_length, length);

  CP_host->freeCudaMem();
  delete (CP_host);
}

TEST(CartPole, GetGravityFromGPU)
{
  auto CP_host = new CartpoleDynamics(1, 1, 2);
  CP_host->GPUSetup();

  auto params = CartpoleDynamicsParams(2.0, 3.0, 4.0);
  CP_host->setParams(params);
  float gravity_gpu;

  launchGravityTestKernel(*CP_host, gravity_gpu);

  EXPECT_FLOAT_EQ(CP_host->getGravity(), gravity_gpu);

  CP_host->freeCudaMem();
  delete (CP_host);
}

TEST(CartPole, TestDynamicsGPU)
{
  auto CP_host = new CartpoleDynamics(1, 1, 2);
  CP_host->GPUSetup();

  auto params = CartpoleDynamicsParams(2.0, 3.0, 4.0);
  CP_host->setParams(params);

  CartpoleDynamics::state_array state;
  state(0) = 0.1;
  state(1) = 0.3;
  state(2) = 0.23;
  state(3) = 0.334;
  CartpoleDynamics::control_array control;
  control(0) = 0.654;

  // These variables will be changed so initialized to the right size only
  CartpoleDynamics::state_array state_der_cpu = CartpoleDynamics::state_array::Zero();

  float state_der_gpu[CartpoleDynamics::STATE_DIM];

  // Run dynamics on CPU
  CP_host->computeDynamics(state, control, state_der_cpu);
  // Run dynamics on GPU
  launchDynamicsTestKernel(*CP_host, state.data(), control.data(), state_der_gpu);

  // Compare CPU and GPU Results
  for (int i = 0; i < CartpoleDynamics::STATE_DIM; i++)
  {
    EXPECT_FLOAT_EQ(state_der_cpu(i), state_der_gpu[i]);
  }

  CP_host->freeCudaMem();
  delete (CP_host);
}

class CartpoleDummy : public CartpoleDynamics
{
public:
  CartpoleDummy(float cartMass, float poleMass, float poleLength, cudaStream_t stream = 0)
    : CartpoleDynamics(cartMass, poleMass, poleLength, stream){};
  bool computeGrad(const Eigen::Ref<const state_array>& state, const Eigen::Ref<const control_array>& control,
                   Eigen::Ref<dfdx> A, Eigen::Ref<dfdu> B)
  {
    return false;
  };
};

TEST(CartPole, TestComputeGradComputation)
{
  Eigen::Matrix<float, CartpoleDynamics::STATE_DIM, CartpoleDynamics::STATE_DIM + CartpoleDynamics::CONTROL_DIM>
      numeric_jac;
  Eigen::Matrix<float, CartpoleDynamics::STATE_DIM, CartpoleDynamics::STATE_DIM + CartpoleDynamics::CONTROL_DIM>
      analytic_jac;
  CartpoleDynamics::state_array state;
  state << 1, 2, 3, 4;
  CartpoleDynamics::control_array control;
  control << 5;

  auto analytic_grad_model = CartpoleDynamics(1, 1, 1);

  CartpoleDynamics::dfdx A_analytic = CartpoleDynamics::dfdx::Zero();
  CartpoleDynamics::dfdu B_analytic = CartpoleDynamics::dfdu::Zero();

  analytic_grad_model.computeGrad(state, control, A_analytic, B_analytic);

  auto numerical_grad_model = CartpoleDummy(1, 1, 1);

  std::shared_ptr<ModelWrapperDDP<CartpoleDummy>> ddp_model =
      std::make_shared<ModelWrapperDDP<CartpoleDummy>>(&numerical_grad_model);

  analytic_jac.leftCols<CartpoleDynamics::STATE_DIM>() = A_analytic;
  analytic_jac.rightCols<CartpoleDynamics::CONTROL_DIM>() = B_analytic;
  numeric_jac = ddp_model->df(state, control);

  ASSERT_LT((numeric_jac - analytic_jac).norm(), 1e-3) << "Numeric Jacobian\n"
                                                       << numeric_jac << "\nAnalytic Jacobian\n"
                                                       << analytic_jac;
}
