#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <mppi/dynamics/dubins/dubins.cuh>
#include <kernel_tests/dynamics/dynamics_generic_kernel_tests.cuh>
#include <mppi/ddp/ddp_model_wrapper.h>
#include <cuda_runtime.h>

TEST(DubinsDynamics, Template)
{
  auto dynamics = DubinsDynamics();
  EXPECT_EQ(3, DubinsDynamics::STATE_DIM);
  EXPECT_EQ(2, DubinsDynamics::CONTROL_DIM);
}

TEST(DubinsDynamics, BindStream)
{
  cudaStream_t stream;

  HANDLE_ERROR(cudaStreamCreate(&stream));

  auto dynamics = DubinsDynamics(stream);

  EXPECT_EQ(dynamics.stream_, stream) << "Stream binding failure.";

  HANDLE_ERROR(cudaStreamDestroy(stream));
}

TEST(DubinsDynamics, ComputeDynamics)
{
  DubinsDynamics dynamics = DubinsDynamics();
  DubinsDynamics::state_array x;
  x << 0, 0, 0;
  DubinsDynamics::control_array u;
  u << 0, 0;

  DubinsDynamics::state_array next_x;
  dynamics.computeDynamics(x, u, next_x);
  EXPECT_FLOAT_EQ(next_x(0), 0);
  EXPECT_FLOAT_EQ(next_x(1), 0);
  EXPECT_FLOAT_EQ(next_x(2), 0);

  x << 1, 2, 0;
  u << 1, 0;
  dynamics.computeDynamics(x, u, next_x);
  EXPECT_FLOAT_EQ(next_x(0), 1);
  EXPECT_FLOAT_EQ(next_x(1), 0);
  EXPECT_FLOAT_EQ(next_x(2), 0);

  x << 1, 2, 0;
  u << 3, 1;
  dynamics.computeDynamics(x, u, next_x);
  EXPECT_FLOAT_EQ(next_x(0), 3);
  EXPECT_FLOAT_EQ(next_x(1), 0);
  EXPECT_FLOAT_EQ(next_x(2), 1);

  x << 1, 2, M_PI_2;
  u << 4, 1;
  dynamics.computeDynamics(x, u, next_x);
  EXPECT_NEAR(next_x(0), 0.0, 1e-5);
  EXPECT_FLOAT_EQ(next_x(1), 4 * sin(M_PI_2));
  EXPECT_FLOAT_EQ(next_x(2), 1);

  // TODO test case for flipping across angle discontinuity
  x << 1, 2, M_PI_2;
  u << 4, 1;
  dynamics.computeDynamics(x, u, next_x);
  EXPECT_NEAR(next_x(0), 0.0, 1e-5);
  EXPECT_FLOAT_EQ(next_x(1), 4 * sin(M_PI_2));
  EXPECT_FLOAT_EQ(next_x(2), 1);
}

TEST(DubinsDynamics, TestDynamicsGPU)
{
  DubinsDynamics dynamics = DubinsDynamics();
  dynamics.GPUSetup();

  DubinsDynamics::state_array state;
  state(0) = 0.5;
  state(1) = 0.7;
  state(2) = M_PI_4;
  DubinsDynamics::control_array control;
  control(0) = 3.0;
  control(1) = 2.0;

  std::vector<std::array<float, 3>> s(1);
  for (int dim = 0; dim < 3; dim++)
  {
    s[0][dim] = state(dim);
  }
  std::vector<std::array<float, 3>> s_der(1);
  // steering, throttle
  std::vector<std::array<float, 2>> u(1);
  for (int dim = 0; dim < 2; dim++)
  {
    u[0][dim] = control(dim);
  }

  // These variables will be changed so initialized to the right size only
  DubinsDynamics::state_array state_der_cpu = DubinsDynamics::state_array::Zero();

  // Run dynamics on dynamicsU
  dynamics.computeDynamics(state, control, state_der_cpu);
  // Run dynamics on GPU
  for (int y_dim = 1; y_dim <= 3; y_dim++)
  {
    launchComputeDynamicsTestKernel<DubinsDynamics, 3, 2>(dynamics, s, u, s_der, y_dim);
    for (int dim = 0; dim < 3; dim++)
    {
      EXPECT_FLOAT_EQ(state_der_cpu(dim), s_der[0][dim]);
    }
  }

  dynamics.freeCudaMem();
}

TEST(DubinsDynamics, TestUpdateStateGPU)
{
  DubinsDynamics dynamics = DubinsDynamics();
  dynamics.GPUSetup();

  DubinsDynamics::state_array state;
  state(0) = 0.5;
  state(1) = 0.7;
  state(2) = M_PI;
  DubinsDynamics::control_array control;
  control(0) = 3.0;
  control(1) = 2.0;

  std::vector<std::array<float, 3>> s(1);
  for (int dim = 0; dim < 3; dim++)
  {
    s[0][dim] = state(dim);
  }
  std::vector<std::array<float, 3>> s_der(1);
  // steering, throttle
  std::vector<std::array<float, 2>> u(1);
  for (int dim = 0; dim < 2; dim++)
  {
    u[0][dim] = control(dim);
  }

  // These variables will be changed so initialized to the right size only
  DubinsDynamics::state_array state_der_cpu = DubinsDynamics::state_array::Zero();

  // Run dynamics on dynamicsU
  dynamics.computeDynamics(state, control, state_der_cpu);
  dynamics.updateState(state, state_der_cpu, 0.1f);
  // Run dynamics on GPU
  for (int y_dim = 1; y_dim <= 3; y_dim++)
  {
    launchComputeStateDerivTestKernel<DubinsDynamics, 3, 2>(dynamics, s, u, s_der, y_dim);
    launchUpdateStateTestKernel<DubinsDynamics, 3>(dynamics, s, s_der, 0.1f, y_dim);
    for (int dim = 0; dim < 3; dim++)
    {
      EXPECT_FLOAT_EQ(state_der_cpu(dim), s_der[0][dim]);
    }
  }
  dynamics.freeCudaMem();
}

class DubinsDummy : public DubinsDynamics
{
public:
  bool computeGrad(const Eigen::Ref<const state_array>& state, const Eigen::Ref<const control_array>& control,
                   Eigen::Ref<dfdx> A, Eigen::Ref<dfdu> B)
  {
    return false;
  };
};

TEST(DubinsDynamics, TestComputeGradComputation)
{
  Eigen::Matrix<float, DubinsDynamics::STATE_DIM, DubinsDynamics::STATE_DIM + DubinsDynamics::CONTROL_DIM> numeric_jac;
  Eigen::Matrix<float, DubinsDynamics::STATE_DIM, DubinsDynamics::STATE_DIM + DubinsDynamics::CONTROL_DIM> analytic_jac;
  DubinsDynamics::state_array state;
  state << 1, 2, 3;
  DubinsDynamics::control_array control;
  control << 5;

  auto analytic_grad_model = DubinsDynamics();

  DubinsDynamics::dfdx A_analytic = DubinsDynamics::dfdx::Zero();
  DubinsDynamics::dfdu B_analytic = DubinsDynamics::dfdu::Zero();

  analytic_grad_model.computeGrad(state, control, A_analytic, B_analytic);

  auto numerical_grad_model = DubinsDummy();

  std::shared_ptr<ModelWrapperDDP<DubinsDummy>> ddp_model =
      std::make_shared<ModelWrapperDDP<DubinsDummy>>(&numerical_grad_model);

  analytic_jac.leftCols<DubinsDynamics::STATE_DIM>() = A_analytic;
  analytic_jac.rightCols<DubinsDynamics::CONTROL_DIM>() = B_analytic;
  numeric_jac = ddp_model->df(state, control);

  ASSERT_LT((numeric_jac - analytic_jac).norm(), 1e-3) << "Numeric Jacobian\n"
                                                       << numeric_jac << "\nAnalytic Jacobian\n"
                                                       << analytic_jac;
}
