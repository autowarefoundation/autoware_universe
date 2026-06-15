#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <mppi/dynamics/racer_dubins/racer_dubins.cuh>
#include <kernel_tests/dynamics/dynamics_generic_kernel_tests.cuh>
#include <mppi/ddp/ddp_model_wrapper.h>
#include <cuda_runtime.h>

TEST(RacerDubins, Template)
{
  auto dynamics = RacerDubins();
  EXPECT_EQ(7, RacerDubins::STATE_DIM);
  EXPECT_EQ(2, RacerDubins::CONTROL_DIM);
}

TEST(RacerDubins, BindStream)
{
  cudaStream_t stream;

  HANDLE_ERROR(cudaStreamCreate(&stream));

  auto dynamics = RacerDubins(stream);

  EXPECT_EQ(dynamics.stream_, stream) << "Stream binding failure.";

  HANDLE_ERROR(cudaStreamDestroy(stream));
}

/*
float c_t = 1.3;
float c_b = 2.5;
float c_v = 3.7;
float c_0 = 4.9;
float wheel_base = 0.3;
 */

TEST(RacerDubins, ComputeDynamics)
{
  RacerDubins dynamics = RacerDubins();
  RacerDubinsParams params = dynamics.getParams();
  RacerDubins::state_array x = RacerDubins::state_array::Zero();
  RacerDubins::control_array u = RacerDubins::control_array::Zero();

  RacerDubins::state_array next_x = RacerDubins::state_array::Zero();
  dynamics.computeDynamics(x, u, next_x);
  EXPECT_FLOAT_EQ(next_x(0), 4.9);
  EXPECT_FLOAT_EQ(next_x(1), 0);
  EXPECT_FLOAT_EQ(next_x(2), 0);
  EXPECT_FLOAT_EQ(next_x(3), 0);
  EXPECT_FLOAT_EQ(next_x(4), 0);
  EXPECT_FLOAT_EQ(next_x(5), 0);
  EXPECT_FLOAT_EQ(next_x(6), 0);

  x << 1, M_PI_2, 0, 3, 0, 0, 0;
  u << 1, 0;
  dynamics.computeDynamics(x, u, next_x);
  EXPECT_FLOAT_EQ(next_x(0), 4.9 + 1.3 - 3.7);
  EXPECT_FLOAT_EQ(next_x(1), 0);
  EXPECT_NEAR(next_x(2), 0, 1e-7);
  EXPECT_FLOAT_EQ(next_x(3), 1);
  EXPECT_FLOAT_EQ(next_x(4), 0);
  EXPECT_FLOAT_EQ(next_x(5), 0);
  EXPECT_FLOAT_EQ(next_x(6), 0);

  x << 1, 0, 0, 3, 0, 0, 0;
  u << -1, 0;
  dynamics.computeDynamics(x, u, next_x);
  EXPECT_FLOAT_EQ(next_x(0), 4.9 - 3.7);
  EXPECT_FLOAT_EQ(next_x(1), 0);
  EXPECT_FLOAT_EQ(next_x(2), 1);
  EXPECT_FLOAT_EQ(next_x(3), 0);
  EXPECT_FLOAT_EQ(next_x(4), 0);
  EXPECT_FLOAT_EQ(next_x(5), 0.33);
  EXPECT_FLOAT_EQ(next_x(6), 0);

  x << 1, 0, 0, 3, 0, 0.33, 0;
  u << -1, 0;
  dynamics.computeDynamics(x, u, next_x);
  EXPECT_FLOAT_EQ(next_x(0), 4.9 - 0.33 * 2.5 - 3.7);
  EXPECT_FLOAT_EQ(next_x(1), 0);
  EXPECT_FLOAT_EQ(next_x(2), 1);
  EXPECT_FLOAT_EQ(next_x(3), 0);
  EXPECT_FLOAT_EQ(next_x(4), 0);
  EXPECT_FLOAT_EQ(next_x(5), 0.33);
  EXPECT_FLOAT_EQ(next_x(6), 0);

  x << 1, 0, 0, 3, 0, 1.0, 0;
  u << 1, 0;
  dynamics.computeDynamics(x, u, next_x);
  EXPECT_FLOAT_EQ(next_x(0), 4.9 - 2.5 + 1.3 - 3.7);
  EXPECT_FLOAT_EQ(next_x(1), 0);
  EXPECT_FLOAT_EQ(next_x(2), 1);
  EXPECT_FLOAT_EQ(next_x(3), 0);
  EXPECT_FLOAT_EQ(next_x(4), 0);
  EXPECT_FLOAT_EQ(next_x(5), -0.9);
  EXPECT_FLOAT_EQ(next_x(6), 0);

  x << -1, 0, 0, 3, 0, 0, 0;
  u << 1, 0;
  dynamics.computeDynamics(x, u, next_x);
  EXPECT_FLOAT_EQ(next_x(0), 4.9 + 3.7 + 1.3);
  EXPECT_FLOAT_EQ(next_x(1), 0);
  EXPECT_FLOAT_EQ(next_x(2), -1);
  EXPECT_FLOAT_EQ(next_x(3), 0);
  EXPECT_FLOAT_EQ(next_x(4), 0);
  EXPECT_FLOAT_EQ(next_x(5), 0);
  EXPECT_FLOAT_EQ(next_x(6), 0);

  x << -1, 0, 0, 3, 0, 1.0, 0;
  u << -1, 0;
  dynamics.computeDynamics(x, u, next_x);
  EXPECT_FLOAT_EQ(next_x(0), 4.9 + 2.5 + 3.7);
  EXPECT_FLOAT_EQ(next_x(1), 0);
  EXPECT_FLOAT_EQ(next_x(2), -1);
  EXPECT_FLOAT_EQ(next_x(3), 0);
  EXPECT_FLOAT_EQ(next_x(4), 0);
  EXPECT_FLOAT_EQ(next_x(5), 0);
  EXPECT_FLOAT_EQ(next_x(6), 0);

  x << -3, 0, 0, 3, 0, 1.0, 0;
  u << -1, 0;
  dynamics.computeDynamics(x, u, next_x);
  EXPECT_FLOAT_EQ(next_x(0), 4.9 + 2.5 + 3.7 * 3);
  EXPECT_FLOAT_EQ(next_x(1), 0);
  EXPECT_FLOAT_EQ(next_x(2), -3);
  EXPECT_FLOAT_EQ(next_x(3), 0);
  EXPECT_FLOAT_EQ(next_x(4), 0);
  EXPECT_FLOAT_EQ(next_x(5), 0);
  EXPECT_FLOAT_EQ(next_x(6), 0);

  x << 4, 0, 0, 3, 0, 1.0, 0;
  u << -1, 0;
  dynamics.computeDynamics(x, u, next_x);
  EXPECT_FLOAT_EQ(next_x(0), 4.9 - 2.5 - 3.7 * 4);
  EXPECT_FLOAT_EQ(next_x(1), 0);
  EXPECT_FLOAT_EQ(next_x(2), 4);
  EXPECT_FLOAT_EQ(next_x(3), 0);
  EXPECT_FLOAT_EQ(next_x(4), 0);
  EXPECT_FLOAT_EQ(next_x(5), 0);
  EXPECT_FLOAT_EQ(next_x(6), 0);

  x << 1, M_PI, 0, 3, 0, 0, 0;
  u << 0, 1;
  dynamics.computeDynamics(x, u, next_x);
  EXPECT_FLOAT_EQ(next_x(0), 4.9 - 3.7);
  EXPECT_FLOAT_EQ(next_x(1), (1 / .3) * tan(0));
  EXPECT_FLOAT_EQ(next_x(2), -1);
  EXPECT_NEAR(next_x(3), 0, 1e-7);
  EXPECT_FLOAT_EQ(next_x(4), 1 * 5 * 0.6);
  EXPECT_FLOAT_EQ(next_x(5), 0);
  EXPECT_FLOAT_EQ(next_x(6), 0);

  x << 1, M_PI, 0, 0, 0.5, 0, 0;
  u << 1, -1;
  dynamics.computeDynamics(x, u, next_x);
  EXPECT_FLOAT_EQ(next_x(0), 4.9 - 3.7 + 1.3);
  EXPECT_FLOAT_EQ(next_x(1), (1 / .3) * tan(0.5 / -9.1));
  EXPECT_FLOAT_EQ(next_x(2), -1);
  EXPECT_NEAR(next_x(3), 0, 1e-7);
  EXPECT_FLOAT_EQ(next_x(4), (-1 * 5 - 0.5) * 0.6);
  EXPECT_FLOAT_EQ(next_x(5), 0);
  EXPECT_FLOAT_EQ(next_x(6), 0);
}

TEST(RacerDubins, enforceLeash)
{
  RacerDubins dynamics = RacerDubins();
  RacerDubins::state_array state_true = RacerDubins::state_array::Zero();
  RacerDubins::state_array state_nominal = RacerDubins::state_array::Zero();
  RacerDubins::state_array leash_values = RacerDubins::state_array::Zero();
  RacerDubins::state_array output;
  double tol = 1e-7;

  // if the two are equal it should match
  state_true = RacerDubins::state_array::Random();
  state_nominal = state_true;
  dynamics.enforceLeash(state_true, state_nominal, leash_values, output);
  // std::cout << "true   : " << state_true.transpose() << std::endl;
  // std::cout << "nominal: " << state_nominal.transpose() << std::endl;
  // std::cout << "output : " << output.transpose() << std::endl;
  EXPECT_LT((output - state_true).norm(), tol);

  // if the two are very far apart but zero leash values
  state_nominal << 1, 1, 1, 1, 1, 1, 1, 1;
  dynamics.enforceLeash(state_true, state_nominal, leash_values, output);
  EXPECT_LT((output - state_true).norm(), tol);

  // if the two are far apart but large leash values
  state_nominal << 1, 1, 1, 1, 1, 1, 1, 1;
  leash_values << 2, 2, 2, 2, 2, 2, 2, 2;
  dynamics.enforceLeash(state_true, state_nominal, leash_values, output);
  EXPECT_LT((output - state_nominal).norm(), tol);

  // check yaw discont
  state_true << -3, -3, -3, -3, -3, -3, -3;
  state_nominal << 3, 3, 3, 3, 3, 3, 3;
  leash_values << 0, 1.0, 0, 0, 0, 0, 0;
  dynamics.enforceLeash(state_true, state_nominal, leash_values, output);
  EXPECT_FLOAT_EQ(output(0), -3);
  EXPECT_FLOAT_EQ(output(1), 3);
  EXPECT_FLOAT_EQ(output(2), -3);
  EXPECT_FLOAT_EQ(output(3), -3);
  EXPECT_FLOAT_EQ(output(4), -3);
  EXPECT_FLOAT_EQ(output(5), -3);
  EXPECT_FLOAT_EQ(output(6), -3);

  // check yaw discont clamp
  state_true << -3, -3, -3, -3, -3, -3, -3;
  state_nominal << 3, 3, 3, 3, 3, 3, 3;
  leash_values << 0, 0.15, 0, 0, 0, 0, 0;
  dynamics.enforceLeash(state_true, state_nominal, leash_values, output);
  EXPECT_FLOAT_EQ(output(0), -3);
  EXPECT_FLOAT_EQ(output(1), angle_utils::normalizeAngle(-3 - 0.15));
  EXPECT_FLOAT_EQ(output(2), -3);
  EXPECT_FLOAT_EQ(output(3), -3);
  EXPECT_FLOAT_EQ(output(4), -3);
  EXPECT_FLOAT_EQ(output(5), -3);
  EXPECT_FLOAT_EQ(output(6), -3);

  // check yaw discont clamp
  state_true << 3, 3, 3, 3, 3, 3, 3;
  state_nominal << -3, -3, -3, -3, -3, -3, -3;
  leash_values << 0, 0.15, 0, 0, 0, 0, 0;
  dynamics.enforceLeash(state_true, state_nominal, leash_values, output);
  EXPECT_FLOAT_EQ(output(0), 3);
  EXPECT_FLOAT_EQ(output(1), angle_utils::normalizeAngle(3 + 0.15));
  EXPECT_FLOAT_EQ(output(2), 3);
  EXPECT_FLOAT_EQ(output(3), 3);
  EXPECT_FLOAT_EQ(output(4), 3);
  EXPECT_FLOAT_EQ(output(5), 3);
  EXPECT_FLOAT_EQ(output(6), 3);

  leash_values = RacerDubins::state_array::Ones();
  std::cout << "=========" << std::endl;

  for (int i = 0; i < RacerDubins::STATE_DIM; i++)
  {
    state_true = RacerDubins::state_array::Zero();
    state_nominal = RacerDubins::state_array::Zero();

    state_true(i) = 1.0;
    state_nominal(i) = 1.1;

    dynamics.enforceLeash(state_true, state_nominal, leash_values, output);

    for (int j = 0; j < RacerDubins::STATE_DIM; j++)
    {
      if (i == j)
      {
        EXPECT_FLOAT_EQ(output(j), 1.1);
      }
      else
      {
        EXPECT_FLOAT_EQ(output(j), 0.0);
      }
    }
  }
}

TEST(RacerDubins, TestModelGPU)
{
  RacerDubins dynamics = RacerDubins();
  dynamics.GPUSetup();

  Eigen::Matrix<float, RacerDubins::CONTROL_DIM, 100> control_trajectory;
  control_trajectory = Eigen::Matrix<float, RacerDubins::CONTROL_DIM, 100>::Random();
  Eigen::Matrix<float, RacerDubins::STATE_DIM, 100> state_trajectory;
  state_trajectory = Eigen::Matrix<float, RacerDubins::STATE_DIM, 100>::Random();

  std::vector<std::array<float, 8>> s(100);
  std::vector<std::array<float, 8>> s_der(100);
  // steering, throttle
  std::vector<std::array<float, 2>> u(100);
  for (int state_index = 0; state_index < s.size(); state_index++)
  {
    for (int dim = 0; dim < s[0].size(); dim++)
    {
      s[state_index][dim] = state_trajectory.col(state_index)(dim);
    }
    for (int dim = 0; dim < u[0].size(); dim++)
    {
      u[state_index][dim] = control_trajectory.col(state_index)(dim);
    }
  }

  // Run dynamics on GPU
  for (int y_dim = 1; y_dim <= 4; y_dim++)
  {
    launchComputeDynamicsTestKernel<RacerDubins, 8, 2>(dynamics, s, u, s_der, y_dim);
    for (int point = 0; point < 100; point++)
    {
      RacerDubins::state_array state = state_trajectory.col(point);
      RacerDubins::control_array control = control_trajectory.col(point);
      RacerDubins::state_array state_der_cpu = RacerDubins::state_array::Zero();

      dynamics.computeDynamics(state, control, state_der_cpu);

      for (int dim = 0; dim < RacerDubins::STATE_DIM; dim++)
      {
        EXPECT_NEAR(state(dim), s[point][dim], 1e-5)
            << "at sample " << point << ", state dim: " << dim << " with y_dim " << y_dim;
        EXPECT_TRUE(isfinite(s[point][dim]));
      }
      for (int dim = 0; dim < RacerDubins::CONTROL_DIM; dim++)
      {
        EXPECT_NEAR(control(dim), u[point][dim], 1e-5)
            << "at sample " << point << ", state dim: " << dim << " with y_dim " << y_dim;
        EXPECT_TRUE(isfinite(u[point][dim]));
      }
      for (int dim = 0; dim < RacerDubins::STATE_DIM; dim++)
      {
        EXPECT_NEAR(state_der_cpu(dim), s_der[point][dim], 1e-5)
            << "at sample " << point << ", state dim: " << dim << " with y_dim " << y_dim;
        EXPECT_TRUE(isfinite(s_der[point][dim]));
      }
    }
  }

  dynamics.freeCudaMem();
}

TEST(RacerDubins, TestUpdateState)
{
  RacerDubins dynamics = RacerDubins();
  std::array<float2, 2> ranges{};
  ranges[0].y = FLT_MAX;
  ranges[0].x = -1;
  ranges[1].y = FLT_MAX;
  ranges[1].x = -FLT_MAX;
  dynamics.setControlRanges(ranges);
  RacerDubins::state_array state;
  RacerDubins::state_array state_der;

  state << 0, 0, 0, 0, 0, 0, 0;
  state_der << 1, 1, 1, 1, 1, 1, 0;
  dynamics.updateState(state, state_der, 0.1);
  EXPECT_TRUE(state_der != RacerDubins::state_array::Zero());
  EXPECT_FLOAT_EQ(state(0), 0.1);
  EXPECT_NEAR(state(1), 0.1, 1e-7);
  EXPECT_FLOAT_EQ(state(2), 0.1);
  EXPECT_FLOAT_EQ(state(3), 0.1);
  EXPECT_FLOAT_EQ(state(4), 0.1);
  EXPECT_FLOAT_EQ(state(5), 0.1);
  EXPECT_FLOAT_EQ(state(6), 1);

  state << 0, M_PI - 0.1, 0, 0, 0, 0, 0;
  state_der << 1, 1, 1, 1, 1, -1, 1;
  dynamics.updateState(state, state_der, 1.0);
  EXPECT_TRUE(state_der != RacerDubins::state_array::Zero());
  EXPECT_FLOAT_EQ(state(0), 1.0);
  EXPECT_FLOAT_EQ(state(1), 1.0 - M_PI - 0.1);
  EXPECT_FLOAT_EQ(state(2), 1.0);
  EXPECT_FLOAT_EQ(state(3), 1.0);
  EXPECT_FLOAT_EQ(state(4), 0.5);  // max steer angle is 0.5
  EXPECT_FLOAT_EQ(state(5), 0);
  EXPECT_FLOAT_EQ(state(6), 1);

  state << 0, M_PI - 0.1, 0, 0, 0, 0, 0;
  state_der << 1, 1, 1, 1, 1, 2, 1;
  dynamics.updateState(state, state_der, 1.0);
  EXPECT_TRUE(state_der != RacerDubins::state_array::Zero());
  EXPECT_FLOAT_EQ(state(0), 1.0);
  EXPECT_FLOAT_EQ(state(1), 1.0 - M_PI - 0.1);
  EXPECT_FLOAT_EQ(state(2), 1.0);
  EXPECT_FLOAT_EQ(state(3), 1.0);
  EXPECT_FLOAT_EQ(state(4), 0.5);  // max steer angle is 0.5
  EXPECT_FLOAT_EQ(state(5), 1.0);
  EXPECT_FLOAT_EQ(state(6), 1);

  state << 0, -M_PI + 0.1, 0, 0, 0, 0, 0;
  state_der << 1, -1, 1, 1, 1, 1, 1;
  dynamics.updateState(state, state_der, 1.0);
  EXPECT_TRUE(state_der != RacerDubins::state_array::Zero());
  EXPECT_FLOAT_EQ(state(0), 1.0);
  EXPECT_FLOAT_EQ(state(1), M_PI + 0.1 - 1.0);
  EXPECT_FLOAT_EQ(state(2), 1.0);
  EXPECT_FLOAT_EQ(state(3), 1.0);
  EXPECT_FLOAT_EQ(state(4), 0.5);  // max steer angle is 0.5
  EXPECT_FLOAT_EQ(state(5), 1);
  EXPECT_FLOAT_EQ(state(6), 1);
}

TEST(RacerDubins, TestUpdateStateGPU)
{
  RacerDubins dynamics = RacerDubins();
  dynamics.GPUSetup();

  Eigen::Matrix<float, RacerDubins::CONTROL_DIM, 100> control_trajectory;
  control_trajectory = Eigen::Matrix<float, RacerDubins::CONTROL_DIM, 100>::Random();
  Eigen::Matrix<float, RacerDubins::STATE_DIM, 100> state_trajectory;
  state_trajectory = Eigen::Matrix<float, RacerDubins::STATE_DIM, 100>::Random();

  std::vector<std::array<float, RacerDubins::STATE_DIM>> s(100);
  std::vector<std::array<float, RacerDubins::STATE_DIM>> s_der(100);
  // steering, throttle
  std::vector<std::array<float, RacerDubins::CONTROL_DIM>> u(100);

  RacerDubins::state_array state;
  RacerDubins::control_array control;
  RacerDubins::state_array state_der_cpu = RacerDubins::state_array::Zero();

  // Run dynamics on dynamicsU
  // Run dynamics on GPU
  for (int y_dim = 1; y_dim <= 4; y_dim++)
  {
    for (int state_index = 0; state_index < s.size(); state_index++)
    {
      for (int dim = 0; dim < s[0].size(); dim++)
      {
        s[state_index][dim] = state_trajectory.col(state_index)(dim);
      }
      for (int dim = 0; dim < u[0].size(); dim++)
      {
        u[state_index][dim] = control_trajectory.col(state_index)(dim);
      }
    }

    launchComputeStateDerivTestKernel<RacerDubins, RacerDubins::STATE_DIM, RacerDubins::CONTROL_DIM>(dynamics, s, u,
                                                                                                     s_der, y_dim);
    launchUpdateStateTestKernel<RacerDubins, RacerDubins::STATE_DIM>(dynamics, s, s_der, 0.1f, y_dim);
    for (int point = 0; point < 100; point++)
    {
      RacerDubins::state_array state = state_trajectory.col(point);
      RacerDubins::control_array control = control_trajectory.col(point);
      RacerDubins::state_array state_der_cpu = RacerDubins::state_array::Zero();

      dynamics.computeDynamics(state, control, state_der_cpu);
      dynamics.updateState(state, state_der_cpu, 0.1f);
      for (int dim = 0; dim < RacerDubins::STATE_DIM; dim++)
      {
        EXPECT_NEAR(state_der_cpu(dim), s_der[point][dim], 1e-5) << "at index " << point << " with y_dim " << y_dim;
        EXPECT_NEAR(state(dim), s[point][dim], 1e-4) << "at index " << point << " with y_dim " << y_dim;
        EXPECT_TRUE(isfinite(s[point][dim]));
      }
    }
  }
  dynamics.freeCudaMem();
}

TEST(RacerDubins, ComputeStateTrajectoryFiniteTest)
{
  RacerDubins dynamics = RacerDubins();
  RacerDubinsParams params;
  params.c_t[0] = 3.0;
  params.c_b[0] = 0.2;
  params.c_v[0] = 0.2;
  params.c_0 = 0.2;
  params.wheel_base = 3.0;
  params.steering_constant = 1.0;
  dynamics.setParams(params);

  Eigen::Matrix<float, RacerDubins::CONTROL_DIM, 500> control_trajectory;
  control_trajectory = Eigen::Matrix<float, RacerDubins::CONTROL_DIM, 500>::Zero();
  Eigen::Matrix<float, RacerDubins::STATE_DIM, 500> state_trajectory;
  state_trajectory = Eigen::Matrix<float, RacerDubins::STATE_DIM, 500>::Zero();
  RacerDubins::state_array state_der;
  RacerDubins::state_array x;
  x << 0, 1.46919e-6, 0.0140179, 1.09739e-8, -0.000735827;

  for (int i = 0; i < 500; i++)
  {
    RacerDubins::control_array u = control_trajectory.col(i);
    dynamics.computeDynamics(x, u, state_der);
    EXPECT_TRUE(x.allFinite());
    EXPECT_TRUE(u.allFinite());
    EXPECT_TRUE(state_der.allFinite());
    dynamics.updateState(x, state_der, 0.02);
    EXPECT_TRUE(x.allFinite());
    EXPECT_TRUE(u.allFinite());
    EXPECT_TRUE(state_der != RacerDubins::state_array::Zero());
  }
  params.steering_constant = 0.5;
  dynamics.setParams(params);

  x << 0, 1.46919e-6, 0.0140179, 1.09739e-8, -1.0;
  for (int i = 0; i < 500; i++)
  {
    RacerDubins::control_array u = control_trajectory.col(i);
    dynamics.computeDynamics(x, u, state_der);
    EXPECT_TRUE(x.allFinite());
    EXPECT_TRUE(u.allFinite());
    EXPECT_TRUE(state_der.allFinite());
    dynamics.updateState(x, state_der, 0.02);
    EXPECT_TRUE(x.allFinite());
    EXPECT_TRUE(u.allFinite());
    EXPECT_TRUE(state_der != RacerDubins::state_array::Zero());
  }
}

/*
class LinearDummy : public RacerDubins {
public:
  bool computeGrad(const Eigen::Ref<const state_array> & state,
                   const Eigen::Ref<const control_array>& control,
                   Eigen::Ref<dfdx> A,
                   Eigen::Ref<dfdu> B) {
    return false;
  };
};

TEST(RacerDubins, TestComputeGradComputation) {
  Eigen::Matrix<float, RacerDubins::STATE_DIM, RacerDubins::STATE_DIM + RacerDubins::CONTROL_DIM> numeric_jac;
  Eigen::Matrix<float, RacerDubins::STATE_DIM, RacerDubins::STATE_DIM + RacerDubins::CONTROL_DIM> analytic_jac;
  RacerDubins::state_array state;
  state << 1, 2, 3, 4;
  RacerDubins::control_array control;
  control << 5;

  auto analytic_grad_model = RacerDubins();

  RacerDubins::dfdx A_analytic = RacerDubins::dfdx::Zero();
  RacerDubins::dfdu B_analytic = RacerDubins::dfdu::Zero();

  analytic_grad_model.computeGrad(state, control, A_analytic, B_analytic);

  auto numerical_grad_model = LinearDummy();

  std::shared_ptr<ModelWrapperDDP<LinearDummy>> ddp_model =
std::make_shared<ModelWrapperDDP<LinearDummy>>(&numerical_grad_model);

  analytic_jac.leftCols<RacerDubins::STATE_DIM>() = A_analytic;
  analytic_jac.rightCols<RacerDubins::CONTROL_DIM>() = B_analytic;
  numeric_jac = ddp_model->df(state, control);

  ASSERT_LT((numeric_jac - analytic_jac).norm(), 1e-3) << "Numeric Jacobian\n" << numeric_jac << "\nAnalytic Jacobian\n"
<< analytic_jac;
}

*/
