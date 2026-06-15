#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <mppi/dynamics/racer_dubins/racer_dubins_elevation_lstm_steering.cuh>
#include <kernel_tests/dynamics/dynamics_generic_kernel_tests.cuh>
#include <mppi/ddp/ddp_model_wrapper.h>
#include <racer_test_networks.h>
#include <cuda_runtime.h>

class RacerDubinsElevationLSTMSteeringTest : public ::testing::Test
{
public:
  cudaStream_t stream;

  void SetUp() override
  {
    CudaCheckError();
    HANDLE_ERROR(cudaStreamCreate(&stream));
  }

  void TearDown() override
  {
    CudaCheckError();
    HANDLE_ERROR(cudaStreamDestroy(stream));
  }

  std::vector<int> init_output_layers = { 23, 100, 8 };
  std::vector<int> output_layers = { 8, 20, 1 };
};

TEST_F(RacerDubinsElevationLSTMSteeringTest, Template)
{
  auto dynamics = RacerDubinsElevationLSTMSteering(3, 20, init_output_layers, 4, 4, output_layers, 11);
  EXPECT_EQ(19, RacerDubinsElevationLSTMSteering::STATE_DIM);
  EXPECT_EQ(2, RacerDubinsElevationLSTMSteering::CONTROL_DIM);
  EXPECT_TRUE(dynamics.checkRequiresBuffer());
  EXPECT_NE(dynamics.getTextureHelper(), nullptr);
  EXPECT_EQ(dynamics.getBlkSharedSizeBytes(), 384);
  EXPECT_EQ(dynamics.getGrdSharedSizeBytes(), 1408);
}

TEST_F(RacerDubinsElevationLSTMSteeringTest, BindStream)
{
  auto dynamics = RacerDubinsElevationLSTMSteering(3, 20, init_output_layers, 4, 4, output_layers, 11);
  dynamics.bindToStream(stream);

  EXPECT_EQ(dynamics.stream_, stream) << "Stream binding failure.";
  EXPECT_NE(dynamics.getTextureHelper(), nullptr);
  EXPECT_EQ(dynamics.getTextureHelper()->stream_, stream);
  EXPECT_NE(dynamics.getHelper(), nullptr);
  EXPECT_EQ(dynamics.getHelper()->getLSTMModel()->stream_, stream);

  auto dynamics2 = RacerDubinsElevationLSTMSteering(3, 20, init_output_layers, 4, 4, output_layers, 11, stream);

  EXPECT_EQ(dynamics2.stream_, stream) << "Stream binding failure.";
  EXPECT_NE(dynamics2.getTextureHelper(), nullptr);
  EXPECT_EQ(dynamics2.getTextureHelper()->stream_, stream);
  EXPECT_NE(dynamics2.getHelper(), nullptr);
  EXPECT_EQ(dynamics2.getHelper()->getLSTMModel()->stream_, stream);
}

/*
float c_t = 1.3;
float c_b = 2.5;
float c_v = 3.7;
float c_0 = 4.9;
float wheel_base = 0.3;
 */

// TEST_F(RacerDubinsElevationLSTMSteeringTest, ComputeDynamics)
// {
//   RacerDubinsElevationLSTMSteering dynamics = RacerDubinsElevationLSTMSteering();
//   auto params = dynamics.getParams();
//   RacerDubinsElevationLSTMSteering::state_array x = RacerDubinsElevationLSTMSteering::state_array::Zero();
//   RacerDubinsElevationLSTMSteering::control_array u = RacerDubinsElevationLSTMSteering::control_array::Zero();

//   // computeDynamics should not touch the roll/pitch element
//   RacerDubinsElevationLSTMSteering::state_array next_x = RacerDubinsElevationLSTMSteering::state_array::Ones() *
//   0.153; dynamics.computeDynamics(x, u, next_x); EXPECT_FLOAT_EQ(next_x(0), 4.9); EXPECT_FLOAT_EQ(next_x(1), 0);
//   EXPECT_FLOAT_EQ(next_x(2), 0);
//   EXPECT_FLOAT_EQ(next_x(3), 0);
//   EXPECT_FLOAT_EQ(next_x(4), 0);
//   EXPECT_FLOAT_EQ(next_x(5), 0.153);
//   EXPECT_FLOAT_EQ(next_x(6), 0.153);

//   x << 1, M_PI_2, 0, 3, 0, 0.5, -0.5, 0.0, 0.0;
//   u << 1, 0;
//   dynamics.computeDynamics(x, u, next_x);
//   EXPECT_FLOAT_EQ(next_x(0), 4.9 + 2.6 - 4.7 - sinf(-0.5) * -9.81);
//   EXPECT_FLOAT_EQ(next_x(1), 0);
//   EXPECT_NEAR(next_x(2), 0, 1e-7);
//   EXPECT_FLOAT_EQ(next_x(3), 1);
//   EXPECT_FLOAT_EQ(next_x(4), 0);
//   EXPECT_FLOAT_EQ(next_x(5), 0.153);
//   EXPECT_FLOAT_EQ(next_x(6), 0.153);

//   x << 1, M_PI_2, 0, 3, 0, 0.5, -0.5, 0.0, 0.0;
//   u << -1, 0;
//   dynamics.computeDynamics(x, u, next_x);
//   EXPECT_FLOAT_EQ(next_x(0), 4.9 - 3.5 - 4.7 - sinf(-0.5) * -9.81);
//   EXPECT_FLOAT_EQ(next_x(1), 0);
//   EXPECT_NEAR(next_x(2), 0, 1e-7);
//   EXPECT_FLOAT_EQ(next_x(3), 1);
//   EXPECT_FLOAT_EQ(next_x(4), 0);
//   EXPECT_FLOAT_EQ(next_x(5), 0.153);
//   EXPECT_FLOAT_EQ(next_x(6), 0.153);

//   x << -1, 0, 0, 3, 0, 0.5, -0.5, 0.0, 0.0;
//   u << 1, 0;
//   dynamics.computeDynamics(x, u, next_x);
//   EXPECT_FLOAT_EQ(next_x(0), 4.9 + 4.7 + 2.6 - sinf(-0.5) * -9.81);
//   EXPECT_FLOAT_EQ(next_x(1), 0);
//   EXPECT_FLOAT_EQ(next_x(2), -1);
//   EXPECT_FLOAT_EQ(next_x(3), 0);
//   EXPECT_FLOAT_EQ(next_x(4), 0);
//   EXPECT_FLOAT_EQ(next_x(5), 0.153);
//   EXPECT_FLOAT_EQ(next_x(6), 0.153);

//   x << -1, 0, 0, 3, 0, 0.5, -0.5, 0.0, 0.0;
//   u << -1, 0;
//   dynamics.computeDynamics(x, u, next_x);
//   EXPECT_FLOAT_EQ(next_x(0), 4.9 + 3.5 + 4.7 - sinf(-0.5) * -9.81);
//   EXPECT_FLOAT_EQ(next_x(1), 0);
//   EXPECT_FLOAT_EQ(next_x(2), -1);
//   EXPECT_FLOAT_EQ(next_x(3), 0);
//   EXPECT_FLOAT_EQ(next_x(4), 0);
//   EXPECT_FLOAT_EQ(next_x(5), 0.153);
//   EXPECT_FLOAT_EQ(next_x(6), 0.153);

//   x << 7, 0, 0, 3, 0, 0.5, -0.5, 0.0, 0.0;
//   u << 1, 0;
//   dynamics.computeDynamics(x, u, next_x);
//   EXPECT_FLOAT_EQ(next_x(0), 4.9 + 3.9 - 5.7 * 7 - sinf(-0.5) * -9.81);
//   EXPECT_FLOAT_EQ(next_x(1), 0);
//   EXPECT_FLOAT_EQ(next_x(2), 7);
//   EXPECT_FLOAT_EQ(next_x(3), 0);
//   EXPECT_FLOAT_EQ(next_x(4), 0);
//   EXPECT_FLOAT_EQ(next_x(5), 0.153);
//   EXPECT_FLOAT_EQ(next_x(6), 0.153);

//   x << -7, 0, 0, 3, 0, 0.5, -0.5, 0.0, 0.0;
//   u << 1, 0;
//   dynamics.computeDynamics(x, u, next_x);
//   EXPECT_FLOAT_EQ(next_x(0), 4.9 + 3.9 + 5.7 * 7 - sinf(-0.5) * -9.81);
//   EXPECT_FLOAT_EQ(next_x(1), 0);
//   EXPECT_FLOAT_EQ(next_x(2), -7);
//   EXPECT_FLOAT_EQ(next_x(3), 0);
//   EXPECT_FLOAT_EQ(next_x(4), 0);
//   EXPECT_FLOAT_EQ(next_x(5), 0.153);
//   EXPECT_FLOAT_EQ(next_x(6), 0.153);

//   x << 7, 0, 0, 3, 0, 0.5, -0.5, 0.0, 0.0;
//   u << -1, 0;
//   dynamics.computeDynamics(x, u, next_x);
//   EXPECT_FLOAT_EQ(next_x(0), 4.9 - 4.5 - 5.7 * 7 - sinf(-0.5) * -9.81);
//   EXPECT_FLOAT_EQ(next_x(1), 0);
//   EXPECT_FLOAT_EQ(next_x(2), 7);
//   EXPECT_FLOAT_EQ(next_x(3), 0);
//   EXPECT_FLOAT_EQ(next_x(4), 0);
//   EXPECT_FLOAT_EQ(next_x(5), 0.153);
//   EXPECT_FLOAT_EQ(next_x(6), 0.153);

//   x << -7, 0, 0, 3, 0, 0.5, -0.5, 0.0, 0.0;
//   u << -1, 0;
//   dynamics.computeDynamics(x, u, next_x);
//   EXPECT_FLOAT_EQ(next_x(0), 4.9 + 4.5 + 5.7 * 7 - sinf(-0.5) * -9.81);
//   EXPECT_FLOAT_EQ(next_x(1), 0);
//   EXPECT_FLOAT_EQ(next_x(2), -7);
//   EXPECT_FLOAT_EQ(next_x(3), 0);
//   EXPECT_FLOAT_EQ(next_x(4), 0);
//   EXPECT_FLOAT_EQ(next_x(5), 0.153);
//   EXPECT_FLOAT_EQ(next_x(6), 0.153);

//   x << 1, M_PI_2, 0, 3, 0, 0.5, -0.5, 0.0, 0.0;
//   u << 0, 1;
//   dynamics.computeDynamics(x, u, next_x);
//   EXPECT_FLOAT_EQ(next_x(0), 4.9 - 4.7 - sinf(-0.5) * -9.81);
//   EXPECT_FLOAT_EQ(next_x(1), (1 / .3) * tan(0));
//   EXPECT_NEAR(next_x(2), 0, 1e-7);
//   EXPECT_FLOAT_EQ(next_x(3), 1);
//   EXPECT_FLOAT_EQ(next_x(4), 5 * 0.6);
//   EXPECT_FLOAT_EQ(next_x(5), 0.153);
//   EXPECT_FLOAT_EQ(next_x(6), 0.153);

//   x << -1, M_PI_2, 0, 3, 5.0, 0.5, -0.5, 0.0, 0.0;
//   u << -1, -1;
//   dynamics.computeDynamics(x, u, next_x);
//   EXPECT_FLOAT_EQ(next_x(0), 4.9 + 4.7 + 3.5 - sinf(-0.5) * -9.81);
//   EXPECT_FLOAT_EQ(next_x(1), (-1 / .3) * tan(5.0 / -10.2));
//   EXPECT_NEAR(next_x(2), 0, 1e-7);
//   EXPECT_FLOAT_EQ(next_x(3), -1);
//   EXPECT_FLOAT_EQ(next_x(4), -5);
//   EXPECT_FLOAT_EQ(next_x(5), 0.153);
//   EXPECT_FLOAT_EQ(next_x(6), 0.153);

//   x << -0.4, M_PI_2, 0, 3, 5.0, 0.5, -0.5, 0.0, 0.0;
//   u << -1, -1;
//   dynamics.computeDynamics(x, u, next_x);
//   EXPECT_FLOAT_EQ(next_x(0), 4.9 + 3.7 * 0.4 + 2.5 * 0.4 - sinf(-0.5) * -9.81);
//   EXPECT_FLOAT_EQ(next_x(1), (-0.4 / .3) * tan(5.0 / -9.1));
//   EXPECT_NEAR(next_x(2), 0, 1e-7);
//   EXPECT_FLOAT_EQ(next_x(3), -0.4);
//   EXPECT_FLOAT_EQ(next_x(4), -5);
//   EXPECT_FLOAT_EQ(next_x(5), 0.153);
//   EXPECT_FLOAT_EQ(next_x(6), 0.153);

//   x << 0.4, M_PI_2, 0, 3, 5.0, 0.5, -0.5, 0.0, 0.0;
//   u << 0.1, -1;
//   dynamics.computeDynamics(x, u, next_x);
//   EXPECT_FLOAT_EQ(next_x(0), 4.9 - 3.7 * 0.4 - sinf(-0.5) * -9.81);
//   EXPECT_FLOAT_EQ(next_x(1), (0.4 / .3) * tan(5.0 / -9.1));
//   EXPECT_NEAR(next_x(2), 0, 1e-7);
//   EXPECT_FLOAT_EQ(next_x(3), 0.4);
//   EXPECT_FLOAT_EQ(next_x(4), -5);
//   EXPECT_FLOAT_EQ(next_x(5), 0.153);
//   EXPECT_FLOAT_EQ(next_x(6), 0.153);
// }

// TEST_F(RacerDubinsElevationLSTMSteeringTest, TestModelGPU)
// {
//   RacerDubinsElevationLSTMSteering dynamics = RacerDubinsElevationLSTMSteering();
//   dynamics.GPUSetup();
//   CudaCheckError();

//   Eigen::Matrix<float, RacerDubinsElevationLSTMSteering::CONTROL_DIM, 100> control_trajectory;
//   control_trajectory = Eigen::Matrix<float, RacerDubinsElevationLSTMSteering::CONTROL_DIM, 100>::Random();
//   Eigen::Matrix<float, RacerDubinsElevationLSTMSteering::STATE_DIM, 100> state_trajectory;
//   state_trajectory = Eigen::Matrix<float, RacerDubinsElevationLSTMSteering::STATE_DIM, 100>::Random();

//   std::vector<std::array<float, 9>> s(100);
//   std::vector<std::array<float, 9>> s_der(100);
//   // steering, throttle
//   std::vector<std::array<float, 2>> u(100);
//   for (int state_index = 0; state_index < s.size(); state_index++)
//   {
//     for (int dim = 0; dim < s[0].size(); dim++)
//     {
//       s[state_index][dim] = state_trajectory.col(state_index)(dim);
//     }
//     for (int dim = 0; dim < u[0].size(); dim++)
//     {
//       u[state_index][dim] = control_trajectory.col(state_index)(dim);
//     }
//   }

//   // These variables will be changed so initialized to the right size only

//   // Run dynamics on dynamicsU
//   // Run dynamics on GPU
//   for (int y_dim = 1; y_dim <= 4; y_dim++)
//   {
//     launchComputeDynamicsTestKernel<RacerDubinsElevationLSTMSteering, 9, 2>(dynamics, s, u, s_der, y_dim);
//     for (int point = 0; point < 100; point++)
//     {
//       RacerDubinsElevationLSTMSteering::state_array state = state_trajectory.col(point);
//       RacerDubinsElevationLSTMSteering::control_array control = control_trajectory.col(point);
//       RacerDubinsElevationLSTMSteering::state_array state_der_cpu =
//       RacerDubinsElevationLSTMSteering::state_array::Zero();

//       dynamics.computeDynamics(state, control, state_der_cpu);
//       for (int dim = 0; dim < 6; dim++)
//       {
//         EXPECT_NEAR(state_der_cpu(dim), s_der[point][dim], 1e-5)
//             << "at point " << point << " dim " << dim << " with y_dim " << y_dim;
//         EXPECT_TRUE(isfinite(s_der[point][dim]));
//       }
//     }
//   }

//   dynamics.freeCudaMem();
//   CudaCheckError();
// }

// TEST_F(RacerDubinsElevationLSTMSteeringTest, TestUpdateState)
// {
//   CudaCheckError();
//   RacerDubinsElevationLSTMSteering dynamics = RacerDubinsElevationLSTMSteering();
//   RacerDubinsElevationLSTMSteering::state_array state;
//   RacerDubinsElevationLSTMSteering::state_array next_state;
//   RacerDubinsElevationLSTMSteering::state_array state_der;

//   // TODO add in the elevation map

//   state << 0, 0, 0, 0, 0, -0.5, 0.5;
//   state_der << 1, 1, 1, 1, 1, 0, 0;
//   dynamics.updateState(state, next_state, state_der, 0.1);
//   EXPECT_TRUE(state_der != RacerDubinsElevationLSTMSteering::state_array::Zero());
//   EXPECT_FLOAT_EQ(next_state(0), 0.1);
//   EXPECT_FLOAT_EQ(next_state(1), 0.1);
//   EXPECT_FLOAT_EQ(next_state(2), 0.1);
//   EXPECT_FLOAT_EQ(next_state(3), 0.1);
//   EXPECT_FLOAT_EQ(next_state(4), 0.1);
//   EXPECT_FLOAT_EQ(next_state(5), 0.0);
//   EXPECT_FLOAT_EQ(next_state(6), 0.0);

//   state << 0, M_PI - 0.1, 0, 0, 0, -0.5, 0.5;
//   state_der << 1, 1, 1, 1, 1;
//   dynamics.updateState(state, next_state, state_der, 1.0);
//   EXPECT_TRUE(state_der != RacerDubinsElevationLSTMSteering::state_array::Zero());
//   EXPECT_FLOAT_EQ(next_state(0), 1.0);
//   EXPECT_FLOAT_EQ(next_state(1), 1.0 - M_PI - 0.1);
//   EXPECT_FLOAT_EQ(next_state(2), 1.0);
//   EXPECT_FLOAT_EQ(next_state(3), 1.0);
//   EXPECT_FLOAT_EQ(next_state(4), 0.5);
//   EXPECT_FLOAT_EQ(next_state(5), 0.0);
//   EXPECT_FLOAT_EQ(next_state(6), 0.0);

//   state << 0, -M_PI + 0.1, 0, 0, 0, -0.5, 0.5;
//   state_der << 1, -1, 1, 1, 1;
//   dynamics.updateState(state, next_state, state_der, 1.0);
//   EXPECT_TRUE(state_der != RacerDubinsElevationLSTMSteering::state_array::Zero());
//   EXPECT_FLOAT_EQ(next_state(0), 1.0);
//   EXPECT_FLOAT_EQ(next_state(1), M_PI + 0.1 - 1.0);
//   EXPECT_FLOAT_EQ(next_state(2), 1.0);
//   EXPECT_FLOAT_EQ(next_state(3), 1.0);
//   EXPECT_FLOAT_EQ(next_state(4), 0.5);
//   EXPECT_FLOAT_EQ(next_state(5), 0.0);
//   EXPECT_FLOAT_EQ(next_state(6), 0.0);

//   CudaCheckError();
// }

TEST_F(RacerDubinsElevationLSTMSteeringTest, TestStep)
{
  GTEST_SKIP() << "Skipping test since they have not been updated to accel ";
  CudaCheckError();
  using DYN = RacerDubinsElevationLSTMSteering;
  const float tol = 1e-6;
  auto dynamics = RacerDubinsElevationLSTMSteering(3, 20, init_output_layers, 4, 4, output_layers, 11);
  auto params = dynamics.getParams();
  params.c_0 = 0;
  params.c_b[0] = 1;
  params.c_b[1] = 10;
  params.c_b[2] = 100;
  params.c_v[0] = 0.25;
  params.c_v[1] = 0.5;
  params.c_v[2] = 0.75;
  params.c_t[0] = 2;
  params.c_t[1] = 20;
  params.c_t[2] = 200;
  params.low_min_throttle = 0.2;
  params.steer_command_angle_scale = 0.5;
  params.steering_constant = 0.5;
  params.wheel_base = 0.5;
  params.max_steer_rate = 5;
  params.max_steer_angle = 5;
  dynamics.setParams(params);
  DYN::state_array state;
  DYN::state_array next_state;
  DYN::state_array state_der = DYN::state_array::Zero();
  DYN::control_array control;
  DYN::output_array output;
  float dt = 0.1;
  // TODO add in the elevation map

  auto model = dynamics.getHelper();
  model->getLSTMModel()->getOutputModel()->setAllWeights(0.1f);
  model->getInitModel()->getOutputModel()->setAllWeights(0.01f);
  model->getLSTMModel()->setAllValues(0.3f);
  model->getInitModel()->setAllValues(0.01f);

  Eigen::MatrixXf buffer = model->getEmptyBufferMatrix();
  buffer.setOnes();
  buffer = buffer * 0.01f;

  // Basic initial state and no movement should stay still
  state << 0, 0, 0, 0, 0, -0.0, 0.0, 0, 0;
  control << 0, 0;
  dynamics.step(state, next_state, state_der, control, output, 0, dt);
  EXPECT_NEAR(next_state(0), 0.0, tol);
  EXPECT_NEAR(next_state(1), 0.0, tol);
  EXPECT_NEAR(next_state(2), 0.0, tol);
  EXPECT_NEAR(next_state(3), 0.0, tol);
  EXPECT_NEAR(next_state(4), 4.1500520706176758 * dt, tol);
  EXPECT_NEAR(next_state(5), 0.0, tol);
  EXPECT_NEAR(next_state(6), 0.0, tol);
  EXPECT_NEAR(next_state(7), 0.0, tol);
  EXPECT_NEAR(next_state(8), 4.1500520706176758, tol);
  EXPECT_NEAR(output(23), 0.0, tol);

  // Apply full throttle from zero state
  state << 0, 0, 0, 0, 0, -0.0, 0.0, 0, 0;
  control << 1, 0;
  dynamics.step(state, next_state, state_der, control, output, 0, dt);
  EXPECT_NEAR(state_der(0), 1.6, tol);
  EXPECT_NEAR(next_state(0), 0.16, tol);
  EXPECT_NEAR(next_state(1), 0.0, tol);
  EXPECT_NEAR(next_state(2), 0.0, tol);
  EXPECT_NEAR(next_state(3), 0.0, tol);
  EXPECT_NEAR(next_state(4), 5.2751355171203613 * dt, tol);
  EXPECT_NEAR(next_state(5), 0.0, tol);
  EXPECT_NEAR(next_state(6), 0.0, tol);
  EXPECT_NEAR(next_state(7), 0.0, tol);
  EXPECT_NEAR(next_state(8), 5.2751355171203613, tol);
  EXPECT_NEAR(output(23), 1.6, tol);

  // Apply throttle to a state with positive velocity
  state << 1, 0, 0, 0, 0, -0.0, 0.0, 0, 0;
  control << 1, 0;
  model->initializeLSTM(buffer);
  dynamics.step(state, next_state, state_der, control, output, 0, dt);
  EXPECT_NEAR(state_der(0), 5.5, tol);
  EXPECT_NEAR(next_state(0), 1.55, tol);
  EXPECT_NEAR(next_state(1), 0.0, tol);
  EXPECT_NEAR(next_state(2), 0.1, tol);
  EXPECT_NEAR(next_state(3), 0.0, tol);
  EXPECT_NEAR(next_state(4), 7.1901092529296875 * dt, tol);
  EXPECT_NEAR(next_state(5), 0.0, tol);
  EXPECT_NEAR(next_state(6), 0.0, tol);
  EXPECT_NEAR(next_state(7), 0.0, tol);
  EXPECT_NEAR(next_state(8), 7.1901092529296875, tol);
  EXPECT_NEAR(output(23), 5.5, tol);

  // Apply full throttle and half left turn to origin state
  state << 0, 0, 0, 0, 0, -0.0, 0.0, 0, 0;
  control << 1, 0.5;
  model->initializeLSTM(buffer);
  dynamics.step(state, next_state, state_der, control, output, 0, dt);
  EXPECT_NEAR(state_der(0), 1.6, tol);
  EXPECT_NEAR(next_state(0), 0.16, tol);
  EXPECT_NEAR(next_state(1), 0.0, tol);
  EXPECT_NEAR(next_state(2), 0.0, tol);
  EXPECT_NEAR(next_state(3), 0.0, tol);
  EXPECT_NEAR(next_state(4), 6.1967658996582031 * dt, tol);
  EXPECT_NEAR(next_state(5), 0.0, tol);
  EXPECT_NEAR(next_state(6), 0.0, tol);
  EXPECT_NEAR(next_state(7), 0.0, tol);
  EXPECT_NEAR(next_state(8), 6.1967658996582031, tol);
  EXPECT_NEAR(output(23), 1.6, tol);

  // Apply full throttle and half left turn to a moving state oriented 30 degrees to the left
  float yaw = M_PI / 6;
  state << 1.0, yaw, 0, 0, 0, -0.0, 0.0, 0, 0;
  control << 1, 0.5;
  model->initializeLSTM(buffer);
  dynamics.step(state, next_state, state_der, control, output, 0, dt);
  EXPECT_NEAR(state_der(0), 5.5, tol);
  EXPECT_NEAR(next_state(0), 1.55, tol);
  EXPECT_NEAR(next_state(1), yaw, tol);
  EXPECT_NEAR(next_state(2), 1 * cos(yaw) * dt, tol);
  EXPECT_NEAR(next_state(3), 1 * sin(yaw) * dt, tol);
  EXPECT_NEAR(next_state(4), 9.0641689300537109 * dt, tol);
  EXPECT_NEAR(next_state(5), 0.0, tol);
  EXPECT_NEAR(next_state(6), 0.0, tol);
  EXPECT_NEAR(next_state(7), 0.0, tol);
  EXPECT_NEAR(next_state(8), 9.0641689300537109, tol);
  EXPECT_NEAR(output(23), 5.5, tol);

  // Apply full throttle and half left turn to a moving state oriented 30 degrees to the left which is already turning
  float steer_angle = M_PI / 8;
  state << 1.0, yaw, 0, 0, steer_angle, -0.0, 0.0, 0, 0;
  control << 1, 0.5;
  model->initializeLSTM(buffer);
  dynamics.step(state, next_state, state_der, control, output, 0, dt);
  EXPECT_NEAR(state_der(0), 5.5, tol);
  EXPECT_NEAR(next_state(0), 1.55, tol);
  EXPECT_NEAR(next_state(1), yaw + -0.086361105 * dt, tol);
  EXPECT_NEAR(next_state(2), 1 * cos(yaw) * dt, tol);
  EXPECT_NEAR(next_state(3), 1 * sin(yaw) * dt, tol);
  EXPECT_NEAR(next_state(4), steer_angle + 9.3808889389038086 * dt, tol);
  EXPECT_NEAR(next_state(5), 0.0, tol);
  EXPECT_NEAR(next_state(6), 0.0, tol);
  EXPECT_NEAR(next_state(7), 0.0, tol);
  EXPECT_NEAR(next_state(8), 9.3808889389038086, tol);
  EXPECT_NEAR(output(23), 5.5, tol);

  // Apply full brake and half left turn to a moving state oriented 30 degrees to the left which is already turning
  state << 1.0, yaw, 0, 0, steer_angle, 1.0, -0.0, 0.0, 0, 0;
  control << -1, 0.5;
  model->initializeLSTM(buffer);
  dynamics.step(state, next_state, state_der, control, output, 0, dt);
  EXPECT_NEAR(state_der(0), -5.5, tol);
  EXPECT_NEAR(next_state(0), 1 - 5.5 * dt, tol);
  EXPECT_NEAR(next_state(1), yaw + -0.086361105 * dt, tol);
  EXPECT_NEAR(next_state(2), 1 * cos(yaw) * dt, tol);
  EXPECT_NEAR(next_state(3), 1 * sin(yaw) * dt, tol);
  EXPECT_NEAR(next_state(4), steer_angle + 9.3808889389038086 * dt, tol);
  EXPECT_NEAR(next_state(5), 1.0, tol);
  EXPECT_NEAR(next_state(6), 0.0, tol);
  EXPECT_NEAR(next_state(7), 0.0, tol);
  EXPECT_NEAR(next_state(8), 9.3808889389038086, tol);
  EXPECT_NEAR(output(23), -5.5, tol);

  /**
   * Apply full brake and half left turn to a moving state oriented 30 degrees to the left which is already turning
   * and on a downward facing hill
   */
  float pitch = 20 * M_PI / 180;
  state << 1.0, yaw, 0, 0, steer_angle, 1, -0.0, pitch, 0, 0;
  control << -1, 0.5;
  model->initializeLSTM(buffer);
  dynamics.step(state, next_state, state_der, control, output, 0, dt);
  EXPECT_NEAR(next_state(0), 1 + (-5.5 + 9.81 * sinf(pitch)) * dt, tol);
  EXPECT_NEAR(next_state(1), yaw + -0.086361105 * dt, tol);
  EXPECT_NEAR(next_state(2), 1 * cos(yaw) * dt, tol);
  EXPECT_NEAR(next_state(3), 1 * sin(yaw) * dt, tol);
  EXPECT_NEAR(next_state(4), steer_angle + 9.3808889389038086 * dt, tol);
  EXPECT_NEAR(next_state(5), 1.0, tol);
  EXPECT_NEAR(next_state(6), 0.0, tol);
  EXPECT_NEAR(next_state(7), 0.0, tol);
  EXPECT_NEAR(next_state(8), 9.3808889389038086, tol);
  EXPECT_NEAR(output(23), (-5.5 + 9.81 * sinf(pitch)), tol);

  /**
   * Apply full brake and half left turn to a backwards moving state oriented 30 degrees to the left which is already
   * turning and on a downward facing hill
   */
  state << -1.0, yaw, 0, 0, steer_angle, 1, -0.0, pitch, 0, 0;
  control << -1, 0.5;
  model->initializeLSTM(buffer);
  dynamics.step(state, next_state, state_der, control, output, 0, dt);
  EXPECT_NEAR(next_state(0), -1 + (5.5 + 9.81 * sinf(pitch)) * dt, tol);
  EXPECT_NEAR(next_state(1), yaw + 0.086361105 * dt, tol);
  EXPECT_NEAR(next_state(2), -1 * cos(yaw) * dt, tol);
  EXPECT_NEAR(next_state(3), -1 * sin(yaw) * dt, tol);
  EXPECT_NEAR(next_state(4), steer_angle + 3.5283551216125488 * dt, tol);
  EXPECT_NEAR(next_state(5), 1.0, tol);
  EXPECT_NEAR(next_state(6), 0.0, tol);
  EXPECT_NEAR(next_state(7), 0.0, tol);
  EXPECT_NEAR(next_state(8), 3.5283551216125488, tol);
  EXPECT_NEAR(output(23), (5.5 + 9.81 * sinf(pitch)), tol);

  /**
   * Apply full brake and half right turn to a backwards moving state oriented 30 degrees to the left which is already
   * turning and on a downward facing hill
   */
  state << -1.0, yaw, 0, 0, steer_angle, 1, -0.0, pitch, 0, 0;
  control << -1, -0.5;
  model->initializeLSTM(buffer);
  dynamics.step(state, next_state, state_der, control, output, 0, dt);
  EXPECT_NEAR(next_state(0), -1 + (5.5 + 9.81 * sinf(pitch)) * dt, tol);
  EXPECT_NEAR(next_state(1), yaw + 0.086361105 * dt, tol);
  EXPECT_NEAR(next_state(2), -1 * cos(yaw) * dt, tol);
  EXPECT_NEAR(next_state(3), -1 * sin(yaw) * dt, tol);
  EXPECT_NEAR(next_state(4), steer_angle + -0.32771033048629761 * dt, tol);
  EXPECT_NEAR(next_state(5), 1.0, tol);
  EXPECT_NEAR(next_state(6), 0.0, tol);
  EXPECT_NEAR(next_state(7), 0.0, tol);
  EXPECT_NEAR(next_state(8), -0.32771033048629761, tol);
  EXPECT_NEAR(output(23), (5.5 + 9.81 * sinf(pitch)), tol);

  /**
   * Apply full brake and half right turn to a backwards moving state with a huge steering angle to test max steer
   * angle and steering rate. We are also on a downward facing hill and are already oriented 30 degrees to the left
   */
  steer_angle *= 100;
  state << -1.0, yaw, 0, 0, steer_angle, 1, -0.0, pitch, 0, 0;
  control << -1, -0.5;
  model->initializeLSTM(buffer);
  dynamics.step(state, next_state, state_der, control, output, 0, dt);
  EXPECT_NEAR(next_state(0), -1 + (5.5 + 9.81 * sinf(pitch)) * dt, tol);
  EXPECT_NEAR(next_state(1), yaw + tan(steer_angle / -9.1) * dt * -2, tol);
  EXPECT_NEAR(next_state(2), -1 * cos(yaw) * dt, tol);
  EXPECT_NEAR(next_state(3), -1 * sin(yaw) * dt, tol);
  EXPECT_NEAR(next_state(4), params.max_steer_angle, tol);
  EXPECT_NEAR(next_state(5), 1.0, tol);
  EXPECT_NEAR(next_state(6), 0.0, tol);
  EXPECT_NEAR(next_state(7), 0.0, tol);
  EXPECT_NEAR(next_state(8), 15.97845268249511, tol);
  EXPECT_NEAR(output(23), (5.5 + 9.81 * sinf(pitch)), tol);
}

TEST_F(RacerDubinsElevationLSTMSteeringTest, TestStepGPUvsCPUNoNetwork)
{
  const int num_rollouts = 1024;
  const float dt = 0.1f;
  CudaCheckError();
  using DYN = RacerDubinsElevationLSTMSteering;
  RacerDubinsElevationLSTMSteering dynamics =
      RacerDubinsElevationLSTMSteering(3, 20, init_output_layers, 4, 4, output_layers, 11);

  cudaExtent extent = make_cudaExtent(10, 20, 0);
  TwoDTextureHelper<float>* helper = dynamics.getTextureHelper();
  helper->setExtent(0, extent);

  std::vector<float> data_vec;
  data_vec.resize(10 * 20);
  for (int i = 0; i < data_vec.size(); i++)
  {
    data_vec[i] = i * 1.0f;
  }

  std::array<float3, 3> new_rot_mat{};
  new_rot_mat[0] = make_float3(0, 1, 0);
  new_rot_mat[1] = make_float3(1, 0, 0);
  new_rot_mat[2] = make_float3(0, 0, 1);
  helper->updateRotation(0, new_rot_mat);
  helper->updateOrigin(0, make_float3(1, 2, 3));

  helper->updateTexture(0, data_vec);
  helper->updateResolution(0, 10);
  helper->enableTexture(0);
  helper->copyToDevice(true);

  CudaCheckError();
  dynamics.GPUSetup();
  CudaCheckError();

  EXPECT_NE(dynamics.getHelper()->getLSTMDevicePtr(), nullptr);
  EXPECT_NE(dynamics.network_d_, nullptr);
  EXPECT_EQ(dynamics.network_d_, dynamics.getHelper()->getLSTMDevicePtr());

  Eigen::Matrix<float, RacerDubinsElevationLSTMSteering::CONTROL_DIM, num_rollouts> control_trajectory;
  control_trajectory = Eigen::Matrix<float, RacerDubinsElevationLSTMSteering::CONTROL_DIM, num_rollouts>::Random();
  Eigen::Matrix<float, RacerDubinsElevationLSTMSteering::STATE_DIM, num_rollouts> state_trajectory;
  state_trajectory = Eigen::Matrix<float, RacerDubinsElevationLSTMSteering::STATE_DIM, num_rollouts>::Random();

  std::vector<std::array<float, RacerDubinsElevationLSTMSteering::STATE_DIM>> s(num_rollouts);
  std::vector<std::array<float, RacerDubinsElevationLSTMSteering::STATE_DIM>> s_next(num_rollouts);
  std::vector<std::array<float, RacerDubinsElevationLSTMSteering::STATE_DIM>> s_der(num_rollouts);
  // steering, throttle
  std::vector<std::array<float, RacerDubinsElevationLSTMSteering::CONTROL_DIM>> u(num_rollouts);

  RacerDubinsElevationLSTMSteering::state_array state;
  RacerDubinsElevationLSTMSteering::state_array next_state_cpu;
  RacerDubinsElevationLSTMSteering::control_array control;
  RacerDubinsElevationLSTMSteering::output_array output;
  RacerDubinsElevationLSTMSteering::state_array state_der_cpu = RacerDubinsElevationLSTMSteering::state_array::Zero();

  // Run dynamics on dynamicsU
  // Run dynamics on GPU
  for (int y_dim = 1; y_dim <= 16; y_dim++)
  {
    DYN::buffer_trajectory buffer;
    buffer["VEL_X"] = Eigen::VectorXf::Random(51);
    buffer["STEER_ANGLE"] = Eigen::VectorXf::Random(51);
    buffer["STEER_ANGLE_RATE"] = Eigen::VectorXf::Random(51);
    buffer["STEER_CMD"] = Eigen::VectorXf::Random(51);

    for (int state_index = 0; state_index < num_rollouts; state_index++)
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
    dynamics.updateFromBuffer(buffer);
    launchStepTestKernel<RacerDubinsElevationLSTMSteering, 16>(dynamics, s, u, s_der, s_next, 0, dt, y_dim);
    for (int point = 0; point < num_rollouts; point++)
    {
      dynamics.initializeDynamics(state, control, output, 0, 0);
      state = state_trajectory.col(point);
      control = control_trajectory.col(point);
      state_der_cpu = RacerDubinsElevationLSTMSteering::state_array::Zero();

      dynamics.step(state, next_state_cpu, state_der_cpu, control, output, 0, dt);
      // for (int dim = 0; dim < RacerDubinsElevationLSTMSteering::STATE_DIM; dim++)
      for (int dim = 0; dim < RacerDubinsElevationLSTMSteering::STATE_DIM; dim++)
      {
        EXPECT_NEAR(state_der_cpu(dim), s_der[point][dim], 1e-4)
            << "at index " << point << " with y_dim " << y_dim << " dim " << dim;
        // EXPECT_NEAR(state(dim), s[point][dim], 1e-4) << "at index " << point << " with y_dim " << y_dim;
        EXPECT_NEAR(next_state_cpu(dim), s_next[point][dim], 1e-4)
            << "at index " << point << " with y_dim " << y_dim << " dim " << dim;
        EXPECT_TRUE(isfinite(s_next[point][dim]));
      }
    }
  }
  dynamics.freeCudaMem();
}

TEST_F(RacerDubinsElevationLSTMSteeringTest, TestStepGPUvsCPU)
{
  const int num_rollouts = 64;
  const float dt = 0.1f;
  CudaCheckError();
  using DYN = RacerDubinsElevationLSTMSteering;
  RacerDubinsElevationLSTMSteering dynamics = RacerDubinsElevationLSTMSteering(mppi::tests::steering_lstm);
  EXPECT_FLOAT_EQ(dynamics.getParams().max_steer_rate, 17.590296);
  EXPECT_FLOAT_EQ(dynamics.getParams().steering_constant, 3.286375);
  EXPECT_FLOAT_EQ(dynamics.getParams().steer_accel_constant, 9.301527);
  EXPECT_FLOAT_EQ(dynamics.getParams().steer_accel_drag_constant, -0.60327667);

  cudaExtent extent = make_cudaExtent(10, 20, 0);
  TwoDTextureHelper<float>* helper = dynamics.getTextureHelper();
  helper->setExtent(0, extent);

  std::vector<float> data_vec;
  data_vec.resize(10 * 20);
  for (int i = 0; i < data_vec.size(); i++)
  {
    data_vec[i] = i * 1.0f;
  }

  std::array<float3, 3> new_rot_mat{};
  new_rot_mat[0] = make_float3(0, 1, 0);
  new_rot_mat[1] = make_float3(1, 0, 0);
  new_rot_mat[2] = make_float3(0, 0, 1);
  helper->updateRotation(0, new_rot_mat);
  helper->updateOrigin(0, make_float3(1, 2, 3));

  helper->updateTexture(0, data_vec);
  helper->updateResolution(0, 10);
  helper->enableTexture(0);
  helper->copyToDevice(true);

  CudaCheckError();
  dynamics.GPUSetup();
  CudaCheckError();

  EXPECT_NE(dynamics.getHelper()->getLSTMDevicePtr(), nullptr);
  EXPECT_NE(dynamics.network_d_, nullptr);
  EXPECT_EQ(dynamics.network_d_, dynamics.getHelper()->getLSTMDevicePtr());

  DYN::buffer_trajectory buffer;
  buffer["VEL_X"] = Eigen::VectorXf::Random(51);
  buffer["STEER_ANGLE"] = Eigen::VectorXf::Random(51);
  buffer["STEER_ANGLE_RATE"] = Eigen::VectorXf::Random(51);
  buffer["STEER_CMD"] = Eigen::VectorXf::Random(51);

  checkGPUComputationStep<RacerDubinsElevationLSTMSteering>(dynamics, dt, 16, 32, buffer, 1.0e-4);
}

// TODO assert they are different when a network is loaded, check params are the same

TEST_F(RacerDubinsElevationLSTMSteeringTest, TestStepGPUvsCPUReverse)
{
  using DYN = RacerDubinsElevationLSTMSteering;

  const int num_rollouts = 1024;
  const float dt = 0.1f;
  CudaCheckError();
  RacerDubinsElevationLSTMSteering dynamics = RacerDubinsElevationLSTMSteering(mppi::tests::steering_lstm);
  auto params = dynamics.getParams();
  params.gear_sign = -1;
  dynamics.setParams(params);
  EXPECT_FLOAT_EQ(dynamics.getParams().max_steer_rate, 17.590296);
  EXPECT_FLOAT_EQ(dynamics.getParams().steering_constant, 3.286375);
  EXPECT_FLOAT_EQ(dynamics.getParams().steer_accel_constant, 9.301527);
  EXPECT_FLOAT_EQ(dynamics.getParams().steer_accel_drag_constant, -0.60327667);

  cudaExtent extent = make_cudaExtent(10, 20, 0);
  TwoDTextureHelper<float>* helper = dynamics.getTextureHelper();
  helper->setExtent(0, extent);

  std::vector<float> data_vec;
  data_vec.resize(10 * 20);
  for (int i = 0; i < data_vec.size(); i++)
  {
    data_vec[i] = i * 1.0f;
  }

  std::array<float3, 3> new_rot_mat{};
  new_rot_mat[0] = make_float3(0, 1, 0);
  new_rot_mat[1] = make_float3(1, 0, 0);
  new_rot_mat[2] = make_float3(0, 0, 1);
  helper->updateRotation(0, new_rot_mat);
  helper->updateOrigin(0, make_float3(1, 2, 3));

  helper->updateTexture(0, data_vec);
  helper->updateResolution(0, 10);
  helper->enableTexture(0);
  helper->copyToDevice(true);

  CudaCheckError();
  dynamics.GPUSetup();
  CudaCheckError();

  EXPECT_NE(dynamics.getHelper()->getLSTMDevicePtr(), nullptr);
  EXPECT_NE(dynamics.network_d_, nullptr);
  EXPECT_EQ(dynamics.network_d_, dynamics.getHelper()->getLSTMDevicePtr());

  DYN::buffer_trajectory buffer;
  buffer["VEL_X"] = Eigen::VectorXf::Random(51);
  buffer["STEER_ANGLE"] = Eigen::VectorXf::Random(51);
  buffer["STEER_ANGLE_RATE"] = Eigen::VectorXf::Random(51);
  buffer["STEER_CMD"] = Eigen::VectorXf::Random(51);

  checkGPUComputationStep<RacerDubinsElevationLSTMSteering>(dynamics, dt, 16, 32, buffer, 1.0e-4);
}

TEST_F(RacerDubinsElevationLSTMSteeringTest, compareToElevationWithoutSteering)
{
  // by default the network will output zeros and not effect any states
  using DYN = RacerDubinsElevationLSTMSteering;
  using DYN_PARAMS = DYN::DYN_PARAMS_T;

  const int num_rollouts = 1024;
  const float dt = 0.1f;
  CudaCheckError();
  auto dynamics = RacerDubinsElevationLSTMSteering(3, 20, init_output_layers, 4, 4, output_layers, 11);
  RacerDubinsElevation dynamics2 = RacerDubinsElevation();
  auto params = dynamics.getParams();
  dynamics.setParams(params);
  dynamics2.setParams(params);

  cudaExtent extent = make_cudaExtent(10, 20, 0);
  TwoDTextureHelper<float>* helper = dynamics.getTextureHelper();
  helper->setExtent(0, extent);

  std::vector<float> data_vec;
  data_vec.resize(10 * 20);
  for (int i = 0; i < data_vec.size(); i++)
  {
    data_vec[i] = i * 1.0f;
  }

  std::array<float3, 3> new_rot_mat{};
  new_rot_mat[0] = make_float3(0, 1, 0);
  new_rot_mat[1] = make_float3(1, 0, 0);
  new_rot_mat[2] = make_float3(0, 0, 1);
  helper->updateRotation(0, new_rot_mat);
  helper->updateOrigin(0, make_float3(1, 2, 3));

  helper->updateTexture(0, data_vec);
  helper->updateResolution(0, 10);
  helper->enableTexture(0);
  helper->copyToDevice(true);

  TwoDTextureHelper<float>* helper2 = dynamics2.getTextureHelper();
  helper2->setExtent(0, extent);

  helper2->updateRotation(0, new_rot_mat);
  helper2->updateOrigin(0, make_float3(1, 2, 3));

  data_vec.resize(10 * 20);
  for (int i = 0; i < data_vec.size(); i++)
  {
    data_vec[i] = i * 1.0f;
  }
  helper2->updateTexture(0, data_vec);
  helper2->updateResolution(0, 10);
  helper2->enableTexture(0);
  helper2->copyToDevice(true);

  CudaCheckError();
  dynamics.GPUSetup();
  dynamics2.GPUSetup();
  CudaCheckError();

  Eigen::Matrix<float, RacerDubinsElevationLSTMSteering::CONTROL_DIM, num_rollouts> control_trajectory;
  control_trajectory = Eigen::Matrix<float, RacerDubinsElevationLSTMSteering::CONTROL_DIM, num_rollouts>::Random();
  Eigen::Matrix<float, RacerDubinsElevationLSTMSteering::STATE_DIM, num_rollouts> state_trajectory;
  state_trajectory = Eigen::Matrix<float, RacerDubinsElevationLSTMSteering::STATE_DIM, num_rollouts>::Random();

  RacerDubinsElevationLSTMSteering::state_array state;
  RacerDubinsElevationLSTMSteering::state_array next_state_cpu;
  RacerDubinsElevationLSTMSteering::control_array control;
  RacerDubinsElevationLSTMSteering::output_array output;
  RacerDubinsElevationLSTMSteering::state_array state_der_cpu = RacerDubinsElevationLSTMSteering::state_array::Zero();

  RacerDubinsElevationLSTMSteering::state_array state2;
  RacerDubinsElevationLSTMSteering::state_array next_state_cpu2;
  RacerDubinsElevationLSTMSteering::control_array control2;
  RacerDubinsElevationLSTMSteering::output_array output2;
  RacerDubinsElevationLSTMSteering::state_array state_der_cpu2 = RacerDubinsElevationLSTMSteering::state_array::Zero();

  DYN::buffer_trajectory buffer;
  buffer["VEL_X"] = Eigen::VectorXf::Random(51);
  buffer["STEER_ANGLE"] = Eigen::VectorXf::Random(51);
  buffer["STEER_ANGLE_RATE"] = Eigen::VectorXf::Random(51);
  buffer["STEER_CMD"] = Eigen::VectorXf::Random(51);

  dynamics.updateFromBuffer(buffer);
  for (int point = 0; point < num_rollouts; point++)
  {
    dynamics.initializeDynamics(state, control, output, 0, 0);
    state = state_trajectory.col(point);
    control = control_trajectory.col(point);
    state_der_cpu = RacerDubinsElevationLSTMSteering::state_array::Zero();

    dynamics2.initializeDynamics(state2, control2, output2, 0, 0);
    state2 = state_trajectory.col(point);
    control2 = control_trajectory.col(point);
    state_der_cpu2 = RacerDubinsElevationLSTMSteering::state_array::Zero();

    dynamics.step(state, next_state_cpu, state_der_cpu, control, output, 0, dt);
    dynamics2.step(state2, next_state_cpu2, state_der_cpu2, control2, output2, 0, dt);

    for (int dim = 0; dim < RacerDubinsElevationLSTMSteering::STATE_DIM; dim++)
    {
      if (dim == S_IND_CLASS(DYN_PARAMS, STEER_ANGLE) or dim == S_IND_CLASS(DYN_PARAMS, STEER_ANGLE_RATE))
      {  // this is done since the steering wheel setup is different, accel version
        continue;
      }
      EXPECT_NEAR(state_der_cpu(dim), state_der_cpu2(dim), 1e-4) << "state der at index " << point << " dim " << dim;
      EXPECT_NEAR(next_state_cpu(dim), next_state_cpu2(dim), 1e-4) << "next state at index " << point << " dim " << dim;
    }
    for (int dim = 0; dim < RacerDubinsElevationLSTMSteering::OUTPUT_DIM; dim++)
    {
      if (dim == O_IND_CLASS(DYN_PARAMS, STEER_ANGLE) or dim == O_IND_CLASS(DYN_PARAMS, STEER_ANGLE_RATE) or
          dim == O_IND_CLASS(DYN_PARAMS, WHEEL_FORCE_UP_MAX) or dim == O_IND_CLASS(DYN_PARAMS, WHEEL_FORCE_FWD_MAX) or
          dim == O_IND_CLASS(DYN_PARAMS, WHEEL_FORCE_SIDE_MAX) or dim == O_IND_CLASS(DYN_PARAMS, FILLER_1))
      {  // this is done since the steering wheel setup is different, accel version
        continue;
      }
      EXPECT_NEAR(output(dim), output2(dim), 1e-4) << "output at index " << point << " dim " << dim;
    }
  }

  params.gear_sign = -1;
  dynamics.setParams(params);
  dynamics2.setParams(params);

  // check in reverse as well
  for (int point = 0; point < num_rollouts; point++)
  {
    dynamics.initializeDynamics(state, control, output, 0, 0);
    state = state_trajectory.col(point);
    control = control_trajectory.col(point);
    state_der_cpu = RacerDubinsElevationLSTMSteering::state_array::Zero();

    dynamics2.initializeDynamics(state2, control2, output2, 0, 0);
    state2 = state_trajectory.col(point);
    control2 = control_trajectory.col(point);
    state_der_cpu2 = RacerDubinsElevationLSTMSteering::state_array::Zero();

    dynamics.step(state, next_state_cpu, state_der_cpu, control, output, 0, dt);
    dynamics2.step(state2, next_state_cpu2, state_der_cpu2, control2, output2, 0, dt);

    for (int dim = 0; dim < RacerDubinsElevationLSTMSteering::STATE_DIM; dim++)
    {
      if (dim == S_IND_CLASS(DYN_PARAMS, STEER_ANGLE) or dim == S_IND_CLASS(DYN_PARAMS, STEER_ANGLE_RATE))
      {  // this is done since the steering wheel setup is different, accel version
        continue;
      }
      EXPECT_NEAR(state_der_cpu(dim), state_der_cpu2(dim), 1e-4) << "at index " << point << " dim " << dim;
      EXPECT_NEAR(next_state_cpu(dim), next_state_cpu2(dim), 1e-4) << "at index " << point << " dim " << dim;
    }
    for (int dim = 0; dim < RacerDubinsElevationLSTMSteering::OUTPUT_DIM; dim++)
    {
      if (dim == O_IND_CLASS(DYN_PARAMS, STEER_ANGLE) or dim == O_IND_CLASS(DYN_PARAMS, STEER_ANGLE_RATE) or
          dim == O_IND_CLASS(DYN_PARAMS, WHEEL_FORCE_UP_MAX) or dim == O_IND_CLASS(DYN_PARAMS, WHEEL_FORCE_FWD_MAX) or
          dim == O_IND_CLASS(DYN_PARAMS, WHEEL_FORCE_SIDE_MAX) or dim == O_IND_CLASS(DYN_PARAMS, FILLER_1))
      {  // this is done since the steering wheel setup is different, accel version
        continue;
      }
      EXPECT_NEAR(output(dim), output2(dim), 1e-4) << "at index " << point << " dim " << dim;
    }
  }
  dynamics.freeCudaMem();
}

/*
class LinearDummy : public RacerDubinsElevationLSTMSteering {
public:
  bool computeGrad(const Eigen::Ref<const state_array> & state,
                   const Eigen::Ref<const control_array>& control,
                   Eigen::Ref<dfdx> A,
                   Eigen::Ref<dfdu> B) {
    return false;
  };
};

TEST_F(RacerDubinsElevationLSTMSteeringTest, TestComputeGradComputation) {
  Eigen::Matrix<float, RacerDubinsElevationLSTMSteering::STATE_DIM, RacerDubinsElevationLSTMSteering::STATE_DIM +
RacerDubinsElevationLSTMSteering::CONTROL_DIM> numeric_jac; Eigen::Matrix<float,
RacerDubinsElevationLSTMSteering::STATE_DIM, RacerDubinsElevationLSTMSteering::STATE_DIM +
RacerDubinsElevationLSTMSteering::CONTROL_DIM> analytic_jac; RacerDubinsElevationLSTMSteering::state_array state; state
<< 1, 2, 3, 4; RacerDubinsElevationLSTMSteering::control_array control; control << 5;

  auto analytic_grad_model = RacerDubinsElevationLSTMSteering();

  RacerDubinsElevationLSTMSteering::dfdx A_analytic = RacerDubinsElevationLSTMSteering::dfdx::Zero();
  RacerDubinsElevationLSTMSteering::dfdu B_analytic = RacerDubinsElevationLSTMSteering::dfdu::Zero();

  analytic_grad_model.computeGrad(state, control, A_analytic, B_analytic);

  auto numerical_grad_model = LinearDummy();

  std::shared_ptr<ModelWrapperDDP<LinearDummy>> ddp_model =
std::make_shared<ModelWrapperDDP<LinearDummy>>(&numerical_grad_model);

  analytic_jac.leftCols<RacerDubinsElevationLSTMSteering::STATE_DIM>() = A_analytic;
  analytic_jac.rightCols<RacerDubinsElevationLSTMSteering::CONTROL_DIM>() = B_analytic;
  numeric_jac = ddp_model->df(state, control);

  ASSERT_LT((numeric_jac - analytic_jac).norm(), 1e-3) << "Numeric Jacobian\n" << numeric_jac << "\nAnalytic Jacobian\n"
<< analytic_jac;
}

*/
