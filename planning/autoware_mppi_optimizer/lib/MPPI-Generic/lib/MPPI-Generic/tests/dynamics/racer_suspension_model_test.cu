#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <mppi/dynamics/racer_suspension/racer_suspension.cuh>
#include <kernel_tests/dynamics/dynamics_generic_kernel_tests.cuh>
#include <mppi/core/mppi_common.cuh>
#include <mppi/ddp/ddp_model_wrapper.h>
#include <cuda_runtime.h>

template <class DYN_T, int NUM_ROLLOUTS, int BLOCKSIZE_X, int BLOCKSIZE_Y, int BLOCKSIZE_Z = 1>
__global__ void runGPUDynamics(DYN_T* dynamics, const int num_timesteps, float dt, const float* __restrict__ x_init_d,
                               const float* __restrict__ u_d, float* __restrict__ x_next_d,
                               float* __restrict__ output_d)
{
  extern __shared__ float entire_buffer[];
  float* x_shared = entire_buffer;
  float* x_dot_shared = &x_shared[mppi::math::nearest_multiple_4(BLOCKSIZE_X * BLOCKSIZE_Z * 2 * DYN_T::STATE_DIM)];
  float* u_shared = &x_dot_shared[mppi::math::nearest_multiple_4(BLOCKSIZE_X * BLOCKSIZE_Z * DYN_T::STATE_DIM)];
  float* y_shared = &u_shared[mppi::math::nearest_multiple_4(BLOCKSIZE_X * BLOCKSIZE_Z * DYN_T::CONTROL_DIM)];
  float* theta_s = &y_shared[mppi::math::nearest_multiple_4(BLOCKSIZE_X * BLOCKSIZE_Z * 2 * DYN_T::OUTPUT_DIM)];

  int global_idx = blockIdx.x * blockDim.x + threadIdx.x;
  int j = 0;

  float* x;
  float* x_next;
  float* x_temp;
  float* x_dot;
  float* u;
  float* y;

  x = &x_shared[(BLOCKSIZE_X * threadIdx.z + threadIdx.x + 0) * DYN_T::STATE_DIM];
  x_next = &x_shared[(BLOCKSIZE_X * threadIdx.z + threadIdx.x + 1) * DYN_T::STATE_DIM];
  x_dot = &x_dot_shared[(BLOCKSIZE_X * threadIdx.z + threadIdx.x) * DYN_T::STATE_DIM];
  u = &u_shared[(BLOCKSIZE_X * threadIdx.z + threadIdx.x) * DYN_T::CONTROL_DIM];
  y = &y_shared[(BLOCKSIZE_X * threadIdx.z + threadIdx.x) * DYN_T::OUTPUT_DIM];
  for (j = threadIdx.y; j < DYN_T::STATE_DIM; j += blockDim.y)
  {
    x[j] = x_init_d[j];
    x_dot[j] = 0;
  }
  for (j = threadIdx.y; j < DYN_T::OUTPUT_DIM; j += blockDim.y)
  {
    y[j] = 0;
  }
  __syncthreads();
  dynamics->initializeDynamics(x, u, y, theta_s, 0, dt);
  __syncthreads();
  for (int t = 0; t < num_timesteps; t++)
  {
    for (j = threadIdx.y; j < DYN_T::CONTROL_DIM; j += blockDim.y)
    {
      u[j] = u_d[global_idx * num_timesteps * DYN_T::CONTROL_DIM + t * DYN_T::CONTROL_DIM + j];
    }
    for (j = threadIdx.y; j < DYN_T::STATE_DIM; j += blockDim.y)
    {
      x_next_d[global_idx * num_timesteps * DYN_T::STATE_DIM + t * DYN_T::STATE_DIM + j] = x[j];
    }
    for (j = threadIdx.y; j < DYN_T::OUTPUT_DIM; j += blockDim.y)
    {
      output_d[global_idx * num_timesteps * DYN_T::OUTPUT_DIM + t * DYN_T::OUTPUT_DIM + j] = y[j];
    }
    __syncthreads();
    dynamics->enforceConstraints(x, u);
    __syncthreads();
    dynamics->step(x, x_next, x_dot, u, y, theta_s, t, dt);
    __syncthreads();
    x_temp = x;
    x = x_next;
    x_next = x_temp;
  }
}

class RacerSuspensionTest : public ::testing::Test
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
};

TEST_F(RacerSuspensionTest, Template)
{
  auto dynamics = RacerSuspension(stream);
  EXPECT_EQ(14, RacerSuspension::STATE_DIM);
  EXPECT_EQ(2, RacerSuspension::CONTROL_DIM);
  EXPECT_NE(dynamics.getTextureHelper(), nullptr);
}

TEST_F(RacerSuspensionTest, BindStream)
{
  auto dynamics = RacerSuspension(stream);

  EXPECT_EQ(dynamics.stream_, stream) << "Stream binding failure.";
  EXPECT_NE(dynamics.getTextureHelper(), nullptr);
  EXPECT_EQ(dynamics.getTextureHelper()->stream_, stream);
}

TEST_F(RacerSuspensionTest, OmegaJacobian)
{
  using DYN_PARAMS = RacerSuspensionParams;
  auto dynamics = RacerSuspension(stream);
  RacerSuspension::state_array x = RacerSuspension::state_array::Zero();
  x[S_IND_CLASS(DYN_PARAMS, ATTITUDE_QW)] = 1;
  x[S_IND_CLASS(DYN_PARAMS, OMEGA_B_X)] = 0.1;
  x[S_IND_CLASS(DYN_PARAMS, OMEGA_B_Y)] = -0.03;
  x[S_IND_CLASS(DYN_PARAMS, OMEGA_B_Z)] = 0.02;
  x[S_IND_CLASS(DYN_PARAMS, V_I_X)] = 2;

  RacerSuspension::state_array x_dot0 = RacerSuspension::state_array::Zero();
  RacerSuspension::state_array x_dot1 = RacerSuspension::state_array::Zero();
  RacerSuspension::control_array u = RacerSuspension::control_array::Zero();
  RacerSuspension::output_array output = RacerSuspension::output_array::Zero();
  Eigen::Matrix3f omega_jac;
  float delta = 0.001;
  dynamics.computeStateDeriv(x, u, x_dot0, output, &omega_jac);
  for (int i = 0; i < 3; i++)
  {
    x[S_IND_CLASS(DYN_PARAMS, OMEGA_B_X) + i] += delta;
    dynamics.computeStateDeriv(x, u, x_dot1, output);
    x[S_IND_CLASS(DYN_PARAMS, OMEGA_B_X) + i] -= delta;

    float abs_tol = 2;
    EXPECT_NEAR(omega_jac.col(i)[0],
                (x_dot1[S_IND_CLASS(DYN_PARAMS, OMEGA_B_X)] - x_dot0[S_IND_CLASS(DYN_PARAMS, OMEGA_B_X)]) / delta,
                abs_tol);
    EXPECT_NEAR(omega_jac.col(i)[1],
                (x_dot1[S_IND_CLASS(DYN_PARAMS, OMEGA_B_Y)] - x_dot0[S_IND_CLASS(DYN_PARAMS, OMEGA_B_Y)]) / delta,
                abs_tol);
    EXPECT_NEAR(omega_jac.col(i)[2],
                (x_dot1[S_IND_CLASS(DYN_PARAMS, OMEGA_B_Z)] - x_dot0[S_IND_CLASS(DYN_PARAMS, OMEGA_B_Z)]) / delta,
                abs_tol);
  }
}

TEST_F(RacerSuspensionTest, CPUvsGPU)
{
  GTEST_SKIP();
  const int NUM_PARALLEL_TESTS = 10;  // MUST BE EVEN
  const int NUM_TIMESTEPS = 8;
  const float dt = 0.02;
  typedef RacerSuspension DYN;
  typedef Eigen::Matrix<float, DYN::STATE_DIM, NUM_TIMESTEPS> state_traj;
  typedef Eigen::Matrix<float, DYN::CONTROL_DIM, NUM_TIMESTEPS> control_traj;
  typedef Eigen::Matrix<float, DYN::OUTPUT_DIM, NUM_TIMESTEPS> output_traj;
  typedef util::EigenAlignedVector<float, DYN::CONTROL_DIM, NUM_TIMESTEPS> control_trajectories;
  typedef util::EigenAlignedVector<float, DYN::STATE_DIM, NUM_TIMESTEPS> state_trajectories;
  typedef util::EigenAlignedVector<float, DYN::OUTPUT_DIM, NUM_TIMESTEPS> output_trajectories;
  using state_array = typename DYN::state_array;
  using control_array = typename DYN::control_array;
  using output_array = typename DYN::output_array;
  using DYN_PARAMS = RacerSuspensionParams;

  auto dynamics = RacerSuspension(stream);
  auto control_range = dynamics.getControlRanges();
  control_range[0] = make_float2(-1, 1);
  control_range[1] = make_float2(-1, 1);
  dynamics.setControlRanges(control_range);
  dynamics.GPUSetup();

  std::default_random_engine generator(15.0);
  std::normal_distribution<float> throttle_distribution(0.3, 0.3);
  std::normal_distribution<float> steering_distribution(0.0, 0.8);

  float* x_init_d;
  float* u_d;
  float* x_next_d;
  float* output_d;

  HANDLE_ERROR(cudaMalloc((void**)&x_init_d, sizeof(float) * DYN::STATE_DIM * NUM_PARALLEL_TESTS));
  HANDLE_ERROR(cudaMalloc((void**)&u_d, sizeof(float) * DYN::CONTROL_DIM * NUM_PARALLEL_TESTS * NUM_TIMESTEPS));
  HANDLE_ERROR(cudaMalloc((void**)&x_next_d, sizeof(float) * DYN::STATE_DIM * NUM_PARALLEL_TESTS * NUM_TIMESTEPS));
  HANDLE_ERROR(cudaMalloc((void**)&output_d, sizeof(float) * DYN::OUTPUT_DIM * NUM_PARALLEL_TESTS * NUM_TIMESTEPS));

  // Input variables
  DYN::state_array x_init = DYN::state_array::Zero();
  x_init[S_IND_CLASS(DYN_PARAMS, P_I_X)] = 10;
  x_init[S_IND_CLASS(DYN_PARAMS, P_I_Y)] = 20;
  x_init[S_IND_CLASS(DYN_PARAMS, P_I_Z)] = 30;
  x_init[S_IND_CLASS(DYN_PARAMS, ATTITUDE_QW)] = 1;
  HANDLE_ERROR(
      cudaMemcpyAsync(x_init_d, x_init.data(), sizeof(float) * DYN::STATE_DIM, cudaMemcpyHostToDevice, stream));

  control_trajectories u_noise;
  for (int s = 0; s < NUM_PARALLEL_TESTS; s++)
  {
    control_traj sample_u = control_traj::Zero();
    for (int t = 0; t < NUM_TIMESTEPS; t++)
    {
      sample_u(0, t) = throttle_distribution(generator);
      sample_u(1, t) = steering_distribution(generator);
    }
    u_noise.push_back(sample_u);
    HANDLE_ERROR(cudaMemcpyAsync(u_d + s * NUM_TIMESTEPS * DYN::CONTROL_DIM, u_noise[s].data(),
                                 sizeof(float) * NUM_TIMESTEPS * DYN::CONTROL_DIM, cudaMemcpyHostToDevice, stream));
  }

  // Output variables
  state_trajectories x_next_CPU(NUM_PARALLEL_TESTS);
  state_trajectories x_next_GPU(NUM_PARALLEL_TESTS);
  output_trajectories output_CPU(NUM_PARALLEL_TESTS);
  output_trajectories output_GPU(NUM_PARALLEL_TESTS);

  // CPU Test
  for (int s = 0; s < NUM_PARALLEL_TESTS; s++)
  {
    state_traj sample_state_traj;
    output_traj sample_output_traj;
    state_array xdot;
    state_array x = x_init;
    state_array x_next = x_init;
    output_array output = output_array::Zero();
    control_array u;
    for (int t = 0; t < NUM_TIMESTEPS; t++)
    {
      if (t == 0)
      {
        dynamics.initializeDynamics(x, u, output, t, dt);
      }
      u = u_noise[s].col(t);
      sample_state_traj.col(t) = x;
      sample_output_traj.col(t) = output;

      dynamics.enforceConstraints(x, u);
      dynamics.step(x, x_next, xdot, u, output, t, dt);
      x = x_next;
    }
    x_next_CPU[s] = sample_state_traj;
    output_CPU[s] = sample_output_traj;
  }

  // GPU Test
  const int BLOCKSIZE_X = 2;
  const int BLOCKSIZE_Y = 8;
  const int NUM_GRIDS_X = (NUM_PARALLEL_TESTS - 1) / BLOCKSIZE_X + 1;
  dim3 block_dim(BLOCKSIZE_X, BLOCKSIZE_Y, 1);
  dim3 grid_dim(NUM_GRIDS_X, 1, 1);
  // Ensure that there won't memory overwriting due to inproper indexing
  static_assert(NUM_PARALLEL_TESTS % BLOCKSIZE_X == 0);

  int num_shared = block_dim.x * block_dim.z;
  unsigned shared_mem = mppi::kernels::calcClassSharedMemSize(&dynamics, block_dim) +
                        sizeof(float) * (3 * mppi::math::nearest_multiple_4(num_shared * DYN::STATE_DIM) +
                                         mppi::math::nearest_multiple_4(num_shared * DYN::OUTPUT_DIM) +
                                         mppi::math::nearest_multiple_4(num_shared * DYN::CONTROL_DIM));
  runGPUDynamics<DYN, NUM_PARALLEL_TESTS, BLOCKSIZE_X, BLOCKSIZE_Y><<<grid_dim, block_dim, shared_mem, stream>>>(
      dynamics.model_d_, NUM_TIMESTEPS, dt, x_init_d, u_d, x_next_d, output_d);
  for (int s = 0; s < NUM_PARALLEL_TESTS; s++)
  {
    HANDLE_ERROR(cudaMemcpyAsync(x_next_GPU[s].data(), x_next_d + s * NUM_TIMESTEPS * DYN::STATE_DIM,
                                 sizeof(float) * NUM_TIMESTEPS * DYN::STATE_DIM, cudaMemcpyDeviceToHost, stream));
    HANDLE_ERROR(cudaMemcpyAsync(output_GPU[s].data(), output_d + s * NUM_TIMESTEPS * DYN::OUTPUT_DIM,
                                 sizeof(float) * NUM_TIMESTEPS * DYN::OUTPUT_DIM, cudaMemcpyDeviceToHost, stream));
  }
  HANDLE_ERROR(cudaStreamSynchronize(stream));
  for (int s = 0; s < NUM_PARALLEL_TESTS; s++)
  {
    for (int t = 0; t < NUM_TIMESTEPS; t++)
    {
      for (int d = 0; d < DYN::STATE_DIM; d++)
      {
        ASSERT_NEAR(x_next_CPU[s](d, t), x_next_GPU[s](d, t), 1.0)
            << "Sample " << s << ", t = " << t << ", state_dim: " << d << std::endl;
      }
      for (int d = 0; d < DYN::OUTPUT_DIM; d++)
      {
        ASSERT_TRUE(isfinite(output_CPU[s](d, t)))
            << "NaNs/inf at sample " << s << " t: " << t << ", dim: " << d << std::endl;
        ASSERT_NEAR(output_CPU[s](d, t), output_GPU[s](d, t), 1.0)
            << "Sample " << s << ", t = " << t << ", output_dim: " << d << std::endl;
      }
    }
  }
}

/*
float c_t = 1.3;
float c_b = 2.5;
float c_v = 3.7;
float c_0 = 4.9;
float wheel_base = 0.3;
 */

// TEST_F(RacerSuspensionTest, ComputeDynamics)
// {
//   RacerSuspension dynamics = RacerSuspension();
//   RacerDubinsParams params = dynamics.getParams();
//   RacerSuspension::state_array x = RacerSuspension::state_array::Zero();
//   RacerSuspension::control_array u = RacerSuspension::control_array::Zero();
//
//   // computeDynamics should not touch the roll/pitch element
//   RacerSuspension::state_array next_x = RacerSuspension::state_array::Ones() * 0.153;
//   dynamics.computeDynamics(x, u, next_x);
//   EXPECT_FLOAT_EQ(next_x(0), 4.9);
//   EXPECT_FLOAT_EQ(next_x(1), 0);
//   EXPECT_FLOAT_EQ(next_x(2), 0);
//   EXPECT_FLOAT_EQ(next_x(3), 0);
//   EXPECT_FLOAT_EQ(next_x(4), 0);
//   EXPECT_FLOAT_EQ(next_x(5), 0.153);
//   EXPECT_FLOAT_EQ(next_x(6), 0.153);
//
//   x << 1, M_PI_2, 0, 3, 0, 0.5, -0.5;
//   u << 1, 0;
//   dynamics.computeDynamics(x, u, next_x);
//   EXPECT_FLOAT_EQ(next_x(0), 4.9 + 1.3 - 3.7);
//   EXPECT_FLOAT_EQ(next_x(1), 0);
//   EXPECT_NEAR(next_x(2), 0, 1e-7);
//   EXPECT_FLOAT_EQ(next_x(3), 1);
//   EXPECT_FLOAT_EQ(next_x(4), 0);
//   EXPECT_FLOAT_EQ(next_x(5), 0.153);
//   EXPECT_FLOAT_EQ(next_x(6), 0.153);
//
//   x << 1, 0, 0, 3, 0, 0.5, -0.5;
//   u << -1, 0;
//   dynamics.computeDynamics(x, u, next_x);
//   EXPECT_FLOAT_EQ(next_x(0), 4.9 - 2.5 - 3.7);
//   EXPECT_FLOAT_EQ(next_x(1), 0);
//   EXPECT_FLOAT_EQ(next_x(2), 1);
//   EXPECT_FLOAT_EQ(next_x(3), 0);
//   EXPECT_FLOAT_EQ(next_x(4), 0);
//   EXPECT_FLOAT_EQ(next_x(5), 0.153);
//   EXPECT_FLOAT_EQ(next_x(6), 0.153);
//
//   x << -1, 0, 0, 3, 0, 0.5, -0.5;
//   u << 1, 0;
//   dynamics.computeDynamics(x, u, next_x);
//   EXPECT_FLOAT_EQ(next_x(0), 4.9 + 3.7 + 1.3);
//   EXPECT_FLOAT_EQ(next_x(1), 0);
//   EXPECT_FLOAT_EQ(next_x(2), -1);
//   EXPECT_FLOAT_EQ(next_x(3), 0);
//   EXPECT_FLOAT_EQ(next_x(4), 0);
//   EXPECT_FLOAT_EQ(next_x(5), 0.153);
//   EXPECT_FLOAT_EQ(next_x(6), 0.153);
//
//   x << -1, 0, 0, 3, 0, 0.5, -0.5;
//   u << -1, 0;
//   dynamics.computeDynamics(x, u, next_x);
//   EXPECT_FLOAT_EQ(next_x(0), 4.9 + 2.5 + 3.7);
//   EXPECT_FLOAT_EQ(next_x(1), 0);
//   EXPECT_FLOAT_EQ(next_x(2), -1);
//   EXPECT_FLOAT_EQ(next_x(3), 0);
//   EXPECT_FLOAT_EQ(next_x(4), 0);
//   EXPECT_FLOAT_EQ(next_x(5), 0.153);
//   EXPECT_FLOAT_EQ(next_x(6), 0.153);
//
//   x << -3, 0, 0, 3, 0, 0.5, -0.5;
//   u << -1, 0;
//   dynamics.computeDynamics(x, u, next_x);
//   EXPECT_FLOAT_EQ(next_x(0), 4.9 + 2.5 * 3 + 3.7 * 3);
//   EXPECT_FLOAT_EQ(next_x(1), 0);
//   EXPECT_FLOAT_EQ(next_x(2), -3);
//   EXPECT_FLOAT_EQ(next_x(3), 0);
//   EXPECT_FLOAT_EQ(next_x(4), 0);
//   EXPECT_FLOAT_EQ(next_x(5), 0.153);
//   EXPECT_FLOAT_EQ(next_x(6), 0.153);
//
//   x << 4, 0, 0, 3, 0, 0.5, -0.5;
//   u << -1, 0;
//   dynamics.computeDynamics(x, u, next_x);
//   EXPECT_FLOAT_EQ(next_x(0), 4.9 - 2.5 * 4 - 3.7 * 4);
//   EXPECT_FLOAT_EQ(next_x(1), 0);
//   EXPECT_FLOAT_EQ(next_x(2), 4);
//   EXPECT_FLOAT_EQ(next_x(3), 0);
//   EXPECT_FLOAT_EQ(next_x(4), 0);
//   EXPECT_FLOAT_EQ(next_x(5), 0.153);
//   EXPECT_FLOAT_EQ(next_x(6), 0.153);
//
//   x << 1, M_PI, 0, 3, 0, 0.5, -0.5;
//   u << 0, 1;
//   dynamics.computeDynamics(x, u, next_x);
//   EXPECT_FLOAT_EQ(next_x(0), 4.9 - 3.7);
//   EXPECT_FLOAT_EQ(next_x(1), (1 / .3) * tan(0));
//   EXPECT_FLOAT_EQ(next_x(2), -1);
//   EXPECT_NEAR(next_x(3), 0, 1e-7);
//   EXPECT_FLOAT_EQ(next_x(4), -1 / 2.45);
//   EXPECT_FLOAT_EQ(next_x(5), 0.153);
//   EXPECT_FLOAT_EQ(next_x(6), 0.153);
//
//   x << 1, M_PI, 0, 0, 0.5, 0.5, -0.5;
//   u << 1, -1;
//   dynamics.computeDynamics(x, u, next_x);
//   EXPECT_FLOAT_EQ(next_x(0), 4.9 - 3.7 + 1.3);
//   EXPECT_FLOAT_EQ(next_x(1), (1 / .3) * tan(0.5));
//   EXPECT_FLOAT_EQ(next_x(2), -1);
//   EXPECT_NEAR(next_x(3), 0, 1e-7);
//   EXPECT_FLOAT_EQ(next_x(4), 1 / 2.45);
//   EXPECT_FLOAT_EQ(next_x(5), 0.153);
//   EXPECT_FLOAT_EQ(next_x(6), 0.153);
// }
//
// TEST_F(RacerSuspensionTest, TestModelGPU)
// {
//   RacerSuspension dynamics = RacerSuspension();
//   dynamics.GPUSetup();
//   CudaCheckError();
//
//   Eigen::Matrix<float, RacerSuspension::CONTROL_DIM, 100> control_trajectory;
//   control_trajectory = Eigen::Matrix<float, RacerSuspension::CONTROL_DIM, 100>::Random();
//   Eigen::Matrix<float, RacerSuspension::STATE_DIM, 100> state_trajectory;
//   state_trajectory = Eigen::Matrix<float, RacerSuspension::STATE_DIM, 100>::Random();
//
//   std::vector<std::array<float, 7>> s(100);
//   std::vector<std::array<float, 7>> s_der(100);
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
//
//   // These variables will be changed so initialized to the right size only
//
//   // Run dynamics on dynamicsU
//   // Run dynamics on GPU
//   for (int y_dim = 1; y_dim <= 4; y_dim++)
//   {
//     launchComputeDynamicsTestKernel<RacerSuspension, 7, 2>(dynamics, s, u, s_der, y_dim);
//     for (int point = 0; point < 100; point++)
//     {
//       RacerSuspension::state_array state = state_trajectory.col(point);
//       RacerSuspension::control_array control = control_trajectory.col(point);
//       RacerSuspension::state_array state_der_cpu = RacerSuspension::state_array::Zero();
//
//       dynamics.computeDynamics(state, control, state_der_cpu);
//       for (int dim = 0; dim < RacerSuspension::STATE_DIM; dim++)
//       {
//         EXPECT_NEAR(state_der_cpu(dim), s_der[point][dim], 1e-5) << "at index " << point << " with y_dim " << y_dim;
//         EXPECT_TRUE(isfinite(s_der[point][dim]));
//       }
//     }
//   }
//
//   dynamics.freeCudaMem();
//   CudaCheckError();
// }
//
// TEST_F(RacerSuspensionTest, TestUpdateState)
// {
//   CudaCheckError();
//   RacerSuspension dynamics = RacerSuspension();
//   RacerSuspension::state_array state;
//   RacerSuspension::state_array state_der;
//
//   // TODO add in the elevation map
//
//   state << 0, 0, 0, 0, 0, -0.5, 0.5;
//   state_der << 1, 1, 1, 1, 1, 0, 0;
//   dynamics.updateState(state, state_der, 0.1);
//   EXPECT_TRUE(state_der == RacerSuspension::state_array::Zero());
//   EXPECT_FLOAT_EQ(state(0), 0.1);
//   EXPECT_FLOAT_EQ(state(1), 0.1);
//   EXPECT_FLOAT_EQ(state(2), 0.1);
//   EXPECT_FLOAT_EQ(state(3), 0.1);
//   EXPECT_FLOAT_EQ(state(4), 1.0 + (0 - 1.0) * expf(-0.6 * 0.1));
//   EXPECT_FLOAT_EQ(state(5), -0.5);
//   EXPECT_FLOAT_EQ(state(6), 0.5);
//
//   state << 0, M_PI - 0.1, 0, 0, 0, -0.5, 0.5;
//   state_der << 1, 1, 1, 1, 1;
//   dynamics.updateState(state, state_der, 1.0);
//   EXPECT_TRUE(state_der == RacerSuspension::state_array::Zero());
//   EXPECT_FLOAT_EQ(state(0), 1.0);
//   EXPECT_FLOAT_EQ(state(1), 1.0 - M_PI - 0.1);
//   EXPECT_FLOAT_EQ(state(2), 1.0);
//   EXPECT_FLOAT_EQ(state(3), 1.0);
//   EXPECT_FLOAT_EQ(state(4), 1.0 + (0 - 1.0) * expf(-0.6 * 1.0));
//   EXPECT_FLOAT_EQ(state(5), -0.5);
//   EXPECT_FLOAT_EQ(state(6), 0.5);
//
//   state << 0, -M_PI + 0.1, 0, 0, 0, -0.5, 0.5;
//   state_der << 1, -1, 1, 1, 1;
//   dynamics.updateState(state, state_der, 1.0);
//   EXPECT_TRUE(state_der == RacerSuspension::state_array::Zero());
//   EXPECT_FLOAT_EQ(state(0), 1.0);
//   EXPECT_FLOAT_EQ(state(1), M_PI + 0.1 - 1.0);
//   EXPECT_FLOAT_EQ(state(2), 1.0);
//   EXPECT_FLOAT_EQ(state(3), 1.0);
//   EXPECT_FLOAT_EQ(state(4), 1.0 + (0 - 1.0) * expf(-0.6 * 1.0));
//   EXPECT_FLOAT_EQ(state(5), -0.5);
//   EXPECT_FLOAT_EQ(state(6), 0.5);
//
//   CudaCheckError();
// }
//
// TEST_F(RacerSuspensionTest, TestUpdateStateGPU)
// {
//   CudaCheckError();
//   RacerSuspension dynamics = RacerSuspension();
//   CudaCheckError();
//   dynamics.GPUSetup();
//   CudaCheckError();
//
//   Eigen::Matrix<float, RacerSuspension::CONTROL_DIM, 100> control_trajectory;
//   control_trajectory = Eigen::Matrix<float, RacerSuspension::CONTROL_DIM, 100>::Random();
//   Eigen::Matrix<float, RacerSuspension::STATE_DIM, 100> state_trajectory;
//   state_trajectory = Eigen::Matrix<float, RacerSuspension::STATE_DIM, 100>::Random();
//
//   std::vector<std::array<float, 7>> s(100);
//   std::vector<std::array<float, 7>> s_der(100);
//   // steering, throttle
//   std::vector<std::array<float, 2>> u(100);
//
//   RacerSuspension::state_array state;
//   RacerSuspension::control_array control;
//   RacerSuspension::state_array state_der_cpu = RacerSuspension::state_array::Zero();
//
//   // Run dynamics on dynamicsU
//   // Run dynamics on GPU
//   for (int y_dim = 1; y_dim <= 10; y_dim++)
//   {
//     for (int state_index = 0; state_index < s.size(); state_index++)
//     {
//       for (int dim = 0; dim < s[0].size(); dim++)
//       {
//         s[state_index][dim] = state_trajectory.col(state_index)(dim);
//       }
//       for (int dim = 0; dim < u[0].size(); dim++)
//       {
//         u[state_index][dim] = control_trajectory.col(state_index)(dim);
//       }
//     }
//
//     launchComputeStateDerivTestKernel<RacerSuspension, RacerSuspension::STATE_DIM,
//                                       RacerSuspension::CONTROL_DIM>(dynamics, s, u, s_der, y_dim);
//     launchUpdateStateTestKernel<RacerSuspension, RacerSuspension::STATE_DIM>(dynamics, s, s_der, 0.1f, y_dim);
//     for (int point = 0; point < 100; point++)
//     {
//       RacerSuspension::state_array state = state_trajectory.col(point);
//       RacerSuspension::control_array control = control_trajectory.col(point);
//       RacerSuspension::state_array state_der_cpu = RacerSuspension::state_array::Zero();
//
//       dynamics.computeDynamics(state, control, state_der_cpu);
//       dynamics.updateState(state, state_der_cpu, 0.1f);
//       for (int dim = 0; dim < RacerSuspension::STATE_DIM; dim++)
//       {
//         if (dim < 5)
//         {
//           EXPECT_FLOAT_EQ(state_der_cpu(dim), s_der[point][dim]) << "at index " << point << " with y_dim " << y_dim;
//           EXPECT_NEAR(state(dim), s[point][dim], 1e-4) << "at index " << point << " with y_dim " << y_dim;
//           EXPECT_TRUE(isfinite(s[point][dim]));
//         }
//         else
//         {
//           EXPECT_FLOAT_EQ(state_der_cpu(dim), s_der[point][dim]) << "at index " << point << " with y_dim " << y_dim;
//           EXPECT_NEAR(s[point][dim], 0.0, 1e-4)
//               << "at index " << point << " with y_dim " << y_dim << " state index " << dim;
//           EXPECT_TRUE(isfinite(s[point][dim]));
//         }
//       }
//     }
//   }
//   dynamics.freeCudaMem();
// }
//
// TEST_F(RacerSuspensionTest, ComputeStateTrajectoryFiniteTest)
// {
//   RacerSuspension dynamics = RacerSuspension();
//   RacerDubinsParams params;
//   params.c_t = 3.0;
//   params.c_b = 0.2;
//   params.c_v = 0.2;
//   params.c_0 = 0.2;
//   params.wheel_base = 3.0;
//   params.steering_constant = 1.0;
//   dynamics.setParams(params);
//
//   Eigen::Matrix<float, RacerSuspension::CONTROL_DIM, 500> control_trajectory;
//   control_trajectory = Eigen::Matrix<float, RacerSuspension::CONTROL_DIM, 500>::Zero();
//   Eigen::Matrix<float, RacerSuspension::STATE_DIM, 500> state_trajectory;
//   state_trajectory = Eigen::Matrix<float, RacerSuspension::STATE_DIM, 500>::Zero();
//   RacerSuspension::state_array state_der;
//   RacerSuspension::state_array x;
//   x << 0, 1.46919e-6, 0.0140179, 1.09739e-8, -0.000735827;
//
//   for (int i = 0; i < 500; i++)
//   {
//     RacerSuspension::control_array u = control_trajectory.col(i);
//     dynamics.computeDynamics(x, u, state_der);
//     EXPECT_TRUE(x.allFinite());
//     EXPECT_TRUE(u.allFinite());
//     EXPECT_TRUE(state_der.allFinite());
//     dynamics.updateState(x, state_der, 0.02);
//     EXPECT_TRUE(x.allFinite());
//     EXPECT_TRUE(u.allFinite());
//     EXPECT_TRUE(state_der == RacerSuspension::state_array::Zero());
//   }
//   params.steering_constant = 0.5;
//   dynamics.setParams(params);
//
//   x << 0, 1.46919e-6, 0.0140179, 1.09739e-8, -1.0;
//   for (int i = 0; i < 500; i++)
//   {
//     RacerSuspension::control_array u = control_trajectory.col(i);
//     dynamics.computeDynamics(x, u, state_der);
//     EXPECT_TRUE(x.allFinite());
//     EXPECT_TRUE(u.allFinite());
//     EXPECT_TRUE(state_der.allFinite());
//     dynamics.updateState(x, state_der, 0.02);
//     EXPECT_TRUE(x.allFinite());
//     EXPECT_TRUE(u.allFinite());
//     EXPECT_TRUE(state_der == RacerSuspension::state_array::Zero());
//   }
// }
//
// /*
// class LinearDummy : public RacerSuspension {
// public:
//   bool computeGrad(const Eigen::Ref<const state_array> & state,
//                    const Eigen::Ref<const control_array>& control,
//                    Eigen::Ref<dfdx> A,
//                    Eigen::Ref<dfdu> B) {
//     return false;
//   };
// };
//
// TEST_F(RacerSuspensionTest, TestComputeGradComputation) {
//   Eigen::Matrix<float, RacerSuspension::STATE_DIM, RacerSuspension::STATE_DIM +
// RacerSuspension::CONTROL_DIM> numeric_jac; Eigen::Matrix<float, RacerSuspension::STATE_DIM,
// RacerSuspension::STATE_DIM + RacerSuspension::CONTROL_DIM> analytic_jac; RacerSuspension::state_array
// state; state << 1, 2, 3, 4; RacerSuspension::control_array control; control << 5;
//
//   auto analytic_grad_model = RacerSuspension();
//
//   RacerSuspension::dfdx A_analytic = RacerSuspension::dfdx::Zero();
//   RacerSuspension::dfdu B_analytic = RacerSuspension::dfdu::Zero();
//
//   analytic_grad_model.computeGrad(state, control, A_analytic, B_analytic);
//
//   auto numerical_grad_model = LinearDummy();
//
//   std::shared_ptr<ModelWrapperDDP<LinearDummy>> ddp_model =
// std::make_shared<ModelWrapperDDP<LinearDummy>>(&numerical_grad_model);
//
//   analytic_jac.leftCols<RacerSuspension::STATE_DIM>() = A_analytic;
//   analytic_jac.rightCols<RacerSuspension::CONTROL_DIM>() = B_analytic;
//   numeric_jac = ddp_model->df(state, control);
//
//   ASSERT_LT((numeric_jac - analytic_jac).norm(), 1e-3) << "Numeric Jacobian\n" << numeric_jac << "\nAnalytic
//   Jacobian\n"
// << analytic_jac;
// }
//
// */
