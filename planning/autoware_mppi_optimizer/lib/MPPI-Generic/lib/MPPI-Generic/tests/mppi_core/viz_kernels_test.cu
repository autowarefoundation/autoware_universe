#include <gtest/gtest.h>
#include <mppi/dynamics/cartpole/cartpole_dynamics.cuh>
#include <mppi/cost_functions/cartpole/cartpole_quadratic_cost.cuh>
#include <mppi/feedback_controllers/DDP/ddp.cuh>
#include <mppi/core/mppi_common.cuh>

#include <mppi/utils/test_helper.h>
#include <random>

class VizualizationKernelsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    CartpoleQuadraticCostParams new_params;
    new_params.cart_position_coeff = 100;
    new_params.pole_angle_coeff = 200;
    new_params.cart_velocity_coeff = 10;
    new_params.pole_angular_velocity_coeff = 20;
    new_params.control_cost_coeff[0] = 1;
    new_params.terminal_cost_coeff = 0;
    new_params.desired_terminal_state[0] = -20;
    new_params.desired_terminal_state[1] = 0;
    new_params.desired_terminal_state[2] = M_PI;
    new_params.desired_terminal_state[3] = 0;
    cost.setParams(new_params);

    cudaStreamCreate(&stream);

    /**
     * Ensure dynamics and costs exist on GPU
     */
    dynamics.bindToStream(stream);
    cost.bindToStream(stream);
    // Call the GPU setup functions of the model and cost
    dynamics.GPUSetup();
    cost.GPUSetup();
    fb_controller.GPUSetup();

    for (int i = 0; i < x0.rows(); i++)
    {
      x0(i) = i * 0.1 + 0.2;
    }

    for (int sample = 0; sample < num_rollouts; sample++)
    {
      control[sample] = control_trajectory::Random();
    }

    // Create x init cuda array
    HANDLE_ERROR(cudaMalloc((void**)&initial_state_d, sizeof(float) * CartpoleDynamics::STATE_DIM * 2));
    // create control u trajectory cuda array
    HANDLE_ERROR(
        cudaMalloc((void**)&control_d, sizeof(float) * CartpoleDynamics::CONTROL_DIM * MAX_TIMESTEPS * num_rollouts));
    // Create cost trajectory cuda array
    HANDLE_ERROR(cudaMalloc((void**)&trajectory_costs_d, sizeof(float) * num_rollouts * (MAX_TIMESTEPS + 1) * 2));
    // Create result state cuda array
    HANDLE_ERROR(cudaMalloc((void**)&result_state_d,
                            sizeof(float) * num_rollouts * MAX_TIMESTEPS * CartpoleDynamics::STATE_DIM * 2));
    // Create result state cuda array
    HANDLE_ERROR(cudaMalloc((void**)&crash_status_d, sizeof(float) * num_rollouts * MAX_TIMESTEPS * 2));
  }

  void TearDown() override
  {
    cudaFree(initial_state_d);
    cudaFree(control_d);
    cudaFree(trajectory_costs_d);
    cudaFree(crash_status_d);
    cudaFree(result_state_d);
  }

  const static int MAX_TIMESTEPS = 100;

  typedef Eigen::Matrix<float, CartpoleDynamics::STATE_DIM, MAX_TIMESTEPS> state_trajectory;      // A state trajectory
  typedef Eigen::Matrix<float, CartpoleDynamics::CONTROL_DIM, MAX_TIMESTEPS> control_trajectory;  // A control
                                                                                                  // trajectory
  typedef Eigen::Matrix<float, MAX_TIMESTEPS + 1, 1> cost_trajectory;
  typedef Eigen::Matrix<int, MAX_TIMESTEPS, 1> crash_status_trajectory;

  CartpoleDynamics dynamics = CartpoleDynamics(1, 1, 1);
  CartpoleQuadraticCost cost;
  DDPFeedback<CartpoleDynamics, MAX_TIMESTEPS> fb_controller =
      DDPFeedback<CartpoleDynamics, MAX_TIMESTEPS>(&dynamics, dt);

  float dt = 0.02;
  int num_rollouts = 20;
  float lambda = 0.5;
  float alpha = 0.001;

  cudaStream_t stream;

  float* initial_state_d;
  float* control_d;
  float* trajectory_costs_d;
  float* result_state_d;
  int* crash_status_d;

  CartpoleDynamics::state_array x0;

  std::vector<control_trajectory> control = std::vector<control_trajectory>(num_rollouts);
  std::vector<state_trajectory> result_state = std::vector<state_trajectory>(num_rollouts);
  std::vector<cost_trajectory> trajectory_costs = std::vector<cost_trajectory>(num_rollouts);
  std::vector<crash_status_trajectory> crash_status = std::vector<crash_status_trajectory>(num_rollouts);
};

// TEST_F(VizualizationKernelsTest, stateAndCostTrajectoryKernelNoZNoFeedbackTest)
// {
//   for (int tdy = 1; tdy < 8; tdy++)
//   {
//     /**
//      * Fill in GPU arrays
//      */
//     HANDLE_ERROR(cudaMemcpyAsync(initial_state_d, x0.data(), sizeof(float) * CartpoleDynamics::STATE_DIM,
//                                  cudaMemcpyHostToDevice, stream));
//     HANDLE_ERROR(cudaMemcpyAsync(initial_state_d + CartpoleDynamics::STATE_DIM, x0.data(),
//                                  sizeof(float) * CartpoleDynamics::STATE_DIM, cudaMemcpyHostToDevice, stream));
//     for (int i = 0; i < num_rollouts; i++)
//     {
//       HANDLE_ERROR(cudaMemcpyAsync(control_d + i * MAX_TIMESTEPS * CartpoleDynamics::CONTROL_DIM, control[i].data(),
//                                    MAX_TIMESTEPS * CartpoleDynamics::CONTROL_DIM * sizeof(float),
//                                    cudaMemcpyHostToDevice, stream));
//     }
//     HANDLE_ERROR(cudaStreamSynchronize(stream));

//     const int gridsize_x = (num_rollouts - 1) / 32 + 1;
//     dim3 dimBlock(32, tdy, 1);
//     dim3 dimGrid(gridsize_x, 1, 1);
//     mppi_common::stateAndCostTrajectoryKernel<CartpoleDynamics, CartpoleQuadraticCost,
//                                               DeviceDDP<CartpoleDynamics, MAX_TIMESTEPS>, 32, 1>
//         <<<dimGrid, dimBlock, 0, stream>>>(dynamics.model_d_, cost.cost_d_, fb_controller.getDevicePointer(),
//         control_d,
//                                            initial_state_d, result_state_d, trajectory_costs_d, crash_status_d,
//                                            num_rollouts, MAX_TIMESTEPS, dt, -1);

//     // Copy the results back to the host
//     for (int i = 0; i < num_rollouts; i++)
//     {
//       result_state[i].col(0) = x0;
//       // shifted by one since we do not save the initial state
//       HANDLE_ERROR(cudaMemcpyAsync(result_state[i].data() + (CartpoleDynamics::STATE_DIM),
//                                    result_state_d + i * MAX_TIMESTEPS * CartpoleDynamics::STATE_DIM,
//                                    (MAX_TIMESTEPS - 1) * CartpoleDynamics::STATE_DIM * sizeof(float),
//                                    cudaMemcpyDeviceToHost, stream));
//       HANDLE_ERROR(cudaMemcpyAsync(trajectory_costs[i].data(), trajectory_costs_d + i * (MAX_TIMESTEPS + 1),
//                                    (MAX_TIMESTEPS + 1) * sizeof(float), cudaMemcpyDeviceToHost, stream));
//       HANDLE_ERROR(cudaMemcpyAsync(crash_status[i].data(), crash_status_d + i * MAX_TIMESTEPS,
//                                    MAX_TIMESTEPS * sizeof(float), cudaMemcpyDeviceToHost, stream));
//     }
//     HANDLE_ERROR(cudaStreamSynchronize(stream));

//     for (int sample = 0; sample < num_rollouts; sample++)
//     {
//       CartpoleDynamics::state_array x = x0;
//       CartpoleDynamics::state_array x_dot;
//       control_trajectory u_traj = control[sample];
//       int crash_status_val = 0;

//       int t = 0;
//       for (; t < MAX_TIMESTEPS; t++)
//       {
//         EXPECT_NEAR(x(0), result_state[sample].col(t)(0), 1e-5)
//             << "\ntdy: " << tdy << "\nsample: " << sample << "\nat time: " << t;
//         EXPECT_NEAR(x(1), result_state[sample].col(t)(1), 1e-5)
//             << "\ntdy: " << tdy << "\nsample: " << sample << "\nat time: " << t;
//         EXPECT_NEAR(x(2), result_state[sample].col(t)(2), 1e-5)
//             << "\ntdy: " << tdy << "\nsample: " << sample << "\nat time: " << t;
//         EXPECT_NEAR(x(3), result_state[sample].col(t)(3), 1e-5)
//             << "\ntdy: " << tdy << "\nsample: " << sample << "\nat time: " << t;

//         CartpoleDynamics::control_array u = u_traj.col(t);
//         float cost_val = cost.computeStateCost(x, t, &crash_status_val);
//         if (t == 0)
//         {
//           // don't weight the first state
//           EXPECT_FLOAT_EQ(trajectory_costs[sample](t), 0);
//         }
//         else
//         {
//           EXPECT_FLOAT_EQ(cost_val, trajectory_costs[sample](t)) << "\nsample: " << sample << "\nat time: " << t;
//         }
//         EXPECT_EQ(crash_status_val, crash_status[sample](t)) << "\nsample: " << sample << "\nat time: " << t;

//         dynamics.enforceConstraints(x, u);
//         dynamics.computeStateDeriv(x, u, x_dot);
//         dynamics.updateState(x, x_dot, dt);
//       }
//       float terminal_cost = cost.terminalCost(x);
//       EXPECT_FLOAT_EQ(terminal_cost, trajectory_costs[sample](t)) << "\nsample: " << sample << "\nat terminal";
//     }
//   }
// }
