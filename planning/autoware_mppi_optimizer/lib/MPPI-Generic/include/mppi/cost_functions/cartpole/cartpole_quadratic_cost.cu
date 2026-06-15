#include <mppi/cost_functions/cartpole/cartpole_quadratic_cost.cuh>

CartpoleQuadraticCost::CartpoleQuadraticCost(cudaStream_t stream)
{
  bindToStream(stream);
}

float CartpoleQuadraticCost::computeStateCost(const Eigen::Ref<const output_array> s, int timestep, int* crash_status)
{
  return (s[0] - params_.desired_terminal_state[0]) * (s[0] - params_.desired_terminal_state[0]) *
             params_.cart_position_coeff +
         (s[1] - params_.desired_terminal_state[1]) * (s[1] - params_.desired_terminal_state[1]) *
             params_.cart_velocity_coeff +
         (s[2] - params_.desired_terminal_state[2]) * (s[2] - params_.desired_terminal_state[2]) *
             params_.pole_angle_coeff +
         (s[3] - params_.desired_terminal_state[3]) * (s[3] - params_.desired_terminal_state[3]) *
             params_.pole_angular_velocity_coeff;
}

__device__ float CartpoleQuadraticCost::computeStateCost(float* state, int timestep, float* theta_c, int* crash_status)
{
  return (state[0] - params_.desired_terminal_state[0]) * (state[0] - params_.desired_terminal_state[0]) *
             params_.cart_position_coeff +
         (state[1] - params_.desired_terminal_state[1]) * (state[1] - params_.desired_terminal_state[1]) *
             params_.cart_velocity_coeff +
         (state[2] - params_.desired_terminal_state[2]) * (state[2] - params_.desired_terminal_state[2]) *
             params_.pole_angle_coeff +
         (state[3] - params_.desired_terminal_state[3]) * (state[3] - params_.desired_terminal_state[3]) *
             params_.pole_angular_velocity_coeff;
}

__device__ float CartpoleQuadraticCost::terminalCost(float* state, float* theta_c)
{
  return ((state[0] - params_.desired_terminal_state[0]) * (state[0] - params_.desired_terminal_state[0]) *
              params_.cart_position_coeff +
          (state[1] - params_.desired_terminal_state[1]) * (state[1] - params_.desired_terminal_state[1]) *
              params_.cart_velocity_coeff +
          (state[2] - params_.desired_terminal_state[2]) * (state[2] - params_.desired_terminal_state[2]) *
              params_.pole_angle_coeff +
          (state[3] - params_.desired_terminal_state[3]) * (state[3] - params_.desired_terminal_state[3]) *
              params_.pole_angular_velocity_coeff) *
         params_.terminal_cost_coeff;
}
float CartpoleQuadraticCost::terminalCost(const Eigen::Ref<const output_array> state)
{
  return ((state[0] - params_.desired_terminal_state[0]) * (state[0] - params_.desired_terminal_state[0]) *
              params_.cart_position_coeff +
          (state[1] - params_.desired_terminal_state[1]) * (state[1] - params_.desired_terminal_state[1]) *
              params_.cart_velocity_coeff +
          (state[2] - params_.desired_terminal_state[2]) * (state[2] - params_.desired_terminal_state[2]) *
              params_.pole_angle_coeff +
          (state[3] - params_.desired_terminal_state[3]) * (state[3] - params_.desired_terminal_state[3]) *
              params_.pole_angular_velocity_coeff) *
         params_.terminal_cost_coeff;
}
