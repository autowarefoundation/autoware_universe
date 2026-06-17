#include <mppi/cost_functions/double_integrator/double_integrator_circle_cost.cuh>

DoubleIntegratorCircleCost::DoubleIntegratorCircleCost(cudaStream_t stream)
{
  bindToStream(stream);
}

__device__ float DoubleIntegratorCircleCost::computeStateCost(float* s, int timestep, float* theta_c, int* crash_status)
{
  float radial_position = s[0] * s[0] + s[1] * s[1];
  float current_velocity = sqrtf(s[2] * s[2] + s[3] * s[3]);
  float current_angular_momentum = s[0] * s[3] - s[1] * s[2];

  float cost = 0;
  //  if ((radial_position < params_.inner_path_radius2) ||
  //      (radial_position > params_.outer_path_radius2)) {
  //    crash_status[0] = 1; // Indicates the system has crashed.
  //  }
  //
  //  if (crash_status[0] > 0) { // If we've crashed once, constantly add the crash cost.
  //    cost += powf(this->params_.discount, timestep)*params_.crash_cost;
  //  }

  if ((radial_position < params_.inner_path_radius2) || (radial_position > params_.outer_path_radius2))
  {
    cost += powf(this->params_.discount, timestep) * params_.crash_cost;
  }

  cost += params_.velocity_cost * abs(current_velocity - params_.velocity_desired);
  cost += params_.velocity_cost * abs(current_angular_momentum - params_.angular_momentum_desired);
  return cost;
}

float DoubleIntegratorCircleCost::computeStateCost(const Eigen::Ref<const output_array> s, int timestep,
                                                   int* crash_status)
{
  float radial_position = s[0] * s[0] + s[1] * s[1];
  float current_velocity = sqrtf(s[2] * s[2] + s[3] * s[3]);
  float current_angular_momentum = s[0] * s[3] - s[1] * s[2];
  float cost = 0;

  //  if ((radial_position < params_.inner_path_radius2) ||
  //      (radial_position > params_.outer_path_radius2)) {
  //    crash_status[0] = 1; // Indicates the system has crashed.
  //  }
  //
  //  if (crash_status[0] > 0) { // If we've crashed once, constantly add the crash cost.
  //    cost += powf(this->params_.discount, timestep)*params_.crash_cost;
  //  }

  if ((radial_position < params_.inner_path_radius2) || (radial_position > params_.outer_path_radius2))
  {
    cost += powf(this->params_.discount, timestep) * params_.crash_cost;
  }

  cost += params_.velocity_cost * abs(current_velocity - params_.velocity_desired);
  cost += params_.velocity_cost * abs(current_angular_momentum - params_.angular_momentum_desired);
  return cost;
}

float DoubleIntegratorCircleCost::terminalCost(const Eigen::Ref<const output_array> s)
{
  return 0;
}

__device__ float DoubleIntegratorCircleCost::terminalCost(float* state, float* theta_c)
{
  return 0;
}
