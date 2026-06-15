#include <mppi/cost_functions/double_integrator/double_integrator_robust_cost.cuh>
#include <mppi/utils/math_utils.h>

DoubleIntegratorRobustCost::DoubleIntegratorRobustCost(cudaStream_t stream)
{
  bindToStream(stream);
}

__device__ float DoubleIntegratorRobustCost::computeStateCost(float* s, int timestep, float* theta_c, int* crash_status)
{
  float radial_position = s[0] * s[0] + s[1] * s[1];
  float current_velocity = sqrtf(s[2] * s[2] + s[3] * s[3]);
  float current_angular_momentum = s[0] * s[3] - s[1] * s[2];

  float cost = 0;
  float normalized_dist_from_center = mppi::math::normDistFromCenter(
      sqrt(radial_position), sqrt(params_.inner_path_radius2), sqrt(params_.outer_path_radius2));
  float steep_percent_boundary = 0.5;
  float steep_cost = 0.5 * params_.crash_cost;  // 10 percent of crash cost
  // Shallow cost region
  if (normalized_dist_from_center <= steep_percent_boundary)
  {
    cost += mppi::math::linInterp(normalized_dist_from_center, 0, steep_percent_boundary, 0, steep_cost);
  }
  // Steep cost region
  if (normalized_dist_from_center > steep_percent_boundary && normalized_dist_from_center <= 1.0)
  {
    cost +=
        mppi::math::linInterp(normalized_dist_from_center, steep_percent_boundary, 1, steep_cost, params_.crash_cost);
  }
  // Crash Cost region
  if (normalized_dist_from_center > 1.0)
  {
    cost += params_.crash_cost;
  }
  cost += params_.velocity_cost * powf(current_velocity - params_.velocity_desired, 2);
  cost += params_.velocity_cost * powf(current_angular_momentum - params_.angular_momentum_desired, 2);
  return cost;
}

float DoubleIntegratorRobustCost::computeStateCost(const Eigen::Ref<const output_array> s, int timestep,
                                                   int* crash_status)
{
  float radial_position = s[0] * s[0] + s[1] * s[1];
  float current_velocity = sqrtf(s[2] * s[2] + s[3] * s[3]);
  float current_angular_momentum = s[0] * s[3] - s[1] * s[2];

  float cost = 0;
  float normalized_dist_from_center = mppi::math::normDistFromCenter(
      sqrt(radial_position), sqrt(params_.inner_path_radius2), sqrt(params_.outer_path_radius2));
  float steep_percent_boundary = 0.75;
  float steep_cost = 0.1 * params_.crash_cost;
  if (normalized_dist_from_center <= steep_percent_boundary)
  {
    cost += mppi::math::linInterp(normalized_dist_from_center, 0, steep_percent_boundary, 0, steep_cost);
  }
  else if (normalized_dist_from_center > steep_percent_boundary && normalized_dist_from_center <= 1.0)
  {
    cost +=
        mppi::math::linInterp(normalized_dist_from_center, steep_percent_boundary, 1, steep_cost, params_.crash_cost);
  }
  else
  {
    cost += params_.crash_cost;
  }
  cost += params_.velocity_cost * powf(current_velocity - params_.velocity_desired, 2);
  cost += params_.velocity_cost * powf(current_angular_momentum - params_.angular_momentum_desired, 2);
  return cost;
}

float DoubleIntegratorRobustCost::terminalCost(const Eigen::Ref<const output_array> s)
{
  return 0;
}

__device__ float DoubleIntegratorRobustCost::terminalCost(float* s, float* theta_c)
{
  return 0;
}
