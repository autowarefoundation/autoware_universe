#pragma once
#ifndef DOUBLE_INTEGRATOR_CIRCLE_COST_CUH_
#define DOUBLE_INTEGRATOR_CIRCLE_COST_CUH_

#include <mppi/cost_functions/cost.cuh>
#include <mppi/dynamics/double_integrator/di_dynamics.cuh>

struct DoubleIntegratorCircleCostParams : public CostParams<2>
{
  float velocity_cost = 1;
  float crash_cost = 1000;
  float velocity_desired = 2;
  float inner_path_radius2 = 1.875 * 1.875;
  float outer_path_radius2 = 2.125 * 2.125;
  float angular_momentum_desired = 2 * velocity_desired;  // Enforces the system travels counter clockwise

  DoubleIntegratorCircleCostParams()
  {
    control_cost_coeff[0] = 0.01;
    control_cost_coeff[1] = 0.01;
    discount = 1.0;
  }
};

class DoubleIntegratorCircleCost
  : public Cost<DoubleIntegratorCircleCost, DoubleIntegratorCircleCostParams, DoubleIntegratorParams>
{
public:
  DoubleIntegratorCircleCost(cudaStream_t stream = nullptr);

  float computeStateCost(const Eigen::Ref<const output_array> s, int timestep = 0, int* crash_status = nullptr);
  float terminalCost(const Eigen::Ref<const output_array> s);

  __device__ float computeStateCost(float* s, int timestep = 0, float* theta_c = nullptr, int* crash_status = nullptr);
  __device__ float terminalCost(float* s, float* theta_c);
};

#if __CUDACC__
#include "double_integrator_circle_cost.cu"
#endif

#endif  //! DOUBLE_INTEGRATOR_CIRCLE_COST_CUH_
