#ifndef DOUBLE_INTEGRATOR_ROBUST_COST_CUH_
#define DOUBLE_INTEGRATOR_ROBUST_COST_CUH_

#include <mppi/cost_functions/double_integrator/double_integrator_circle_cost.cuh>

class DoubleIntegratorRobustCost
  : public Cost<DoubleIntegratorRobustCost, DoubleIntegratorCircleCostParams, DoubleIntegratorParams>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  DoubleIntegratorRobustCost(cudaStream_t stream = nullptr);

  float computeStateCost(const Eigen::Ref<const output_array> s, int timestep = 0, int* crash_status = nullptr);
  float terminalCost(const Eigen::Ref<const output_array> s);

  __device__ float computeStateCost(float* s, int timestep = 0, float* theta_c = nullptr, int* crash_status = nullptr);
  __device__ float terminalCost(float* s, float* theta_c);

  float getLipshitzConstantCost()
  {
    return params_.crash_cost;
  };
};

#if __CUDACC__
#include "double_integrator_robust_cost.cu"
#endif

#endif  //! DOUBLE_INTEGRATOR_ROBUST_COST_CUH_
