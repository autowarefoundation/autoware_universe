#ifndef AR_ROBUST_COST_CUH_
#define AR_ROBUST_COST_CUH_

#include <mppi/cost_functions/autorally/ar_standard_cost.cuh>

struct ARRobustCostParams : public ARStandardCostParams
{
  // Miscellaneous
  float heading_coeff = 0.0;

  ARRobustCostParams()
  {
    control_cost_coeff[0] = 0.0;  // steering_coeff
    control_cost_coeff[1] = 0.0;  // throttle_coeff
    // values, if negative ignored
    desired_speed = -1;
    max_slip_ang = 1.5;

    // Cost term coefficients
    track_coeff = 33.0;
    slip_coeff = 0.0;
    speed_coeff = 20.0;
    // Constraint penalty thresholds
    crash_coeff = 125000;
    boundary_threshold = 0.75;
    // Miscellaneous
    track_slop = 0;
  }
};

template <class CLASS_T, class PARAMS_T = ARRobustCostParams>
class ARRobustCostImpl : public ARStandardCostImpl<CLASS_T, PARAMS_T>
{
public:
  using PARENT_CLASS = ARStandardCostImpl<CLASS_T, PARAMS_T>;
  using output_array = typename PARENT_CLASS::output_array;

  ARRobustCostImpl(cudaStream_t stream = 0);  // : ARStandardCost<PARAMS_T>(steam);
  ~ARRobustCostImpl();

  std::string getCostFunctionName()
  {
    return "AutoRally robust cost function";
  }

  __host__ __device__ float getStabilizingCost(const float* s);
  __host__ __device__ float getCostmapCost(const float* s);
  __host__ __device__ float computeStateCost(const float* s, int timestep, float* theta_c, int* crash_status);

  float computeStateCost(const Eigen::Ref<const output_array> y, int timestep, int* crash_status);

  using PARENT_CLASS::terminalCost;

private:
};

#if __CUDACC__
#include "ar_robust_cost.cu"
#endif

class ARRobustCost : public ARRobustCostImpl<ARRobustCost>
{
public:
  ARRobustCost(cudaStream_t stream = 0) : ARRobustCostImpl<ARRobustCost>(stream)
  {
  }
};

#endif  // AR_ROBUST_COST_CUH_
