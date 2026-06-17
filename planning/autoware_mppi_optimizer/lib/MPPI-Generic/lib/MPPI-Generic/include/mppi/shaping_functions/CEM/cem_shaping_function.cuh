#ifndef MPPIGENERIC_CEM_SHAPING_FUNCTION_CUH
#define MPPIGENERIC_CEM_SHAPING_FUNCTION_CUH

#include <mppi/core/mppi_common.cuh>
#include <mppi/utils/managed.cuh>
#include <mppi/shaping_functions/shaping_function.cuh>

struct CEMShapingFunctionParams
{
  float gamma;  // represents the elite sample percentage
};

template <class CLASS_T, class PARAMS_T, int NUM_ROLLOUTS, int BDIM_X>
class CEMShapingFunctionImpl : public ShapingFunctionImpl<CLASS_T, PARAMS_T, NUM_ROLLOUTS, BDIM_X>
{
public:
  typedef Eigen::Matrix<float, NUM_ROLLOUTS, 1> cost_traj;

  CEMShapingFunctionImpl()
  {
    this->normalizer_ = 1.0;
  }

  __host__ __device__ float computeWeight(float* traj_cost, float baseline, int global_idx);

  void computeWeights(cost_traj& trajectory_costs, float* trajectory_costs_d, cudaStream_t stream = nullptr);

  /*
   * TODO what should this be for CEM?
  void computeFreeEnergy(float& free_energy, float& free_energy_var,
                         float& free_energy_modified,
                         float* cost_rollouts_host, float baseline);
                         */
};

template <int NUM_ROLLOUTS, int BDIM_X>
class CEMShapingFunction : public CEMShapingFunctionImpl<CEMShapingFunction<NUM_ROLLOUTS, BDIM_X>,
                                                         CEMShapingFunctionParams, NUM_ROLLOUTS, BDIM_X>
{
public:
};

#if __CUDACC__
#include "cem_shaping_function.cu"
#endif

#endif  // MPPIGENERIC_CEM_SHAPING_FUNCTION_CUH
