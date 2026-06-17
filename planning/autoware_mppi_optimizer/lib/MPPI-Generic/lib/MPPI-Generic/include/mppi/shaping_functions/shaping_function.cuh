#ifndef MPPIGENERIC_SHAPING_FUNCTION_CUH
#define MPPIGENERIC_SHAPING_FUNCTION_CUH

#include <mppi/core/mppi_common.cuh>
#include <mppi/utils/managed.cuh>

namespace mppi_common
{
template <class CLASS_T, int NUM_ROLLOUTS>
__global__ void weightKernel(float* trajectory_costs_d, float baseline, CLASS_T* shape_function);
}

struct ShapingFunctionParams
{
  float lambda_inv = 1.0;  // also known as gamma
};

template <class CLASS_T, class PARAMS_T, int NUM_ROLLOUTS, int BDIM_X>
class ShapingFunctionImpl : public Managed
{
public:
  typedef Eigen::Matrix<float, NUM_ROLLOUTS, 1> cost_traj;

  ShapingFunctionImpl()
  {
    paramsToDevice();
  }

  __host__ __device__ float computeWeight(float* traj_cost, float baseline, int global_idx);

  void computeWeights(cost_traj& trajectory_costs, float* trajectory_costs_d, cudaStream_t stream = nullptr);

  void launchWeightKernel(float* trajectory_costs_d, float baseline, cudaStream_t stream = nullptr);

  void computeFreeEnergy(float& free_energy, float& free_energy_var, float& free_energy_modified,
                         float* cost_rollouts_host, float baseline);

  PARAMS_T getParams()
  {
    return params_;
  }
  float getBaseline()
  {
    return baseline_;
  }
  float getNormalizer()
  {
    return normalizer_;
  }

  void setParams(const PARAMS_T& params)
  {
    this->params_ = params;
    paramsToDevice();
  }

  void GPUSetup();
  void freeCudaMem();
  void paramsToDevice();

  CLASS_T* shaping_function_d_ = nullptr;

protected:
  PARAMS_T params_;
  float baseline_;
  float normalizer_;
};

template <int NUM_ROLLOUTS, int BDIM_X>
class ShapingFunction
  : public ShapingFunctionImpl<ShapingFunction<NUM_ROLLOUTS, BDIM_X>, ShapingFunctionParams, NUM_ROLLOUTS, BDIM_X>
{
public:
};

#if __CUDACC__
#include "shaping_function.cu"
#endif

#endif  // MPPIGENERIC_SHAPING_FUNCTION_CUH
