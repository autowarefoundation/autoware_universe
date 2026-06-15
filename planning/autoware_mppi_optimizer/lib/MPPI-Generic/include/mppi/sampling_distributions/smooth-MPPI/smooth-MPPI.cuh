#pragma once
/**
 * Created by Bogdan, Jan 8, 2024
 */

#include <mppi/sampling_distributions/gaussian/gaussian.cuh>

namespace mppi
{
namespace sampling_distributions
{
__global__ void integrateNoise(const float* __restrict__ action_deriv_d, const float* __restrict__ control_mean_d,
                               float* __restrict__ control_d, const int num_samples, const int num_timesteps,
                               const int control_dim, const float dt);
__global__ void shiftControlTrajectory(float* __restrict__ control_trajectory_d, const int num_distributions,
                                       const int num_timesteps, const int control_dim, const int shift_index);

template <int C_DIM, int MAX_DISTRIBUTIONS_T = 2>
struct SmoothMPPIParamsImpl : public GaussianParamsImpl<C_DIM, MAX_DISTRIBUTIONS_T>
{
  float dt = 0.015f;
  dim3 shift_trajectory_block;
  SmoothMPPIParamsImpl(int num_rollouts = 1, int num_timesteps = 1, int num_distributions = 1)
    : GaussianParamsImpl<C_DIM, MAX_DISTRIBUTIONS_T>(num_rollouts, num_timesteps, num_distributions)
  {
  }
};

template <int C_DIM>
using SmoothMPPIParams = SmoothMPPIParamsImpl<C_DIM, 2>;

template <class CLASS_T, template <int> class PARAMS_TEMPLATE = SmoothMPPIParams, class DYN_PARAMS_T = DynamicsParams>
class SmoothMPPIDistributionImpl : public GaussianDistributionImpl<CLASS_T, PARAMS_TEMPLATE, DYN_PARAMS_T>
{
public:
  using PARENT_CLASS = GaussianDistributionImpl<CLASS_T, PARAMS_TEMPLATE, DYN_PARAMS_T>;
  using SAMPLING_PARAMS_T = typename PARENT_CLASS::SAMPLING_PARAMS_T;
  using control_array = typename PARENT_CLASS::control_array;

  static const int CONTROL_DIM = PARENT_CLASS::CONTROL_DIM;

  SmoothMPPIDistributionImpl(cudaStream_t stream = 0);
  SmoothMPPIDistributionImpl(const SAMPLING_PARAMS_T& params, cudaStream_t stream = 0);

  ~SmoothMPPIDistributionImpl()
  {
    freeCudaMem();
  }

  __host__ virtual std::string getSamplingDistributionName() const override
  {
    return "Smooth-MPPI";
  }

  __host__ void allocateCUDAMemoryHelper();

  __host__ void freeCudaMem();

  __host__ void generateSamples(const int& optimization_stride, const int& iteration_num, curandGenerator_t& gen,
                                bool synchronize = true);

  // __host__ void copyImportanceSamplerToDevice(const float* importance_sampler,
  //                                             const int& distribution_idx, bool synchronize = true);
  // __host__ void setHostOptimalControlSequence(float* optimal_control_trajectory, const int& distribution_i,
  //                                             bool synchronize = true);

  __host__ void updateDistributionParamsFromDevice(const float* trajectory_weights_d, float normalizer,
                                                   const int& distribution_i, bool synchronize = false) override;

protected:
  float* deriv_action_mean_d_ = nullptr;
  float* deriv_action_noise_d_ = nullptr;
};

template <class DYN_PARAMS_T>
class SmoothMPPIDistribution
  : public SmoothMPPIDistributionImpl<SmoothMPPIDistribution<DYN_PARAMS_T>, SmoothMPPIParams, DYN_PARAMS_T>
{
public:
  using PARENT_CLASS = SmoothMPPIDistributionImpl<SmoothMPPIDistribution, SmoothMPPIParams, DYN_PARAMS_T>;
  using SAMPLING_PARAMS_T = typename PARENT_CLASS::SAMPLING_PARAMS_T;

  SmoothMPPIDistribution(cudaStream_t stream = 0) : PARENT_CLASS(stream)
  {
  }
  SmoothMPPIDistribution(const SAMPLING_PARAMS_T& params, cudaStream_t stream = 0) : PARENT_CLASS(params, stream)
  {
  }
};
}  // namespace sampling_distributions
}  // namespace mppi

#ifdef __CUDACC__
#include "smooth-MPPI.cu"
#endif
