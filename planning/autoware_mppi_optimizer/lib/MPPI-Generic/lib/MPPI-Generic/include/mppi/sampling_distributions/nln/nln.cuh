#pragma once
/**
 * Created by Bogdan, Jan 7, 2024
 * based off of https://github.com/IhabMohamed/log-MPPI_ros
 */

#include <mppi/sampling_distributions/gaussian/gaussian.cuh>

#include <vector>

__global__ void createNLNNoise(float* __restrict__ normal_noise, const float* __restrict__ log_normal_noise,
                               const int num_trajectories, const int num_timesteps, const int control_dim);

namespace mppi
{
namespace sampling_distributions
{
template <class CLASS_T, template <int> class PARAMS_TEMPLATE = GaussianParams, class DYN_PARAMS_T = DynamicsParams>
class NLNDistributionImpl : public GaussianDistributionImpl<CLASS_T, PARAMS_TEMPLATE, DYN_PARAMS_T>
{
public:
  using PARENT_CLASS = GaussianDistributionImpl<CLASS_T, PARAMS_TEMPLATE, DYN_PARAMS_T>;
  using SAMPLING_PARAMS_T = typename PARENT_CLASS::SAMPLING_PARAMS_T;
  using control_array = typename PARENT_CLASS::control_array;

  static const int CONTROL_DIM = PARENT_CLASS::CONTROL_DIM;

  NLNDistributionImpl(cudaStream_t stream = 0);
  NLNDistributionImpl(const SAMPLING_PARAMS_T& params, cudaStream_t stream = 0);

  ~NLNDistributionImpl()
  {
    freeCudaMem();
  }

  __host__ virtual std::string getSamplingDistributionName() const override
  {
    return "NLN";
  }

  void setParams(const SAMPLING_PARAMS_T& params, bool synchronize = true);

  void calculateLogMeanAndVariance();

  __host__ void allocateCUDAMemoryHelper();

  __host__ void freeCudaMem();

  __host__ void generateSamples(const int& optimization_stride, const int& iteration_num, curandGenerator_t& gen,
                                bool synchronize = true);

protected:
  float* log_normal_noise_d_ = nullptr;
  std::vector<float> log_noise_mean_;
  std::vector<float> log_noise_std_dev_;
};

template <class DYN_PARAMS_T>
class NLNDistribution : public NLNDistributionImpl<NLNDistribution<DYN_PARAMS_T>, GaussianParams, DYN_PARAMS_T>
{
public:
  using PARENT_CLASS = NLNDistributionImpl<NLNDistribution, GaussianParams, DYN_PARAMS_T>;
  using SAMPLING_PARAMS_T = typename PARENT_CLASS::SAMPLING_PARAMS_T;

  NLNDistribution(cudaStream_t stream = 0) : PARENT_CLASS(stream)
  {
  }
  NLNDistribution(const SAMPLING_PARAMS_T& params, cudaStream_t stream = 0) : PARENT_CLASS(params, stream)
  {
  }
};
}  // namespace sampling_distributions
}  // namespace mppi

#ifdef __CUDACC__
#include "nln.cu"
#endif
