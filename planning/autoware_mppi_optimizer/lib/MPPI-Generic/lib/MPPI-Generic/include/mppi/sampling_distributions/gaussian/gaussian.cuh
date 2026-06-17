#pragma once
/**
 * Created by Bogdan Vlahov on 3/24/2023
 **/

#include <mppi/sampling_distributions/sampling_distribution.cuh>

#include <vector>

namespace mppi
{
namespace sampling_distributions
{
__global__ void setGaussianControls(const float* __restrict__ mean_d, const float* __restrict__ std_dev_d,
                                    float* __restrict__ control_samples_d, const int control_dim,
                                    const int num_timesteps, const int num_rollouts, const int num_distributions,
                                    const int optimization_stride, const float std_dev_decay,
                                    const float pure_noise_percentage, const bool time_specific_std_dev = false);

// Set the default number of distributions to 2 since that is currently the most we would use
template <int C_DIM, int MAX_DISTRIBUTIONS_T = 2>
struct GaussianParamsImpl : public SamplingParams<C_DIM>
{
  static const int MAX_DISTRIBUTIONS = MAX_DISTRIBUTIONS_T;
  float std_dev[C_DIM * MAX_DISTRIBUTIONS] MPPI_ALIGN(sizeof(float4)) = { 0.0f };
  float control_cost_coeff[C_DIM] MPPI_ALIGN(sizeof(float4)) = { 0.0f };
  float pure_noise_trajectories_percentage = 0.01f;
  float std_dev_decay = 1.0f;
  // Kernel launching params
  dim3 rewrite_controls_block_dim = dim3(32, 16, 1);
  int sum_strides = 32;
  // Various flags
  bool time_specific_std_dev = false;

  GaussianParamsImpl(int num_rollouts = 1, int num_timesteps = 1, int num_distributions = 1)
    : SamplingParams<C_DIM>::SamplingParams(num_rollouts, num_timesteps, num_distributions)
  {
    for (int i = 0; i < this->CONTROL_DIM * MAX_DISTRIBUTIONS; i++)
    {
      std_dev[i] = 1.0f;
    }
  }

  void copyStdDevToDistribution(const int src_distribution_idx, const int dest_distribution_idx)
  {
    bool src_out_of_distribution = src_distribution_idx >= MAX_DISTRIBUTIONS;
    if (src_out_of_distribution || dest_distribution_idx >= MAX_DISTRIBUTIONS)
    {
      printf("%s Distribution %d is out of range. There are only %d total distributions\n",
             src_out_of_distribution ? "Src" : "Dest",
             src_out_of_distribution ? src_distribution_idx : dest_distribution_idx, MAX_DISTRIBUTIONS);
      return;
    }
    float* std_dev_src = std_dev[this->CONTROL_DIM * src_distribution_idx];
    float* std_dev_dest = std_dev[this->CONTROL_DIM * dest_distribution_idx];
    for (int i = 0; i < this->CONTROL_DIM; i++)
    {
      std_dev_dest[i] = std_dev_src[i];
    }
  }
};

template <int C_DIM>
using GaussianParams = GaussianParamsImpl<C_DIM, 2>;

template <int C_DIM, int MAX_TIMESTEPS = 1, int MAX_DISTRIBUTIONS_T = 2>
struct GaussianTimeVaryingStdDevParams : public GaussianParamsImpl<C_DIM, MAX_DISTRIBUTIONS_T>
{
  float std_dev[C_DIM * MAX_TIMESTEPS * MAX_DISTRIBUTIONS_T] = { 0.0f };
  GaussianTimeVaryingStdDevParams(int num_rollouts = 1, int num_timesteps = 1, int num_distributions = 1)
    : GaussianParamsImpl<C_DIM, MAX_DISTRIBUTIONS_T>::GaussianParamsImpl(num_rollouts, num_timesteps, num_distributions)
  {
    this->time_specific_std_dev = true;
    for (int i = 0; i < this->CONTROL_DIM * MAX_TIMESTEPS * this->MAX_DISTRIBUTIONS; i++)
    {
      std_dev[i] = 1.0f;
    }
  }

  void copyStdDevToDistribution(const int src_distribution_idx, const int dest_distribution_idx)
  {
    bool src_out_of_distribution = src_distribution_idx >= this->MAX_DISTRIBUTIONS;
    if (src_out_of_distribution || dest_distribution_idx >= this->MAX_DISTRIBUTIONS)
    {
      printf("%s Distribution %d is out of range. There are only %d total distributions\n",
             src_out_of_distribution ? "Src" : "Dest",
             src_out_of_distribution ? src_distribution_idx : dest_distribution_idx, this->MAX_DISTRIBUTIONS);
      return;
    }
    float* std_dev_src = std_dev[this->CONTROL_DIM * this->num_timesteps * src_distribution_idx];
    float* std_dev_dest = std_dev[this->CONTROL_DIM * this->num_timesteps * dest_distribution_idx];
    for (int i = 0; i < this->CONTROL_DIM * this->num_timesteps; i++)
    {
      std_dev_dest[i] = std_dev_src[i];
    }
  }
};

template <class CLASS_T, template <int> class PARAMS_TEMPLATE = GaussianParams, class DYN_PARAMS_T = DynamicsParams>
class GaussianDistributionImpl : public SamplingDistribution<CLASS_T, PARAMS_TEMPLATE, DYN_PARAMS_T>
{
public:
  using PARENT_CLASS = SamplingDistribution<CLASS_T, PARAMS_TEMPLATE, DYN_PARAMS_T>;
  using SAMPLING_PARAMS_T = typename PARENT_CLASS::SAMPLING_PARAMS_T;
  using control_array = typename PARENT_CLASS::control_array;
  static const int CONTROL_DIM = PARENT_CLASS::CONTROL_DIM;
  typedef Eigen::Matrix<float, CONTROL_DIM, CONTROL_DIM> TEST_TYPE;

  GaussianDistributionImpl(cudaStream_t stream = 0);
  GaussianDistributionImpl(const SAMPLING_PARAMS_T& params, cudaStream_t stream = 0);

  ~GaussianDistributionImpl()
  {
    freeCudaMem();
  }

  __host__ virtual std::string getSamplingDistributionName() const override
  {
    return "Gaussian";
  }

  __host__ void allocateCUDAMemoryHelper();

  __host__ __device__ float computeFeedbackCost(const float* __restrict__ u_fb, float* __restrict__ theta_d,
                                                const int t, const int distribution_idx, const float lambda = 1.0,
                                                const float alpha = 0.0);

  __host__ float computeFeedbackCost(const Eigen::Ref<const control_array>& u_fb, const int t,
                                     const int distribution_idx, const float lambda = 1.0, const float alpha = 0.0);

  __host__ __device__ float computeLikelihoodRatioCost(const float* __restrict__ u, float* __restrict__ theta_d,
                                                       const int sample_index, const int t, const int distribution_idx,
                                                       const float lambda = 1.0, const float alpha = 0.0);

  __host__ float computeLikelihoodRatioCost(const Eigen::Ref<const control_array>& u, const int sample_index,
                                            const int t, const int distribution_idx, const float lambda = 1.0,
                                            const float alpha = 0.0);

  __host__ void copyImportanceSamplerToDevice(const float* importance_sampler, const int& distribution_idx,
                                              bool synchronize = true);

  __host__ void freeCudaMem();

  __host__ void generateSamples(const int& optimization_stride, const int& iteration_num, curandGenerator_t& gen,
                                bool synchronize = true);

  __host__ void paramsToDevice(bool synchronize = true);

  __host__ void setHostOptimalControlSequence(float* optimal_control_trajectory, const int& distribution_idx,
                                              bool synchronize = true);

  __host__ void setNumDistributions(const int num_distributions, bool synchronize = false)
  {
    if (num_distributions > SAMPLING_PARAMS_T::MAX_DISTRIBUTIONS)
    {
      this->logger_->error("GaussianParams can't handle more than %d distributions but %d were requested\n",
                           SAMPLING_PARAMS_T::MAX_DISTRIBUTIONS, num_distributions);
      throw std::out_of_range("Can't set num distributions higher than allowed in params");
    }
    PARENT_CLASS::setNumDistributions(num_distributions, synchronize);
  }

  __host__ void updateDistributionParamsFromDevice(const float* trajectory_weights_d, float normalizer,
                                                   const int& distribution_i, bool synchronize = false) override;

protected:
  float* std_dev_d_ = nullptr;
  float* control_means_d_ = nullptr;
  std::vector<float> means_;
};

template <class DYN_PARAMS_T>
class GaussianDistribution
  : public GaussianDistributionImpl<GaussianDistribution<DYN_PARAMS_T>, GaussianParams, DYN_PARAMS_T>
{
public:
  using PARENT_CLASS = GaussianDistributionImpl<GaussianDistribution, GaussianParams, DYN_PARAMS_T>;
  using SAMPLING_PARAMS_T = typename PARENT_CLASS::SAMPLING_PARAMS_T;

  GaussianDistribution(cudaStream_t stream = 0) : PARENT_CLASS(stream)
  {
  }
  GaussianDistribution(const SAMPLING_PARAMS_T& params, cudaStream_t stream = 0) : PARENT_CLASS(params, stream)
  {
  }
};

template <int C_DIM, int MAX_DISTRIBUTIONS_T>
const int GaussianParamsImpl<C_DIM, MAX_DISTRIBUTIONS_T>::MAX_DISTRIBUTIONS;
}  // namespace sampling_distributions
}  // namespace mppi

#if __CUDACC__
#include "gaussian.cu"
#endif
