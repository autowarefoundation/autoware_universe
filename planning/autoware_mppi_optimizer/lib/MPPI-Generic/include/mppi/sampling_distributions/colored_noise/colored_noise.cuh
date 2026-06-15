#pragma once
/**
 * Created by Bogdan, Dec 16, 2021
 * based off of https://github.com/felixpatzelt/colorednoise/blob/master/colorednoise.py
 */

#include <mppi/sampling_distributions/gaussian/gaussian.cuh>
#include <mppi/utils/gpu_err_chk.cuh>

#include <Eigen/Dense>
#include <cufft.h>
#include <curand.h>

#include <algorithm>
#include <iostream>
#include <vector>

__global__ void configureFrequencyNoise(cufftComplex* noise, float* variance, int num_samples, int control_dim,
                                        int num_freq);

__global__ void rearrangeNoise(float* input, float* output, float* variance, int num_trajectories, int num_timesteps,
                               int control_dim, int offset_t, float decay_rate = 0.0);

void fftfreq(const int num_samples, std::vector<float>& result, const float spacing = 1)
{
  // result is of size floor(n/2) + 1
  int result_size = num_samples / 2 + 1;
  result.clear();
  result.resize(result_size);
  for (int i = 0; i < result_size; i++)
  {
    result[i] = i / (spacing * num_samples);
  }
}

void powerlaw_psd_gaussian(std::vector<float>& exponents, int num_timesteps, int num_trajectories,
                           float* control_noise_d, int offset_t, curandGenerator_t& gen, float offset_decay_rate,
                           cudaStream_t stream = 0, float fmin = 0.0);

namespace mppi
{
namespace sampling_distributions
{
template <int C_DIM, int MAX_DISTRIBUTIONS = 2>
struct ColoredNoiseParamsImpl : public GaussianParamsImpl<C_DIM, MAX_DISTRIBUTIONS>
{
  float exponents[C_DIM * MAX_DISTRIBUTIONS] = { 0.0f };
  float offset_decay_rate = 0.97;
  float fmin = 0.0;

  ColoredNoiseParamsImpl(int num_rollouts = 1, int num_timesteps = 1, int num_distributions = 1)
    : GaussianParamsImpl<C_DIM, MAX_DISTRIBUTIONS>(num_rollouts, num_timesteps, num_distributions)
  {
  }

  void copyExponentToDistribution(const int src_distribution_idx, const int dest_distribution_idx)
  {
    bool src_out_of_distribution = src_distribution_idx >= MAX_DISTRIBUTIONS;
    if (src_out_of_distribution || dest_distribution_idx >= MAX_DISTRIBUTIONS)
    {
      printf("%s Distribution %d is out of range. There are only %d total distributions\n",
             src_out_of_distribution ? "Src" : "Dest",
             src_out_of_distribution ? src_distribution_idx : dest_distribution_idx, MAX_DISTRIBUTIONS);
      return;
    }
    float* exponents_src = exponents[C_DIM * src_distribution_idx];
    float* exponents_dest = exponents[C_DIM * dest_distribution_idx];
    for (int i = 0; i < C_DIM; i++)
    {
      exponents_dest[i] = exponents_src[i];
    }
  }
};

template <int C_DIM>
using ColoredNoiseParams = ColoredNoiseParamsImpl<C_DIM, 2>;

template <class CLASS_T, template <int> class PARAMS_TEMPLATE = ColoredNoiseParams, class DYN_PARAMS_T = DynamicsParams>
class ColoredNoiseDistributionImpl : public GaussianDistributionImpl<CLASS_T, PARAMS_TEMPLATE, DYN_PARAMS_T>
{
public:
  using PARENT_CLASS = GaussianDistributionImpl<CLASS_T, PARAMS_TEMPLATE, DYN_PARAMS_T>;
  using SAMPLING_PARAMS_T = typename PARENT_CLASS::SAMPLING_PARAMS_T;
  using control_array = typename PARENT_CLASS::control_array;

  static const int CONTROL_DIM = PARENT_CLASS::CONTROL_DIM;

  ColoredNoiseDistributionImpl(cudaStream_t stream = 0);
  ColoredNoiseDistributionImpl(const SAMPLING_PARAMS_T& params, cudaStream_t stream = 0);

  ~ColoredNoiseDistributionImpl()
  {
    freeCudaMem();
  }

  __host__ virtual std::string getSamplingDistributionName() const override
  {
    return "Colored Noise";
  }

  __host__ void allocateCUDAMemoryHelper();

  __host__ void freeCudaMem();

  __host__ void generateSamples(const int& optimization_stride, const int& iteration_num, curandGenerator_t& gen,
                                bool synchronize = true);

  __host__ __device__ float getOffsetDecayRate() const
  {
    return this->params_.offset_decay_rate;
  }

  void setOffsetDecayRate(const float decay_rate)
  {
    this->params_.offset_decay_rate = decay_rate;
  }

protected:
  cufftHandle plan_;
  float* frequency_sigma_d_ = nullptr;
  float* noise_in_time_d_ = nullptr;
  cufftComplex* samples_in_freq_complex_d_ = nullptr;
  float* freq_coeffs_d_ = nullptr;
};

template <class DYN_PARAMS_T>
class ColoredNoiseDistribution
  : public ColoredNoiseDistributionImpl<ColoredNoiseDistribution<DYN_PARAMS_T>, ColoredNoiseParams, DYN_PARAMS_T>
{
public:
  using PARENT_CLASS = ColoredNoiseDistributionImpl<ColoredNoiseDistribution, ColoredNoiseParams, DYN_PARAMS_T>;
  using SAMPLING_PARAMS_T = typename PARENT_CLASS::SAMPLING_PARAMS_T;

  ColoredNoiseDistribution(cudaStream_t stream = 0) : PARENT_CLASS(stream)
  {
  }
  ColoredNoiseDistribution(const SAMPLING_PARAMS_T& params, cudaStream_t stream = 0) : PARENT_CLASS(params, stream)
  {
  }
};

}  // namespace sampling_distributions
}  // namespace mppi

#ifdef __CUDACC__
#include "colored_noise.cu"
#endif
