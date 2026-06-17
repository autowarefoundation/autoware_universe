#pragma once

#include <mppi/sampling_distributions/diffusion/diffusion_model.hpp>
#include <mppi/sampling_distributions/gaussian/gaussian.cuh>

#include <random>
#include <string>

namespace mppi
{
namespace sampling_distributions
{

template <int C_DIM, int MAX_DISTRIBUTIONS_T = 2>
struct DiffusionParamsImpl : public GaussianParamsImpl<C_DIM, MAX_DISTRIBUTIONS_T>
{
  /** Fallback std_dev for MPPI likelihood term when alpha < 1. */
  float likelihood_std_dev = 0.35F;

  DiffusionParamsImpl(int num_rollouts = 1, int num_timesteps = 1, int num_distributions = 1)
  : GaussianParamsImpl<C_DIM, MAX_DISTRIBUTIONS_T>(num_rollouts, num_timesteps, num_distributions)
  {
  }
};

template <int C_DIM>
using DiffusionParams = DiffusionParamsImpl<C_DIM, 2>;

template <
  class CLASS_T, template <int> class PARAMS_TEMPLATE = DiffusionParams,
  class DYN_PARAMS_T = DynamicsParams>
class DiffusionDistributionImpl
: public GaussianDistributionImpl<CLASS_T, PARAMS_TEMPLATE, DYN_PARAMS_T>
{
public:
  using PARENT_CLASS = GaussianDistributionImpl<CLASS_T, PARAMS_TEMPLATE, DYN_PARAMS_T>;
  using SAMPLING_PARAMS_T = typename PARENT_CLASS::SAMPLING_PARAMS_T;
  using control_array = typename PARENT_CLASS::control_array;
  static const int CONTROL_DIM = PARENT_CLASS::CONTROL_DIM;

  DiffusionDistributionImpl(cudaStream_t stream = 0);
  DiffusionDistributionImpl(const SAMPLING_PARAMS_T & params, cudaStream_t stream = 0);

  __host__ std::string getSamplingDistributionName() const override { return "Diffusion"; }

  __host__ void loadWeights(const std::string & path, bool synchronize = true);

  __host__ void setWeightsPath(const std::string & path) { pending_weights_path_ = path; }

  /** Scene/obstacle context vector (length kDiffusionContextDim or weights_.context_dim). */
  __host__ void setObstacleContext(const float * context, int context_dim);

  __host__ void setObstacleContextFromMovingCars(
    float ego_x, float ego_y, float ego_yaw, float ego_vel, float boundary_left,
    float boundary_right, const std::vector<mppi::cost::MovingCarObstacle> & cars, float sim_time);

  __host__ void setObstacleContextFromParkedCars(
    float ego_x, float ego_y, float ego_yaw, float ego_vel, float boundary_left,
    float boundary_right, const std::vector<mppi::cost::ParkedCarObstacle> & cars);

  __host__ void generateSamples(
    const int & optimization_stride, const int & iteration_num, curandGenerator_t & gen,
    bool synchronize = true);

  __host__ __device__ float computeLikelihoodRatioCost(
    const float * __restrict__ u, float * __restrict__ theta_d, const int sample_index, const int t,
    const int distribution_idx, const float lambda = 1.0, const float alpha = 0.0);

  __host__ float computeLikelihoodRatioCost(
    const Eigen::Ref<const control_array> & u, const int sample_index, const int t,
    const int distribution_idx, const float lambda = 1.0, const float alpha = 0.0);

protected:
  diffusion::DiffusionModelWeights weights_;
  bool weights_loaded_ = false;
  std::string pending_weights_path_;
  std::vector<float> obstacle_context_;
  std::mt19937 rng_{std::random_device{}()};
};

template <class DYN_PARAMS_T>
class DiffusionDistribution : public DiffusionDistributionImpl<
                                DiffusionDistribution<DYN_PARAMS_T>, DiffusionParams, DYN_PARAMS_T>
{
public:
  using PARENT_CLASS =
    DiffusionDistributionImpl<DiffusionDistribution, DiffusionParams, DYN_PARAMS_T>;
  using SAMPLING_PARAMS_T = typename PARENT_CLASS::SAMPLING_PARAMS_T;

  DiffusionDistribution(cudaStream_t stream = 0) : PARENT_CLASS(stream) {}
  DiffusionDistribution(const SAMPLING_PARAMS_T & params, cudaStream_t stream = 0)
  : PARENT_CLASS(params, stream)
  {
  }
};

}  // namespace sampling_distributions
}  // namespace mppi

#if __CUDACC__
#include "diffusion_distribution.cu"
#endif
