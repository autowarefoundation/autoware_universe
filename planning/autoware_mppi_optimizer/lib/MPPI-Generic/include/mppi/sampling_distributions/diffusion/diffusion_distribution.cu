#include <mppi/sampling_distributions/diffusion/diffusion_distribution.cuh>

#include <mppi/core/mppi_common.cuh>

#include <cstring>

namespace mppi
{
namespace sampling_distributions
{
#define DIFFUSION_TEMPLATE template <class CLASS_T, template <int> class PARAMS_TEMPLATE, class DYN_PARAMS_T>
#define DIFFUSION_CLASS DiffusionDistributionImpl<CLASS_T, PARAMS_TEMPLATE, DYN_PARAMS_T>

DIFFUSION_TEMPLATE
DIFFUSION_CLASS::DiffusionDistributionImpl(cudaStream_t stream) : PARENT_CLASS(stream)
{
}

DIFFUSION_TEMPLATE
DIFFUSION_CLASS::DiffusionDistributionImpl(const SAMPLING_PARAMS_T& params, cudaStream_t stream)
  : PARENT_CLASS(params, stream)
{
}

DIFFUSION_TEMPLATE
__host__ void DIFFUSION_CLASS::loadWeights(const std::string& path, bool synchronize)
{
  weights_ = diffusion::loadDiffusionModelWeights(path);
  weights_loaded_ = true;
  constexpr int kControlDim = static_cast<int>(DYN_PARAMS_T::ControlIndex::NUM_CONTROLS);
  if (weights_.horizon != this->getNumTimesteps())
  {
    this->logger_->error("Diffusion weights horizon %d != sampler num_timesteps %d\n", weights_.horizon,
                         this->getNumTimesteps());
    throw std::runtime_error("diffusion horizon mismatch");
  }
  if (weights_.control_dim != kControlDim)
  {
    this->logger_->error("Diffusion weights control_dim %d != dynamics control_dim %d\n", weights_.control_dim,
                         kControlDim);
    throw std::runtime_error("diffusion control_dim mismatch");
  }
  (void)synchronize;
}

DIFFUSION_TEMPLATE
__host__ void DIFFUSION_CLASS::setObstacleContext(const float* context, const int context_dim)
{
  obstacle_context_.assign(static_cast<size_t>(context_dim), 0.0F);
  if (context != nullptr && context_dim > 0)
  {
    std::memcpy(obstacle_context_.data(), context, sizeof(float) * static_cast<size_t>(context_dim));
  }
}

DIFFUSION_TEMPLATE
__host__ void DIFFUSION_CLASS::setObstacleContextFromMovingCars(
    const float ego_x, const float ego_y, const float ego_yaw, const float ego_vel, const float boundary_left,
    const float boundary_right, const std::vector<mppi::cost::MovingCarObstacle>& cars, const float sim_time)
{
  obstacle_context_.assign(static_cast<size_t>(diffusion::kDiffusionContextDim), 0.0F);
  diffusion::encodeDiffusionObstacleContextFromMovingCars(ego_x, ego_y, ego_yaw, ego_vel, boundary_left, boundary_right,
                                                          cars, sim_time, obstacle_context_.data());
}

DIFFUSION_TEMPLATE
__host__ void DIFFUSION_CLASS::setObstacleContextFromParkedCars(
    const float ego_x, const float ego_y, const float ego_yaw, const float ego_vel, const float boundary_left,
    const float boundary_right, const std::vector<mppi::cost::ParkedCarObstacle>& cars)
{
  obstacle_context_.assign(static_cast<size_t>(diffusion::kDiffusionContextDim), 0.0F);
  diffusion::encodeDiffusionObstacleContextFromParkedCars(ego_x, ego_y, ego_yaw, ego_vel, boundary_left, boundary_right,
                                                          cars, obstacle_context_.data());
}

DIFFUSION_TEMPLATE
__host__ void DIFFUSION_CLASS::generateSamples(const int& optimization_stride, const int& iteration_num,
                                               curandGenerator_t& gen, bool synchronize)
{
  (void)gen;
  (void)iteration_num;
  if (!weights_loaded_ && !pending_weights_path_.empty())
  {
    loadWeights(pending_weights_path_, false);
  }
  if (!weights_loaded_)
  {
    this->logger_->error("DiffusionDistribution: weights not loaded; call loadWeights() first.\n");
    PARENT_CLASS::generateSamples(optimization_stride, iteration_num, gen, synchronize);
    return;
  }

  if (weights_.context_dim > 0 &&
      static_cast<int>(obstacle_context_.size()) != weights_.context_dim)
  {
    this->logger_->error("Diffusion context size %zu != weights context_dim %d\n", obstacle_context_.size(),
                         weights_.context_dim);
    PARENT_CLASS::generateSamples(optimization_stride, iteration_num, gen, synchronize);
    return;
  }

  const float* context_ptr =
      weights_.context_dim > 0 && !obstacle_context_.empty() ? obstacle_context_.data() : nullptr;

  const int num_rollouts = this->getNumRollouts();
  const int num_timesteps = this->getNumTimesteps();
  const int num_distributions = this->getNumDistributions();
  const int traj_dim = weights_.traj_dim;

  const int control_dim = weights_.control_dim;
  std::vector<float> host_samples(static_cast<size_t>(num_distributions) * static_cast<size_t>(num_rollouts) *
                                  static_cast<size_t>(num_timesteps) * static_cast<size_t>(control_dim));
  std::vector<float> u_nom(static_cast<size_t>(traj_dim));
  std::vector<float> u_sample(static_cast<size_t>(traj_dim));

  for (int d = 0; d < num_distributions; ++d)
  {
    HANDLE_ERROR(cudaMemcpyAsync(u_nom.data(), &(this->control_means_d_[d * num_timesteps * control_dim]),
                                 sizeof(float) * static_cast<size_t>(traj_dim), cudaMemcpyDeviceToHost, this->stream_));
    HANDLE_ERROR(cudaStreamSynchronize(this->stream_));

    for (int r = 0; r < num_rollouts; ++r)
    {
      float* dst = &host_samples[(static_cast<size_t>(d) * static_cast<size_t>(num_rollouts) *
                                    static_cast<size_t>(num_timesteps) * static_cast<size_t>(control_dim)) +
                                   (static_cast<size_t>(r) * static_cast<size_t>(num_timesteps) *
                                    static_cast<size_t>(control_dim))];
      if (r == 0)
      {
        std::memcpy(dst, u_nom.data(), sizeof(float) * static_cast<size_t>(traj_dim));
        continue;
      }

      diffusion::diffusionSampleTrajectory(weights_, u_nom.data(), context_ptr, u_sample.data(), rng_);
      std::memcpy(dst, u_sample.data(), sizeof(float) * static_cast<size_t>(traj_dim));

      for (int t = 0; t < std::min(optimization_stride, num_timesteps); ++t)
      {
        std::memcpy(&dst[t * control_dim], &u_nom[static_cast<size_t>(t * control_dim)],
                    sizeof(float) * static_cast<size_t>(control_dim));
      }
    }
  }

  HANDLE_ERROR(cudaMemcpyAsync(this->control_samples_d_, host_samples.data(), host_samples.size() * sizeof(float),
                               cudaMemcpyHostToDevice, this->stream_));
  if (synchronize)
  {
    HANDLE_ERROR(cudaStreamSynchronize(this->stream_));
  }
}

DIFFUSION_TEMPLATE
__host__ __device__ float DIFFUSION_CLASS::computeLikelihoodRatioCost(const float* __restrict__ u,
                                                                      float* __restrict__ theta_d,
                                                                      const int sample_index, const int t,
                                                                      const int distribution_idx, const float lambda,
                                                                      const float alpha)
{
  (void)theta_d;
  if (alpha >= 1.0F)
  {
    return 0.0F;
  }
  const float std_dev = this->params_.likelihood_std_dev;
  const float inv_var = 1.0F / (std_dev * std_dev);
  const int distribution_i = distribution_idx >= this->params_.num_distributions ? 0 : distribution_idx;
  const float* mean = &(this->control_means_d_[(this->params_.num_timesteps * distribution_i + t) * CONTROL_DIM]);
  float cost = 0.0F;
  for (int i = 0; i < CONTROL_DIM; ++i)
  {
    const float m = mean[i];
    cost += this->params_.control_cost_coeff[i] * m * (m - 2.0F * u[i]) * inv_var;
  }
  return 0.5F * lambda * (1.0F - alpha) * cost;
}

DIFFUSION_TEMPLATE
__host__ float DIFFUSION_CLASS::computeLikelihoodRatioCost(const Eigen::Ref<const control_array>& u,
                                                           const int sample_index, const int t,
                                                           const int distribution_idx, const float lambda,
                                                           const float alpha)
{
  if (alpha >= 1.0F)
  {
    return 0.0F;
  }
  const float std_dev = this->params_.likelihood_std_dev;
  const float inv_var = 1.0F / (std_dev * std_dev);
  const int distribution_i = distribution_idx >= this->params_.num_distributions ? 0 : distribution_idx;
  const int mean_index = (distribution_i * this->getNumTimesteps() + t) * CONTROL_DIM;
  float cost = 0.0F;
  for (int i = 0; i < CONTROL_DIM; ++i)
  {
    const float m = this->means_[static_cast<size_t>(mean_index + i)];
    cost += this->params_.control_cost_coeff[i] * m * (m - 2.0F * u(i)) * inv_var;
  }
  return 0.5F * lambda * (1.0F - alpha) * cost;
}

#undef DIFFUSION_TEMPLATE
#undef DIFFUSION_CLASS
}  // namespace sampling_distributions
}  // namespace mppi
