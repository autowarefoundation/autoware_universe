
/**
 * Created by Bogdan Vlahov on Jan 7, 2024
 **/

#define NLN_TEMPLATE template <class CLASS_T, template <int> class PARAMS_TEMPLATE, class DYN_PARAMS_T>
#define NLN_NOISE NLNDistributionImpl<CLASS_T, PARAMS_TEMPLATE, DYN_PARAMS_T>

#include <mppi/sampling_distributions/nln/nln.cuh>
#include <mppi/utils/cuda_math_utils.cuh>
#include <mppi/utils/math_utils.h>

__global__ void createNLNNoise(float* __restrict__ normal_noise, const float* __restrict__ log_normal_noise,
                               const int num_trajectories, const int num_timesteps, const int control_dim)
{
  const int sample_index = blockIdx.x * blockDim.x + threadIdx.x;
  const int time_index = blockIdx.y * blockDim.y + threadIdx.y;
  const int control_index = blockIdx.z * blockDim.z + threadIdx.z;
  if (sample_index < num_trajectories && time_index < num_timesteps && control_index < control_dim)
  {
    const int normal_index = (sample_index * num_timesteps + time_index) * control_dim + control_index;
    const int log_normal_index = (control_index * num_trajectories + sample_index) * num_timesteps + time_index;
    normal_noise[normal_index] = normal_noise[normal_index] * log_normal_noise[log_normal_index];
  }
}

namespace mppi
{
namespace sampling_distributions
{
NLN_TEMPLATE
NLN_NOISE::NLNDistributionImpl(cudaStream_t stream) : PARENT_CLASS::GaussianDistributionImpl(stream)
{
  calculateLogMeanAndVariance();
}

NLN_TEMPLATE
NLN_NOISE::NLNDistributionImpl(const SAMPLING_PARAMS_T& params, cudaStream_t stream)
  : PARENT_CLASS::GaussianDistributionImpl(params, stream)
{
  calculateLogMeanAndVariance();
}

NLN_TEMPLATE
__host__ void NLN_NOISE::freeCudaMem()
{
  if (this->GPUMemStatus_)
  {
    cudaFree(log_normal_noise_d_);
  }
  PARENT_CLASS::freeCudaMem();
}

NLN_TEMPLATE
__host__ void NLN_NOISE::allocateCUDAMemoryHelper()
{
  PARENT_CLASS::allocateCUDAMemoryHelper();
  if (this->GPUMemStatus_)
  {
    if (log_normal_noise_d_)
    {
      HANDLE_ERROR(cudaFreeAsync(log_normal_noise_d_, this->stream_));
    }
    HANDLE_ERROR(cudaMallocAsync((void**)&log_normal_noise_d_,
                                 sizeof(float) * this->CONTROL_DIM * this->getNumRollouts() * this->getNumTimesteps() *
                                     this->getNumDistributions(),
                                 this->stream_));
  }
}

NLN_TEMPLATE
void NLN_NOISE::setParams(const SAMPLING_PARAMS_T& params, bool synchronize)
{
  bool adjusted_variance = false;
  for (int i = 0; i < this->CONTROL_DIM * this->getNumDistributions(); i++)
  {
    if (this->params_.std_dev[i] != params.std_dev[i])
    {
      adjusted_variance = true;
      break;
    }
  }
  PARENT_CLASS::setParams(params, synchronize);
  if (adjusted_variance)
  {
    calculateLogMeanAndVariance();
  }
}

NLN_TEMPLATE
void NLN_NOISE::calculateLogMeanAndVariance()
{
  float normal_variance, log_variance;
  log_noise_mean_.resize(this->CONTROL_DIM * this->getNumDistributions());
  log_noise_std_dev_.resize(this->CONTROL_DIM * this->getNumDistributions());
  for (int i = 0; i < this->CONTROL_DIM * this->getNumDistributions(); i++)
  {
    normal_variance = this->params_.std_dev[i] * this->params_.std_dev[i];
    log_noise_mean_[i] = expf(0.5 * normal_variance);
    float exp_normal_variance = expf(normal_variance);
    log_variance = exp_normal_variance * (exp_normal_variance - 1.0f);
    log_noise_std_dev_[i] = sqrtf(log_variance);
  }
}

NLN_TEMPLATE
__host__ void NLN_NOISE::generateSamples(const int& optimization_stride, const int& iteration_num,
                                         curandGenerator_t& gen, bool synchronize)
{
  // generate log normal noise
  for (int i = 0; i < this->CONTROL_DIM; i++)
  {
    HANDLE_CURAND_ERROR(
        curandGenerateLogNormal(gen, log_normal_noise_d_ + i * this->getNumRollouts() * this->getNumTimesteps(),
                                this->getNumRollouts() * this->getNumTimesteps(), 0.0f, this->params_.std_dev[i]));
  }
  // generate normal noise
  HANDLE_CURAND_ERROR(curandGenerateNormal(gen, this->control_samples_d_,
                                           this->getNumTimesteps() * this->getNumRollouts() * CONTROL_DIM, 0.0f, 1.0f));

  // create NLN noise by multiplying normal and log normal noise
  const int BLOCKSIZE_X = this->params_.rewrite_controls_block_dim.x;
  const int BLOCKSIZE_Y = this->params_.rewrite_controls_block_dim.y;
  const int BLOCKSIZE_Z = this->params_.rewrite_controls_block_dim.z;
  dim3 combine_noise_grid;
  combine_noise_grid.x = mppi::math::int_ceil(this->getNumRollouts(), BLOCKSIZE_X);
  combine_noise_grid.y = mppi::math::int_ceil(this->getNumTimesteps(), BLOCKSIZE_Y);
  combine_noise_grid.z = mppi::math::int_ceil(this->CONTROL_DIM, BLOCKSIZE_Z);
  createNLNNoise<<<combine_noise_grid, this->params_.rewrite_controls_block_dim, 0, this->stream_>>>(
      this->control_samples_d_, log_normal_noise_d_, this->getNumRollouts(), this->getNumTimesteps(),
      this->CONTROL_DIM);
  HANDLE_ERROR(cudaGetLastError());
  for (int i = 1; i < this->getNumDistributions(); i++)
  {
    HANDLE_ERROR(cudaMemcpyAsync(
        &this->control_samples_d_[this->getNumRollouts() * this->getNumTimesteps() * CONTROL_DIM * i],
        this->control_samples_d_, sizeof(float) * this->getNumRollouts() * this->getNumTimesteps() * CONTROL_DIM,
        cudaMemcpyDeviceToDevice, this->stream_));
  }
  /**
   * Generate noise samples with mean added
   **/
  dim3 control_writing_grid;
  control_writing_grid.x = mppi::math::int_ceil(this->getNumRollouts(), BLOCKSIZE_X);
  control_writing_grid.y = mppi::math::int_ceil(this->getNumTimesteps(), BLOCKSIZE_Y);
  control_writing_grid.z = mppi::math::int_ceil(this->getNumDistributions(), BLOCKSIZE_Z);
  unsigned int std_dev_mem_size = this->getNumDistributions() * CONTROL_DIM;
  // Allocate shared memory for std_deviations per timestep or constant across the trajectory
  std_dev_mem_size = mppi::math::nearest_multiple_4(
      this->params_.time_specific_std_dev ? std_dev_mem_size * this->getNumTimesteps() : std_dev_mem_size);
  unsigned int shared_mem_size =
      std_dev_mem_size +
      mppi::math::nearest_multiple_4(this->getNumDistributions() * this->getNumTimesteps() * CONTROL_DIM) +
      mppi::math::nearest_multiple_4(BLOCKSIZE_X * BLOCKSIZE_Y * BLOCKSIZE_Z * CONTROL_DIM);
  shared_mem_size *= sizeof(float);
  setGaussianControls<<<control_writing_grid, this->params_.rewrite_controls_block_dim, shared_mem_size,
                        this->stream_>>>(
      this->control_means_d_, this->std_dev_d_, this->control_samples_d_, CONTROL_DIM, this->getNumTimesteps(),
      this->getNumRollouts(), this->getNumDistributions(), optimization_stride,
      powf(this->params_.std_dev_decay, iteration_num), this->params_.pure_noise_trajectories_percentage,
      this->params_.time_specific_std_dev);

  HANDLE_ERROR(cudaGetLastError());
  if (synchronize)
  {
    HANDLE_ERROR(cudaStreamSynchronize(this->stream_));
  }
}
}  // namespace sampling_distributions
}  // namespace mppi
#undef NLN_NOISE
#undef NLN_TEMPLATE
