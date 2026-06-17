/**
 * Created by Bogdan Vlahov on Jan 8, 2024
 **/

#define SMOOTH_MPPI_TEMPLATE template <class CLASS_T, template <int> class PARAMS_TEMPLATE, class DYN_PARAMS_T>
#define SMOOTH_MPPI_NOISE SmoothMPPIDistributionImpl<CLASS_T, PARAMS_TEMPLATE, DYN_PARAMS_T>

#include <mppi/sampling_distributions/smooth-MPPI/smooth-MPPI.cuh>
#include <mppi/utils/cuda_math_utils.cuh>
#include <mppi/utils/math_utils.h>

namespace mppi
{
namespace sampling_distributions
{
__global__ void integrateNoise(const float* __restrict__ action_deriv_d, const float* __restrict__ control_mean_d,
                               float* __restrict__ control_d, const int num_samples, const int num_timesteps,
                               const int control_dim, const float dt)
{
  const int sample_index = blockIdx.x * blockDim.x + threadIdx.x;
  const int control_index = blockIdx.y * blockDim.y + threadIdx.y;
  int noise_index = 0, mean_index = 0;
  if (sample_index < num_samples && control_index < control_dim)
  {
    for (int t = 0; t < num_timesteps; t++)
    {
      noise_index = (sample_index * num_timesteps + t) * control_dim + control_index;
      mean_index = t * control_dim + control_index;
      control_d[noise_index] = control_mean_d[mean_index] + action_deriv_d[noise_index] * dt;
    }
  }
}

__global__ void shiftControlTrajectory(float* __restrict__ control_trajectory_d, const int num_distributions,
                                       const int num_timesteps, const int control_dim, const int shift_index)
{
  extern __shared__ float shared_mean[];
  const int dist_index = blockIdx.x;
  const int control_index = threadIdx.x;

  mppi::p1::loadArrayParallel<mppi::p1::Parallel1Dir::THREAD_X>(
      shared_mean, 0, control_trajectory_d, dist_index * num_timesteps * control_dim, num_timesteps * control_dim);
  __syncthreads();
  int p_index, p_step;
  mppi::p1::getParallel1DIndex<mppi::p1::Parallel1Dir::THREAD_X>(p_index, p_step);
  for (int t = 0; t < num_timesteps; t++)
  {
    int write_index = t * control_dim;
    int read_index = min(t + shift_index, num_timesteps - 1) * control_dim;

    // if (control_dim % 4 == 0)
    // {
    //   for (int i = p_index; i < control_dim / 4; i += 4 * p_step)
    //   {
    //     reinterpret_cast<float4*>(&shared_mean[write_index])[i] =
    //         reinterpret_cast<float4*>(&shared_mean[read_index])[i];
    //   }
    // }
    // else if (control_dim % 2 == 0)
    // {
    //   for (int i = p_index; i < control_dim / 2; i += 2 * p_step)
    //   {
    //     reinterpret_cast<float2*>(&shared_mean[write_index])[i] =
    //         reinterpret_cast<float2*>(&shared_mean[read_index])[i];
    //   }
    // }
    // else
    // {
    for (int i = 0; i < control_dim; i += 1)
    {
      shared_mean[write_index + i] = shared_mean[read_index + i];
    }
    // }
  }
  __syncthreads();
  mppi::p1::loadArrayParallel<mppi::p1::Parallel1Dir::THREAD_X>(
      control_trajectory_d, dist_index * num_timesteps * control_dim, shared_mean, 0, num_timesteps * control_dim);
}

SMOOTH_MPPI_TEMPLATE
SMOOTH_MPPI_NOISE::SmoothMPPIDistributionImpl(cudaStream_t stream) : PARENT_CLASS::GaussianDistributionImpl(stream)
{
}

SMOOTH_MPPI_TEMPLATE
SMOOTH_MPPI_NOISE::SmoothMPPIDistributionImpl(const SAMPLING_PARAMS_T& params, cudaStream_t stream)
  : PARENT_CLASS::GaussianDistributionImpl(params, stream)
{
}

SMOOTH_MPPI_TEMPLATE
__host__ void SMOOTH_MPPI_NOISE::freeCudaMem()
{
  if (this->GPUMemStatus_)
  {
    HANDLE_ERROR(cudaFree(deriv_action_mean_d_));
    HANDLE_ERROR(cudaFree(deriv_action_noise_d_));
  }
  PARENT_CLASS::freeCudaMem();
}

SMOOTH_MPPI_TEMPLATE
__host__ void SMOOTH_MPPI_NOISE::allocateCUDAMemoryHelper()
{
  PARENT_CLASS::allocateCUDAMemoryHelper();
  if (this->GPUMemStatus_)
  {
    if (deriv_action_noise_d_)
    {
      HANDLE_ERROR(cudaFreeAsync(deriv_action_noise_d_, this->stream_));
    }
    HANDLE_ERROR(cudaMallocAsync((void**)&deriv_action_noise_d_,
                                 sizeof(float) * this->CONTROL_DIM * this->getNumRollouts() *
                                     this->getNumDistributions() * this->getNumTimesteps(),
                                 this->stream_));
    if (deriv_action_mean_d_)
    {
      HANDLE_ERROR(cudaFreeAsync(deriv_action_mean_d_, this->stream_));
    }
    HANDLE_ERROR(cudaMallocAsync(
        (void**)&deriv_action_mean_d_,
        sizeof(float) * this->CONTROL_DIM * this->getNumTimesteps() * this->getNumDistributions(), this->stream_));
  }
}

SMOOTH_MPPI_TEMPLATE
__host__ void SMOOTH_MPPI_NOISE::generateSamples(const int& optimization_stride, const int& iteration_num,
                                                 curandGenerator_t& gen, bool synchronize)
{
  if (this->params_.use_same_noise_for_all_distributions)
  {
    HANDLE_CURAND_ERROR(curandGenerateNormal(
        gen, this->deriv_action_noise_d_, this->getNumTimesteps() * this->getNumRollouts() * CONTROL_DIM, 0.0f, 1.0f));
    for (int i = 1; i < this->getNumDistributions(); i++)
    {
      HANDLE_ERROR(cudaMemcpyAsync(
          &this->deriv_action_noise_d_[this->getNumRollouts() * this->getNumTimesteps() * CONTROL_DIM * i],
          this->deriv_action_noise_d_, sizeof(float) * this->getNumRollouts() * this->getNumTimesteps() * CONTROL_DIM,
          cudaMemcpyDeviceToDevice, this->stream_));
    }
  }
  else
  {
    HANDLE_CURAND_ERROR(curandGenerateNormal(
        gen, this->deriv_action_noise_d_,
        this->getNumTimesteps() * this->getNumRollouts() * this->getNumDistributions() * CONTROL_DIM, 0.0f, 1.0f));
  }
  // Shift derivative action sequence
  dim3 control_block = dim3(this->CONTROL_DIM, 1, 1);
  dim3 control_grid = dim3(this->getNumDistributions(), 1, 1);
  unsigned int shared_mean_size = sizeof(float) * this->CONTROL_DIM * this->getNumTimesteps();
  shiftControlTrajectory<<<control_grid, control_block, shared_mean_size, this->stream_>>>(
      deriv_action_mean_d_, this->getNumDistributions(), this->getNumTimesteps(), this->CONTROL_DIM,
      optimization_stride);

  const int BLOCKSIZE_X = this->params_.rewrite_controls_block_dim.x;
  const int BLOCKSIZE_Y = this->params_.rewrite_controls_block_dim.y;
  const int BLOCKSIZE_Z = this->params_.rewrite_controls_block_dim.z;
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
      this->deriv_action_mean_d_, this->std_dev_d_, this->deriv_action_noise_d_, CONTROL_DIM, this->getNumTimesteps(),
      this->getNumRollouts(), this->getNumDistributions(), optimization_stride,
      powf(this->params_.std_dev_decay, iteration_num), this->params_.pure_noise_trajectories_percentage,
      this->params_.time_specific_std_dev);

  // integrate derivative controls to make true controls
  dim3 integrate_noise_grid;
  integrate_noise_grid.x = mppi::math::int_ceil(this->getNumRollouts() * this->getNumDistributions(), BLOCKSIZE_X);
  integrate_noise_grid.y = 1;
  integrate_noise_grid.z = 1;
  dim3 integrate_noise_block(BLOCKSIZE_X, this->CONTROL_DIM, 1);
  // control_writing_grid.y = mppi::math::int_ceil(this->getNumTimesteps(), BLOCKSIZE_Y);
  // control_writing_grid.z = mppi::math::int_ceil(this->getNumDistributions(), BLOCKSIZE_Z);
  // std::cout << "Integrating samples " << this->getNumRollouts() * this->getNumDistributions() << std::endl;
  integrateNoise<<<control_writing_grid, integrate_noise_block, 0, this->stream_>>>(
      this->deriv_action_noise_d_, this->control_means_d_, this->control_samples_d_,
      this->getNumRollouts() * this->getNumDistributions(), this->getNumTimesteps(), this->CONTROL_DIM,
      this->params_.dt);

  HANDLE_ERROR(cudaGetLastError());
  if (synchronize)
  {
    HANDLE_ERROR(cudaStreamSynchronize(this->stream_));
  }
}

SMOOTH_MPPI_TEMPLATE
__host__ void SMOOTH_MPPI_NOISE::updateDistributionParamsFromDevice(const float* trajectory_weights_d, float normalizer,
                                                                    const int& distribution_i, bool synchronize)
{
  if (distribution_i >= this->getNumDistributions())
  {
    std::cerr << "Updating distributional params for distribution " << distribution_i << " out of "
              << this->getNumDistributions() << " total." << std::endl;
    return;
  }
  int mean_index = distribution_i * this->getNumTimesteps() * this->CONTROL_DIM;
  int sample_index = distribution_i * this->getNumRollouts() * this->getNumTimesteps() * this->CONTROL_DIM;
  float* deriv_action_noise_i_d = &(this->deriv_action_noise_d_[sample_index]);
  float* deriv_action_mean_i_d = &(this->deriv_action_mean_d_[mean_index]);
  mppi::kernels::launchWeightedReductionKernel<CONTROL_DIM>(
      trajectory_weights_d, deriv_action_noise_i_d, deriv_action_mean_i_d, normalizer, this->getNumTimesteps(),
      this->getNumRollouts(), this->params_.sum_strides, this->stream_, synchronize);
  dim3 grid(1, 1, 1);
  dim3 block(1, this->CONTROL_DIM, 1);
  // std::cout << "Integrating optimal sequence" << std::endl;
  integrateNoise<<<grid, block, 0, this->stream_>>>(deriv_action_mean_i_d, &this->control_means_d_[mean_index],
                                                    &this->control_samples_d_[sample_index], 1, this->getNumTimesteps(),
                                                    this->CONTROL_DIM, this->params_.dt);
  HANDLE_ERROR(cudaMemcpyAsync(&this->control_means_d_[mean_index], &this->control_samples_d_[sample_index],
                               sizeof(float) * this->getNumTimesteps() * this->CONTROL_DIM, cudaMemcpyDeviceToDevice,
                               this->stream_));
  HANDLE_ERROR(cudaMemcpyAsync(&this->means_[mean_index], &this->control_means_d_[mean_index],
                               sizeof(float) * this->getNumTimesteps() * CONTROL_DIM, cudaMemcpyDeviceToHost,
                               this->stream_));
  // HANDLE_ERROR(cudaMemcpyAsync(&means_[distribution_i * this->getNumTimesteps() * CONTROL_DIM],
  // deriv_action_mean_i_d,
  //                              sizeof(float) * this->getNumTimesteps() * CONTROL_DIM, cudaMemcpyDeviceToHost,
  //                              this->stream_));
  if (synchronize)
  {
    HANDLE_ERROR(cudaStreamSynchronize(this->stream_));
  }
}

// SMOOTH_MPPI_TEMPLATE
// __host__ void SMOOTH_MPPI_NOISE::setHostOptimalControlSequence(float* optimal_control_trajectory,
//                                                             const int& distribution_i, bool synchronize)
// {
//   if (distribution_i >= this->getNumDistributions())
//   {
//     std::cerr << "Asking for optimal control sequence from distribution " << distribution_i << " out of "
//               << this->getNumDistributions() << " total." << std::endl;
//     return;
//   }

//   HANDLE_ERROR(cudaMemcpyAsync(
//       optimal_control_trajectory, &(this->control_means_d_[this->getNumTimesteps() * CONTROL_DIM * distribution_i]),
//       sizeof(float) * this->getNumTimesteps() * CONTROL_DIM, cudaMemcpyDeviceToHost, this->stream_));
//   if (synchronize)
//   {
//     HANDLE_ERROR(cudaStreamSynchronize(this->stream_));
//   }
// }
}  // namespace sampling_distributions
}  // namespace mppi
