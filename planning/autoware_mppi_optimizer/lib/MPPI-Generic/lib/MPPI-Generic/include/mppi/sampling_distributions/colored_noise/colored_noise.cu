/**
 * Created by Bogdan Vlahov on 3/25/2023
 **/

#define COLORED_TEMPLATE template <class CLASS_T, template <int> class PARAMS_TEMPLATE, class DYN_PARAMS_T>
#define COLORED_NOISE ColoredNoiseDistributionImpl<CLASS_T, PARAMS_TEMPLATE, DYN_PARAMS_T>

#include <mppi/sampling_distributions/colored_noise/colored_noise.cuh>
#include <mppi/utils/cuda_math_utils.cuh>
#include <mppi/utils/math_utils.h>

__global__ void configureFrequencyNoise(cufftComplex* noise, float* variance, int num_samples, int control_dim,
                                        int num_freq)
{
  int sample_index = blockDim.x * blockIdx.x + threadIdx.x;
  int freq_index = blockDim.y * blockIdx.y + threadIdx.y;
  int control_index = blockDim.z * blockIdx.z + threadIdx.z;

  if (sample_index < num_samples && freq_index < num_freq && control_index < control_dim)
  {
    int noise_index = (sample_index * control_dim + control_index) * num_freq + freq_index;
    int variance_index = control_index * num_freq + freq_index;
    noise[noise_index].x *= variance[variance_index];
    if (freq_index == 0)
    {
      noise[noise_index].y = 0;
    }
    else if (num_freq % 2 == 1 && freq_index == num_freq - 1)
    {
      noise[noise_index].y = 0;
    }
    else
    {
      noise[noise_index].y *= variance[variance_index];
    }
  }
}

__global__ void rearrangeNoise(float* input, float* output, float* variance, int num_trajectories, int num_timesteps,
                               int control_dim, int offset_t, float decay_rate)
{
  const int sample_index = blockIdx.x * blockDim.x + threadIdx.x;
  const int time_index = blockIdx.y * blockDim.y + threadIdx.y;
  const int control_index = blockIdx.z * blockDim.z + threadIdx.z;
  const float decayed_offset = decay_rate == 0 ? 0 : powf(decay_rate, time_index);
  if (sample_index < num_trajectories && time_index < (num_timesteps) && control_index < control_dim)
  {  // cuFFT does not normalize inverse transforms so a division by the num_timesteps is required
    output[(sample_index * num_timesteps + time_index) * control_dim + control_index] =
        (input[(sample_index * control_dim + control_index) * 2 * num_timesteps + time_index] -
         input[(sample_index * control_dim + control_index) * 2 * num_timesteps + offset_t] * decayed_offset) /
        (variance[control_index] * 2 * num_timesteps);
    // printf("ROLLOUT %d CONTROL %d TIME %d: in %f out: %f\n", sample_index, control_index, time_index,
    //     input[(sample_index * control_dim + control_index) * num_timesteps + time_index],
    //     output[(sample_index * num_timesteps + time_index) * control_dim + control_index]);
  }
}

void powerlaw_psd_gaussian(std::vector<float>& exponents, int num_timesteps, int num_trajectories,
                           float* control_noise_d, int offset_t, curandGenerator_t& gen, float offset_decay_rate,
                           cudaStream_t stream, float fmin)
{
  const int BLOCKSIZE_X = 32;
  const int BLOCKSIZE_Y = 32;
  const int BLOCKSIZE_Z = 1;
  int control_dim = exponents.size();

  std::vector<float> sample_freq;
  const int sample_num_timesteps = num_timesteps * 2;
  fftfreq(sample_num_timesteps, sample_freq);
  float cutoff_freq = fmaxf(fmin, 1.0f / sample_num_timesteps);
  int freq_size = sample_freq.size();

  int smaller_index = 0;
  Eigen::MatrixXf sample_freqs(freq_size, control_dim);

  // Adjust the weighting of each frequency by the exponents
  for (int i = 0; i < freq_size; i++)
  {
    if (sample_freq[i] < cutoff_freq)
    {
      smaller_index++;
    }
    else if (smaller_index < freq_size)
    {
      for (int j = 0; j < smaller_index; j++)
      {
        sample_freq[j] = sample_freq[smaller_index];
        for (int k = 0; k < control_dim; k++)
        {
          sample_freqs(j, k) = powf(sample_freq[smaller_index], -exponents[k] / 2.0f);
        }
      }
    }
    for (int j = 0; j < control_dim; j++)
    {
      sample_freqs(i, j) = powf(sample_freq[i], -exponents[j] / 2.0f);
    }
  }

  // Calculate variance
  float sigma[control_dim] = { 0 };
  for (int i = 0; i < control_dim; i++)
  {
    for (int j = 1; j < freq_size - 1; j++)
    {
      sigma[i] += SQ(sample_freqs(j, i));
    }
    // std::for_each(sample_freq.begin() + 1, sample_freq.end() - 1, [&sigma, &i](float j) { sigma[i] += powf(j, 2); });
    sigma[i] += SQ(sample_freqs(freq_size - 1, i) * ((1.0f + (sample_num_timesteps % 2)) / 2.0f));
    sigma[i] = 2.0f * sqrtf(sigma[i]) / sample_num_timesteps;
  }

  // Sample the noise in frequency domain and reutrn to time domain
  cufftHandle plan;
  const int batch = num_trajectories * control_dim;
  // Need 2 * (sample_num_timesteps / 2 + 1) * batch of randomly sampled values
  // float* samples_in_freq_d;
  float* sigma_d;
  float* noise_in_time_d;
  cufftComplex* samples_in_freq_complex_d;
  float* freq_coeffs_d;
  // HANDLE_ERROR(cudaMallocAsync((void**)&samples_in_freq_d, sizeof(float) * 2 * batch * freq_size, stream));
  // HANDLE_ERROR(cudaMallocAsync((void**)&samples_in_freq_d, sizeof(float) * 2 * batch * sample_num_timesteps,
  // stream));
#if defined(CUDART_VERSION) && CUDART_VERSION > 11200
  HANDLE_ERROR(cudaMallocAsync((void**)&freq_coeffs_d, sizeof(float) * freq_size * control_dim, stream));
  HANDLE_ERROR(cudaMallocAsync((void**)&samples_in_freq_complex_d, sizeof(cufftComplex) * batch * freq_size, stream));
  HANDLE_ERROR(cudaMallocAsync((void**)&noise_in_time_d, sizeof(float) * batch * sample_num_timesteps, stream));
  HANDLE_ERROR(cudaMallocAsync((void**)&sigma_d, sizeof(float) * control_dim, stream));
#else
  HANDLE_ERROR(cudaMalloc((void**)&freq_coeffs_d, sizeof(float) * freq_size * control_dim));
  HANDLE_ERROR(cudaMalloc((void**)&samples_in_freq_complex_d, sizeof(cufftComplex) * batch * freq_size));
  HANDLE_ERROR(cudaMalloc((void**)&noise_in_time_d, sizeof(float) * batch * sample_num_timesteps));
  HANDLE_ERROR(cudaMalloc((void**)&sigma_d, sizeof(float) * control_dim));
#endif
  // curandSetStream(gen, stream);
  HANDLE_CURAND_ERROR(curandGenerateNormal(gen, (float*)samples_in_freq_complex_d, 2 * batch * freq_size, 0.0, 1.0));
  HANDLE_ERROR(cudaMemcpyAsync(freq_coeffs_d, sample_freqs.data(), sizeof(float) * freq_size * control_dim,
                               cudaMemcpyHostToDevice, stream));
  HANDLE_ERROR(cudaMemcpyAsync(sigma_d, sigma, sizeof(float) * control_dim, cudaMemcpyHostToDevice, stream));
  const int variance_grid_x = (num_trajectories - 1) / BLOCKSIZE_X + 1;
  const int variance_grid_y = (freq_size - 1) / BLOCKSIZE_Y + 1;
  const int variance_grid_z = (control_dim - 1) / BLOCKSIZE_Z + 1;
  dim3 grid(variance_grid_x, variance_grid_y, variance_grid_z);
  dim3 block(BLOCKSIZE_X, BLOCKSIZE_Y, BLOCKSIZE_Z);
  // configureFrequencyNoise<<<grid, block, 0, stream>>>((cuComplex*) samples_in_freq_d, freq_coeffs_d, freq_size,
  // batch);
  configureFrequencyNoise<<<grid, block, 0, stream>>>(samples_in_freq_complex_d, freq_coeffs_d, num_trajectories,
                                                      control_dim, freq_size);
  HANDLE_ERROR(cudaGetLastError());
  HANDLE_CUFFT_ERROR(cufftPlan1d(&plan, sample_num_timesteps, CUFFT_C2R, batch));
  HANDLE_CUFFT_ERROR(cufftSetStream(plan, stream));
  // freq_data needs to be batch number of sample_num_timesteps/2 + 1 cuComplex values
  // time_data needs to be batch * sample_num_timesteps floats
  HANDLE_CUFFT_ERROR(cufftExecC2R(plan, samples_in_freq_complex_d, noise_in_time_d));
  const int reorder_grid_x = (num_trajectories - 1) / BLOCKSIZE_X + 1;
  const int reorder_grid_y = (num_timesteps - 1) / BLOCKSIZE_Y + 1;
  const int reorder_grid_z = (control_dim - 1) / BLOCKSIZE_Z + 1;
  dim3 reorder_grid(reorder_grid_x, reorder_grid_y, reorder_grid_z);
  dim3 reorder_block(BLOCKSIZE_X, BLOCKSIZE_Y, BLOCKSIZE_Z);
  // std::cout << "Grid: " << reorder_grid.x << ", " << reorder_grid.y << ", " << reorder_grid.z << std::endl;
  // std::cout << "Block: " << reorder_block.x << ", " << reorder_block.y << ", " << reorder_block.z << std::endl;
  rearrangeNoise<<<reorder_grid, reorder_block, 0, stream>>>(noise_in_time_d, control_noise_d, sigma_d,
                                                             num_trajectories, num_timesteps, control_dim, offset_t,
                                                             offset_decay_rate);
  HANDLE_ERROR(cudaGetLastError());
  HANDLE_ERROR(cudaStreamSynchronize(stream));
  HANDLE_CUFFT_ERROR(cufftDestroy(plan));
#if defined(CUDART_VERSION) && CUDART_VERSION > 11200
  // HANDLE_ERROR(cudaFreeAsync(samples_in_freq_d, stream));
  HANDLE_ERROR(cudaFreeAsync(freq_coeffs_d, stream));
  HANDLE_ERROR(cudaFreeAsync(sigma_d, stream));
  HANDLE_ERROR(cudaFreeAsync(samples_in_freq_complex_d, stream));
  HANDLE_ERROR(cudaFreeAsync(noise_in_time_d, stream));
#else
  // HANDLE_ERROR(cudaFree(samples_in_freq_d));
  HANDLE_ERROR(cudaFree(freq_coeffs_d));
  HANDLE_ERROR(cudaFree(sigma_d));
  HANDLE_ERROR(cudaFree(samples_in_freq_complex_d));
  HANDLE_ERROR(cudaFree(noise_in_time_d));
#endif
}

namespace mppi
{
namespace sampling_distributions
{
COLORED_TEMPLATE
COLORED_NOISE::ColoredNoiseDistributionImpl(cudaStream_t stream) : PARENT_CLASS::GaussianDistributionImpl(stream)
{
}

COLORED_TEMPLATE
COLORED_NOISE::ColoredNoiseDistributionImpl(const SAMPLING_PARAMS_T& params, cudaStream_t stream)
  : PARENT_CLASS::GaussianDistributionImpl(params, stream)
{
}

COLORED_TEMPLATE
__host__ void COLORED_NOISE::freeCudaMem()
{
  if (this->GPUMemStatus_)
  {
    cudaFree(freq_coeffs_d_);
    cudaFree(samples_in_freq_complex_d_);
    cudaFree(noise_in_time_d_);
    cudaFree(frequency_sigma_d_);
    freq_coeffs_d_ = nullptr;
    frequency_sigma_d_ = nullptr;
    noise_in_time_d_ = nullptr;
    samples_in_freq_complex_d_ = nullptr;
    cufftDestroy(plan_);
  }
  PARENT_CLASS::freeCudaMem();
}

COLORED_TEMPLATE
__host__ void COLORED_NOISE::allocateCUDAMemoryHelper()
{
  PARENT_CLASS::allocateCUDAMemoryHelper();
  if (this->GPUMemStatus_)
  {
    const int sample_num_timesteps = 2 * this->getNumTimesteps();
    const int freq_size = sample_num_timesteps / 2 + 1;
#if defined(CUDART_VERSION) && CUDART_VERSION > 11200
    if (frequency_sigma_d_)
    {
      HANDLE_ERROR(cudaFreeAsync(frequency_sigma_d_, this->stream_));
    }
    if (samples_in_freq_complex_d_)
    {
      HANDLE_ERROR(cudaFreeAsync(samples_in_freq_complex_d_, this->stream_));
    }
    if (noise_in_time_d_)
    {
      HANDLE_ERROR(cudaFreeAsync(noise_in_time_d_, this->stream_));
    }
    if (freq_coeffs_d_)
    {
      HANDLE_ERROR(cudaFreeAsync(freq_coeffs_d_, this->stream_));
    }
    HANDLE_ERROR(
        cudaMallocAsync((void**)&freq_coeffs_d_, sizeof(float) * freq_size * this->CONTROL_DIM, this->stream_));
    HANDLE_ERROR(cudaMallocAsync((void**)&frequency_sigma_d_, sizeof(float) * this->CONTROL_DIM, this->stream_));
    HANDLE_ERROR(cudaMallocAsync((void**)&samples_in_freq_complex_d_,
                                 sizeof(cufftComplex) * this->getNumRollouts() * this->CONTROL_DIM * freq_size *
                                     this->getNumDistributions(),
                                 this->stream_));
    HANDLE_ERROR(cudaMallocAsync((void**)&noise_in_time_d_,
                                 sizeof(float) * this->getNumRollouts() * this->CONTROL_DIM * sample_num_timesteps *
                                     this->getNumDistributions(),
                                 this->stream_));
#else
    if (frequency_sigma_d_)
    {
      HANDLE_ERROR(cudaFree(frequency_sigma_d_));
    }
    if (samples_in_freq_complex_d_)
    {
      HANDLE_ERROR(cudaFree(samples_in_freq_complex_d_));
    }
    if (noise_in_time_d_)
    {
      HANDLE_ERROR(cudaFree(noise_in_time_d_));
    }
    if (freq_coeffs_d_)
    {
      HANDLE_ERROR(cudaFree(freq_coeffs_d_));
    }
    HANDLE_ERROR(cudaMalloc((void**)&freq_coeffs_d_, sizeof(float) * freq_size * this->CONTROL_DIM));
    HANDLE_ERROR(cudaMalloc((void**)&frequency_sigma_d_, sizeof(float) * this->CONTROL_DIM));
    HANDLE_ERROR(cudaMalloc((void**)&samples_in_freq_complex_d_, sizeof(cufftComplex) * this->getNumRollouts() *
                                                                     this->CONTROL_DIM * freq_size *
                                                                     this->getNumDistributions()));
    HANDLE_ERROR(cudaMalloc((void**)&noise_in_time_d_, sizeof(float) * this->getNumRollouts() * this->CONTROL_DIM *
                                                           sample_num_timesteps * this->getNumDistributions()));
#endif
    // Recreate FFT Plan
    HANDLE_CUFFT_ERROR(cufftPlan1d(&plan_, sample_num_timesteps, CUFFT_C2R,
                                   this->getNumRollouts() * this->getNumDistributions() * this->CONTROL_DIM));
    HANDLE_CUFFT_ERROR(cufftSetStream(plan_, this->stream_));
  }
}

COLORED_TEMPLATE
__host__ void COLORED_NOISE::generateSamples(const int& optimization_stride, const int& iteration_num,
                                             curandGenerator_t& gen, bool synchronize)
{
  const int BLOCKSIZE_X = this->params_.rewrite_controls_block_dim.x;
  const int BLOCKSIZE_Y = this->params_.rewrite_controls_block_dim.y;
  const int BLOCKSIZE_Z = this->params_.rewrite_controls_block_dim.z;
  const int num_trajectories = this->getNumRollouts() * this->getNumDistributions();

  std::vector<float> sample_freq;
  const int sample_num_timesteps = 2 * this->getNumTimesteps();
  fftfreq(sample_num_timesteps, sample_freq);
  const float cutoff_freq = fmaxf(this->params_.fmin, 1.0f / sample_num_timesteps);
  const int freq_size = sample_freq.size();

  int smaller_index = 0;
  const int local_control_dim = this->CONTROL_DIM;  // Needed for methods which use pass by reference
  Eigen::MatrixXf sample_freqs(freq_size, local_control_dim);

  // Adjust the weighting of each frequency by the exponents
  for (int i = 0; i < freq_size; i++)
  {
    if (sample_freq[i] < cutoff_freq)
    {
      smaller_index++;
    }
    else if (smaller_index < freq_size)
    {
      for (int j = 0; j < smaller_index; j++)
      {
        sample_freq[j] = sample_freq[smaller_index];
        for (int k = 0; k < this->CONTROL_DIM; k++)
        {
          sample_freqs(j, k) = powf(sample_freq[smaller_index], -this->params_.exponents[k] / 2.0f);
        }
      }
    }
    for (int j = 0; j < this->CONTROL_DIM; j++)
    {
      sample_freqs(i, j) = powf(sample_freq[i], -this->params_.exponents[j] / 2.0f);
    }
  }

  // Calculate variance
  float sigma[this->CONTROL_DIM] = { 0 };
  for (int i = 0; i < this->CONTROL_DIM; i++)
  {
    for (int j = 1; j < freq_size - 1; j++)
    {
      sigma[i] += SQ(sample_freqs(j, i));
    }
    sigma[i] += SQ(sample_freqs(freq_size - 1, i) * ((1.0f + (sample_num_timesteps % 2)) / 2.0f));
    sigma[i] = 2.0f * sqrtf(sigma[i]) / sample_num_timesteps;
  }

  // Sample the noise in frequency domain and reutrn to time domain
  const int batch = num_trajectories * this->CONTROL_DIM;
  // Need 2 * (sample_num_timesteps / 2 + 1) * batch of randomly sampled values
  HANDLE_CURAND_ERROR(curandGenerateNormal(gen, (float*)samples_in_freq_complex_d_, 2 * batch * freq_size, 0.0, 1.0));
  HANDLE_ERROR(cudaMemcpyAsync(freq_coeffs_d_, sample_freqs.data(), sizeof(float) * freq_size * this->CONTROL_DIM,
                               cudaMemcpyHostToDevice, this->stream_));
  HANDLE_ERROR(cudaMemcpyAsync(frequency_sigma_d_, sigma, sizeof(float) * this->CONTROL_DIM, cudaMemcpyHostToDevice,
                               this->stream_));
  const int num_trajectories_grid_x = mppi::math::int_ceil(num_trajectories, BLOCKSIZE_X);
  const int variance_grid_y = (freq_size - 1) / BLOCKSIZE_Y + 1;
  const int control_grid_z = mppi::math::int_ceil(local_control_dim, BLOCKSIZE_Z);
  dim3 grid(num_trajectories_grid_x, variance_grid_y, control_grid_z);
  dim3 block(BLOCKSIZE_X, BLOCKSIZE_Y, BLOCKSIZE_Z);
  configureFrequencyNoise<<<grid, block, 0, this->stream_>>>(samples_in_freq_complex_d_, freq_coeffs_d_,
                                                             num_trajectories, this->CONTROL_DIM, freq_size);
  HANDLE_ERROR(cudaGetLastError());
  // freq_data needs to be batch number of num_timesteps/2 + 1 cuComplex values
  // time_data needs to be batch * num_timesteps floats
  HANDLE_CUFFT_ERROR(cufftExecC2R(plan_, samples_in_freq_complex_d_, noise_in_time_d_));

  // Change axes ordering from [trajectories, control, time] to [trajectories, time, control]
  const int reorder_grid_y = mppi::math::int_ceil(this->getNumTimesteps(), BLOCKSIZE_Y);
  dim3 reorder_grid(num_trajectories_grid_x, reorder_grid_y, control_grid_z);
  rearrangeNoise<<<reorder_grid, block, 0, this->stream_>>>(
      noise_in_time_d_, this->control_samples_d_, frequency_sigma_d_, num_trajectories, this->getNumTimesteps(),
      this->CONTROL_DIM, optimization_stride, this->getOffsetDecayRate());

  // Rewrite pure noise into actual control samples
  dim3 control_writing_grid;
  control_writing_grid.x = mppi::math::int_ceil(this->getNumRollouts(), BLOCKSIZE_X);
  control_writing_grid.y = mppi::math::int_ceil(this->getNumTimesteps(), BLOCKSIZE_Y);
  control_writing_grid.z = mppi::math::int_ceil(this->getNumDistributions(), BLOCKSIZE_Z);
  unsigned int std_dev_mem_size = this->getNumDistributions() * this->CONTROL_DIM;
  // Allocate shared memory for std_deviations per timestep or constant across the trajectory
  std_dev_mem_size = mppi::math::nearest_multiple_4(
      this->params_.time_specific_std_dev ? std_dev_mem_size * this->getNumTimesteps() : std_dev_mem_size);
  unsigned int shared_mem_size =
      std_dev_mem_size +
      mppi::math::nearest_multiple_4(this->getNumDistributions() * this->getNumTimesteps() * this->CONTROL_DIM) +
      mppi::math::nearest_multiple_4(BLOCKSIZE_X * BLOCKSIZE_Y * BLOCKSIZE_Z * this->CONTROL_DIM);
  shared_mem_size *= sizeof(float);
  setGaussianControls<<<control_writing_grid, this->params_.rewrite_controls_block_dim, shared_mem_size,
                        this->stream_>>>(
      this->control_means_d_, this->std_dev_d_, this->control_samples_d_, this->CONTROL_DIM, this->getNumTimesteps(),
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
#undef COLORED_TEMPLATE
#undef COLORED_NOISE
