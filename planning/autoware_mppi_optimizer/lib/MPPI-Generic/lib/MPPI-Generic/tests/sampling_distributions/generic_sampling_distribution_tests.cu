#include <gtest/gtest.h>
#include <mppi/core/mppi_common.cuh>
#include <mppi/ddp/util.h>
#include <mppi/dynamics/linear/linear.cuh>
#include <mppi/sampling_distributions/colored_noise/colored_noise.cuh>
#include <mppi/sampling_distributions/gaussian/gaussian.cuh>
#include <mppi/sampling_distributions/nln/nln.cuh>
#include <mppi/utils/math_utils.h>
#include <mppi/utils/type_printing.h>

#if __cplusplus < 201703L  // std::void_t is a C++17 feature, used for inherited_from_gaussian struct
namespace std
{
template <typename... Ts>
struct make_void
{
  typedef void type;
};

template <typename... Ts>
using void_t = typename make_void<Ts...>::type;
}  // namespace std
#endif

namespace ms = mppi::sampling_distributions;

template <class SAMPLER_T>
__global__ void getControlSamplesKernel(SAMPLER_T* __restrict__ sampler, const int num_control_samples,
                                        const int* __restrict__ sample_indexes_d, const int* __restrict__ times_d,
                                        const int* __restrict__ distribution_indexes_d,
                                        const float* __restrict__ outputs_d, float* __restrict__ control_samples_d)
{
  const int size_of_theta_d_bytes = mppi::kernels::calcClassSharedMemSize(sampler, blockDim);
  using OutputIndex = typename SAMPLER_T::OutputIndex;
  const int OUTPUT_DIM = E_INDEX(OutputIndex, NUM_OUTPUTS);
  extern __shared__ float entire_buffer[];
  float* theta_d_shared = &entire_buffer[0 / sizeof(float)];
  float* outputs_shared = &entire_buffer[size_of_theta_d_bytes / sizeof(float)];  // THREAD_BLOCK_X * OUTPUT_DIM in size

  int x_index, x_step;
  mppi::p1::getParallel1DIndex<mppi::p1::Parallel1Dir::THREAD_X>(x_index, x_step);
  int y_index, y_step;
  mppi::p1::getParallel1DIndex<mppi::p1::Parallel1Dir::THREAD_Y>(y_index, y_step);
  int test_index = threadIdx.x + blockDim.x * blockIdx.x;
  // load output into shared memory
  if (test_index < num_control_samples)
  {
    for (int j = y_index; j < OUTPUT_DIM; j += y_step)
    {
      outputs_shared[j + x_index * OUTPUT_DIM] = outputs_d[j + test_index * OUTPUT_DIM];
    }
  }
  __syncthreads();
  float* output = &outputs_shared[x_index * OUTPUT_DIM];
  // initialize sampling distributions
  sampler->initializeDistributions(output, 0.0f, 0.01f, theta_d_shared);
  __syncthreads();
  // get control samples
  if (test_index < num_control_samples)
  {
    int sample_index = sample_indexes_d[test_index];
    int t = times_d[test_index];
    int distribution_idx = distribution_indexes_d[test_index];
    float* u_to_save_to = &control_samples_d[test_index * SAMPLER_T::CONTROL_DIM];
    sampler->readControlSample(sample_index, t, distribution_idx, u_to_save_to, theta_d_shared, y_step, y_index,
                               output);
  }
}

template <class SAMPLER_T>
__global__ void getLikelihoodCostKernel(SAMPLER_T* __restrict__ sampler, const int num_control_samples, float lambda,
                                        float alpha, const int* __restrict__ sample_indexes_d,
                                        const int* __restrict__ times_d, const int* __restrict__ distribution_indexes_d,
                                        const float* __restrict__ outputs_d, float* __restrict__ control_samples_d,
                                        float* __restrict__ costs_d)
{
  const int size_of_theta_d_bytes = mppi::kernels::calcClassSharedMemSize(sampler, blockDim);
  using OutputIndex = typename SAMPLER_T::OutputIndex;
  const int OUTPUT_DIM = E_INDEX(OutputIndex, NUM_OUTPUTS);
  extern __shared__ float entire_buffer[];
  float* theta_d_shared = &entire_buffer[0 / sizeof(float)];
  float* outputs_shared =
      &theta_d_shared[size_of_theta_d_bytes / sizeof(float)];  // THREAD_BLOCK_X * OUTPUT_DIM in size
  float* running_cost_shared = &outputs_shared[blockDim.x * OUTPUT_DIM];
  int running_cost_index = threadIdx.x + blockDim.x * (threadIdx.y + blockDim.y * threadIdx.z);
  float* running_cost = &running_cost_shared[running_cost_index];
  running_cost[0] = 0.0f;

  int x_index, x_step;
  mppi::p1::getParallel1DIndex<mppi::p1::Parallel1Dir::THREAD_X>(x_index, x_step);
  int y_index, y_step;
  mppi::p1::getParallel1DIndex<mppi::p1::Parallel1Dir::THREAD_Y>(y_index, y_step);
  int test_index = threadIdx.x + blockDim.x * blockIdx.x;
  // load output into shared memory
  if (test_index < num_control_samples)
  {
    for (int j = y_index; j < OUTPUT_DIM; j += y_step)
    {
      outputs_shared[j + x_index * OUTPUT_DIM] = outputs_d[j + test_index * OUTPUT_DIM];
    }
  }
  __syncthreads();
  float* output = &outputs_shared[x_index * OUTPUT_DIM];
  // initialize sampling distributions
  sampler->initializeDistributions(output, 0.0f, 0.01f, theta_d_shared);
  __syncthreads();
  // get control samples
  if (test_index < num_control_samples)
  {
    int sample_index = sample_indexes_d[test_index];
    int t = times_d[test_index];
    int distribution_idx = distribution_indexes_d[test_index];
    float* u_to_save_to = &control_samples_d[test_index * SAMPLER_T::CONTROL_DIM];
    sampler->readControlSample(sample_index, t, distribution_idx, u_to_save_to, theta_d_shared, y_step, y_index,
                               output);
  }
  __syncthreads();
  // Calculate likelihood ratio cost
  if (test_index < num_control_samples)
  {
    int sample_index = sample_indexes_d[test_index];
    int t = times_d[test_index];
    int distribution_idx = distribution_indexes_d[test_index];
    float* u = &control_samples_d[test_index * SAMPLER_T::CONTROL_DIM];
    running_cost[0] =
        sampler->computeLikelihoodRatioCost(u, theta_d_shared, sample_index, t, distribution_idx, lambda, alpha);
  }
  // __syncthreads();
  // if (threadIdx.x == 1 && threadIdx.y + threadIdx.z == 0 && blockIdx.x == 0)
  // {
  //   printf("Running costs:\n");
  //   for (int i = 0; i < blockDim.x; i++)
  //   {
  //     float cost_sum = 0.0f;
  //     for (int j = 0; j < blockDim.y; j++)
  //     {
  //       cost_sum += running_cost_shared[i + j * blockDim.x];
  //       printf("(%2d, %2d): %6.3f\n", i, j, running_cost_shared[i + j * blockDim.x]);
  //     }
  //     printf("Sum of y dim for %2d: %8.5f\n", i, cost_sum);
  //   }
  // }
  running_cost = &running_cost_shared[blockDim.x * blockDim.y * threadIdx.z];
  __syncthreads();
  mppi::kernels::costArrayReduction(&running_cost[x_index], blockDim.y, threadIdx.y, blockDim.y, threadIdx.y == 0,
                                    blockDim.x);
  if (test_index < num_control_samples)
  {
    costs_d[test_index] = running_cost[x_index];
  }
}

template <class SAMPLER_T>
__global__ void getFeedbackCostKernel(SAMPLER_T* __restrict__ sampler, const int num_control_samples, float lambda,
                                      float alpha, const int* __restrict__ sample_indexes_d,
                                      const int* __restrict__ times_d, const int* __restrict__ distribution_indexes_d,
                                      const float* __restrict__ outputs_d, float* __restrict__ control_samples_d,
                                      float* __restrict__ costs_d)
{
  const int size_of_theta_d_bytes = mppi::kernels::calcClassSharedMemSize(sampler, blockDim);
  using OutputIndex = typename SAMPLER_T::OutputIndex;
  const int OUTPUT_DIM = E_INDEX(OutputIndex, NUM_OUTPUTS);
  extern __shared__ float entire_buffer[];
  float* theta_d_shared = &entire_buffer[0 / sizeof(float)];
  float* outputs_shared =
      &theta_d_shared[size_of_theta_d_bytes / sizeof(float)];  // THREAD_BLOCK_X * OUTPUT_DIM in size
  float* running_cost_shared = &outputs_shared[blockDim.x * OUTPUT_DIM];
  int running_cost_index = threadIdx.x + blockDim.x * (threadIdx.y + blockDim.y * threadIdx.z);
  float* running_cost = &running_cost_shared[running_cost_index];
  running_cost[0] = 0.0f;

  int x_index, x_step;
  mppi::p1::getParallel1DIndex<mppi::p1::Parallel1Dir::THREAD_X>(x_index, x_step);
  int y_index, y_step;
  mppi::p1::getParallel1DIndex<mppi::p1::Parallel1Dir::THREAD_Y>(y_index, y_step);
  int test_index = threadIdx.x + blockDim.x * blockIdx.x;
  // load output into shared memory
  if (test_index < num_control_samples)
  {
    for (int j = y_index; j < OUTPUT_DIM; j += y_step)
    {
      outputs_shared[j + x_index * OUTPUT_DIM] = outputs_d[j + test_index * OUTPUT_DIM];
    }
  }
  __syncthreads();
  float* output = &outputs_shared[x_index * OUTPUT_DIM];
  // initialize sampling distributions
  sampler->initializeDistributions(output, 0.0f, 0.01f, theta_d_shared);
  __syncthreads();
  // get control samples
  if (test_index < num_control_samples)
  {
    int sample_index = sample_indexes_d[test_index];
    int t = times_d[test_index];
    int distribution_idx = distribution_indexes_d[test_index];
    float* u_to_save_to = &control_samples_d[test_index * SAMPLER_T::CONTROL_DIM];
    sampler->readControlSample(sample_index, t, distribution_idx, u_to_save_to, theta_d_shared, y_step, y_index,
                               output);
  }
  __syncthreads();
  // Calculate Feedback Control cost using random control samples from distribution
  if (test_index < num_control_samples)
  {
    int sample_index = sample_indexes_d[test_index];
    int t = times_d[test_index];
    int distribution_idx = distribution_indexes_d[test_index];
    float* u = &control_samples_d[test_index * SAMPLER_T::CONTROL_DIM];
    running_cost[0] = sampler->computeFeedbackCost(u, theta_d_shared, t, distribution_idx, lambda, alpha);
  }
  running_cost = &running_cost_shared[blockDim.x * blockDim.y * threadIdx.z];
  __syncthreads();
  mppi::kernels::costArrayReduction(&running_cost[x_index], blockDim.y, threadIdx.y, blockDim.y, threadIdx.y == 0,
                                    blockDim.x);
  if (test_index < num_control_samples)
  {
    costs_d[test_index] = running_cost[x_index];
  }
}

template <class SAMPLER_T>
class SamplingDistributionTests : public ::testing::Test
{
public:
  using SAMPLER_PARAMS_T = typename SAMPLER_T::SAMPLING_PARAMS_T;
  static const int OUTPUT_DIM = E_INDEX(SAMPLER_T::OutputIndex, NUM_OUTPUTS);
  typedef Eigen::Matrix<float, OUTPUT_DIM, 1> output_array;
  int num_samples_ = 1000;
  int num_timesteps_ = 100;
  int num_distributions_ = 1;
  int num_verifications_ = 0;
  int* sampling_indices_d_ = nullptr;
  int* times_d_ = nullptr;
  int* distribution_indices_d_ = nullptr;
  float* outputs_d_ = nullptr;
  float* controls_d_ = nullptr;
  float* costs_d_ = nullptr;
  float lambda_ = 1.0f;
  float alpha_ = 0.0f;

  std::shared_ptr<SAMPLER_T> sampler_ = nullptr;
  cudaStream_t stream_ = 0;
  curandGenerator_t gen_;
  dim3 thread_block_ = dim3(32, 2, 1);

  std::vector<int> sampled_indices_;
  std::vector<int> sampled_times_;
  std::vector<int> sampled_distributions_;
  util::EigenAlignedVector<float, OUTPUT_DIM, 1> sampled_outputs_;
  util::EigenAlignedVector<float, SAMPLER_T::CONTROL_DIM, 1> sampled_controls_cpu_;
  util::EigenAlignedVector<float, SAMPLER_T::CONTROL_DIM, 1> sampled_controls_gpu_;

  void SetUp() override
  {
    cudaStreamCreate(&stream_);
    sampler_ = std::make_shared<SAMPLER_T>(stream_);
    sampler_->GPUSetup();
    setUpCudaMemory(3000);
    HANDLE_CURAND_ERROR(curandCreateGenerator(&gen_, CURAND_RNG_PSEUDO_DEFAULT));
    HANDLE_CURAND_ERROR(curandSetPseudoRandomGeneratorSeed(gen_, 42));
  }

  template <class T>
  void freeCudaPtr(T*& ptr)
  {
    if (ptr != nullptr)
    {
      cudaFree(ptr);
    }
    ptr = nullptr;
  }

  void setUpCudaMemory(const int num_verifications)
  {
    if (num_verifications > num_verifications_)
    {
      freeCudaPtr(outputs_d_);
      freeCudaPtr(sampling_indices_d_);
      freeCudaPtr(times_d_);
      freeCudaPtr(distribution_indices_d_);
      freeCudaPtr(controls_d_);
      freeCudaPtr(costs_d_);

      // Allocate GPU memory
      HANDLE_ERROR(cudaMalloc((void**)&sampling_indices_d_, sizeof(int) * num_verifications));
      HANDLE_ERROR(cudaMalloc((void**)&times_d_, sizeof(int) * num_verifications));
      HANDLE_ERROR(cudaMalloc((void**)&distribution_indices_d_, sizeof(int) * num_verifications));
      HANDLE_ERROR(cudaMalloc((void**)&outputs_d_, sizeof(float) * OUTPUT_DIM * num_verifications));
      HANDLE_ERROR(cudaMalloc((void**)&controls_d_, sizeof(float) * SAMPLER_T::CONTROL_DIM * num_verifications));
      HANDLE_ERROR(cudaMalloc((void**)&costs_d_, sizeof(float) * num_verifications));

      num_verifications_ = num_verifications;
    }
  }

  void generateNewSamples()
  {
    // Create sample index
    std::vector<int> samples =
        mppi::math::sample_without_replacement(num_verifications_, num_timesteps_ * num_samples_ * num_distributions_);

    // Fill in sampled indices and copy to GPU
    sampled_indices_.resize(num_verifications_);
    sampled_times_.resize(num_verifications_);
    sampled_distributions_.resize(num_verifications_);
    sampled_outputs_.resize(num_verifications_);
    for (int i = 0; i < num_verifications_; i++)
    {
      const int sample = samples[i];
      sampled_indices_[i] = (sample / (num_timesteps_ * num_distributions_)) % num_samples_;
      sampled_times_[i] = (sample / num_distributions_) % num_timesteps_;
      sampled_distributions_[i] = sample % num_distributions_;
      sampled_outputs_[i] = output_array::Random();
      HANDLE_ERROR(cudaMemcpyAsync(outputs_d_ + i * OUTPUT_DIM, sampled_outputs_[i].data(), sizeof(float) * OUTPUT_DIM,
                                   cudaMemcpyHostToDevice, stream_));
    }
    HANDLE_ERROR(cudaMemcpyAsync(sampling_indices_d_, sampled_indices_.data(), sizeof(int) * num_verifications_,
                                 cudaMemcpyHostToDevice, stream_));
    HANDLE_ERROR(cudaMemcpyAsync(times_d_, sampled_times_.data(), sizeof(int) * num_verifications_,
                                 cudaMemcpyHostToDevice, stream_));
    HANDLE_ERROR(cudaMemcpyAsync(distribution_indices_d_, sampled_distributions_.data(),
                                 sizeof(int) * num_verifications_, cudaMemcpyHostToDevice, stream_));

    // Create non-zero mean
    Eigen::MatrixXf random_mean = Eigen::MatrixXf::Random(SAMPLER_T::CONTROL_DIM, num_timesteps_);
    for (int i = 0; i < num_distributions_; i++)
    {
      sampler_->copyImportanceSamplerToDevice(random_mean.data(), i, false);
    }
    float mean_update = 1.0f;
    sampled_controls_cpu_.resize(num_verifications_);
    sampled_controls_gpu_.resize(num_verifications_);
    sampler_->generateSamples(0, 0, gen_, false);
    HANDLE_ERROR(cudaMemcpyAsync(costs_d_, &mean_update, sizeof(float), cudaMemcpyHostToDevice, stream_));
    for (int i = 0; i < num_distributions_; i++)
    {
      sampler_->updateDistributionParamsFromDevice(costs_d_, 1.0f, i, false);
    }
    HANDLE_ERROR(cudaStreamSynchronize(stream_));
  }

  void updateSampler()
  {
    sampler_->setNumTimesteps(num_timesteps_);
    sampler_->setNumRollouts(num_samples_);
    sampler_->setNumDistributions(num_distributions_);
  }

  void TearDown() override
  {
    HANDLE_ERROR(cudaStreamSynchronize(stream_));
    freeCudaPtr(sampling_indices_d_);
    freeCudaPtr(times_d_);
    freeCudaPtr(distribution_indices_d_);
    freeCudaPtr(outputs_d_);
    freeCudaPtr(controls_d_);
    freeCudaPtr(costs_d_);
  }
};

/**
 * Special setup for Gaussian-based distributions
 **/
template <class T, typename = void>
struct inherited_from_gaussian : std::false_type
{
public:
  void operator()(std::shared_ptr<T> sampler) const
  {  // Empty setup
  }
};

template <class T>
struct inherited_from_gaussian<
    T, std::void_t<decltype(T::SAMPLING_PARAMS_T::MAX_DISTRIBUTIONS),
                   typename std::enable_if<
                       std::is_base_of<ms::GaussianParamsImpl<T::CONTROL_DIM, T::SAMPLING_PARAMS_T::MAX_DISTRIBUTIONS>,
                                       typename T::SAMPLING_PARAMS_T>::value,
                       bool>::type>> : std::true_type
{
public:
  void operator()(std::shared_ptr<T> sampler) const
  {  // Setup std dev and cost coefficients
    auto params = sampler->getParams();
    for (int i = 0; i < T::CONTROL_DIM; i++)
    {
      params.control_cost_coeff[i] = 1.0f;
    }
    for (int dist_i = 0; dist_i < params.num_distributions; dist_i++)
    {
      for (int std_dev_i = 0; std_dev_i < T::CONTROL_DIM; std_dev_i++)
      {
        params.std_dev[std_dev_i + dist_i * T::CONTROL_DIM] = 2.0f;
      }
    }
    sampler->setParams(params);
  }
};

using TYPE_TESTS = ::testing::Types<
    ms::GaussianDistribution<LinearDynamicsParams<4, 1>>, ms::GaussianDistribution<LinearDynamicsParams<4, 2>>,
    ms::GaussianDistribution<LinearDynamicsParams<4, 4>>, ms::GaussianDistribution<LinearDynamicsParams<4, 3>>,
    ms::GaussianDistribution<LinearDynamicsParams<1, 7>>, ms::ColoredNoiseDistribution<LinearDynamicsParams<4, 1>>,
    ms::ColoredNoiseDistribution<LinearDynamicsParams<4, 2>>, ms::ColoredNoiseDistribution<LinearDynamicsParams<4, 4>>,
    ms::ColoredNoiseDistribution<LinearDynamicsParams<3, 3>>, ms::NLNDistribution<LinearDynamicsParams<4, 1>>,
    ms::NLNDistribution<LinearDynamicsParams<4, 2>>, ms::NLNDistribution<LinearDynamicsParams<4, 4>>,
    ms::NLNDistribution<LinearDynamicsParams<3, 3>>>;

TYPED_TEST_SUITE(SamplingDistributionTests, TYPE_TESTS);

TYPED_TEST(SamplingDistributionTests, TestCreation)
{
  using T = TypeParam;
  EXPECT_TRUE(true);
  // testMethod<T, inherited_from_gaussian<T>>();
  inherited_from_gaussian<T>()(this->sampler_);
}

TYPED_TEST(SamplingDistributionTests, SetNumDistributions)
{
  using T = TypeParam;
  this->num_distributions_ = 2;
  this->updateSampler();
  this->generateNewSamples();
  EXPECT_TRUE(true);
}

TYPED_TEST(SamplingDistributionTests, ReadControlsFromGPU)
{
  using T = TypeParam;
  this->updateSampler();
  this->generateNewSamples();

  dim3 grid_dim(mppi::math::int_ceil(this->num_verifications_, this->thread_block_.x), 1, 1);
  std::size_t shared_mem_bytes = mppi::kernels::calcClassSharedMemSize<T>(this->sampler_.get(), this->thread_block_) +
                                 sizeof(float) * this->OUTPUT_DIM * this->thread_block_.x;

  getControlSamplesKernel<T><<<grid_dim, this->thread_block_, shared_mem_bytes, this->stream_>>>(
      this->sampler_->sampling_d_, this->num_verifications_, this->sampling_indices_d_, this->times_d_,
      this->distribution_indices_d_, this->outputs_d_, this->controls_d_);
  HANDLE_ERROR(cudaGetLastError());
  // Copy back to CPU
  for (int i = 0; i < this->num_verifications_; i++)
  {
    HANDLE_ERROR(cudaMemcpyAsync(this->sampled_controls_gpu_[i].data(), this->controls_d_ + i * T::CONTROL_DIM,
                                 sizeof(float) * T::CONTROL_DIM, cudaMemcpyDeviceToHost, this->stream_));
    // Query on CPU
    float* u_i =
        this->sampler_->getControlSample(this->sampled_indices_[i], this->sampled_times_[i],
                                         this->sampled_distributions_[i], nullptr, this->sampled_outputs_[i].data());
    HANDLE_ERROR(cudaMemcpyAsync(this->sampled_controls_cpu_[i].data(), u_i, sizeof(float) * T::CONTROL_DIM,
                                 cudaMemcpyDeviceToHost, this->stream_));
  }
  HANDLE_ERROR(cudaStreamSynchronize(this->stream_));

  for (int i = 0; i < this->num_verifications_; i++)
  {
    float diff = (this->sampled_controls_cpu_[i] - this->sampled_controls_gpu_[i]).array().abs().sum();
    ASSERT_FLOAT_EQ(diff, 0.0f);
  }
}

TYPED_TEST(SamplingDistributionTests, ReadControlsFromGPUMoreDistributions)
{
  using T = TypeParam;
  this->num_distributions_ = 2;
  this->updateSampler();
  this->generateNewSamples();

  dim3 grid_dim(mppi::math::int_ceil(this->num_verifications_, this->thread_block_.x), 1, 1);
  std::size_t shared_mem_bytes = mppi::kernels::calcClassSharedMemSize<T>(this->sampler_.get(), this->thread_block_) +
                                 sizeof(float) * this->OUTPUT_DIM * this->thread_block_.x;

  getControlSamplesKernel<T><<<grid_dim, this->thread_block_, shared_mem_bytes, this->stream_>>>(
      this->sampler_->sampling_d_, this->num_verifications_, this->sampling_indices_d_, this->times_d_,
      this->distribution_indices_d_, this->outputs_d_, this->controls_d_);
  HANDLE_ERROR(cudaGetLastError());
  // Copy back to CPU
  for (int i = 0; i < this->num_verifications_; i++)
  {
    HANDLE_ERROR(cudaMemcpyAsync(this->sampled_controls_gpu_[i].data(), this->controls_d_ + i * T::CONTROL_DIM,
                                 sizeof(float) * T::CONTROL_DIM, cudaMemcpyDeviceToHost, this->stream_));
    // Query on CPU
    float* u_i =
        this->sampler_->getControlSample(this->sampled_indices_[i], this->sampled_times_[i],
                                         this->sampled_distributions_[i], nullptr, this->sampled_outputs_[i].data());
    HANDLE_ERROR(cudaMemcpyAsync(this->sampled_controls_cpu_[i].data(), u_i, sizeof(float) * T::CONTROL_DIM,
                                 cudaMemcpyDeviceToHost, this->stream_));
  }
  HANDLE_ERROR(cudaStreamSynchronize(this->stream_));

  for (int i = 0; i < this->num_verifications_; i++)
  {
    float diff = (this->sampled_controls_cpu_[i] - this->sampled_controls_gpu_[i]).array().abs().sum();
    ASSERT_FLOAT_EQ(diff, 0.0f);
  }
}

TYPED_TEST(SamplingDistributionTests, ReadControlsFromGPUDifferentNoiseForDistributions)
{
  using T = TypeParam;
  this->num_distributions_ = 2;
  auto params = this->sampler_->getParams();
  params.use_same_noise_for_all_distributions = false;
  this->sampler_->setParams(params);
  this->updateSampler();
  this->generateNewSamples();

  dim3 grid_dim(mppi::math::int_ceil(this->num_verifications_, this->thread_block_.x), 1, 1);
  std::size_t shared_mem_bytes = mppi::kernels::calcClassSharedMemSize<T>(this->sampler_.get(), this->thread_block_) +
                                 sizeof(float) * this->OUTPUT_DIM * this->thread_block_.x;

  getControlSamplesKernel<T><<<grid_dim, this->thread_block_, shared_mem_bytes, this->stream_>>>(
      this->sampler_->sampling_d_, this->num_verifications_, this->sampling_indices_d_, this->times_d_,
      this->distribution_indices_d_, this->outputs_d_, this->controls_d_);
  HANDLE_ERROR(cudaGetLastError());
  // Copy back to CPU
  for (int i = 0; i < this->num_verifications_; i++)
  {
    HANDLE_ERROR(cudaMemcpyAsync(this->sampled_controls_gpu_[i].data(), this->controls_d_ + i * T::CONTROL_DIM,
                                 sizeof(float) * T::CONTROL_DIM, cudaMemcpyDeviceToHost, this->stream_));
    // Query on CPU
    float* u_i =
        this->sampler_->getControlSample(this->sampled_indices_[i], this->sampled_times_[i],
                                         this->sampled_distributions_[i], nullptr, this->sampled_outputs_[i].data());
    HANDLE_ERROR(cudaMemcpyAsync(this->sampled_controls_cpu_[i].data(), u_i, sizeof(float) * T::CONTROL_DIM,
                                 cudaMemcpyDeviceToHost, this->stream_));
  }
  HANDLE_ERROR(cudaStreamSynchronize(this->stream_));

  for (int i = 0; i < this->num_verifications_; i++)
  {
    float diff = (this->sampled_controls_cpu_[i] - this->sampled_controls_gpu_[i]).array().abs().sum();
    ASSERT_FLOAT_EQ(diff, 0.0f);
  }
}

TYPED_TEST(SamplingDistributionTests, CompareLikelihoodRatioCostsCPUvsGPU)
{
  using T = TypeParam;
  this->num_distributions_ = 2;
  auto params = this->sampler_->getParams();
  params.use_same_noise_for_all_distributions = false;
  this->sampler_->setParams(params);
  inherited_from_gaussian<T>()(this->sampler_);
  this->updateSampler();
  this->generateNewSamples();

  dim3 grid_dim(mppi::math::int_ceil(this->num_verifications_, this->thread_block_.x), 1, 1);
  std::size_t shared_mem_bytes =
      mppi::kernels::calcClassSharedMemSize<T>(this->sampler_.get(), this->thread_block_) +
      sizeof(float) * (this->OUTPUT_DIM * this->thread_block_.x +
                       this->thread_block_.x * this->thread_block_.y * this->thread_block_.z);

  getLikelihoodCostKernel<T><<<grid_dim, this->thread_block_, shared_mem_bytes, this->stream_>>>(
      this->sampler_->sampling_d_, this->num_verifications_, this->lambda_, this->alpha_, this->sampling_indices_d_,
      this->times_d_, this->distribution_indices_d_, this->outputs_d_, this->controls_d_, this->costs_d_);
  HANDLE_ERROR(cudaGetLastError());

  std::vector<float> costs_gpu(this->num_verifications_);
  // Copy back to CPU
  for (int i = 0; i < this->num_verifications_; i++)
  {
    HANDLE_ERROR(cudaMemcpyAsync(this->sampled_controls_gpu_[i].data(), this->controls_d_ + i * T::CONTROL_DIM,
                                 sizeof(float) * T::CONTROL_DIM, cudaMemcpyDeviceToHost, this->stream_));
    // Query on CPU
    float* u_i =
        this->sampler_->getControlSample(this->sampled_indices_[i], this->sampled_times_[i],
                                         this->sampled_distributions_[i], nullptr, this->sampled_outputs_[i].data());
    HANDLE_ERROR(cudaMemcpyAsync(this->sampled_controls_cpu_[i].data(), u_i, sizeof(float) * T::CONTROL_DIM,
                                 cudaMemcpyDeviceToHost, this->stream_));
  }
  HANDLE_ERROR(cudaMemcpyAsync(costs_gpu.data(), this->costs_d_, sizeof(float) * this->num_verifications_,
                               cudaMemcpyDeviceToHost, this->stream_));
  HANDLE_ERROR(cudaStreamSynchronize(this->stream_));

  float cost;
  for (int i = 2; i < this->num_verifications_; i++)
  {
    cost = 0.0f;
    cost = this->sampler_->computeLikelihoodRatioCost(this->sampled_controls_cpu_[i], this->sampled_indices_[i],
                                                      this->sampled_times_[i], this->sampled_distributions_[i],
                                                      this->lambda_, this->alpha_);
    ASSERT_NEAR(cost, costs_gpu[i], fabsf(cost) * 5e-5)
        << " failed on sample " << this->sampled_indices_[i] << ", time " << this->sampled_times_[i]
        << ", dist: " << this->sampled_distributions_[i];
  }
}

TYPED_TEST(SamplingDistributionTests, CompareFeedbackCostsCPUvsGPU)
{
  using T = TypeParam;
  this->num_distributions_ = 2;
  auto params = this->sampler_->getParams();
  params.use_same_noise_for_all_distributions = false;
  this->sampler_->setParams(params);
  inherited_from_gaussian<T>()(this->sampler_);
  this->updateSampler();
  this->generateNewSamples();

  dim3 grid_dim(mppi::math::int_ceil(this->num_verifications_, this->thread_block_.x), 1, 1);
  std::size_t shared_mem_bytes =
      mppi::kernels::calcClassSharedMemSize<T>(this->sampler_.get(), this->thread_block_) +
      sizeof(float) * (this->OUTPUT_DIM * this->thread_block_.x +
                       this->thread_block_.x * this->thread_block_.y * this->thread_block_.z);

  getFeedbackCostKernel<T><<<grid_dim, this->thread_block_, shared_mem_bytes, this->stream_>>>(
      this->sampler_->sampling_d_, this->num_verifications_, this->lambda_, this->alpha_, this->sampling_indices_d_,
      this->times_d_, this->distribution_indices_d_, this->outputs_d_, this->controls_d_, this->costs_d_);
  HANDLE_ERROR(cudaGetLastError());

  std::vector<float> costs_gpu(this->num_verifications_);
  // Copy back to CPU
  for (int i = 0; i < this->num_verifications_; i++)
  {
    HANDLE_ERROR(cudaMemcpyAsync(this->sampled_controls_gpu_[i].data(), this->controls_d_ + i * T::CONTROL_DIM,
                                 sizeof(float) * T::CONTROL_DIM, cudaMemcpyDeviceToHost, this->stream_));
    // Query on CPU
    float* u_i =
        this->sampler_->getControlSample(this->sampled_indices_[i], this->sampled_times_[i],
                                         this->sampled_distributions_[i], nullptr, this->sampled_outputs_[i].data());
    HANDLE_ERROR(cudaMemcpyAsync(this->sampled_controls_cpu_[i].data(), u_i, sizeof(float) * T::CONTROL_DIM,
                                 cudaMemcpyDeviceToHost, this->stream_));
  }
  HANDLE_ERROR(cudaMemcpyAsync(costs_gpu.data(), this->costs_d_, sizeof(float) * this->num_verifications_,
                               cudaMemcpyDeviceToHost, this->stream_));
  HANDLE_ERROR(cudaStreamSynchronize(this->stream_));

  float cost;
  for (int i = 0; i < this->num_verifications_; i++)
  {
    cost = 0.0f;
    cost = this->sampler_->computeFeedbackCost(this->sampled_controls_cpu_[i], this->sampled_times_[i],
                                               this->sampled_distributions_[i], this->lambda_, this->alpha_);
    ASSERT_FLOAT_EQ(cost, costs_gpu[i]) << " failed on sample " << this->sampled_indices_[i] << ", time "
                                        << this->sampled_times_[i] << ", dist: " << this->sampled_distributions_[i];
  }
}
