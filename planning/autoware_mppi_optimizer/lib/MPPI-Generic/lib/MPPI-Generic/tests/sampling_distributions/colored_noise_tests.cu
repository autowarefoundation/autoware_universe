//
// Created by Bogdan on 12/26/21
//

#include <gtest/gtest.h>
#include <mppi/sampling_distributions/colored_noise/colored_noise.cuh>

#include <numeric>
#include "gtest/gtest.h"

void assert_float_rel_near(const float known, const float compute, float rel_err)
{
  float err = fabsf(compute - known) / known;
  ASSERT_NEAR(known, compute, rel_err) << "Relative error is " << err;
}

TEST(cuFFT, checkErrorCode)
{
  cufftHandle plan;
  cuComplex* input_d;
  float* output_d;
  // As this call is intended to cause issues, disable compiler warning
  // src: https://stackoverflow.com/questions/14831051/how-to-disable-a-specific-nvcc-compiler-warnings
  // https://stackoverflow.com/questions/56193080/how-do-i-apply-a-flag-setting-nvcc-pragma-to-only-a-few-lines-of-code
  // https://docs.nvidia.com/cuda/cuda-c-programming-guide/index.html#nv-diagnostic-pragmas
#pragma push
#pragma diag_suppress = used_before_set
  auto status = cufftExecC2R(plan, input_d, output_d);
#pragma pop
  std::string error_string = cufftGetErrorString(status);
  // std::cout << error_string << std::endl;
  EXPECT_TRUE(error_string == "cuFFT was passed an invalid plan handle");
}

template <int C_DIM>
struct TestDynamicsParams : public DynamicsParams
{
  enum class ControlIndex : int
  {
    NUM_CONTROLS = C_DIM,
  };
  enum class OutputIndex : int
  {
    EMPTY = 0,
    NUM_OUTPUTS
  };
};

template <class DYN_PARAMS_T>
class TestNoise : public ::testing::Test
{
public:
  const int NUM_TIMESTEPS = 250;
  const int NUM_ROLLOUTS = 5000;
  const int CONTROL_DIM = C_IND_CLASS(DYN_PARAMS_T, NUM_CONTROLS);
  using SAMPLER_T = mppi::sampling_distributions::ColoredNoiseDistribution<DYN_PARAMS_T>;
  using SAMPLER_PARAMS_T = typename SAMPLER_T::SAMPLING_PARAMS_T;

  SAMPLER_T* sampler;
  cudaStream_t stream;
  curandGenerator_t* gen;

protected:
  void SetUp() override
  {
    SAMPLER_PARAMS_T params;
    for (int i = 0; i < CONTROL_DIM; i++)
    {
      params.exponents[i] = 0.0;
    }
    params.num_timesteps = NUM_TIMESTEPS;
    params.num_rollouts = NUM_ROLLOUTS;
    params.pure_noise_trajectories_percentage = 0.0;
    params.offset_decay_rate = 0.0;

    cudaStreamCreate(&stream);
    gen = new curandGenerator_t();
    curandCreateGenerator(gen, CURAND_RNG_PSEUDO_DEFAULT);
    curandSetPseudoRandomGeneratorSeed(*gen, 42);
    curandSetGeneratorOffset(*gen, 0);
    curandSetStream(*gen, stream);
    sampler = new SAMPLER_T(params, stream);
    sampler->GPUSetup();
  }

  void TearDown() override
  {
    delete gen;
    delete sampler;
  }
};

using DIFFERENT_CONTROL_DIMS =
    ::testing::Types<TestDynamicsParams<1>, TestDynamicsParams<2>, TestDynamicsParams<3>, TestDynamicsParams<4>>;

TYPED_TEST_SUITE(TestNoise, DIFFERENT_CONTROL_DIMS);

TYPED_TEST(TestNoise, WhiteNoise)
{
  int full_buffer_size = this->NUM_ROLLOUTS * this->NUM_TIMESTEPS * this->CONTROL_DIM;
  float* colored_noise_output = new float[full_buffer_size]{ 0 };

  auto sampler_params = this->sampler->getParams();
  for (int i = 0; i < this->CONTROL_DIM; i++)
  {
    sampler_params.exponents[i] = 0.0;
    sampler_params.std_dev[i] = 1.0;
  }
  this->sampler->setParams(sampler_params);

  this->sampler->generateSamples(0, 0, *(this->gen), false);
  HANDLE_ERROR(cudaMemcpyAsync(colored_noise_output, this->sampler->getControlSample(0, 0, 0),
                               sizeof(float) * full_buffer_size, cudaMemcpyDeviceToHost, this->stream));
  HANDLE_ERROR(cudaStreamSynchronize(this->stream));

  std::vector<int> num_within_std_dev(3, 0);
  // Ignore first rollout as that will be all zeros
  for (int i = this->NUM_TIMESTEPS * this->CONTROL_DIM; i < full_buffer_size; i++)
  {
    for (int j = 0; j < num_within_std_dev.size(); j++)
    {
      if (fabsf(colored_noise_output[i]) < j + 1.0)
      {
        num_within_std_dev[j]++;
        break;
      }
    }
  }

  float perc_within_n_std_dev[num_within_std_dev.size()];
  // Percentages from https://en.wikipedia.org/wiki/68%E2%80%9395%E2%80%9399.7_rule
  float known_percentages[3] = { 0.6827, 0.9545, 0.9973 };
  for (int i = 0; i < num_within_std_dev.size(); i++)
  {
    perc_within_n_std_dev[i] =
        std::accumulate(num_within_std_dev.begin(), num_within_std_dev.begin() + i + 1, 0.0) / full_buffer_size;
    assert_float_rel_near(known_percentages[i], perc_within_n_std_dev[i], 0.001);
  }
  delete[] colored_noise_output;
}

TYPED_TEST(TestNoise, MultiNoise)
{
  int full_buffer_size = this->NUM_ROLLOUTS * this->NUM_TIMESTEPS * this->CONTROL_DIM;
  float* colored_noise_output = new float[full_buffer_size]{ 0 };

  auto sampler_params = this->sampler->getParams();
  for (int i = 0; i < this->CONTROL_DIM; i++)
  {
    sampler_params.exponents[i] = i;
    sampler_params.std_dev[i] = 1.0;
  }
  this->sampler->setParams(sampler_params);

  this->sampler->generateSamples(0, 0, *(this->gen), false);
  HANDLE_ERROR(cudaMemcpyAsync(colored_noise_output, this->sampler->getControlSample(0, 0, 0),
                               sizeof(float) * full_buffer_size, cudaMemcpyDeviceToHost, this->stream));
  HANDLE_ERROR(cudaStreamSynchronize(this->stream));

  const int num_std_devs = 3;
  // std::vector<int> num_within_std_dev(3, 0);
  std::vector<std::array<int, num_std_devs>> control_std_dev_count;
  for (int i = 0; i < this->CONTROL_DIM; i++)
  {
    std::array<int, num_std_devs> std_dev_count_i = { 0, 0, 0 };
    control_std_dev_count.push_back(std_dev_count_i);
  }

  for (int n = 1; n < this->NUM_ROLLOUTS; n++)
  {
    for (int t = 0; t < this->NUM_TIMESTEPS; t++)
    {
      for (int i = 0; i < this->CONTROL_DIM; i++)
      {
        const int sample_idx = (n * this->NUM_TIMESTEPS + t) * this->CONTROL_DIM + i;
        for (int j = 0; j < num_std_devs; j++)
        {
          if (fabsf(colored_noise_output[sample_idx]) < j + 1.0)
          {
            control_std_dev_count[i][j]++;
            break;
          }
        }
      }
    }
  }

  float perc_within_n_std_dev[num_std_devs];
  // Percentages from https://en.wikipedia.org/wiki/68%E2%80%9395%E2%80%9399.7_rule
  float known_percentages[num_std_devs] = { 0.6827, 0.9545, 0.9973 };
  float proper_count = (this->NUM_ROLLOUTS - 1) * this->NUM_TIMESTEPS;
  for (int i = 0; i < num_std_devs; i++)
  {
    perc_within_n_std_dev[i] =
        std::accumulate(control_std_dev_count[0].begin(), control_std_dev_count[0].begin() + i + 1, 0.0) / proper_count;
    assert_float_rel_near(known_percentages[i], perc_within_n_std_dev[i], 0.001);
  }
  if (this->CONTROL_DIM > 1)
  {
    for (int i = 0; i < num_std_devs; i++)
    {
      perc_within_n_std_dev[i] =
          std::accumulate(control_std_dev_count[1].begin(), control_std_dev_count[1].begin() + i + 1, 0.0) /
          proper_count;
      std::cout << "Control 1 values within " << i + 1 << " std dev: " << perc_within_n_std_dev[i] << std::endl;
    }
  }
  delete[] colored_noise_output;
}

TEST(ColoredNoise, DISABLED_checkWhiteNoise)
{
  int NUM_TIMESTEPS = 50000;
  int NUM_ROLLOUTS = 1;
  int CONTROL_DIM = 1;
  std::vector<float> exponents(CONTROL_DIM, 0.0);
  int full_buffer_size = NUM_ROLLOUTS * NUM_TIMESTEPS * CONTROL_DIM;
  float* colored_noise_d;
  float colored_noise_output[full_buffer_size] = { 0 };
  HANDLE_ERROR(cudaMalloc((void**)&colored_noise_d, sizeof(float) * full_buffer_size));
  cudaStream_t stream;
  curandGenerator_t gen;
  cudaStreamCreate(&stream);
  curandCreateGenerator(&gen, CURAND_RNG_PSEUDO_DEFAULT);
  curandSetPseudoRandomGeneratorSeed(gen, 42);
  curandSetStream(gen, stream);

  powerlaw_psd_gaussian(exponents, NUM_TIMESTEPS, NUM_ROLLOUTS, colored_noise_d, 0, gen, 0.0f, stream);
  HANDLE_ERROR(cudaMemcpyAsync(colored_noise_output, colored_noise_d, sizeof(float) * full_buffer_size,
                               cudaMemcpyDeviceToHost, stream));
  HANDLE_ERROR(cudaStreamSynchronize(stream));

  // Check percentages for 3 standard deviations
  std::vector<int> num_within_std_dev(3, 0);
  for (int i = 0; i < full_buffer_size; i++)
  {
    for (int j = 0; j < num_within_std_dev.size(); j++)
    {
      if (fabsf(colored_noise_output[i]) < j + 1.0)
      {
        num_within_std_dev[j]++;
        break;
      }
    }
  }

  float perc_within_n_std_dev[num_within_std_dev.size()];
  // Percentages from https://en.wikipedia.org/wiki/68%E2%80%9395%E2%80%9399.7_rule
  float known_percentages[3] = { 0.6827, 0.9545, 0.9973 };
  for (int i = 0; i < num_within_std_dev.size(); i++)
  {
    perc_within_n_std_dev[i] =
        std::accumulate(num_within_std_dev.begin(), num_within_std_dev.begin() + i + 1, 0.0) / full_buffer_size;
    assert_float_rel_near(known_percentages[i], perc_within_n_std_dev[i], 0.001);
  }
}

TEST(ColoredNoise, DISABLED_checkPinkNoise)
{
  int NUM_TIMESTEPS = 50000;
  int NUM_ROLLOUTS = 1;
  int CONTROL_DIM = 1;
  std::vector<float> exponents(CONTROL_DIM, 1.0);
  int full_buffer_size = NUM_ROLLOUTS * NUM_TIMESTEPS * CONTROL_DIM;
  float* colored_noise_d;
  float colored_noise_output[full_buffer_size] = { 0 };
  HANDLE_ERROR(cudaMalloc((void**)&colored_noise_d, sizeof(float) * full_buffer_size));
  cudaStream_t stream;
  curandGenerator_t gen;
  cudaStreamCreate(&stream);
  curandCreateGenerator(&gen, CURAND_RNG_PSEUDO_DEFAULT);
  curandSetPseudoRandomGeneratorSeed(gen, 42);
  curandSetStream(gen, stream);

  powerlaw_psd_gaussian(exponents, NUM_TIMESTEPS, NUM_ROLLOUTS, colored_noise_d, 0, gen, 0.0f, stream);
  HANDLE_ERROR(cudaMemcpyAsync(colored_noise_output, colored_noise_d, sizeof(float) * full_buffer_size,
                               cudaMemcpyDeviceToHost, stream));
  HANDLE_ERROR(cudaStreamSynchronize(stream));
  int within_std_dev = 0;
  int within_2_std_dev = 0;
  for (int i = 0; i < full_buffer_size; i++)
  {
    if (fabsf(colored_noise_output[i]) < 1.0)
    {
      within_std_dev++;
    }
    else if (fabsf(colored_noise_output[i]) < 2.0)
    {
      within_2_std_dev++;
    }
  }
  float perc_one_std_dev = (float)within_std_dev / full_buffer_size;
  float perc_two_std_dev = (float)(within_std_dev + within_2_std_dev) / full_buffer_size;
  std::cout << "Percentage within 1 std dev: " << 100 * perc_one_std_dev << std::endl;
  std::cout << "Percentage within 2 std dev: " << 100 * perc_two_std_dev << std::endl;
  // assert_float_rel_near(0.6827, perc_one_std_dev, 0.001);
  // assert_float_rel_near(0.9545, perc_two_std_dev, 0.001);
}

TEST(ColoredNoise, DISABLED_checkRedNoise)
{
  int NUM_TIMESTEPS = 50000;
  int NUM_ROLLOUTS = 1;
  int CONTROL_DIM = 1;
  std::vector<float> exponents(CONTROL_DIM, 2.0);
  int full_buffer_size = NUM_ROLLOUTS * NUM_TIMESTEPS * CONTROL_DIM;
  float* colored_noise_d;
  float colored_noise_output[full_buffer_size] = { 0 };
  HANDLE_ERROR(cudaMalloc((void**)&colored_noise_d, sizeof(float) * full_buffer_size));
  cudaStream_t stream;
  curandGenerator_t gen;
  cudaStreamCreate(&stream);
  curandCreateGenerator(&gen, CURAND_RNG_PSEUDO_DEFAULT);
  curandSetPseudoRandomGeneratorSeed(gen, 42);
  curandSetStream(gen, stream);

  powerlaw_psd_gaussian(exponents, NUM_TIMESTEPS, NUM_ROLLOUTS, colored_noise_d, 0, gen, 0.0f, stream);
  HANDLE_ERROR(cudaMemcpyAsync(colored_noise_output, colored_noise_d, sizeof(float) * full_buffer_size,
                               cudaMemcpyDeviceToHost, stream));
  HANDLE_ERROR(cudaStreamSynchronize(stream));

  // Check percentages for 3 standard deviations
  std::vector<int> num_within_std_dev(3, 0);
  for (int i = 0; i < full_buffer_size; i++)
  {
    for (int j = 0; j < num_within_std_dev.size(); j++)
    {
      if (fabsf(colored_noise_output[i]) < j + 1.0)
      {
        num_within_std_dev[j]++;
        break;
      }
    }
  }

  float perc_within_n_std_dev[num_within_std_dev.size()];
  // Percentages from https://en.wikipedia.org/wiki/68%E2%80%9395%E2%80%9399.7_rule
  float known_percentages[3] = { 0.6827, 0.9545, 0.9973 };
  for (int i = 0; i < num_within_std_dev.size(); i++)
  {
    perc_within_n_std_dev[i] =
        std::accumulate(num_within_std_dev.begin(), num_within_std_dev.begin() + i + 1, 0.0) / full_buffer_size;
    std::cout << "Percentage within " << i + 1 << " std dev: " << 100 * perc_within_n_std_dev[i] << std::endl;
    // assert_float_rel_near(known_percentages[i], perc_within_n_std_dev[i], 0.001);
  }
}

TEST(ColoredNoise, DISABLED_checkMultiNoise)
{
  int NUM_TIMESTEPS = 6000;
  int NUM_ROLLOUTS = 50;
  int CONTROL_DIM = 3;
  std::vector<float> exponents(CONTROL_DIM, 0.0);
  exponents[1] = 0.5;
  exponents[2] = 2.0;
  // exponents[3] = 1.25;
  // exponents[4] = 0.75;
  int full_buffer_size = NUM_ROLLOUTS * NUM_TIMESTEPS * CONTROL_DIM;
  float* colored_noise_d;
  float colored_noise_output[full_buffer_size] = { 0 };
  HANDLE_ERROR(cudaMalloc((void**)&colored_noise_d, sizeof(float) * full_buffer_size));
  cudaStream_t stream;
  curandGenerator_t gen;
  cudaStreamCreate(&stream);
  curandCreateGenerator(&gen, CURAND_RNG_PSEUDO_DEFAULT);
  curandSetPseudoRandomGeneratorSeed(gen, 42);
  curandSetStream(gen, stream);

  powerlaw_psd_gaussian(exponents, NUM_TIMESTEPS, NUM_ROLLOUTS, colored_noise_d, 0, gen, 0.0f, stream);
  HANDLE_ERROR(cudaMemcpyAsync(colored_noise_output, colored_noise_d, sizeof(float) * full_buffer_size,
                               cudaMemcpyDeviceToHost, stream));
  HANDLE_ERROR(cudaStreamSynchronize(stream));

  // Check percentages for 3 standard deviations
  std::vector<int> num_within_std_dev(3, 0);
  float perc_within_n_std_dev[num_within_std_dev.size()];
  // Percentages from https://en.wikipedia.org/wiki/68%E2%80%9395%E2%80%9399.7_rule
  float known_percentages[3] = { 0.6827, 0.9545, 0.9973 };
  for (int c = 0; c < CONTROL_DIM; c++)
  {
    std::fill(num_within_std_dev.begin(), num_within_std_dev.end(), 0);
    for (int i = 0; i < NUM_TIMESTEPS * NUM_ROLLOUTS; i++)
    {
      for (int j = 0; j < num_within_std_dev.size(); j++)
      {
        if (fabsf(colored_noise_output[i * CONTROL_DIM + c]) < j + 1.0)
        {
          num_within_std_dev[j]++;
          break;
        }
      }
    }

    for (int i = 0; i < num_within_std_dev.size(); i++)
    {
      perc_within_n_std_dev[i] = std::accumulate(num_within_std_dev.begin(), num_within_std_dev.begin() + i + 1, 0.0) /
                                 (NUM_ROLLOUTS * NUM_TIMESTEPS);
      std::cout << "Colored Noise " << exponents[c] << " ";
      std::cout << "percent of samples within " << i + 1 << " std dev: " << 100 * perc_within_n_std_dev[i] << std::endl;
      if (exponents[c] == 0)
      {
        assert_float_rel_near(known_percentages[i], perc_within_n_std_dev[i], 0.001);
      }
    }
  }
}
