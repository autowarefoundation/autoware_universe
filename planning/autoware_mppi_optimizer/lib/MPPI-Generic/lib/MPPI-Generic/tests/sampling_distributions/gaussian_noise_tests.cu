#include <gtest/gtest.h>
#include <mppi/core/mppi_common.cuh>
#include <mppi/dynamics/linear/linear.cuh>
#include <mppi/sampling_distributions/colored_noise/colored_noise.cuh>
#include <mppi/sampling_distributions/gaussian/gaussian.cuh>

namespace ms = mppi::sampling_distributions;

template <class SAMPLER_T>
class GaussianTests : public ::testing::Test
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
  dim3 thread_block_ = dim3(32, 1, 1);

  std::vector<int> sampled_indices_;
  std::vector<int> sampled_times_;
  std::vector<int> sampled_distributions_;
  std::vector<Eigen::MatrixXf> means_cpu_;

  void SetUp() override
  {
    cudaStreamCreate(&stream_);
    sampler_ = std::make_shared<SAMPLER_T>(stream_);
    updateSamplerSizes();
    sampler_->GPUSetup();
    HANDLE_CURAND_ERROR(curandCreateGenerator(&gen_, CURAND_RNG_PSEUDO_DEFAULT));
    HANDLE_CURAND_ERROR(curandSetPseudoRandomGeneratorSeed(gen_, 42));
  }

  void TearDown() override
  {
  }

  void updateSamplerSizes()
  {
    sampler_->setNumTimesteps(num_timesteps_);
    sampler_->setNumRollouts(num_samples_);
    sampler_->setNumDistributions(num_distributions_);
    means_cpu_.resize(num_distributions_);
  }

  void setRandomMeans()
  {
    for (int i = 0; i < num_distributions_; i++)
    {
      Eigen::MatrixXf mean_i = 5 * Eigen::MatrixXf::Random(SAMPLER_T::CONTROL_DIM, num_timesteps_);
      sampler_->copyImportanceSamplerToDevice(mean_i.data(), i, false);
      means_cpu_[i] = mean_i;
    }
  }
};

template <int C_DIM>
using GaussianParams3 = ms::GaussianParamsImpl<C_DIM, 3>;

template <class DYN_PARAMS_T>
class TestGaussianDistribution
  : public ms::GaussianDistributionImpl<TestGaussianDistribution<DYN_PARAMS_T>, GaussianParams3, DYN_PARAMS_T>
{
public:
  using PARENT_CLASS = ms::GaussianDistributionImpl<TestGaussianDistribution, GaussianParams3, DYN_PARAMS_T>;
  using SAMPLING_PARAMS_T = typename PARENT_CLASS::SAMPLING_PARAMS_T;

  TestGaussianDistribution(cudaStream_t stream = 0) : PARENT_CLASS(stream)
  {
  }
  TestGaussianDistribution(const SAMPLING_PARAMS_T& params, cudaStream_t stream = 0) : PARENT_CLASS(params, stream)
  {
  }
};

using TYPE_TESTS = ::testing::Types<ms::GaussianDistribution<LinearDynamicsParams<4, 1>>,
                                    ms::GaussianDistribution<LinearDynamicsParams<4, 2>>,
                                    TestGaussianDistribution<LinearDynamicsParams<4, 2>>>;

TYPED_TEST_SUITE(GaussianTests, TYPE_TESTS);

TYPED_TEST(GaussianTests, SetNumDistributions)
{
  using T = TypeParam;
  this->num_distributions_ = 3;
  if (this->num_distributions_ > T::SAMPLING_PARAMS_T::MAX_DISTRIBUTIONS)
  {
    EXPECT_THROW(this->sampler_->setNumDistributions(this->num_distributions_), std::out_of_range);
  }
  else
  {
    this->sampler_->setNumDistributions(this->num_distributions_);
    auto params = this->sampler_->getParams();
    EXPECT_EQ(this->num_distributions_, params.num_distributions);
  }
}

TYPED_TEST(GaussianTests, CheckSamplesAreGaussian)
{
  using T = TypeParam;
  float std_dev = 2.3f;
  auto params = this->sampler_->getParams();
  params.pure_noise_trajectories_percentage = 0.0f;
  for (int i = 0; i < this->num_distributions_; i++)
  {
    for (int j = 0; j < T::CONTROL_DIM; j++)
    {
      params.std_dev[j + i * T::CONTROL_DIM] = std_dev;
    }
  }
  this->sampler_->setParams(params);
  this->setRandomMeans();
  this->setRandomMeans();

  this->sampler_->generateSamples(0, 0, this->gen_, false);
  std::vector<float> sampled_controls(T::CONTROL_DIM * this->num_timesteps_ * this->num_samples_ *
                                      this->num_distributions_);
  HANDLE_ERROR(cudaMemcpyAsync(sampled_controls.data(), this->sampler_->getControlSample(0, 0, 0),
                               sizeof(float) * this->num_samples_ * this->num_timesteps_ * this->num_distributions_ *
                                   T::CONTROL_DIM,
                               cudaMemcpyDeviceToHost, this->stream_));
  HANDLE_ERROR(cudaStreamSynchronize(this->stream_));

  int k_bins = 3;
  std::vector<int> binned_counts(k_bins, 0);
  for (int d = 0; d < this->num_distributions_; d++)
  {
    for (int s = 0; s < this->num_samples_; s++)
    {
      for (int t = 0; t < this->num_timesteps_; t++)
      {
        for (int i = 0; i < T::CONTROL_DIM; i++)
        {
          int index = ((d * this->num_samples_ + s) * this->num_timesteps_ + t) * T::CONTROL_DIM + i;
          float z_score = (sampled_controls[index] - this->means_cpu_[d](i, t)) / std_dev;
          for (int j = k_bins; j >= 0; j--)
          {
            if (fabsf(z_score) < j + 1)
            {
              binned_counts[j]++;
            }
            else
            {
              break;
            }
          }
        }
      }
    }
  }

  // Check how many samples are properly sampled
  std::vector<double> normalized_bins(k_bins, 0.0f);
  std::vector<double> sigma_rules(k_bins);
  for (int i = 0; i < k_bins; i++)
  {
    sigma_rules[i] = 0.5 * (erf((i + 1) / sqrt(2.0)) - erf(-(i + 1) / sqrt(2.0)));
    normalized_bins[i] = (double)binned_counts[i] /
                         (this->num_distributions_ * this->num_samples_ * this->num_timesteps_ * T::CONTROL_DIM);
    double diff = fabsf(normalized_bins[i] - sigma_rules[i]) / sigma_rules[i];
    EXPECT_NEAR(diff, 0.0, 1e-3);
  }
}

TYPED_TEST(GaussianTests, CheckZeroMeanSamplesAreGaussian)
{
  using T = TypeParam;
  auto params = this->sampler_->getParams();
  params.pure_noise_trajectories_percentage = 1.0f;
  float std_dev = 2.3f;
  for (int i = 0; i < this->num_distributions_; i++)
  {
    for (int j = 0; j < T::CONTROL_DIM; j++)
    {
      params.std_dev[j + i * T::CONTROL_DIM] = std_dev;
    }
  }
  this->sampler_->setParams(params);
  this->setRandomMeans();  // should end up doing not effecting the samples

  this->sampler_->generateSamples(0, 0, this->gen_, false);
  std::vector<float> sampled_controls(T::CONTROL_DIM * this->num_timesteps_ * this->num_samples_ *
                                      this->num_distributions_);
  HANDLE_ERROR(cudaMemcpyAsync(sampled_controls.data(), this->sampler_->getControlSample(0, 0, 0),
                               sizeof(float) * this->num_samples_ * this->num_timesteps_ * this->num_distributions_ *
                                   T::CONTROL_DIM,
                               cudaMemcpyDeviceToHost, this->stream_));
  HANDLE_ERROR(cudaStreamSynchronize(this->stream_));

  int k_bins = 3;
  std::vector<int> binned_counts(k_bins, 0);
  for (int d = 0; d < this->num_distributions_; d++)
  {
    for (int s = 0; s < this->num_samples_; s++)
    {
      for (int t = 0; t < this->num_timesteps_; t++)
      {
        for (int i = 0; i < T::CONTROL_DIM; i++)
        {
          int index = ((d * this->num_samples_ + s) * this->num_timesteps_ + t) * T::CONTROL_DIM + i;
          float z_score = (sampled_controls[index]) / std_dev;
          for (int j = k_bins; j >= 0; j--)
          {
            if (fabsf(z_score) < j + 1)
            {
              binned_counts[j]++;
            }
            else
            {
              break;
            }
          }
        }
      }
    }
  }

  // Check how many samples are properly sampled
  std::vector<double> normalized_bins(k_bins, 0.0f);
  std::vector<double> sigma_rules(k_bins);
  for (int i = 0; i < k_bins; i++)
  {
    sigma_rules[i] = 0.5 * (erf((i + 1) / sqrt(2.0)) - erf(-(i + 1) / sqrt(2.0)));
    normalized_bins[i] = (double)binned_counts[i] /
                         (this->num_distributions_ * this->num_samples_ * this->num_timesteps_ * T::CONTROL_DIM);
    double diff = fabsf(normalized_bins[i] - sigma_rules[i]) / sigma_rules[i];
    EXPECT_NEAR(diff, 0.0, 1e-3);
  }
}
