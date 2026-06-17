#include <gtest/gtest.h>

#include <mppi/utils/test_helper.h>
#include <random>
#include <algorithm>
#include <math.h>
#include <cmath>

#include <mppi/utils/activation_functions.cuh>
#include <mppi/utils/gpu_err_chk.cuh>
#include "mppi/utils/math_utils.h"

class ActivationFunctionTest : public testing::Test
{
protected:
  void SetUp() override
  {
    generator = std::default_random_engine(7.0);
    distribution = std::normal_distribution<float>(0.0, 2.0);
  }

  void TearDown() override
  {
  }

  std::default_random_engine generator;
  std::normal_distribution<float> distribution;
};

TEST_F(ActivationFunctionTest, TanhCPU)
{
  for (int i = 0; i < 1e5; i++)
  {
    float num = distribution(generator);
    EXPECT_FLOAT_EQ(mppi::nn::tanh(num), std::tanh(num));
  }
}

__global__ void tanhTestKernel(float* input, int num, int times)
{
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid < num)
  {
    for (int i = 0; i < times; i++)
    {
      input[tid] = mppi::nn::tanh(input[tid]);
    }
  }
}

__global__ void tanhStableTestKernel(float* input, int num, int times)
{
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid < num)
  {
    for (int i = 0; i < times; i++)
    {
      input[tid] = mppi::nn::tanh_accurate(input[tid]);
    }
  }
}

template <int BLOCKDIM_X = 128>
float launchTanhTestKernel(std::vector<float>& input, int times = 100)
{
  float* input_d;
  float* input_stable_d;
  int count = input.size();
  HANDLE_ERROR(cudaMalloc((void**)&input_d, sizeof(float) * count));
  HANDLE_ERROR(cudaMalloc((void**)&input_stable_d, sizeof(float) * count));
  HANDLE_ERROR(cudaMemcpy(input_d, input.data(), sizeof(float) * count, cudaMemcpyHostToDevice));
  HANDLE_ERROR(cudaMemcpy(input_stable_d, input.data(), sizeof(float) * count, cudaMemcpyHostToDevice));

  const int gridsize_x = (count - 1) / BLOCKDIM_X + 1;
  dim3 threadsPerBlock(BLOCKDIM_X);
  dim3 numBlocks(gridsize_x, 1);

  cudaStream_t stream;
  HANDLE_ERROR(cudaStreamCreate(&stream));

  cudaEvent_t start, stop;
  float time_stable, time_fast;
  cudaEventCreate(&start);
  cudaEventCreate(&stop);
  std::cout << "\n===== TANH ======\n";

  cudaEventRecord(start, stream);
  tanhTestKernel<<<numBlocks, threadsPerBlock, 0, stream>>>(input_d, count, times);
  cudaEventRecord(stop, stream);
  cudaEventSynchronize(stop);
  cudaEventElapsedTime(&time_fast, start, stop);
  std::cout << "time to compute fast " << time_fast << std::endl;

  cudaEventRecord(start, stream);
  tanhStableTestKernel<<<numBlocks, threadsPerBlock, 0, stream>>>(input_stable_d, count, times);
  cudaEventRecord(stop, stream);
  cudaEventSynchronize(stop);
  cudaEventElapsedTime(&time_stable, start, stop);
  std::cout << "time to compute stable " << time_stable << std::endl;

  std::cout << "time difference: " << time_stable - time_fast << std::endl;

  HANDLE_ERROR(cudaMemcpy(input.data(), input_d, sizeof(float) * count, cudaMemcpyDeviceToHost));
  cudaFree(input_d);
  cudaFree(input_stable_d);

  return time_stable - time_fast;
}

TEST_F(ActivationFunctionTest, TanhGPU)
{
  std::vector<float> vec(1e8);
  for (int i = 0; i < vec.size(); i++)
  {
    vec[i] = distribution(generator);
  }
  std::vector<float> output_vec = vec;
  launchTanhTestKernel(output_vec, 1);
  for (int i = 0; i < vec.size(); i++)
  {
    EXPECT_NEAR(output_vec[i], std::tanh(vec[i]), 2.0e-7);
  }

  float average = 0;
  for (int i = 0; i < 10; i++)
  {
    average += launchTanhTestKernel(output_vec);
  }
  average /= 10.0f;
  std::cout << "got average diff " << average << std::endl;
  // below condition is kind of a janky hardcode, might change on different systems
  EXPECT_LT(std::abs(average), 3);
  EXPECT_LT(average, 3);  // in ms
  EXPECT_GT(average, -3);
}

TEST_F(ActivationFunctionTest, SigmoidCPU)
{
  for (int i = 0; i < 1e5; i++)
  {
    float num = distribution(generator);
    EXPECT_FLOAT_EQ(mppi::nn::sigmoid(num), (1.0f / (1.0f + std::exp(-num))));
  }
}

__global__ void sigmoidTestKernel(float* input, int num, int times)
{
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid < num)
  {
    for (int i = 0; i < times; i++)
    {
      input[tid] = mppi::nn::sigmoid(input[tid]);
    }
  }
}

__global__ void sigmoidStableTestKernel(float* input, int num, int times)
{
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid < num)
  {
    for (int i = 0; i < times; i++)
    {
      input[tid] = mppi::nn::sigmoid_accurate(input[tid]);
    }
  }
}

template <int BLOCKDIM_X = 128>
float launchSigmoidTestKernel(std::vector<float>& input, int times = 100)
{
  float* input_d;
  float* input_stable_d;
  int count = input.size();
  HANDLE_ERROR(cudaMalloc((void**)&input_d, sizeof(float) * count));
  HANDLE_ERROR(cudaMalloc((void**)&input_stable_d, sizeof(float) * count));
  HANDLE_ERROR(cudaMemcpy(input_d, input.data(), sizeof(float) * count, cudaMemcpyHostToDevice));
  HANDLE_ERROR(cudaMemcpy(input_stable_d, input.data(), sizeof(float) * count, cudaMemcpyHostToDevice));

  const int gridsize_x = (count - 1) / BLOCKDIM_X + 1;
  dim3 threadsPerBlock(BLOCKDIM_X);
  dim3 numBlocks(gridsize_x, 1);

  cudaStream_t stream;
  HANDLE_ERROR(cudaStreamCreate(&stream));

  cudaEvent_t start, stop;
  float time_fast, time_stable;
  cudaEventCreate(&start);
  cudaEventCreate(&stop);
  std::cout << "\n===== SIGMOID ======\n";

  cudaEventRecord(start, stream);
  sigmoidTestKernel<<<numBlocks, threadsPerBlock, 0, stream>>>(input_d, count, times);
  cudaEventRecord(stop, stream);
  cudaEventSynchronize(stop);
  cudaEventElapsedTime(&time_fast, start, stop);
  std::cout << "time to compute fast " << time_fast << std::endl;

  cudaEventRecord(start, stream);
  sigmoidStableTestKernel<<<numBlocks, threadsPerBlock, 0, stream>>>(input_stable_d, count, times);
  cudaEventRecord(stop, stream);
  cudaEventSynchronize(stop);
  cudaEventElapsedTime(&time_stable, start, stop);
  std::cout << "time to compute stable " << time_stable << std::endl;

  std::cout << "time difference: " << time_stable - time_fast << std::endl;

  HANDLE_ERROR(cudaMemcpy(input.data(), input_d, sizeof(float) * count, cudaMemcpyDeviceToHost));
  cudaFree(input_d);
  cudaFree(input_stable_d);

  return time_stable - time_fast;
}

TEST_F(ActivationFunctionTest, SigmoidGPU)
{
  std::vector<float> vec(1e8);
  for (int i = 0; i < vec.size(); i++)
  {
    vec[i] = distribution(generator);
  }
  std::vector<float> output_vec = vec;
  launchSigmoidTestKernel(output_vec, 1);
  for (int i = 0; i < vec.size(); i++)
  {
    EXPECT_NEAR(output_vec[i], (1.0f / (1.0f + std::exp(-vec[i]))), 2.0e-7);
  }

  float average = 0.0f;
  for (int i = 0; i < 10; i++)
  {
    average += launchSigmoidTestKernel(output_vec);
  }
  average /= 10.0f;
  std::cout << "got average difference " << average << std::endl;
  EXPECT_GT(average, 0.0f);
  // below condition is kind of a janky hardcode, might change on different systems
  // if should still be faster to use the faster version, so number should be positive
  EXPECT_GT(average, 1.0f);  // in ms
}
