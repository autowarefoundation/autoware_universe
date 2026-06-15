#include <gtest/gtest.h>
#include <mppi/utils/cuda_math_utils.cuh>
#include <mppi/utils/gpu_err_chk.cuh>

#include <random>

template <class T = float2>
__global__ void VectorVectorAddTestKernel(const T* __restrict__ input1, const T* __restrict__ input2,
                                          T* __restrict__ output2)
{
  *output2 = *input1 + *input2;
}

template <class T = float2>
__global__ void VectorVectorSubTestKernel(const T* __restrict__ input1, const T* __restrict__ input2,
                                          T* __restrict__ output2)
{
  *output2 = *input1 - *input2;
}

template <class T = float2>
__global__ void VectorVectorMultTestKernel(const T* __restrict__ input1, const T* __restrict__ input2,
                                           T* __restrict__ output2)
{
  *output2 = (*input1) * (*input2);
}

template <class T = float2>
__global__ void VectorVectorDivTestKernel(const T* __restrict__ input1, const T* __restrict__ input2,
                                          T* __restrict__ output2)
{
  *output2 = (*input1) / (*input2);
}

template <class T = float2, class S = float>
__global__ void VectorScalarAddTestKernel(const T* __restrict__ input1, const S* __restrict__ input2,
                                          T* __restrict__ output2)
{
  *output2 = *input1 + *input2;
}

template <class T = float2, class S = float>
__global__ void VectorScalarMultTestKernel(const T* __restrict__ input1, const S* __restrict__ input2,
                                           T* __restrict__ output2)
{
  *output2 = (*input1) * (*input2);
}

template <class T = float2, class S = float>
__global__ void VectorScalarSubTestKernel(const T* __restrict__ input1, const S* __restrict__ input2,
                                          T* __restrict__ output2)
{
  *output2 = *input1 - *input2;
}

template <class T = float2, class S = float>
__global__ void VectorScalarDivTestKernel(const T* __restrict__ input1, const S* __restrict__ input2,
                                          T* __restrict__ output2)
{
  *output2 = (*input1) / (*input2);
}

template <class T = float2, class S = float>
__global__ void VectorVectorScalarAddMultTestKernel(const T* __restrict__ input1, const T* __restrict__ input2,
                                                    const S* __restrict__ scalar, T* __restrict__ output2)
{
  *output2 = *input1 + (*input2) * (*scalar);
}

template <class T = float2>
class CudaFloatStructsTests : public ::testing::Test
{
public:
  T input1_cpu;
  T input2_cpu;
  T output_cpu;
  T output_gpu;
  T* input1_d;
  T* input2_d;
  T* output_d;

  float scalar;
  float* scalar_d;

  void SetUp() override
  {
    HANDLE_ERROR(cudaMalloc((void**)&scalar_d, sizeof(float)));
    HANDLE_ERROR(cudaMalloc((void**)&input1_d, sizeof(T)));
    HANDLE_ERROR(cudaMalloc((void**)&input2_d, sizeof(T)));
    HANDLE_ERROR(cudaMalloc((void**)&output_d, sizeof(T)));

    // Setup random number generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dist(0.0f, 10.f);

    initializeStruct(input1_cpu, gen, dist);
    initializeStruct(input2_cpu, gen, dist);
    HANDLE_ERROR(cudaMemcpy(input1_d, &input1_cpu, sizeof(T), cudaMemcpyHostToDevice));
    HANDLE_ERROR(cudaMemcpy(input2_d, &input2_cpu, sizeof(T), cudaMemcpyHostToDevice));
    HANDLE_ERROR(cudaMemcpy(scalar_d, &scalar, sizeof(float), cudaMemcpyHostToDevice));
  }

  void initializeStruct(T& val, std::mt19937& gen, std::uniform_real_distribution<float>& dist)
  {
    std::cout << "Did not specialize!" << std::endl;
  }

  T addVec(const T& val1, const T& val2)
  {
    T a;
    std::cout << "Wrong addVec!" << std::endl;
    return a;
  }

  T addScalar(const T& val1, const float& val2)
  {
    T a;
    std::cout << "Wrong addScalar!" << std::endl;
    return a;
  }

  T subVec(const T& val1, const T& val2)
  {
    T a;
    std::cout << "Wrong subVec!" << std::endl;
    return a;
  }

  T subScalar(const T& val1, const float& val2)
  {
    T a;
    std::cout << "Wrong subScalar!" << std::endl;
    return a;
  }

  T multVec(const T& val1, const T& val2)
  {
    T a;
    std::cout << "Wrong multVec!" << std::endl;
    return a;
  }

  T multScalar(const T& val1, const float& val2)
  {
    T a;
    std::cout << "Wrong multScalar!" << std::endl;
    return a;
  }

  T divVec(const T& val1, const T& val2)
  {
    T a;
    std::cout << "Wrong divVec!" << std::endl;
    return a;
  }

  T divScalar(const T& val1, const float& val2)
  {
    T a;
    std::cout << "Wrong divScalar!" << std::endl;
    return a;
  }

  bool assert_same(const T& val1, const T& val2, const std::string& str = "");

  void TearDown() override
  {
    HANDLE_ERROR(cudaFree(scalar_d));
    HANDLE_ERROR(cudaFree(input1_d));
    HANDLE_ERROR(cudaFree(input2_d));
    HANDLE_ERROR(cudaFree(output_d));
  }
};

template <>
void CudaFloatStructsTests<float2>::initializeStruct(float2& val, std::mt19937& gen,
                                                     std::uniform_real_distribution<float>& dist)
{
  val = make_float2(dist(gen), dist(gen));
}

template <>
bool CudaFloatStructsTests<float2>::assert_same(const float2& val1, const float2& val2, const std::string& str)
{
  bool result = true;
  result = result && (val1.x == val2.x);
  result = result && (val1.y == val2.y);
  if (!result)
  {
    printf("(%f, %f) != (%f, %f)", val1.x, val1.y, val2.x, val2.y);
    std::cout << str << std::endl;
  }
  return result;
}

template <>
float2 CudaFloatStructsTests<float2>::addVec(const float2& val1, const float2& val2)
{
  float2 output;
  output.x = val1.x + val2.x;
  output.y = val1.y + val2.y;
  return output;
}

template <>
float2 CudaFloatStructsTests<float2>::multVec(const float2& val1, const float2& val2)
{
  float2 output;
  output.x = val1.x * val2.x;
  output.y = val1.y * val2.y;
  return output;
}

template <>
float2 CudaFloatStructsTests<float2>::addScalar(const float2& val1, const float& val2)
{
  float2 output;
  output.x = val1.x + val2;
  output.y = val1.y + val2;
  return output;
}

template <>
float2 CudaFloatStructsTests<float2>::multScalar(const float2& val1, const float& val2)
{
  float2 output;
  output.x = val1.x * val2;
  output.y = val1.y * val2;
  return output;
}

template <>
float2 CudaFloatStructsTests<float2>::subVec(const float2& val1, const float2& val2)
{
  float2 output;
  output.x = val1.x - val2.x;
  output.y = val1.y - val2.y;
  return output;
}

template <>
float2 CudaFloatStructsTests<float2>::divVec(const float2& val1, const float2& val2)
{
  float2 output;
  output.x = val1.x / val2.x;
  output.y = val1.y / val2.y;
  return output;
}

template <>
float2 CudaFloatStructsTests<float2>::subScalar(const float2& val1, const float& val2)
{
  float2 output;
  output.x = val1.x - val2;
  output.y = val1.y - val2;
  return output;
}

template <>
float2 CudaFloatStructsTests<float2>::divScalar(const float2& val1, const float& val2)
{
  float2 output;
  output.x = val1.x / val2;
  output.y = val1.y / val2;
  return output;
}

template <>
void CudaFloatStructsTests<float4>::initializeStruct(float4& val, std::mt19937& gen,
                                                     std::uniform_real_distribution<float>& dist)
{
  val = make_float4(dist(gen), dist(gen), dist(gen), dist(gen));
}

template <>
bool CudaFloatStructsTests<float4>::assert_same(const float4& val1, const float4& val2, const std::string& str)
{
  bool result = true;
  result = result && (val1.x == val2.x);
  result = result && (val1.y == val2.y);
  result = result && (val1.z == val2.z);
  result = result && (val1.w == val2.w);
  if (!result)
  {
    printf("(%f, %f, %f, %f) != (%f, %f, %f, %f)", val1.x, val1.y, val1.z, val1.w, val2.x, val2.y, val2.z, val2.w);
    std::cout << str << std::endl;
  }
  return result;
}

template <>
float4 CudaFloatStructsTests<float4>::addVec(const float4& val1, const float4& val2)
{
  float4 output;
  output.x = val1.x + val2.x;
  output.y = val1.y + val2.y;
  output.z = val1.z + val2.z;
  output.w = val1.w + val2.w;
  return output;
}

template <>
float4 CudaFloatStructsTests<float4>::multVec(const float4& val1, const float4& val2)
{
  float4 output;
  output.x = val1.x * val2.x;
  output.y = val1.y * val2.y;
  output.z = val1.z * val2.z;
  output.w = val1.w * val2.w;
  return output;
}

template <>
float4 CudaFloatStructsTests<float4>::addScalar(const float4& val1, const float& val2)
{
  float4 output;
  output.x = val1.x + val2;
  output.y = val1.y + val2;
  output.z = val1.z + val2;
  output.w = val1.w + val2;
  return output;
}

template <>
float4 CudaFloatStructsTests<float4>::multScalar(const float4& val1, const float& val2)
{
  float4 output;
  output.x = val1.x * val2;
  output.y = val1.y * val2;
  output.z = val1.z * val2;
  output.w = val1.w * val2;
  return output;
}

template <>
float4 CudaFloatStructsTests<float4>::subVec(const float4& val1, const float4& val2)
{
  float4 output;
  output.x = val1.x - val2.x;
  output.y = val1.y - val2.y;
  output.z = val1.z - val2.z;
  output.w = val1.w - val2.w;
  return output;
}

template <>
float4 CudaFloatStructsTests<float4>::divVec(const float4& val1, const float4& val2)
{
  float4 output;
  output.x = val1.x / val2.x;
  output.y = val1.y / val2.y;
  output.z = val1.z / val2.z;
  output.w = val1.w / val2.w;
  return output;
}

template <>
float4 CudaFloatStructsTests<float4>::subScalar(const float4& val1, const float& val2)
{
  float4 output;
  output.x = val1.x - val2;
  output.y = val1.y - val2;
  output.z = val1.z - val2;
  output.w = val1.w - val2;
  return output;
}

template <>
float4 CudaFloatStructsTests<float4>::divScalar(const float4& val1, const float& val2)
{
  float4 output;
  output.x = val1.x / val2;
  output.y = val1.y / val2;
  output.z = val1.z / val2;
  output.w = val1.w / val2;
  return output;
}

template <>
void CudaFloatStructsTests<float3>::initializeStruct(float3& val, std::mt19937& gen,
                                                     std::uniform_real_distribution<float>& dist)
{
  val = make_float3(dist(gen), dist(gen), dist(gen));
}

template <>
bool CudaFloatStructsTests<float3>::assert_same(const float3& val1, const float3& val2, const std::string& str)
{
  bool result = true;
  result = result && (val1.x == val2.x);
  result = result && (val1.y == val2.y);
  result = result && (val1.z == val2.z);
  if (!result)
  {
    printf("(%f, %f, %f) != (%f, %f, %f)", val1.x, val1.y, val1.z, val2.x, val2.y, val2.z);
    std::cout << str << std::endl;
  }
  return result;
}

template <>
float3 CudaFloatStructsTests<float3>::addVec(const float3& val1, const float3& val2)
{
  float3 output;
  output.x = val1.x + val2.x;
  output.y = val1.y + val2.y;
  output.z = val1.z + val2.z;
  return output;
}

template <>
float3 CudaFloatStructsTests<float3>::multVec(const float3& val1, const float3& val2)
{
  float3 output;
  output.x = val1.x * val2.x;
  output.y = val1.y * val2.y;
  output.z = val1.z * val2.z;
  return output;
}

template <>
float3 CudaFloatStructsTests<float3>::addScalar(const float3& val1, const float& val2)
{
  float3 output;
  output.x = val1.x + val2;
  output.y = val1.y + val2;
  output.z = val1.z + val2;
  return output;
}

template <>
float3 CudaFloatStructsTests<float3>::multScalar(const float3& val1, const float& val2)
{
  float3 output;
  output.x = val1.x * val2;
  output.y = val1.y * val2;
  output.z = val1.z * val2;
  return output;
}

template <>
float3 CudaFloatStructsTests<float3>::subVec(const float3& val1, const float3& val2)
{
  float3 output;
  output.x = val1.x - val2.x;
  output.y = val1.y - val2.y;
  output.z = val1.z - val2.z;
  return output;
}

template <>
float3 CudaFloatStructsTests<float3>::divVec(const float3& val1, const float3& val2)
{
  float3 output;
  output.x = val1.x / val2.x;
  output.y = val1.y / val2.y;
  output.z = val1.z / val2.z;
  return output;
}

template <>
float3 CudaFloatStructsTests<float3>::subScalar(const float3& val1, const float& val2)
{
  float3 output;
  output.x = val1.x - val2;
  output.y = val1.y - val2;
  output.z = val1.z - val2;
  return output;
}

template <>
float3 CudaFloatStructsTests<float3>::divScalar(const float3& val1, const float& val2)
{
  float3 output;
  output.x = val1.x / val2;
  output.y = val1.y / val2;
  output.z = val1.z / val2;
  return output;
}

using TYPE_TESTS = ::testing::Types<float2, float3, float4>;

TYPED_TEST_SUITE(CudaFloatStructsTests, TYPE_TESTS);

TYPED_TEST(CudaFloatStructsTests, VecAddScalar)
{
  using T = TypeParam;
  T ground_truth_output = this->addScalar(this->input1_cpu, this->scalar);

  this->output_cpu = this->input1_cpu + this->scalar;
  std::string cpu_fail = "Doesn't pass cpu test";
  ASSERT_TRUE(this->assert_same(ground_truth_output, this->output_cpu)) << cpu_fail;

  // Parallelization tests
  T zero = this->output_gpu;
  for (int blocks = 1; blocks < 10; blocks++)
  {
    for (int threads = 1; threads < 128; threads++)
    {
      HANDLE_ERROR(cudaMemcpy(this->output_d, &zero, sizeof(T), cudaMemcpyHostToDevice));
      VectorScalarAddTestKernel<T, float><<<blocks, threads>>>(this->input1_d, this->scalar_d, this->output_d);
      HANDLE_ERROR(cudaMemcpy(&this->output_gpu, this->output_d, sizeof(T), cudaMemcpyDeviceToHost));
      std::string fail_gpu =
          " failed with " + std::to_string(blocks) + " blocks of " + std::to_string(threads) + " threads";
      ASSERT_TRUE(this->assert_same(ground_truth_output, this->output_gpu, fail_gpu)) << fail_gpu;
    }
  }
}

TYPED_TEST(CudaFloatStructsTests, VecSubScalar)
{
  using T = TypeParam;
  T ground_truth_output = this->subScalar(this->input1_cpu, this->scalar);

  this->output_cpu = this->input1_cpu - this->scalar;
  std::string cpu_fail = "Doesn't pass cpu test";
  ASSERT_TRUE(this->assert_same(ground_truth_output, this->output_cpu)) << cpu_fail;

  // Parallelization tests
  T zero = this->output_gpu;
  for (int blocks = 1; blocks < 10; blocks++)
  {
    for (int threads = 1; threads < 128; threads++)
    {
      HANDLE_ERROR(cudaMemcpy(this->output_d, &zero, sizeof(T), cudaMemcpyHostToDevice));
      VectorScalarSubTestKernel<T, float><<<blocks, threads>>>(this->input1_d, this->scalar_d, this->output_d);
      HANDLE_ERROR(cudaMemcpy(&this->output_gpu, this->output_d, sizeof(T), cudaMemcpyDeviceToHost));
      std::string fail_gpu =
          " failed with " + std::to_string(blocks) + " blocks of " + std::to_string(threads) + " threads";
      ASSERT_TRUE(this->assert_same(ground_truth_output, this->output_gpu, fail_gpu)) << fail_gpu;
    }
  }
}

TYPED_TEST(CudaFloatStructsTests, VecMultScalar)
{
  using T = TypeParam;
  T ground_truth_output = this->multScalar(this->input1_cpu, this->scalar);

  this->output_cpu = this->input1_cpu * this->scalar;
  std::string cpu_fail = "Doesn't pass cpu test";
  ASSERT_TRUE(this->assert_same(ground_truth_output, this->output_cpu)) << cpu_fail;

  // Parallelization tests
  T zero = this->output_gpu;
  for (int blocks = 1; blocks < 10; blocks++)
  {
    for (int threads = 1; threads < 128; threads++)
    {
      HANDLE_ERROR(cudaMemcpy(this->output_d, &zero, sizeof(T), cudaMemcpyHostToDevice));
      VectorScalarMultTestKernel<T, float><<<blocks, threads>>>(this->input1_d, this->scalar_d, this->output_d);
      HANDLE_ERROR(cudaMemcpy(&this->output_gpu, this->output_d, sizeof(T), cudaMemcpyDeviceToHost));
      std::string fail_gpu =
          " failed with " + std::to_string(blocks) + " blocks of " + std::to_string(threads) + " threads";
      ASSERT_TRUE(this->assert_same(ground_truth_output, this->output_gpu, fail_gpu)) << fail_gpu;
    }
  }
}

TYPED_TEST(CudaFloatStructsTests, VecDivScalar)
{
  using T = TypeParam;
  T ground_truth_output = this->divScalar(this->input1_cpu, this->scalar);

  this->output_cpu = this->input1_cpu / this->scalar;
  std::string cpu_fail = "Doesn't pass cpu test";
  ASSERT_TRUE(this->assert_same(ground_truth_output, this->output_cpu)) << cpu_fail;

  // Parallelization tests
  T zero = this->output_gpu;
  for (int blocks = 1; blocks < 10; blocks++)
  {
    for (int threads = 1; threads < 128; threads++)
    {
      HANDLE_ERROR(cudaMemcpy(this->output_d, &zero, sizeof(T), cudaMemcpyHostToDevice));
      VectorScalarDivTestKernel<T, float><<<blocks, threads>>>(this->input1_d, this->scalar_d, this->output_d);
      HANDLE_ERROR(cudaMemcpy(&this->output_gpu, this->output_d, sizeof(T), cudaMemcpyDeviceToHost));
      std::string fail_gpu =
          " failed with " + std::to_string(blocks) + " blocks of " + std::to_string(threads) + " threads";
      ASSERT_TRUE(this->assert_same(ground_truth_output, this->output_gpu, fail_gpu)) << fail_gpu;
    }
  }
}

TYPED_TEST(CudaFloatStructsTests, VecAddVec)
{
  using T = TypeParam;
  T ground_truth_output = this->addVec(this->input1_cpu, this->input2_cpu);

  this->output_cpu = this->input1_cpu + this->input2_cpu;
  std::string cpu_fail = "Doesn't pass cpu test";
  ASSERT_TRUE(this->assert_same(ground_truth_output, this->output_cpu)) << cpu_fail;

  // Parallelization tests
  T zero = this->output_gpu;
  for (int blocks = 1; blocks < 10; blocks++)
  {
    for (int threads = 1; threads < 128; threads++)
    {
      HANDLE_ERROR(cudaMemcpy(this->output_d, &zero, sizeof(T), cudaMemcpyHostToDevice));
      VectorVectorAddTestKernel<T><<<blocks, threads>>>(this->input1_d, this->input2_d, this->output_d);
      HANDLE_ERROR(cudaMemcpy(&this->output_gpu, this->output_d, sizeof(T), cudaMemcpyDeviceToHost));
      std::string fail_gpu =
          " failed with " + std::to_string(blocks) + " blocks of " + std::to_string(threads) + " threads";
      ASSERT_TRUE(this->assert_same(ground_truth_output, this->output_gpu, fail_gpu)) << fail_gpu;
    }
  }
}

TYPED_TEST(CudaFloatStructsTests, VecSubVec)
{
  using T = TypeParam;
  T ground_truth_output = this->subVec(this->input1_cpu, this->input2_cpu);

  this->output_cpu = this->input1_cpu - this->input2_cpu;
  std::string cpu_fail = "Doesn't pass cpu test";
  ASSERT_TRUE(this->assert_same(ground_truth_output, this->output_cpu)) << cpu_fail;

  // Parallelization tests
  T zero = this->output_gpu;
  for (int blocks = 1; blocks < 10; blocks++)
  {
    for (int threads = 1; threads < 128; threads++)
    {
      HANDLE_ERROR(cudaMemcpy(this->output_d, &zero, sizeof(T), cudaMemcpyHostToDevice));
      VectorVectorSubTestKernel<T><<<blocks, threads>>>(this->input1_d, this->input2_d, this->output_d);
      HANDLE_ERROR(cudaMemcpy(&this->output_gpu, this->output_d, sizeof(T), cudaMemcpyDeviceToHost));
      std::string fail_gpu =
          " failed with " + std::to_string(blocks) + " blocks of " + std::to_string(threads) + " threads";
      ASSERT_TRUE(this->assert_same(ground_truth_output, this->output_gpu, fail_gpu)) << fail_gpu;
    }
  }
}

TYPED_TEST(CudaFloatStructsTests, VecDivVec)
{
  using T = TypeParam;
  T ground_truth_output = this->divVec(this->input1_cpu, this->input2_cpu);

  this->output_cpu = this->input1_cpu / this->input2_cpu;
  std::string cpu_fail = "Doesn't pass cpu test";
  ASSERT_TRUE(this->assert_same(ground_truth_output, this->output_cpu)) << cpu_fail;

  // Parallelization tests
  T zero = this->output_gpu;
  for (int blocks = 1; blocks < 10; blocks++)
  {
    for (int threads = 1; threads < 128; threads++)
    {
      HANDLE_ERROR(cudaMemcpy(this->output_d, &zero, sizeof(T), cudaMemcpyHostToDevice));
      VectorVectorDivTestKernel<T><<<blocks, threads>>>(this->input1_d, this->input2_d, this->output_d);
      HANDLE_ERROR(cudaMemcpy(&this->output_gpu, this->output_d, sizeof(T), cudaMemcpyDeviceToHost));
      std::string fail_gpu =
          " failed with " + std::to_string(blocks) + " blocks of " + std::to_string(threads) + " threads";
      ASSERT_TRUE(this->assert_same(ground_truth_output, this->output_gpu, fail_gpu)) << fail_gpu;
    }
  }
}

TYPED_TEST(CudaFloatStructsTests, VecAddVecMultScalar)
{
  using T = TypeParam;
  T ground_truth_output = this->addVec(this->input1_cpu, this->multScalar(this->input2_cpu, this->scalar));

  this->output_cpu = this->input1_cpu + this->input2_cpu * this->scalar;
  std::string cpu_fail = "Doesn't pass cpu test";
  ASSERT_TRUE(this->assert_same(ground_truth_output, this->output_cpu)) << cpu_fail;

  // Parallelization tests
  T zero = this->output_gpu;
  for (int blocks = 1; blocks < 10; blocks++)
  {
    for (int threads = 1; threads < 128; threads++)
    {
      HANDLE_ERROR(cudaMemcpy(this->output_d, &zero, sizeof(T), cudaMemcpyHostToDevice));
      VectorVectorScalarAddMultTestKernel<T, float>
          <<<blocks, threads>>>(this->input1_d, this->input2_d, this->scalar_d, this->output_d);
      HANDLE_ERROR(cudaMemcpy(&this->output_gpu, this->output_d, sizeof(T), cudaMemcpyDeviceToHost));
      std::string fail_gpu =
          " failed with " + std::to_string(blocks) + " blocks of " + std::to_string(threads) + " threads";
      ASSERT_TRUE(this->assert_same(ground_truth_output, this->output_gpu, fail_gpu)) << fail_gpu;
    }
  }
}
