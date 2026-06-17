#include <gtest/gtest.h>

#include <kernel_tests/cost_functions/cost_generic_kernel_tests.cuh>
#include <mppi/cost_functions/cartpole/cartpole_quadratic_cost.cuh>
#include <mppi/cost_functions/double_integrator/double_integrator_circle_cost.cuh>
#include <mppi/cost_functions/quadrotor/quadrotor_quadratic_cost.cuh>

template <class COST_T>
__global__ void copyParamsOnGPU(COST_T* cost_d, typename COST_T::COST_PARAMS_T* params_d)
{
  *params_d = cost_d->getParams();
}

template <class COST_T>
class GeneralCostTest : public ::testing::Test
{
public:
  COST_T cost;

protected:
  void SetUp() override
  {
  }

  void TearDown() override
  {
  }
};

using DIFFERENT_COSTS = ::testing::Types<DoubleIntegratorCircleCost, CartpoleQuadraticCost>;

TYPED_TEST_SUITE(GeneralCostTest, DIFFERENT_COSTS);

TYPED_TEST(GeneralCostTest, GPUSetup)
{
  ASSERT_EQ(this->cost.cost_d_, nullptr);
  this->cost.GPUSetup();
  ASSERT_NE(this->cost.cost_d_, nullptr);
}

TYPED_TEST(GeneralCostTest, VerifyGPUParams)
{
  using PARAMS_T = typename TypeParam::COST_PARAMS_T;
  this->cost.GPUSetup();

  PARAMS_T* params_d;
  HANDLE_ERROR(cudaMalloc((void**)&params_d, sizeof(PARAMS_T)));

  PARAMS_T params_gpu_after, params_gpu_before;
  copyParamsOnGPU<TypeParam><<<1, 1>>>(this->cost.cost_d_, params_d);
  HANDLE_ERROR(cudaMemcpy(&params_gpu_before, params_d, sizeof(PARAMS_T), cudaMemcpyDeviceToHost));

  PARAMS_T cost_params = this->cost.getParams();
  cost_params.control_cost_coeff[0] = 27;
  cost_params.discount = 0.97;
  this->cost.setParams(cost_params);
  copyParamsOnGPU<TypeParam><<<1, 1>>>(this->cost.cost_d_, params_d);
  HANDLE_ERROR(cudaMemcpy(&params_gpu_after, params_d, sizeof(PARAMS_T), cudaMemcpyDeviceToHost));

  // Ensure params before params update are different
  ASSERT_NE(params_gpu_before.discount, cost_params.discount);
  ASSERT_NE(params_gpu_before.control_cost_coeff[0], cost_params.control_cost_coeff[0]);
  // Ensure params after params update are the same
  ASSERT_EQ(params_gpu_after.discount, cost_params.discount);
  ASSERT_EQ(params_gpu_after.control_cost_coeff[0], cost_params.control_cost_coeff[0]);
}

TYPED_TEST(GeneralCostTest, CPUvsGPURolloutCost)
{
  checkGPURolloutCost<TypeParam>(this->cost, 0.01);
}
