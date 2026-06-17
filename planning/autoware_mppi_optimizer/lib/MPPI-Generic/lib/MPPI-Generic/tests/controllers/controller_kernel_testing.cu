#include <mppi/controllers/MPPI/mppi_controller.cuh>
#include <mppi/controllers/ColoredMPPI/colored_mppi_controller.cuh>
#include <mppi/controllers/Tube-MPPI/tube_mppi_controller.cuh>
#include <mppi/controllers/R-MPPI/robust_mppi_controller.cuh>
#include <mppi/dynamics/double_integrator/di_dynamics.cuh>
#include <mppi/cost_functions/cost.cuh>
#include <mppi/cost_functions/double_integrator/double_integrator_circle_cost.cuh>
#include <mppi/feedback_controllers/DDP/ddp.cuh>

class DoubleIntegratorDummyCost
  : public Cost<DoubleIntegratorDummyCost, DoubleIntegratorCircleCostParams, DoubleIntegratorParams>
{
public:
  DoubleIntegratorDummyCost(cudaStream_t stream = 0)
  {
    bindToStream(stream);
  }

  __device__ float computeStateCost(float* s, int timestep, float* theta_c, int* crash_status)
  {
    float cost = SQ(s[0]);
    for (int i = 0; i < 100; i++)
    {
      cost = abs(cos(cost));
    }
    return cost;
  }

  float computeStateCost(const Eigen::Ref<const output_array> s, int timestep, int* crash_status)
  {
    float cost = SQ(s[0]);
    for (int i = 0; i < 100; i++)
    {
      cost = abs(cos(cost));
    }
    return cost;
  }

  float terminalCost(const Eigen::Ref<const output_array> s)
  {
    return 0;
  }

  __device__ float terminalCost(float* s, float* theta_c)
  {
    return 0;
  }
};

#include <gtest/gtest.h>

const int MAX_TIMESTEPS = 200;
using DI_FEEDBACK_T = DDPFeedback<DoubleIntegratorDynamics, MAX_TIMESTEPS>;

template <int NUM_ROLLOUTS>
using DI_Vanilla = VanillaMPPIController<DoubleIntegratorDynamics, DoubleIntegratorDummyCost, DI_FEEDBACK_T,
                                         MAX_TIMESTEPS, NUM_ROLLOUTS>;

template <int NUM_ROLLOUTS>
using DI_Colored = ColoredMPPIController<DoubleIntegratorDynamics, DoubleIntegratorDummyCost, DI_FEEDBACK_T,
                                         MAX_TIMESTEPS, NUM_ROLLOUTS>;

template <int NUM_ROLLOUTS>
using DI_Tube =
    TubeMPPIController<DoubleIntegratorDynamics, DoubleIntegratorDummyCost, DI_FEEDBACK_T, MAX_TIMESTEPS, NUM_ROLLOUTS>;

template <int NUM_ROLLOUTS>
using DI_Robust = RobustMPPIController<DoubleIntegratorDynamics, DoubleIntegratorDummyCost, DI_FEEDBACK_T,
                                       MAX_TIMESTEPS, NUM_ROLLOUTS>;

// TODO: Add more dynamics/cost function specializations

template <class CONTROLLER_T>
class ControllerKernelChoiceTest : public ::testing::Test
{
public:
  using DYN_T = typename CONTROLLER_T::TEMPLATED_DYNAMICS;
  using COST_T = typename CONTROLLER_T::TEMPLATED_COSTS;
  using FB_T = typename CONTROLLER_T::TEMPLATED_FEEDBACK;
  using SAMPLER_T = typename CONTROLLER_T::TEMPLATED_SAMPLING;
  using CONTROLLER_PARAMS_T = typename CONTROLLER_T::TEMPLATED_PARAMS;
  using SAMPLER_PARAMS_T = typename SAMPLER_T::SAMPLING_PARAMS_T;

  DYN_T* model = nullptr;
  COST_T* cost = nullptr;
  SAMPLER_T* sampler = nullptr;
  FB_T* fb_controller = nullptr;
  std::shared_ptr<CONTROLLER_T> controller;
  mppi::util::MPPILoggerPtr logger = std::make_shared<mppi::util::MPPILogger>(mppi::util::LOG_LEVEL::DEBUG);

  void SetUp() override
  {
    model = new DYN_T();
    cost = new COST_T();
    fb_controller = new FB_T(model, dt);

    SAMPLER_PARAMS_T sampler_params;
    for (int i = 0; i < DYN_T::CONTROL_DIM; i++)
    {
      sampler_params.std_dev[i] = std_dev;
    }
    sampler = new SAMPLER_T(sampler_params);

    HANDLE_ERROR(cudaStreamCreate(&stream));

    CONTROLLER_PARAMS_T controller_params;
    setUpControllerParams(controller_params);
    controller = std::make_shared<CONTROLLER_T>(model, cost, fb_controller, sampler, controller_params, stream);
    controller->setLogger(logger);
    controller->setLogLevel(mppi::util::LOG_LEVEL::WARNING);
  }

  void setUpControllerParams(CONTROLLER_PARAMS_T& params)
  {
    params.dt_ = dt;
    params.num_timesteps_ = num_timesteps;
    params.dynamics_rollout_dim_ = rollout_dyn;
    params.cost_rollout_dim_ = rollout_cost;
  }

  void TearDown() override
  {
    controller.reset();
    delete fb_controller;
    delete sampler;
    delete model;
    delete cost;
    HANDLE_ERROR(cudaStreamSynchronize(stream));
  }

  cudaStream_t stream;

protected:
  const int num_timesteps = 150;
  float dt = 0.01;
  // float lambda = 1.0;
  // float alpha = 0.0;
  float std_dev = 2.0;

  dim3 rollout_dyn = dim3(64, DYN_T::STATE_DIM, 1);
  dim3 rollout_cost = dim3(96, 1, 1);
  dim3 eval_dyn = dim3(64, DYN_T::STATE_DIM, 1);
  dim3 eval_cost = dim3(96, 1, 1);
};

template <int NUM_ROLLOUTS>
using DIFFERENT_CONTROLLERS =
    ::testing::Types<DI_Vanilla<NUM_ROLLOUTS>, DI_Colored<NUM_ROLLOUTS>, DI_Tube<NUM_ROLLOUTS>, DI_Robust<NUM_ROLLOUTS>,
                     DI_Vanilla<NUM_ROLLOUTS * 4>, DI_Colored<NUM_ROLLOUTS * 4>, DI_Tube<NUM_ROLLOUTS * 4>,
                     DI_Robust<NUM_ROLLOUTS * 4>, DI_Vanilla<NUM_ROLLOUTS * 16>, DI_Colored<NUM_ROLLOUTS * 16>,
                     DI_Tube<NUM_ROLLOUTS * 16>, DI_Robust<NUM_ROLLOUTS * 16>, DI_Vanilla<NUM_ROLLOUTS * 64>,
                     DI_Colored<NUM_ROLLOUTS * 64>, DI_Tube<NUM_ROLLOUTS * 64>, DI_Robust<NUM_ROLLOUTS * 64>>;

TYPED_TEST_SUITE(ControllerKernelChoiceTest, DIFFERENT_CONTROLLERS<128>);

TYPED_TEST(ControllerKernelChoiceTest, CheckAppropriateKernelSelection)
{
  const int further_evaluations = 20;

  auto empty_state = this->model->getZeroState();
  this->controller->setLogLevel(mppi::util::LOG_LEVEL::DEBUG);
  this->controller->chooseAppropriateKernel();
  auto auto_kernel_choice = this->controller->getKernelChoiceAsEnum();

  // Start testing single kernel
  this->controller->setKernelChoice(kernelType::USE_SINGLE_KERNEL);
  auto start_single_kernel_time = std::chrono::steady_clock::now();
  for (int i = 0; i < further_evaluations; i++)
  {
    this->controller->computeControl(empty_state, 1);
  }
  auto end_single_kernel_time = std::chrono::steady_clock::now();

  // Start testing split kernel
  this->controller->setKernelChoice(kernelType::USE_SPLIT_KERNELS);
  auto start_split_kernel_time = std::chrono::steady_clock::now();
  for (int i = 0; i < further_evaluations; i++)
  {
    this->controller->computeControl(empty_state, 1);
  }
  auto end_split_kernel_time = std::chrono::steady_clock::now();

  float single_kernel_duration = mppi::math::timeDiffms(end_single_kernel_time, start_single_kernel_time);
  float split_kernel_duration = mppi::math::timeDiffms(end_split_kernel_time, start_split_kernel_time);

  if (fabsf(single_kernel_duration - split_kernel_duration) < 1.0f)
  {  // the kernels are within 1 ms of each other
    this->logger->info("Durations of both kernels too close to determine winner: split = %f ms, single = %f ms\n",
                       split_kernel_duration, single_kernel_duration);
  }
  else
  {
    ASSERT_EQ(split_kernel_duration <= single_kernel_duration, auto_kernel_choice == kernelType::USE_SPLIT_KERNELS)
        << "chooseAppropriateKernel() did not choose the faster kernel, single: " << single_kernel_duration
        << " ms, split: " << split_kernel_duration;
  }
}

TYPED_TEST(ControllerKernelChoiceTest, KernelChoiceThroughSharedMemCheck)
{
  auto controller_params = this->controller->getParams();
  // set Rollout Cost dim so high as to ensure the kernel would run out of shared mem if used
  controller_params.cost_rollout_dim_.x = 320000;
  this->controller->setParams(controller_params);
  this->controller->chooseAppropriateKernel();
  ASSERT_EQ(this->controller->getKernelChoiceAsEnum(), kernelType::USE_SINGLE_KERNEL);
}

TYPED_TEST(ControllerKernelChoiceTest, NoUsableKernelCheck)
{
  auto controller_params = this->controller->getParams();
  // set Rollout Cost dim so high as to ensure the kernel would run out of shared mem if used
  controller_params.cost_rollout_dim_.x = 320000;
  controller_params.dynamics_rollout_dim_.x = 320000;
  this->controller->setParams(controller_params);
  EXPECT_ANY_THROW({ this->controller->chooseAppropriateKernel(); });
}

TYPED_TEST(ControllerKernelChoiceTest, MoreEvaluationsDoNotAdjustChoice)
{
  int num_evaluations = this->controller->getNumKernelEvaluations();
  this->controller->setLogLevel(mppi::util::LOG_LEVEL::DEBUG);
  this->controller->chooseAppropriateKernel();
  auto curr_select_kernel = this->controller->getKernelChoiceAsEnum();

  float prev_num_eval_time = 0;

  for (int i = 0; i < 5; i++)
  {
    this->controller->setNumKernelEvaluations(num_evaluations);
    auto start_num_eval_time = std::chrono::steady_clock::now();
    this->controller->chooseAppropriateKernel();
    auto end_num_eval_time = std::chrono::steady_clock::now();
    float num_eval_time = mppi::math::timeDiffms(end_num_eval_time, start_num_eval_time);
    ASSERT_EQ(this->controller->getKernelChoiceAsEnum(), curr_select_kernel);
    if (i != 0)
    {
      ASSERT_TRUE(prev_num_eval_time < num_eval_time);
    }
    prev_num_eval_time = num_eval_time;
    num_evaluations *= 2;
  }
}
