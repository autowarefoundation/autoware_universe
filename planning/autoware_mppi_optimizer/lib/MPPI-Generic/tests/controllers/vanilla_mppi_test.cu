#include <gtest/gtest.h>
#include <mppi/instantiations/cartpole_mppi/cartpole_mppi.cuh>
#include <mppi/instantiations/quadrotor_mppi/quadrotor_mppi.cuh>

class Cartpole_VanillaMPPI : public ::testing::Test
{
public:
  static const int NUM_TIMESTEPS = 100;
  static const int NUM_ROLLOUTS = 2048;
  using DYN_T = CartpoleDynamics;
  using COST_T = CartpoleQuadraticCost;
  using FB_T = DDPFeedback<DYN_T, NUM_TIMESTEPS>;
  using SAMPLING_T = mppi::sampling_distributions::GaussianDistribution<DYN_T::DYN_PARAMS_T>;
  using CONTROLLER_T = VanillaMPPIController<DYN_T, COST_T, FB_T, NUM_TIMESTEPS, NUM_ROLLOUTS, SAMPLING_T>;
  using control_trajectory = CONTROLLER_T::control_trajectory;
  using control_array = CONTROLLER_T::control_array;

  DYN_T model = DYN_T(1.0, 1.0, 1.0);
  COST_T cost;
  FB_T* fb_controller;
  SAMPLING_T* sampler;
  CONTROLLER_T* controller;

  float dt = 0.01;
  int max_iter = 10;
  float gamma = 0.5;
  float lambda = 0.25;
  float alpha = 0.01;
  control_trajectory init_control = control_trajectory::Constant(0);
  control_array control_std_dev = control_array::Constant(5.0);
  cudaStream_t stream;

  void SetUp() override
  {
    fb_controller = new FB_T(&model, dt);
    HANDLE_ERROR(cudaStreamCreate(&stream));
    auto sampler_params = SAMPLING_T::SAMPLING_PARAMS_T();
    for (int i = 0; i < DYN_T::CONTROL_DIM; i++)
    {
      sampler_params.std_dev[i] = control_std_dev[i];
    }
    EXPECT_EQ(model.getGrdSharedSizeBytes(), 0);
    EXPECT_EQ(model.getBlkSharedSizeBytes(), 0);
    sampler = new SAMPLING_T(sampler_params);
    controller = new CONTROLLER_T(&model, &cost, fb_controller, sampler, dt, max_iter, lambda, alpha, NUM_TIMESTEPS,
                                  init_control, stream);
  }

  void TearDown() override
  {
    delete controller;
    delete fb_controller;
    delete sampler;
  }
};

TEST_F(Cartpole_VanillaMPPI, BindToStream)
{
  EXPECT_EQ(controller->stream_, controller->model_->stream_) << "Stream bind to dynamics failure";
  EXPECT_EQ(controller->stream_, controller->cost_->stream_) << "Stream bind to cost failure";
  EXPECT_EQ(controller->stream_, controller->fb_controller_->getHostPointer()->stream_) << "Stream  bind to feedback "
                                                                                           "failure";
  EXPECT_EQ(controller->stream_, controller->sampler_->stream_) << "Stream bind to sampling distribution failure";
  HANDLE_ERROR(cudaStreamDestroy(stream));
}

TEST_F(Cartpole_VanillaMPPI, UpdateNoiseStdDev)
{
  control_array new_control_std_dev = control_array::Constant(3.5);
  auto sampler_params = sampler->getParams();
  for (int i = 0; i < DYN_T::CONTROL_DIM; i++)
  {
    sampler_params.std_dev[i] = new_control_std_dev[i];
  }
  sampler->setParams(sampler_params, true);
  EXPECT_FLOAT_EQ(new_control_std_dev[0], controller->sampler_->getParams().std_dev[0]);
}

TEST_F(Cartpole_VanillaMPPI, SwingUpTest)
{
  CartpoleQuadraticCostParams new_params;
  new_params.cart_position_coeff = 100;
  new_params.pole_angle_coeff = 200;
  new_params.cart_velocity_coeff = 10;
  new_params.pole_angular_velocity_coeff = 20;
  new_params.control_cost_coeff[0] = 1;
  new_params.terminal_cost_coeff = 0;
  new_params.desired_terminal_state[0] = -20;
  new_params.desired_terminal_state[1] = 0;
  new_params.desired_terminal_state[2] = M_PI;
  new_params.desired_terminal_state[3] = 0;

  cost.setParams(new_params);

  auto sampler_params = sampler->getParams();
  sampler_params.control_cost_coeff[0] = 1.0;
  sampler_params.pure_noise_trajectories_percentage = 0.01f;
  sampler_params.rewrite_controls_block_dim = dim3(32, 16, 1);
  sampler->setParams(sampler_params);

  auto controller_params = controller->getParams();
  max_iter = 1;
  controller_params.dynamics_rollout_dim_ = dim3(64, 4, 1);
  controller_params.cost_rollout_dim_ = dim3(64, 1, 1);
  controller_params.num_iters_ = max_iter;
  controller_params.slide_control_scale_[0] = 1.0;
  controller->setParams(controller_params);

  controller->chooseAppropriateKernel();

  DYN_T::state_array current_state = DYN_T::state_array::Zero();
  int time_horizon = 1000;

  DYN_T::state_array xdot(4, 1);

  for (int i = 0; i < time_horizon; ++i)
  {
    if (i % 50 == 0)
    {
      printf("Current Time: %f    ", i * dt);
      printf("Current Baseline Cost: %f    ", controller->getBaselineCost());
      model.printState(current_state.data());
    }
    // Compute the control
    controller->computeControl(current_state, 1);

    DYN_T::control_array control;
    control = controller->getControlSeq().block(0, 0, DYN_T::CONTROL_DIM, 1);
    // Increment the state
    model.computeStateDeriv(current_state, control, xdot);
    model.updateState(current_state, xdot, dt);

    controller->slideControlSequence(1);
  }
  EXPECT_LT(controller->getBaselineCost(), 1.0);
}

TEST_F(Cartpole_VanillaMPPI, getSampledStateTrajectoriesTest)
{
  // float dt = 0.01;
  // float max_iter = 1;
  // float lambda = 0.25;
  // float alpha = 0.01;

  // CartpoleDynamics::control_array control_std_dev = CartpoleDynamics::control_array::Constant(5.0);

  // auto fb_controller = DDPFeedback<CartpoleDynamics, 100>(&model, dt);
  // auto controller =
  //     VanillaMPPIController<CartpoleDynamics, CartpoleQuadraticCost, DDPFeedback<CartpoleDynamics, 100>, 100, 2048,
  //     64,
  //                           8>(&model, &cost, &fb_controller, dt, max_iter, lambda, alpha, control_std_dev);
  DYN_T::state_array current_state = DYN_T::state_array::Zero();
  controller->setPercentageSampledControlTrajectories(0.25);

  controller->calculateSampledStateTrajectories();
  const auto outputs = controller->getSampledOutputTrajectories();
  EXPECT_EQ(outputs.size(), 0.25 * NUM_ROLLOUTS);
}

class Quadrotor_VanillaMPPI : public ::testing::Test
{
public:
  static const int NUM_TIMESTEPS = 150;
  static const int NUM_ROLLOUTS = 2048;
  using DYN_T = QuadrotorDynamics;
  using COST_T = QuadrotorQuadraticCost;
  using FB_T = DDPFeedback<DYN_T, NUM_TIMESTEPS>;
  using SAMPLING_T = mppi::sampling_distributions::GaussianDistribution<DYN_T::DYN_PARAMS_T>;
  using CONTROLLER_T = VanillaMPPIController<DYN_T, COST_T, FB_T, NUM_TIMESTEPS, NUM_ROLLOUTS, SAMPLING_T>;
  using control_trajectory = CONTROLLER_T::control_trajectory;
  using control_array = CONTROLLER_T::control_array;

  float dt = 0.01;
  int max_iter = 1;
  float lambda = 4.0;
  float alpha = 0.9;
  cudaStream_t stream;
  control_array control_std_dev = control_array::Constant(0.5);
  control_trajectory init_control = control_trajectory::Constant(0.0);

  DYN_T model;
  COST_T cost;
  FB_T fb_controller = FB_T(&model, dt);
  SAMPLING_T* sampler;
  CONTROLLER_T* controller;
  void SetUp() override
  {
    HANDLE_ERROR(cudaStreamCreate(&stream));
    auto sampler_params = SAMPLING_T::SAMPLING_PARAMS_T();
    for (int i = 0; i < DYN_T::CONTROL_DIM; i++)
    {
      sampler_params.std_dev[i] = control_std_dev[i];
    }
    for (int i = 0; i < NUM_TIMESTEPS; i++)
    {
      init_control(3, i) = mppi::math::GRAVITY;
    }
    sampler = new SAMPLING_T(sampler_params);
    controller = new CONTROLLER_T(&model, &cost, &fb_controller, sampler, dt, max_iter, lambda, alpha, NUM_TIMESTEPS,
                                  init_control, stream);
  }

  void TearDown() override
  {
    delete controller;
    delete sampler;
  }
};

TEST_F(Quadrotor_VanillaMPPI, HoverTest)
{
  QuadrotorQuadraticCostParams new_params;
  new_params.x_goal()[2] = 1;
  new_params.x_coeff = 400;
  new_params.v_coeff = 150;
  // new_params.q_coeff = 15;
  new_params.roll_coeff = 15;
  new_params.pitch_coeff = 15;
  new_params.yaw_coeff = 15;
  new_params.w_coeff = 5;
  float acceptable_distance = 0.15;  // meters

  std::cout << "Goal: " << new_params.getDesiredState().transpose() << std::endl;

  cost.setParams(new_params);

  // int max_iter = 1;
  // float lambda = 4.0;
  // float alpha = 0.9;

  // DYN_T::control_array control_std_dev = DYN_T::control_array::Constant(0.5);
  // control_std_dev[3] = 2;
  auto sampler_params = sampler->getParams();
  sampler_params.std_dev[3] = 2.0f;
  sampler_params.rewrite_controls_block_dim = dim3(32, 16, 1);
  sampler_params.sum_strides = 32;
  sampler->setParams(sampler_params);

  // CONTROLLER::control_trajectory init_control = CONTROLLER::control_trajectory::Zero();
  // for (int i = 0; i < num_timesteps; i++)
  // {
  //   init_control(3, i) = mppi::math::GRAVITY;
  // }

  // auto controller = CONTROLLER(&model, &cost, &fb_controller, dt, max_iter, lambda, alpha, control_std_dev,
  //                              num_timesteps, init_control);
  auto controller_params = controller->getParams();
  controller_params.dynamics_rollout_dim_ = dim3(32, 4, 1);
  controller_params.cost_rollout_dim_ = dim3(32, 4, 1);
  controller->setParams(controller_params);
  controller->setDebug(true);
  controller->chooseAppropriateKernel();
  DYN_T::state_array current_state = DYN_T::state_array::Zero();
  // current_state(6) = 1;  // set q_w to 1
  current_state(S_IND_CLASS(DYN_T::DYN_PARAMS_T, QUAT_W)) = 1;  // set q_w to 1
  int time_horizon = 3000;

  // float xdot[DYN_T::STATE_DIM];
  DYN_T::state_array xdot(4, 1);
  DYN_T::control_array control;
  control = controller->getControlSeq().block(0, 0, DYN_T::CONTROL_DIM, 1);

  int far_away_cnt = 0;
  Eigen::Vector3f goal_pos = new_params.getDesiredState().block<3, 1>(0, 0);

  std::chrono::steady_clock::time_point loop_start = std::chrono::steady_clock::now();
  std::chrono::steady_clock::time_point loop_end = std::chrono::steady_clock::now();
  for (int i = 0; i < time_horizon; ++i)
  {
    if (i % 50 == 0)
    {
      printf("Current Time: %6.2f    ", i * dt);
      printf("Current Baseline Cost: %f \n", controller->getBaselineCost());
      printf("State Cost: %f\n", controller->cost_->computeStateCost(current_state));
      model.printState(current_state.data());
      std::cout << "Control: " << control.transpose() << std::endl;
      std::cout << "ComputeControl took " << mppi::math::timeDiffms(loop_end, loop_start) << " ms" << std::endl;
    }
    if (std::isnan(controller->getBaselineCost()) || control.hasNaN() || current_state.hasNaN())
    {
      printf("ENCOUNTERED A NAN!!\n");
      printf("Current Time: %f    ", i * dt);
      printf("Current Baseline Cost: %f \n", controller->getBaselineCost());
      model.printState(current_state.data());
      std::cout << "Control: " << control.transpose() << std::endl;
      FAIL();
    }

    // Compute the control
    loop_start = std::chrono::steady_clock::now();
    controller->computeControl(current_state, 1);
    loop_end = std::chrono::steady_clock::now();

    control = controller->getControlSeq().block(0, 0, DYN_T::CONTROL_DIM, 1);
    // Increment the state
    model.computeStateDeriv(current_state, control, xdot);
    model.updateState(current_state, xdot, dt);

    controller->slideControlSequence(1);
    Eigen::Vector3f pos = current_state.block<3, 1>(0, 0);
    float dist = (pos - goal_pos).norm();
    if (dist > acceptable_distance)
    {
      far_away_cnt++;
    }
  }
  float percentage_in_ball = float(far_away_cnt) / time_horizon;
  std::cout << "Amount of time outside " << acceptable_distance << " m: " << percentage_in_ball * 100 << "%"
            << std::endl;

  EXPECT_LT(percentage_in_ball, 0.1);
}
