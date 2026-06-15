#include <gtest/gtest.h>
// #include <mppi/ddp/ddp_model_wrapper.h>
// #include <mppi/ddp/ddp_tracking_costs.h>
// #include <mppi/ddp/ddp.h>
#include <mppi/instantiations/cartpole_mppi/cartpole_mppi.cuh>
#include <mppi/instantiations/quadrotor_mppi/quadrotor_mppi.cuh>

class ModelWrapper_Test : public testing::Test
{
public:
  CartpoleDynamics model = CartpoleDynamics(1, 1, 1);
  std::shared_ptr<ModelWrapperDDP<CartpoleDynamics>> ddp_model =
      std::make_shared<ModelWrapperDDP<CartpoleDynamics>>(&model);

  CartpoleDynamics::state_array state;
  CartpoleDynamics::control_array control;
};

TEST_F(ModelWrapper_Test, StateDerivative_1)
{
  CartpoleDynamics::state_array result;
  CartpoleDynamics::state_array known_result;

  state << 0, 0, 0, 0;
  control << 0;
  result = ddp_model->f(state, control);
  known_result << 0, 0, 0, 0;

  ASSERT_EQ(known_result, result);
}

TEST_F(ModelWrapper_Test, StateDerivative_2)
{
  CartpoleDynamics::state_array result;
  CartpoleDynamics::state_array known_result;

  state << 1, 2, 3, 4;
  control << 5;
  result = ddp_model->f(state, control);
  model.computeStateDeriv(state, control, known_result);

  ASSERT_EQ(known_result, result);
}

TEST_F(ModelWrapper_Test, Jacobian_1)
{
  CartpoleDynamics::Jacobian result;
  CartpoleDynamics::Jacobian known_result;

  CartpoleDynamics::dfdx A = CartpoleDynamics::dfdx::Zero();
  CartpoleDynamics::dfdu B = CartpoleDynamics::dfdu::Zero();

  state << 1, 2, 3, 4;
  control << 5;

  model.computeGrad(state, control, A, B);

  known_result.leftCols<CartpoleDynamics::STATE_DIM>() = A;
  known_result.rightCols<CartpoleDynamics::CONTROL_DIM>() = B;
  result = ddp_model->df(state, control);

  ASSERT_EQ(known_result, result);
}

class TrackingCosts_Test : public testing::Test
{
public:
  typedef Eigen::Matrix<float, CartpoleDynamics::CONTROL_DIM, CartpoleDynamics::CONTROL_DIM> square_u_matrix;
  typedef Eigen::Matrix<float, CartpoleDynamics::STATE_DIM + CartpoleDynamics::CONTROL_DIM,
                        CartpoleDynamics::STATE_DIM + CartpoleDynamics::CONTROL_DIM>
      square_xu_matrix;
  CartpoleDynamics::dfdx Q;
  square_u_matrix R;
  square_xu_matrix QR;
  int num_timesteps;

  std::shared_ptr<TrackingCostDDP<ModelWrapperDDP<CartpoleDynamics>>> tracking_cost;
  std::shared_ptr<TrackingTerminalCost<ModelWrapperDDP<CartpoleDynamics>>> terminal_cost;

  CartpoleDynamics::state_array state;
  CartpoleDynamics::control_array control;

protected:
  void SetUp() override
  {
    Q = CartpoleDynamics::dfdx::Identity();
    R = square_u_matrix::Identity();
    QR = square_xu_matrix::Identity();
    num_timesteps = 100;

    tracking_cost = std::make_shared<TrackingCostDDP<ModelWrapperDDP<CartpoleDynamics>>>(Q, R, num_timesteps);
    terminal_cost = std::make_shared<TrackingTerminalCost<ModelWrapperDDP<CartpoleDynamics>>>(Q);

    state.resize(CartpoleDynamics::STATE_DIM, 1);
    control.resize(CartpoleDynamics::CONTROL_DIM, 1);
  }
};

TEST_F(TrackingCosts_Test, ComputeCost)
{
  state << 1, 2, 3, 4;
  control << 5;
  int timestep = 0;
  ASSERT_FLOAT_EQ(1 + 4 + 9 + 16 + 25, tracking_cost->c(state, control, timestep));
  ASSERT_FLOAT_EQ(1 + 4 + 9 + 16, terminal_cost->c(state));
}

TEST_F(TrackingCosts_Test, ComputeCostGradient)
{
  Eigen::Matrix<float, 5, 1> known_result;
  state << 1, 2, 3, 4;
  control << 5;
  known_result.block(0, 0, 4, 1) = state;
  known_result.block(4, 0, 1, 1) = control;
  //  std::cout << known_result << std::endl;
  int timestep = 0;
  ASSERT_EQ(QR * known_result, tracking_cost->dc(state, control, timestep))
      << "Known:\n"
      << QR * known_result << "\nTracking cost: \n"
      << tracking_cost->dc(state, control, timestep);
  ASSERT_EQ(Q * state, terminal_cost->dc(state)) << "Known:\n"
                                                 << Q * state << "\nTerminal cost: \n"
                                                 << terminal_cost->dc(state);
}

TEST_F(TrackingCosts_Test, ComputeCostHessian)
{
  state << 1, 2, 3, 4;
  control << 5;
  int timestep = 0;
  ASSERT_EQ(QR, tracking_cost->d2c(state, control, timestep));
  ASSERT_EQ(Q, terminal_cost->d2c(state));
}

TEST(DDPSolver_Test, Cartpole_Tracking)
{
  using SAMPLER_T = mppi::sampling_distributions::GaussianDistribution<typename CartpoleDynamics::DYN_PARAMS_T>;
  // SETUP THE MPPI CONTROLLER
  CartpoleDynamics model = CartpoleDynamics(1.0, 1.0, 1.0);
  CartpoleQuadraticCost cost;

  CartpoleQuadraticCostParams new_params;
  new_params.cart_position_coeff = 100;
  new_params.pole_angle_coeff = 200;
  new_params.cart_velocity_coeff = 10;
  new_params.pole_angular_velocity_coeff = 20;
  new_params.control_cost_coeff[0] = 1;
  new_params.terminal_cost_coeff = 0;
  new_params.desired_terminal_state[0] = 0;
  new_params.desired_terminal_state[1] = 0;
  new_params.desired_terminal_state[2] = M_PI;
  new_params.desired_terminal_state[3] = 0;

  cost.setParams(new_params);

  float dt = 0.01;
  int max_iter = 100;
  float lambda = 0.25;
  float alpha = 0.001;
  const int num_timesteps = 100;
  auto fb_controller = DDPFeedback<CartpoleDynamics, num_timesteps>(&model, dt);

  auto sampler = SAMPLER_T();
  auto sampler_params = sampler.getParams();
  for (int i = 0; i < CartpoleDynamics::CONTROL_DIM; i++)
  {
    sampler_params.std_dev[i] = 5.0;
  }
  sampler.setParams(sampler_params);

  DDPParams<CartpoleDynamics> fb_params;
  fb_params.Q = 100 *
                CartpoleDynamics::dfdx::
                    Identity();  // Eigen::MatrixXf::Identity(CartpoleDynamics::STATE_DIM,CartpoleDynamics::STATE_DIM);
  fb_params.R = TrackingCosts_Test::square_u_matrix::
      Identity();  // Eigen::MatrixXf::Identity(CartpoleDynamics::CONTROL_DIM,CartpoleDynamics::CONTROL_DIM);
  fb_params.Q_f = fb_params.Q;
  fb_params.num_iterations = 20;
  fb_controller.setParams(fb_params);
  fb_controller.initTrackingController();

  auto controller = VanillaMPPIController<CartpoleDynamics, CartpoleQuadraticCost,
                                          DDPFeedback<CartpoleDynamics, num_timesteps>, num_timesteps, 2048, SAMPLER_T>(
      &model, &cost, &fb_controller, &sampler, dt, max_iter, lambda, alpha);

  auto controller_params = controller.getParams();
  controller_params.dynamics_rollout_dim_ = dim3(64, 8, 1);
  controller_params.cost_rollout_dim_ = dim3(num_timesteps, 1, 1);
  controller.setParams(controller_params);
  CartpoleDynamics::state_array current_state = model.getZeroState();
  // Compute the control
  controller.computeControl(current_state, 0);
  auto nominal_state = controller.getTargetStateSeq();
  auto nominal_control = controller.getControlSeq();
  // END MPPI CONTROLLER

  // util::DefaultLogger logger;
  // bool verbose = false;
  // int num_iterations = 20;

  // Eigen::MatrixXf Q;
  // Eigen::MatrixXf R;
  // Eigen::MatrixXf QR;
  // Q = 100*Eigen::MatrixXf::Identity(CartpoleDynamics::STATE_DIM,CartpoleDynamics::STATE_DIM);
  // R = Eigen::MatrixXf::Identity(CartpoleDynamics::CONTROL_DIM,CartpoleDynamics::CONTROL_DIM);
  // QR = Eigen::MatrixXf(CartpoleDynamics::STATE_DIM+ CartpoleDynamics::CONTROL_DIM,
  //                      CartpoleDynamics::STATE_DIM + CartpoleDynamics::CONTROL_DIM);
  // QR.template topLeftCorner<CartpoleDynamics::STATE_DIM, CartpoleDynamics::STATE_DIM>() = Q;
  // QR.template bottomRightCorner<CartpoleDynamics::CONTROL_DIM, CartpoleDynamics::CONTROL_DIM>() = R;

  // std::shared_ptr<ModelWrapperDDP<CartpoleDynamics>> ddp_model =
  // std::make_shared<ModelWrapperDDP<CartpoleDynamics>>(&model);

  // std::shared_ptr<DDP<ModelWrapperDDP<CartpoleDynamics>>> ddp_solver_ =
  //         std::make_shared<DDP<ModelWrapperDDP<CartpoleDynamics>>>(dt, num_timesteps, num_iterations, &logger,
  //         verbose);

  // std::shared_ptr<TrackingCostDDP<ModelWrapperDDP<CartpoleDynamics>>> tracking_cost =
  //         std::make_shared<TrackingCostDDP<ModelWrapperDDP<CartpoleDynamics>>>(Q,R,num_timesteps);

  // std::shared_ptr<TrackingTerminalCost<ModelWrapperDDP<CartpoleDynamics>>> terminal_cost =
  //         std::make_shared<TrackingTerminalCost<ModelWrapperDDP<CartpoleDynamics>>>(Q);

  // tracking_cost->setTargets(nominal_state.data(), nominal_control.data(), num_timesteps);
  // terminal_cost->xf = tracking_cost->traj_target_x_.col(num_timesteps - 1);

  Eigen::Matrix<float, CartpoleDynamics::STATE_DIM, 1> s;
  s << 0.0, 0.0, 0.0, 0.0;

  Eigen::MatrixXf control_traj = Eigen::MatrixXf::Zero(CartpoleDynamics::CONTROL_DIM, num_timesteps);

  fb_controller.computeFeedback(s, nominal_state, nominal_control);

  // OptimizerResult<ModelWrapperDDP<CartpoleDynamics>> result_ = ddp_solver_->run(s, control_traj,
  //                                                                    *ddp_model, *tracking_cost, *terminal_cost);

  //  std::cout << result_.state_trajectory << std::endl;

  for (int i = 0; i < num_timesteps; ++i)
  {
    ASSERT_NEAR((nominal_state.col(i) - fb_controller.result_.state_trajectory.col(i)).norm(), 0.0f, 1e-2)
        << "Failed on timestep: " << i;
  }
}

TEST(DDPSolver_Test, Quadrotor_Tracking)
{
  const int num_timesteps = 500;

  using DYN = QuadrotorDynamics;
  using COST = QuadrotorQuadraticCost;
  using CONTROLLER = VanillaMPPIController<DYN, COST, DDPFeedback<DYN, num_timesteps>, num_timesteps, 2048>;

  std::array<float2, DYN::CONTROL_DIM> control_ranges;
  for (int i = 0; i < 3; i++)
  {
    control_ranges[i] = make_float2(-2.5, 2.5);
  }
  control_ranges[3] = make_float2(0, 36);
  DYN model(control_ranges);

  DYN::state_array x_goal;
  x_goal << 6, 4, 3,               // position
      0, 0, 0,                     // velocity
      0.7071068, 0, 0, 0.7071068,  // quaternion
      0, 0, 0;                     // angular speed

  float dt = 0.01;

  // Create DDP and find feedback gains
  util::DefaultLogger logger;
  // bool verbose = false;

  // CONTROLLER::StateCostWeight Q;
  // CONTROLLER::ControlCostWeight R;
  // CONTROLLER::Hessian Qf;
  DDPParams<DYN> fb_params;
  fb_params.Q = DDPParams<DYN>::StateCostWeight::Identity();
  fb_params.R = DDPParams<DYN>::ControlCostWeight::Identity();
  fb_params.Q_f = DDPParams<DYN>::Hessian::Identity();
  fb_params.Q.diagonal() << 25, 25, 300, 15, 15, 300, 0, 0, 0, 0, 30, 30, 30;
  fb_params.Q_f.diagonal() << 250, 250, 3000, 150, 150, 3000, 0, 0, 0, 0, 300, 300, 300;
  fb_params.R.diagonal() << 550, 550, 550, 1;
  fb_params.num_iterations = 100;

  Eigen::MatrixXf control_traj = CONTROLLER::control_trajectory::Zero();
  CONTROLLER::state_trajectory ddp_state_traj = CONTROLLER::state_trajectory::Zero();
  for (int i = 0; i < num_timesteps; i++)
  {
    ddp_state_traj.col(i) = x_goal;
    control_traj.col(i) = model.zero_control_;
  }

  auto fb_controller = DDPFeedback<DYN, num_timesteps>(&model, dt);
  fb_controller.setParams(fb_params);
  fb_controller.initTrackingController();

  // auto ddp_model = std::make_shared<ModelWrapperDDP<DYN>>(&model);

  // auto ddp_solver = std::make_shared<DDP<ModelWrapperDDP<DYN>>>(dt,
  //                                                               num_timesteps,
  //                                                               num_iterations,
  //                                                               &logger,
  //                                                               verbose);

  // auto tracking_cost =
  //     std::make_shared<TrackingCostDDP<ModelWrapperDDP<DYN>>>(Q, R, num_timesteps);

  // auto terminal_cost =
  //     std::make_shared<TrackingTerminalCost<ModelWrapperDDP<DYN>>>(Qf);

  // tracking_cost->setTargets(ddp_state_traj.data(), control_traj.data(),
  //                           num_timesteps);
  // terminal_cost->xf = tracking_cost->traj_target_x_.col(num_timesteps - 1);

  DYN::state_array x_real;
  x_real << 0, -0.5, 0,  // position
      0, 0.5, 0,         // velocity
      1, 0, 0, 0,        // quaternion
      0, 0, 0;           // angular speed

  // DYN::control_array control_min, control_max;
  // for (int i = 0; i < DYN::CONTROL_DIM; i++) {
  //   control_min(i) = control_ranges[i].x;
  //   control_max(i) = control_ranges[i].y;
  // }

  std::cout << "Starting DDP" << std::endl;
  fb_controller.computeFeedback(x_real, ddp_state_traj, control_traj);
  // OptimizerResult<ModelWrapperDDP<DYN>> result_ = ddp_solver->run(x_real,
  //                                                                 control_traj,
  //                                                                 *ddp_model,
  //                                                                 *tracking_cost,
  //                                                                 *terminal_cost,
  //                                                                 control_min,
  //                                                                 control_max);
  auto control_feedback_gains = fb_controller.result_.feedback_gain;
  std::cout << "DDP Optimal State Sequence:\n" << fb_controller.result_.state_trajectory.transpose() << std::endl;
  DYN::state_array x_deriv, x, x_nom;
  x = x_real;
  DYN::control_array u_total, fb_u;
  for (int t = 0; t < num_timesteps; ++t)
  {
    x_nom = fb_controller.result_.state_trajectory.col(t);
    fb_u = fb_controller.k(x, x_nom, t);
    u_total = fb_u;
    model.enforceConstraints(x, u_total);
    std::cout << " t = " << t * dt << ", State_diff norm: " << (x - x_goal).norm()
              << ", Control:  " << u_total.transpose() << std::endl;
    std::cout << "Feedback matrix:\n" << control_feedback_gains[t] << std::endl;
    model.computeStateDeriv(x, u_total, x_deriv);
    model.updateState(x, x_deriv, dt);
  }
  std::cout << std::fixed << std::setprecision(5) << "X_goal: " << x_goal.transpose() << "\nX_end: " << x.transpose()
            << "\ndiff in state: " << (x - x_goal).norm() << std::endl;
  EXPECT_LE((x - x_goal).norm(), 3);
}
