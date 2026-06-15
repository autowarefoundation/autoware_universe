#include <mppi/dynamics/double_integrator/di_dynamics.cuh>
#include <mppi/cost_functions/double_integrator/double_integrator_circle_cost.cuh>
#include <mppi/cost_functions/double_integrator/double_integrator_robust_cost.cuh>
#include <mppi/controllers/MPPI/mppi_controller.cuh>
#include <mppi/controllers/Tube-MPPI/tube_mppi_controller.cuh>
#include <mppi/controllers/R-MPPI/robust_mppi_controller.cuh>
#include <mppi/feedback_controllers/DDP/ddp.cuh>

#include <cnpy.h>
#include <random>  // Used to generate random noise for control trajectories

bool tubeFailure(float* s)
{
  float inner_path_radius2 = 1.675 * 1.675;
  float outer_path_radius2 = 2.325 * 2.325;
  float radial_position = s[0] * s[0] + s[1] * s[1];
  if ((radial_position < inner_path_radius2) || (radial_position > outer_path_radius2))
  {
    return true;
  }
  else
  {
    return false;
  }
}

using Dyn = DoubleIntegratorDynamics;
using SCost = DoubleIntegratorCircleCost;
using RCost = DoubleIntegratorRobustCost;
const int num_timesteps = 50;  // Optimization time horizon
const int total_time_horizon = 5000;
using Feedback = DDPFeedback<Dyn, num_timesteps>;
using Sampler = mppi::sampling_distributions::GaussianDistribution<Dyn::DYN_PARAMS_T>;

// Problem setup
const float dt = 0.02;   // Timestep of dynamics propagation
const int max_iter = 1;  // Maximum running iterations of optimization
const float lambda = 2;  // Learning rate parameter
const float alpha = 0.0;

typedef Eigen::Matrix<float, Dyn::STATE_DIM, num_timesteps> state_trajectory;

void saveTraj(const Eigen::Ref<const state_trajectory>& traj, int t, std::vector<float>& vec)
{
  for (int i = 0; i < num_timesteps; i++)
  {
    for (int j = 0; j < Dyn::STATE_DIM; j++)
    {
      vec[t * num_timesteps * Dyn::STATE_DIM + i * Dyn::STATE_DIM + j] = traj(j, i);
    }
  }
}

void saveState(const Eigen::Ref<const Dyn::state_array>& state, int t, std::vector<float>& vec)
{
  for (int j = 0; j < Dyn::STATE_DIM; j++)
  {
    vec[t * Dyn::STATE_DIM + j] = state(j);
  }
}

void runVanilla(const Eigen::Ref<const Eigen::Matrix<float, Dyn::STATE_DIM, total_time_horizon>>& noise)
{
  // Set the initial state
  Dyn::state_array x;
  x << 2, 0, 0, 1;
  Dyn::state_array xdot;

  // control variance
  Sampler::SAMPLING_PARAMS_T sampler_params;
  for (int i = 0; i < Dyn::CONTROL_DIM; i++)
  {
    sampler_params.std_dev[i] = 1;
  }

  // Save actual trajectories, nominal_trajectory, free energy
  std::vector<float> van_trajectory(Dyn::STATE_DIM * total_time_horizon, 0);
  std::vector<float> van_nominal_traj(Dyn::STATE_DIM * num_timesteps * total_time_horizon, 0);
  std::vector<float> van_free_energy(total_time_horizon, 0);

  // Initialize the controllers
  Dyn model;
  SCost cost;
  Sampler sampler(sampler_params);
  // DDP cost parameters
  Feedback fb_controller(&model, dt);
  auto fb_params = fb_controller.getParams();
  fb_params.Q.diagonal() << 500, 500, 100, 100;
  fb_controller.setParams(fb_params);
  auto controller = VanillaMPPIController<Dyn, SCost, Feedback, num_timesteps, 1024, Sampler>(
      &model, &cost, &fb_controller, &sampler, dt, max_iter, lambda, alpha);
  auto controller_params = controller.getParams();
  controller_params.dynamics_rollout_dim_ = dim3(64, 1, 1);
  controller_params.cost_rollout_dim_ = dim3(64, 1, 1);
  controller.setParams(controller_params);
  controller.initFeedback();

  // Start the loop
  for (int t = 0; t < total_time_horizon; ++t)
  {
    /********************** Vanilla **********************/
    // Compute the control
    controller.computeControl(x, 1);

    // Compute the feedback gains
    controller.computeFeedback(x);

    // Propagate the feedback trajectory
    controller.computeFeedbackPropagatedStateSeq();

    auto nominal_trajectory = controller.getTargetStateSeq();
    auto nominal_control = controller.getControlSeq();
    auto fe_stat = controller.getFreeEnergyStatistics();

    // Save everything
    saveState(x, t, van_trajectory);
    saveTraj(nominal_trajectory, t, van_nominal_traj);
    van_free_energy[t] = fe_stat.real_sys.freeEnergyMean;

    // Get the open loop control
    DoubleIntegratorDynamics::control_array current_control = nominal_control.col(0);

    // Apply the feedback given the current state
    Dyn::control_array fb_control = controller.getFeedbackControl(x, nominal_trajectory.col(0), 0);
    current_control += fb_control;

    // Propagate the state forward
    model.computeDynamics(x, current_control, xdot);
    model.updateState(x, xdot, dt);

    // Add disturbance
    x += noise.col(t) * sqrt(model.getParams().system_noise) * dt;

    // Slide the control sequence
    controller.slideControlSequence(1);
  }
  /************* Save CNPY *********************/
  cnpy::npy_save("vanilla_state_trajectory.npy", van_trajectory.data(),
                 { total_time_horizon, DoubleIntegratorDynamics::STATE_DIM }, "w");
  cnpy::npy_save("vanilla_nominal_trajectory.npy", van_nominal_traj.data(),
                 { total_time_horizon, num_timesteps, DoubleIntegratorDynamics::STATE_DIM }, "w");
  cnpy::npy_save("vanilla_free_energy.npy", van_free_energy.data(), { total_time_horizon }, "w");
}

void runVanillaLarge(const Eigen::Ref<const Eigen::Matrix<float, Dyn::STATE_DIM, total_time_horizon>>& noise)
{
  // Set the initial state
  DoubleIntegratorDynamics::state_array x;
  x << 2, 0, 0, 1;
  DoubleIntegratorDynamics::state_array xdot;

  // control variance
  Sampler::SAMPLING_PARAMS_T sampler_params;
  for (int i = 0; i < Dyn::CONTROL_DIM; i++)
  {
    sampler_params.std_dev[i] = 1;
  }

  // Save actual trajectories, nominal_trajectory, free energy
  std::vector<float> van_large_trajectory(Dyn::STATE_DIM * total_time_horizon, 0);
  std::vector<float> van_large_nominal_traj(Dyn::STATE_DIM * num_timesteps * total_time_horizon, 0);
  std::vector<float> van_large_free_energy(total_time_horizon, 0);

  // Initialize the controllers
  Dyn model(100);
  SCost cost;
  Sampler sampler(sampler_params);
  // DDP cost parameters
  Feedback fb_controller(&model, dt);
  auto fb_params = fb_controller.getParams();
  fb_params.Q.diagonal() << 500, 500, 100, 100;
  fb_controller.setParams(fb_params);
  auto controller = VanillaMPPIController<Dyn, SCost, Feedback, num_timesteps, 1024, Sampler>(
      &model, &cost, &fb_controller, &sampler, dt, max_iter, lambda, alpha);
  auto controller_params = controller.getParams();
  controller_params.dynamics_rollout_dim_ = dim3(64, 1, 1);
  controller_params.cost_rollout_dim_ = dim3(64, 1, 1);
  controller.setParams(controller_params);
  controller.initFeedback();

  // Start the loop
  for (int t = 0; t < total_time_horizon; ++t)
  {
    /********************** Vanilla Large **********************/
    // Compute the control
    controller.computeControl(x, 1);

    // Compute the feedback gains
    controller.computeFeedback(x);

    // Propagate the feedback trajectory
    controller.computeFeedbackPropagatedStateSeq();

    auto nominal_trajectory = controller.getTargetStateSeq();
    auto nominal_control = controller.getControlSeq();
    auto fe_stat = controller.getFreeEnergyStatistics();

    // Save everything
    saveState(x, t, van_large_trajectory);
    saveTraj(nominal_trajectory, t, van_large_nominal_traj);
    van_large_free_energy[t] = fe_stat.real_sys.freeEnergyMean;

    // Get the open loop control
    DoubleIntegratorDynamics::control_array current_control = nominal_control.col(0);

    // Apply the feedback given the current state
    Dyn::control_array fb_control = controller.getFeedbackControl(x, nominal_trajectory.col(0), 0);
    current_control += fb_control;

    // Propagate the state forward
    model.computeDynamics(x, current_control, xdot);
    model.updateState(x, xdot, dt);

    // Add disturbance
    x += noise.col(t) * sqrt(model.getParams().system_noise) * dt;

    // Slide the control sequence
    controller.slideControlSequence(1);
  }
  /************* Save CNPY *********************/
  cnpy::npy_save("vanilla_large_state_trajectory.npy", van_large_trajectory.data(),
                 { total_time_horizon, DoubleIntegratorDynamics::STATE_DIM }, "w");
  cnpy::npy_save("vanilla_large_nominal_trajectory.npy", van_large_nominal_traj.data(),
                 { total_time_horizon, num_timesteps, DoubleIntegratorDynamics::STATE_DIM }, "w");
  cnpy::npy_save("vanilla_large_free_energy.npy", van_large_free_energy.data(), { total_time_horizon }, "w");
}

void runVanillaLargeRC(const Eigen::Ref<const Eigen::Matrix<float, Dyn::STATE_DIM, total_time_horizon>>& noise)
{
  // Set the initial state
  DoubleIntegratorDynamics::state_array x;
  x << 2, 0, 0, 1;
  DoubleIntegratorDynamics::state_array xdot;

  // control variance
  Sampler::SAMPLING_PARAMS_T sampler_params;
  for (int i = 0; i < Dyn::CONTROL_DIM; i++)
  {
    sampler_params.std_dev[i] = 1;
  }

  // Save actual trajectories, nominal_trajectory, free energy
  std::vector<float> van_large_trajectory(Dyn::STATE_DIM * total_time_horizon, 0);
  std::vector<float> van_large_nominal_traj(Dyn::STATE_DIM * num_timesteps * total_time_horizon, 0);
  std::vector<float> van_large_free_energy(total_time_horizon, 0);

  // Initialize the controllers
  Dyn model(100);

  RCost cost;
  Sampler sampler(sampler_params);
  auto params = cost.getParams();
  params.crash_cost = 100;
  cost.setParams(params);
  // DDP cost parameters
  Feedback fb_controller(&model, dt);
  auto fb_params = fb_controller.getParams();
  fb_params.Q.diagonal() << 500, 500, 100, 100;
  fb_controller.setParams(fb_params);

  auto controller = VanillaMPPIController<Dyn, RCost, Feedback, num_timesteps, 1024, Sampler>(
      &model, &cost, &fb_controller, &sampler, dt, max_iter, lambda, alpha);
  auto controller_params = controller.getParams();
  controller_params.dynamics_rollout_dim_ = dim3(64, 1, 1);
  controller_params.cost_rollout_dim_ = dim3(64, 1, 1);
  controller.setParams(controller_params);
  controller.initFeedback();

  // Start the loop
  for (int t = 0; t < total_time_horizon; ++t)
  {
    /********************** Vanilla Large **********************/
    // Compute the control
    controller.computeControl(x, 1);

    // Compute the feedback gains
    controller.computeFeedback(x);

    // Propagate the feedback trajectory
    controller.computeFeedbackPropagatedStateSeq();

    auto nominal_trajectory = controller.getTargetStateSeq();
    auto nominal_control = controller.getControlSeq();
    auto fe_stat = controller.getFreeEnergyStatistics();

    // Save everything
    saveState(x, t, van_large_trajectory);
    saveTraj(nominal_trajectory, t, van_large_nominal_traj);
    van_large_free_energy[t] = fe_stat.real_sys.freeEnergyMean;

    // Get the open loop control
    DoubleIntegratorDynamics::control_array current_control = nominal_control.col(0);

    // Apply the feedback given the current state
    Dyn::control_array fb_control = controller.getFeedbackControl(x, nominal_trajectory.col(0), 0);
    current_control += fb_control;

    // Propagate the state forward
    model.computeDynamics(x, current_control, xdot);
    model.updateState(x, xdot, dt);

    // Add disturbance
    x += noise.col(t) * sqrt(model.getParams().system_noise) * dt;

    // Slide the control sequence
    controller.slideControlSequence(1);
  }
  /************* Save CNPY *********************/
  cnpy::npy_save("vanilla_large_robust_state_trajectory.npy", van_large_trajectory.data(),
                 { total_time_horizon, DoubleIntegratorDynamics::STATE_DIM }, "w");
  cnpy::npy_save("vanilla_large_robust_nominal_trajectory.npy", van_large_nominal_traj.data(),
                 { total_time_horizon, num_timesteps, DoubleIntegratorDynamics::STATE_DIM }, "w");
  cnpy::npy_save("vanilla_large_robust_free_energy.npy", van_large_free_energy.data(), { total_time_horizon }, "w");
}

void runTube(const Eigen::Ref<const Eigen::Matrix<float, Dyn::STATE_DIM, total_time_horizon>>& noise)
{
  // Set the initial state
  DoubleIntegratorDynamics::state_array x;
  x << 2, 0, 0, 1;
  DoubleIntegratorDynamics::state_array xdot;

  // control variance
  Sampler::SAMPLING_PARAMS_T sampler_params;
  for (int i = 0; i < Dyn::CONTROL_DIM; i++)
  {
    sampler_params.std_dev[i] = 1;
  }

  // Save actual trajectories, nominal_trajectory, free energy
  std::vector<float> tube_trajectory(Dyn::STATE_DIM * total_time_horizon, 0);
  std::vector<float> tube_nominal_traj(Dyn::STATE_DIM * num_timesteps * total_time_horizon, 0);
  std::vector<float> tube_nominal_free_energy(total_time_horizon, 0);
  std::vector<float> tube_real_free_energy(total_time_horizon, 0);
  std::vector<float> tube_nominal_state_used(total_time_horizon, 0);

  // Initialize the controllers
  Dyn model(100);
  SCost cost;
  Sampler sampler(sampler_params);
  // DDP cost parameters
  Feedback fb_controller(&model, dt);
  auto fb_params = fb_controller.getParams();
  fb_params.Q.diagonal() << 500, 500, 100, 100;
  fb_controller.setParams(fb_params);
  auto controller = TubeMPPIController<Dyn, SCost, Feedback, num_timesteps, 1024, Sampler>(
      &model, &cost, &fb_controller, &sampler, dt, max_iter, lambda, alpha);
  auto controller_params = controller.getParams();
  controller_params.dynamics_rollout_dim_ = dim3(64, 1, 1);
  controller_params.cost_rollout_dim_ = dim3(64, 1, 1);
  controller.setParams(controller_params);
  controller.setNominalThreshold(20);
  // Start the loop
  for (int t = 0; t < total_time_horizon; ++t)
  {
    /********************** Tube **********************/
    // Compute the control
    controller.computeControl(x, 1);

    // Compute the feedback gains
    controller.computeFeedback(x);

    // Propagate the feedback trajectory
    controller.computeFeedbackPropagatedStateSeq();

    auto nominal_trajectory = controller.getTargetStateSeq();
    auto nominal_control = controller.getControlSeq();
    auto fe_stat = controller.getFreeEnergyStatistics();

    // Save everything
    saveState(x, t, tube_trajectory);
    saveTraj(nominal_trajectory, t, tube_nominal_traj);
    tube_nominal_free_energy[t] = fe_stat.nominal_sys.freeEnergyMean;
    tube_real_free_energy[t] = fe_stat.real_sys.freeEnergyMean;
    tube_nominal_state_used[t] = fe_stat.nominal_state_used;

    // Get the open loop control
    DoubleIntegratorDynamics::control_array current_control = nominal_control.col(0);

    // Apply the feedback given the current state
    Dyn::control_array fb_control = controller.getFeedbackControl(x, nominal_trajectory.col(0), 0);
    current_control += fb_control;

    // Propagate the state forward
    model.computeDynamics(x, current_control, xdot);
    model.updateState(x, xdot, dt);
    controller.updateNominalState(current_control);

    // Add disturbance
    x += noise.col(t) * sqrt(model.getParams().system_noise) * dt;

    // Slide the control sequence
    controller.slideControlSequence(1);
  }
  /************* Save CNPY *********************/
  cnpy::npy_save("tube_state_trajectory.npy", tube_trajectory.data(),
                 { total_time_horizon, DoubleIntegratorDynamics::STATE_DIM }, "w");
  cnpy::npy_save("tube_nominal_trajectory.npy", tube_nominal_traj.data(),
                 { total_time_horizon, num_timesteps, DoubleIntegratorDynamics::STATE_DIM }, "w");
  cnpy::npy_save("tube_nominal_free_energy.npy", tube_nominal_free_energy.data(), { total_time_horizon }, "w");
  cnpy::npy_save("tube_real_free_energy.npy", tube_real_free_energy.data(), { total_time_horizon }, "w");
  cnpy::npy_save("tube_nominal_state_used.npy", tube_nominal_state_used.data(), { total_time_horizon }, "w");
}

void runTubeRC(const Eigen::Ref<const Eigen::Matrix<float, Dyn::STATE_DIM, total_time_horizon>>& noise)
{
  // Set the initial state
  DoubleIntegratorDynamics::state_array x;
  x << 2, 0, 0, 1;
  DoubleIntegratorDynamics::state_array xdot;

  // control variance
  Sampler::SAMPLING_PARAMS_T sampler_params;
  for (int i = 0; i < Dyn::CONTROL_DIM; i++)
  {
    sampler_params.std_dev[i] = 1;
  }

  // Save actual trajectories, nominal_trajectory, free energy
  std::vector<float> tube_trajectory(Dyn::STATE_DIM * total_time_horizon, 0);
  std::vector<float> tube_nominal_traj(Dyn::STATE_DIM * num_timesteps * total_time_horizon, 0);
  std::vector<float> tube_nominal_free_energy(total_time_horizon, 0);
  std::vector<float> tube_real_free_energy(total_time_horizon, 0);
  std::vector<float> tube_nominal_state_used(total_time_horizon, 0);

  // Initialize the controllers
  Dyn model(100);
  RCost cost;
  Sampler sampler(sampler_params);
  auto params = cost.getParams();
  params.crash_cost = 100;
  cost.setParams(params);
  // DDP cost parameters
  Feedback fb_controller(&model, dt);
  auto fb_params = fb_controller.getParams();
  fb_params.Q.diagonal() << 500, 500, 100, 100;
  fb_controller.setParams(fb_params);
  auto controller = TubeMPPIController<Dyn, RCost, Feedback, num_timesteps, 1024>(
      &model, &cost, &fb_controller, &sampler, dt, max_iter, lambda, alpha);
  auto controller_params = controller.getParams();
  controller_params.dynamics_rollout_dim_ = dim3(64, 1, 1);
  controller_params.cost_rollout_dim_ = dim3(64, 1, 1);
  controller.setParams(controller_params);
  controller.setNominalThreshold(2);
  // Start the loop
  for (int t = 0; t < total_time_horizon; ++t)
  {
    /********************** Tube **********************/
    // Compute the control
    controller.computeControl(x, 1);

    // Compute the feedback gains
    controller.computeFeedback(x);

    // Propagate the feedback trajectory
    controller.computeFeedbackPropagatedStateSeq();

    auto nominal_trajectory = controller.getTargetStateSeq();
    auto nominal_control = controller.getControlSeq();
    auto fe_stat = controller.getFreeEnergyStatistics();

    // Save everything
    saveState(x, t, tube_trajectory);
    saveTraj(nominal_trajectory, t, tube_nominal_traj);
    tube_nominal_free_energy[t] = fe_stat.nominal_sys.freeEnergyMean;
    tube_real_free_energy[t] = fe_stat.real_sys.freeEnergyMean;
    tube_nominal_state_used[t] = fe_stat.nominal_state_used;

    // Get the open loop control
    DoubleIntegratorDynamics::control_array current_control = nominal_control.col(0);

    // Apply the feedback given the current state
    Dyn::control_array fb_control = controller.getFeedbackControl(x, nominal_trajectory.col(0), 0);
    current_control += fb_control;

    // Propagate the state forward
    model.computeDynamics(x, current_control, xdot);
    model.updateState(x, xdot, dt);
    controller.updateNominalState(current_control);

    // Add disturbance
    x += noise.col(t) * sqrt(model.getParams().system_noise) * dt;

    // Slide the control sequence
    controller.slideControlSequence(1);
  }
  /************* Save CNPY *********************/
  cnpy::npy_save("tube_robust_state_trajectory.npy", tube_trajectory.data(),
                 { total_time_horizon, DoubleIntegratorDynamics::STATE_DIM }, "w");
  cnpy::npy_save("tube_robust_nominal_trajectory.npy", tube_nominal_traj.data(),
                 { total_time_horizon, num_timesteps, DoubleIntegratorDynamics::STATE_DIM }, "w");
  cnpy::npy_save("tube_robust_nominal_free_energy.npy", tube_nominal_free_energy.data(), { total_time_horizon }, "w");
  cnpy::npy_save("tube_robust_real_free_energy.npy", tube_real_free_energy.data(), { total_time_horizon }, "w");
  cnpy::npy_save("tube_robust_nominal_state_used.npy", tube_nominal_state_used.data(), { total_time_horizon }, "w");
}

void runRobustSc(const Eigen::Ref<const Eigen::Matrix<float, Dyn::STATE_DIM, total_time_horizon>>& noise)
{
  // Set the initial state
  DoubleIntegratorDynamics::state_array x;
  x << 2, 0, 0, 1;
  DoubleIntegratorDynamics::state_array xdot;

  // control variance
  Sampler::SAMPLING_PARAMS_T sampler_params;
  for (int i = 0; i < Dyn::CONTROL_DIM; i++)
  {
    sampler_params.std_dev[i] = 1;
  }

  // Save actual trajectories, nominal_trajectory, free energy
  std::vector<float> robust_sc_trajectory(Dyn::STATE_DIM * total_time_horizon, 0);
  std::vector<float> robust_sc_nominal_traj(Dyn::STATE_DIM * num_timesteps * total_time_horizon, 0);
  std::vector<float> robust_sc_nominal_free_energy(total_time_horizon, 0);
  std::vector<float> robust_sc_real_free_energy(total_time_horizon, 0);
  std::vector<float> robust_sc_nominal_free_energy_bound(total_time_horizon, 0);
  std::vector<float> robust_sc_real_free_energy_bound(total_time_horizon, 0);
  std::vector<float> robust_sc_real_free_energy_growth_bound(total_time_horizon, 0);
  std::vector<float> robust_sc_nominal_state_used(total_time_horizon, 0);

  // Initialize the controllers
  Dyn model(100);
  SCost cost;
  Sampler sampler(sampler_params);
  // DDP cost parameters
  Feedback fb_controller(&model, dt);
  auto fb_params = fb_controller.getParams();
  fb_params.Q.diagonal() << 500, 500, 100, 100;
  fb_controller.setParams(fb_params);
  // Value function threshold
  float value_function_threshold = 20.0;
  auto controller = RobustMPPIController<Dyn, SCost, Feedback, num_timesteps, 1024, Sampler>(
      &model, &cost, &fb_controller, &sampler, dt, max_iter, lambda, alpha, value_function_threshold);
  auto controller_params = controller.getParams();
  controller_params.dynamics_rollout_dim_ = dim3(64, 1, 1);
  controller_params.cost_rollout_dim_ = dim3(64, 1, 1);
  controller.setParams(controller_params);

  // Start the loop
  for (int t = 0; t < total_time_horizon; ++t)
  {
    /********************** Vanilla **********************/
    // Compute the control
    controller.updateImportanceSamplingControl(x, 1);
    controller.computeControl(x, 1);

    // Compute the feedback gains
    controller.computeFeedback(x);

    // Propagate the feedback trajectory
    controller.computeFeedbackPropagatedStateSeq();

    auto nominal_trajectory = controller.getTargetStateSeq();
    auto nominal_control = controller.getControlSeq();
    auto fe_stat = controller.getFreeEnergyStatistics();

    // Save everything
    saveState(x, t, robust_sc_trajectory);
    saveTraj(nominal_trajectory, t, robust_sc_nominal_traj);
    robust_sc_nominal_free_energy[t] = fe_stat.nominal_sys.freeEnergyMean;
    robust_sc_real_free_energy[t] = fe_stat.real_sys.freeEnergyMean;
    robust_sc_nominal_free_energy_bound[t] =
        value_function_threshold + 2 * fe_stat.nominal_sys.freeEnergyModifiedVariance;
    robust_sc_real_free_energy_bound[t] = 0;
    robust_sc_real_free_energy_growth_bound[t] = 0;
    robust_sc_nominal_state_used[t] = fe_stat.nominal_state_used;

    // Get the open loop control
    DoubleIntegratorDynamics::control_array current_control = nominal_control.col(0);

    // Apply the feedback given the current state
    Dyn::control_array fb_control = controller.getFeedbackControl(x, nominal_trajectory.col(0), 0);
    current_control += fb_control;

    // Propagate the state forward
    model.computeDynamics(x, current_control, xdot);
    model.updateState(x, xdot, dt);

    // Add disturbance
    x += noise.col(t) * sqrt(model.getParams().system_noise) * dt;

    // Slide the control sequence
    controller.slideControlSequence(1);
  }
  /************* Save CNPY *********************/
  cnpy::npy_save("robust_sc_state_trajectory.npy", robust_sc_trajectory.data(),
                 { total_time_horizon, DoubleIntegratorDynamics::STATE_DIM }, "w");
  cnpy::npy_save("robust_sc_nominal_trajectory.npy", robust_sc_nominal_traj.data(),
                 { total_time_horizon, num_timesteps, DoubleIntegratorDynamics::STATE_DIM }, "w");
  cnpy::npy_save("robust_sc_nominal_free_energy.npy", robust_sc_nominal_free_energy.data(), { total_time_horizon },
                 "w");
  cnpy::npy_save("robust_sc_real_free_energy.npy", robust_sc_real_free_energy.data(), { total_time_horizon }, "w");
  cnpy::npy_save("robust_sc_nominal_state_used.npy", robust_sc_nominal_state_used.data(), { total_time_horizon }, "w");
  cnpy::npy_save("robust_sc_real_free_energy_bound.npy", robust_sc_nominal_free_energy_bound.data(),
                 { total_time_horizon }, "w");
  cnpy::npy_save("robust_sc_nominal_free_energy_bound.npy", robust_sc_real_free_energy_bound.data(),
                 { total_time_horizon }, "w");
  cnpy::npy_save("robust_sc_real_free_energy_growth_bound.npy", robust_sc_real_free_energy_growth_bound.data(),
                 { total_time_horizon }, "w");
}

void runRobustRc(const Eigen::Ref<const Eigen::Matrix<float, Dyn::STATE_DIM, total_time_horizon>>& noise)
{
  // Set the initial state
  DoubleIntegratorDynamics::state_array x;
  x << 2, 0, 0, 1;
  DoubleIntegratorDynamics::state_array xdot;

  // control variance
  Sampler::SAMPLING_PARAMS_T sampler_params;
  for (int i = 0; i < Dyn::CONTROL_DIM; i++)
  {
    sampler_params.std_dev[i] = 1;
  }

  // Save actual trajectories, nominal_trajectory, free energy
  std::vector<float> robust_rc_trajectory(Dyn::STATE_DIM * total_time_horizon, 0);
  std::vector<float> robust_rc_nominal_traj(Dyn::STATE_DIM * num_timesteps * total_time_horizon, 0);
  std::vector<float> robust_rc_nominal_free_energy(total_time_horizon, 0);
  std::vector<float> robust_rc_real_free_energy(total_time_horizon, 0);
  std::vector<float> robust_rc_nominal_free_energy_bound(total_time_horizon, 0);
  std::vector<float> robust_rc_real_free_energy_bound(total_time_horizon, 0);
  std::vector<float> robust_rc_real_free_energy_growth_bound(total_time_horizon, 0);
  std::vector<float> robust_rc_nominal_free_energy_growth(total_time_horizon, 0);
  std::vector<float> robust_rc_real_free_energy_growth(total_time_horizon, 0);
  std::vector<float> robust_rc_nominal_state_used(total_time_horizon, 0);

  // Initialize the controllers
  Dyn model(100);
  RCost cost;
  auto params = cost.getParams();
  params.crash_cost = 100;
  cost.setParams(params);
  Sampler sampler(sampler_params);

  // DDP cost parameters
  Feedback fb_controller(&model, dt);
  auto fb_params = fb_controller.getParams();
  fb_params.Q.diagonal() << 500, 500, 100, 100;
  fb_controller.setParams(fb_params);

  // Value function threshold
  float value_function_threshold = 20.0;
  auto controller = RobustMPPIController<Dyn, RCost, Feedback, num_timesteps, 1024, Sampler>(
      &model, &cost, &fb_controller, &sampler, dt, max_iter, lambda, alpha, value_function_threshold);
  auto controller_params = controller.getParams();
  controller_params.dynamics_rollout_dim_ = dim3(64, 1, 1);
  controller_params.cost_rollout_dim_ = dim3(64, 1, 1);
  controller.setParams(controller_params);

  // Start the loop
  for (int t = 0; t < total_time_horizon; ++t)
  {
    /********************** Robust Robust Cost **********************/
    // Compute the control
    controller.updateImportanceSamplingControl(x, 1);
    controller.computeControl(x, 1);

    // Compute the feedback gains
    controller.computeFeedback(x);

    // Propagate the feedback trajectory
    controller.computeFeedbackPropagatedStateSeq();

    auto nominal_trajectory = controller.getTargetStateSeq();
    auto nominal_control = controller.getControlSeq();
    auto fe_stat = controller.getFreeEnergyStatistics();

    // Save everything
    saveState(x, t, robust_rc_trajectory);
    saveTraj(nominal_trajectory, t, robust_rc_nominal_traj);
    robust_rc_nominal_free_energy[t] = fe_stat.nominal_sys.freeEnergyMean;
    robust_rc_real_free_energy[t] = fe_stat.real_sys.freeEnergyMean;
    robust_rc_nominal_free_energy_bound[t] =
        value_function_threshold + 2 * fe_stat.nominal_sys.freeEnergyModifiedVariance;
    robust_rc_real_free_energy_bound[t] = fe_stat.nominal_sys.freeEnergyMean +
                                          cost.getLipshitzConstantCost() * 1 * (x - nominal_trajectory.col(0)).norm();
    robust_rc_real_free_energy_growth_bound[t] = (value_function_threshold - fe_stat.nominal_sys.freeEnergyMean) +
                                                 cost.getLipshitzConstantCost() * 8 * 20 * controller.computeDF() +
                                                 2 * fe_stat.nominal_sys.freeEnergyModifiedVariance;
    robust_rc_nominal_free_energy_growth[t] = fe_stat.nominal_sys.increase;
    robust_rc_real_free_energy_growth[t] = fe_stat.real_sys.increase;
    robust_rc_nominal_state_used[t] = fe_stat.nominal_state_used;

    // Get the open loop control
    DoubleIntegratorDynamics::control_array current_control = nominal_control.col(0);

    // Apply the feedback given the current state
    Dyn::control_array fb_control = controller.getFeedbackControl(x, nominal_trajectory.col(0), 0);
    current_control += fb_control;

    // Propagate the state forward
    model.computeDynamics(x, current_control, xdot);
    model.updateState(x, xdot, dt);

    // Add disturbance
    x += noise.col(t) * sqrt(model.getParams().system_noise) * dt;

    // Slide the control sequence
    controller.slideControlSequence(1);
  }
  /************* Save CNPY *********************/
  cnpy::npy_save("robust_rc_state_trajectory.npy", robust_rc_trajectory.data(),
                 { total_time_horizon, DoubleIntegratorDynamics::STATE_DIM }, "w");
  cnpy::npy_save("robust_rc_nominal_trajectory.npy", robust_rc_nominal_traj.data(),
                 { total_time_horizon, num_timesteps, DoubleIntegratorDynamics::STATE_DIM }, "w");
  cnpy::npy_save("robust_rc_nominal_free_energy.npy", robust_rc_nominal_free_energy.data(), { total_time_horizon },
                 "w");
  cnpy::npy_save("robust_rc_real_free_energy.npy", robust_rc_real_free_energy.data(), { total_time_horizon }, "w");
  cnpy::npy_save("robust_rc_nominal_state_used.npy", robust_rc_nominal_state_used.data(), { total_time_horizon }, "w");
  cnpy::npy_save("robust_rc_real_free_energy_bound.npy", robust_rc_real_free_energy_bound.data(),
                 { total_time_horizon }, "w");
  cnpy::npy_save("robust_rc_nominal_free_energy_bound.npy", robust_rc_nominal_free_energy_bound.data(),
                 { total_time_horizon }, "w");
  cnpy::npy_save("robust_rc_real_free_energy_growth_bound.npy", robust_rc_real_free_energy_growth_bound.data(),
                 { total_time_horizon }, "w");
  cnpy::npy_save("robust_rc_real_free_energy_growth.npy", robust_rc_real_free_energy_growth.data(),
                 { total_time_horizon }, "w");
  cnpy::npy_save("robust_rc_nominal_free_energy_growth.npy", robust_rc_nominal_free_energy_growth.data(),
                 { total_time_horizon }, "w");
}

int main()
{
  // Run the double integrator example on all the controllers with the SAME noise 20 times.

  // Create a random number generator
  // Random number generator for system noise
  std::mt19937 gen;  // Standard mersenne_twister_engine which will be seeded
  std::normal_distribution<float> normal_distribution;
  gen.seed(7);  // Seed the 7, so everyone gets the same noise
  normal_distribution = std::normal_distribution<float>(0, 1);

  Eigen::Matrix<float, Dyn::STATE_DIM, total_time_horizon> universal_noise;
  universal_noise.setZero();

  // Create the noise for all systems
  for (int t = 0; t < total_time_horizon; ++t)
  {
    for (int i = 2; i < 4; ++i)
    {
      universal_noise(i, t) = normal_distribution(gen);
    }
  }

  runVanilla(universal_noise);
  std::cout << "Finished Vanilla" << std::endl;

  runVanillaLarge(universal_noise);
  std::cout << "Finished Vanilla Large" << std::endl;

  runVanillaLargeRC(universal_noise);
  std::cout << "Finished Vanilla Large with Robust Cost" << std::endl;

  runTube(universal_noise);
  std::cout << "Finished Tube with Standard Cost" << std::endl;

  runTubeRC(universal_noise);
  std::cout << "Finished Tube with Robust Cost" << std::endl;

  runRobustSc(universal_noise);
  std::cout << "Finished RMPPI with Standard Cost" << std::endl;

  runRobustRc(universal_noise);
  std::cout << "Finished RMPPI with Robust Cost" << std::endl;

  return 0;
}
