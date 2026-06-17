#include <mppi/dynamics/double_integrator/di_dynamics.cuh>
#include <mppi/cost_functions/quadratic_cost/quadratic_cost.cuh>
#include <mppi/controllers/MPPI/mppi_controller.cuh>
#include <mppi/feedback_controllers/DDP/ddp.cuh>

#include <mppi/sampling_distributions/colored_noise/colored_noise.cuh>

#include <iomanip>
// #include <sstream>

#define USE_COLORED_NOISE

const int TIMESTEPS = 65;
const int NUM_ROLLOUTS = 128;

using DYN = DoubleIntegratorDynamics;
using COST = QuadraticCost<DYN>;
using FB_CONTROLLER = DDPFeedback<DYN, TIMESTEPS>;

#ifdef USE_COLORED_NOISE
using SAMPLER = mppi::sampling_distributions::ColoredNoiseDistribution<DYN::DYN_PARAMS_T>;
#else
using SAMPLER = mppi::sampling_distributions::GaussianDistribution<DYN::DYN_PARAMS_T>;
#endif

int main()
{
  float dt = 0.015;

  // Set the initial state
  DYN::state_array x;
  x << -9, -9, 0.1, 0.1;
  DYN::state_array xdot;

  SAMPLER::SAMPLING_PARAMS_T sampler_params;
  for (int i = 0; i < DYN::CONTROL_DIM; i++)
  {
    sampler_params.std_dev[i] = 0.5;
#ifdef USE_COLORED_NOISE
    sampler_params.exponents[i] = 1.0;
#endif
  }
  SAMPLER sampler(sampler_params);

  // Create the dynamics, cost function, and feedback controller
  DYN model;
  COST cost;
  FB_CONTROLLER fb_controller = FB_CONTROLLER(&model, dt);
  auto cost_params = cost.getParams();

  // Set up cost function
  DYN::state_array x_goal;
  x_goal << -4, -4, 0, 0;
  DYN::state_array q_coeffs;
  q_coeffs << 5, 5, 0.5, 0.5;
  for (int i = 0; i < DYN::STATE_DIM; i++)
  {
    cost_params.s_coeffs[i] = q_coeffs[i];
    cost_params.s_goal[i] = x_goal[i];
  }
  cost.setParams(cost_params);

  // Create MPPI Controller
  float lambda = 1;
  float alpha = 1.0;
  int max_iter = 1;
  int total_time_horizon = 300;

  auto controller = VanillaMPPIController<DYN, COST, FB_CONTROLLER, TIMESTEPS, NUM_ROLLOUTS, SAMPLER>(
      &model, &cost, &fb_controller, &sampler, dt, max_iter, lambda, alpha);

  auto controller_params = controller.getParams();
  controller_params.dynamics_rollout_dim_ = dim3(64, 1, 1);
  controller_params.cost_rollout_dim_ = dim3(64, 1, 1);
  controller.setParams(controller_params);

  /********************** Vanilla MPPI **********************/
  float cumulative_cost = 0;
  int crash = 0;
  for (int t = 0; t < total_time_horizon; ++t)
  {
    // Compute the control
    controller.computeControl(x, 1);

    auto nominal_control = controller.getControlSeq();
    auto fe_stat = controller.getFreeEnergyStatistics();

    DYN::control_array current_control = nominal_control.col(0);
    // Propagate dynamics forward
    DYN::output_array y;
    DYN::state_array x_next;
    // model.computeDynamics(x, current_control, xdot);
    // model.updateState(x, xdot, dt);
    model.step(x, x_next, xdot, current_control, y, t, dt);
    x = x_next;
    if (t % 10 == 0)
    {
      std::cout << "T: " << std::fixed << std::setprecision(3) << t * dt;
      // << "s Free Energy: " << fe_stat.real_sys.freeEnergyMean
      // << " +- " << fe_stat.real_sys.freeEnergyVariance << std::endl;
      std::cout << " X: " << x.transpose() << std::endl;
    }

    // Slide the control sequence
    controller.slideControlSequence(1);
    cumulative_cost += cost.computeRunningCost(y, current_control, t, &crash);
  }
  std::cout << "Total Cost: " << cumulative_cost << std::endl;

  return 0;
}
