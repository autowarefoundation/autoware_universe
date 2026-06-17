#include <mppi/instantiations/cartpole_mppi/cartpole_mppi.cuh>
#include <iostream>
#include <chrono>

using SAMPLER_T = mppi::sampling_distributions::GaussianDistribution<CartpoleDynamics::DYN_PARAMS_T>;

int main(int argc, char** argv)
{
  auto model = new CartpoleDynamics(1.0, 1.0, 1.0);
  auto cost = new CartpoleQuadraticCost;

  model->control_rngs_->x = -5;
  model->control_rngs_->y = 5;

  CartpoleQuadraticCostParams new_params;
  new_params.cart_position_coeff = 50;
  new_params.pole_angle_coeff = 200;
  new_params.cart_velocity_coeff = 10;
  new_params.pole_angular_velocity_coeff = 1;
  new_params.control_cost_coeff[0] = 0;
  new_params.terminal_cost_coeff = 0;
  new_params.desired_terminal_state[0] = 20;
  new_params.desired_terminal_state[1] = 0;
  new_params.desired_terminal_state[2] = M_PI;
  new_params.desired_terminal_state[3] = 0;

  cost->setParams(new_params);

  float dt = 0.02;
  int max_iter = 1;
  float lambda = 0.25;
  float alpha = 0.0;
  const int num_timesteps = 100;

  // Set up Gaussian Distribution
  auto sampler_params = SAMPLER_T::SAMPLING_PARAMS_T();
  for (int i = 0; i < CartpoleDynamics::CONTROL_DIM; i++)
  {
    sampler_params.std_dev[i] = 5.0;
  }
  auto sampler = new SAMPLER_T(sampler_params);

  // Feedback Controller
  auto fb_controller = new DDPFeedback<CartpoleDynamics, num_timesteps>(model, dt);

  auto CartpoleController =
      new VanillaMPPIController<CartpoleDynamics, CartpoleQuadraticCost, DDPFeedback<CartpoleDynamics, num_timesteps>,
                                num_timesteps, 2048>(model, cost, fb_controller, sampler, dt, max_iter, lambda, alpha);
  auto controller_params = CartpoleController->getParams();
  controller_params.dynamics_rollout_dim_ = dim3(64, 4, 1);
  controller_params.cost_rollout_dim_ = dim3(64, 4, 1);
  CartpoleController->setParams(controller_params);

  CartpoleDynamics::state_array current_state = model->getZeroState();
  CartpoleDynamics::state_array next_state = model->getZeroState();
  CartpoleDynamics::output_array output = CartpoleDynamics::output_array::Zero();

  int time_horizon = 5000;

  CartpoleDynamics::state_array xdot = model->getZeroState();

  auto time_start = std::chrono::system_clock::now();
  for (int i = 0; i < time_horizon; ++i)
  {
    // Compute the control
    CartpoleController->computeControl(current_state, 1);

    // Increment the state
    CartpoleDynamics::control_array control;
    control = CartpoleController->getControlSeq().block(0, 0, CartpoleDynamics::CONTROL_DIM, 1);
    model->enforceConstraints(current_state, control);
    model->step(current_state, next_state, xdot, control, output, i, dt);
    current_state = next_state;

    if (i % 50 == 0)
    {
      printf("Current Time: %f    ", i * dt);
      printf("Current Baseline Cost: %f    ", CartpoleController->getBaselineCost());
      model->printState(current_state.data());
      //      std::cout << control << std::endl;
    }

    // Slide the controls down before calling the optimizer again
    CartpoleController->slideControlSequence(1);
  }
  auto time_end = std::chrono::system_clock::now();
  auto diff = std::chrono::duration<double, std::milli>(time_end - time_start);
  printf("The elapsed time is: %f milliseconds\n", diff.count());
  //    std::cout << "The current control at timestep " << i << " is: " << CartpoleController.get_control_seq()[i] <<
  //    std::endl;

  // cost->freeCudaMem();
  delete (CartpoleController);
  delete (cost);
  delete (model);
  delete (fb_controller);
  delete sampler;

  return 0;
}
