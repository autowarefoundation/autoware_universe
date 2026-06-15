//
// Created by mgandhi on 5/23/20.
//

#include "rmppi_kernel_test.cuh"

template <class DYN_T, class COST_T, class SAMPLER_T, class FB_T>
void launchCPURMPPIRolloutKernel(DYN_T* model, COST_T* cost, SAMPLER_T* sampler, FB_T* fb_controller, const float dt,
                                 const int num_timesteps, const int num_rollouts, const float lambda, const float alpha,
                                 const float value_func_threshold, const int nominal_idx, const int real_idx,
                                 const Eigen::Ref<const typename DYN_T::state_array>& initial_real_state,
                                 const Eigen::Ref<const typename DYN_T::state_array>& initial_nominal_state,
                                 Eigen::Ref<Eigen::MatrixXf> trajectory_costs)
{
  using state_array = typename DYN_T::state_array;
  using output_array = typename DYN_T::output_array;
  using control_array = typename DYN_T::control_array;
  Eigen::MatrixXf control_noise_real = Eigen::MatrixXf::Zero(DYN_T::CONTROL_DIM, num_rollouts * num_timesteps);
  Eigen::MatrixXf control_noise_nom = Eigen::MatrixXf::Zero(DYN_T::CONTROL_DIM, num_rollouts * num_timesteps);
  Eigen::MatrixXi crash_status_real = Eigen::MatrixXi::Zero(num_rollouts, 1);
  Eigen::MatrixXi crash_status_nom = Eigen::MatrixXi::Zero(num_rollouts, 1);
  HANDLE_ERROR(cudaMemcpy(control_noise_nom.data(), sampler->getControlSample(0, 0, nominal_idx),
                          sizeof(float) * DYN_T::CONTROL_DIM * num_rollouts * num_timesteps, cudaMemcpyDeviceToHost));
  HANDLE_ERROR(cudaMemcpy(control_noise_real.data(), sampler->getControlSample(0, 0, real_idx),
                          sizeof(float) * DYN_T::CONTROL_DIM * num_rollouts * num_timesteps, cudaMemcpyDeviceToHost));

  state_array curr_real_state, curr_nom_state, next_real_state, next_nom_state, state_der_real, state_der_nom;
  output_array output_real, output_nom;
  control_array u_real, u_nom, u_feedback;
  auto sampler_params = sampler->getParams();
  for (int sample_idx = 0; sample_idx < num_rollouts; sample_idx++)
  {
    curr_real_state = initial_real_state;
    curr_nom_state = initial_nominal_state;
    float running_real_cost = 0;
    float running_nom_cost = 0;
    float tracking_nom_cost = 0;
    float tracking_real_cost = 0;
    model->initializeDynamics(curr_real_state, u_real, output_real, 0, dt);
    cost->initializeCosts(output_real, u_real, 0, dt);
    for (int t = 0; t < num_timesteps; t++)
    {
      u_real = control_noise_real.col(sample_idx * num_timesteps + t);
      u_nom = control_noise_nom.col(sample_idx * num_timesteps + t);
      u_feedback = fb_controller->k(curr_real_state, curr_nom_state, t);
      u_real += u_feedback;
      model->enforceConstraints(curr_real_state, u_real);
      model->enforceConstraints(curr_nom_state, u_nom);
      model->step(curr_real_state, next_real_state, state_der_real, u_real, output_real, t, dt);
      model->step(curr_nom_state, next_nom_state, state_der_nom, u_nom, output_nom, t, dt);
      float real_cost = cost->computeRunningCost(output_real, u_real, t, &crash_status_real(sample_idx));
      float nom_cost = cost->computeRunningCost(output_nom, u_nom, t, &crash_status_nom(sample_idx));
      tracking_real_cost += real_cost + sampler->computeFeedbackCost(u_feedback.data(), (float*)&sampler_params, t,
                                                                     real_idx, lambda, alpha);
      running_real_cost += real_cost + sampler->computeLikelihoodRatioCost(u_real, t, real_idx, lambda, alpha);
      running_nom_cost += nom_cost;
      tracking_nom_cost += sampler->computeLikelihoodRatioCost(u_nom, t, nominal_idx, lambda, alpha);

      curr_real_state = next_real_state;
      curr_nom_state = next_nom_state;
    }
    running_real_cost += cost->terminalCost(output_real);
    tracking_real_cost += cost->terminalCost(output_real);
    running_nom_cost += cost->terminalCost(output_nom);

    tracking_nom_cost /= num_timesteps;
    tracking_real_cost /= num_timesteps;
    running_nom_cost /= num_timesteps;
    running_real_cost /= num_timesteps;

    running_nom_cost =
        0.5f * running_nom_cost + 0.5f * fmaxf(fminf(tracking_real_cost, value_func_threshold), running_nom_cost);
    running_nom_cost += tracking_nom_cost;
    trajectory_costs(sample_idx + nominal_idx * num_rollouts, 0) = running_nom_cost;
    trajectory_costs(sample_idx + real_idx * num_rollouts, 0) = running_real_cost;
  }
}

template <class DYN_T, class COST_T, class SAMPLER_T>
void launchCPUInitEvalKernel(DYN_T* model, COST_T* cost, SAMPLER_T* sampler, const float dt, const int num_timesteps,
                             const int num_candidates, const int num_samples, const float lambda, const float alpha,
                             const Eigen::Ref<const Eigen::MatrixXf>& candidates,
                             const Eigen::Ref<const Eigen::MatrixXi>& strides,
                             Eigen::Ref<Eigen::MatrixXf> trajectory_costs)
{
  using state_array = typename DYN_T::state_array;
  using output_array = typename DYN_T::output_array;
  using control_array = typename DYN_T::control_array;
  const int num_rollouts = num_candidates * num_samples;
  Eigen::MatrixXi crash_status = Eigen::MatrixXi::Zero(num_rollouts, 1);

  // Get control samples from sampler
  Eigen::MatrixXf control_noise = Eigen::MatrixXf::Zero(DYN_T::CONTROL_DIM, num_rollouts * num_timesteps);
  HANDLE_ERROR(cudaMemcpy(control_noise.data(), sampler->getControlSample(0, 0, 0),
                          sizeof(float) * DYN_T::CONTROL_DIM * num_rollouts * num_timesteps, cudaMemcpyDeviceToHost));

  state_array curr_state, next_state, state_der;
  output_array output;
  control_array u;
  for (int candidate_idx = 0; candidate_idx < num_candidates; candidate_idx++)
  {
    for (int sample_idx = 0; sample_idx < num_samples; sample_idx++)
    {
      int global_idx = candidate_idx * num_samples + sample_idx;
      float& running_cost = trajectory_costs(global_idx, 0);
      curr_state = candidates.col(candidate_idx);
      model->initializeDynamics(curr_state, u, output, 0, dt);
      cost->initializeCosts(output, u, 0, dt);
      // sampler->initializeDistributions();
      for (int t = 0; t < num_timesteps; t++)
      {
        int control_index = sample_idx * num_timesteps + min(t + strides(candidate_idx, 0), num_timesteps - 1);
        u = control_noise.col(control_index);
        model->enforceConstraints(curr_state, u);
        model->step(curr_state, next_state, state_der, u, output, t, dt);
        // if (t > 0)
        // {
        running_cost += cost->computeRunningCost(output, u, t, &crash_status(global_idx));
        running_cost += sampler->computeLikelihoodRatioCost(u, t, 0, lambda, alpha);
        // }
        curr_state = next_state;
      }
      running_cost += cost->terminalCost(output);
      running_cost /= (num_timesteps);
    }
  }
}
