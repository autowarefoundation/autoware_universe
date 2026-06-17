#include <mppi/controllers/ColoredMPPI/colored_mppi_controller.cuh>
#include <mppi/core/mppi_common.cuh>
#include <algorithm>
#include <iostream>
#include <mppi/sampling_distributions/colored_noise/colored_noise.cuh>

#define ColoredMPPI_TEMPLATE                                                                                           \
  template <class DYN_T, class COST_T, class FB_T, int MAX_TIMESTEPS, int NUM_ROLLOUTS, class SAMPLING_T,              \
            class PARAMS_T>
#define ColoredMPPI ColoredMPPIController<DYN_T, COST_T, FB_T, MAX_TIMESTEPS, NUM_ROLLOUTS, SAMPLING_T, PARAMS_T>

ColoredMPPI_TEMPLATE ColoredMPPI::ColoredMPPIController(DYN_T* model, COST_T* cost, FB_T* fb_controller,
                                                        SAMPLING_T* sampler, float dt, int max_iter, float lambda,
                                                        float alpha, int num_timesteps,
                                                        const Eigen::Ref<const control_trajectory>& init_control_traj,
                                                        cudaStream_t stream)
  : PARENT_CLASS(model, cost, fb_controller, sampler, dt, max_iter, lambda, alpha, num_timesteps, init_control_traj,
                 stream)
{
  // Allocate CUDA memory for the controller
  allocateCUDAMemory();

  // Copy the noise std_dev to the device
  // this->copyControlStdDevToDevice();
  chooseAppropriateKernel();
}

ColoredMPPI_TEMPLATE ColoredMPPI::ColoredMPPIController(DYN_T* model, COST_T* cost, FB_T* fb_controller,
                                                        SAMPLING_T* sampler, PARAMS_T& params, cudaStream_t stream)
  : PARENT_CLASS(model, cost, fb_controller, sampler, params, stream)
{
  // Allocate CUDA memory for the controller
  allocateCUDAMemory();

  // // Copy the noise std_dev to the device
  // this->copyControlStdDevToDevice();
  chooseAppropriateKernel();
}

ColoredMPPI_TEMPLATE void ColoredMPPI::chooseAppropriateKernel()
{
  cudaDeviceProp deviceProp;
  HANDLE_ERROR(cudaGetDeviceProperties(&deviceProp, 0));
  unsigned single_kernel_byte_size = mppi::kernels::calcRolloutCombinedKernelSharedMemSize(
      this->model_, this->cost_, this->sampler_, this->params_.dynamics_rollout_dim_);
  unsigned split_dyn_kernel_byte_size = mppi::kernels::calcRolloutDynamicsKernelSharedMemSize(
      this->model_, this->sampler_, this->params_.dynamics_rollout_dim_);
  unsigned split_cost_kernel_byte_size =
      mppi::kernels::calcRolloutCostKernelSharedMemSize(this->cost_, this->sampler_, this->params_.cost_rollout_dim_);
  unsigned vis_single_kernel_byte_size = mppi::kernels::calcVisualizeKernelSharedMemSize(
      this->model_, this->cost_, this->sampler_, this->getNumTimesteps(), this->params_.visualize_dim_);

  bool too_much_mem_single_kernel = single_kernel_byte_size > deviceProp.sharedMemPerBlock;
  bool too_much_mem_vis_kernel = vis_single_kernel_byte_size > deviceProp.sharedMemPerBlock;
  bool too_much_mem_split_kernel = split_dyn_kernel_byte_size > deviceProp.sharedMemPerBlock;
  too_much_mem_split_kernel = too_much_mem_split_kernel || split_cost_kernel_byte_size > deviceProp.sharedMemPerBlock;
  too_much_mem_single_kernel = too_much_mem_single_kernel || too_much_mem_vis_kernel;

  if (too_much_mem_split_kernel && too_much_mem_single_kernel)
  {
    std::string error_msg =
        "There is not enough shared memory on the GPU for either rollout kernel option. The combined rollout kernel "
        "takes " +
        std::to_string(single_kernel_byte_size) + " bytes, the cost rollout kernel takes " +
        std::to_string(split_cost_kernel_byte_size) + " bytes, the dynamics rollout kernel takes " +
        std::to_string(split_dyn_kernel_byte_size) + " bytes, the combined visualization kernel takes " +
        std::to_string(vis_single_kernel_byte_size) + " bytes, and the max is " +
        std::to_string(deviceProp.sharedMemPerBlock) +
        " bytes. Considering lowering the corresponding thread block sizes.";
    throw std::runtime_error(error_msg);
  }
  else if (too_much_mem_single_kernel)
  {
    this->setKernelChoice(kernelType::USE_SPLIT_KERNELS);
    return;
  }
  else if (too_much_mem_split_kernel)
  {
    this->setKernelChoice(kernelType::USE_SINGLE_KERNEL);
    return;
  }

  // Send the nominal control to the device
  this->copyNominalControlToDevice(false);
  state_array zero_state = this->model_->getZeroState();
  // Send zero state to the device
  HANDLE_ERROR(cudaMemcpyAsync(this->initial_state_d_, zero_state.data(), DYN_T::STATE_DIM * sizeof(float),
                               cudaMemcpyHostToDevice, this->stream_));
  // Generate noise data
  this->sampler_->generateSamples(1, 0, this->gen_, true);

  float single_kernel_time_ms = std::numeric_limits<float>::infinity();
  float split_kernel_time_ms = std::numeric_limits<float>::infinity();

  // Evaluate each kernel that is applicable
  auto start_single_kernel_time = std::chrono::steady_clock::now();
  for (int i = 0; i < this->getNumKernelEvaluations() && !too_much_mem_single_kernel; i++)
  {
    mppi::kernels::launchRolloutKernel<DYN_T, COST_T, SAMPLING_T>(
        this->model_, this->cost_, this->sampler_, this->getDt(), this->getNumTimesteps(), NUM_ROLLOUTS,
        this->getLambda(), this->getAlpha(), this->initial_state_d_, this->trajectory_costs_d_,
        this->params_.dynamics_rollout_dim_, this->stream_, true);
  }
  auto end_single_kernel_time = std::chrono::steady_clock::now();
  auto start_split_kernel_time = std::chrono::steady_clock::now();
  for (int i = 0; i < this->getNumKernelEvaluations() && !too_much_mem_split_kernel; i++)
  {
    mppi::kernels::launchSplitRolloutKernel<DYN_T, COST_T, SAMPLING_T>(
        this->model_, this->cost_, this->sampler_, this->getDt(), this->getNumTimesteps(), NUM_ROLLOUTS,
        this->getLambda(), this->getAlpha(), this->initial_state_d_, this->output_d_, this->trajectory_costs_d_,
        this->params_.dynamics_rollout_dim_, this->params_.cost_rollout_dim_, this->stream_, true);
  }
  auto end_split_kernel_time = std::chrono::steady_clock::now();

  // calc times
  if (!too_much_mem_single_kernel)
  {
    single_kernel_time_ms = mppi::math::timeDiffms(end_single_kernel_time, start_single_kernel_time);
  }
  if (!too_much_mem_split_kernel)
  {
    split_kernel_time_ms = mppi::math::timeDiffms(end_split_kernel_time, start_split_kernel_time);
  }
  std::string kernel_choice = "";
  if (split_kernel_time_ms < single_kernel_time_ms)
  {
    this->setKernelChoice(kernelType::USE_SPLIT_KERNELS);
    kernel_choice = "split ";
  }
  else
  {
    this->setKernelChoice(kernelType::USE_SINGLE_KERNEL);
    kernel_choice = "single";
  }
  this->logger_->info("Choosing %s kernel based on split taking %f ms and single taking %f ms after %d iterations\n",
                     kernel_choice.c_str(), split_kernel_time_ms, single_kernel_time_ms,
                     this->getNumKernelEvaluations());
}

ColoredMPPI_TEMPLATE ColoredMPPI::~ColoredMPPIController()
{
  // all implemented in standard controller
}

ColoredMPPI_TEMPLATE void ColoredMPPI::computeControl(const Eigen::Ref<const state_array>& state,
                                                      int optimization_stride)
{
  this->free_energy_statistics_.real_sys.previousBaseline = this->getBaselineCost();
  state_array local_state = state;

  if (getLeashActive())
  {
    this->model_->enforceLeash(state, this->state_.col(leash_jump_), this->params_.state_leash_dist_, local_state);
  }

  // Send the initial condition to the device
  HANDLE_ERROR(cudaMemcpyAsync(this->initial_state_d_, local_state.data(), DYN_T::STATE_DIM * sizeof(float),
                               cudaMemcpyHostToDevice, this->stream_));

  float baseline_prev = 1e8;
  for (int opt_iter = 0; opt_iter < this->getNumIters(); opt_iter++)
  {
    // Send the nominal control to the device
    this->copyNominalControlToDevice(false);

    // Generate noise data
    this->sampler_->generateSamples(optimization_stride, opt_iter, this->gen_, false);
    // Launch the rollout kernel
    if (this->getKernelChoiceAsEnum() == kernelType::USE_SPLIT_KERNELS)
    {
      mppi::kernels::launchSplitRolloutKernel<DYN_T, COST_T, SAMPLING_T>(
          this->model_, this->cost_, this->sampler_, this->getDt(), this->getNumTimesteps(), NUM_ROLLOUTS,
          this->getLambda(), this->getAlpha(), this->initial_state_d_, this->output_d_, this->trajectory_costs_d_,
          this->params_.dynamics_rollout_dim_, this->params_.cost_rollout_dim_, this->stream_, false);
    }
    else if (this->getKernelChoiceAsEnum() == kernelType::USE_SINGLE_KERNEL)
    {
      mppi::kernels::launchRolloutKernel<DYN_T, COST_T, SAMPLING_T>(
          this->model_, this->cost_, this->sampler_, this->getDt(), this->getNumTimesteps(), NUM_ROLLOUTS,
          this->getLambda(), this->getAlpha(), this->initial_state_d_, this->trajectory_costs_d_,
          this->params_.dynamics_rollout_dim_, this->stream_, false);
    }

    // Copy the costs back to the host
    HANDLE_ERROR(cudaMemcpyAsync(this->trajectory_costs_.data(), this->trajectory_costs_d_,
                                 NUM_ROLLOUTS * sizeof(float), cudaMemcpyDeviceToHost, this->stream_));
    HANDLE_ERROR(cudaStreamSynchronize(this->stream_));

    this->setBaseline(mppi::kernels::computeBaselineCost(this->trajectory_costs_.data(), NUM_ROLLOUTS));

    if (this->getBaselineCost() > baseline_prev + 1)
    {
      this->logger_->debug("Previous Baseline: %f\n         Baseline: %f\n", baseline_prev, this->getBaselineCost());
    }

    baseline_prev = this->getBaselineCost();

    // Launch the norm exponential kernel
    if (getGamma() == 0 || getRExp() == 0)
    {
      mppi::kernels::launchNormExpKernel(NUM_ROLLOUTS, this->getNormExpThreads(), this->trajectory_costs_d_,
                                         1.0 / this->getLambda(), this->getBaselineCost(), this->stream_, false);
    }
    else
    {
      mppi::kernels::launchTsallisKernel(NUM_ROLLOUTS, this->getNormExpThreads(), this->trajectory_costs_d_, getGamma(),
                                         getRExp(), this->getBaselineCost(), this->stream_, false);
    }
    HANDLE_ERROR(cudaMemcpyAsync(this->trajectory_costs_.data(), this->trajectory_costs_d_,
                                 NUM_ROLLOUTS * sizeof(float), cudaMemcpyDeviceToHost, this->stream_));
    HANDLE_ERROR(cudaStreamSynchronize(this->stream_));
    // Compute the normalizer
    this->setNormalizer(mppi::kernels::computeNormalizer(this->trajectory_costs_.data(), NUM_ROLLOUTS));

    mppi::kernels::computeFreeEnergy(this->free_energy_statistics_.real_sys.freeEnergyMean,
                                     this->free_energy_statistics_.real_sys.freeEnergyVariance,
                                     this->free_energy_statistics_.real_sys.freeEnergyModifiedVariance,
                                     this->trajectory_costs_.data(), NUM_ROLLOUTS, this->getBaselineCost(),
                                     this->getLambda());

    // Compute the cost weighted average //TODO SUM_STRIDE is BDIM_X, but should it be its own parameter?
    this->sampler_->updateDistributionParamsFromDevice(this->trajectory_costs_d_, this->getNormalizerCost(), 0, false);

    // Transfer the new control to the host
    this->sampler_->setHostOptimalControlSequence(this->control_.data(), 0, true);
  }

  this->free_energy_statistics_.real_sys.normalizerPercent = this->getNormalizerCost() / NUM_ROLLOUTS;
  this->free_energy_statistics_.real_sys.increase =
      this->getBaselineCost() - this->free_energy_statistics_.real_sys.previousBaseline;
  smoothControlTrajectory();
  computeStateTrajectory(local_state);
  state_array zero_state = this->model_->getZeroState();
  for (int i = 0; i < this->getNumTimesteps(); i++)
  {
    // this->model_->enforceConstraints(zero_state, this->control_.col(i));
    this->control_.col(i)[1] =
        fminf(fmaxf(this->control_.col(i)[1], this->model_->control_rngs_[1].x), this->model_->control_rngs_[1].y);
  }

  // Copy back sampled trajectories
  this->copySampledControlFromDevice(false);
  if (this->getKernelChoiceAsEnum() == kernelType::USE_SINGLE_KERNEL)
  {  // copy initial state to vis initial state for use with visualizeKernel
    HANDLE_ERROR(cudaMemcpyAsync(this->vis_initial_state_d_, this->initial_state_d_, sizeof(float) * DYN_T::STATE_DIM,
                                 cudaMemcpyDeviceToDevice, this->vis_stream_));
  }
  this->copyTopControlFromDevice(true);
}

ColoredMPPI_TEMPLATE void ColoredMPPI::allocateCUDAMemory()
{
  PARENT_CLASS::allocateCUDAMemoryHelper();
}

ColoredMPPI_TEMPLATE void ColoredMPPI::computeStateTrajectory(const Eigen::Ref<const state_array>& x0)
{
  this->computeOutputTrajectoryHelper(this->output_, this->state_, x0, this->control_);
}

ColoredMPPI_TEMPLATE void ColoredMPPI::slideControlSequence(int steps)
{
  // TODO does the logic of handling control history reasonable?
  leash_jump_ = steps;
  // Save the control history
  this->saveControlHistoryHelper(steps, this->control_, this->control_history_);

  this->slideControlSequenceHelper(steps, this->control_);
}

ColoredMPPI_TEMPLATE void ColoredMPPI::smoothControlTrajectory()
{
  this->smoothControlTrajectoryHelper(this->control_, this->control_history_);
}

ColoredMPPI_TEMPLATE void ColoredMPPI::calculateSampledStateTrajectories()
{
  int num_sampled_trajectories = this->getTotalSampledTrajectories();
  // controls already copied in compute control
  if (this->getKernelChoiceAsEnum() == kernelType::USE_SPLIT_KERNELS)
  {
    mppi::kernels::launchVisualizeCostKernel<COST_T, SAMPLING_T>(
        this->cost_, this->sampler_, this->getDt(), this->getNumTimesteps(), num_sampled_trajectories,
        this->getLambda(), this->getAlpha(), this->sampled_outputs_d_, this->sampled_crash_status_d_,
        this->sampled_costs_d_, this->params_.cost_rollout_dim_, this->stream_, false);
  }
  else if (this->getKernelChoiceAsEnum() == kernelType::USE_SINGLE_KERNEL)
  {
    mppi::kernels::launchVisualizeKernel<DYN_T, COST_T, SAMPLING_T>(
        this->model_, this->cost_, this->sampler_, this->getDt(), this->getNumTimesteps(), num_sampled_trajectories,
        this->getLambda(), this->getAlpha(), this->vis_initial_state_d_, this->sampled_outputs_d_,
        this->sampled_costs_d_, this->sampled_crash_status_d_, this->params_.visualize_dim_, this->stream_, false);
  }
  // #if true
  //   mppi::kernels::launchVisualizeCostKernel<COST_T, SAMPLING_T>(
  //       this->cost_, this->sampler_, this->getDt(), this->getNumTimesteps(),
  //       num_sampled_trajectories, this->getLambda(), this->getAlpha(), this->sampled_outputs_d_,
  //       this->sampled_crash_status_d_, this->sampled_costs_d_, this->params_.cost_rollout_dim_, this->vis_stream_,
  //       false);
  // #else
  //   mppi_common::launchVisualizeCostKernel<COST_T, 128, COST_B_Y, 1>(
  //       this->cost_, this->getDt(), this->getNumTimesteps(), num_sampled_trajectories, this->getLambda(),
  //       this->getAlpha(), this->sampled_outputs_d_, this->sampled_noise_d_, this->sampled_crash_status_d_,
  //       this->control_std_dev_d_, this->sampled_costs_d_, this->vis_stream_, false);
  // #endif
  for (int i = 0; i < num_sampled_trajectories; i++)
  {
    // set initial state to the first location
    // shifted by one since we do not save the initial state
    HANDLE_ERROR(cudaMemcpyAsync(this->sampled_trajectories_[i].data(),
                                 this->sampled_outputs_d_ + i * this->getNumTimesteps() * DYN_T::OUTPUT_DIM,
                                 (this->getNumTimesteps() - 1) * DYN_T::OUTPUT_DIM * sizeof(float),
                                 cudaMemcpyDeviceToHost, this->vis_stream_));
    HANDLE_ERROR(
        cudaMemcpyAsync(this->sampled_costs_[i].data(), this->sampled_costs_d_ + (i * (this->getNumTimesteps() + 1)),
                        (this->getNumTimesteps() + 1) * sizeof(float), cudaMemcpyDeviceToHost, this->vis_stream_));
    HANDLE_ERROR(cudaMemcpyAsync(this->sampled_crash_status_[i].data(),
                                 this->sampled_crash_status_d_ + (i * this->getNumTimesteps()),
                                 this->getNumTimesteps() * sizeof(float), cudaMemcpyDeviceToHost, this->vis_stream_));
  }
  HANDLE_ERROR(cudaStreamSynchronize(this->vis_stream_));
}

#undef ColoredMPPI
