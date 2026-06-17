#include "tube_mppi_controller.cuh"
#include <mppi/core/mppi_common.cuh>

#define TUBE_MPPI_TEMPLATE                                                                                             \
  template <class DYN_T, class COST_T, class FB_T, int MAX_TIMESTEPS, int NUM_ROLLOUTS, class SAMPLING_T,              \
            class PARAMS_T>

#define TubeMPPI TubeMPPIController<DYN_T, COST_T, FB_T, MAX_TIMESTEPS, NUM_ROLLOUTS, SAMPLING_T, PARAMS_T>

TUBE_MPPI_TEMPLATE
TubeMPPI::TubeMPPIController(DYN_T* model, COST_T* cost, FB_T* fb_controller, SAMPLING_T* sampler, float dt,
                             int max_iter, float lambda, float alpha, int num_timesteps,
                             const Eigen::Ref<const control_trajectory>& init_control_traj, cudaStream_t stream)
  : PARENT_CLASS(model, cost, fb_controller, sampler, dt, max_iter, lambda, alpha, num_timesteps, init_control_traj,
                 stream)
{
  nominal_control_trajectory_ = init_control_traj;

  // call rollout kernel with z = 2 since we have a nominal state
  this->params_.dynamics_rollout_dim_.z = max(2, this->params_.dynamics_rollout_dim_.z);
  this->params_.cost_rollout_dim_.z = max(2, this->params_.cost_rollout_dim_.z);
  this->sampler_->setNumDistributions(2);

  // Allocate CUDA memory for the controller
  allocateCUDAMemory();

  // Initialize Feedback
  this->fb_controller_->initTrackingController();
  this->enable_feedback_ = true;
  chooseAppropriateKernel();
}

TUBE_MPPI_TEMPLATE
TubeMPPI::TubeMPPIController(DYN_T* model, COST_T* cost, FB_T* fb_controller, SAMPLING_T* sampler, PARAMS_T& params,
                             cudaStream_t stream)
  : PARENT_CLASS(model, cost, fb_controller, sampler, params, stream)
{
  nominal_control_trajectory_ = this->params_.init_control_traj_;

  // call rollout kernel with z = 2 since we have a nominal state
  this->params_.dynamics_rollout_dim_.z = max(2, this->params_.dynamics_rollout_dim_.z);
  this->params_.cost_rollout_dim_.z = max(2, this->params_.cost_rollout_dim_.z);
  this->sampler_->setNumDistributions(2);

  // Allocate CUDA memory for the controller
  allocateCUDAMemory();

  // Initialize Feedback
  this->fb_controller_->initTrackingController();
  this->enable_feedback_ = true;
  chooseAppropriateKernel();
}

TUBE_MPPI_TEMPLATE
void TubeMPPI::chooseAppropriateKernel()
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
  HANDLE_ERROR(cudaMemcpyAsync(this->initial_state_d_ + DYN_T::STATE_DIM, zero_state.data(),
                               DYN_T::STATE_DIM * sizeof(float), cudaMemcpyHostToDevice, this->stream_));
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

TUBE_MPPI_TEMPLATE
void TubeMPPI::computeControl(const Eigen::Ref<const state_array>& state, int optimization_stride)
{
  if (!nominalStateInit_)
  {
    // set the nominal state to the actual state
    nominal_state_trajectory_.col(0) = state;
    nominalStateInit_ = true;
  }

  this->free_energy_statistics_.real_sys.previousBaseline = this->getBaselineCost(0);
  this->free_energy_statistics_.nominal_sys.previousBaseline = this->getBaselineCost(1);

  //  std::cout << "Post disturbance Actual State: "; this->model_->printState(state.data());
  //  std::cout << "                Nominal State: "; this->model_->printState(nominal_state_trajectory_.col(0).data());

  // Handy reference pointers to the nominal state
  float* trajectory_costs_nominal_d = this->trajectory_costs_d_ + NUM_ROLLOUTS;
  float* initial_state_nominal_d = this->initial_state_d_ + DYN_T::STATE_DIM;

  for (int opt_iter = 0; opt_iter < this->getNumIters(); opt_iter++)
  {
    // Send the initial condition to the device
    HANDLE_ERROR(cudaMemcpyAsync(this->initial_state_d_, state.data(), DYN_T::STATE_DIM * sizeof(float),
                                 cudaMemcpyHostToDevice, this->stream_));
    HANDLE_ERROR(cudaMemcpyAsync(initial_state_nominal_d, nominal_state_trajectory_.data(),
                                 DYN_T::STATE_DIM * sizeof(float), cudaMemcpyHostToDevice, this->stream_));

    // Send the nominal control to the device
    copyControlToDevice(false);

    // Generate noise data
    this->sampler_->generateSamples(optimization_stride, opt_iter, this->gen_, false);

    // call rollout kernel with z = 2 since we have a nominal state
    this->params_.dynamics_rollout_dim_.z = max(2, this->params_.dynamics_rollout_dim_.z);
    this->params_.cost_rollout_dim_.z = max(2, this->params_.cost_rollout_dim_.z);

    // Launch the rollout kernel
    if (this->getKernelChoiceAsEnum() == kernelType::USE_SPLIT_KERNELS)
    {
      mppi::kernels::launchSplitRolloutKernel<DYN_T, COST_T, SAMPLING_T>(
          this->model_, this->cost_, this->sampler_, this->getDt(), this->getNumTimesteps(), NUM_ROLLOUTS,
          this->getLambda(), this->getAlpha(), this->initial_state_d_, this->output_d_, this->trajectory_costs_d_,
          this->params_.dynamics_rollout_dim_, this->params_.cost_rollout_dim_, this->stream_, false);
    }
    else
    {
      mppi::kernels::launchRolloutKernel<DYN_T, COST_T, SAMPLING_T>(
          this->model_, this->cost_, this->sampler_, this->getDt(), this->getNumTimesteps(), NUM_ROLLOUTS,
          this->getLambda(), this->getAlpha(), this->initial_state_d_, this->trajectory_costs_d_,
          this->params_.dynamics_rollout_dim_, this->stream_, false);
    }

    // Copy the costs back to the host
    HANDLE_ERROR(cudaMemcpyAsync(this->trajectory_costs_.data(), this->trajectory_costs_d_,
                                 NUM_ROLLOUTS * sizeof(float), cudaMemcpyDeviceToHost, this->stream_));

    HANDLE_ERROR(cudaMemcpyAsync(trajectory_costs_nominal_.data(), trajectory_costs_nominal_d,
                                 NUM_ROLLOUTS * sizeof(float), cudaMemcpyDeviceToHost, this->stream_));
    HANDLE_ERROR(cudaStreamSynchronize(this->stream_));

    this->setBaseline(mppi::kernels::computeBaselineCost(this->trajectory_costs_.data(), NUM_ROLLOUTS), 0);
    this->setBaseline(mppi::kernels::computeBaselineCost(this->trajectory_costs_nominal_.data(), NUM_ROLLOUTS), 1);

    // Launch the norm exponential kernel for both actual and nominal
    mppi::kernels::launchNormExpKernel(NUM_ROLLOUTS, this->getNormExpThreads(), this->trajectory_costs_d_,
                                       1.0 / this->getLambda(), this->getBaselineCost(0), this->stream_, false);

    mppi::kernels::launchNormExpKernel(NUM_ROLLOUTS, this->getNormExpThreads(), trajectory_costs_nominal_d,
                                       1.0 / this->getLambda(), this->getBaselineCost(1), this->stream_, false);

    HANDLE_ERROR(cudaMemcpyAsync(this->trajectory_costs_.data(), this->trajectory_costs_d_,
                                 NUM_ROLLOUTS * sizeof(float), cudaMemcpyDeviceToHost, this->stream_));
    HANDLE_ERROR(cudaMemcpyAsync(trajectory_costs_nominal_.data(), trajectory_costs_nominal_d,
                                 NUM_ROLLOUTS * sizeof(float), cudaMemcpyDeviceToHost, this->stream_));
    HANDLE_ERROR(cudaStreamSynchronize(this->stream_));

    // Compute the normalizer
    this->setNormalizer(mppi::kernels::computeNormalizer(this->trajectory_costs_.data(), NUM_ROLLOUTS), 0);
    this->setNormalizer(mppi::kernels::computeNormalizer(this->trajectory_costs_nominal_.data(), NUM_ROLLOUTS), 1);

    // Compute real free energy
    mppi::kernels::computeFreeEnergy(this->free_energy_statistics_.real_sys.freeEnergyMean,
                                     this->free_energy_statistics_.real_sys.freeEnergyVariance,
                                     this->free_energy_statistics_.real_sys.freeEnergyModifiedVariance,
                                     this->trajectory_costs_.data(), NUM_ROLLOUTS, this->getBaselineCost(0),
                                     this->getLambda());

    // Compute Nominal State free Energy
    mppi::kernels::computeFreeEnergy(this->free_energy_statistics_.nominal_sys.freeEnergyMean,
                                     this->free_energy_statistics_.nominal_sys.freeEnergyVariance,
                                     this->free_energy_statistics_.nominal_sys.freeEnergyModifiedVariance,
                                     this->trajectory_costs_nominal_.data(), NUM_ROLLOUTS, this->getBaselineCost(1),
                                     this->getLambda());

    // Compute the cost weighted average
    this->sampler_->updateDistributionParamsFromDevice(this->trajectory_costs_d_, this->getNormalizerCost(0), 0, false);
    this->sampler_->updateDistributionParamsFromDevice(trajectory_costs_nominal_d, this->getNormalizerCost(1), 1,
                                                       false);

    // Transfer the new control to the host
    this->sampler_->setHostOptimalControlSequence(this->control_.data(), 0, false);
    this->sampler_->setHostOptimalControlSequence(this->nominal_control_trajectory_.data(), 1, true);

    // Compute the nominal and actual state trajectories
    computeStateTrajectory(state);  // Input is the actual state

    // this->logger_->debug("Actual baseline: %f\n", this->getBaselineCost(0));
    // this->logger_->debug("Nominal baseline: %f\n", this->getBaselineCost(1));

    if (this->getBaselineCost(0) < this->getBaselineCost(1) + getNominalThreshold())
    {
      // In this case, the disturbance the made the nominal and actual states differ improved the cost.
      // std::copy(state_trajectory.begin(), state_trajectory.end(), nominal_state_trajectory_.begin());
      // std::copy(control_trajectory.begin(), control_trajectory.end(), nominal_control_.begin());
      this->free_energy_statistics_.nominal_state_used = 0;
      nominal_state_trajectory_ = this->state_;
      nominal_control_trajectory_ = this->control_;
    }
    else
    {
      this->free_energy_statistics_.nominal_state_used = 1;
    }

    // Outside of this loop, we will utilize the nominal state trajectory and the nominal control trajectory to compute
    // the optimal feedback gains using our ancillary controller, then apply feedback inside our main while loop at the
    // same rate as our state estimator.
  }
  smoothControlTrajectory();
  computeStateTrajectory(state);  // Input is the actual state

  this->free_energy_statistics_.real_sys.normalizerPercent = this->getNormalizerCost(0) / NUM_ROLLOUTS;
  this->free_energy_statistics_.real_sys.increase =
      this->getBaselineCost(0) - this->free_energy_statistics_.real_sys.previousBaseline;
  this->free_energy_statistics_.nominal_sys.normalizerPercent = this->getNormalizerCost(1) / NUM_ROLLOUTS;
  this->free_energy_statistics_.nominal_sys.increase =
      this->getBaselineCost(1) - this->free_energy_statistics_.nominal_sys.previousBaseline;

  // Copy back sampled trajectories
  this->copySampledControlFromDevice(false);
  this->copyTopControlFromDevice(true);
}

TUBE_MPPI_TEMPLATE
void TubeMPPI::copyControlToDevice(bool synchronize)
{
  this->sampler_->copyImportanceSamplerToDevice(this->control_.data(), 0, false);
  this->sampler_->copyImportanceSamplerToDevice(this->nominal_control_trajectory_.data(), 1, synchronize);
}

TUBE_MPPI_TEMPLATE
void TubeMPPI::allocateCUDAMemory()
{
  PARENT_CLASS::allocateCUDAMemoryHelper(1);
}

TUBE_MPPI_TEMPLATE
void TubeMPPI::slideControlSequence(int steps)
{
  // Propagate the nominal trajectory forward
  updateNominalState(nominal_control_trajectory_.col(0));

  // Save the control history
  this->saveControlHistoryHelper(steps, nominal_control_trajectory_, this->control_history_);

  this->slideControlSequenceHelper(steps, nominal_control_trajectory_);
  this->slideControlSequenceHelper(steps, this->control_);
}

TUBE_MPPI_TEMPLATE
void TubeMPPI::smoothControlTrajectory()
{
  this->smoothControlTrajectoryHelper(nominal_control_trajectory_, this->control_history_);
}

TUBE_MPPI_TEMPLATE
void TubeMPPI::computeStateTrajectory(const Eigen::Ref<const state_array>& x0_actual)
{
  // update the nominal state
  this->computeStateTrajectoryHelper(nominal_state_trajectory_, nominal_state_trajectory_.col(0),
                                     nominal_control_trajectory_);
  // update the actual state
  this->computeStateTrajectoryHelper(this->state_, x0_actual, this->control_);
}

TUBE_MPPI_TEMPLATE
void TubeMPPI::updateNominalState(const Eigen::Ref<const control_array>& u)
{
  state_array xdot;
  output_array output;
  this->model_->step(nominal_state_trajectory_.col(0), nominal_state_trajectory_.col(0), xdot, u, output, 0,
                     this->getDt());
}

TUBE_MPPI_TEMPLATE
void TubeMPPI::calculateSampledStateTrajectories()
{
  int num_sampled_trajectories = this->getTotalSampledTrajectories();
  // control already copied in compute control, so run kernel
  mppi::kernels::launchVisualizeCostKernel<COST_T, SAMPLING_T>(
      this->cost_->cost_d_, this->sampler_->sampling_d_, this->getDt(), this->getNumTimesteps(),
      num_sampled_trajectories, this->getLambda(), this->getAlpha(), this->sampled_outputs_d_,
      this->sampled_crash_status_d_, this->sampled_costs_d_, this->params_.cost_rollout_dim_, this->stream_, false);

  // copy back results
  for (int i = 0; i < num_sampled_trajectories * 2; i++)
  {
    HANDLE_ERROR(cudaMemcpyAsync(this->sampled_trajectories_[i].data(),
                                 this->sampled_outputs_d_ + i * this->getNumTimesteps() * DYN_T::OUTPUT_DIM,
                                 this->getNumTimesteps() * DYN_T::OUTPUT_DIM * sizeof(float), cudaMemcpyDeviceToHost,
                                 this->vis_stream_));
  }
  HANDLE_ERROR(cudaMemcpyAsync(this->sampled_costs_.data(), this->sampled_costs_d_,
                               this->getNumTimesteps() * 2 * sizeof(float), cudaMemcpyDeviceToHost, this->vis_stream_));
  HANDLE_ERROR(cudaMemcpyAsync(this->sampled_crash_status_.data(), this->sampled_crash_status_d_,
                               this->getNumTimesteps() * 2 * sizeof(float), cudaMemcpyDeviceToHost, this->vis_stream_));
  HANDLE_ERROR(cudaStreamSynchronize(this->vis_stream_));
}
