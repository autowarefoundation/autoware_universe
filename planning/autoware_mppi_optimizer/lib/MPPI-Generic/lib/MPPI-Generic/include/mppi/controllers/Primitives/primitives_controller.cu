#include <mppi/controllers/Primitives/primitives_controller.cuh>
#include <mppi/core/mppi_common.cuh>
#include <algorithm>
#include <iostream>
#include <mppi/sampling_distributions/piecewise_linear/piecewise_linear_noise.cuh>
#include <mppi/sampling_distributions/colored_noise/colored_noise.cuh>

#define Primitives                                                                                                     \
  PrimitivesController<DYN_T, COST_T, FB_T, MAX_TIMESTEPS, NUM_ROLLOUTS, BDIM_X, BDIM_Y, COST_B_X, COST_B_Y, PARAMS_T>

template <class DYN_T, class COST_T, class FB_T, int MAX_TIMESTEPS, int NUM_ROLLOUTS, int BDIM_X, int BDIM_Y,
          int COST_B_X, int COST_B_Y, class PARAMS_T>
Primitives::PrimitivesController(DYN_T* model, COST_T* cost, FB_T* fb_controller, float dt, int max_iter, float lambda,
                                 float alpha, const Eigen::Ref<const control_array>& control_std_dev, int num_timesteps,
                                 const Eigen::Ref<const control_trajectory>& init_control_traj, cudaStream_t stream)
  : PARENT_CLASS(model, cost, fb_controller, dt, max_iter, lambda, alpha, control_std_dev, num_timesteps,
                 init_control_traj, stream)
{
  // Allocate CUDA memory for the controller
  allocateCUDAMemory();
  std::vector<float> tmp_vec(DYN_T::CONTROL_DIM, 0.0);
  getColoredNoiseExponentsLValue() = std::move(tmp_vec);
  getScalePiecewiseNoiseLValue() = std::move(tmp_vec);

  // Copy the noise std_dev to the device
  this->copyControlStdDevToDevice();

  control_mppi_history_ = Eigen::Matrix<float, DYN_T::CONTROL_DIM, 2>::Zero();
}

template <class DYN_T, class COST_T, class FB_T, int MAX_TIMESTEPS, int NUM_ROLLOUTS, int BDIM_X, int BDIM_Y,
          int COST_B_X, int COST_B_Y, class PARAMS_T>
Primitives::PrimitivesController(DYN_T* model, COST_T* cost, FB_T* fb_controller, PARAMS_T& params, cudaStream_t stream)
  : PARENT_CLASS(model, cost, fb_controller, params, stream)
{
  // Allocate CUDA memory for the controller
  allocateCUDAMemory();
  if (this->getColoredNoiseExponentsLValue().size() == 0)
  {
    std::vector<float> tmp_vec(DYN_T::CONTROL_DIM, 0.0);
    getColoredNoiseExponentsLValue() = std::move(tmp_vec);
  }
  if (this->getScalePiecewiseNoiseLValue().size() == 0)
  {
    std::vector<float> tmp_vec(DYN_T::CONTROL_DIM, 0.0);
    getScalePiecewiseNoiseLValue() = std::move(tmp_vec);
  }

  // Copy the noise std_dev to the device
  this->copyControlStdDevToDevice();
}

template <class DYN_T, class COST_T, class FB_T, int MAX_TIMESTEPS, int NUM_ROLLOUTS, int BDIM_X, int BDIM_Y,
          int COST_B_X, int COST_B_Y, class PARAMS_T>
Primitives::~PrimitivesController()
{
  // all implemented in standard controller
}

template <class DYN_T, class COST_T, class FB_T, int MAX_TIMESTEPS, int NUM_ROLLOUTS, int BDIM_X, int BDIM_Y,
          int COST_B_X, int COST_B_Y, class PARAMS_T>
void Primitives::computeControl(const Eigen::Ref<const state_array>& state, int optimization_stride)
{
  // this->free_energy_statistics_.real_sys.previousBaseline = this->getBaselineCost();
  state_array local_state = state;

  if (getLeashActive())
  {
    this->model_->enforceLeash(state, this->state_.col(leash_jump_), this->params_.state_leash_dist_, local_state);
  }

  // Send the initial condition to the device
  HANDLE_ERROR(cudaMemcpyAsync(this->initial_state_d_, local_state.data(), DYN_T::STATE_DIM * sizeof(float),
                               cudaMemcpyHostToDevice, this->stream_));

  /////////////////
  // BEGIN INTERMEDIATE PLANNER
  // Compute intermediate plan using piecewise linear noise, and choosing the best

  int prev_controls_idx = 1;
  float primitives_baseline = 0.0;
  float baseline_prev = 0.0;
  int best_idx = -1;

  // Send the nominal control to the device
  this->copyNominalControlToDevice(false);

  for (int opt_iter = 0; opt_iter < getNumPrimitiveIterations(); opt_iter++)
  {
    powerlaw_psd_gaussian(getColoredNoiseExponentsLValue(), this->getNumTimesteps(), NUM_ROLLOUTS,
                          this->control_noise_d_, 0, this->gen_, this->getOffsetDecayRate(), this->stream_);

    // Generate piecewise linear noise data, update control_noise_d_
    piecewise_linear_noise(this->getNumTimesteps(), NUM_ROLLOUTS, DYN_T::CONTROL_DIM, getPiecewiseSegments(),
                           optimization_stride, getScalePiecewiseNoiseLValue(), getFracRandomNoiseTrajLValue(),
                           getScaleAddNominalNoiseLValue(), this->control_d_, this->control_noise_d_,
                           this->control_std_dev_d_, this->gen_, this->stream_);

    // // Set nominal controls to zero because we want to use the noise directly
    // this->control_ = control_trajectory::Zero();

    // // Send the zero nominal control to the device
    // this->copyNominalControlToDevice();

    // Launch the rollout kernel
    mppi_common::launchFastRolloutKernel<DYN_T, COST_T, NUM_ROLLOUTS, BDIM_X, BDIM_Y, 1, COST_B_X, COST_B_Y>(
        this->model_->model_d_, this->cost_->cost_d_, this->getDt(), this->getNumTimesteps(), optimization_stride,
        this->getLambda(), this->getAlpha(), this->initial_state_d_, this->output_d_, this->control_d_,
        this->control_noise_d_, this->control_std_dev_d_, this->trajectory_costs_d_, this->stream_, false);

    // Copy the costs back to the host
    HANDLE_ERROR(cudaMemcpyAsync(this->trajectory_costs_.data(), this->trajectory_costs_d_,
                                 NUM_ROLLOUTS * sizeof(float), cudaMemcpyDeviceToHost, this->stream_));
    HANDLE_ERROR(cudaStreamSynchronize(this->stream_));

    primitives_baseline = mppi_common::computeBaselineCost(this->trajectory_costs_.data(), NUM_ROLLOUTS);

    // get previous control cost (at index = 1, since index = 0 is zero control traj)
    baseline_prev = this->trajectory_costs_.data()[prev_controls_idx];
    if (this->debug_)
    {
      std::cerr << "Previous Baseline: " << baseline_prev << "         Baseline: " << this->getBaselineCost()
                << std::endl;
    }

    // if baseline is too high and trajectory is unsafe, create and issue a stopping trajectory
    // reminder:  baseline_ is the average cost along trajectory
    if (getStoppingCostThreshold() > 0 && primitives_baseline > getStoppingCostThreshold())
    {
      std::cerr << "Baseline is too high, issuing stopping trajectory!" << std::endl;
      computeStoppingTrajectory(local_state);
      primitives_baseline = std::numeric_limits<float>::min();
    }
    // else if (primitives_baseline > baseline_prev - getHysteresisCostThreshold())
    // {
    //   // baseline is not decreasing enough, use controls from the previous iteration
    //   if (this->debug_)
    //   {
    //     std::cout << "Not enough improvement, use prev controls." << std::endl;
    //   }
    //   HANDLE_ERROR(cudaMemcpyAsync(
    //       this->control_.data(),
    //       this->control_noise_d_ + prev_controls_idx * this->getNumTimesteps() * DYN_T::CONTROL_DIM,
    //       sizeof(float) * this->getNumTimesteps() * DYN_T::CONTROL_DIM, cudaMemcpyDeviceToHost, this->stream_));

    //   primitives_baseline = baseline_prev;
    // }
    else
    {  // otherwise, update the nominal control
      // Copy best control from device to the host
      best_idx = mppi_common::computeBestIndex(this->trajectory_costs_.data(), NUM_ROLLOUTS);
      HANDLE_ERROR(cudaMemcpyAsync(
          this->control_.data(), this->control_noise_d_ + best_idx * this->getNumTimesteps() * DYN_T::CONTROL_DIM,
          sizeof(float) * this->getNumTimesteps() * DYN_T::CONTROL_DIM, cudaMemcpyDeviceToHost, this->stream_));
    }

    this->copyNominalControlToDevice(false);

    cudaStreamSynchronize(this->stream_);
  }

  // Copy back sampled trajectories for visualization
  if (getVisualizePrimitives())
  {
    this->copySampledControlFromDevice(false);
    this->copyTopControlFromDevice(true);
  }

  //  END INTERMEDIATE PLANNER
  ////////////////

  ////////////////
  // BEGIN MPPI
  for (int opt_iter = 0; opt_iter < this->getNumIters(); opt_iter++)
  {
    // Send the nominal control to the device
    copyMPPIControlToDevice(false);

    // Generate noise data
    // const int colored_num_timesteps = (this->getNumTimesteps() > optimization_stride) ?
    //                                       this->getNumTimesteps() - optimization_stride :
    //                                       this->getNumTimesteps();
    // const int colored_stride = (this->getNumTimesteps() > optimization_stride) ? optimization_stride : 0;
    // if (colored_stride == 0)
    // {
    //   std::cout << "We tripped the fail-safe" << std::endl;
    // }
    powerlaw_psd_gaussian(getColoredNoiseExponentsLValue(), this->getNumTimesteps(), NUM_ROLLOUTS,
                          this->control_noise_d_, 0, this->gen_, this->getOffsetDecayRate(), this->stream_);
    // curandGenerateNormal(this->gen_, this->control_noise_d_, NUM_ROLLOUTS * this->getNumTimesteps() *
    // DYN_T::CONTROL_DIM,
    //                      0.0, 1.0);
    /*
    std::vector<float> noise = this->getSampledNoise();
    float mean = 0;
    for(int k = 0; k < noise.size(); k++) {
      mean += (noise[k]/noise.size());
    }

    float std_dev = 0;
    for(int k = 0; k < noise.size(); k++) {
      std_dev += powf(noise[k] - mean, 2);
    }
    std_dev = sqrt(std_dev/noise.size());
    printf("CPU 1 side N(%f, %f)\n", mean, std_dev);
     */

    // Launch the rollout kernel
    mppi_common::launchFastRolloutKernel<DYN_T, COST_T, NUM_ROLLOUTS, BDIM_X, BDIM_Y, 1, COST_B_X, COST_B_Y>(
        this->model_->model_d_, this->cost_->cost_d_, this->getDt(), this->getNumTimesteps(), optimization_stride,
        this->getLambda(), this->getAlpha(), this->initial_state_d_, this->output_d_, control_mppi_d_,
        this->control_noise_d_, this->control_std_dev_d_, this->trajectory_costs_d_, this->stream_, false);
    /*
    noise = this->getSampledNoise();
    mean = 0;
    for(int k = 0; k < noise.size(); k++) {
      mean += (noise[k]/noise.size());
    }

    std_dev = 0;
    for(int k = 0; k < noise.size(); k++) {
      std_dev += powf(noise[k] - mean, 2);
    }
    std_dev = sqrt(std_dev/noise.size());
    printf("CPU 2 side N(%f, %f)\n", mean, std_dev);
     */

    // Copy the costs back to the host
    HANDLE_ERROR(cudaMemcpyAsync(this->trajectory_costs_.data(), this->trajectory_costs_d_,
                                 NUM_ROLLOUTS * sizeof(float), cudaMemcpyDeviceToHost, this->stream_));
    HANDLE_ERROR(cudaStreamSynchronize(this->stream_));

    this->setBaseline(mppi_common::computeBaselineCost(this->trajectory_costs_.data(), NUM_ROLLOUTS));

    // if (this->getBaselineCost() > baseline_prev + 1)
    // {
    //   // TODO handle printing
    //   if (this->debug_)
    //   {
    //     std::cout << "Previous Baseline: " << baseline_prev << std::endl;
    //     std::cout << "         Baseline: " << this->getBaselineCost() << std::endl;
    //   }
    // }

    // baseline_prev = this->getBaselineCost();

    // Launch the norm exponential kernel
    if (getGamma() == 0 || getRExp() == 0)
    {
      mppi_common::launchNormExpKernel(NUM_ROLLOUTS, BDIM_X, this->trajectory_costs_d_, 1.0 / this->getLambda(),
                                       this->getBaselineCost(), this->stream_, false);
    }
    else
    {
      mppi_common::launchTsallisKernel(NUM_ROLLOUTS, BDIM_X, this->trajectory_costs_d_, getGamma(), getRExp(),
                                       this->getBaselineCost(), this->stream_, false);
    }
    HANDLE_ERROR(cudaMemcpyAsync(this->trajectory_costs_.data(), this->trajectory_costs_d_,
                                 NUM_ROLLOUTS * sizeof(float), cudaMemcpyDeviceToHost, this->stream_));
    HANDLE_ERROR(cudaStreamSynchronize(this->stream_));

    // Compute the normalizer
    this->setNormalizer(mppi_common::computeNormalizer(this->trajectory_costs_.data(), NUM_ROLLOUTS));

    mppi_common::computeFreeEnergy(this->free_energy_statistics_.real_sys.freeEnergyMean,
                                   this->free_energy_statistics_.real_sys.freeEnergyVariance,
                                   this->free_energy_statistics_.real_sys.freeEnergyModifiedVariance,
                                   this->trajectory_costs_.data(), NUM_ROLLOUTS, this->getBaselineCost(),
                                   this->getLambda());

    // Compute the cost weighted average //TODO SUM_STRIDE is BDIM_X, but should it be its own parameter?
    mppi_common::launchWeightedReductionKernel<DYN_T, NUM_ROLLOUTS, BDIM_X>(
        this->trajectory_costs_d_, this->control_noise_d_, control_mppi_d_, this->getNormalizerCost(),
        this->getNumTimesteps(), this->stream_, false);

    /*
    noise = this->getSampledNoise();
    mean = 0;
    for(int k = 0; k < noise.size(); k++) {
      mean += (noise[k]/noise.size());
    }

    std_dev = 0;
    for(int k = 0; k < noise.size(); k++) {
      std_dev += powf(noise[k] - mean, 2);
    }
    std_dev = sqrt(std_dev/noise.size());
    printf("CPU 3 side N(%f, %f)\n", mean, std_dev);
     */

    // Transfer the new control to the host
    HANDLE_ERROR(cudaMemcpyAsync(control_mppi_.data(), control_mppi_d_,
                                 sizeof(float) * this->getNumTimesteps() * DYN_T::CONTROL_DIM, cudaMemcpyDeviceToHost,
                                 this->stream_));
    cudaStreamSynchronize(this->stream_);
  }

  this->free_energy_statistics_.real_sys.normalizerPercent = this->getNormalizerCost() / NUM_ROLLOUTS;
  this->free_energy_statistics_.real_sys.increase =
      this->getBaselineCost() - this->free_energy_statistics_.real_sys.previousBaseline;

  // END MPPI
  ////////////////////////

  // decide between using the MPPI control or the primitives control
  if (this->debug_)
  {
    std::cerr << "mppi baseline: " << this->getBaselineCost() << ", primitives baseline: " << primitives_baseline
              << ", prev baseline: " << baseline_prev << std::endl;
  }
  if ((getNumPrimitiveIterations() == 0 && this->getNumIters() > 0) ||
      ((getNumPrimitiveIterations() > 0 && this->getNumIters() > 0) &&
       (this->getBaselineCost() < primitives_baseline + getHysteresisCostThreshold())))
  {
    this->control_ = control_mppi_;
    this->copyNominalControlToDevice();
    if (this->debug_)
    {
      std::cout << "Using MPPI control" << std::endl;
    }
    this->free_energy_statistics_.nominal_state_used = 0;
  }
  else
  {
    // control_mppi_ = this->control_; // don't do this, we want to save the MPPI control
    if (this->debug_)
    {
      std::cout << "Using primitives control, ";
    }
    if (best_idx > 0 && best_idx <= int((getFracRandomNoiseTrajLValue())[0] * NUM_ROLLOUTS))
    {
      if (this->debug_)
      {
        std::cout << "colored noise added to nominal." << std::endl;
      }
      this->free_energy_statistics_.nominal_state_used = 1;
    }
    else if (best_idx <= int((getFracRandomNoiseTrajLValue()[0] + getFracRandomNoiseTrajLValue()[1]) * NUM_ROLLOUTS))
    {
      if (this->debug_)
      {
        std::cout << "piecewise noise added to nominal." << std::endl;
      }
      this->free_energy_statistics_.nominal_state_used = 2;
    }
    else
    {
      if (this->debug_)
      {
        std::cout << "new piecewise trajectory." << std::endl;
      }
      this->free_energy_statistics_.nominal_state_used = 3;
    }
  }

  smoothControlTrajectory();
  computeStateTrajectory(local_state);
  state_array zero_state = state_array::Zero();
  for (int i = 0; i < this->getNumTimesteps(); i++)
  {
    this->model_->enforceConstraints(zero_state, this->control_.col(i));
    this->model_->enforceConstraints(zero_state, control_mppi_.col(i));
    // this->control_.col(i)[1] =
    //     fminf(fmaxf(this->control_.col(i)[1], this->model_->control_rngs_[1].x), this->model_->control_rngs_[1].y);
  }

  // Copy back sampled trajectories for visualization
  if (!getVisualizePrimitives())
  {
    this->copySampledControlFromDevice(false);
    this->copyTopControlFromDevice(true);
  }
}

template <class DYN_T, class COST_T, class FB_T, int MAX_TIMESTEPS, int NUM_ROLLOUTS, int BDIM_X, int BDIM_Y,
          int COST_B_X, int COST_B_Y, class PARAMS_T>
void Primitives::allocateCUDAMemory()
{
  PARENT_CLASS::allocateCUDAMemoryHelper();
  HANDLE_ERROR(cudaMalloc((void**)&control_mppi_d_, sizeof(float) * DYN_T::CONTROL_DIM * MAX_TIMESTEPS));
}

template <class DYN_T, class COST_T, class FB_T, int MAX_TIMESTEPS, int NUM_ROLLOUTS, int BDIM_X, int BDIM_Y,
          int COST_B_X, int COST_B_Y, class PARAMS_T>
void Primitives::copyMPPIControlToDevice(bool synchronize)
{
  HANDLE_ERROR(cudaMemcpyAsync(control_mppi_d_, control_mppi_.data(), sizeof(float) * control_mppi_.size(),
                               cudaMemcpyHostToDevice, this->stream_));
  if (synchronize)
  {
    HANDLE_ERROR(cudaStreamSynchronize(this->stream_));
  }
}

template <class DYN_T, class COST_T, class FB_T, int MAX_TIMESTEPS, int NUM_ROLLOUTS, int BDIM_X, int BDIM_Y,
          int COST_B_X, int COST_B_Y, class PARAMS_T>
void Primitives::computeStateTrajectory(const Eigen::Ref<const state_array>& x0)
{
  this->computeStateTrajectoryHelper(this->state_, x0, this->control_);
}

template <class DYN_T, class COST_T, class FB_T, int MAX_TIMESTEPS, int NUM_ROLLOUTS, int BDIM_X, int BDIM_Y,
          int COST_B_X, int COST_B_Y, class PARAMS_T>
void Primitives::computeStoppingTrajectory(const Eigen::Ref<const state_array>& x0)
{
  state_array xdot;
  state_array state = x0;
  state_array xnext;
  output_array output;
  control_array u_i = control_array::Zero();
  this->model_->initializeDynamics(state, u_i, output, 0, this->getDt());
  for (int i = 0; i < this->getNumTimesteps() - 1; ++i)
  {
    this->model_->getStoppingControl(state, u_i);
    this->model_->enforceConstraints(state, u_i);
    this->control_.col(i) = u_i;
    this->model_->step(state, xnext, xdot, u_i, output, i, this->getDt());
    state = xnext;
  }
}

template <class DYN_T, class COST_T, class FB_T, int MAX_TIMESTEPS, int NUM_ROLLOUTS, int BDIM_X, int BDIM_Y,
          int COST_B_X, int COST_B_Y, class PARAMS_T>
void Primitives::slideControlSequence(int steps)
{
  // TODO does the logic of handling control history reasonable?
  leash_jump_ = steps;
  // Save the control history
  this->saveControlHistoryHelper(steps, this->control_, this->control_history_);
  this->saveControlHistoryHelper(steps, control_mppi_, control_mppi_history_);

  this->slideControlSequenceHelper(steps, this->control_);
  this->slideControlSequenceHelper(steps, control_mppi_);
}

template <class DYN_T, class COST_T, class FB_T, int MAX_TIMESTEPS, int NUM_ROLLOUTS, int BDIM_X, int BDIM_Y,
          int COST_B_X, int COST_B_Y, class PARAMS_T>
void Primitives::smoothControlTrajectory()
{
  this->smoothControlTrajectoryHelper(this->control_, this->control_history_);
  this->smoothControlTrajectoryHelper(control_mppi_, control_mppi_history_);
}

template <class DYN_T, class COST_T, class FB_T, int MAX_TIMESTEPS, int NUM_ROLLOUTS, int BDIM_X, int BDIM_Y,
          int COST_B_X, int COST_B_Y, class PARAMS_T>
void Primitives::calculateSampledStateTrajectories()
{
  int num_sampled_trajectories = this->getTotalSampledTrajectories();
  // controls already copied in compute control

  mppi_common::launchStateAndCostTrajectoryKernel<DYN_T, COST_T, FEEDBACK_GPU, BDIM_X, BDIM_Y>(
      this->model_->model_d_, this->cost_->cost_d_, this->fb_controller_->getDevicePointer(), this->sampled_noise_d_,
      this->initial_state_d_, this->sampled_outputs_d_, this->sampled_costs_d_, this->sampled_crash_status_d_,
      num_sampled_trajectories, this->getNumTimesteps(), this->getDt(), this->vis_stream_);

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

#undef Primitives
