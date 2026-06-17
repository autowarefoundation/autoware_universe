/*
 * Software License Agreement (BSD License)
 * Copyright (c) 2013, Georgia Institute of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/**********************************************
 * @file mppi_controller.cuh
 * @author Grady Williams <gradyrw@gmail.com>
 * @date May 24, 2017
 * @copyright 2017 Georgia Institute of Technology
 * @brief Class definition for the MPPI controller.
 ***********************************************/

#ifndef MPPI_GEN_RMPPI_CONTROLLER_CUH_
#define MPPI_GEN_RMPPI_CONTROLLER_CUH_

#include <curand.h>
#include <mppi/controllers/controller.cuh>
#include <mppi/sampling_distributions/gaussian/gaussian.cuh>
#include <mppi/core/rmppi_kernels.cuh>

// Needed for list of candidate states
#include <mppi/ddp/util.h>

template <int S_DIM, int C_DIM, int MAX_TIMESTEPS>
struct RobustMPPIParams : public ControllerParams<S_DIM, C_DIM, MAX_TIMESTEPS>
{
  float value_function_threshold_ = 1000.0;
  int optimization_stride_ = 1;
  int num_candidate_nominal_states_ = 9;
  dim3 eval_cost_kernel_dim_;
  dim3 eval_dyn_kernel_dim_;
};

// template <class DYN_T, class COST_T, class FB_T, int MAX_TIMESTEPS, int NUM_ROLLOUTS = 2560, int BDIM_X = 64,
//           int BDIM_Y = 1, class PARAMS_T = RobustMPPIParams<DYN_T::STATE_DIM, DYN_T::CONTROL_DIM, MAX_TIMESTEPS>,
//           int SAMPLES_PER_CONDITION_MULTIPLIER = 1>
template <class DYN_T, class COST_T, class FB_T, int MAX_TIMESTEPS, int NUM_ROLLOUTS = 2560,
          class SAMPLING_T = ::mppi::sampling_distributions::GaussianDistribution<typename DYN_T::DYN_PARAMS_T>,
          class PARAMS_T = RobustMPPIParams<DYN_T::STATE_DIM, DYN_T::CONTROL_DIM, MAX_TIMESTEPS>>
class RobustMPPIController : public Controller<DYN_T, COST_T, FB_T, SAMPLING_T, MAX_TIMESTEPS, NUM_ROLLOUTS, PARAMS_T>
{
public:
  /**
   * Set up useful types
   */
  typedef Controller<DYN_T, COST_T, FB_T, SAMPLING_T, MAX_TIMESTEPS, NUM_ROLLOUTS, PARAMS_T> PARENT_CLASS;

  using control_array = typename PARENT_CLASS::control_array;
  using control_trajectory = typename PARENT_CLASS::control_trajectory;
  using state_trajectory = typename PARENT_CLASS::state_trajectory;
  using state_array = typename PARENT_CLASS::state_array;
  using sampled_cost_traj = typename PARENT_CLASS::sampled_cost_traj;
  using FEEDBACK_STATE = typename PARENT_CLASS::TEMPLATED_FEEDBACK_STATE;
  using FEEDBACK_PARAMS = typename PARENT_CLASS::TEMPLATED_FEEDBACK_PARAMS;
  using FEEDBACK_GPU = typename PARENT_CLASS::TEMPLATED_FEEDBACK_GPU;
  using NominalCandidateVector = typename util::NamedEigenAlignedVector<state_array>;

  static const int STATE_DIM = DYN_T::STATE_DIM;
  static const int CONTROL_DIM = DYN_T::CONTROL_DIM;

  // Number of samples per condition must be a multiple of the blockDIM
  // static const int SAMPLES_PER_CANDIDATE = SAMPLES_PER_CANDIDATE_MULTIPLIER;

  int getNumEvalSamplesPerCandidate() const
  {
    return this->params_.eval_dyn_kernel_dim_.x;
  }

  // float value_function_threshold_ = 1000.0;

  state_array nominal_state_ = state_array::Zero();

  /**
   * @brief Constructor for mppi controller class
   * @param model A basis function model of the system dynamics.
   * @param cost An MppiCost object.
   * @param dt The time increment. horizon = num_timesteps*dt
   * @param max_iter number of times to repeat control sequence calculations
   * @param gamma
   * @param num_timesteps The number of timesteps to look ahead for.
   * TODO Finish this description
   */
  RobustMPPIController(DYN_T* model, COST_T* cost, FB_T* fb_controller, SAMPLING_T* sampler, float dt, int max_iter,
                       float lambda, float alpha, float value_function_threshold, int num_timesteps = MAX_TIMESTEPS,
                       const Eigen::Ref<const control_trajectory>& init_control_traj = control_trajectory::Zero(),
                       int num_candidate_nominal_states = 9, int optimization_stride = 1,
                       cudaStream_t stream = nullptr);

  RobustMPPIController(DYN_T* model, COST_T* cost, FB_T* fb_controller, SAMPLING_T* sampler, PARAMS_T& params,
                       cudaStream_t stream = nullptr);

  /**
   * @brief Destructor for mppi controller class.
   */
  ~RobustMPPIController();

  std::string getControllerName()
  {
    return "Robust MPPI";
  };

  // Initializes the num_candidates, candidate_nominal_states, linesearch_weights,
  // and allocates the associated CUDA memory
  void updateNumCandidates(int new_num_candidates);

  // Update the importance sampler prior to calling computeControl
  void updateImportanceSamplingControl(const Eigen::Ref<const state_array>& state, int stride);

  /**
   * @brief Compute the control given the current state of the system.
   * @param state The current state of the autorally system.
   */
  void computeControl(const Eigen::Ref<const state_array>& state, int optimization_stride = 1);

  control_trajectory getControlSeq() const override
  {
    return this->control_;
  };

  state_trajectory getTargetStateSeq() const override
  {
    return nominal_state_trajectory_;
  };

  state_array getNominalState() const
  {
    return nominal_state_;
  }

  int getBestIndex() const
  {
    return best_index_;
  };

  float getValueFunctionThreshold() const
  {
    return this->params_.value_function_threshold_;
  }

  int getOptimizationStride() const
  {
    return this->params_.optimization_stride_;
  }

  int getNumCandidates() const
  {
    return this->params_.num_candidate_nominal_states_;
  }

  int getNumEvalRollouts() const
  {
    return getNumCandidates() * getNumEvalSamplesPerCandidate();
  }

  int getEvalKernelChoiceAsInt() const
  {
    return static_cast<int>(use_eval_kernel_);
  }

  kernelType getEvalKernelChoiceAsEnum() const
  {
    return use_eval_kernel_;
  }

  // Does nothing. This reason is because the control sliding happens during the importance sampler update.
  // The control applied to the real system (during the MPPI rollouts) is the nominal control (which slides
  // during the importance sampler update), plus the feedback term. Inside the runControlIteration function
  // slideControl sequence is called prior to optimization, after the importance sampler update.
  void slideControlSequence(int steps) override{};

  // Feedback gain computation is done after the importance sampling update. The nominal trajectory computed
  // during the importance sampling update does not change after the optimization, thus the feedback gains will
  // not change either. In the current implementation of runControlIteration, the compute feedback gains is called
  // after the computation of the optimal control.
  void computeFeedback(const Eigen::Ref<const state_array>& state) override{};

  Eigen::MatrixXf getCandidateFreeEnergy()
  {
    return candidate_free_energy_;
  };

  float computeDF();

  void setPercentageSampledControlTrajectories(float new_perc)
  {
    this->setPercentageSampledControlTrajectoriesHelper(new_perc, 2);
  }

  void setValueFunctionThreshold(float value_function_threshold)
  {
    this->params_.value_function_threshold_ = value_function_threshold;
  }

  void setOptimizationStride(float optimization_stride)
  {
    this->params_.optimization_stride_ = optimization_stride;
  }

  void setNumCandidates(int num_candidate_nominal_states)
  {
    this->params_.num_candidate_nominal_states_ = num_candidate_nominal_states;
  }

  void setNumSamplesPerCandidate(const int& num_samples)
  {
    this->params_.eval_dyn_kernel_dim_.x = num_samples;
    updateCandidateMemory();
  }

  void setEvalKernelChoice(const int& kernel_type)
  {
    use_eval_kernel_ = static_cast<kernelType>(kernel_type);
  }

  void setEvalKernelChoice(const kernelType& kernel_type)
  {
    use_eval_kernel_ = kernel_type;
  }

  void setParams(const PARAMS_T& p);

  void calculateSampledStateTrajectories() override;

  void chooseAppropriateKernel() override;

  void chooseAppropriateEvalKernel();

protected:
  bool importance_sampling_cuda_mem_init_ = false;
  // int num_candidate_nominal_states_;
  int best_index_ = 0;  // Selected nominal state candidate
  // int optimization_stride_;  // Number of timesteps to apply the optimal control (== 1 for true MPC)
  int nominal_stride_ = 0;  // Stride for the chosen nominal state of the importance sampler
  int real_stride_ = 0;     // Stride for the optimal controller sliding
  bool nominal_state_init_ = false;

  // Free energy variables
  float nominal_free_energy_mean_ = 0;
  float nominal_free_energy_variance_ = 0;
  float nominal_free_energy_modified_variance_ = 0;

  // Storage classes
  control_trajectory nominal_control_trajectory_ = control_trajectory::Zero();
  state_trajectory nominal_state_trajectory_ = state_trajectory::Zero();
  sampled_cost_traj trajectory_costs_nominal_ = sampled_cost_traj::Zero();

  // Make the control history size flexible, related to issue #30
  Eigen::Matrix<float, DYN_T::CONTROL_DIM, 2> nominal_control_history_;  // History used for nominal_state IS

  NominalCandidateVector candidate_nominal_states_ = { state_array::Zero() };
  Eigen::MatrixXf line_search_weights_;         // At minimum there must be 3 candidates
  Eigen::MatrixXi importance_sampler_strides_;  // Time index where control trajectory starts for each nominal state
                                                // candidate
  Eigen::MatrixXf candidate_trajectory_costs_;
  Eigen::MatrixXf candidate_free_energy_;

  void allocateCUDAMemory();

  void copyNominalControlToDevice(bool synchronize = true);

  void deallocateNominalStateCandidateMemory();

  void updateCandidateMemory();

  void resetCandidateCudaMem();

  void getInitNominalStateCandidates(const Eigen::Ref<const state_array>& nominal_x_k,
                                     const Eigen::Ref<const state_array>& nominal_x_kp1,
                                     const Eigen::Ref<const state_array>& real_x_kp1);

  // compute the line search weights
  void computeLineSearchWeights();

  void computeNominalStateAndStride(const Eigen::Ref<const state_array>& state, int stride);

  // compute the importance sampler strides
  void computeImportanceSamplerStride(int stride);

  // Compute the baseline of the candidates
  float computeCandidateBaseline();

  // Get the best index based on the candidate free energy
  void computeBestIndex();

  // Computes and saves the feedback gains used in the rollout kernel and tracking.
  virtual void computeNominalFeedbackGains(const Eigen::Ref<const state_array>& state);

  // CUDA Memory
  float* importance_sampling_costs_d_ = nullptr;
  float* importance_sampling_outputs_d_ = nullptr;
  float* importance_sampling_states_d_ = nullptr;
  int* importance_sampling_strides_d_ = nullptr;
  float* feedback_gain_array_d_ = nullptr;
  kernelType use_eval_kernel_ = kernelType::USE_SPLIT_KERNELS;
};

#if __CUDACC__
#include "robust_mppi_controller.cu"
#endif

#endif /* MPPI_GEN_RMPPI_CONTROLLER_CUH_ */
