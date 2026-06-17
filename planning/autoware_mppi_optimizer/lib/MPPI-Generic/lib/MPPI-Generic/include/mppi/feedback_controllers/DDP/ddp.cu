#include <mppi/feedback_controllers/DDP/ddp.cuh>

template <class GPU_FB_T, class DYN_T, int NUM_TIMESTEPS>
DeviceDDPImpl<GPU_FB_T, DYN_T, NUM_TIMESTEPS>::DeviceDDPImpl(int num_timesteps, cudaStream_t stream)
  : num_timesteps_(num_timesteps)
  , GPUFeedbackController<GPU_FB_T, DYN_T, DDPFeedbackState<DYN_T, NUM_TIMESTEPS>>(stream)
{
}

template <class GPU_FB_T, class DYN_T, int NUM_TIMESTEPS>
__device__ void DeviceDDPImpl<GPU_FB_T, DYN_T, NUM_TIMESTEPS>::k(const float* __restrict__ x_act,
                                                                 const float* __restrict__ x_goal, const int t,
                                                                 float* __restrict__ theta,
                                                                 float* __restrict__ control_output)
{
  float* fb_gain_t = &(this->state_.fb_gain_traj_[DYN_T::STATE_DIM * DYN_T::CONTROL_DIM * t]);
  float e = 0;
  for (int i = 0; i < DYN_T::STATE_DIM; i++)
  {
    e = x_act[i] - x_goal[i];
    if (DYN_T::CONTROL_DIM % 4 == 0)
    {  // load 4 floats in at a time to save on global memory reads
      float4* fb_gain_t4 = reinterpret_cast<float4*>(&fb_gain_t[i * DYN_T::CONTROL_DIM]);
      for (int j = 0; j < DYN_T::CONTROL_DIM / 4; j++)
      {
        reinterpret_cast<float4*>(control_output)[j] = fb_gain_t4[j] * e;
      }
    }
    else if (DYN_T::CONTROL_DIM % 2 == 0)
    {  // load 2 floats in at a time to save on global memory reads
      float2* fb_gain_t2 = reinterpret_cast<float2*>(&fb_gain_t[i * DYN_T::CONTROL_DIM]);
      for (int j = 0; j < DYN_T::CONTROL_DIM / 2; j++)
      {
        reinterpret_cast<float2*>(control_output)[j] = fb_gain_t2[j] * e;
      }
    }
    else
    {
      for (int j = 0; j < DYN_T::CONTROL_DIM; j++)
      {
        control_output[j] += fb_gain_t[i * DYN_T::CONTROL_DIM + j] * e;
      }
    }
  }
}

/**
 * CPU Class for DDP Methods
 */
template <class DYN_T, int NUM_TIMESTEPS>
DDPFeedback<DYN_T, NUM_TIMESTEPS>::DDPFeedback(DYN_T* model, float dt, int num_timesteps, cudaStream_t stream)
{
  model_ = model;
  this->dt_ = dt;
  this->num_timesteps_ = std::max(num_timesteps, NUM_TIMESTEPS);
  this->gpu_controller_->freeCudaMem();  // Remove allocated CUDA mem from default constructor
  this->gpu_controller_ = std::make_shared<DeviceDDP<DYN_T, NUM_TIMESTEPS>>(this->num_timesteps_, stream);
}

template <class DYN_T, int NUM_TIMESTEPS>
void DDPFeedback<DYN_T, NUM_TIMESTEPS>::initTrackingController()
{
  util::DefaultLogger logger;
  bool verbose = false;
  ddp_model_ = std::make_shared<ModelWrapperDDP<DYN_T>>(model_);
  ddp_solver_ = std::make_shared<DDP<ModelWrapperDDP<DYN_T>>>(this->dt_, this->num_timesteps_,
                                                              this->params_.num_iterations, &logger, verbose);

  result_ = OptimizerResult<ModelWrapperDDP<DYN_T>>();
  result_.feedback_gain = feedback_gain_trajectory(this->num_timesteps_);
  for (int i = 0; i < this->num_timesteps_; i++)
  {
    result_.feedback_gain[i] = DYN_T::feedback_matrix::Zero();
  }

  run_cost_ =
      std::make_shared<TrackingCostDDP<ModelWrapperDDP<DYN_T>>>(this->params_.Q, this->params_.R, this->num_timesteps_);
  terminal_cost_ = std::make_shared<TrackingTerminalCost<ModelWrapperDDP<DYN_T>>>(this->params_.Q_f);
}

template <class DYN_T, int NUM_TIMESTEPS>
void DDPFeedback<DYN_T, NUM_TIMESTEPS>::setParams(const DDPParams<DYN_T>& params)
{
  this->params_ = params;
  run_cost_ =
      std::make_shared<TrackingCostDDP<ModelWrapperDDP<DYN_T>>>(this->params_.Q, this->params_.R, this->num_timesteps_);
  terminal_cost_ = std::make_shared<TrackingTerminalCost<ModelWrapperDDP<DYN_T>>>(this->params_.Q_f);
}

template <class DYN_T, int NUM_TIMESTEPS>
void DDPFeedback<DYN_T, NUM_TIMESTEPS>::computeFeedback(const Eigen::Ref<const state_array>& init_state,
                                                        const Eigen::Ref<const state_trajectory>& goal_traj,
                                                        const Eigen::Ref<const control_trajectory>& control_traj)
{
  run_cost_->setTargets(goal_traj.data(), control_traj.data(), this->num_timesteps_);

  terminal_cost_->xf = run_cost_->traj_target_x_.col(this->num_timesteps_ - 1);

  // update control ranges
  for (int i = 0; i < DYN_T::CONTROL_DIM; i++)
  {
    control_min_(i) = model_->control_rngs_[i].x;
    control_max_(i) = model_->control_rngs_[i].y;
  }

  result_ =
      ddp_solver_->run(init_state, control_traj, *ddp_model_, *run_cost_, *terminal_cost_, control_min_, control_max_);

  // Copy Feedback Gains into Feedback State
  for (size_t i = 0; i < result_.feedback_gain.size(); i++)
  {
    int i_index = i * DYN_T::STATE_DIM * DYN_T::CONTROL_DIM;
    for (size_t j = 0; j < DYN_T::CONTROL_DIM * DYN_T::STATE_DIM; j++)
    {
      this->getFeedbackStatePointer()->fb_gain_traj_[i_index + j] = result_.feedback_gain[i].data()[j];
    }
  }
  // Actually put new feedback gain trajectory onto the GPU
  // this->gpu_controller_->copyToDevice();
}
