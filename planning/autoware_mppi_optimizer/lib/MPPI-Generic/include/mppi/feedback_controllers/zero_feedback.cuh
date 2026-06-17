/**
 * No-op feedback controller (zero control correction).
 */
#pragma once

#include <mppi/feedback_controllers/feedback.cuh>

template <class DYN_T, int NUM_TIMESTEPS>
class ZeroFeedbackImpl : public GPUFeedbackController<ZeroFeedbackImpl<DYN_T, NUM_TIMESTEPS>, DYN_T, GPUState>
{
public:
  ZeroFeedbackImpl(cudaStream_t stream = 0)
    : GPUFeedbackController<ZeroFeedbackImpl<DYN_T, NUM_TIMESTEPS>, DYN_T, GPUState>(stream)
  {
  }

  __device__ void k(const float* __restrict__ x_act, const float* __restrict__ x_goal, const int t,
                  float* __restrict__ theta, float* __restrict__ control_output)
  {
    (void)x_act;
    (void)x_goal;
    (void)t;
    (void)theta;
    (void)control_output;
  }
};

template <class DYN_T, int NUM_TIMESTEPS>
class ZeroFeedback : public FeedbackController<ZeroFeedbackImpl<DYN_T, NUM_TIMESTEPS>, int, NUM_TIMESTEPS>
{
public:
  using PARENT_CLASS = FeedbackController<ZeroFeedbackImpl<DYN_T, NUM_TIMESTEPS>, int, NUM_TIMESTEPS>;
  using control_array = typename PARENT_CLASS::control_array;
  using state_array = typename PARENT_CLASS::state_array;
  using TEMPLATED_FEEDBACK_STATE = typename PARENT_CLASS::TEMPLATED_FEEDBACK_STATE;

  ZeroFeedback(DYN_T* dyn = nullptr, float dt = 0.01F) : PARENT_CLASS(dt, NUM_TIMESTEPS)
  {
    (void)dyn;
  }

  void initTrackingController() override
  {
  }

  control_array k_(const Eigen::Ref<const state_array>& x_act, const Eigen::Ref<const state_array>& x_goal, int t,
                  TEMPLATED_FEEDBACK_STATE& fb_state) override
  {
    (void)x_act;
    (void)x_goal;
    (void)t;
    (void)fb_state;
    return control_array::Zero();
  }

  void computeFeedback(const Eigen::Ref<const state_array>& init_state,
                       const Eigen::Ref<const typename PARENT_CLASS::state_trajectory>& goal_traj,
                       const Eigen::Ref<const typename PARENT_CLASS::control_trajectory>& control_traj) override
  {
    (void)init_state;
    (void)goal_traj;
    (void)control_traj;
  }
};
