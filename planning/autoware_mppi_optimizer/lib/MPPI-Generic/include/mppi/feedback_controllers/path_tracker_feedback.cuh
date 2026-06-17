/**
 * Path-tracking feedforward feedback: steer from pure-pursuit lookahead on the reference path.
 *
 * At horizon step t, the vehicle pose is the current state (t = 0) or the goal trajectory (t > 0).
 * A lookahead point is taken on the reference polyline at arc length s(t) + L_d, with
 * L_d = clamp(lookahead_time * v, min_lookahead_distance, max_lookahead_distance).
 * Steer uses the bicycle pure-pursuit law: kappa = 2*sin(alpha)/L_d, delta = atan(kappa * L).
 *
 * Set use_pure_pursuit = false to use path curvature at each sample instead.
 *
 * Vanilla MPPI does not call feedback inside rollout kernels; bias the nominal control
 * with applyFeedforwardToNominal() before each solve (see racer_dubins_stadium example).
 */
#pragma once

#include <mppi/feedback_controllers/feedback.cuh>
#include <mppi/path/path2d.hpp>
#include <mppi/path/path_reference_generator.hpp>
#include <mppi/utils/angle_utils.cuh>

#include <array>
#include <cmath>
#include <vector>

struct PathTrackerFeedbackParams
{
  /** If true, pure-pursuit lookahead steer; if false, atan(kappa * L) from path curvature at each sample. */
  bool use_pure_pursuit = true;
  /** Lookahead horizon as time x speed [m] (clamped by min/max distance). */
  float lookahead_time = 0.4F;
  float min_lookahead_distance = 0.5F;
  float max_lookahead_distance = 8.0F;
  /** Scales steer feedforward; use >1 if the nominal path still turns too little. */
  float steer_feedforward_gain = 1.0F;
  /** How many leading u_nom columns get steer replaced (rest keep MPPI warm start). */
  int feedforward_horizon_steps = 1;
};

template <int NUM_TIMESTEPS>
struct PathTrackerFeedbackState : GPUState
{
  float ref_kappa[NUM_TIMESTEPS] = {};
  float goal_yaw[NUM_TIMESTEPS] = {};
  float look_x[NUM_TIMESTEPS] = {};
  float look_y[NUM_TIMESTEPS] = {};
  float lookahead_dist[NUM_TIMESTEPS] = {};
  float wheel_base = 0.3F;
  float steer_angle_scale = 1.0F;
  float steer_command_angle_scale = 1.0F;
  float steer_feedforward_gain = 1.0F;
  /** Max |steer command|; set from dynamics before GPU rollout. */
  float steer_command_limit = 1.0F;
  float dt = 0.01F;
  int num_timesteps = NUM_TIMESTEPS;
  int ref_kappa_valid = 0;
  int goal_traj_valid = 0;
  int use_pure_pursuit = 1;
};

namespace mppi
{
namespace feedback
{
namespace detail
{

template <class Params>
struct HasBicyclePathSteer
{
private:
  template <class U, class = decltype(U::StateIndex::YAW), class = decltype(U::StateIndex::VEL_X),
            class = decltype(U::ControlIndex::STEER_CMD), class = decltype(std::declval<U&>().wheel_base),
            class = decltype(std::declval<U&>().steer_angle_scale),
            class = decltype(std::declval<U&>().steer_command_angle_scale)>
  static std::true_type test(int);
  template <class U>
  static std::false_type test(...);

public:
  static constexpr bool value = decltype(test<Params>(0))::value;
};

template <class Params>
struct SteerCommandMagnitudeLimit
{
  template <class U, class = decltype(std::declval<const U&>().max_steer_angle)>
  static __host__ __device__ float limit(const U& p)
  {
    return p.max_steer_angle;
  }

  static __host__ __device__ float limit(...)
  {
    return 1.0F;
  }
};

template <class Params>
__host__ __device__ inline float steerCommandFromCurvature(const Params& p, float kappa, float steer_gain = 1.0F,
                                                            float steer_limit = 0.0F)
{
  const float steer_angle = atanf(kappa * p.wheel_base) * p.steer_angle_scale;
  const float cmd = steer_gain * steer_angle / p.steer_command_angle_scale;
  const float lim = steer_limit > 0.0F ? steer_limit : SteerCommandMagnitudeLimit<Params>::limit(p);
  return fmaxf(-lim, fminf(lim, cmd));
}

__host__ __device__ inline float clampLookaheadDistance(const float lookahead_dist, const float min_dist,
                                                        const float max_dist)
{
  return fmaxf(min_dist, fminf(max_dist, lookahead_dist));
}

/** Pure-pursuit bicycle steer toward (look_x, look_y) with commanded lookahead distance L_d [m]. */
template <class Params>
__host__ __device__ inline float purePursuitSteerCommand(const Params& p, const float veh_x, const float veh_y,
                                                         const float veh_yaw, const float look_x, const float look_y,
                                                         const float lookahead_dist, const float steer_gain,
                                                         const float steer_limit = 0.0F)
{
  const float dx = look_x - veh_x;
  const float dy = look_y - veh_y;
#ifdef __CUDA_ARCH__
  const float heading_to_point = atan2f(dy, dx);
  const float eta = angle_utils::shortestAngularDistance(veh_yaw, heading_to_point);
  const float sin_eta = sinf(eta);
#else
  const float heading_to_point = std::atan2(dy, dx);
  const float eta = angle_utils::shortestAngularDistance(veh_yaw, heading_to_point);
  const float sin_eta = std::sin(eta);
#endif
  const float Ld = fmaxf(lookahead_dist, 1.0E-3F);
  const float kappa = 2.0F * sin_eta / Ld;
  return steerCommandFromCurvature(p, kappa, steer_gain, steer_limit);
}

template <class DYN_T, int NUM_TIMESTEPS, bool Enabled = HasBicyclePathSteer<typename DYN_T::DYN_PARAMS_T>::value>
struct PathTrackerSteerControl;

template <class DYN_T, int NUM_TIMESTEPS>
struct PathTrackerSteerControl<DYN_T, NUM_TIMESTEPS, true>
{
  using Params = typename DYN_T::DYN_PARAMS_T;
  using control_array = typename DYN_T::control_array;
  using state_trajectory = Eigen::Matrix<float, DYN_T::STATE_DIM, NUM_TIMESTEPS>;

  static control_array computePurePursuit(const Params& dyn_params, const state_trajectory& goal_traj,
                                          const typename DYN_T::state_array& x_act, const int t, const float look_x,
                                          const float look_y, const float lookahead_dist, const float steer_gain)
  {
    control_array u = control_array::Zero();
    constexpr int kYaw = static_cast<int>(Params::StateIndex::YAW);
    constexpr int kX = static_cast<int>(Params::StateIndex::POS_X);
    constexpr int kY = static_cast<int>(Params::StateIndex::POS_Y);
    constexpr int kSteer = static_cast<int>(Params::ControlIndex::STEER_CMD);

    float veh_x = x_act(kX);
    float veh_y = x_act(kY);
    float veh_yaw = x_act(kYaw);
    if (t > 0)
    {
      veh_x = goal_traj(kX, t);
      veh_y = goal_traj(kY, t);
      veh_yaw = goal_traj(kYaw, t);
    }

    u(kSteer) = purePursuitSteerCommand(dyn_params, veh_x, veh_y, veh_yaw, look_x, look_y, lookahead_dist, steer_gain);
    return u;
  }

  static control_array computeCurvature(DYN_T& dyn, const state_trajectory& goal_traj,
                                        const typename DYN_T::state_array& x_act, const int t, const float dt,
                                        const float path_kappa, const bool path_kappa_valid, const float steer_gain)
  {
    control_array u = control_array::Zero();
    constexpr int kYaw = static_cast<int>(Params::StateIndex::YAW);
    constexpr int kVel = static_cast<int>(Params::StateIndex::VEL_X);
    constexpr int kSteer = static_cast<int>(Params::ControlIndex::STEER_CMD);

    float kappa = path_kappa;
    if (!path_kappa_valid)
    {
      const int t_next = std::min(t + 1, NUM_TIMESTEPS - 1);
      const float yaw0 = goal_traj(kYaw, t);
      const float yaw1 = goal_traj(kYaw, t_next);
      const float vel = std::max(std::fabs(x_act(kVel)), 0.1F);
      const float ds = std::max(vel * dt * static_cast<float>(t_next - t), 1.0E-3F);
      kappa = angle_utils::shortestAngularDistance(yaw0, yaw1) / ds;
    }

    u(kSteer) = steerCommandFromCurvature(dyn.getParams(), kappa, steer_gain);
    return u;
  }
};

template <class DYN_T, int NUM_TIMESTEPS>
struct PathTrackerSteerControl<DYN_T, NUM_TIMESTEPS, false>
{
  using control_array = typename DYN_T::control_array;

  static control_array compute(const DYN_T&, const Eigen::Matrix<float, DYN_T::STATE_DIM, NUM_TIMESTEPS>&,
                               const typename DYN_T::state_array&, int, float, float, bool, float)
  {
    return control_array::Zero();
  }
};

}  // namespace detail

template <class DYN_T, int NUM_TIMESTEPS>
Eigen::Matrix<float, DYN_T::STATE_DIM, NUM_TIMESTEPS> goalTrajectoryFromPathReference(
    const std::vector<mppi::path::PathReferenceSample>& ref)
{
  using Params = typename DYN_T::DYN_PARAMS_T;
  Eigen::Matrix<float, DYN_T::STATE_DIM, NUM_TIMESTEPS> goal_traj =
      Eigen::Matrix<float, DYN_T::STATE_DIM, NUM_TIMESTEPS>::Zero();

  constexpr int kVel = static_cast<int>(Params::StateIndex::VEL_X);
  constexpr int kYaw = static_cast<int>(Params::StateIndex::YAW);
  constexpr int kX = static_cast<int>(Params::StateIndex::POS_X);
  constexpr int kY = static_cast<int>(Params::StateIndex::POS_Y);

  for (int t = 0; t < NUM_TIMESTEPS; ++t)
  {
    const mppi::path::PathReferenceSample& s = ref[static_cast<size_t>(std::min(t, static_cast<int>(ref.size()) - 1))];
    goal_traj(kVel, t) = s.v;
    goal_traj(kYaw, t) = s.yaw;
    goal_traj(kX, t) = s.x;
    goal_traj(kY, t) = s.y;
  }
  return goal_traj;
}

template <int NUM_TIMESTEPS>
std::array<float, NUM_TIMESTEPS> referenceCurvaturesFromPath(const mppi::path::Path2D& path,
                                                             const std::vector<mppi::path::PathReferenceSample>& ref)
{
  std::array<float, NUM_TIMESTEPS> kappa{};
  for (int t = 0; t < NUM_TIMESTEPS; ++t)
  {
    const mppi::path::PathReferenceSample& s = ref[static_cast<size_t>(std::min(t, static_cast<int>(ref.size()) - 1))];
    kappa[static_cast<size_t>(t)] = path.curvatureAt(s.arc_length_s);
  }
  return kappa;
}

template <int NUM_TIMESTEPS>
inline int lookaheadReferenceIndex(const std::vector<mppi::path::PathReferenceSample>& ref, const int start_idx,
                                   const float lookahead_ds)
{
  if (ref.empty())
  {
    return 0;
  }
  const int n = static_cast<int>(ref.size());
  const int i0 = std::max(0, std::min(start_idx, n - 1));
  const float s_target = ref[static_cast<size_t>(i0)].arc_length_s + lookahead_ds;
  for (int i = i0; i < n; ++i)
  {
    if (ref[static_cast<size_t>(i)].arc_length_s >= s_target)
    {
      return i;
    }
  }
  return n - 1;
}

template <int NUM_TIMESTEPS>
inline void fillPurePursuitLookaheadTargets(const std::vector<mppi::path::PathReferenceSample>& ref,
                                            const PathTrackerFeedbackParams& params, float* look_x, float* look_y,
                                            float* lookahead_dist)
{
  if (ref.empty())
  {
    return;
  }
  for (int t = 0; t < NUM_TIMESTEPS; ++t)
  {
    const int ref_idx = std::min(t, static_cast<int>(ref.size()) - 1);
    const mppi::path::PathReferenceSample& sample = ref[static_cast<size_t>(ref_idx)];
    const float v = std::max(sample.v, 0.1F);
    const float Ld = detail::clampLookaheadDistance(params.lookahead_time * v, params.min_lookahead_distance,
                                                    params.max_lookahead_distance);
    const int look_idx = lookaheadReferenceIndex<NUM_TIMESTEPS>(ref, ref_idx, Ld);
    const mppi::path::PathReferenceSample& look = ref[static_cast<size_t>(look_idx)];
    look_x[t] = look.x;
    look_y[t] = look.y;
    lookahead_dist[t] = Ld;
  }
}

}  // namespace feedback
}  // namespace mppi

template <class DYN_T, int NUM_TIMESTEPS>
class PathTrackerFeedbackImpl
    : public GPUFeedbackController<PathTrackerFeedbackImpl<DYN_T, NUM_TIMESTEPS>, DYN_T,
                                   PathTrackerFeedbackState<NUM_TIMESTEPS>>
{
public:
  using FEEDBACK_STATE_T = PathTrackerFeedbackState<NUM_TIMESTEPS>;
  using Params = typename DYN_T::DYN_PARAMS_T;

  PathTrackerFeedbackImpl(cudaStream_t stream = 0)
    : GPUFeedbackController<PathTrackerFeedbackImpl<DYN_T, NUM_TIMESTEPS>, DYN_T, FEEDBACK_STATE_T>(stream)
  {
  }

  __device__ void k(const float* __restrict__ x_act, const float* __restrict__ x_goal, const int t,
                    float* __restrict__ theta, float* __restrict__ control_output)
  {
    (void)x_goal;
    (void)theta;
    for (int i = 0; i < DYN_T::CONTROL_DIM; ++i)
    {
      control_output[i] = 0.0F;
    }

    const FEEDBACK_STATE_T& st = this->state_;
    if (st.goal_traj_valid == 0)
    {
      return;
    }

    constexpr int kX = static_cast<int>(Params::StateIndex::POS_X);
    constexpr int kY = static_cast<int>(Params::StateIndex::POS_Y);
    constexpr int kYaw = static_cast<int>(Params::StateIndex::YAW);
    constexpr int kVel = static_cast<int>(Params::StateIndex::VEL_X);
    constexpr int kSteer = static_cast<int>(Params::ControlIndex::STEER_CMD);

    const int t_clamped = min(max(t, 0), st.num_timesteps - 1);

    Params p;
    p.wheel_base = st.wheel_base;
    p.steer_angle_scale = st.steer_angle_scale;
    p.steer_command_angle_scale = st.steer_command_angle_scale;

    if (st.use_pure_pursuit != 0)
    {
      control_output[kSteer] = mppi::feedback::detail::purePursuitSteerCommand(
          p, x_act[kX], x_act[kY], x_act[kYaw], st.look_x[t_clamped], st.look_y[t_clamped], st.lookahead_dist[t_clamped],
          st.steer_feedforward_gain, st.steer_command_limit);
      return;
    }

    float kappa = 0.0F;
    if (st.ref_kappa_valid != 0)
    {
      kappa = st.ref_kappa[t_clamped];
    }
    else
    {
      const int t_next = min(t_clamped + 1, st.num_timesteps - 1);
      const float vel = fmaxf(fabsf(x_act[kVel]), 0.1F);
      const float ds = fmaxf(vel * st.dt * static_cast<float>(t_next - t_clamped), 1.0E-3F);
      kappa = angle_utils::shortestAngularDistance(st.goal_yaw[t_clamped], st.goal_yaw[t_next]) / ds;
    }

    control_output[kSteer] = mppi::feedback::detail::steerCommandFromCurvature(p, kappa, st.steer_feedforward_gain,
                                                                               st.steer_command_limit);
  }
};

template <class DYN_T, int NUM_TIMESTEPS>
class PathTrackerFeedback
    : public FeedbackController<PathTrackerFeedbackImpl<DYN_T, NUM_TIMESTEPS>, PathTrackerFeedbackParams, NUM_TIMESTEPS>
{
public:
  using PARENT_CLASS =
      FeedbackController<PathTrackerFeedbackImpl<DYN_T, NUM_TIMESTEPS>, PathTrackerFeedbackParams, NUM_TIMESTEPS>;
  using control_array = typename PARENT_CLASS::control_array;
  using control_trajectory = typename PARENT_CLASS::control_trajectory;
  using state_array = typename PARENT_CLASS::state_array;
  using state_trajectory = typename PARENT_CLASS::state_trajectory;
  using TEMPLATED_FEEDBACK_STATE = typename PARENT_CLASS::TEMPLATED_FEEDBACK_STATE;
  using FEEDBACK_STATE_T = PathTrackerFeedbackState<NUM_TIMESTEPS>;

  PathTrackerFeedback(DYN_T* dyn = nullptr, float dt = 0.01F) : PARENT_CLASS(dt, NUM_TIMESTEPS), dyn_(dyn)
  {
  }

  void initTrackingController() override
  {
  }

  void updateReference(const mppi::path::Path2D& path, const std::vector<mppi::path::PathReferenceSample>& ref)
  {
    ref_samples_ = ref;
    ref_kappa_ = mppi::feedback::referenceCurvaturesFromPath<NUM_TIMESTEPS>(path, ref);
    ref_kappa_valid_ = true;
    mppi::feedback::fillPurePursuitLookaheadTargets<NUM_TIMESTEPS>(ref_samples_, this->params_, look_x_.data(),
                                                                   look_y_.data(), lookahead_dist_.data());
  }

  control_array k_(const Eigen::Ref<const state_array>& x_act, const Eigen::Ref<const state_array>& x_goal, int t,
                  TEMPLATED_FEEDBACK_STATE& fb_state) override
  {
    (void)x_goal;
    (void)fb_state;
    if (dyn_ == nullptr || !goal_traj_valid_)
    {
      return control_array::Zero();
    }
    const float steer_gain = this->params_.steer_feedforward_gain;
    if (this->params_.use_pure_pursuit)
    {
      return mppi::feedback::detail::PathTrackerSteerControl<DYN_T, NUM_TIMESTEPS>::computePurePursuit(
          dyn_->getParams(), goal_traj_, x_act, t, look_x_[static_cast<size_t>(t)], look_y_[static_cast<size_t>(t)],
          lookahead_dist_[static_cast<size_t>(t)], steer_gain);
    }
    const float path_kappa = ref_kappa_valid_ ? ref_kappa_[static_cast<size_t>(t)] : 0.0F;
    return mppi::feedback::detail::PathTrackerSteerControl<DYN_T, NUM_TIMESTEPS>::computeCurvature(
        *dyn_, goal_traj_, x_act, t, this->getDt(), path_kappa, ref_kappa_valid_, steer_gain);
  }

  void computeFeedback(const Eigen::Ref<const state_array>& init_state,
                       const Eigen::Ref<const state_trajectory>& goal_traj,
                       const Eigen::Ref<const control_trajectory>& control_traj) override
  {
    (void)init_state;
    (void)control_traj;
    goal_traj_ = goal_traj;
    goal_traj_valid_ = true;
    syncFeedbackStateToGpu();
  }

  /**
   * Set nominal steer from pure-pursuit / path curvature (throttle entries unchanged).
   * Replaces steer each step; do not accumulate on top of the previous MPPI solution.
   */
  void applyFeedforwardToNominal(Eigen::Ref<control_trajectory> u_nom, const Eigen::Ref<const state_array>& x,
                                 const Eigen::Ref<const state_trajectory>& goal_traj)
  {
    computeFeedback(x, goal_traj, u_nom);
    using Params = typename DYN_T::DYN_PARAMS_T;
    constexpr int kSteer = static_cast<int>(Params::ControlIndex::STEER_CMD);
    const int n_ff = std::max(1, std::min(this->params_.feedforward_horizon_steps, NUM_TIMESTEPS));
    TEMPLATED_FEEDBACK_STATE fb_state{};
    for (int t = 0; t < n_ff; ++t)
    {
      const control_array u_ff = k_(x, goal_traj.col(t), t, fb_state);
      u_nom(kSteer, t) = u_ff(kSteer);
    }
  }

private:
  void syncFeedbackStateToGpu()
  {
    if (dyn_ == nullptr || !goal_traj_valid_)
    {
      return;
    }

    FEEDBACK_STATE_T gpu_state{};
    gpu_state.num_timesteps = NUM_TIMESTEPS;
    gpu_state.dt = this->getDt();
    gpu_state.goal_traj_valid = 1;
    gpu_state.ref_kappa_valid = ref_kappa_valid_ ? 1 : 0;
    gpu_state.use_pure_pursuit = this->params_.use_pure_pursuit ? 1 : 0;

    using Params = typename DYN_T::DYN_PARAMS_T;
    constexpr int kYaw = static_cast<int>(Params::StateIndex::YAW);
    for (int t = 0; t < NUM_TIMESTEPS; ++t)
    {
      gpu_state.goal_yaw[t] = goal_traj_(kYaw, t);
      gpu_state.ref_kappa[t] = ref_kappa_valid_ ? ref_kappa_[static_cast<size_t>(t)] : 0.0F;
      gpu_state.look_x[t] = look_x_[static_cast<size_t>(t)];
      gpu_state.look_y[t] = look_y_[static_cast<size_t>(t)];
      gpu_state.lookahead_dist[t] = lookahead_dist_[static_cast<size_t>(t)];
    }

    const auto& p = dyn_->getParams();
    gpu_state.wheel_base = p.wheel_base;
    gpu_state.steer_angle_scale = p.steer_angle_scale;
    gpu_state.steer_command_angle_scale = p.steer_command_angle_scale;
    gpu_state.steer_feedforward_gain = this->params_.steer_feedforward_gain;
    gpu_state.steer_command_limit = mppi::feedback::detail::SteerCommandMagnitudeLimit<Params>::limit(p);

    this->getHostPointer()->setFeedbackState(gpu_state);
  }

  DYN_T* dyn_ = nullptr;
  state_trajectory goal_traj_ = state_trajectory::Zero();
  std::vector<mppi::path::PathReferenceSample> ref_samples_;
  std::array<float, NUM_TIMESTEPS> ref_kappa_{};
  std::array<float, NUM_TIMESTEPS> look_x_{};
  std::array<float, NUM_TIMESTEPS> look_y_{};
  std::array<float, NUM_TIMESTEPS> lookahead_dist_{};
  bool goal_traj_valid_ = false;
  bool ref_kappa_valid_ = false;
};
