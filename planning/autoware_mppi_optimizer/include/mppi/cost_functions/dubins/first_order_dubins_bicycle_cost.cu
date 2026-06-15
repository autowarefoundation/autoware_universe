#include <mppi/cost_functions/dubins/first_order_dubins_bicycle_cost.cuh>
#include <mppi/cost_functions/path_tracking_geometry.cuh>
#include <mppi/utils/angle_utils.cuh>

#include <algorithm>
#include <cmath>

namespace
{
using O = FirstOrderDubinsBicycleParams::OutputIndex;
using C = FirstOrderDubinsBicycleParams::ControlIndex;
using mppi::cost::detail::distancePointToSegment;
using mppi::cost::detail::orientedBoxesOverlap;
using mppi::cost::detail::signedLateralOffsetPointToSegment;
using mppi::cost::detail::vectorLength;

template <int NUM_TIMESTEPS>
__host__ __device__ float referenceEndYaw(
  const float * x, const float * y, const float * yaw, int count)
{
  if (count <= 0) {
    return 0.0F;
  }
  if (yaw != nullptr) {
    return yaw[count - 1];
  }
  if (count >= 2) {
#ifdef __CUDA_ARCH__
    return atan2f(y[count - 1] - y[count - 2], x[count - 1] - x[count - 2]);
#else
    return std::atan2(y[count - 1] - y[count - 2], x[count - 1] - x[count - 2]);
#endif
  }
  return 0.0F;
}

template <class PARAMS_T>
__host__ __device__ void comfortTerms(
  const PARAMS_T & params, const float * u, const float * y, float & lateral_accel,
  float & lateral_jerk, float & longitudinal_jerk)
{
  const float v = y[static_cast<int>(O::BASELINK_VEL_B_X)];
  const float steer = y[static_cast<int>(O::STEER_ANGLE)];
  const float accel = y[static_cast<int>(O::ACCELERATION)];
  const float accel_cmd = u[static_cast<int>(C::ACCELERATION_CMD)];
  const float steer_cmd = u[static_cast<int>(C::STEER_CMD)];

  const float accel_tau = fmaxf(params.accel_time_constant, 1.0E-4F);
  const float steer_tau = fmaxf(params.steer_time_constant, 1.0E-4F);
  const float wheel_base = fmaxf(params.wheel_base, 1.0E-4F);

  longitudinal_jerk = (accel_cmd - accel) / accel_tau;

  const float steer_rate = (steer_cmd - steer) / steer_tau;
  const float curvature = tanf(steer) / wheel_base;
#ifdef __CUDA_ARCH__
  const float sec_sq = 1.0F / fmaxf(cosf(steer) * cosf(steer), 1.0E-6F);
#else
  const float sec_sq = 1.0F / std::max(std::cos(steer) * std::cos(steer), 1.0E-6F);
#endif
  const float curvature_dot = sec_sq * steer_rate / wheel_base;

  lateral_accel = v * v * curvature;
  lateral_jerk = v * v * curvature_dot + 2.0F * v * accel * curvature;
}
}  // namespace

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::
  FirstOrderDubinsBicycleCostImpl(cudaStream_t stream)
{
  this->bindToStream(stream);
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
void FirstOrderDubinsBicycleCostImpl<
  CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::paramsToDevice()
{
  PARENT_CLASS::paramsToDevice();
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
void FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::dataToDevice()
{
  if (!this->GPUMemStatus_) {
    return;
  }

  HANDLE_ERROR(cudaMemcpyAsync(
    this->cost_d_->ref_x_, ref_x_, sizeof(ref_x_), cudaMemcpyHostToDevice, this->stream_));
  HANDLE_ERROR(cudaMemcpyAsync(
    this->cost_d_->ref_y_, ref_y_, sizeof(ref_y_), cudaMemcpyHostToDevice, this->stream_));
  HANDLE_ERROR(cudaMemcpyAsync(
    this->cost_d_->ref_v_, ref_v_, sizeof(ref_v_), cudaMemcpyHostToDevice, this->stream_));
  HANDLE_ERROR(cudaMemcpyAsync(
    this->cost_d_->ref_yaw_, ref_yaw_, sizeof(ref_yaw_), cudaMemcpyHostToDevice, this->stream_));
  HANDLE_ERROR(cudaMemcpyAsync(
    &this->cost_d_->num_obstacles_, &num_obstacles_, sizeof(num_obstacles_), cudaMemcpyHostToDevice,
    this->stream_));
  HANDLE_ERROR(cudaMemcpyAsync(
    this->cost_d_->obs_x_, obs_x_, sizeof(obs_x_), cudaMemcpyHostToDevice, this->stream_));
  HANDLE_ERROR(cudaMemcpyAsync(
    this->cost_d_->obs_y_, obs_y_, sizeof(obs_y_), cudaMemcpyHostToDevice, this->stream_));
  HANDLE_ERROR(cudaMemcpyAsync(
    this->cost_d_->obs_yaw_, obs_yaw_, sizeof(obs_yaw_), cudaMemcpyHostToDevice, this->stream_));
  HANDLE_ERROR(cudaMemcpyAsync(
    this->cost_d_->obs_half_length_, obs_half_length_, sizeof(obs_half_length_),
    cudaMemcpyHostToDevice, this->stream_));
  HANDLE_ERROR(cudaMemcpyAsync(
    this->cost_d_->obs_half_width_, obs_half_width_, sizeof(obs_half_width_),
    cudaMemcpyHostToDevice, this->stream_));
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
void FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::
  setReferenceTrajectory(
    const float * x, const float * y, const float * v, const int count, const float * yaw)
{
  const int n = std::max(0, std::min(count, NUM_TIMESTEPS));
  const float end_yaw = referenceEndYaw<NUM_TIMESTEPS>(x, y, yaw, n);
  for (int i = 0; i < n; ++i) {
    ref_x_[i] = x[i];
    ref_y_[i] = y[i];
    ref_v_[i] = v != nullptr ? v[i] : this->params_.desired_speed;
    if (yaw != nullptr) {
      ref_yaw_[i] = yaw[i];
    } else if (i >= 1) {
#ifdef __CUDA_ARCH__
      ref_yaw_[i] = atan2f(y[i] - y[i - 1], x[i] - x[i - 1]);
#else
      ref_yaw_[i] = std::atan2(y[i] - y[i - 1], x[i] - x[i - 1]);
#endif
    } else {
      ref_yaw_[i] = end_yaw;
    }
  }
  if (n > 0) {
    for (int i = n; i < NUM_TIMESTEPS; ++i) {
      ref_x_[i] = x[n - 1];
      ref_y_[i] = y[n - 1];
      ref_v_[i] = v != nullptr ? v[n - 1] : this->params_.desired_speed;
      ref_yaw_[i] = end_yaw;
    }
  }
  dataToDevice();
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
void FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::
  setOrientedBoxObstacles(
    const float * x, const float * y, const float * yaw, const float * half_length,
    const float * half_width, const int count)
{
  const int n = std::max(0, std::min(count, kMaxObstacles));
  num_obstacles_ = n;
  for (int i = 0; i < n; ++i) {
    obs_half_length_[i] = half_length[i];
    obs_half_width_[i] = half_width[i];
    for (int t = 0; t < NUM_TIMESTEPS; ++t) {
      obs_x_[i][t] = x[i];
      obs_y_[i][t] = y[i];
      obs_yaw_[i][t] = yaw[i];
    }
  }
  dataToDevice();
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
void FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::
  setOrientedBoxObstacleTrajectories(
    const float * x, const float * y, const float * yaw, const float * half_length,
    const float * half_width, const int obstacle_count, const int num_timesteps)
{
  const int n = std::max(0, std::min(obstacle_count, kMaxObstacles));
  const int nt = std::max(0, std::min(num_timesteps, NUM_TIMESTEPS));
  num_obstacles_ = n;
  for (int i = 0; i < n; ++i) {
    obs_half_length_[i] = half_length[i];
    obs_half_width_[i] = half_width[i];
    for (int t = 0; t < nt; ++t) {
      const int idx = i * nt + t;
      obs_x_[i][t] = x[idx];
      obs_y_[i][t] = y[idx];
      obs_yaw_[i][t] = yaw[idx];
    }
    for (int t = nt; t < NUM_TIMESTEPS; ++t) {
      obs_x_[i][t] = obs_x_[i][nt - 1];
      obs_y_[i][t] = obs_y_[i][nt - 1];
      obs_yaw_[i][t] = obs_yaw_[i][nt - 1];
    }
  }
  dataToDevice();
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
void FirstOrderDubinsBicycleCostImpl<
  CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::clearObstacles()
{
  num_obstacles_ = 0;
  dataToDevice();
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
__host__ __device__ float
FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::computeTrackValue(
  float x, float y) const
{
  float min_dist = 0.0F;
  if (NUM_TIMESTEPS <= 1) {
    min_dist = vectorLength(x - ref_x_[0], y - ref_y_[0]);
  } else {
    min_dist = 1.0E8F;
    for (int i = 0; i < NUM_TIMESTEPS - 1; ++i) {
      const float segment_dist =
        distancePointToSegment(x, y, ref_x_[i], ref_y_[i], ref_x_[i + 1], ref_y_[i + 1]);
#ifdef __CUDA_ARCH__
      min_dist = fminf(min_dist, segment_dist);
#else
      min_dist = std::min(min_dist, segment_dist);
#endif
    }
  }

  return min_dist;
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
__host__ __device__ float
FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::computeGoalCost(
  const float x, const float y, const float yaw, const float vel) const
{
  constexpr int kEnd = NUM_TIMESTEPS - 1;
  float cost = 0.0F;

  if (this->params_.goal_pos_coeff > 0.0F) {
    cost += this->params_.goal_pos_coeff * vectorLength(x - ref_x_[kEnd], y - ref_y_[kEnd]);
  }
  if (this->params_.goal_speed_coeff > 0.0F) {
    const float vel_diff = vel - ref_v_[kEnd];
    cost += this->params_.goal_speed_coeff * vel_diff * vel_diff;
  }
  if (this->params_.goal_yaw_coeff > 0.0F) {
    const float yaw_diff = angle_utils::shortestAngularDistance(yaw, ref_yaw_[kEnd]);
#ifdef __CUDA_ARCH__
    cost += this->params_.goal_yaw_coeff * fabsf(yaw_diff);
#else
    cost += this->params_.goal_yaw_coeff * std::abs(yaw_diff);
#endif
  }

  return cost;
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
__host__ __device__ float FirstOrderDubinsBicycleCostImpl<
  CLASS_T, NUM_TIMESTEPS, PARAMS_T,
  DYN_PARAMS_T>::computeSignedLateralOffset(const float x, const float y) const
{
  if (NUM_TIMESTEPS <= 1) {
    return x - ref_x_[0];
  }

  float best_abs = 1.0E8F;
  float best_signed = 0.0F;
  for (int i = 0; i < NUM_TIMESTEPS - 1; ++i) {
    const float signed_offset =
      signedLateralOffsetPointToSegment(x, y, ref_x_[i], ref_y_[i], ref_x_[i + 1], ref_y_[i + 1]);
#ifdef __CUDA_ARCH__
    const float abs_offset = fabsf(signed_offset);
    if (abs_offset < best_abs) {
      best_abs = abs_offset;
      best_signed = signed_offset;
    }
#else
    const float abs_offset = std::fabs(signed_offset);
    if (abs_offset < best_abs) {
      best_abs = abs_offset;
      best_signed = signed_offset;
    }
#endif
  }

  return best_signed;
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
__host__ __device__ bool
FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::isOffRoad(
  const float x, const float y) const
{
  const bool asymmetric =
    this->params_.boundary_threshold_left >= 0.0F || this->params_.boundary_threshold_right >= 0.0F;
  if (!asymmetric) {
    return computeTrackValue(x, y) >= this->params_.boundary_threshold;
  }

  const float left_limit = this->params_.boundary_threshold_left >= 0.0F
                             ? this->params_.boundary_threshold_left
                             : this->params_.boundary_threshold;
  const float right_limit = this->params_.boundary_threshold_right >= 0.0F
                              ? this->params_.boundary_threshold_right
                              : this->params_.boundary_threshold;
  const float signed_lat = computeSignedLateralOffset(x, y);
  return signed_lat > left_limit || signed_lat < -right_limit;
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
__host__ __device__ bool
FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::
  egoIntersectsObstacleAtStep(
    const float x, const float y, const float yaw, const int timestep) const
{
  int t = timestep;
  if (t < 0) {
    t = 0;
  } else if (t >= NUM_TIMESTEPS) {
    t = NUM_TIMESTEPS - 1;
  }

#ifdef __CUDA_ARCH__
  const float ego_cos = cosf(yaw);
  const float ego_sin = sinf(yaw);
#else
  const float ego_cos = std::cos(yaw);
  const float ego_sin = std::sin(yaw);
#endif
  const float ego_cx = x + this->params_.ego_axle_to_box_center * ego_cos;
  const float ego_cy = y + this->params_.ego_axle_to_box_center * ego_sin;
  const float margin = this->params_.obstacle_collision_margin;
  const float ego_hl = this->params_.ego_length * 0.5F + margin;
  const float ego_hw = this->params_.ego_width * 0.5F + margin;

#ifdef __CUDA_ARCH__
#pragma unroll
#endif
  for (int i = 0; i < num_obstacles_; ++i) {
#ifdef __CUDA_ARCH__
    const float obs_cos = cosf(obs_yaw_[i][t]);
    const float obs_sin = sinf(obs_yaw_[i][t]);
#else
    const float obs_cos = std::cos(obs_yaw_[i][t]);
    const float obs_sin = std::sin(obs_yaw_[i][t]);
#endif
    if (orientedBoxesOverlap(
          ego_cx, ego_cy, ego_cos, ego_sin, ego_hl, ego_hw, obs_x_[i][t], obs_y_[i][t], obs_cos,
          obs_sin, obs_half_length_[i], obs_half_width_[i])) {
      return true;
    }
  }
  return false;
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
__host__ __device__ bool
FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::isCrashLatched(
  const int * crash_status) const
{
  return crash_status != nullptr && crash_status[0] > 0;
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
__host__ __device__ bool
FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::
  detectAndLatchCrash(
    const float x, const float y, const float yaw, const int timestep, int * crash_status) const
{
  const bool off_road = isOffRoad(x, y);
  const bool hit_car = egoIntersectsObstacleAtStep(x, y, yaw, timestep);
  const int violations = static_cast<int>(off_road) + static_cast<int>(hit_car);
  if (violations > 0) {
    if (crash_status != nullptr) {
      crash_status[0] = violations;
    }
    return true;
  }
  return false;
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
__host__ __device__ float
FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::latchedCrashCost(
  const int * crash_status) const
{
  return isCrashLatched(crash_status)
           ? static_cast<float>(crash_status[0]) * this->params_.crash_coeff
           : 0.0F;
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
__device__ float
FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::computeStateCost(
  float * y, int timestep, float * theta_c, int * crash_status)
{
  (void)theta_c;

  const float x_pos = y[static_cast<int>(O::BASELINK_POS_I_X)];
  const float y_pos = y[static_cast<int>(O::BASELINK_POS_I_Y)];
  const float yaw = y[static_cast<int>(O::YAW)];
  const float vel = y[static_cast<int>(O::TOTAL_VELOCITY)];

  const float track_val = computeTrackValue(x_pos, y_pos);
  const float vel_diff = vel - ref_v_[timestep];
  const float speed_cost = this->params_.speed_coeff * (vel_diff * vel_diff);
  const float track_cost = this->params_.track_coeff * track_val;
  const float goal_cost = computeGoalCost(x_pos, y_pos, yaw, vel);
  const float crash_cost =
    isCrashLatched(crash_status) || detectAndLatchCrash(x_pos, y_pos, yaw, timestep, crash_status)
      ? latchedCrashCost(crash_status)
      : 0.0F;

  return speed_cost + track_cost + goal_cost + crash_cost;
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
float FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::
  computeStateCost(const Eigen::Ref<const output_array> & y, int timestep, int * crash_status)
{
  const float x_pos = y[static_cast<int>(O::BASELINK_POS_I_X)];
  const float y_pos = y[static_cast<int>(O::BASELINK_POS_I_Y)];
  const float yaw = y[static_cast<int>(O::YAW)];
  const float vel = y[static_cast<int>(O::TOTAL_VELOCITY)];

  const float track_val = computeTrackValue(x_pos, y_pos);
  const float vel_diff = vel - ref_v_[timestep];
  const float speed_cost = this->params_.speed_coeff * (vel_diff * vel_diff);
  const float track_cost = this->params_.track_coeff * track_val;
  const float goal_cost = computeGoalCost(x_pos, y_pos, yaw, vel);
  const float crash_cost =
    isCrashLatched(crash_status) || detectAndLatchCrash(x_pos, y_pos, yaw, timestep, crash_status)
      ? latchedCrashCost(crash_status)
      : 0.0F;

  return speed_cost + track_cost + goal_cost + crash_cost;
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
__device__ float
FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::computeControlCost(
  float * u, int timestep, float * theta_c, int * crash)
{
  (void)timestep;
  (void)theta_c;
  (void)crash;
  const float accel_cmd = u[static_cast<int>(C::ACCELERATION_CMD)];
  const float steer_cmd = u[static_cast<int>(C::STEER_CMD)];
  return this->params_.accel_cmd_coeff * (accel_cmd * accel_cmd) +
         this->params_.steer_cmd_coeff * (steer_cmd * steer_cmd);
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
float FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::
  computeControlCost(const Eigen::Ref<const control_array> & u, int timestep, int * crash)
{
  (void)timestep;
  (void)crash;
  const float accel_cmd = u(static_cast<int>(C::ACCELERATION_CMD));
  const float steer_cmd = u(static_cast<int>(C::STEER_CMD));
  return this->params_.accel_cmd_coeff * (accel_cmd * accel_cmd) +
         this->params_.steer_cmd_coeff * (steer_cmd * steer_cmd);
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
__device__ float
FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::terminalCost(
  float * y, float * theta_c)
{
  (void)theta_c;
  const float x_pos = y[static_cast<int>(O::BASELINK_POS_I_X)];
  const float y_pos = y[static_cast<int>(O::BASELINK_POS_I_Y)];
  const float yaw = y[static_cast<int>(O::YAW)];
  const float vel = y[static_cast<int>(O::TOTAL_VELOCITY)];
  const float track_val = computeTrackValue(x_pos, y_pos);
  const float track_cost = this->params_.track_coeff * track_val * 10.0F;
  const float goal_cost =
    computeGoalCost(x_pos, y_pos, yaw, vel) * this->params_.goal_terminal_scale;
  return track_cost + goal_cost;
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
float FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::
  computeComfortCost(
    const Eigen::Ref<const control_array> & u, const Eigen::Ref<const output_array> & y,
    int timestep)
{
  (void)timestep;
  float lateral_accel = 0.0F;
  float lateral_jerk = 0.0F;
  float longitudinal_jerk = 0.0F;
  comfortTerms(this->params_, u.data(), y.data(), lateral_accel, lateral_jerk, longitudinal_jerk);
  return this->params_.lateral_acceleration_coeff * std::abs(lateral_accel) +
         this->params_.lateral_jerk_coeff * std::abs(lateral_jerk) +
         this->params_.longitudinal_jerk_coeff * std::abs(longitudinal_jerk);
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
__device__ float
FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::computeComfortCost(
  float * u, float * y, int timestep)
{
  (void)timestep;
  float lateral_accel = 0.0F;
  float lateral_jerk = 0.0F;
  float longitudinal_jerk = 0.0F;
  comfortTerms(this->params_, u, y, lateral_accel, lateral_jerk, longitudinal_jerk);
  return this->params_.lateral_acceleration_coeff * fabsf(lateral_accel) +
         this->params_.lateral_jerk_coeff * fabsf(lateral_jerk) +
         this->params_.longitudinal_jerk_coeff * fabsf(longitudinal_jerk);
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
float FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::
  computeRunningCost(
    const Eigen::Ref<const output_array> & y, const Eigen::Ref<const control_array> & u,
    int timestep, int * crash)
{
  if (isCrashLatched(crash)) {
    return latchedCrashCost(crash);
  }

  const float state_cost = computeStateCost(y, timestep, crash);
  if (isCrashLatched(crash)) {
    return state_cost;
  }

  return state_cost + computeControlCost(u, timestep, crash) + computeComfortCost(u, y, timestep);
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
__device__ float
FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::computeRunningCost(
  float * y, float * u, int timestep, float * theta_c, int * crash)
{
  if (threadIdx.y == 0) {
    if (isCrashLatched(crash)) {
      return latchedCrashCost(crash);
    }

    const float state_cost = computeStateCost(y, timestep, theta_c, crash);
    if (isCrashLatched(crash)) {
      return state_cost;
    }

    return state_cost + computeControlCost(u, timestep, theta_c, crash) +
           computeComfortCost(u, y, timestep);
  }
  return 0.0F;
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
constexpr int
  FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::kMaxObstacles;
