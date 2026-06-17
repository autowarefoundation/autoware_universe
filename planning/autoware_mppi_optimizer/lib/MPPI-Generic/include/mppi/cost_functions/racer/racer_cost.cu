#include <mppi/cost_functions/path_tracking_geometry.cuh>
#include <mppi/cost_functions/racer/racer_cost.cuh>

#include <algorithm>
#include <cmath>

namespace
{
using mppi::cost::detail::distancePointToSegment;
using mppi::cost::detail::orientedBoxesOverlap;
using mppi::cost::detail::vectorLength;

__host__ __device__ inline float racerRolloutSpeed(const float* y)
{
  return fabsf(y[static_cast<int>(RacerDubinsParams::OutputIndex::BASELINK_VEL_B_X)]);
}

__host__ __device__ inline float racerRolloutSteerAngle(const float* y)
{
  return y[static_cast<int>(RacerDubinsParams::OutputIndex::BASELINK_POS_I_Z)];
}

__host__ __device__ inline float racerRolloutSteerRate(const float* y)
{
  return y[static_cast<int>(RacerDubinsParams::OutputIndex::ROLL)];
}

__host__ __device__ inline float racerRolloutYaw(const float* y)
{
  return y[static_cast<int>(RacerDubinsParams::OutputIndex::YAW)];
}

}  // namespace

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
RacerCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::RacerCostImpl(cudaStream_t stream)
{
  this->bindToStream(stream);
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
void RacerCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::paramsToDevice()
{
  PARENT_CLASS::paramsToDevice();
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
void RacerCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::dataToDevice()
{
  if (!this->GPUMemStatus_)
  {
    return;
  }

  HANDLE_ERROR(cudaMemcpyAsync(this->cost_d_->ref_x_, ref_x_, sizeof(ref_x_), cudaMemcpyHostToDevice, this->stream_));
  HANDLE_ERROR(cudaMemcpyAsync(this->cost_d_->ref_y_, ref_y_, sizeof(ref_y_), cudaMemcpyHostToDevice, this->stream_));
  HANDLE_ERROR(cudaMemcpyAsync(&this->cost_d_->num_obstacles_, &num_obstacles_, sizeof(num_obstacles_),
                               cudaMemcpyHostToDevice, this->stream_));
  HANDLE_ERROR(cudaMemcpyAsync(this->cost_d_->obs_x_, obs_x_, sizeof(obs_x_), cudaMemcpyHostToDevice, this->stream_));
  HANDLE_ERROR(cudaMemcpyAsync(this->cost_d_->obs_y_, obs_y_, sizeof(obs_y_), cudaMemcpyHostToDevice, this->stream_));
  HANDLE_ERROR(cudaMemcpyAsync(this->cost_d_->obs_yaw_, obs_yaw_, sizeof(obs_yaw_), cudaMemcpyHostToDevice, this->stream_));
  HANDLE_ERROR(cudaMemcpyAsync(this->cost_d_->obs_half_length_, obs_half_length_, sizeof(obs_half_length_),
                               cudaMemcpyHostToDevice, this->stream_));
  HANDLE_ERROR(cudaMemcpyAsync(this->cost_d_->obs_half_width_, obs_half_width_, sizeof(obs_half_width_),
                               cudaMemcpyHostToDevice, this->stream_));
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
void RacerCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::setReferenceTrajectory(const float* x,
                                                                                             const float* y,
                                                                                             const int count)
{
  const int n = std::max(0, std::min(count, NUM_TIMESTEPS));
  for (int i = 0; i < n; ++i)
  {
    ref_x_[i] = x[i];
    ref_y_[i] = y[i];
  }
  if (n > 0)
  {
    for (int i = n; i < NUM_TIMESTEPS; ++i)
    {
      ref_x_[i] = x[n - 1];
      ref_y_[i] = y[n - 1];
    }
  }
  dataToDevice();
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
void RacerCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::setOrientedBoxObstacles(
    const float* x, const float* y, const float* yaw, const float* half_length, const float* half_width, const int count)
{
  const int n = std::max(0, std::min(count, kMaxObstacles));
  num_obstacles_ = n;
  for (int i = 0; i < n; ++i)
  {
    obs_x_[i] = x[i];
    obs_y_[i] = y[i];
    obs_yaw_[i] = yaw[i];
    obs_half_length_[i] = half_length[i];
    obs_half_width_[i] = half_width[i];
  }
  dataToDevice();
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
__host__ __device__ float RacerCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::computeTrackValue(float x,
                                                                                                           float y) const
{
  float min_dist = 0.0F;
  if (NUM_TIMESTEPS <= 1)
  {
    min_dist = vectorLength(x - ref_x_[0], y - ref_y_[0]);
  }
  else
  {
    min_dist = 1.0E8F;
    for (int i = 0; i < NUM_TIMESTEPS - 1; ++i)
    {
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
__host__ __device__ bool RacerCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::egoIntersectsParkedCar(
    const float x, const float y, const float yaw) const
{
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

  for (int i = 0; i < num_obstacles_; ++i)
  {
#ifdef __CUDA_ARCH__
    const float obs_cos = cosf(obs_yaw_[i]);
    const float obs_sin = sinf(obs_yaw_[i]);
#else
    const float obs_cos = std::cos(obs_yaw_[i]);
    const float obs_sin = std::sin(obs_yaw_[i]);
#endif
    if (orientedBoxesOverlap(ego_cx, ego_cy, ego_cos, ego_sin, ego_hl, ego_hw, obs_x_[i], obs_y_[i], obs_cos, obs_sin,
                             obs_half_length_[i], obs_half_width_[i]))
    {
      return true;
    }
  }
  return false;
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
__device__ float RacerCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::computeStateCost(float* y, int timestep,
                                                                                                 float* theta_c,
                                                                                                 int* crash_status)
{
  (void)theta_c;

  const float x_pos = y[static_cast<int>(RacerDubinsParams::OutputIndex::BASELINK_POS_I_X)];
  const float y_pos = y[static_cast<int>(RacerDubinsParams::OutputIndex::BASELINK_POS_I_Y)];
  const float yaw = racerRolloutYaw(y);
  const float vel = racerRolloutSpeed(y);

  const float track_val = computeTrackValue(x_pos, y_pos);

  const float vel_diff = vel - this->params_.desired_speed;
  const float speed_cost = this->params_.speed_coeff * (vel_diff * vel_diff);
  const float track_cost = this->params_.track_coeff * track_val;
  float crash_cost = 0.0F;
  const bool off_road = track_val >= this->params_.boundary_threshold;
  const bool hit_car = egoIntersectsParkedCar(x_pos, y_pos, yaw);
  if (off_road || hit_car)
  {
    crash_cost = this->params_.crash_coeff;
    if (crash_status != nullptr)
    {
      *crash_status = 1;
      return crash_cost;
    }
  }

  return speed_cost + track_cost + crash_cost;
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
float RacerCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::computeStateCost(
    const Eigen::Ref<const output_array>& y, int timestep, int* crash_status)
{
  (void)timestep;
  const float x_pos = y[static_cast<int>(RacerDubinsParams::OutputIndex::BASELINK_POS_I_X)];
  const float y_pos = y[static_cast<int>(RacerDubinsParams::OutputIndex::BASELINK_POS_I_Y)];
  const float yaw = racerRolloutYaw(y.data());
  const float vel = racerRolloutSpeed(y.data());

  const float track_val = computeTrackValue(x_pos, y_pos);

  const float vel_diff = vel - this->params_.desired_speed;
  const float speed_cost = this->params_.speed_coeff * (vel_diff * vel_diff);
  const float track_cost = this->params_.track_coeff * track_val;
  float crash_cost = 0.0F;
  const bool off_road = track_val >= this->params_.boundary_threshold;
  const bool hit_car = egoIntersectsParkedCar(x_pos, y_pos, yaw);
  if (off_road || hit_car)
  {
    crash_cost = this->params_.crash_coeff;
    if (crash_status != nullptr)
    {
      *crash_status = 1;
      return crash_cost;
    }
  }

  return speed_cost + track_cost + crash_cost;
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
__device__ float RacerCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::computeControlCost(float* u,
                                                                                                     int timestep,
                                                                                                     float* theta_c,
                                                                                                     int* crash)
{
  (void)timestep;
  (void)theta_c;
  (void)crash;
  const float steer = u[static_cast<int>(RacerDubinsParams::ControlIndex::STEER_CMD)];
  return this->params_.steer_coeff * (steer * steer);
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
float RacerCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::computeControlCost(
    const Eigen::Ref<const control_array>& u, int timestep, int* crash)
{
  (void)timestep;
  (void)crash;
  const float steer = u(static_cast<int>(RacerDubinsParams::ControlIndex::STEER_CMD));
  return this->params_.steer_coeff * (steer * steer);
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
__device__ float RacerCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::terminalCost(float* y, float* theta_c)
{
  (void)theta_c;
  const float x_pos = y[static_cast<int>(RacerDubinsParams::OutputIndex::BASELINK_POS_I_X)];
  const float y_pos = y[static_cast<int>(RacerDubinsParams::OutputIndex::BASELINK_POS_I_Y)];
  const float track_val = computeTrackValue(x_pos, y_pos);
  return this->params_.track_coeff * track_val * 10.0F;
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
float RacerCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::computeComfortCost(
    const Eigen::Ref<const control_array>& u, const Eigen::Ref<const output_array>& y, int timestep)
{
  (void)u;
  (void)timestep;
  const float vel = racerRolloutSpeed(y.data());
  const float steer_angle = racerRolloutSteerAngle(y.data());
  const float steer_angle_rate = racerRolloutSteerRate(y.data());

  const float phi = steer_angle / this->params_.steer_angle_scale;
  const float cos_phi = std::cos(phi);
  const float sec_sq_phi = 1.0F / std::max(cos_phi * cos_phi, 1.0E-6F);

  // kappa = tan(phi) / L
  const float curvature = std::tan(phi) / this->params_.wheel_base;
  // kappa_dot = sec^2(phi) * phi_dot / L, with phi_dot = steer_angle_rate / steer_angle_scale
  const float curvature_derivative =
      (sec_sq_phi * steer_angle_rate) / (this->params_.wheel_base * this->params_.steer_angle_scale);

  // a_y = v^2 * kappa; j_y ≈ v^2 * kappa_dot (longitudinal coupling omitted: ACCEL_X not in rollout output)
  const float lateral_acceleration = vel * vel * curvature;
  const float lateral_jerk = vel * vel * curvature_derivative;

  return this->params_.lateral_acceleration_coeff * std::abs(lateral_acceleration) +
         this->params_.lateral_jerk_coeff * std::abs(lateral_jerk);
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
__device__ float RacerCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::computeComfortCost(float* u, float* y, int timestep)
{
  (void)u;
  (void)timestep;
  const float vel = racerRolloutSpeed(y);
  const float steer_angle = racerRolloutSteerAngle(y);
  const float steer_angle_rate = racerRolloutSteerRate(y);

  const float phi = steer_angle / this->params_.steer_angle_scale;
  const float cos_phi = cosf(phi);
  const float sec_sq_phi = 1.0F / fmaxf(cos_phi * cos_phi, 1.0E-6F);

  const float curvature = tanf(phi) / this->params_.wheel_base;
  const float curvature_derivative =
      (sec_sq_phi * steer_angle_rate) / (this->params_.wheel_base * this->params_.steer_angle_scale);

  const float lateral_acceleration = vel * vel * curvature;
  const float lateral_jerk = vel * vel * curvature_derivative;

  return this->params_.lateral_acceleration_coeff * fabsf(lateral_acceleration) +
         this->params_.lateral_jerk_coeff * fabsf(lateral_jerk);
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
float RacerCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::computeRunningCost(
    const Eigen::Ref<const output_array>& y, const Eigen::Ref<const control_array>& u, int timestep, int* crash)
{
  return this->computeStateCost(y, timestep, crash) + this->computeControlCost(u, timestep, crash) +
         this->computeComfortCost(u, y, timestep);
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
__device__ float RacerCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::computeRunningCost(
    float* y, float* u, int timestep, float* theta_c, int* crash)
{
  if (threadIdx.y == 0)
  {
    return this->computeStateCost(y, timestep, theta_c, crash) + this->computeControlCost(u, timestep, theta_c, crash) +
           this->computeComfortCost(u, y, timestep);
  }
  else
  {
    return 0.0f;
  }
}