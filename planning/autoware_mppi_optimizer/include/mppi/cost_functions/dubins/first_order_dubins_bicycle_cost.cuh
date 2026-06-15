/**
 * Analytic path-tracking cost for FirstOrderDubinsBicycle (reference polyline + parked-car OBBs).
 */
#pragma once

#ifndef MPPI_COST_FUNCTIONS_FIRST_ORDER_DUBINS_BICYCLE_COST_CUH_
#define MPPI_COST_FUNCTIONS_FIRST_ORDER_DUBINS_BICYCLE_COST_CUH_

#include <mppi/cost_functions/cost.cuh>
#include <mppi/dynamics/dubins/first_order_dubins_bicycle.cuh>

template <int NUM_TIMESTEPS>
struct FirstOrderDubinsBicycleCostParams : public CostParams<2>
{
  float desired_speed = 2.5F;
  float speed_coeff = 500.0F;
  float track_coeff = 1000.0F;
  /** Per-violation crash penalty; latched crash_status counts violations (1=off-road or hit,
   * 2=both). */
  float crash_coeff = 100000.0F;
  float boundary_threshold = 0.8F;
  /** Off-road if signed lateral offset exceeds these (path-left = +); <0 falls back to
   * boundary_threshold. */
  float boundary_threshold_left = -1.0F;
  float boundary_threshold_right = -1.0F;
  float accel_cmd_coeff = 0.0F;
  float steer_cmd_coeff = 0.0F;
  float lateral_acceleration_coeff = 300.0F;
  float lateral_jerk_coeff = 300.0F;
  float longitudinal_jerk_coeff = 10.0F;
  float wheel_base = 0.32F;
  float accel_time_constant = 0.15F;
  float steer_time_constant = 0.08F;
  /** Ego OBB for parked-car collision (rear axle at pose; box center offset forward). */
  float ego_length = 0.55F * 1.5F;
  float ego_width = 0.28F * 1.5F;
  float ego_axle_to_box_center = 0.2F;
  /** Added to ego half-length/width in OBB collision test (~standoff to obstacle surfaces). */
  float obstacle_collision_margin = 0.2F;
  /** Pull toward ref end position (Euclidean distance [m]); 0 disables. */
  float goal_pos_coeff = 2000.0F;
  /** Pull toward ref end speed: coeff * (v - ref_v_end)^2; 0 disables. */
  float goal_speed_coeff = 0.0F;
  /** Pull toward ref end heading: coeff * |yaw - ref_yaw_end|; 0 disables. */
  float goal_yaw_coeff = 0.0F;
  /** Multiplier on goal terms in terminalCost (running state cost uses scale 1). */
  float goal_terminal_scale = 10.0F;
};

template <
  class CLASS_T, int NUM_TIMESTEPS,
  class PARAMS_T = FirstOrderDubinsBicycleCostParams<NUM_TIMESTEPS>,
  class DYN_PARAMS_T = FirstOrderDubinsBicycleParams>
class FirstOrderDubinsBicycleCostImpl : public Cost<CLASS_T, PARAMS_T, DYN_PARAMS_T>
{
public:
  static constexpr int kMaxObstacles = 64;

  using PARENT_CLASS = Cost<CLASS_T, PARAMS_T, DYN_PARAMS_T>;
  using output_array = typename PARENT_CLASS::output_array;
  using control_array = typename PARENT_CLASS::control_array;

  FirstOrderDubinsBicycleCostImpl(cudaStream_t stream = 0);

  void paramsToDevice();

  void setReferenceTrajectory(
    const float * x, const float * y, const float * v, int count, const float * yaw = nullptr);

  /** Static obstacles: same pose replicated at every MPPI horizon step. */
  void setOrientedBoxObstacles(
    const float * x, const float * y, const float * yaw, const float * half_length,
    const float * half_width, int count);

  void setOrientedBoxObstacleTrajectories(
    const float * x, const float * y, const float * yaw, const float * half_length,
    const float * half_width, int obstacle_count, int num_timesteps);

  void clearObstacles();

  __host__ __device__ float computeTrackValue(float x, float y) const;

  /** Distance-to-goal cost vs ref[NUM_TIMESTEPS - 1] (position, speed, yaw). */
  __host__ __device__ float computeGoalCost(float x, float y, float yaw, float vel) const;

  __host__ __device__ float computeSignedLateralOffset(float x, float y) const;

  __host__ __device__ bool isOffRoad(const float x, const float y) const;

  __host__ __device__ bool egoIntersectsObstacleAtStep(
    const float x, const float y, const float yaw, int timestep) const;

  __host__ __device__ bool isCrashLatched(const int * crash_status) const;

  __host__ __device__ bool detectAndLatchCrash(
    const float x, const float y, const float yaw, int timestep, int * crash_status) const;

  __host__ __device__ float latchedCrashCost(const int * crash_status) const;

  float computeStateCost(
    const Eigen::Ref<const output_array> & y, int timestep, int * crash_status);

  __device__ float computeStateCost(float * y, int timestep, float * theta_c, int * crash_status);

  float computeControlCost(const Eigen::Ref<const control_array> & u, int timestep, int * crash);

  __device__ float computeControlCost(float * u, int timestep, float * theta_c, int * crash);

  float computeComfortCost(
    const Eigen::Ref<const control_array> & u, const Eigen::Ref<const output_array> & y,
    int timestep);

  __device__ float computeComfortCost(float * u, float * y, int timestep);

  __device__ float terminalCost(float * y, float * theta_c);

  float computeRunningCost(
    const Eigen::Ref<const output_array> & y, const Eigen::Ref<const control_array> & u,
    int timestep, int * crash);

  __device__ float computeRunningCost(
    float * y, float * u, int timestep, float * theta_c, int * crash);

  float ref_x_[NUM_TIMESTEPS] = {};
  float ref_y_[NUM_TIMESTEPS] = {};
  float ref_v_[NUM_TIMESTEPS] = {};
  float ref_yaw_[NUM_TIMESTEPS] = {};
  int num_obstacles_ = 0;
  float obs_x_[kMaxObstacles][NUM_TIMESTEPS] = {};
  float obs_y_[kMaxObstacles][NUM_TIMESTEPS] = {};
  float obs_yaw_[kMaxObstacles][NUM_TIMESTEPS] = {};
  float obs_half_length_[kMaxObstacles] = {};
  float obs_half_width_[kMaxObstacles] = {};

private:
  void dataToDevice();
};

template <int NUM_TIMESTEPS>
class FirstOrderDubinsBicycleCost
: public FirstOrderDubinsBicycleCostImpl<FirstOrderDubinsBicycleCost<NUM_TIMESTEPS>, NUM_TIMESTEPS>
{
public:
  FirstOrderDubinsBicycleCost(cudaStream_t stream = 0)
  : FirstOrderDubinsBicycleCostImpl<FirstOrderDubinsBicycleCost<NUM_TIMESTEPS>, NUM_TIMESTEPS>(
      stream)
  {
  }
};

#if __CUDACC__
#include "first_order_dubins_bicycle_cost.cu"
#endif

#endif  // MPPI_COST_FUNCTIONS_FIRST_ORDER_DUBINS_BICYCLE_COST_CUH_
