/**
 * Kinematic Dubins bicycle with first-order actuation on longitudinal acceleration and steering.
 *
 * State: speed, yaw, position, steer angle, applied acceleration.
 * Controls: acceleration command [m/s^2], steer angle command [rad].
 *
 *   d(accel)/dt = (u_accel - accel) / accel_time_constant
 *   d(v)/dt     = accel
 *   d(steer)/dt = (u_steer - steer) / steer_time_constant   (rate-limited)
 *   d(yaw)/dt   = (v / L) * tan(steer)
 *   d(x,y)/dt   = v * [cos(yaw), sin(yaw)]
 */
#pragma once

#ifndef MPPIGENERIC_FIRST_ORDER_DUBINS_BICYCLE_CUH
#define MPPIGENERIC_FIRST_ORDER_DUBINS_BICYCLE_CUH

#include <mppi/dynamics/dynamics.cuh>
#include <mppi/utils/angle_utils.cuh>

struct FirstOrderDubinsBicycleParams : public DynamicsParams
{
  enum class StateIndex : int
  {
    VEL_X = 0,
    YAW,
    POS_X,
    POS_Y,
    STEER_ANGLE,
    ACCELERATION,
    NUM_STATES
  };

  enum class ControlIndex : int
  {
    ACCELERATION_CMD = 0,
    STEER_CMD,
    NUM_CONTROLS
  };

  enum class OutputIndex : int
  {
    BASELINK_VEL_B_X = 0,
    BASELINK_VEL_B_Y,
    BASELINK_POS_I_X,
    BASELINK_POS_I_Y,
    YAW,
    STEER_ANGLE,
    ACCELERATION,
    TOTAL_VELOCITY,
    LATERAL_JERK,
    LONGITUDINAL_JERK,
    NUM_OUTPUTS
  };

  float wheel_base = 0.32F;
  /** PathTrackerFeedback: steer_cmd [rad] = atan(kappa * L) * steer_angle_scale / steer_command_angle_scale */
  float steer_angle_scale = 1.0F;
  float steer_command_angle_scale = 1.0F;
  /** First-order lag: accel_dot = (u_accel - accel) / accel_time_constant */
  float accel_time_constant = 0.15F;
  /** First-order lag: steer_dot = (u_steer - steer) / steer_time_constant */
  float steer_time_constant = 0.08F;
  float max_steer_angle = 0.45F;
  float max_steer_rate = 3.0F;
  float min_accel = -6.0F;
  float max_accel = 4.0F;
};

using namespace MPPI_internal;

template <class CLASS_T, class PARAMS_T = FirstOrderDubinsBicycleParams>
class FirstOrderDubinsBicycleImpl : public Dynamics<CLASS_T, PARAMS_T>
{
public:
  using PARENT_CLASS = Dynamics<CLASS_T, PARAMS_T>;
  using state_array = typename PARENT_CLASS::state_array;
  using control_array = typename PARENT_CLASS::control_array;
  using output_array = typename PARENT_CLASS::output_array;
  using dfdx = typename PARENT_CLASS::dfdx;
  using dfdu = typename PARENT_CLASS::dfdu;
  using PARENT_CLASS::updateState;

  FirstOrderDubinsBicycleImpl(cudaStream_t stream = nullptr);

  FirstOrderDubinsBicycleImpl(PARAMS_T& params, cudaStream_t stream = nullptr);

  std::string getDynamicsModelName() const override
  {
    return "First-Order Dubins Bicycle";
  }

  void computeDynamics(const Eigen::Ref<const state_array>& state, const Eigen::Ref<const control_array>& control,
                       Eigen::Ref<state_array> state_der);

  void updateState(const Eigen::Ref<const state_array> state, Eigen::Ref<state_array> next_state,
                   Eigen::Ref<state_array> state_der, const float dt);

  __device__ void updateState(float* state, float* next_state, float* state_der, const float dt);

  state_array interpolateState(const Eigen::Ref<state_array> state_1, const Eigen::Ref<state_array> state_2,
                               const float alpha);

  bool computeGrad(const Eigen::Ref<const state_array>& state, const Eigen::Ref<const control_array>& control,
                   Eigen::Ref<dfdx> A, Eigen::Ref<dfdu> B);

  __device__ void computeDynamics(float* state, float* control, float* state_der, float* theta = nullptr);

  void stateToOutput(const Eigen::Ref<const state_array>& state, Eigen::Ref<output_array> output);

  __host__ __device__ void stateToOutput(const float* state, float* output);

  state_array stateFromMap(const std::map<std::string, float>& map) override;
};

class FirstOrderDubinsBicycle : public FirstOrderDubinsBicycleImpl<FirstOrderDubinsBicycle>
{
public:
  FirstOrderDubinsBicycle(cudaStream_t stream = nullptr) : FirstOrderDubinsBicycleImpl<FirstOrderDubinsBicycle>(stream)
  {
  }

  FirstOrderDubinsBicycle(FirstOrderDubinsBicycleParams& params, cudaStream_t stream = nullptr)
    : FirstOrderDubinsBicycleImpl<FirstOrderDubinsBicycle>(params, stream)
  {
  }
};

#if __CUDACC__
#include "first_order_dubins_bicycle.cu"
#endif

#endif  // MPPIGENERIC_FIRST_ORDER_DUBINS_BICYCLE_CUH
