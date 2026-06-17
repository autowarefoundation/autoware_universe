/*
 * Created on Tue Jun 02 2020 by Bogdan Vlahov
 *
 */
#ifndef QUADROTOR_DYNAMICS_CUH_
#define QUADROTOR_DYNAMICS_CUH_

#include <mppi/dynamics/dynamics.cuh>

struct QuadrotorDynamicsParams : public DynamicsParams
{
  enum class StateIndex : int
  {
    POS_X = 0,
    POS_Y,
    POS_Z,
    VEL_X,
    VEL_Y,
    VEL_Z,
    QUAT_W,
    QUAT_X,
    QUAT_Y,
    QUAT_Z,
    ANG_VEL_X,
    ANG_VEL_Y,
    ANG_VEL_Z,
    NUM_STATES
  };

  enum class ControlIndex : int
  {
    ANG_RATE_X = 0,
    ANG_RATE_Y,
    ANG_RATE_Z,
    THRUST,
    NUM_CONTROLS
  };

  enum class OutputIndex : int
  {
    POS_X = 0,
    POS_Y,
    POS_Z,
    VEL_X,
    VEL_Y,
    VEL_Z,
    QUAT_W,
    QUAT_X,
    QUAT_Y,
    QUAT_Z,
    ANG_VEL_X,
    ANG_VEL_Y,
    ANG_VEL_Z,
    NUM_OUTPUTS
  };
  float tau_roll = 0.25;
  float tau_pitch = 0.25;
  float tau_yaw = 0.25;
  float mass = 1;  // kg
  QuadrotorDynamicsParams(float mass_in) : mass(mass_in){};
  QuadrotorDynamicsParams() = default;
  ~QuadrotorDynamicsParams() = default;
};

using namespace MPPI_internal;

class QuadrotorDynamics : public Dynamics<QuadrotorDynamics, QuadrotorDynamicsParams>
{
  /**
   * State for this class is defined as follows:
   *    x - position in 3D space (x, y, z) - meters
   *    v - velocity in 3D space (v_x, v_y_ v_z) - meters/sec
   *    q - quaternion (q_w, q_x, q_y, q_z)
   *    w - angular velocities (roll_dot, pitch_dot, yaw_dot) - rad/sec
   *
   * Coordinate Frame is NWU
   *
   * Control:
   *    roll_rate  - rad/sec
   *    pitch_rate - rad/sec
   *    yaw_rate   - rad/sec
   *    thrust     - Newtons
   */
public:
  using PARENT_CLASS = Dynamics<QuadrotorDynamics, QuadrotorDynamicsParams>;

  using state_array = typename PARENT_CLASS::state_array;
  using control_array = typename PARENT_CLASS::control_array;
  using dfdx = typename PARENT_CLASS::dfdx;
  using dfdu = typename PARENT_CLASS::dfdu;

  // Constructor
  QuadrotorDynamics(cudaStream_t stream = 0);
  QuadrotorDynamics(std::array<float2, CONTROL_DIM> control_rngs, cudaStream_t stream = 0);

  using PARENT_CLASS::updateState;  // needed as overloading updateState here hides all parent versions of updateState

  std::string getDynamicsModelName() const override
  {
    return "Quadrotor Model";
  }

  void computeDynamics(const Eigen::Ref<const state_array>& state, const Eigen::Ref<const control_array>& control,
                       Eigen::Ref<state_array> state_der);

  bool computeGrad(const Eigen::Ref<const state_array>& state, const Eigen::Ref<const control_array>& control,
                   Eigen::Ref<dfdx> A, Eigen::Ref<dfdu> B);

  void updateState(const Eigen::Ref<const state_array> state, Eigen::Ref<state_array> next_state,
                   Eigen::Ref<state_array> state_der, const float dt);

  void printState(float* state);

  __device__ void computeDynamics(float* state, float* control, float* state_der, float* theta = nullptr);

  __device__ void updateState(float* state, float* next_state, float* state_der, const float dt);

  state_array stateFromMap(const std::map<std::string, float>& map) override;

  __host__ __device__ void getZeroState(float* state) const;

  state_array getZeroState() const;
};

#if __CUDACC__
#include "quadrotor_dynamics.cu"
#endif
#endif  // QUADROTOR_DYNAMICS_CUH_
