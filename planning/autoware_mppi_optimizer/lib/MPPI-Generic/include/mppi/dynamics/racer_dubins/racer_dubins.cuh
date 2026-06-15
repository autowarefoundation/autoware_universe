#ifndef MPPIGENERIC_RACER_DUBINS_CUH
#define MPPIGENERIC_RACER_DUBINS_CUH

#include <mppi/dynamics/dynamics.cuh>
#include <mppi/utils/angle_utils.cuh>

namespace RACER
{
template <class OUTPUT_T, class TEX_T>
__device__ __host__ static void computeStaticSettling(TEX_T* tex_helper, const float yaw, const float x, const float y,
                                                      float& roll, float& pitch, float* output);
};

struct RacerDubinsParams : public DynamicsParams
{
  enum class StateIndex : int
  {
    VEL_X = 0,
    YAW,
    POS_X,
    POS_Y,
    STEER_ANGLE,
    BRAKE_STATE,
    STEER_ANGLE_RATE,
    NUM_STATES
  };

  enum class ControlIndex : int
  {
    THROTTLE_BRAKE = 0,
    STEER_CMD,
    NUM_CONTROLS
  };

  enum class OutputIndex : int
  {
    BASELINK_VEL_B_X = 0,
    BASELINK_VEL_B_Y,
    BASELINK_POS_I_X,
    BASELINK_POS_I_Y,
    BASELINK_POS_I_Z,
    YAW,
    ROLL,
    PITCH,
    STEER_ANGLE,
    STEER_ANGLE_RATE,
    WHEEL_FORCE_UP_MAX,
    WHEEL_FORCE_FWD_MAX,
    WHEEL_FORCE_SIDE_MAX,
    // WHEEL_FORCE_UP_FL,
    // WHEEL_FORCE_UP_FR,
    // WHEEL_FORCE_UP_RL,
    // WHEEL_FORCE_UP_RR,
    // WHEEL_FORCE_FWD_FL,
    // WHEEL_FORCE_FWD_FR,
    // WHEEL_FORCE_FWD_RL,
    // WHEEL_FORCE_FWD_RR,
    // WHEEL_FORCE_SIDE_FL,
    // WHEEL_FORCE_SIDE_FR,
    // WHEEL_FORCE_SIDE_RL,
    // WHEEL_FORCE_SIDE_RR,
    ACCEL_X,
    ACCEL_Y,
    OMEGA_Z,
    TOTAL_VELOCITY,
    UNCERTAINTY_POS_X,
    UNCERTAINTY_POS_Y,
    UNCERTAINTY_YAW,
    UNCERTAINTY_VEL_X,
    UNCERTAINTY_POS_X_Y,
    UNCERTAINTY_POS_X_YAW,
    UNCERTAINTY_POS_X_VEL_X,
    UNCERTAINTY_POS_Y_YAW,
    UNCERTAINTY_POS_Y_VEL_X,
    UNCERTAINTY_YAW_VEL_X,
    FILLER_1,
    NUM_OUTPUTS
  };

  // engine model component
  float c_t[3] = { 1.3, 2.6, 3.9 };
  float c_b[3] = { 2.5, 3.5, 4.5 };
  float c_v[3] = { 3.7, 4.7, 5.7 };
  float c_0 = 4.9;
  // steering component
  float steering_constant = .6;
  float steer_command_angle_scale = 5;
  float steer_angle_scale = -9.1;
  float max_steer_angle = 0.5;
  float max_steer_rate = 5;
  float steer_accel_constant = 12.1f;
  float steer_accel_drag_constant = 1.0f;
  // brake parametric component
  float brake_delay_constant = 6.6;
  float brake_delay_constant_neg = 8.2;
  float max_brake_rate_neg = 0.9;
  float max_brake_rate_pos = 0.33;
  // system parameters
  float wheel_base = 0.3;
  float low_min_throttle = 0.13;
  float gravity = -9.81;
  int gear_sign = 1;
};

using namespace MPPI_internal;

template <class CLASS_T, class PARAMS_T = RacerDubinsParams>
class RacerDubinsImpl : public Dynamics<CLASS_T, PARAMS_T>
{
public:
  typedef Dynamics<CLASS_T, PARAMS_T> PARENT_CLASS;
  typedef typename PARENT_CLASS::state_array state_array;
  typedef typename PARENT_CLASS::control_array control_array;
  typedef typename PARENT_CLASS::output_array output_array;
  typedef typename PARENT_CLASS::dfdx dfdx;
  typedef typename PARENT_CLASS::dfdu dfdu;
  using PARENT_CLASS::updateState;  // needed as overloading updateState here hides all parent versions of updateState

  RacerDubinsImpl(cudaStream_t stream = nullptr) : PARENT_CLASS(stream)
  {
  }

  RacerDubinsImpl(PARAMS_T& params, cudaStream_t stream = nullptr) : PARENT_CLASS(params, stream)
  {
  }
  std::string getDynamicsModelName() const override
  {
    return "RACER Dubins Model";
  }

  void computeDynamics(const Eigen::Ref<const state_array>& state, const Eigen::Ref<const control_array>& control,
                       Eigen::Ref<state_array> state_der);

  // void computeStateDeriv(const Eigen::Ref<const state_array>& state, const Eigen::Ref<const control_array>& control,
  //                         Eigen::Ref<state_array> state_der, output_array* output=nullptr); // TODO

  void computeParametricDelayDeriv(const Eigen::Ref<const state_array>& state,
                                   const Eigen::Ref<const control_array>& control, Eigen::Ref<state_array> state_der);

  void computeParametricSteerDeriv(const Eigen::Ref<const state_array>& state,
                                   const Eigen::Ref<const control_array>& control, Eigen::Ref<state_array> state_der);

  __device__ void computeParametricDelayDeriv(float* state, float* control, float* state_der, PARAMS_T* params_p);

  __device__ void computeParametricSteerDeriv(float* state, float* control, float* state_der, PARAMS_T* params_p);

  void updateState(const Eigen::Ref<const state_array> state, Eigen::Ref<state_array> next_state,
                   Eigen::Ref<state_array> state_der, const float dt);

  __device__ void updateState(float* state, float* next_state, float* state_der, const float dt);

  state_array interpolateState(const Eigen::Ref<state_array> state_1, const Eigen::Ref<state_array> state_2,
                               const float alpha);

  bool computeGrad(const Eigen::Ref<const state_array>& state, const Eigen::Ref<const control_array>& control,
                   Eigen::Ref<dfdx> A, Eigen::Ref<dfdu> B);

  __device__ void computeDynamics(float* state, float* control, float* state_der, float* theta = nullptr);

  __host__ __device__ void setOutputs(const float* state_der, const float* next_state, float* output);

  // __device__ void computeStateDeriv(float* state, float* control, float* state_der, float* theta_s, float
  // *output=nullptr); // TODO

  void getStoppingControl(const Eigen::Ref<const state_array>& state, Eigen::Ref<control_array> u);

  Eigen::Quaternionf attitudeFromState(const Eigen::Ref<const state_array>& state);
  Eigen::Vector3f positionFromState(const Eigen::Ref<const state_array>& state);
  Eigen::Vector3f velocityFromState(const Eigen::Ref<const state_array>& state);
  Eigen::Vector3f angularRateFromState(const Eigen::Ref<const state_array>& state);
  state_array stateFromOdometry(const Eigen::Quaternionf& q, const Eigen::Vector3f& pos, const Eigen::Vector3f& vel,
                                const Eigen::Vector3f& omega);

  void enforceLeash(const Eigen::Ref<const state_array>& state_true, const Eigen::Ref<const state_array>& state_nominal,
                    const Eigen::Ref<const state_array>& leash_values, Eigen::Ref<state_array> state_output);

  state_array stateFromMap(const std::map<std::string, float>& map) override;
};

class RacerDubins : public RacerDubinsImpl<RacerDubins>
{
public:
  RacerDubins(cudaStream_t stream = nullptr) : RacerDubinsImpl<RacerDubins>(stream)
  {
  }

  RacerDubins(RacerDubinsParams& params, cudaStream_t stream = nullptr) : RacerDubinsImpl<RacerDubins>(params, stream)
  {
  }
};

#if __CUDACC__
#include "racer_dubins.cu"
#endif

#endif  // MPPIGENERIC_RACER_DUBINS_CUH
