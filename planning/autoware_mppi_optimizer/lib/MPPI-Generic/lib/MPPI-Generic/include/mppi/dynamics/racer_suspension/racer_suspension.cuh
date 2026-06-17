#ifndef MPPIGENERIC_RACER_SUSPENSION_CUH
#define MPPIGENERIC_RACER_SUSPENSION_CUH

#include <mppi/dynamics/dynamics.cuh>
#include <mppi/utils/texture_helpers/two_d_texture_helper.cuh>
#include <mppi/utils/angle_utils.cuh>

struct RacerSuspensionParams : public DynamicsParams
{
  enum class StateIndex : int
  {
    P_I_X = 0,
    P_I_Y,
    P_I_Z,
    ATTITUDE_QW,
    ATTITUDE_QX,
    ATTITUDE_QY,
    ATTITUDE_QZ,
    V_I_X,
    V_I_Y,
    V_I_Z,
    OMEGA_B_X,
    OMEGA_B_Y,
    OMEGA_B_Z,
    STEER_ANGLE,
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
    BASELINK_VEL_B_Z,
    BASELINK_POS_I_X,
    BASELINK_POS_I_Y,
    BASELINK_POS_I_Z,
    YAW,
    ROLL,
    PITCH,
    STEER_ANGLE,
    STEER_ANGLE_RATE,
    WHEEL_POS_I_FL_X,
    WHEEL_POS_I_FL_Y,
    WHEEL_POS_I_FR_X,
    WHEEL_POS_I_FR_Y,
    WHEEL_POS_I_RL_X,
    WHEEL_POS_I_RL_Y,
    WHEEL_POS_I_RR_X,
    WHEEL_POS_I_RR_Y,
    WHEEL_FORCE_B_FL,
    WHEEL_FORCE_B_FR,
    WHEEL_FORCE_B_RL,
    WHEEL_FORCE_B_RR,
    ACCEL_X,
    ACCEL_Y,
    OMEGA_Z,
    NUM_OUTPUTS
  };
  // suspension model params
  float wheel_radius = 0.32;
  float mass = 1447;
  float wheel_base = 2.981;
  float width = 1.5;
  float height = 1.5;
  float gravity = -9.81;
  float k_s[4] = { 14000, 14000, 14000, 14000 };
  float c_s[4] = { 2000, 2000, 2000, 2000 };
  float l_0[4] = {
    wheel_radius + mass / 4 * (-gravity) / k_s[0],
    wheel_radius + mass / 4 * (-gravity) / k_s[1],
    wheel_radius + mass / 4 * (-gravity) / k_s[2],
    wheel_radius + mass / 4 * (-gravity) / k_s[3],
  };
  float3 cg_pos_wrt_base_link = make_float3(wheel_base / 2, 0, 0.2);
  // float3 wheel_pos_front_left = make_float3(wheel_base, width/2, 0);
  // float3 wheel_pos_front_right = make_float3(wheel_base, -width/2, 0);
  // float3 wheel_pos_rear_left = make_float3(0, width/2, 0);
  // float3 wheel_pos_rear_right = make_float3(0, -width/2, 0);
  float3 wheel_pos_wrt_base_link[4] = { make_float3(wheel_base, width / 2, 0), make_float3(wheel_base, -width / 2, 0),
                                        make_float3(0, width / 2, 0), make_float3(0, -width / 2, 0) };
  float Jxx = 1.0 / 12 * mass * (height * height + width * width);
  float Jyy = 1.0 / 12 * mass * (height * height + wheel_base * wheel_base);
  float Jzz = 1.0 / 12 * mass * (wheel_base * wheel_base + width * width);
  float mu = 0.65;
  float v_slip = 0.1;
  static const int WHEEL_FRONT_LEFT = 0;
  static const int WHEEL_FRONT_RIGHT = 1;
  static const int WHEEL_REAR_LEFT = 2;
  static const int WHEEL_REAR_RIGHT = 3;

  // throttle model params
  float c_t = 3.0;
  float c_b = 10.0;
  float c_v = 0.2;
  float c_0 = 0;

  // steering model params
  float steering_constant = .6;
  float steer_command_angle_scale = -2.45;
  int gear_sign = 1;
  RacerSuspensionParams()
  {
    recalcParams();
  }

  void __host__ recalcParams()
  {
    cg_pos_wrt_base_link = make_float3(wheel_base / 2, 0, 0.2);
    l_0[0] = wheel_radius + mass / 4 * (-gravity) / k_s[0];
    l_0[1] = wheel_radius + mass / 4 * (-gravity) / k_s[1];
    l_0[2] = wheel_radius + mass / 4 * (-gravity) / k_s[2];
    l_0[3] = wheel_radius + mass / 4 * (-gravity) / k_s[3];
    wheel_pos_wrt_base_link[0] = make_float3(wheel_base, width / 2, 0);
    wheel_pos_wrt_base_link[1] = make_float3(wheel_base, -width / 2, 0);
    wheel_pos_wrt_base_link[2] = make_float3(0, width / 2, 0);
    wheel_pos_wrt_base_link[3] = make_float3(0, -width / 2, 0);
    Jxx = 1.0 / 12 * mass * (height * height + width * width);
    Jyy = 1.0 / 12 * mass * (height * height + wheel_base * wheel_base);
    Jzz = 1.0 / 12 * mass * (wheel_base * wheel_base + width * width);
  }
};

using namespace MPPI_internal;
/**
 * state: x, y, z, quat [w, x, y, z], vx_I, vy_I, vz_I, omega_x, omega_y, omega_z, shaft angle, shaft angle velocity
 * control: throttle/brake, steering angle command
 * sensors: texture 0 is elevation map (normal x, normal y, normal z, height)
 */
class RacerSuspension : public Dynamics<RacerSuspension, RacerSuspensionParams>
{
public:
  typedef Dynamics<RacerSuspension, RacerSuspensionParams> PARENT_CLASS;
  typedef typename PARENT_CLASS::state_array state_array;
  typedef typename PARENT_CLASS::control_array control_array;
  typedef typename PARENT_CLASS::output_array output_array;
  typedef typename PARENT_CLASS::dfdx dfdx;
  typedef typename PARENT_CLASS::dfdu dfdu;
  using PARENT_CLASS::updateState;  // needed as overloading updateState here hides all parent versions of updateState

  // number of floats for computing the state derivative BLOCK_DIM_X * BLOCK_DIM_Z times
  static const int SHARED_MEM_REQUEST_BLK_BYTES = 0;

  static const int TEXTURE_ELEVATION_MAP = 0;

  RacerSuspension(cudaStream_t stream = nullptr) : PARENT_CLASS(stream)
  {
    tex_helper_ = new TwoDTextureHelper<float>(1, stream);
  }
  RacerSuspension(RacerSuspensionParams& params, cudaStream_t stream) : PARENT_CLASS(params, stream)
  {
    tex_helper_ = new TwoDTextureHelper<float>(1, stream);
  }

  std::string getDynamicsModelName() const override
  {
    return "RACER Suspension Model";
  }

  ~RacerSuspension()
  {
    delete tex_helper_;
  }

  void GPUSetup();

  void freeCudaMem();

  void paramsToDevice();

  void updateState(const Eigen::Ref<const state_array> state, Eigen::Ref<state_array> next_state,
                   Eigen::Ref<state_array> state_der, const float dt);
  void step(Eigen::Ref<state_array> state, Eigen::Ref<state_array> next_state, Eigen::Ref<state_array> state_der,
            const Eigen::Ref<const control_array> control, Eigen::Ref<output_array> output, const float t,
            const float dt);

  __device__ __host__ void computeStateDeriv(const Eigen::Ref<const state_array>& state,
                                             const Eigen::Ref<const control_array>& control,
                                             Eigen::Ref<state_array> state_der, Eigen::Ref<output_array> output,
                                             Eigen::Matrix3f* omegaJacobian = nullptr);

  __device__ void updateState(float* state, float* next_state, float* state_der, const float dt);

  __device__ void computeStateDeriv(float* state, float* control, float* state_der, float* theta_s,
                                    float* output = nullptr);
  void enforceLeash(const Eigen::Ref<const state_array>& state_true, const Eigen::Ref<const state_array>& state_nominal,
                    const Eigen::Ref<const state_array>& leash_values, Eigen::Ref<state_array> state_output);

  __device__ void step(float* state, float* next_state, float* state_der, float* control, float* output, float* theta_s,
                       const float t, const float dt);

  TwoDTextureHelper<float>* getTextureHelper()
  {
    return tex_helper_;
  }

  Eigen::Quaternionf attitudeFromState(const Eigen::Ref<const state_array>& state);
  Eigen::Vector3f positionFromState(const Eigen::Ref<const state_array>& state);
  Eigen::Vector3f velocityFromState(const Eigen::Ref<const state_array>& state);
  Eigen::Vector3f angularRateFromState(const Eigen::Ref<const state_array>& state);
  state_array stateFromOdometry(const Eigen::Quaternionf& q_B_to_I, const Eigen::Vector3f& pos_base_link_I,
                                const Eigen::Vector3f& vel_base_link_B, const Eigen::Vector3f& omega_B);

  state_array stateFromMap(const std::map<std::string, float>& map) override;

protected:
  TwoDTextureHelper<float>* tex_helper_ = nullptr;
};

#if __CUDACC__
#include "racer_suspension.cu"
#endif

#endif  // MPPIGENERIC_RACER_SUSPENSION_CUH
