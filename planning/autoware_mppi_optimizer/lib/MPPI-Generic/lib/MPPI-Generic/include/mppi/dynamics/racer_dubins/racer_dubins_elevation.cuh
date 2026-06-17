#ifndef MPPIGENERIC_RACER_DUBINS_ELEVATION_CUH
#define MPPIGENERIC_RACER_DUBINS_ELEVATION_CUH

#include <mppi/dynamics/dynamics.cuh>
#include <mppi/dynamics/racer_dubins/racer_dubins.cuh>
#include <mppi/utils/texture_helpers/two_d_texture_helper.cuh>
#include <mppi/utils/angle_utils.cuh>

using namespace MPPI_internal;

#ifndef U_INDEX
#define U_IND_CLASS(CLASS, enum_val) E_INDEX(CLASS::UncertaintyIndex, enum_val)
#define U_INDEX(enum_val) U_IND_CLASS(PARENT_CLASS::DYN_PARAMS_T, enum_val)
#endif

struct RacerDubinsElevationParams : public RacerDubinsParams
{
  enum class StateIndex : int
  {
    VEL_X = 0,
    YAW,
    POS_X,
    POS_Y,
    STEER_ANGLE,
    BRAKE_STATE,
    ROLL,
    PITCH,
    STEER_ANGLE_RATE,
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
    NUM_STATES
  };
  enum class UncertaintyIndex : int
  {
    VEL_X = 0,
    YAW,
    POS_X,
    POS_Y,
    NUM_UNCERTAINTIES
  };
  float clamp_ax = 5.5f;
  // Uncertainty feedback and Noise coefficients
  float K_x = 1.0f;                                       // feedback for pos_x
  float K_y = 1.0f;                                       // feedback for pos_y
  float K_yaw = 1.0f;                                     // feedback for yaw
  float K_vel_x = 1.0f;                                   // feedback for vel x
  float Q_x_acc = 1.0f;                                   // Add noise to vel x based on accel_x
  float Q_x_v[3] = { 41.74219, -0.8187027, -2.2131343 };  // Add noise to vel x based on vel x
  float Q_y_f = 0.1f;                                     // Add noise to pos x and y based on side force
  float Q_omega_v = 0.001f;                               // Add noise to yaw based on vel x
  float Q_omega_steering = 0.0f;                          // Add noise to yaw based on steering
};

template <class CLASS_T, class PARAMS_T>
class RacerDubinsElevationImpl : public RacerDubinsImpl<CLASS_T, PARAMS_T>
{
public:
  // static const int SHARED_MEM_REQUEST_GRD_BYTES = sizeof(DYN_PARAMS_T);
  using PARENT_CLASS = RacerDubinsImpl<CLASS_T, PARAMS_T>;
  using PARENT_CLASS::initializeDynamics;

  typedef PARAMS_T DYN_PARAMS_T;
  static const int UNCERTAINTY_DIM = U_IND_CLASS(PARAMS_T, NUM_UNCERTAINTIES);

  struct __align__(16) SharedBlock
  {
    float A[UNCERTAINTY_DIM * UNCERTAINTY_DIM] MPPI_ALIGN(16) = { 0.0f };
    float Sigma_a[UNCERTAINTY_DIM * UNCERTAINTY_DIM] MPPI_ALIGN(16) = { 0.0f };
    float Sigma_b[UNCERTAINTY_DIM * UNCERTAINTY_DIM] MPPI_ALIGN(16) = { 0.0f };
  };

  typedef typename PARENT_CLASS::state_array state_array;
  typedef typename PARENT_CLASS::control_array control_array;
  typedef typename PARENT_CLASS::output_array output_array;
  typedef typename PARENT_CLASS::dfdx dfdx;
  typedef typename PARENT_CLASS::dfdu dfdu;

  RacerDubinsElevationImpl(cudaStream_t stream = nullptr) : PARENT_CLASS(stream)
  {
    tex_helper_ = new TwoDTextureHelper<float>(1, stream);
    this->SHARED_MEM_REQUEST_BLK_BYTES = sizeof(SharedBlock);
  }
  RacerDubinsElevationImpl(PARAMS_T& params, cudaStream_t stream = nullptr) : PARENT_CLASS(params, stream)
  {
    tex_helper_ = new TwoDTextureHelper<float>(1, stream);
    this->SHARED_MEM_REQUEST_BLK_BYTES = sizeof(SharedBlock);
  }

  std::string getDynamicsModelName() const override
  {
    return "RACER Dubins w/ Elevation Model";
  }

  ~RacerDubinsElevationImpl()
  {
    freeCudaMem();
    delete tex_helper_;
  }

  void updateState(const Eigen::Ref<const state_array> state, Eigen::Ref<state_array> next_state,
                   Eigen::Ref<state_array> state_der, const float dt)
  {
    this->PARENT_CLASS::updateState(state, next_state, state_der, dt);
  }

  void updateRotation(std::array<float3, 3>& rotation)
  {
    this->tex_helper_->updateRotation(0, rotation);
  }

  void bindToStream(cudaStream_t stream)
  {
    this->tex_helper_->bindToStream(stream);
    this->PARENT_CLASS::bindToStream(stream);
  }

  void GPUSetup();

  void freeCudaMem();

  void paramsToDevice();

  void computeParametricAccelDeriv(const Eigen::Ref<const state_array>& state,
                                   const Eigen::Ref<const control_array>& control, Eigen::Ref<state_array> state_der,
                                   const float dt);

  __host__ __device__ void setOutputs(const float* state_der, const float* next_state, float* output);

  __device__ void computeParametricAccelDeriv(float* state, float* control, float* state_der, const float dt,
                                              DYN_PARAMS_T* params_p);

  bool computeGrad(const Eigen::Ref<const state_array>& state = state_array(),
                   const Eigen::Ref<const control_array>& control = control_array(), Eigen::Ref<dfdx> A = dfdx(),
                   Eigen::Ref<dfdu> B = dfdu());

  __host__ __device__ void computeUncertaintyPropagation(const float* state, const float* control,
                                                         const float* state_der, float* next_state, float dt,
                                                         DYN_PARAMS_T* params_p, SharedBlock* uncertainty_data,
                                                         float* theta_s);

  __host__ __device__ void uncertaintyMatrixToOutput(const float* uncertainty_matrix, float* output);
  __host__ __device__ void uncertaintyMatrixToState(const float* uncertainty_matrix, float* state);
  __host__ __device__ void uncertaintyStateToMatrix(const float* state, float* uncertainty_matrix);
  __host__ __device__ bool computeUncertaintyJacobian(const float* state, const float* control, float* A,
                                                      DYN_PARAMS_T* params_p);
  __host__ __device__ bool computeQ(const float* state, const float* control, const float* state_der, float* Q,
                                    DYN_PARAMS_T* params_p, float* theta_s);

  __device__ void updateState(float* state, float* next_state, float* state_der, const float dt,
                              DYN_PARAMS_T* params_p);

  void computeStateDeriv(const Eigen::Ref<const state_array>& state, const Eigen::Ref<const control_array>& control,
                         Eigen::Ref<state_array> state_der)
  {
  }

  state_array computeStateError(const Eigen::Ref<const state_array>& pred_state,
                                const Eigen::Ref<const state_array>& true_state)
  {
    state_array state_diff = pred_state - true_state;
    state_diff(S_INDEX(YAW)) = angle_utils::shortestAngularDistance(true_state(S_INDEX(YAW)), pred_state(S_INDEX(YAW)));
    state_diff(S_INDEX(ROLL)) =
        angle_utils::shortestAngularDistance(true_state(S_INDEX(ROLL)), pred_state(S_INDEX(ROLL)));
    state_diff(S_INDEX(PITCH)) =
        angle_utils::shortestAngularDistance(true_state(S_INDEX(PITCH)), pred_state(S_INDEX(PITCH)));
    return state_diff;
  }

  void step(Eigen::Ref<state_array> state, Eigen::Ref<state_array> next_state, Eigen::Ref<state_array> state_der,
            const Eigen::Ref<const control_array>& control, Eigen::Ref<output_array> output, const float t,
            const float dt);

  __device__ inline void step(float* state, float* next_state, float* state_der, float* control, float* output,
                              float* theta_s, const float t, const float dt);

  __device__ void initializeDynamics(float* state, float* control, float* output, float* theta_s, float t_0, float dt);

  Eigen::Quaternionf attitudeFromState(const Eigen::Ref<const state_array>& state);
  Eigen::Vector3f positionFromState(const Eigen::Ref<const state_array>& state);

  state_array stateFromMap(const std::map<std::string, float>& map) override;

  TwoDTextureHelper<float>* getTextureHelper()
  {
    return tex_helper_;
  }

protected:
  TwoDTextureHelper<float>* tex_helper_ = nullptr;
};

class RacerDubinsElevation : public RacerDubinsElevationImpl<RacerDubinsElevation, RacerDubinsElevationParams>
{
public:
  RacerDubinsElevation(cudaStream_t stream = nullptr)
    : RacerDubinsElevationImpl<RacerDubinsElevation, RacerDubinsElevationParams>(stream)
  {
  }
  RacerDubinsElevation(RacerDubinsElevationParams& params, cudaStream_t stream = nullptr)
    : RacerDubinsElevationImpl<RacerDubinsElevation, RacerDubinsElevationParams>(params, stream)
  {
  }
};

#if __CUDACC__
#include "racer_dubins_elevation.cu"
#endif

#endif  // MPPIGENERIC_RACER_DUBINS_ELEVATION_CUH
