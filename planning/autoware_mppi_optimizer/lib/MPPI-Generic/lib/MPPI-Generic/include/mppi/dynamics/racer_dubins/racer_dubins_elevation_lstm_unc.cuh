#include "racer_dubins_elevation_suspension_lstm.cuh"

#ifndef MPPIGENERIC_RACER_DUBINS_ELEVATION_LSTM_UNC_CUH
#define MPPIGENERIC_RACER_DUBINS_ELEVATION_LSTM_UNC_CUH

struct RacerDubinsElevationUncertaintyParams : public RacerDubinsElevationSuspensionParams
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
    CG_POS_Z,
    CG_VEL_I_Z,
    ROLL_RATE,
    PITCH_RATE,
    STEER_ANGLE_RATE,
    OMEGA_Z,
    STATIC_ROLL,
    STATIC_PITCH,
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

  float unc_scale[7] = { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
  float pos_quad_brake_c[3] = { 2.0f, 0.5f, 0.3 };
  float neg_quad_brake_c[3] = { 5.84f, 0.15f, 1.7f };
  bool use_static_settling = true;
};

class RacerDubinsElevationLSTMUncertainty
  : public RacerDubinsElevationSuspensionImpl<RacerDubinsElevationLSTMUncertainty,
                                              RacerDubinsElevationUncertaintyParams>
{
public:
  using PARENT_CLASS =
      RacerDubinsElevationSuspensionImpl<RacerDubinsElevationLSTMUncertainty, RacerDubinsElevationUncertaintyParams>;

  typedef typename PARENT_CLASS::DYN_PARAMS_T DYN_PARAMS_T;

  typedef typename PARENT_CLASS::state_array state_array;
  typedef typename PARENT_CLASS::control_array control_array;
  typedef typename PARENT_CLASS::output_array output_array;
  typedef typename PARENT_CLASS::dfdx dfdx;
  typedef typename PARENT_CLASS::dfdu dfdu;

  RacerDubinsElevationLSTMUncertainty(LSTMLSTMConfig& steer_config, LSTMLSTMConfig& mean_config,
                                      LSTMLSTMConfig& unc_config, cudaStream_t stream = nullptr);
  RacerDubinsElevationLSTMUncertainty(std::string path, cudaStream_t stream = nullptr);

  std::string getDynamicsModelName() const override
  {
    // TODO check if mean model is none or not
    return "RACER Dubins LSTM Uncertainty Model";
  }

  void GPUSetup();
  void bindToStream(cudaStream_t stream);
  void freeCudaMem();

  bool updateFromBuffer(const buffer_trajectory& buffer);

  void step(Eigen::Ref<state_array> state, Eigen::Ref<state_array> next_state, Eigen::Ref<state_array> state_der,
            const Eigen::Ref<const control_array>& control, Eigen::Ref<output_array> output, const float t,
            const float dt);

  void initializeDynamics(const Eigen::Ref<const state_array>& state, const Eigen::Ref<const control_array>& control,
                          Eigen::Ref<output_array> output, float t_0, float dt);
  state_array stateFromMap(const std::map<std::string, float>& map);

  __host__ __device__ bool computeQ(const float* state, const float* control, const float* state_der, float* Q,
                                    DYN_PARAMS_T* params_p, float* theta_s);

  __device__ inline void step(float* state, float* next_state, float* state_der, float* control, float* output,
                              float* theta_s, const float t, const float dt);

  __device__ void initializeDynamics(float* state, float* control, float* output, float* theta_s, float t_0, float dt);

  // __device__ void updateState(float* state, float* next_state, float* state_der, const float dt);

  // void updateState(const Eigen::Ref<const state_array> state, Eigen::Ref<state_array> next_state,
  //                  Eigen::Ref<state_array> state_der, const float dt);

  std::shared_ptr<LSTMLSTMHelper<>> getUncertaintyHelper()
  {
    return uncertainty_helper_;
  }
  std::shared_ptr<LSTMLSTMHelper<>> getMeanHelper()
  {
    return mean_helper_;
  }

  LSTMHelper<>* uncertainty_d_ = nullptr;
  LSTMHelper<>* mean_d_ = nullptr;

protected:
  std::shared_ptr<LSTMLSTMHelper<>> uncertainty_helper_;
  std::shared_ptr<LSTMLSTMHelper<>> mean_helper_;
};

#if __CUDACC__
#include "racer_dubins_elevation_lstm_unc.cu"
#endif

#endif  // MPPIGENERIC_RACER_DUBINS_ELEVATION_LSTM_UNC_CUH
