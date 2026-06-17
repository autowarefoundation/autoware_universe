//
// Created by jason on 8/31/22.
//

#include "racer_dubins_elevation.cuh"
#include <mppi/utils/nn_helpers/lstm_lstm_helper.cuh>

#ifndef MPPIGENERIC_RACER_DUBINS_ELEVATION_LSTM_STEERING_CUH
#define MPPIGENERIC_RACER_DUBINS_ELEVATION_LSTM_STEERING_CUH

template <class CLASS_T, class PARAMS_T = RacerDubinsElevationParams>
class RacerDubinsElevationLSTMSteeringImpl : public RacerDubinsElevationImpl<CLASS_T, PARAMS_T>
{
  static_assert(std::is_base_of<RacerDubinsElevationParams, PARAMS_T>::value,
                "Params don't inherit from RacerDubinsElevationParams.");

public:
  using PARENT_CLASS = RacerDubinsElevationImpl<CLASS_T, PARAMS_T>;
  using DYN_PARAMS_T = typename PARENT_CLASS::DYN_PARAMS_T;
  using SharedBlock = typename PARENT_CLASS::SharedBlock;
  using state_array = typename PARENT_CLASS::state_array;
  using control_array = typename PARENT_CLASS::control_array;
  using output_array = typename PARENT_CLASS::output_array;
  using buffer_trajectory = typename PARENT_CLASS::buffer_trajectory;
  using PARENT_CLASS::computeParametricAccelDeriv;
  using PARENT_CLASS::computeParametricDelayDeriv;
  using PARENT_CLASS::computeUncertaintyPropagation;
  using PARENT_CLASS::setOutputs;

  RacerDubinsElevationLSTMSteeringImpl(LSTMLSTMConfig config, cudaStream_t stream = nullptr)
    : RacerDubinsElevationLSTMSteeringImpl(config.init_config.input_dim, config.init_config.hidden_dim,
                                           config.init_config.output_layers, config.pred_config.input_dim,
                                           config.pred_config.hidden_dim, config.pred_config.output_layers,
                                           config.init_len, stream)
  {
  }

  RacerDubinsElevationLSTMSteeringImpl(int init_input_dim, int init_hidden_dim, std::vector<int>& init_output_layers,
                                       int input_dim, int hidden_dim, std::vector<int>& output_layers, int init_len,
                                       cudaStream_t stream = nullptr);
  RacerDubinsElevationLSTMSteeringImpl(std::string path, cudaStream_t stream = nullptr);

  std::string getDynamicsModelName() const override
  {
    return "RACER Dubins LSTM Steering Model";
  }

  void bindToStream(cudaStream_t stream);

  void GPUSetup();

  void freeCudaMem();

  bool updateFromBuffer(const buffer_trajectory& buffer);

  void computeLSTMSteering(Eigen::Ref<state_array> state, const Eigen::Ref<const control_array>& control,
                           Eigen::Ref<state_array> state_der);

  void step(Eigen::Ref<state_array> state, Eigen::Ref<state_array> next_state, Eigen::Ref<state_array> state_der,
            const Eigen::Ref<const control_array>& control, Eigen::Ref<output_array> output, const float t,
            const float dt);

  void initializeDynamics(const Eigen::Ref<const state_array>& state, const Eigen::Ref<const control_array>& control,
                          Eigen::Ref<output_array> output, float t_0, float dt);

  __device__ inline void step(float* state, float* next_state, float* state_der, float* control, float* output,
                              float* theta_s, const float t, const float dt);

  __device__ void initializeDynamics(float* state, float* control, float* output, float* theta_s, float t_0, float dt);

  __device__ void updateState(float* state, float* next_state, float* state_der, const float dt);

  __device__ void computeLSTMSteering(float* state, float* control, float* state_der, DYN_PARAMS_T* params_p,
                                      float* theta_s, const int grd_shift, const int blk_shift, const int sb_shift);

  void updateState(const Eigen::Ref<const state_array> state, Eigen::Ref<state_array> next_state,
                   Eigen::Ref<state_array> state_der, const float dt);

  std::shared_ptr<LSTMLSTMHelper<>> getHelper()
  {
    return lstm_lstm_helper_;
  }

  LSTMHelper<>* network_d_ = nullptr;

protected:
  std::shared_ptr<LSTMLSTMHelper<>> lstm_lstm_helper_;

  RacerDubinsElevationLSTMSteeringImpl(cudaStream_t stream = 0)
  {
  }
};

class RacerDubinsElevationLSTMSteering : public RacerDubinsElevationLSTMSteeringImpl<RacerDubinsElevationLSTMSteering>
{
public:
  using PARENT_CLASS = RacerDubinsElevationLSTMSteeringImpl<RacerDubinsElevationLSTMSteering>;
  RacerDubinsElevationLSTMSteering(int init_input_dim, int init_hidden_dim, std::vector<int>& init_output_layers,
                                   int input_dim, int hidden_dim, std::vector<int>& output_layers, int init_len,
                                   cudaStream_t stream = nullptr)
    : PARENT_CLASS(init_input_dim, init_hidden_dim, init_output_layers, input_dim, hidden_dim, output_layers, init_len,
                   stream)
  {
  }
  RacerDubinsElevationLSTMSteering(std::string path, cudaStream_t stream = nullptr) : PARENT_CLASS(path, stream)
  {
  }

protected:
  RacerDubinsElevationLSTMSteering(cudaStream_t stream = nullptr) : PARENT_CLASS(stream)
  {
  }
};

#if __CUDACC__
#include "racer_dubins_elevation_lstm_steering.cu"
#endif

#endif  // MPPIGENERIC_RACER_DUBINS_ELEVATION_LSTM_STEERING_CUH
