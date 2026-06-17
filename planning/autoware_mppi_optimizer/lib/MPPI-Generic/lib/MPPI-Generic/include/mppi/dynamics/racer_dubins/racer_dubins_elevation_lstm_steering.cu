//
// Created by jason on 8/31/22.
//

#include "racer_dubins_elevation_lstm_steering.cuh"

#define TEMPLATE_TYPE template <class CLASS_T, class PARAMS_T>
#define TEMPLATE_NAME RacerDubinsElevationLSTMSteeringImpl<CLASS_T, PARAMS_T>

TEMPLATE_TYPE
TEMPLATE_NAME::RacerDubinsElevationLSTMSteeringImpl(int init_input_dim, int init_hidden_dim,
                                                    std::vector<int>& init_output_layers, int input_dim, int hidden_dim,
                                                    std::vector<int>& output_layers, int init_len, cudaStream_t stream)
  : PARENT_CLASS(stream)
{
  this->requires_buffer_ = true;
  lstm_lstm_helper_ = std::make_shared<LSTMLSTMHelper<>>(init_input_dim, init_hidden_dim, init_output_layers, input_dim,
                                                         hidden_dim, output_layers, init_len, stream);
  this->SHARED_MEM_REQUEST_GRD_BYTES = lstm_lstm_helper_->getLSTMModel()->getGrdSharedSizeBytes();
  this->SHARED_MEM_REQUEST_BLK_BYTES = sizeof(SharedBlock) + lstm_lstm_helper_->getLSTMModel()->getBlkSharedSizeBytes();
}

TEMPLATE_TYPE
TEMPLATE_NAME::RacerDubinsElevationLSTMSteeringImpl(std::string path, cudaStream_t stream) : PARENT_CLASS(stream)
{
  if (!fileExists(path))
  {
    std::cerr << "Could not load neural net model at path: " << path.c_str();
    exit(-1);
  }
  cnpy::npz_t param_dict = cnpy::npz_load(path);
  this->params_.max_steer_rate = param_dict.at("parameters/max_rate_pos").data<float>()[0];
  this->params_.steering_constant = param_dict.at("parameters/constant").data<float>()[0];
  this->params_.steer_accel_constant = param_dict.at("parameters/accel_constant").data<float>()[0];
  this->params_.steer_accel_drag_constant = param_dict.at("parameters/accel_drag_constant").data<float>()[0];
  lstm_lstm_helper_ = std::make_shared<LSTMLSTMHelper<>>(path, stream);

  this->requires_buffer_ = true;
  this->SHARED_MEM_REQUEST_GRD_BYTES = lstm_lstm_helper_->getLSTMModel()->getGrdSharedSizeBytes();
  this->SHARED_MEM_REQUEST_BLK_BYTES = sizeof(SharedBlock) + lstm_lstm_helper_->getLSTMModel()->getBlkSharedSizeBytes();
}

TEMPLATE_TYPE
void TEMPLATE_NAME::GPUSetup()
{
  lstm_lstm_helper_->GPUSetup();
  // makes sure that the device ptr sees the correct lstm model
  this->network_d_ = lstm_lstm_helper_->getLSTMDevicePtr();
  PARENT_CLASS::GPUSetup();
}

TEMPLATE_TYPE
void TEMPLATE_NAME::bindToStream(cudaStream_t stream)
{
  PARENT_CLASS::bindToStream(stream);
  lstm_lstm_helper_->getLSTMModel()->bindToStream(stream);
}

TEMPLATE_TYPE
void TEMPLATE_NAME::freeCudaMem()
{
  PARENT_CLASS::freeCudaMem();
  lstm_lstm_helper_->freeCudaMem();
}

TEMPLATE_TYPE
void TEMPLATE_NAME::computeLSTMSteering(Eigen::Ref<state_array> state, const Eigen::Ref<const control_array>& control,
                                        Eigen::Ref<state_array> state_der)
{
  const float parametric_accel =
      (control(C_INDEX(STEER_CMD)) * this->params_.steer_command_angle_scale - state(S_INDEX(STEER_ANGLE))) *
      this->params_.steering_constant;
  state_der(S_INDEX(STEER_ANGLE_RATE)) =
      fmaxf(fminf((parametric_accel - state(S_INDEX(STEER_ANGLE_RATE))) * this->params_.steer_accel_constant -
                      state(S_INDEX(STEER_ANGLE_RATE)) * this->params_.steer_accel_drag_constant,
                  this->params_.max_steer_rate),
            -this->params_.max_steer_rate);

  Eigen::VectorXf input = lstm_lstm_helper_->getLSTMModel()->getZeroInputVector();
  input(0) = state(S_INDEX(STEER_ANGLE)) * 0.2f;
  input(1) = state(S_INDEX(STEER_ANGLE_RATE)) * 0.2f;
  input(2) = control(C_INDEX(STEER_CMD));
  input(3) = state_der(S_INDEX(STEER_ANGLE_RATE)) * 0.2f;  // this is the parametric part as input
  Eigen::VectorXf nn_output = lstm_lstm_helper_->getLSTMModel()->getZeroOutputVector();
  lstm_lstm_helper_->forward(input, nn_output);
  state_der(S_INDEX(STEER_ANGLE_RATE)) += nn_output(0) * 5.0f;
  state_der(S_INDEX(STEER_ANGLE)) = state(S_INDEX(STEER_ANGLE_RATE));
}

TEMPLATE_TYPE
void TEMPLATE_NAME::step(Eigen::Ref<state_array> state, Eigen::Ref<state_array> next_state,
                         Eigen::Ref<state_array> state_der, const Eigen::Ref<const control_array>& control,
                         Eigen::Ref<output_array> output, const float t, const float dt)
{
  this->computeParametricDelayDeriv(state, control, state_der);
  this->computeParametricAccelDeriv(state, control, state_der, dt);
  this->computeLSTMSteering(state, control, state_der);

  // Integrate using racer_dubins updateState
  updateState(state, next_state, state_der, dt);
  SharedBlock sb;
  computeUncertaintyPropagation(state.data(), control.data(), state_der.data(), next_state.data(), dt, &this->params_,
                                &sb, nullptr);

  float roll = state(S_INDEX(ROLL));
  float pitch = state(S_INDEX(PITCH));
  RACER::computeStaticSettling<typename DYN_PARAMS_T::OutputIndex, TwoDTextureHelper<float>>(
      this->tex_helper_, next_state(S_INDEX(YAW)), next_state(S_INDEX(POS_X)), next_state(S_INDEX(POS_Y)), roll, pitch,
      output.data());
  next_state[S_INDEX(PITCH)] = pitch;
  next_state[S_INDEX(ROLL)] = roll;

  setOutputs(state_der.data(), next_state.data(), output.data());
}

TEMPLATE_TYPE
__device__ void TEMPLATE_NAME::initializeDynamics(float* state, float* control, float* output, float* theta_s,
                                                  float t_0, float dt)
{
  // const int shift = PARENT_CLASS::SHARED_MEM_REQUEST_GRD_BYTES / sizeof(float);
  // if (PARENT_CLASS::SHARED_MEM_REQUEST_GRD_BYTES != 0)
  // {  // Allows us to turn on or off global or shared memory version of params
  //   DYN_PARAMS_T* shared_params = (DYN_PARAMS_T*)theta_s;
  //   *shared_params = this->params_;
  // }
  network_d_->initialize(theta_s, this->SHARED_MEM_REQUEST_BLK_BYTES, this->SHARED_MEM_REQUEST_GRD_BYTES,
                         sizeof(SharedBlock) / sizeof(float));
  setOutputs(state, state, output);
}

template <class CLASS_T, class PARAMS_T>
__device__ void RacerDubinsElevationLSTMSteeringImpl<CLASS_T, PARAMS_T>::computeLSTMSteering(
    float* state, float* control, float* state_der, DYN_PARAMS_T* params_p, float* theta_s, const int grd_shift,
    const int blk_shift, const int sb_shift)
{
  const uint tdy = threadIdx.y;

  // loads in the input to the network
  float* input_loc = network_d_->getInputLocation(theta_s, grd_shift, blk_shift, sb_shift);
  if (tdy == 0)
  {
    const float parametric_accel =
        (control[C_INDEX(STEER_CMD)] * params_p->steer_command_angle_scale - state[S_INDEX(STEER_ANGLE)]) *
        params_p->steering_constant;
    state_der[S_INDEX(STEER_ANGLE_RATE)] =
        fmaxf(fminf((parametric_accel - state[S_INDEX(STEER_ANGLE_RATE)]) * params_p->steer_accel_constant -
                        state[S_INDEX(STEER_ANGLE_RATE)] * params_p->steer_accel_drag_constant,
                    params_p->max_steer_rate),
              -params_p->max_steer_rate);

    input_loc[0] = state[S_INDEX(STEER_ANGLE)] * 0.2f;
    input_loc[1] = state[S_INDEX(STEER_ANGLE_RATE)] * 0.2f;
    input_loc[2] = control[C_INDEX(STEER_CMD)];
    input_loc[3] = state_der[S_INDEX(STEER_ANGLE_RATE)] * 0.2f;  // this is the parametric part as input
  }
  __syncthreads();
  // runs the network
  float* cur_hidden_cell = network_d_->getHiddenCellLocation(theta_s, grd_shift, blk_shift, sb_shift);
  float* nn_output = network_d_->forward(nullptr, theta_s, cur_hidden_cell);
  // copies the results of the network to state derivative
  if (tdy == 0)
  {
    state_der[S_INDEX(STEER_ANGLE_RATE)] += nn_output[0] * 5.0f;
    state_der[S_INDEX(STEER_ANGLE)] = state[S_INDEX(STEER_ANGLE_RATE)];
  }
  __syncthreads();
}

TEMPLATE_TYPE
__device__ inline void TEMPLATE_NAME::step(float* state, float* next_state, float* state_der, float* control,
                                           float* output, float* theta_s, const float t, const float dt)
{
  DYN_PARAMS_T* params_p;
  SharedBlock* sb;
  // TODO below conficts in a bad way
  // if (PARENT_CLASS::SHARED_MEM_REQUEST_GRD_BYTES != 0)
  // {  // Allows us to turn on or off global or shared memory version of params
  //   params_p = (DYN_PARAMS_T*)theta_s;
  // }
  // else
  // {
  //   params_p = &(this->params_);
  // }
  params_p = &(this->params_);
  const int grd_shift = this->SHARED_MEM_REQUEST_GRD_BYTES / sizeof(float);  // grid size to shift by
  const int blk_shift = this->SHARED_MEM_REQUEST_BLK_BYTES * (threadIdx.x + blockDim.x * threadIdx.z) /
                        sizeof(float);                       // blk size to shift by
  const int sb_shift = sizeof(SharedBlock) / sizeof(float);  // how much to shift inside a block to lstm values
  if (this->SHARED_MEM_REQUEST_BLK_BYTES != 0)
  {
    float* sb_mem = &theta_s[grd_shift];  // does the grid shift
    sb = (SharedBlock*)(sb_mem + blk_shift);
  }
  computeParametricDelayDeriv(state, control, state_der, params_p);
  computeParametricAccelDeriv(state, control, state_der, dt, params_p);

  computeLSTMSteering(state, control, state_der, params_p, theta_s, grd_shift, blk_shift, sb_shift);

  updateState(state, next_state, state_der, dt);
  computeUncertaintyPropagation(state, control, state_der, next_state, dt, params_p, sb, theta_s);
  if (threadIdx.y == 0)
  {
    float roll = state[S_INDEX(ROLL)];
    float pitch = state[S_INDEX(PITCH)];
    RACER::computeStaticSettling<DYN_PARAMS_T::OutputIndex, TwoDTextureHelper<float>>(
        this->tex_helper_, next_state[S_INDEX(YAW)], next_state[S_INDEX(POS_X)], next_state[S_INDEX(POS_Y)], roll,
        pitch, output);
    next_state[S_INDEX(PITCH)] = pitch;
    next_state[S_INDEX(ROLL)] = roll;
  }
  __syncthreads();
  setOutputs(state_der, next_state, output);
}

TEMPLATE_TYPE
bool TEMPLATE_NAME::updateFromBuffer(const buffer_trajectory& buffer)
{
  std::vector<std::string> keys = { "STEER_ANGLE", "STEER_ANGLE_RATE", "CAN_STEER_CMD" };

  bool missing_key = this->checkIfKeysInBuffer(buffer, keys);
  if (missing_key)
  {
    return false;
  }
  Eigen::MatrixXf init_buffer = lstm_lstm_helper_->getEmptyBufferMatrix(buffer.at("STEER_ANGLE").rows());

  init_buffer.row(0) = buffer.at("STEER_ANGLE") * 0.2f;
  init_buffer.row(1) = buffer.at("STEER_ANGLE_RATE") * 0.2f;
  init_buffer.row(2) = buffer.at("CAN_STEER_CMD");

  lstm_lstm_helper_->initializeLSTM(init_buffer);
  return true;
}

TEMPLATE_TYPE
void TEMPLATE_NAME::initializeDynamics(const Eigen::Ref<const state_array>& state,
                                       const Eigen::Ref<const control_array>& control, Eigen::Ref<output_array> output,
                                       float t_0, float dt)
{
  this->lstm_lstm_helper_->resetLSTMHiddenCellCPU();
  PARENT_CLASS::initializeDynamics(state, control, output, t_0, dt);
}

TEMPLATE_TYPE
__device__ void TEMPLATE_NAME::updateState(float* state, float* next_state, float* state_der, const float dt)
{
  int i;
  int tdy = threadIdx.y;
  // Add the state derivative time dt to the current state.
  for (i = tdy; i < 6; i += blockDim.y)
  {
    next_state[i] = state[i] + state_der[i] * dt;
    if (i == S_INDEX(YAW))
    {
      next_state[i] = angle_utils::normalizeAngle(next_state[i]);
    }
    if (i == S_INDEX(STEER_ANGLE))
    {
      next_state[i] = fmaxf(fminf(next_state[i], this->params_.max_steer_angle), -this->params_.max_steer_angle);
      next_state[S_INDEX(STEER_ANGLE_RATE)] =
          state[S_INDEX(STEER_ANGLE_RATE)] + state_der[S_INDEX(STEER_ANGLE_RATE)] * dt;
    }
    if (i == S_INDEX(BRAKE_STATE))
    {
      next_state[i] = fminf(fmaxf(next_state[i], 0.0f), 1.0f);
    }
  }
}

TEMPLATE_TYPE
void TEMPLATE_NAME::updateState(const Eigen::Ref<const state_array> state, Eigen::Ref<state_array> next_state,
                                Eigen::Ref<state_array> state_der, const float dt)
{
  // Segmented it to ensure that roll and pitch don't get overwritten
  for (int i = 0; i < 6; i++)
  {
    next_state[i] = state[i] + state_der[i] * dt;
  }
  next_state(S_INDEX(YAW)) = angle_utils::normalizeAngle(next_state(S_INDEX(YAW)));
  next_state(S_INDEX(STEER_ANGLE)) =
      fmaxf(fminf(next_state(S_INDEX(STEER_ANGLE)), this->params_.max_steer_angle), -this->params_.max_steer_angle);
  next_state(S_INDEX(STEER_ANGLE_RATE)) = state(S_INDEX(STEER_ANGLE_RATE)) + state_der(S_INDEX(STEER_ANGLE_RATE)) * dt;
  next_state(S_INDEX(BRAKE_STATE)) =
      fminf(fmaxf(next_state(S_INDEX(BRAKE_STATE)), 0.0f), -this->control_rngs_[C_INDEX(THROTTLE_BRAKE)].x);
}

#undef TEMPLATE_NAME
#undef TEMPLATE_TYPE
