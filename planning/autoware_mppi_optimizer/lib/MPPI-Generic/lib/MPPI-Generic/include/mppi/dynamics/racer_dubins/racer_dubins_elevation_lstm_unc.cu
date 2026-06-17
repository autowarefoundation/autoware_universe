#include "racer_dubins_elevation_lstm_unc.cuh"

// TODO fix the clamping of the brake value
// TODO look into replacing roll/pitch values with static settled values

RacerDubinsElevationLSTMUncertainty::RacerDubinsElevationLSTMUncertainty(LSTMLSTMConfig& steer_config,
                                                                         LSTMLSTMConfig& mean_config,
                                                                         LSTMLSTMConfig& unc_config,
                                                                         cudaStream_t stream)
  : PARENT_CLASS(steer_config, stream)
{
  this->mean_helper_ = std::make_shared<LSTMLSTMHelper<>>(mean_config, stream);
  this->uncertainty_helper_ = std::make_shared<LSTMLSTMHelper<>>(unc_config, stream);

  this->SHARED_MEM_REQUEST_GRD_BYTES = lstm_lstm_helper_->getLSTMModel()->getGrdSharedSizeBytes() +
                                       mean_helper_->getLSTMModel()->getGrdSharedSizeBytes() +
                                       uncertainty_helper_->getLSTMModel()->getGrdSharedSizeBytes();
  this->SHARED_MEM_REQUEST_BLK_BYTES = sizeof(SharedBlock) +
                                       lstm_lstm_helper_->getLSTMModel()->getBlkSharedSizeBytes() +
                                       mean_helper_->getLSTMModel()->getBlkSharedSizeBytes() +
                                       uncertainty_helper_->getLSTMModel()->getBlkSharedSizeBytes();
  this->requires_buffer_ = true;
}

RacerDubinsElevationLSTMUncertainty::RacerDubinsElevationLSTMUncertainty(std::string path, cudaStream_t stream)
  : PARENT_CLASS(stream)
{
  if (!fileExists(path))
  {
    std::cerr << "Could not load neural net model at path: " << path.c_str();
    exit(-1);
  }

  lstm_lstm_helper_ = std::make_shared<LSTMLSTMHelper<>>(path, "steering/model/", stream);
  uncertainty_helper_ = std::make_shared<LSTMLSTMHelper<>>(path, "terra/uncertainty_network/", stream);
  mean_helper_ = std::make_shared<LSTMLSTMHelper<>>(path, "terra/mean_network/", stream);

  // empty path means that the parent class will not load things
  cnpy::npz_t param_dict = cnpy::npz_load(path);
  // loads the steering constants
  this->params_.max_steer_rate = param_dict.at("steering/parameters/max_rate_pos").data<double>()[0];
  this->params_.steering_constant = param_dict.at("steering/parameters/constant").data<double>()[0];
  this->params_.steer_accel_constant = param_dict.at("steering/parameters/accel_constant").data<double>()[0];
  this->params_.steer_accel_drag_constant = param_dict.at("steering/parameters/accel_drag_constant").data<double>()[0];

  this->params_.max_brake_rate_pos = param_dict.at("brake/parameters/max_rate_pos").data<double>()[0];
  this->params_.max_brake_rate_neg = param_dict.at("brake/parameters/max_rate_neg").data<double>()[0];
  for (int i = 0; i < 3; i++)
  {
    this->params_.c_t[i] = param_dict.at("terra/parameters/c_t").data<double>()[i];
    this->params_.c_b[i] = param_dict.at("terra/parameters/c_b").data<double>()[i];
    this->params_.c_v[i] = param_dict.at("terra/parameters/c_v").data<double>()[i];

    this->params_.pos_quad_brake_c[i] = param_dict.at("brake/parameters/constant").data<double>()[i];
    this->params_.neg_quad_brake_c[i] = param_dict.at("brake/parameters/negative_constant").data<double>()[i];
  }
  this->params_.clamp_ax = 1000.0f;
  this->params_.wheel_base = 3.0f;
  this->params_.c_0 = 0.0f;
  this->params_.max_steer_angle = 5.0f;
  this->params_.low_min_throttle = 0.0f;
  this->params_.gravity = param_dict.at("terra/parameters/gravity").data<double>()[0];
  this->params_.steer_angle_scale = param_dict.at("terra/parameters/steer_angle_scale").data<double>()[0];
  for (int i = 0; i < uncertainty_helper_->getLSTMModel()->getOutputDim(); i++)
  {
    this->params_.unc_scale[i] = param_dict.at("terra/parameters/uncertainty_scale").data<double>()[i];
  }

  this->SHARED_MEM_REQUEST_GRD_BYTES = lstm_lstm_helper_->getLSTMModel()->getGrdSharedSizeBytes() +
                                       mean_helper_->getLSTMModel()->getGrdSharedSizeBytes() +
                                       uncertainty_helper_->getLSTMModel()->getGrdSharedSizeBytes();
  this->SHARED_MEM_REQUEST_BLK_BYTES = sizeof(SharedBlock) +
                                       lstm_lstm_helper_->getLSTMModel()->getBlkSharedSizeBytes() +
                                       mean_helper_->getLSTMModel()->getBlkSharedSizeBytes() +
                                       uncertainty_helper_->getLSTMModel()->getBlkSharedSizeBytes();
  this->requires_buffer_ = true;
}

void RacerDubinsElevationLSTMUncertainty::GPUSetup()
{
  uncertainty_helper_->GPUSetup();
  this->uncertainty_d_ = uncertainty_helper_->getLSTMDevicePtr();
  mean_helper_->GPUSetup();
  this->mean_d_ = mean_helper_->getLSTMDevicePtr();
  PARENT_CLASS::GPUSetup();
}

void RacerDubinsElevationLSTMUncertainty::bindToStream(cudaStream_t stream)
{
  PARENT_CLASS::bindToStream(stream);
  uncertainty_helper_->getLSTMModel()->bindToStream(stream);
  mean_helper_->getLSTMModel()->bindToStream(stream);
}

void RacerDubinsElevationLSTMUncertainty::freeCudaMem()
{
  uncertainty_helper_->freeCudaMem();
  mean_helper_->freeCudaMem();
  PARENT_CLASS::freeCudaMem();
}

bool RacerDubinsElevationLSTMUncertainty::updateFromBuffer(const buffer_trajectory& buffer)
{
  bool parent_success = PARENT_CLASS::updateFromBuffer(buffer);

  std::vector<std::string> keys = { "STEER_ANGLE",   "STEER_ANGLE_RATE", "CAN_STEER_CMD", "ROLL",
                                    "PITCH",         "CAN_THROTTLE_CMD", "VEL_X",         "BRAKE_STATE",
                                    "CAN_BRAKE_CMD", "OMEGA_Z" };

  bool missing_key = this->checkIfKeysInBuffer(buffer, keys);
  if (missing_key || !parent_success)
  {
    return false;
  }

  Eigen::MatrixXf mean_init_buffer = mean_helper_->getEmptyBufferMatrix(buffer.at("VEL_X").rows());
  mean_init_buffer.row(0) = buffer.at("VEL_X");
  mean_init_buffer.row(1) = buffer.at("OMEGA_Z");
  mean_init_buffer.row(2) = buffer.at("BRAKE_STATE");
  mean_init_buffer.row(3) = buffer.at("STEER_ANGLE");
  mean_init_buffer.row(4) = buffer.at("STEER_ANGLE_RATE");
  mean_init_buffer.row(5) = buffer.at("CAN_THROTTLE_CMD");
  mean_init_buffer.row(6) = buffer.at("CAN_BRAKE_CMD");
  mean_init_buffer.row(7) = buffer.at("CAN_STEER_CMD");
  mean_init_buffer.row(8) = buffer.at("PITCH").unaryExpr(&sinf);
  // TODO option here if problem
  // mean_init_buffer.row(9) = buffer.at("ROLL").unaryExpr(&sinf);
  mean_helper_->initializeLSTM(mean_init_buffer);

  Eigen::MatrixXf unc_init_buffer = uncertainty_helper_->getEmptyBufferMatrix(buffer.at("VEL_X").rows());
  unc_init_buffer.row(0) = buffer.at("VEL_X");
  unc_init_buffer.row(1) = buffer.at("OMEGA_Z");
  unc_init_buffer.row(2) = buffer.at("BRAKE_STATE");
  unc_init_buffer.row(3) = buffer.at("STEER_ANGLE");
  unc_init_buffer.row(4) = buffer.at("STEER_ANGLE_RATE");
  unc_init_buffer.row(5) = buffer.at("CAN_THROTTLE_CMD");
  unc_init_buffer.row(6) = buffer.at("CAN_BRAKE_CMD");
  unc_init_buffer.row(7) = buffer.at("CAN_STEER_CMD");
  unc_init_buffer.row(8) = buffer.at("PITCH").unaryExpr(&sinf);
  unc_init_buffer.row(9) = buffer.at("ROLL").unaryExpr(&sinf);
  uncertainty_helper_->initializeLSTM(unc_init_buffer);
  return true;
}

RacerDubinsElevationLSTMUncertainty::state_array
RacerDubinsElevationLSTMUncertainty::stateFromMap(const std::map<std::string, float>& map)
{
  std::vector<std::string> needed_keys = { "VEL_X",       "VEL_Z",  "POS_X", "POS_Y", "POS_Z",       "OMEGA_X",
                                           "OMEGA_Y",     "ROLL",   "PITCH", "YAW",   "STEER_ANGLE", "STEER_ANGLE_RATE",
                                           "BRAKE_STATE", "OMEGA_Z" };
  std::vector<std::string> uncertainty_keys = { "UNCERTAINTY_POS_X",       "UNCERTAINTY_POS_Y",
                                                "UNCERTAINTY_YAW",         "UNCERTAINTY_VEL_X",
                                                "UNCERTAINTY_POS_X_Y",     "UNCERTAINTY_POS_X_YAW",
                                                "UNCERTAINTY_POS_X_VEL_X", "UNCERTAINTY_POS_Y_YAW",
                                                "UNCERTAINTY_POS_Y_VEL_X", "UNCERTAINTY_YAW_VEL_X" };
  const bool use_uncertainty = false;
  if (use_uncertainty)
  {
    needed_keys.insert(needed_keys.end(), uncertainty_keys.begin(), uncertainty_keys.end());
  }

  bool missing_key = false;
  state_array s = state_array::Zero();
  for (const auto& key : needed_keys)
  {
    if (map.find(key) == map.end())
    {  // Print out all missing keys
      std::cout << "Could not find key " << key << " for elevation with simple suspension dynamics." << std::endl;
      missing_key = true;
    }
  }
  if (missing_key)
  {
    return state_array::Constant(NAN);
  }

  s(S_INDEX(POS_X)) = map.at("POS_X");
  s(S_INDEX(POS_Y)) = map.at("POS_Y");
  s(S_INDEX(CG_POS_Z)) = map.at("POS_Z");
  s(S_INDEX(VEL_X)) = map.at("VEL_X");
  float bl_v_I_z = map.at("VEL_Z") * cosf(map.at("PITCH")) - map.at("VEL_X") * sinf(map.at("PITCH"));
  s(S_INDEX(CG_VEL_I_Z)) = bl_v_I_z - map.at("OMEGA_Y") * this->params_.c_g.x;
  s(S_INDEX(STEER_ANGLE)) = map.at("STEER_ANGLE");
  s(S_INDEX(STEER_ANGLE_RATE)) = map.at("STEER_ANGLE_RATE");
  s(S_INDEX(ROLL)) = map.at("ROLL");
  s(S_INDEX(PITCH)) = map.at("PITCH");
  s(S_INDEX(YAW)) = map.at("YAW");
  s(S_INDEX(OMEGA_Z)) = map.at("OMEGA_Z");
  // Set z position to cg's z position in world frame
  float3 rotation = make_float3(s(S_INDEX(ROLL)), s(S_INDEX(PITCH)), s(S_INDEX(YAW)));
  float3 world_pose = make_float3(s(S_INDEX(POS_X)), s(S_INDEX(POS_Y)), s(S_INDEX(CG_POS_Z)));
  float3 cg_in_world_frame;
  mppi::math::bodyOffsetToWorldPoseEuler(this->params_.c_g, world_pose, rotation, cg_in_world_frame);
  s(S_INDEX(CG_POS_Z)) = cg_in_world_frame.z;
  s(S_INDEX(ROLL_RATE)) = map.at("OMEGA_X");
  s(S_INDEX(PITCH_RATE)) = map.at("OMEGA_Y");
  s(S_INDEX(BRAKE_STATE)) = map.at("BRAKE_STATE");

  if (use_uncertainty)
  {
    s(S_INDEX(UNCERTAINTY_POS_X)) = map.at("UNCERTAINTY_POS_X");
    s(S_INDEX(UNCERTAINTY_POS_Y)) = map.at("UNCERTAINTY_POS_Y");
    s(S_INDEX(UNCERTAINTY_YAW)) = map.at("UNCERTAINTY_YAW");
    s(S_INDEX(UNCERTAINTY_VEL_X)) = map.at("UNCERTAINTY_VEL_X");
    s(S_INDEX(UNCERTAINTY_POS_X_Y)) = map.at("UNCERTAINTY_POS_X_Y");
    s(S_INDEX(UNCERTAINTY_POS_X_YAW)) = map.at("UNCERTAINTY_POS_X_YAW");
    s(S_INDEX(UNCERTAINTY_POS_X_VEL_X)) = map.at("UNCERTAINTY_POS_X_VEL_X");
    s(S_INDEX(UNCERTAINTY_POS_Y_YAW)) = map.at("UNCERTAINTY_POS_Y_YAW");
    s(S_INDEX(UNCERTAINTY_POS_Y_VEL_X)) = map.at("UNCERTAINTY_POS_Y_VEL_X");
    s(S_INDEX(UNCERTAINTY_YAW_VEL_X)) = map.at("UNCERTAINTY_YAW_VEL_X");
  }
  float eps = 1e-6f;
  if (s(S_INDEX(UNCERTAINTY_POS_X)) < eps)
  {
    s(S_INDEX(UNCERTAINTY_POS_X)) = eps;
  }
  if (s(S_INDEX(UNCERTAINTY_POS_Y)) < eps)
  {
    s(S_INDEX(UNCERTAINTY_POS_Y)) = eps;
  }
  if (s(S_INDEX(UNCERTAINTY_YAW)) < eps)
  {
    s(S_INDEX(UNCERTAINTY_YAW)) = eps;
  }
  if (s(S_INDEX(UNCERTAINTY_VEL_X)) < eps)
  {
    s(S_INDEX(UNCERTAINTY_VEL_X)) = eps;
  }
  return s;
}

void RacerDubinsElevationLSTMUncertainty::initializeDynamics(const Eigen::Ref<const state_array>& state,
                                                             const Eigen::Ref<const control_array>& control,
                                                             Eigen::Ref<output_array> output, float t_0, float dt)
{
  uncertainty_helper_->resetLSTMHiddenCellCPU();
  mean_helper_->resetLSTMHiddenCellCPU();
  PARENT_CLASS::initializeDynamics(state, control, output, t_0, dt);
}

void RacerDubinsElevationLSTMUncertainty::step(Eigen::Ref<state_array> state, Eigen::Ref<state_array> next_state,
                                               Eigen::Ref<state_array> state_der,
                                               const Eigen::Ref<const control_array>& control,
                                               Eigen::Ref<output_array> output, const float t, const float dt)
{
  // this is the new brake model with quadratics
  bool enable_brake = control(C_INDEX(THROTTLE_BRAKE)) < 0.0f;
  const float brake_error = (enable_brake * -control(C_INDEX(THROTTLE_BRAKE)) - state(S_INDEX(BRAKE_STATE)));

  state_der(S_INDEX(BRAKE_STATE)) =
      fminf(fmaxf((brake_error > 0) * (brake_error * this->params_.pos_quad_brake_c[0] +
                                       brake_error * fabsf(brake_error) * this->params_.pos_quad_brake_c[1]) +
                      (brake_error < 0) * (brake_error * this->params_.neg_quad_brake_c[0] +
                                           brake_error * fabsf(brake_error) * this->params_.neg_quad_brake_c[1]),
                  -this->params_.max_brake_rate_neg),
            this->params_.max_brake_rate_pos);
  this->computeParametricAccelDeriv(state, control, state_der, dt);
  this->computeLSTMSteering(state, control, state_der);
  this->computeSimpleSuspensionStep(state, state_der, output);

  // only on in reverse, default back to parametric in reverse
  if (this->params_.gear_sign == 1)
  {
    // compute the mean LSTM
    Eigen::VectorXf input = mean_helper_->getLSTMModel()->getZeroInputVector();
    input(0) = state(S_INDEX(VEL_X));
    input(1) = state(S_INDEX(OMEGA_Z));
    input(2) = state(S_INDEX(BRAKE_STATE));
    input(3) = state(S_INDEX(STEER_ANGLE));
    input(4) = state(S_INDEX(STEER_ANGLE_RATE));
    input(5) = control(C_INDEX(THROTTLE_BRAKE)) >= 0.0f ? control(C_INDEX(THROTTLE_BRAKE)) : 0.0f;
    input(6) = control(C_INDEX(THROTTLE_BRAKE)) <= 0.0f ? -control(C_INDEX(THROTTLE_BRAKE)) : 0.0f;
    input(7) = control(C_INDEX(STEER_CMD));
    input(8) = sinf(state(S_INDEX(STATIC_PITCH)));
    input(9) = state_der(S_INDEX(VEL_X));
    input(10) = state_der(S_INDEX(YAW));
    Eigen::VectorXf mean_output = mean_helper_->getLSTMModel()->getZeroOutputVector();
    mean_helper_->forward(input, mean_output);
    state_der(S_INDEX(VEL_X)) += mean_output(0);
    state_der(S_INDEX(YAW)) += mean_output(1);
  }
  next_state[S_INDEX(OMEGA_Z)] = state_der[S_INDEX(YAW)];

  // Integrate using racer_dubins updateState
  updateState(state, next_state, state_der, dt);
  SharedBlock sb;
  computeUncertaintyPropagation(state.data(), control.data(), state_der.data(), next_state.data(), dt, &this->params_,
                                &sb, nullptr);
  float roll = state(S_INDEX(STATIC_ROLL));
  float pitch = state(S_INDEX(STATIC_PITCH));
  RACER::computeStaticSettling<typename DYN_PARAMS_T::OutputIndex, TwoDTextureHelper<float>>(
      this->tex_helper_, next_state(S_INDEX(YAW)), next_state(S_INDEX(POS_X)), next_state(S_INDEX(POS_Y)), roll, pitch,
      output.data());
  next_state[S_INDEX(STATIC_PITCH)] = pitch;
  next_state[S_INDEX(STATIC_ROLL)] = roll;

  this->setOutputs(state_der.data(), next_state.data(), output.data());
}

__device__ __host__ bool RacerDubinsElevationLSTMUncertainty::computeQ(const float* state, const float* control,
                                                                       const float* state_der, float* Q,
                                                                       RacerDubinsElevationUncertaintyParams* params_p,
                                                                       float* theta_s)
{
  // in reverse just use base level implementation
  if (params_p->gear_sign == -1)
  {
    PARENT_CLASS::computeQ(state, control, state_der, Q, params_p, theta_s);
  }
  // TODO make the roll and pitch the ones from static settling, not the ones calculated online
  float sin_yaw, cos_yaw;
  float* uncertainty_output;
  int step, pi;
  mp1::getParallel1DIndex<mp1::Parallel1Dir::THREAD_Y>(pi, step);
#ifdef __CUDA_ARCH__

  const float yaw_norm = angle_utils::normalizeAngle(state[S_INDEX(YAW)]);
  __sincosf(yaw_norm, &sin_yaw, &cos_yaw);

  const int grd_shift = this->SHARED_MEM_REQUEST_GRD_BYTES / sizeof(float);  // grid size to shift by
  const int blk_shift = this->SHARED_MEM_REQUEST_BLK_BYTES * (threadIdx.x + blockDim.x * threadIdx.z) /
                        sizeof(float);  // blk size to shift by
  const int sb_shift =
      sizeof(SharedBlock) / sizeof(float) + network_d_->getBlkSharedSizeBytes() / sizeof(float) +
      mean_d_->getBlkSharedSizeBytes() / sizeof(float);  // how much to shift inside a block to lstm values

  float* input_loc = uncertainty_d_->getInputLocation(theta_s, grd_shift, blk_shift, sb_shift);

  for (int i = pi; i < uncertainty_d_->getInputDim(); i += step)
  {
    switch (i)
    {
      case 0:  // vx
        input_loc[i] = state[S_INDEX(VEL_X)];
        break;
      case 1:  // omega
        input_loc[i] = state[S_INDEX(OMEGA_Z)];
        break;
      case 2:  // brake state
        input_loc[i] = state[S_INDEX(BRAKE_STATE)];
        break;
      case 3:  // shaft angle
        input_loc[i] = state[S_INDEX(STEER_ANGLE)];
        break;
      case 4:  // shaft velocity
        input_loc[i] = state[S_INDEX(STEER_ANGLE_RATE)];
        break;
      case 5:  // throttle_cmd
        input_loc[i] = control[C_INDEX(THROTTLE_BRAKE)] >= 0.0f ? control[C_INDEX(THROTTLE_BRAKE)] : 0.0f;
        break;
      case 6:  // brake_cmd
        input_loc[i] = control[C_INDEX(THROTTLE_BRAKE)] <= 0.0f ? -control[C_INDEX(THROTTLE_BRAKE)] : 0.0f;
        break;
      case 7:  // steering cmd
        input_loc[i] = control[C_INDEX(STEER_CMD)];
        break;
      case 8:  // sin roll
        input_loc[i] = __sinf(state[S_INDEX(STATIC_ROLL)]);
        break;
      case 9:  // sin pitch
        input_loc[i] = __sinf(state[S_INDEX(STATIC_PITCH)]);
        break;
      case 10:  // accel x
        input_loc[i] = state_der[S_INDEX(VEL_X)];
        break;
      case 11:  // yaw rate calculation
        input_loc[i] = state_der[S_INDEX(YAW)];
        break;
    }
  }
  __syncthreads();

  float* cur_hidden_cell = uncertainty_d_->getHiddenCellLocation(theta_s, grd_shift, blk_shift, sb_shift);
  uncertainty_output = uncertainty_d_->forward(
      nullptr, theta_s + (network_d_->getGrdSharedSizeBytes() + mean_d_->getGrdSharedSizeBytes()) / sizeof(float),
      cur_hidden_cell);
  int output_dim = uncertainty_d_->getOutputDim();
#else
  sincosf(state[S_INDEX(YAW)], &sin_yaw, &cos_yaw);

  Eigen::VectorXf input = uncertainty_helper_->getLSTMModel()->getZeroInputVector();
  input(0) = state[S_INDEX(VEL_X)];
  input(1) = state[S_INDEX(OMEGA_Z)];
  input(2) = state[S_INDEX(BRAKE_STATE)];
  input(3) = state[S_INDEX(STEER_ANGLE)];
  input(4) = state[S_INDEX(STEER_ANGLE_RATE)];
  input(5) = control[C_INDEX(THROTTLE_BRAKE)] >= 0.0f ? control[C_INDEX(THROTTLE_BRAKE)] : 0.0f;
  input(6) = control[C_INDEX(THROTTLE_BRAKE)] < 0.0f ? -control[C_INDEX(THROTTLE_BRAKE)] : 0.0f;
  input(7) = control[C_INDEX(STEER_CMD)];
  input(8) = sinf(state[S_INDEX(STATIC_ROLL)]);
  input(9) = sinf(state[S_INDEX(STATIC_PITCH)]);
  input(10) = state_der[S_INDEX(VEL_X)];  // these should be using the modified means
  input(11) = state_der[S_INDEX(YAW)];
  Eigen::VectorXf uncertainty_output_eig = uncertainty_helper_->getLSTMModel()->getZeroOutputVector();
  uncertainty_helper_->forward(input, uncertainty_output_eig);

  // TODO need to sigmoid and scale
  uncertainty_output = uncertainty_output_eig.data();

  int output_dim = uncertainty_helper_->getLSTMModel()->getOutputDim();
#endif

  for (int i = pi; i < output_dim; i += step)
  {
    uncertainty_output[i] = fabsf(mppi::nn::sigmoid(uncertainty_output[i]) * params_p->unc_scale[i]);
  }
#ifdef __CUDA_ARCH__
  __syncthreads();
#endif

  const float linear_brake_slope = 0.2f;
  const int index = (fabsf(state[S_INDEX(VEL_X)]) > linear_brake_slope && fabsf(state[S_INDEX(VEL_X)]) <= 3.0f) +
                    (fabsf(state[S_INDEX(VEL_X)]) > 3.0f) * 2;
  // TODO add in additional uncertainty based on output dim of network
  for (int i = pi; i < UNCERTAINTY_DIM * UNCERTAINTY_DIM; i += step)
  {
    switch (i)
    {
      // vel_x
      case mm::columnMajorIndex(U_INDEX(VEL_X), U_INDEX(VEL_X), UNCERTAINTY_DIM):
        Q[i] = uncertainty_output[0] +
               SQ(params_p->c_b[index] * (index == 0 ? state[S_INDEX(VEL_X)] : 1.0f)) * uncertainty_output[4];
        if (output_dim == 7)
        {
          Q[i] += uncertainty_output[5];  // additional Q_vx
        }
        break;
      case mm::columnMajorIndex(U_INDEX(VEL_X), U_INDEX(YAW), UNCERTAINTY_DIM):
        Q[i] = 0.0f;
        break;
      case mm::columnMajorIndex(U_INDEX(VEL_X), U_INDEX(POS_X), UNCERTAINTY_DIM):
        Q[i] = 0.0f;
        break;
      case mm::columnMajorIndex(U_INDEX(VEL_X), U_INDEX(POS_Y), UNCERTAINTY_DIM):
        Q[i] = 0.0f;
        break;
        // yaw
      case mm::columnMajorIndex(U_INDEX(YAW), U_INDEX(VEL_X), UNCERTAINTY_DIM):
        Q[i] = 0.0f;
        break;
      case mm::columnMajorIndex(U_INDEX(YAW), U_INDEX(YAW), UNCERTAINTY_DIM):
#ifdef __CUDA_ARCH__
        Q[i] =
            uncertainty_output[1] +
            SQ((state[S_INDEX(VEL_X)] / params_p->wheel_base) * 1.0f /
               (SQ(__cosf(state[S_INDEX(STEER_ANGLE)] / params_p->steer_angle_scale)) * params_p->steer_angle_scale)) *
                uncertainty_output[3];
#else
        Q[i] = uncertainty_output[1] +
               SQ((state[S_INDEX(VEL_X)] / params_p->wheel_base) * 1.0f /
                  (SQ(cosf(state[S_INDEX(STEER_ANGLE)] / params_p->steer_angle_scale)) * params_p->steer_angle_scale)) *
                   uncertainty_output[3];
#endif
        if (output_dim == 7)
        {
          Q[i] += uncertainty_output[6];  // additional Q_yaw
        }
        break;
      case mm::columnMajorIndex(U_INDEX(YAW), U_INDEX(POS_X), UNCERTAINTY_DIM):
        Q[i] = 0.0f;
        break;
      case mm::columnMajorIndex(U_INDEX(YAW), U_INDEX(POS_Y), UNCERTAINTY_DIM):
        Q[i] = 0.0f;
        break;
        // pos x
      case mm::columnMajorIndex(U_INDEX(POS_X), U_INDEX(VEL_X), UNCERTAINTY_DIM):
        Q[i] = 0.0f;
        break;
      case mm::columnMajorIndex(U_INDEX(POS_X), U_INDEX(YAW), UNCERTAINTY_DIM):
        Q[i] = 0.0f;
        break;
      case mm::columnMajorIndex(U_INDEX(POS_X), U_INDEX(POS_X), UNCERTAINTY_DIM):
        Q[i] = uncertainty_output[2] * sin_yaw * sin_yaw;
        break;
      case mm::columnMajorIndex(U_INDEX(POS_X), U_INDEX(POS_Y), UNCERTAINTY_DIM):
        Q[i] = -uncertainty_output[2] * sin_yaw * cos_yaw;
        break;
        // pos y
      case mm::columnMajorIndex(U_INDEX(POS_Y), U_INDEX(VEL_X), UNCERTAINTY_DIM):
        Q[i] = 0.0f;
        break;
      case mm::columnMajorIndex(U_INDEX(POS_Y), U_INDEX(YAW), UNCERTAINTY_DIM):
        Q[i] = 0.0f;
        break;
      case mm::columnMajorIndex(U_INDEX(POS_Y), U_INDEX(POS_Y), UNCERTAINTY_DIM):
        Q[i] = uncertainty_output[2] * cos_yaw * cos_yaw;
        break;
      case mm::columnMajorIndex(U_INDEX(POS_Y), U_INDEX(POS_X), UNCERTAINTY_DIM):
        Q[i] = -uncertainty_output[2] * sin_yaw * cos_yaw;
        break;
    }
  }
  return true;
}

__device__ void RacerDubinsElevationLSTMUncertainty::step(float* state, float* next_state, float* state_der,
                                                          float* control, float* output, float* theta_s, const float t,
                                                          const float dt)
{
  DYN_PARAMS_T* params_p = &(this->params_);
  const int grd_shift = this->SHARED_MEM_REQUEST_GRD_BYTES / sizeof(float);  // grid size to shift by
  const int blk_shift = this->SHARED_MEM_REQUEST_BLK_BYTES * (threadIdx.x + blockDim.x * threadIdx.z) /
                        sizeof(float);  // blk size to shift by
  float* sb_mem = &theta_s[grd_shift];  // does the grid shift
  SharedBlock* sb = (SharedBlock*)(sb_mem + blk_shift);
  const int sb_shift =
      sizeof(SharedBlock) / sizeof(float) +
      network_d_->getBlkSharedSizeBytes() / sizeof(float);  // how much to shift inside a block to lstm values

  // this is the new brake model with quadratics
  bool enable_brake = control[C_INDEX(THROTTLE_BRAKE)] < 0.0f;
  const float brake_error = (enable_brake * -control[C_INDEX(THROTTLE_BRAKE)] - state[S_INDEX(BRAKE_STATE)]);

  state_der[S_INDEX(BRAKE_STATE)] =
      fminf(fmaxf((brake_error > 0) * (brake_error * params_p->pos_quad_brake_c[0] +
                                       brake_error * fabsf(brake_error) * params_p->pos_quad_brake_c[1]) +
                      (brake_error < 0) * (brake_error * params_p->neg_quad_brake_c[0] +
                                           brake_error * fabsf(brake_error) * params_p->neg_quad_brake_c[1]),
                  -params_p->max_brake_rate_neg),
            params_p->max_brake_rate_pos);
  computeParametricAccelDeriv(state, control, state_der, dt, params_p);
  computeLSTMSteering(state, control, state_der, params_p, theta_s, grd_shift, blk_shift,
                      sizeof(SharedBlock) / sizeof(float));
  computeSimpleSuspensionStep(state, state_der, output, params_p, theta_s);

  // TODO use the settling pitch and roll not the state based one
  // only on in reverse, default back to parametric in reverse
  if (params_p->gear_sign == 1)
  {
    // computes the mean compensation LSTM
    float* input_loc = mean_d_->getInputLocation(theta_s, grd_shift, blk_shift, sb_shift);
    int pi, step;
    mppi::p1::getParallel1DIndex<mppi::p1::Parallel1Dir::THREAD_Y>(pi, step);
    for (int i = pi; i < mean_d_->getInputDim(); i += step)
    {
      switch (i)
      {
        case 0:  // vx
          input_loc[i] = state[S_INDEX(VEL_X)];
          break;
        case 1:  // omega
          input_loc[i] = state[S_INDEX(OMEGA_Z)];
          break;
        case 2:  // brake state
          input_loc[i] = state[S_INDEX(BRAKE_STATE)];
          break;
        case 3:  // shaft angle
          input_loc[i] = state[S_INDEX(STEER_ANGLE)];
          break;
        case 4:  // shaft velocity
          input_loc[i] = state[S_INDEX(STEER_ANGLE_RATE)];
          break;
        case 5:  // throttle_cmd
          input_loc[i] = control[C_INDEX(THROTTLE_BRAKE)] >= 0.0f ? control[C_INDEX(THROTTLE_BRAKE)] : 0.0f;
          break;
        case 6:  // brake_cmd
          input_loc[i] = control[C_INDEX(THROTTLE_BRAKE)] <= 0.0f ? -control[C_INDEX(THROTTLE_BRAKE)] : 0.0f;
          break;
        case 7:  // steering cmd
          input_loc[i] = control[C_INDEX(STEER_CMD)];
          break;
        case 8:  // sin pitch
#ifdef __CUDA_ARCH__
          input_loc[i] = __sinf(state[S_INDEX(STATIC_PITCH)]);
#else
          input_loc[i] = sinf(state[S_INDEX(STATIC_PITCH)]);
#endif
          break;
        case 9:  // accel x
          input_loc[i] = state_der[S_INDEX(VEL_X)];
          break;
        case 10:  // yaw rate calculation
          input_loc[i] = state_der[S_INDEX(YAW)];
          break;
      }
    }
    __syncthreads();

    float* cur_hidden_cell = mean_d_->getHiddenCellLocation(theta_s, grd_shift, blk_shift, sb_shift);
    float* mean_output =
        mean_d_->forward(nullptr, theta_s + network_d_->getGrdSharedSizeBytes() / sizeof(float), cur_hidden_cell);

    if (threadIdx.y == 0)
    {
      state_der[S_INDEX(VEL_X)] += mean_output[0];
      state_der[S_INDEX(YAW)] += mean_output[1];
      next_state[S_INDEX(OMEGA_Z)] = state_der[S_INDEX(YAW)];
    }
    __syncthreads();
  }
  next_state[S_INDEX(OMEGA_Z)] = state_der[S_INDEX(YAW)];

  updateState(state, next_state, state_der, dt);
  computeUncertaintyPropagation(state, control, state_der, next_state, dt, params_p, sb, theta_s);

  float roll = state[S_INDEX(STATIC_ROLL)];
  float pitch = state[S_INDEX(STATIC_PITCH)];
  RACER::computeStaticSettling<DYN_PARAMS_T::OutputIndex, TwoDTextureHelper<float>>(
      this->tex_helper_, next_state[S_INDEX(YAW)], next_state[S_INDEX(POS_X)], next_state[S_INDEX(POS_Y)], roll, pitch,
      output);
  next_state[S_INDEX(STATIC_PITCH)] = pitch;
  next_state[S_INDEX(STATIC_ROLL)] = roll;
  //__syncthreads();
  this->setOutputs(state_der, next_state, output);
}

__device__ void RacerDubinsElevationLSTMUncertainty::initializeDynamics(float* state, float* control, float* output,
                                                                        float* theta_s, float t_0, float dt)
{
  PARENT_CLASS::initializeDynamics(state, control, output, theta_s, t_0, dt);
  int blk_shift = sizeof(SharedBlock) / sizeof(float) + network_d_->getBlkSharedSizeBytes() / sizeof(float);
  mean_d_->initialize(theta_s, this->SHARED_MEM_REQUEST_BLK_BYTES, this->SHARED_MEM_REQUEST_GRD_BYTES, blk_shift,
                      network_d_->getGrdSharedSizeBytes());
  blk_shift = sizeof(SharedBlock) / sizeof(float) + network_d_->getBlkSharedSizeBytes() / sizeof(float) +
              mean_d_->getBlkSharedSizeBytes() / sizeof(float);
  uncertainty_d_->initialize(theta_s, this->SHARED_MEM_REQUEST_BLK_BYTES, this->SHARED_MEM_REQUEST_GRD_BYTES, blk_shift,
                             network_d_->getGrdSharedSizeBytes() + mean_d_->getGrdSharedSizeBytes());
}
