#include <mppi/dynamics/racer_dubins/racer_dubins_elevation.cuh>
#include <mppi/utils/math_utils.h>

namespace mm = mppi::matrix_multiplication;
namespace mp1 = mppi::p1;

template <class CLASS_T, class PARAMS_T>
void RacerDubinsElevationImpl<CLASS_T, PARAMS_T>::GPUSetup()
{
  PARENT_CLASS::GPUSetup();
  tex_helper_->GPUSetup();
  // makes sure that the device ptr sees the correct texture object
  HANDLE_ERROR(cudaMemcpyAsync(&(this->model_d_->tex_helper_), &(tex_helper_->ptr_d_),
                               sizeof(TwoDTextureHelper<float>*), cudaMemcpyHostToDevice, this->stream_));
}

template <class CLASS_T, class PARAMS_T>
void RacerDubinsElevationImpl<CLASS_T, PARAMS_T>::freeCudaMem()
{
  tex_helper_->freeCudaMem();
  PARENT_CLASS::freeCudaMem();
}

template <class CLASS_T, class PARAMS_T>
void RacerDubinsElevationImpl<CLASS_T, PARAMS_T>::paramsToDevice()
{
  // does all the internal texture updates
  tex_helper_->copyToDevice();
  PARENT_CLASS::paramsToDevice();
}

template <class CLASS_T, class PARAMS_T>
void RacerDubinsElevationImpl<CLASS_T, PARAMS_T>::computeParametricAccelDeriv(
    const Eigen::Ref<const state_array>& state, const Eigen::Ref<const control_array>& control,
    Eigen::Ref<state_array> state_der, const float dt)
{
  float linear_brake_slope = 0.2f;
  bool enable_brake = control(C_INDEX(THROTTLE_BRAKE)) < 0.0f;
  int index = (abs(state(S_INDEX(VEL_X))) > linear_brake_slope && abs(state(S_INDEX(VEL_X))) <= 3.0f) +
              (abs(state(S_INDEX(VEL_X))) > 3.0f) * 2;

  const float brake_state = fmin(fmax(state(S_INDEX(BRAKE_STATE)), 0.0f), 0.25f);

  // applying position throttle
  float throttle = this->params_.c_t[index] * control(C_INDEX(THROTTLE_BRAKE));
  float brake = this->params_.c_b[index] * brake_state * (state(S_INDEX(VEL_X)) >= 0.0f ? -1.0f : 1.0f);
  if (abs(state(S_INDEX(VEL_X))) <= linear_brake_slope)
  {
    throttle = this->params_.c_t[index] * max(control(C_INDEX(THROTTLE_BRAKE)) - this->params_.low_min_throttle, 0.0f);
    brake = this->params_.c_b[index] * brake_state * -state(S_INDEX(VEL_X));
  }

  state_der(S_INDEX(VEL_X)) = (!enable_brake) * throttle * this->params_.gear_sign + brake -
                              this->params_.c_v[index] * state(S_INDEX(VEL_X)) + this->params_.c_0;
  state_der(S_INDEX(VEL_X)) = min(max(state_der(S_INDEX(VEL_X)), -this->params_.clamp_ax), this->params_.clamp_ax);
  if (fabsf(state[S_INDEX(PITCH)]) < static_cast<float>(M_PI_2))
  {
    state_der[S_INDEX(VEL_X)] -= this->params_.gravity * sinf(state[S_INDEX(PITCH)]);
  }
  state_der(S_INDEX(YAW)) = (state(S_INDEX(VEL_X)) / this->params_.wheel_base) *
                            tanf(state(S_INDEX(STEER_ANGLE)) / this->params_.steer_angle_scale);
  float yaw, sin_yaw, cos_yaw;
  yaw = state[S_INDEX(YAW)];
  sincosf(yaw, &sin_yaw, &cos_yaw);
  state_der(S_INDEX(POS_X)) = state(S_INDEX(VEL_X)) * cos_yaw;
  state_der(S_INDEX(POS_Y)) = state(S_INDEX(VEL_X)) * sin_yaw;
}

template <class CLASS_T, class PARAMS_T>
__host__ __device__ void RacerDubinsElevationImpl<CLASS_T, PARAMS_T>::setOutputs(const float* state_der,
                                                                                 const float* next_state, float* output)
{
  // Setup output
  // output[O_INDEX(BASELINK_VEL_B_X)] = next_state[S_INDEX(VEL_X)];
  // output[O_INDEX(BASELINK_VEL_B_Y)] = 0.0f;
  // output[O_INDEX(BASELINK_VEL_B_Z)] = 0.0f;
  // output[O_INDEX(BASELINK_POS_I_X)] = next_state[S_INDEX(POS_X)];
  // output[O_INDEX(BASELINK_POS_I_Y)] = next_state[S_INDEX(POS_Y)];
  // output[O_INDEX(PITCH)] = next_state[S_INDEX(PITCH)];
  // output[O_INDEX(ROLL)] = next_state[S_INDEX(ROLL)];
  // output[O_INDEX(YAW)] = next_state[S_INDEX(YAW)];
  // output[O_INDEX(STEER_ANGLE)] = next_state[S_INDEX(STEER_ANGLE)];
  // output[O_INDEX(STEER_ANGLE_RATE)] = next_state[S_INDEX(STEER_ANGLE_RATE)];
  // output[O_INDEX(WHEEL_FORCE_B_FL)] = 10000.0f;
  // output[O_INDEX(WHEEL_FORCE_B_FR)] = 10000.0f;
  // output[O_INDEX(WHEEL_FORCE_B_RL)] = 10000.0f;
  // output[O_INDEX(WHEEL_FORCE_B_RR)] = 10000.0f;
  // output[O_INDEX(ACCEL_X)] = state_der[S_INDEX(VEL_X)];
  // output[O_INDEX(ACCEL_Y)] = 0.0f;
  // output[O_INDEX(OMEGA_Z)] = state_der[S_INDEX(YAW)];
  // output[O_INDEX(UNCERTAINTY_VEL_X)] = next_state[S_INDEX(UNCERTAINTY_VEL_X)];
  // output[O_INDEX(UNCERTAINTY_YAW_VEL_X)] = next_state[S_INDEX(UNCERTAINTY_YAW_VEL_X)];
  // output[O_INDEX(UNCERTAINTY_POS_X_VEL_X)] = next_state[S_INDEX(UNCERTAINTY_POS_X_VEL_X)];
  // output[O_INDEX(UNCERTAINTY_POS_Y_VEL_X)] = next_state[S_INDEX(UNCERTAINTY_POS_Y_VEL_X)];
  // output[O_INDEX(UNCERTAINTY_YAW)] = next_state[S_INDEX(UNCERTAINTY_YAW)];
  // output[O_INDEX(UNCERTAINTY_POS_X_YAW)] = next_state[S_INDEX(UNCERTAINTY_POS_X_YAW)];
  // output[O_INDEX(UNCERTAINTY_POS_Y_YAW)] = next_state[S_INDEX(UNCERTAINTY_POS_Y_YAW)];
  // output[O_INDEX(UNCERTAINTY_POS_X)] = next_state[S_INDEX(UNCERTAINTY_POS_X)];
  // output[O_INDEX(UNCERTAINTY_POS_X_Y)] = next_state[S_INDEX(UNCERTAINTY_POS_X_Y)];
  // output[O_INDEX(UNCERTAINTY_POS_Y)] = next_state[S_INDEX(UNCERTAINTY_POS_Y)];

  int step, pi;
  mp1::getParallel1DIndex<mp1::Parallel1Dir::THREAD_Y>(pi, step);
  for (int i = pi; i < this->OUTPUT_DIM; i += step)
  {
    switch (i)
    {
      case O_INDEX(BASELINK_VEL_B_X):
        output[i] = next_state[S_INDEX(VEL_X)];
        break;
      case O_INDEX(BASELINK_VEL_B_Y):
        output[i] = 0.0f;
        break;
      // case O_INDEX(BASELINK_VEL_B_Z):
      //   output[i] = 0.0f;
      //   break;
      case O_INDEX(BASELINK_POS_I_X):
        output[i] = next_state[S_INDEX(POS_X)];
        break;
      case O_INDEX(BASELINK_POS_I_Y):
        output[i] = next_state[S_INDEX(POS_Y)];
        break;
      case O_INDEX(PITCH):
        output[i] = next_state[S_INDEX(PITCH)];
        break;
      case O_INDEX(ROLL):
        output[i] = next_state[S_INDEX(ROLL)];
        break;
      case O_INDEX(YAW):
        output[i] = next_state[S_INDEX(YAW)];
        break;
      case O_INDEX(STEER_ANGLE):
        output[i] = next_state[S_INDEX(STEER_ANGLE)];
        break;
      case O_INDEX(STEER_ANGLE_RATE):
        output[i] = next_state[S_INDEX(STEER_ANGLE_RATE)];
        break;
      case O_INDEX(WHEEL_FORCE_UP_MAX):
        output[i] = NAN;
        break;
      case O_INDEX(WHEEL_FORCE_FWD_MAX):
        output[i] = NAN;
        break;
      case O_INDEX(WHEEL_FORCE_SIDE_MAX):
        output[i] = NAN;
        break;
      // case O_INDEX(WHEEL_FORCE_UP_FL):
      //   output[i] = NAN;
      //   break;
      // case O_INDEX(WHEEL_FORCE_UP_FR):
      //   output[i] = NAN;
      //   break;
      // case O_INDEX(WHEEL_FORCE_UP_RL):
      //   output[i] = NAN;
      //   break;
      // case O_INDEX(WHEEL_FORCE_UP_RR):
      //   output[i] = NAN;
      //   break;
      // case O_INDEX(WHEEL_FORCE_FWD_FL):
      //   output[i] = NAN;
      //   break;
      // case O_INDEX(WHEEL_FORCE_FWD_FR):
      //   output[i] = NAN;
      //   break;
      // case O_INDEX(WHEEL_FORCE_FWD_RL):
      //   output[i] = NAN;
      //   break;
      // case O_INDEX(WHEEL_FORCE_FWD_RR):
      //   output[i] = NAN;
      //   break;
      // case O_INDEX(WHEEL_FORCE_SIDE_FL):
      //   output[i] = NAN;
      //   break;
      // case O_INDEX(WHEEL_FORCE_SIDE_FR):
      //   output[i] = NAN;
      //   break;
      // case O_INDEX(WHEEL_FORCE_SIDE_RL):
      //   output[i] = NAN;
      //   break;
      // case O_INDEX(WHEEL_FORCE_SIDE_RR):
      //   output[i] = NAN;
      //   break;
      case O_INDEX(ACCEL_X):
        output[i] = state_der[S_INDEX(VEL_X)];
        break;
      case O_INDEX(ACCEL_Y):
        output[i] = 0.0f;
        break;
      case O_INDEX(OMEGA_Z):
        output[i] = state_der[S_INDEX(YAW)];
        break;
      case O_INDEX(UNCERTAINTY_VEL_X):
        output[i] = next_state[S_INDEX(UNCERTAINTY_VEL_X)];
        break;
      case O_INDEX(UNCERTAINTY_YAW_VEL_X):
        output[i] = next_state[S_INDEX(UNCERTAINTY_YAW_VEL_X)];
        break;
      case O_INDEX(UNCERTAINTY_POS_X_VEL_X):
        output[i] = next_state[S_INDEX(UNCERTAINTY_POS_X_VEL_X)];
        break;
      case O_INDEX(UNCERTAINTY_POS_Y_VEL_X):
        output[i] = next_state[S_INDEX(UNCERTAINTY_POS_Y_VEL_X)];
        break;
      case O_INDEX(UNCERTAINTY_YAW):
        output[i] = next_state[S_INDEX(UNCERTAINTY_YAW)];
        break;
      case O_INDEX(UNCERTAINTY_POS_X_YAW):
        output[i] = next_state[S_INDEX(UNCERTAINTY_POS_X_YAW)];
        break;
      case O_INDEX(UNCERTAINTY_POS_Y_YAW):
        output[i] = next_state[S_INDEX(UNCERTAINTY_POS_Y_YAW)];
        break;
      case O_INDEX(UNCERTAINTY_POS_X):
        output[i] = next_state[S_INDEX(UNCERTAINTY_POS_X)];
        break;
      case O_INDEX(UNCERTAINTY_POS_X_Y):
        output[i] = next_state[S_INDEX(UNCERTAINTY_POS_X_Y)];
        break;
      case O_INDEX(UNCERTAINTY_POS_Y):
        output[i] = next_state[S_INDEX(UNCERTAINTY_POS_Y)];
        break;
      case O_INDEX(TOTAL_VELOCITY):
        output[i] = fabsf(next_state[S_INDEX(VEL_X)]);
        break;
    }
  }
}

template <class CLASS_T, class PARAMS_T>
void RacerDubinsElevationImpl<CLASS_T, PARAMS_T>::step(Eigen::Ref<state_array> state,
                                                       Eigen::Ref<state_array> next_state,
                                                       Eigen::Ref<state_array> state_der,
                                                       const Eigen::Ref<const control_array>& control,
                                                       Eigen::Ref<output_array> output, const float t, const float dt)
{
  this->computeParametricDelayDeriv(state, control, state_der);
  this->computeParametricSteerDeriv(state, control, state_der);
  computeParametricAccelDeriv(state, control, state_der, dt);
  SharedBlock sb;

  // Integrate using racer_dubins updateState
  this->PARENT_CLASS::updateState(state, next_state, state_der, dt);

  computeUncertaintyPropagation(state.data(), control.data(), state_der.data(), next_state.data(), dt, &this->params_,
                                &sb, nullptr);
  float roll = state(S_INDEX(ROLL));
  float pitch = state(S_INDEX(PITCH));
  RACER::computeStaticSettling<typename DYN_PARAMS_T::OutputIndex, TwoDTextureHelper<float>>(
      this->tex_helper_, next_state(S_INDEX(YAW)), next_state(S_INDEX(POS_X)), next_state(S_INDEX(POS_Y)), roll, pitch,
      output.data());
  next_state[S_INDEX(PITCH)] = pitch;
  next_state[S_INDEX(ROLL)] = roll;

  this->setOutputs(state_der.data(), next_state.data(), output.data());
}

template <class CLASS_T, class PARAMS_T>
bool RacerDubinsElevationImpl<CLASS_T, PARAMS_T>::computeGrad(const Eigen::Ref<const state_array>& state,
                                                              const Eigen::Ref<const control_array>& control,
                                                              Eigen::Ref<dfdx> A, Eigen::Ref<dfdu> B)
{
  A = dfdx::Zero();
  B = dfdu::Zero();

  float eps = 0.01f;
  bool enable_brake = control(C_INDEX(THROTTLE_BRAKE)) < 0.0f;

  // vx
  float linear_brake_slope = 0.2f;
  int index = (abs(state(S_INDEX(VEL_X))) > linear_brake_slope && abs(state(S_INDEX(VEL_X))) <= 3.0f) +
              (abs(state(S_INDEX(VEL_X))) > 3.0f) * 2;

  A(0, 0) = -this->params_.c_v[index];
  if (abs(state(S_INDEX(VEL_X))) < linear_brake_slope)
  {
    A(0, 5) = this->params_.c_b[index] * -state(S_INDEX(VEL_X));
  }
  else
  {
    A(0, 5) = this->params_.c_b[index] * (state(S_INDEX(VEL_X)) >= 0.0f ? -1.0f : 1.0f);
  }
  // TODO zero out if we are above the threshold to match??

  // yaw
  A(1, 0) = (1.0f / this->params_.wheel_base) * tanf(state(S_INDEX(STEER_ANGLE)) / this->params_.steer_angle_scale);
  A(1, 4) = (state(S_INDEX(VEL_X)) / this->params_.wheel_base) *
            (1.0 / SQ(cosf(state(S_INDEX(STEER_ANGLE)) / this->params_.steer_angle_scale))) /
            this->params_.steer_angle_scale;
  // pos x
  A(2, 0) = cosf(state(S_INDEX(YAW)));
  A(2, 1) = -sinf(state(S_INDEX(YAW))) * state(S_INDEX(VEL_X));
  // pos y
  A(3, 0) = sinf(state(S_INDEX(YAW)));
  A(3, 1) = cosf(state(S_INDEX(YAW))) * state(S_INDEX(VEL_X));
  // steer angle
  float steer_dot =
      (control(C_INDEX(STEER_CMD)) * this->params_.steer_command_angle_scale - state(S_INDEX(STEER_ANGLE))) *
      this->params_.steering_constant;
  if (steer_dot - eps < -this->params_.max_steer_rate || steer_dot + eps > this->params_.max_steer_rate)
  {
    A(4, 4) = 0.0f;
  }
  else
  {
    A(4, 4) = -this->params_.steering_constant;
  }
  A(4, 4) = max(min(A(4, 4), this->params_.max_steer_rate), -this->params_.max_steer_rate);
  // gravity
  A(0, 7) = -this->params_.gravity * cosf(state(S_INDEX(PITCH)));

  // brake delay
  float brake_dot = (enable_brake * -control(C_INDEX(THROTTLE_BRAKE)) - state(S_INDEX(BRAKE_STATE))) *
                    this->params_.brake_delay_constant;
  if (brake_dot - eps < -this->params_.max_brake_rate_neg || brake_dot + eps > this->params_.max_brake_rate_pos)
  {
    A(5, 5) = 0.0f;
  }
  else
  {
    A(5, 5) = -this->params_.brake_delay_constant;
  }

  // steering command
  B(4, 1) = this->params_.steer_command_angle_scale * this->params_.steering_constant;
  // throttle command
  B(0, 0) = this->params_.c_t[index] * this->params_.gear_sign * (!enable_brake);
  // brake command
  if ((state(S_INDEX(BRAKE_STATE)) < -this->control_rngs_[C_INDEX(THROTTLE_BRAKE)].x && brake_dot < 0.0f) ||
      (state(S_INDEX(BRAKE_STATE)) > 0.0f && brake_dot > 0.0f))
  {
    B(5, 0) = -this->params_.brake_delay_constant * enable_brake;
  }
  return true;
}

template <class CLASS_T, class PARAMS_T>
__host__ __device__ bool RacerDubinsElevationImpl<CLASS_T, PARAMS_T>::computeUncertaintyJacobian(const float* state,
                                                                                                 const float* control,
                                                                                                 float* A,
                                                                                                 DYN_PARAMS_T* params_p)
{
  float sin_yaw, cos_yaw, tan_steer_angle, cos_2_delta;
#ifdef __CUDA_ARCH__
  float yaw_norm = angle_utils::normalizeAngle(state[S_INDEX(YAW)]);
  float delta = state[S_INDEX(STEER_ANGLE)] / params_p->steer_angle_scale;
  __sincosf(yaw_norm, &sin_yaw, &cos_yaw);
  tan_steer_angle = __tanf(delta);
  cos_2_delta = SQ(__cosf(delta));
#else
  sincosf(state[S_INDEX(YAW)], &sin_yaw, &cos_yaw);
  float delta = state[S_INDEX(STEER_ANGLE)] / params_p->steer_angle_scale;
  tan_steer_angle = tanf(delta);
  cos_2_delta = SQ(cosf(delta));
#endif
  // const float cos_2_delta = cos_yaw * cos_yaw;

  // vx
  float linear_brake_slope = 0.2f;
  int index = (fabsf(state[S_INDEX(VEL_X)]) > linear_brake_slope && fabsf(state[S_INDEX(VEL_X)]) <= 3.0f) +
              (fabsf(state[S_INDEX(VEL_X)]) > 3.0f) * 2;

  int step, pi;
  mp1::getParallel1DIndex<mp1::Parallel1Dir::THREAD_Y>(pi, step);
  const float brake_state = fmin(fmax(state[S_INDEX(BRAKE_STATE)], 0.0f), 0.25f);
  // A = df/dx + df/du * K
  for (int i = pi; i < UNCERTAINTY_DIM * UNCERTAINTY_DIM; i += step)
  {
    switch (i)
    {
      case mm::columnMajorIndex(U_INDEX(VEL_X), U_INDEX(VEL_X), UNCERTAINTY_DIM):
        A[i] = -params_p->c_v[index] - params_p->K_vel_x - (index == 0 ? 1.0f : 0.0f) * params_p->c_b[0] * brake_state;
        break;
      case mm::columnMajorIndex(U_INDEX(VEL_X), U_INDEX(YAW), UNCERTAINTY_DIM):
        A[i] = 0.0f;
        break;
      case mm::columnMajorIndex(U_INDEX(VEL_X), U_INDEX(POS_X), UNCERTAINTY_DIM):
        A[i] = -params_p->K_x * cos_yaw;
        break;
      case mm::columnMajorIndex(U_INDEX(VEL_X), U_INDEX(POS_Y), UNCERTAINTY_DIM):
        A[i] = -params_p->K_x * sin_yaw;
        break;

      // yaw
      case mm::columnMajorIndex(U_INDEX(YAW), U_INDEX(VEL_X), UNCERTAINTY_DIM):
        A[i] = tan_steer_angle / (params_p->wheel_base);
        break;
      case mm::columnMajorIndex(U_INDEX(YAW), U_INDEX(YAW), UNCERTAINTY_DIM):
        A[i] = -fabsf(state[S_INDEX(VEL_X)]) * params_p->K_yaw / (params_p->wheel_base * cos_2_delta);
        break;
      case mm::columnMajorIndex(U_INDEX(YAW), U_INDEX(POS_X), UNCERTAINTY_DIM):
        A[i] = state[S_INDEX(VEL_X)] * params_p->K_y * sin_yaw / (params_p->wheel_base * cos_2_delta);
        break;
      case mm::columnMajorIndex(U_INDEX(YAW), U_INDEX(POS_Y), UNCERTAINTY_DIM):
        A[i] = -state[S_INDEX(VEL_X)] * params_p->K_y * cos_yaw / (params_p->wheel_base * cos_2_delta);
        break;
      // pos x
      case mm::columnMajorIndex(U_INDEX(POS_X), U_INDEX(VEL_X), UNCERTAINTY_DIM):
        A[i] = cos_yaw;
        break;
      case mm::columnMajorIndex(U_INDEX(POS_X), U_INDEX(YAW), UNCERTAINTY_DIM):
        A[i] = -sin_yaw * state[S_INDEX(VEL_X)];
        break;
      case mm::columnMajorIndex(U_INDEX(POS_X), U_INDEX(POS_X), UNCERTAINTY_DIM):
        A[i] = 0.0f;
        break;
      case mm::columnMajorIndex(U_INDEX(POS_X), U_INDEX(POS_Y), UNCERTAINTY_DIM):
        A[i] = 0.0f;
        break;
      // pos y
      case mm::columnMajorIndex(U_INDEX(POS_Y), U_INDEX(VEL_X), UNCERTAINTY_DIM):
        A[i] = sin_yaw;
        break;
      case mm::columnMajorIndex(U_INDEX(POS_Y), U_INDEX(YAW), UNCERTAINTY_DIM):
        A[i] = cos_yaw * state[S_INDEX(VEL_X)];
        break;
      case mm::columnMajorIndex(U_INDEX(POS_Y), U_INDEX(POS_Y), UNCERTAINTY_DIM):
        A[i] = 0.0f;
        break;
      case mm::columnMajorIndex(U_INDEX(POS_Y), U_INDEX(POS_X), UNCERTAINTY_DIM):
        A[i] = 0.0f;
        break;
    }
  }
  return true;
}

template <class CLASS_T, class PARAMS_T>
__host__ __device__ bool RacerDubinsElevationImpl<CLASS_T, PARAMS_T>::computeQ(const float* state, const float* control,
                                                                               const float* state_der, float* Q,
                                                                               DYN_PARAMS_T* params_p, float* theta_s)
{
  const float abs_vx = fabsf(state[S_INDEX(VEL_X)]);
  const float abs_acc_x = fabsf(state_der[S_INDEX(VEL_X)]);
  const float delta = state[S_INDEX(STEER_ANGLE)] / params_p->steer_angle_scale;

  float sin_yaw, cos_yaw, tan_steer_angle, sin_roll;
#ifdef __CUDA_ARCH__
  const float yaw_norm = angle_utils::normalizeAngle(state[S_INDEX(YAW)]);
  __sincosf(yaw_norm, &sin_yaw, &cos_yaw);
  tan_steer_angle = __tanf(delta);
  sin_roll = __sinf(angle_utils::normalizeAngle(state[S_INDEX(ROLL)]));
#else
  sincosf(state[S_INDEX(YAW)], &sin_yaw, &cos_yaw);
  tan_steer_angle = tanf(delta);
#endif
  const float side_force = SQ(abs_vx) * tan_steer_angle / params_p->wheel_base + params_p->gravity * sin_roll;
  // const float Q_11 = params_p->Q_y_f * side_force * side_force * abs_vx;
  const float Q_11 = fabsf(params_p->Q_y_f * fabsf(side_force) * fmaxf(abs_vx - 2, 0.0f));

  const float linear_brake_slope = 0.2f;
  const int index = (fabsf(state[S_INDEX(VEL_X)]) > linear_brake_slope && fabsf(state[S_INDEX(VEL_X)]) <= 3.0f) +
                    (fabsf(state[S_INDEX(VEL_X)]) > 3.0f) * 2;

  int step, pi;
  mp1::getParallel1DIndex<mp1::Parallel1Dir::THREAD_Y>(pi, step);
  for (int i = pi; i < UNCERTAINTY_DIM * UNCERTAINTY_DIM; i += step)
  {
    switch (i)
    {
      // vel_x
      case mm::columnMajorIndex(U_INDEX(VEL_X), U_INDEX(VEL_X), UNCERTAINTY_DIM):
        Q[i] = params_p->Q_x_acc * abs_acc_x + params_p->Q_x_v[index] * abs_vx;
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
        Q[i] = abs_vx * (params_p->Q_omega_steering * fabsf(delta) + params_p->Q_omega_v);
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
        Q[i] = Q_11 * sin_yaw * sin_yaw;
        break;
      case mm::columnMajorIndex(U_INDEX(POS_X), U_INDEX(POS_Y), UNCERTAINTY_DIM):
        Q[i] = -Q_11 * sin_yaw * cos_yaw;
        break;
      // pos y
      case mm::columnMajorIndex(U_INDEX(POS_Y), U_INDEX(VEL_X), UNCERTAINTY_DIM):
        Q[i] = 0.0f;
        break;
      case mm::columnMajorIndex(U_INDEX(POS_Y), U_INDEX(YAW), UNCERTAINTY_DIM):
        Q[i] = 0.0f;
        break;
      case mm::columnMajorIndex(U_INDEX(POS_Y), U_INDEX(POS_Y), UNCERTAINTY_DIM):
        Q[i] = Q_11 * cos_yaw * cos_yaw;
        break;
      case mm::columnMajorIndex(U_INDEX(POS_Y), U_INDEX(POS_X), UNCERTAINTY_DIM):
        Q[i] = -Q_11 * sin_yaw * cos_yaw;
        break;
    }
  }
  return true;
}

template <class CLASS_T, class PARAMS_T>
__host__ __device__ void
RacerDubinsElevationImpl<CLASS_T, PARAMS_T>::uncertaintyStateToMatrix(const float* state, float* uncertainty_matrix)
{
  int step, pi;
  mp1::getParallel1DIndex<mp1::Parallel1Dir::THREAD_Y>(pi, step);
  for (int i = pi; i < UNCERTAINTY_DIM * UNCERTAINTY_DIM; i += step)
  {
    switch (i)
    {
      case mm::columnMajorIndex(U_INDEX(VEL_X), U_INDEX(VEL_X), UNCERTAINTY_DIM):
        uncertainty_matrix[i] = state[S_INDEX(UNCERTAINTY_VEL_X)];
        break;
      case mm::columnMajorIndex(U_INDEX(YAW), U_INDEX(VEL_X), UNCERTAINTY_DIM):
        uncertainty_matrix[i] = state[S_INDEX(UNCERTAINTY_YAW_VEL_X)];
        break;
      case mm::columnMajorIndex(U_INDEX(POS_X), U_INDEX(VEL_X), UNCERTAINTY_DIM):
        uncertainty_matrix[i] = state[S_INDEX(UNCERTAINTY_POS_X_VEL_X)];
        break;
      case mm::columnMajorIndex(U_INDEX(POS_Y), U_INDEX(VEL_X), UNCERTAINTY_DIM):
        uncertainty_matrix[i] = state[S_INDEX(UNCERTAINTY_POS_Y_VEL_X)];
        break;
      case mm::columnMajorIndex(U_INDEX(VEL_X), U_INDEX(YAW), UNCERTAINTY_DIM):
        uncertainty_matrix[i] = state[S_INDEX(UNCERTAINTY_YAW_VEL_X)];
        break;
      case mm::columnMajorIndex(U_INDEX(YAW), U_INDEX(YAW), UNCERTAINTY_DIM):
        uncertainty_matrix[i] = state[S_INDEX(UNCERTAINTY_YAW)];
        break;
      case mm::columnMajorIndex(U_INDEX(POS_X), U_INDEX(YAW), UNCERTAINTY_DIM):
        uncertainty_matrix[i] = state[S_INDEX(UNCERTAINTY_POS_X_YAW)];
        break;
      case mm::columnMajorIndex(U_INDEX(POS_Y), U_INDEX(YAW), UNCERTAINTY_DIM):
        uncertainty_matrix[i] = state[S_INDEX(UNCERTAINTY_POS_Y_YAW)];
        break;
      case mm::columnMajorIndex(U_INDEX(VEL_X), U_INDEX(POS_X), UNCERTAINTY_DIM):
        uncertainty_matrix[i] = state[S_INDEX(UNCERTAINTY_POS_X_VEL_X)];
        break;
      case mm::columnMajorIndex(U_INDEX(YAW), U_INDEX(POS_X), UNCERTAINTY_DIM):
        uncertainty_matrix[i] = state[S_INDEX(UNCERTAINTY_POS_X_YAW)];
        break;
      case mm::columnMajorIndex(U_INDEX(POS_X), U_INDEX(POS_X), UNCERTAINTY_DIM):
        uncertainty_matrix[i] = state[S_INDEX(UNCERTAINTY_POS_X)];
        break;
      case mm::columnMajorIndex(U_INDEX(POS_Y), U_INDEX(POS_X), UNCERTAINTY_DIM):
        uncertainty_matrix[i] = state[S_INDEX(UNCERTAINTY_POS_X_Y)];
        break;
      case mm::columnMajorIndex(U_INDEX(VEL_X), U_INDEX(POS_Y), UNCERTAINTY_DIM):
        uncertainty_matrix[i] = state[S_INDEX(UNCERTAINTY_POS_Y_VEL_X)];
        break;
      case mm::columnMajorIndex(U_INDEX(YAW), U_INDEX(POS_Y), UNCERTAINTY_DIM):
        uncertainty_matrix[i] = state[S_INDEX(UNCERTAINTY_POS_Y_YAW)];
        break;
      case mm::columnMajorIndex(U_INDEX(POS_X), U_INDEX(POS_Y), UNCERTAINTY_DIM):
        uncertainty_matrix[i] = state[S_INDEX(UNCERTAINTY_POS_X_Y)];
        break;
      case mm::columnMajorIndex(U_INDEX(POS_Y), U_INDEX(POS_Y), UNCERTAINTY_DIM):
        uncertainty_matrix[i] = state[S_INDEX(UNCERTAINTY_POS_Y)];
        break;
    }
  }
}

template <class CLASS_T, class PARAMS_T>
__host__ __device__ void
RacerDubinsElevationImpl<CLASS_T, PARAMS_T>::uncertaintyMatrixToState(const float* uncertainty_matrix, float* state)
{
  int step, pi;
  mp1::getParallel1DIndex<mp1::Parallel1Dir::THREAD_Y>(pi, step);
  for (int i = pi + S_INDEX(UNCERTAINTY_POS_X); i < this->STATE_DIM; i += step)
  {
    switch (i)
    {
      case S_INDEX(UNCERTAINTY_VEL_X):
        state[i] = uncertainty_matrix[mm::columnMajorIndex(U_INDEX(VEL_X), U_INDEX(VEL_X), UNCERTAINTY_DIM)];
        break;
      case S_INDEX(UNCERTAINTY_YAW_VEL_X):
        state[i] = uncertainty_matrix[mm::columnMajorIndex(U_INDEX(YAW), U_INDEX(VEL_X), UNCERTAINTY_DIM)];
        break;
      case S_INDEX(UNCERTAINTY_POS_X_VEL_X):
        state[i] = uncertainty_matrix[mm::columnMajorIndex(U_INDEX(POS_X), U_INDEX(VEL_X), UNCERTAINTY_DIM)];
        break;
      case S_INDEX(UNCERTAINTY_POS_Y_VEL_X):
        state[i] = uncertainty_matrix[mm::columnMajorIndex(U_INDEX(POS_Y), U_INDEX(VEL_X), UNCERTAINTY_DIM)];
        break;
      case S_INDEX(UNCERTAINTY_YAW):
        state[i] = uncertainty_matrix[mm::columnMajorIndex(U_INDEX(YAW), U_INDEX(YAW), UNCERTAINTY_DIM)];
        break;
      case S_INDEX(UNCERTAINTY_POS_X_YAW):
        state[i] = uncertainty_matrix[mm::columnMajorIndex(U_INDEX(POS_X), U_INDEX(YAW), UNCERTAINTY_DIM)];
        break;
      case S_INDEX(UNCERTAINTY_POS_Y_YAW):
        state[i] = uncertainty_matrix[mm::columnMajorIndex(U_INDEX(POS_Y), U_INDEX(YAW), UNCERTAINTY_DIM)];
        break;
      case S_INDEX(UNCERTAINTY_POS_X):
        state[i] = uncertainty_matrix[mm::columnMajorIndex(U_INDEX(POS_X), U_INDEX(POS_X), UNCERTAINTY_DIM)];
        break;
      case S_INDEX(UNCERTAINTY_POS_X_Y):
        state[i] = uncertainty_matrix[mm::columnMajorIndex(U_INDEX(POS_Y), U_INDEX(POS_X), UNCERTAINTY_DIM)];
        break;
      case S_INDEX(UNCERTAINTY_POS_Y):
        state[i] = uncertainty_matrix[mm::columnMajorIndex(U_INDEX(POS_Y), U_INDEX(POS_Y), UNCERTAINTY_DIM)];
        break;
      default:
        break;
    }
  }
}

template <class CLASS_T, class PARAMS_T>
__host__ __device__ void
RacerDubinsElevationImpl<CLASS_T, PARAMS_T>::uncertaintyMatrixToOutput(const float* uncertainty_matrix, float* output)
{
  int step, pi;
  mp1::getParallel1DIndex<mp1::Parallel1Dir::THREAD_Y>(pi, step);
  for (int i = pi + O_INDEX(UNCERTAINTY_POS_X); i < this->OUTPUT_DIM; i += step)
  {
    switch (i)
    {
      case O_INDEX(UNCERTAINTY_VEL_X):
        output[i] = uncertainty_matrix[mm::columnMajorIndex(U_INDEX(VEL_X), U_INDEX(VEL_X), UNCERTAINTY_DIM)];
        break;
      case O_INDEX(UNCERTAINTY_YAW_VEL_X):
        output[i] = uncertainty_matrix[mm::columnMajorIndex(U_INDEX(YAW), U_INDEX(VEL_X), UNCERTAINTY_DIM)];
        break;
      case O_INDEX(UNCERTAINTY_POS_X_VEL_X):
        output[i] = uncertainty_matrix[mm::columnMajorIndex(U_INDEX(POS_X), U_INDEX(VEL_X), UNCERTAINTY_DIM)];
        break;
      case O_INDEX(UNCERTAINTY_POS_Y_VEL_X):
        output[i] = uncertainty_matrix[mm::columnMajorIndex(U_INDEX(POS_Y), U_INDEX(VEL_X), UNCERTAINTY_DIM)];
        break;
      case O_INDEX(UNCERTAINTY_YAW):
        output[i] = uncertainty_matrix[mm::columnMajorIndex(U_INDEX(YAW), U_INDEX(YAW), UNCERTAINTY_DIM)];
        break;
      case O_INDEX(UNCERTAINTY_POS_X_YAW):
        output[i] = uncertainty_matrix[mm::columnMajorIndex(U_INDEX(POS_X), U_INDEX(YAW), UNCERTAINTY_DIM)];
        break;
      case O_INDEX(UNCERTAINTY_POS_Y_YAW):
        output[i] = uncertainty_matrix[mm::columnMajorIndex(U_INDEX(POS_Y), U_INDEX(YAW), UNCERTAINTY_DIM)];
        break;
      case O_INDEX(UNCERTAINTY_POS_X):
        output[i] = uncertainty_matrix[mm::columnMajorIndex(U_INDEX(POS_X), U_INDEX(POS_X), UNCERTAINTY_DIM)];
        break;
      case O_INDEX(UNCERTAINTY_POS_X_Y):
        output[i] = uncertainty_matrix[mm::columnMajorIndex(U_INDEX(POS_Y), U_INDEX(POS_X), UNCERTAINTY_DIM)];
        break;
      case O_INDEX(UNCERTAINTY_POS_Y):
        output[i] = uncertainty_matrix[mm::columnMajorIndex(U_INDEX(POS_Y), U_INDEX(POS_Y), UNCERTAINTY_DIM)];
        break;
      default:
        break;
    }
  }
}

template <class CLASS_T, class PARAMS_T>
__host__ __device__ void RacerDubinsElevationImpl<CLASS_T, PARAMS_T>::computeUncertaintyPropagation(
    const float* state, const float* control, const float* state_der, float* next_state, float dt,
    DYN_PARAMS_T* params_p, SharedBlock* uncertainty_data, float* theta_s)
{
  CLASS_T* derived = static_cast<CLASS_T*>(this);
  int step, pi;
  mp1::getParallel1DIndex<mp1::Parallel1Dir::THREAD_Y>(pi, step);
  // if (params_p->gear_sign == -1)
  // {  // Set Uncertainty to zero in reverse
  //   for (int i = pi; i < UNCERTAINTY_DIM * UNCERTAINTY_DIM; i += step)
  //   {
  //     uncertainty_data->Sigma_a[i] = 0.0f;
  //   }
  // #ifdef __CUDA_ARCH__
  //   __syncthreads();
  // #endif
  //   derived->uncertaintyMatrixToState(uncertainty_data->Sigma_a, next_state);
  // #ifdef __CUDA_ARCH__
  //   __syncthreads();
  // #endif
  //   return;
  // }
  derived->computeUncertaintyJacobian(state, control, uncertainty_data->A, params_p);
  derived->uncertaintyStateToMatrix(state, uncertainty_data->Sigma_a);
#ifdef __CUDA_ARCH__
  __syncthreads();  // TODO: Check if this syncthreads is even needed
#endif
  // Turn A into (I + A dt)
  for (int i = pi; i < UNCERTAINTY_DIM * UNCERTAINTY_DIM; i += step)
  {
    uncertainty_data->A[i] = (i % (UNCERTAINTY_DIM + 1) == 0) + uncertainty_data->A[i] * dt;
  }

#ifdef __CUDA_ARCH__
  __syncthreads();
  mm::gemm1<UNCERTAINTY_DIM, UNCERTAINTY_DIM, UNCERTAINTY_DIM, mp1::Parallel1Dir::THREAD_Y>(
      uncertainty_data->A, uncertainty_data->Sigma_a, uncertainty_data->Sigma_b, 1.0f, 0.0f, mm::MAT_OP::NONE,
      mm::MAT_OP::NONE);
  __syncthreads();
  mm::gemm1<UNCERTAINTY_DIM, UNCERTAINTY_DIM, UNCERTAINTY_DIM, mp1::Parallel1Dir::THREAD_Y>(
      uncertainty_data->Sigma_b, uncertainty_data->A, uncertainty_data->Sigma_a, 1.0f, 0.0f, mm::MAT_OP::NONE,
      mm::MAT_OP::TRANSPOSE);
  __syncthreads();
#else
  typedef Eigen::Matrix<float, UNCERTAINTY_DIM, UNCERTAINTY_DIM> eigen_uncertainty_matrx;
  Eigen::Map<eigen_uncertainty_matrx> A_eigen(uncertainty_data->A);
  Eigen::Map<eigen_uncertainty_matrx> Sigma_a_eigen(uncertainty_data->Sigma_a);
  Eigen::Map<eigen_uncertainty_matrx> Sigma_b_eigen(uncertainty_data->Sigma_b);
  Sigma_a_eigen = A_eigen * Sigma_a_eigen * A_eigen.transpose();
#endif
  // float Q[UNCERTAINTY_DIM * UNCERTAINTY_DIM] = {
  //   1.0f, 0.0f, 0.0f, 0.0f,
  //   0.0f, 0.01f, 0.0f, 0.0f,
  //   0.0f, 0.0f, 1.0f, 0.0f,
  //   0.0f, 0.0f, 0.0f, 0.25f,
  // };
  derived->computeQ(state, control, state_der, uncertainty_data->Sigma_b, params_p, theta_s);
#ifdef __CUDA_ARCH__
  __syncthreads();  // TODO: Check if this syncthreads is even needed
#endif
  for (int i = pi; i < UNCERTAINTY_DIM * UNCERTAINTY_DIM; i += step)
  {
    uncertainty_data->Sigma_a[i] += uncertainty_data->Sigma_b[i] * dt;
  }
#ifdef __CUDA_ARCH__
  __syncthreads();
#endif
  derived->uncertaintyMatrixToState(uncertainty_data->Sigma_a, next_state);
#ifdef __CUDA_ARCH__
  __syncthreads();
#endif
}

template <class CLASS_T, class PARAMS_T>
__device__ void RacerDubinsElevationImpl<CLASS_T, PARAMS_T>::initializeDynamics(float* state, float* control,
                                                                                float* output, float* theta_s,
                                                                                float t_0, float dt)
{
  PARENT_CLASS::initializeDynamics(state, control, output, theta_s, t_0, dt);
  if (this->SHARED_MEM_REQUEST_GRD_BYTES != 0)
  {  // Allows us to turn on or off global or shared memory version of params
    DYN_PARAMS_T* shared_params = (DYN_PARAMS_T*)theta_s;
    *shared_params = this->params_;
  }
}

template <class CLASS_T, class PARAMS_T>
__device__ void RacerDubinsElevationImpl<CLASS_T, PARAMS_T>::computeParametricAccelDeriv(float* state, float* control,
                                                                                         float* state_der,
                                                                                         const float dt,
                                                                                         DYN_PARAMS_T* params_p)
{
  const int tdy = threadIdx.y;
  float linear_brake_slope = 0.2f;

  // Compute dynamics
  bool enable_brake = control[C_INDEX(THROTTLE_BRAKE)] < 0.0f;
  int index = (fabsf(state[S_INDEX(VEL_X)]) > linear_brake_slope && fabsf(state[S_INDEX(VEL_X)]) <= 3.0f) +
              (fabsf(state[S_INDEX(VEL_X)]) > 3.0f) * 2;
  const float brake_state = fminf(fmaxf(state[S_INDEX(BRAKE_STATE)], 0.0f), 0.25f);
  // applying position throttle
  float throttle = params_p->c_t[index] * control[C_INDEX(THROTTLE_BRAKE)];
  float brake = params_p->c_b[index] * brake_state * (state[S_INDEX(VEL_X)] >= 0.0f ? -1.0f : 1.0f);
  if (fabsf(state[S_INDEX(VEL_X)]) <= linear_brake_slope)
  {
    throttle = params_p->c_t[index] * fmaxf(control[C_INDEX(THROTTLE_BRAKE)] - params_p->low_min_throttle, 0.0f);
    brake = params_p->c_b[index] * brake_state * -state[S_INDEX(VEL_X)];
  }

  if (tdy == 0)
  {
    state_der[S_INDEX(VEL_X)] = (!enable_brake) * throttle * params_p->gear_sign + brake -
                                params_p->c_v[index] * state[S_INDEX(VEL_X)] + params_p->c_0;
    state_der[S_INDEX(VEL_X)] = fminf(fmaxf(state_der[S_INDEX(VEL_X)], -params_p->clamp_ax), params_p->clamp_ax);
    if (fabsf(state[S_INDEX(PITCH)]) < static_cast<float>(M_PI_2))
    {
      state_der[S_INDEX(VEL_X)] -= params_p->gravity * __sinf(angle_utils::normalizeAngle(state[S_INDEX(PITCH)]));
    }
  }
  state_der[S_INDEX(YAW)] =
      (state[S_INDEX(VEL_X)] / params_p->wheel_base) *
      __tanf(angle_utils::normalizeAngle(state[S_INDEX(STEER_ANGLE)] / params_p->steer_angle_scale));
  const float yaw_norm = angle_utils::normalizeAngle(state[S_INDEX(YAW)]);
  state_der[S_INDEX(POS_X)] = state[S_INDEX(VEL_X)] * __cosf(yaw_norm);
  state_der[S_INDEX(POS_Y)] = state[S_INDEX(VEL_X)] * __sinf(yaw_norm);
}

template <class CLASS_T, class PARAMS_T>
__device__ void RacerDubinsElevationImpl<CLASS_T, PARAMS_T>::updateState(float* state, float* next_state,
                                                                         float* state_der, const float dt,
                                                                         DYN_PARAMS_T* params_p)
{
  // const uint tdy = threadIdx.y;
  int pi, step;
  mp1::getParallel1DIndex<mp1::Parallel1Dir::THREAD_Y>(pi, step);

  // Set to 6 as the last 3 states do not do euler integration
  for (int i = pi; i < 6; i += step)
  {
    // if (i == S_INDEX(ROLL) || i == S_INDEX(PITCH) || i == S_INDEX(STEER_ANGLE_RATE))
    // {
    //   continue;
    // }
    next_state[i] = state[i] + state_der[i] * dt;
    switch (i)
    {
      case S_INDEX(YAW):
        next_state[i] = angle_utils::normalizeAngle(next_state[i]);
        break;
      case S_INDEX(STEER_ANGLE):
        next_state[S_INDEX(STEER_ANGLE)] =
            fmaxf(fminf(next_state[S_INDEX(STEER_ANGLE)], params_p->max_steer_angle), -params_p->max_steer_angle);
        next_state[S_INDEX(STEER_ANGLE_RATE)] = state_der[S_INDEX(STEER_ANGLE)];
        break;
      case S_INDEX(BRAKE_STATE):
        next_state[S_INDEX(BRAKE_STATE)] = fminf(fmaxf(next_state[S_INDEX(BRAKE_STATE)], 0.0f), 1.0f);
        break;
    }
  }
  __syncthreads();
}

template <class CLASS_T, class PARAMS_T>
__device__ inline void RacerDubinsElevationImpl<CLASS_T, PARAMS_T>::step(float* state, float* next_state,
                                                                         float* state_der, float* control,
                                                                         float* output, float* theta_s, const float t,
                                                                         const float dt)
{
  DYN_PARAMS_T* params_p;
  SharedBlock* sb;
  // TODO the below two if statements conflict in a bad way
  if (this->SHARED_MEM_REQUEST_GRD_BYTES != 0)
  {  // Allows us to turn on or off global or shared memory version of params
    params_p = (DYN_PARAMS_T*)theta_s;
  }
  else
  {
    params_p = &(this->params_);
  }
  if (this->SHARED_MEM_REQUEST_BLK_BYTES != 0)
  {
    float* sb_mem =
        &theta_s[mppi::math::int_multiple_const(this->SHARED_MEM_REQUEST_GRD_BYTES, sizeof(float4)) / sizeof(float)];
    sb = (SharedBlock*)&sb_mem[(this->SHARED_MEM_REQUEST_BLK_BYTES * threadIdx.x +
                                this->SHARED_MEM_REQUEST_BLK_BYTES * blockDim.x * threadIdx.z) /
                               sizeof(float)];
  }
  this->computeParametricDelayDeriv(state, control, state_der, params_p);
  this->computeParametricSteerDeriv(state, control, state_der, params_p);
  computeParametricAccelDeriv(state, control, state_der, dt, params_p);

  updateState(state, next_state, state_der, dt, params_p);
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
  this->setOutputs(state_der, next_state, output);
}

template <class CLASS_T, class PARAMS_T>
RacerDubinsElevationImpl<CLASS_T, PARAMS_T>::state_array
RacerDubinsElevationImpl<CLASS_T, PARAMS_T>::stateFromMap(const std::map<std::string, float>& map)
{
  std::vector<std::string> keys = { "VEL_X",      "VEL_Y", "POS_X",       "POS_Y",
                                    "ROLL",       "PITCH", "STEER_ANGLE", "STEER_ANGLE_RATE",
                                    "BRAKE_STATE" };
  bool look_for_uncertainty = false;
  if (look_for_uncertainty)
  {
    keys.push_back("UNCERTAINTY_POS_X");
    keys.push_back("UNCERTAINTY_POS_Y");
    keys.push_back("UNCERTAINTY_YAW");
    keys.push_back("UNCERTAINTY_VEL_X");
    keys.push_back("UNCERTAINTY_POS_X_Y");
    keys.push_back("UNCERTAINTY_POS_X_YAW");
    keys.push_back("UNCERTAINTY_POS_X_VEL_X");
    keys.push_back("UNCERTAINTY_POS_Y_YAW");
    keys.push_back("UNCERTAINTY_POS_Y_VEL_X");
    keys.push_back("UNCERTAINTY_YAW_VEL_X");
  }

  bool found_all_keys = true;
  for (const auto& key : keys)
  {
    if (map.find(key) == map.end())
    {
      this->logger_->warning("Could not find %s key for elevation dynamics\n", key.c_str());
      found_all_keys = false;
    }
  }

  state_array s = this->getZeroState();
  if (!found_all_keys)
  {
    return state_array::Constant(NAN);
  }
  s(S_INDEX(POS_X)) = map.at("POS_X");
  s(S_INDEX(POS_Y)) = map.at("POS_Y");
  s(S_INDEX(VEL_X)) = map.at("VEL_X");
  s(S_INDEX(YAW)) = map.at("YAW");
  s(S_INDEX(STEER_ANGLE)) = map.at("STEER_ANGLE");
  s(S_INDEX(STEER_ANGLE_RATE)) = map.at("STEER_ANGLE_RATE");
  s(S_INDEX(ROLL)) = map.at("ROLL");
  s(S_INDEX(PITCH)) = map.at("PITCH");
  s(S_INDEX(BRAKE_STATE)) = map.at("BRAKE_STATE");
  if (look_for_uncertainty)
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

template <class CLASS_T, class PARAMS_T>
Eigen::Quaternionf
RacerDubinsElevationImpl<CLASS_T, PARAMS_T>::attitudeFromState(const Eigen::Ref<const state_array>& state)
{
  Eigen::Quaternionf q;
  mppi::math::Euler2QuatNWU(state(S_INDEX(ROLL)), state(S_INDEX(PITCH)), state(S_INDEX(YAW)), q);
  return q;
}

template <class CLASS_T, class PARAMS_T>
Eigen::Vector3f
RacerDubinsElevationImpl<CLASS_T, PARAMS_T>::positionFromState(const Eigen::Ref<const state_array>& state)
{
  return Eigen::Vector3f(state[S_INDEX(POS_X)], state[S_INDEX(POS_Y)], 0.0f);
}
