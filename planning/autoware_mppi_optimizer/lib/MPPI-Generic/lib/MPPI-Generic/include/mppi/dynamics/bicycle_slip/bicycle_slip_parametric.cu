//
// Created by jason on 12/12/22.
//

#include "bicycle_slip_parametric.cuh"

template <class CLASS_T, class PARAMS_T>
BicycleSlipParametricImpl<CLASS_T, PARAMS_T>::BicycleSlipParametricImpl(cudaStream_t stream) : PARENT_CLASS(stream)
{
  normals_tex_helper_ = new TwoDTextureHelper<float4>(1, stream);
}

template <class CLASS_T, class PARAMS_T>
void BicycleSlipParametricImpl<CLASS_T, PARAMS_T>::paramsToDevice()
{
  normals_tex_helper_->copyToDevice();
  PARENT_CLASS::paramsToDevice();
}

template <class CLASS_T, class PARAMS_T>
BicycleSlipParametricImpl<CLASS_T, PARAMS_T>::state_array
BicycleSlipParametricImpl<CLASS_T, PARAMS_T>::stateFromMap(const std::map<std::string, float>& map)
{
  state_array s = state_array::Zero();
  if (map.find("VEL_X") == map.end() || map.find("VEL_Y") == map.end() || map.find("POS_X") == map.end() ||
      map.find("POS_Y") == map.end() || map.find("ROLL") == map.end() || map.find("PITCH") == map.end())
  {
    std::cout << "WARNING: could not find all keys for ackerman slip dynamics" << std::endl;
    for (const auto& it : map)
    {
      std::cout << "got key " << it.first << std::endl;
    }
    return s;
  }
  s(S_INDEX(POS_X)) = map.at("POS_X");
  s(S_INDEX(POS_Y)) = map.at("POS_Y");
  s(S_INDEX(VEL_X)) = map.at("VEL_X");
  s(S_INDEX(VEL_Y)) = map.at("VEL_Y");
  s(S_INDEX(OMEGA_Z)) = map.at("OMEGA_Z");
  s(S_INDEX(YAW)) = map.at("YAW");
  s(S_INDEX(ROLL)) = map.at("ROLL");
  s(S_INDEX(PITCH)) = map.at("PITCH");
  if (map.find("STEER_ANGLE") != map.end())
  {
    s(S_INDEX(STEER_ANGLE)) = map.at("STEER_ANGLE");
    s(S_INDEX(STEER_ANGLE_RATE)) = map.at("STEER_ANGLE_RATE");
  }
  else
  {
    std::cout << "WARNING: unable to find BRAKE_STATE or STEER_ANGLE_RATE, using 0" << std::endl;
    s(S_INDEX(STEER_ANGLE)) = 0;
    s(S_INDEX(STEER_ANGLE_RATE)) = 0;
  }
  if (map.find("BRAKE_STATE") != map.end())
  {
    s(S_INDEX(BRAKE_STATE)) = map.at("BRAKE_STATE");
  }
  else if (map.find("BRAKE_CMD") != map.end())
  {
    std::cout << "WARNING: unable to find BRAKE_STATE" << std::endl;
    s(S_INDEX(BRAKE_STATE)) = map.at("BRAKE_CMD");
  }
  else
  {
    std::cout << "WARNING: unable to find BRAKE_CMD or BRAKE_STATE" << std::endl;
    s(S_INDEX(BRAKE_STATE)) = 0;
  }
  return s;
}

template <class CLASS_T, class PARAMS_T>
void BicycleSlipParametricImpl<CLASS_T, PARAMS_T>::GPUSetup()
{
  PARENT_CLASS::GPUSetup();

  normals_tex_helper_->GPUSetup();
  // makes sure that the device ptr sees the correct texture object
  HANDLE_ERROR(cudaMemcpyAsync(&(this->model_d_->normals_tex_helper_), &(normals_tex_helper_->ptr_d_),
                               sizeof(TwoDTextureHelper<float4>*), cudaMemcpyHostToDevice, this->stream_));
}

template <class CLASS_T, class PARAMS_T>
void BicycleSlipParametricImpl<CLASS_T, PARAMS_T>::freeCudaMem()
{
  normals_tex_helper_->freeCudaMem();
  PARENT_CLASS::freeCudaMem();
}

template <class CLASS_T, class PARAMS_T>
void BicycleSlipParametricImpl<CLASS_T, PARAMS_T>::computeDynamics(const Eigen::Ref<const state_array>& state,
                                                                   const Eigen::Ref<const control_array>& control,
                                                                   Eigen::Ref<state_array> state_der)
{
  state_der = state_array::Zero();
  bool enable_brake = control[C_INDEX(THROTTLE_BRAKE)] < 0;

  // runs parametric delay model
  // this->computeParametricDelayDeriv(state, control, state_der);

  // // runs the parametric part of the steering model
  // // TODO this needs to be the NN
  // this->computeParametricSteerDeriv(state, control, state_der);

  // // TODO need to predict the brake state change

  // float normal_x_avg, normal_y_avg, normal_z_avg;
  // RACER::computeBodyFrameNormals<TwoDTextureHelper<float4>>(
  //     this->normals_tex_helper_, state(S_INDEX(YAW)), state(S_INDEX(POS_X)), state(S_INDEX(POS_Y)),
  //     state(S_INDEX(ROLL)), state(S_INDEX(PITCH)), normal_x_avg, normal_y_avg, normal_z_avg);

  // const float brake = tanh_vel_scale(state(S_INDEX(BRAKE_STATE)), state(S_INDEX(VEL_X)), this->params.c_brake);
  // const float drag_x = tanh_scale(state(S_INDEX(VEL_X)), this->params.c_rolling);
  // const float scaled_rpm = state(S_INDEX(ENGINE_RPM)) / 1.0e3 - this->params_.min_rpm;
  // const float throttle = tanhf(powf(scaled_rpm, 2) * this->params_.c_rpm[2] + scaled_rpm * this->params_.c_rpm[1] +
  //                              this->params_.c_rpm[0]) *
  //                        this->params_.rpm_scale;

  // const float x_force = throttle - brake - drag_x;

  // const float wheel_angle = tanf(state(S_INDEX(STEER_ANGLE)) / this->params_.steer_angle_scale);
  // const float parametric_omega = (state(S_INDEX(VEL_X)) / this->params_.wheel_base) * wheel_angle;
  // state_der(S_INDEX(OMEGA_Z)) = (parametric_omega - state(S_INDEX(OMEGA_Z))) * this->params_.c_omega -
  //                               state(S_INDEX(OMEGA_Z)) * this->params_.c_v_omega;

  // const float drag_y = tanh_scale(state(S_INDEX(VEL_Y)), this->params_.c_sliding);
  // const float y_force =
  //     tanhf(state(S_INDEX(VEL_X)) * state(S_INDEX(OMEGA_Z)) * this->params_.y_f_c[0]) * this->params_.y_f_c[1] -
  //     drag_y;

  // float wheel_angle, sin_wheel_angle, cos_wheel_angle;
  // sincosf(wheel_angle, &sin_wheel_angle, &cos_wheel_angle);

  // state_der(S_INDEX(VEL_X)) = (x_force + x_force * cos_wheel_angle - y_force * sin_wheel_angle) / this->params_.mass
  // -
  //                             state(S_INDEX(VEL_X)) * this->params_.c_vx - normal_x_avg * this->params_.gravity_x +
  //                             state(S_INDEX(VEL_Y)) * state(S_INDEX(OMEGA_Z));

  // state_der(S_INDEX(VEL_Y)) = (y_force + y_force * cos_wheel_angle + x_force * sin_wheel_angle) / this->params_.mass
  // -
  //                             state(S_INDEX(VEL_Y)) * this->params_.c_vy - normal_y_avg * this->params_.gravity_y -
  //                             state(S_INDEX(VEL_X)) * state(S_INDEX(OMEGA_Z));

  // // kinematics component
  // state_der(S_INDEX(YAW)) = state(S_INDEX(OMEGA_Z));
  // float yaw, sin_yaw, cos_yaw;
  // yaw = state[S_INDEX(YAW)];
  // sincosf(yaw, &sin_yaw, &cos_yaw);
  // state_der(S_INDEX(POS_X)) = state(S_INDEX(VEL_X)) * cos_yaw - state(S_INDEX(VEL_Y)) * sin_yaw;
  // state_der(S_INDEX(POS_Y)) = state(S_INDEX(VEL_X)) * sin_yaw + state(S_INDEX(VEL_Y)) * cos_yaw;
}

template <class CLASS_T, class PARAMS_T>
void BicycleSlipParametricImpl<CLASS_T, PARAMS_T>::updateState(const Eigen::Ref<const state_array> state,
                                                               Eigen::Ref<state_array> next_state,
                                                               Eigen::Ref<state_array> state_der, const float dt)
{
  next_state = state + state_der * dt;
  next_state(S_INDEX(YAW)) = angle_utils::normalizeAngle(next_state(S_INDEX(YAW)));
  next_state(S_INDEX(STEER_ANGLE)) =
      max(min(next_state(S_INDEX(STEER_ANGLE)), this->params_.max_steer_angle), -this->params_.max_steer_angle);
  next_state(S_INDEX(STEER_ANGLE_RATE)) = state_der(S_INDEX(STEER_ANGLE));
  next_state(S_INDEX(BRAKE_STATE)) =
      min(max(next_state(S_INDEX(BRAKE_STATE)), 0.0f), -this->control_rngs_[C_INDEX(THROTTLE_BRAKE)].x);
  next_state(S_INDEX(ROLL)) = state(S_INDEX(ROLL));
  next_state(S_INDEX(PITCH)) = state(S_INDEX(PITCH));
}

template <class CLASS_T, class PARAMS_T>
void BicycleSlipParametricImpl<CLASS_T, PARAMS_T>::step(Eigen::Ref<state_array> state,
                                                        Eigen::Ref<state_array> next_state,
                                                        Eigen::Ref<state_array> state_der,
                                                        const Eigen::Ref<const control_array>& control,
                                                        Eigen::Ref<output_array> output, const float t, const float dt)
{
  computeDynamics(state, control, state_der);
  updateState(state, next_state, state_der, dt);

#ifdef BICYCLE_UNCERTAINTY
  typename PARENT_CLASS::SharedBlock sb;
#endif

  output = output_array::Zero();

#ifdef BICYCLE_UNCERTAINTY
  PARENT_CLASS::computeUncertaintyPropagation(state.data(), control.data(), state_der.data(), next_state.data(), dt,
                                              &this->params_, &sb, nullptr);
#endif

  float roll = state(S_INDEX(ROLL));
  float pitch = state(S_INDEX(PITCH));
  RACER::computeStaticSettling<typename DYN_PARAMS_T::OutputIndex, TwoDTextureHelper<float>>(
      this->tex_helper_, next_state(S_INDEX(YAW)), next_state(S_INDEX(POS_X)), next_state(S_INDEX(POS_Y)), roll, pitch,
      output.data());
  next_state[S_INDEX(PITCH)] = pitch;
  next_state[S_INDEX(ROLL)] = roll;

  this->setOutputs(state_der.data(), next_state.data(), output.data());

  output[O_INDEX(BASELINK_VEL_B_Y)] = next_state[S_INDEX(VEL_Y)];
  output[O_INDEX(ACCEL_Y)] = state_der[S_INDEX(VEL_Y)];
  output[O_INDEX(OMEGA_Z)] = next_state[S_INDEX(OMEGA_Z)];
  output[O_INDEX(TOTAL_VELOCITY)] =
      mppi::math::sign(next_state(S_INDEX(VEL_X))) * sqrt(next_state[S_INDEX(VEL_X)] * next_state[S_INDEX(VEL_X)] +
                                                          next_state[S_INDEX(VEL_Y)] * next_state[S_INDEX(VEL_Y)]);
}

template <class CLASS_T, class PARAMS_T>
__device__ void BicycleSlipParametricImpl<CLASS_T, PARAMS_T>::updateState(float* state, float* next_state,
                                                                          float* state_der, const float dt,
                                                                          typename PARENT_CLASS::DYN_PARAMS_T* params_p)
{
  for (int i = threadIdx.y; i < 8; i += blockDim.y)
  {
    next_state[i] = state[i] + state_der[i] * dt;
    switch (i)
    {
      case S_INDEX(YAW):
        next_state[i] = angle_utils::normalizeAngle(next_state[i]);
        break;
      case S_INDEX(STEER_ANGLE):
        next_state[S_INDEX(STEER_ANGLE)] =
            max(min(next_state[S_INDEX(STEER_ANGLE)], params_p->max_steer_angle), -params_p->max_steer_angle);
        next_state[S_INDEX(STEER_ANGLE_RATE)] = state_der[S_INDEX(STEER_ANGLE)];
        break;
      case S_INDEX(BRAKE_STATE):
        next_state[S_INDEX(BRAKE_STATE)] =
            min(max(next_state[S_INDEX(BRAKE_STATE)], 0.0f), -this->control_rngs_[C_INDEX(THROTTLE_BRAKE)].x);
    }
  }

  __syncthreads();
}

template <class CLASS_T, class PARAMS_T>
__device__ void BicycleSlipParametricImpl<CLASS_T, PARAMS_T>::computeDynamics(float* state, float* control,
                                                                              float* state_der, float* theta)
{
  DYN_PARAMS_T* params_p = nullptr;

  if (PARENT_CLASS::SHARED_MEM_REQUEST_GRD_BYTES != 0)
  {  // Allows us to turn on or off global or shared memory version of params
    params_p = (DYN_PARAMS_T*)theta;
  }
  else
  {
    params_p = &(this->params_);
  }

  // parametric part of the brake
  this->computeParametricDelayDeriv(state, control, state_der, params_p);

  // runs the parametric part of the steering model
  this->computeParametricSteerDeriv(state, control, state_der, params_p);

  // nullptr if not shared memory
  // bool enable_brake = control[C_INDEX(THROTTLE_BRAKE)] < 0;
  // const float brake_cmd = -enable_brake * control[C_INDEX(THROTTLE_BRAKE)];
  // float linear_brake_slope = params_p->c_b[1] / (0.9f * (2.0f / 0.02));
  // int index = (fabsf(state[S_INDEX(VEL_X)]) > linear_brake_slope && fabsf(state[S_INDEX(VEL_X)]) <= 3.0f) +
  //             (fabsf(state[S_INDEX(VEL_X)]) > 3.0f) * 2;
  // float throttle = params_p->c_t[index] * control[C_INDEX(THROTTLE_BRAKE)] * params_p->gear_sign;

  // // calculates the average normals
  // float normal_x_avg, normal_y_avg, normal_z_avg;
  // RACER::computeBodyFrameNormals<TwoDTextureHelper<float4>>(
  //     this->normals_tex_helper_, state[S_INDEX(YAW)], state[S_INDEX(POS_X)], state[S_INDEX(POS_Y)],
  //     state[S_INDEX(ROLL)], state[S_INDEX(PITCH)], normal_x_avg, normal_y_avg, normal_z_avg);
  // const float gravity_x_accel = act_func::tanhshrink_scale(normal_x_avg, params_p->min_normal_x) *
  // params_p->gravity_x; const float gravity_y_accel = act_func::tanhshrink_scale(normal_y_avg, params_p->min_normal_y)
  // * params_p->gravity_y;

  // const float brake_vel = mppi::math::clamp(state[S_INDEX(VEL_X)], -params_p->brake_vel, params_p->brake_vel);
  // const float rolling_vel =
  //     mppi::math::clamp(state[S_INDEX(VEL_X)], -params_p->max_roll_resistance_vel,
  //     params_p->max_roll_resistance_vel);
  // const float sliding_vel = mppi::math::clamp(state[S_INDEX(VEL_Y)], -params_p->max_slide_vel,
  // params_p->max_slide_vel);

  // const float brake = params_p->c_b[0] * state[S_INDEX(BRAKE_STATE)] *
  //                     mppi::math::clamp(state[S_INDEX(VEL_X)], -params_p->brake_vel, params_p->brake_vel);
  // const float x_drag = params_p->c_v[0] * state[S_INDEX(VEL_X)] + rolling_vel * normal_z_avg * params_p->c_rolling;
  // const float y_vel_comp = state[S_INDEX(VEL_Y)] * state[S_INDEX(OMEGA_Z)];
  // const float accel_x = throttle - brake - x_drag;
  // const float mu_actual = (params_p->mu + params_p->environment * params_p->mu_env) * normal_z_avg;
  // state_der[S_INDEX(VEL_X)] = mppi::math::clamp(accel_x, -mu_actual, mu_actual) - gravity_x_accel + y_vel_comp;

  // float y_accel = -state[S_INDEX(VEL_X)] * state[S_INDEX(OMEGA_Z)] +
  //                 mppi::math::sign(state[S_INDEX(VEL_X)]) * state[S_INDEX(OMEGA_Z)] * params_p->vy_omega;
  // const float drag_y = params_p->c_vy * state[S_INDEX(VEL_Y)] + sliding_vel * normal_z_avg * params_p->c_sliding;
  // state_der[S_INDEX(VEL_Y)] = y_accel - drag_y - gravity_y_accel;

  // const float parametric_omega =
  //     (state[S_INDEX(VEL_X)] / params_p->wheel_base) * tanf(state[S_INDEX(STEER_ANGLE)] /
  //     params_p->steer_angle_scale);
  // state_der[S_INDEX(OMEGA_Z)] =
  //     (parametric_omega - state[S_INDEX(OMEGA_Z)]) * params_p->c_omega - state[S_INDEX(OMEGA_Z)] *
  //     params_p->c_v_omega;

  // // kinematics component
  // state_der[S_INDEX(YAW)] = state[S_INDEX(OMEGA_Z)];
  // float yaw, sin_yaw, cos_yaw;
  // yaw = angle_utils::normalizeAngle(state[S_INDEX(YAW)]);
  // __sincosf(yaw, &sin_yaw, &cos_yaw);
  // state_der[S_INDEX(POS_X)] = state[S_INDEX(VEL_X)] * cos_yaw - state[S_INDEX(VEL_Y)] * sin_yaw;
  // state_der[S_INDEX(POS_Y)] = state[S_INDEX(VEL_X)] * sin_yaw + state[S_INDEX(VEL_Y)] * cos_yaw;
}

template <class CLASS_T, class PARAMS_T>
__device__ void BicycleSlipParametricImpl<CLASS_T, PARAMS_T>::step(float* state, float* next_state, float* state_der,
                                                                   float* control, float* output, float* theta_s,
                                                                   const float t, const float dt)
{
  DYN_PARAMS_T* params_p;
#ifdef BICYCLE_UNCERTAINTY
  typename PARENT_CLASS::SharedBlock *sb_mem, *sb;
#endif
  if (PARENT_CLASS::SHARED_MEM_REQUEST_GRD_BYTES != 0)
  {  // Allows us to turn on or off global or shared memory version of params
    params_p = (DYN_PARAMS_T*)theta_s;
  }
  else
  {
    params_p = &(this->params_);
  }
#ifdef BICYCLE_UNCERTAINTY
  if (this->SHARED_MEM_REQUEST_BLK_BYTES != 0)
  {
    sb_mem = (typename PARENT_CLASS::SharedBlock*)&theta_s[mppi::math::int_multiple_const(
                                                               this->SHARED_MEM_REQUEST_GRD_BYTES, sizeof(float4)) /
                                                           sizeof(float)];
    sb = &sb_mem[threadIdx.x + blockDim.x * threadIdx.z];
  }
#endif
  const uint tdy = threadIdx.y;

  computeDynamics(state, control, state_der, theta_s);
  updateState(state, next_state, state_der, dt, params_p);
#ifdef BICYCLE_UNCERTAINTY
  PARENT_CLASS::computeUncertaintyPropagation(state, control, state_der, next_state, dt, params_p, sb, theta_s);
#endif

  if (tdy == 0)
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

  // Rather than need a syncthreads, just overwrite the given outputs using the proper thread
  if (tdy == O_INDEX(BASELINK_VEL_B_Y) % blockIdx.y)
  {
    output[O_INDEX(BASELINK_VEL_B_Y)] = next_state[S_INDEX(VEL_Y)];
  }
  if (tdy == O_INDEX(ACCEL_Y) % blockIdx.y)
  {
    output[O_INDEX(ACCEL_Y)] = state_der[S_INDEX(VEL_Y)];
  }
  if (tdy == O_INDEX(OMEGA_Z) % blockIdx.y)
  {
    output[O_INDEX(OMEGA_Z)] = next_state[S_INDEX(OMEGA_Z)];
  }
  if (tdy == O_INDEX(TOTAL_VELOCITY) % blockIdx.y)
  {
    const float vel_x_sign = output[O_INDEX(TOTAL_VELOCITY)] =
        mppi::math::sign(next_state[S_INDEX(VEL_X)]) * sqrtf(next_state[S_INDEX(VEL_X)] * next_state[S_INDEX(VEL_X)] +
                                                             next_state[S_INDEX(VEL_Y)] * next_state[S_INDEX(VEL_Y)]);
  }
}

template <class CLASS_T, class PARAMS_T>
Eigen::Vector3f
BicycleSlipParametricImpl<CLASS_T, PARAMS_T>::velocityFromState(const Eigen::Ref<const state_array>& state)
{
  return Eigen::Vector3f(state[S_INDEX(VEL_X)], state(S_INDEX(VEL_Y)), 0);
}

template <class CLASS_T, class PARAMS_T>
Eigen::Vector3f
BicycleSlipParametricImpl<CLASS_T, PARAMS_T>::angularRateFromState(const Eigen::Ref<const state_array>& state)
{
  return Eigen::Vector3f(0, 0, state[S_INDEX(OMEGA_Z)]);
}

template <class TEX_T>
__device__ __host__ void RACER::computeBodyFrameNormals(TEX_T* tex_helper, const float yaw, const float x,
                                                        const float y, const float roll, const float pitch,
                                                        float& mean_normals_x, float& mean_normals_y,
                                                        float& mean_normals_z)
{
  float3 front_left = make_float3(2.981f, 0.737f, 0.0f);
  float3 front_right = make_float3(2.981f, -0.737f, 0.f);
  float3 rear_left = make_float3(0.0f, 0.737f, 0.0f);
  float3 rear_right = make_float3(0.0f, -0.737f, 0.0f);
  float3 body_pose = make_float3(x, y, 0.0f);
  float3 rotation = make_float3(roll, pitch, yaw);

  if (tex_helper->checkTextureUse(0))
  {
    float4 front_left_normals = tex_helper->queryTextureAtWorldOffsetPose(0, body_pose, front_left, rotation);
    float4 front_right_normals = tex_helper->queryTextureAtWorldOffsetPose(0, body_pose, front_right, rotation);
    float4 rear_left_normals = tex_helper->queryTextureAtWorldOffsetPose(0, body_pose, rear_left, rotation);
    float4 rear_right_normals = tex_helper->queryTextureAtWorldOffsetPose(0, body_pose, rear_right, rotation);

    // TODO need to rotate the normals for x,y

#ifdef __CUDA_ARCH__
    const float cos_yaw = __cosf(yaw);
    const float sin_yaw = __sinf(yaw);
#else
    const float cos_yaw = cosf(yaw);
    const float sin_yaw = sinf(yaw);
#endif

    float3 front_left_normals_rot;
    front_left_normals_rot.x = cos_yaw * front_left_normals.x - sin_yaw * front_left_normals.y;
    front_left_normals_rot.y = sin_yaw * front_left_normals.x + cos_yaw * front_left_normals.y;
    front_left_normals_rot.z = front_left_normals.z;

    float3 front_right_normals_rot;
    front_right_normals_rot.x = cos_yaw * front_right_normals.x - sin_yaw * front_right_normals.y;
    front_right_normals_rot.y = sin_yaw * front_right_normals.x + cos_yaw * front_right_normals.y;
    front_right_normals_rot.z = front_right_normals.z;

    float3 rear_left_normals_rot;
    rear_left_normals_rot.x = cos_yaw * rear_left_normals.x - sin_yaw * rear_left_normals.y;
    rear_left_normals_rot.y = sin_yaw * rear_left_normals.x + cos_yaw * rear_left_normals.y;
    rear_left_normals_rot.z = rear_left_normals.z;

    float3 rear_right_normals_rot;
    rear_right_normals_rot.x = cos_yaw * rear_right_normals.x - sin_yaw * rear_right_normals.y;
    rear_right_normals_rot.y = sin_yaw * rear_right_normals.x + cos_yaw * rear_right_normals.y;
    rear_right_normals_rot.z = rear_right_normals.z;

    mean_normals_x =
        (front_left_normals_rot.x + front_right_normals_rot.x + rear_right_normals_rot.x + rear_left_normals_rot.x) /
        4.0f;
    mean_normals_y =
        (front_left_normals_rot.y + front_right_normals_rot.y + rear_right_normals_rot.y + rear_left_normals_rot.y) /
        4.0f;
    mean_normals_z =
        (front_left_normals_rot.z + front_right_normals_rot.z + rear_right_normals_rot.z + rear_left_normals_rot.z) /
        4.0f;

    // using 2pi so any rotation that accidently uses this will be using identity
    if (!isfinite(mean_normals_x) || !isfinite(mean_normals_y) || !isfinite(mean_normals_z))
    {
      mean_normals_x = 0.0f;
      mean_normals_y = 0.0f;
      mean_normals_z = 1.0f;
    }
  }
  else
  {
    mean_normals_x = 0.0f;
    mean_normals_y = 0.0f;
    mean_normals_z = 1.0f;
  }
}

#ifdef BICYCLE_UNCERTAINTY
template <class CLASS_T, class PARAMS_T>
__host__ __device__ bool BicycleSlipParametricImpl<CLASS_T, PARAMS_T>::computeUncertaintyJacobian(const float* state,
                                                                                                  const float* control,
                                                                                                  float* A,
                                                                                                  PARAMS_T* params_p)
{
  float sin_yaw, cos_yaw, cos_2_delta;
#ifdef __CUDA_ARCH__
  float yaw_norm = angle_utils::normalizeAngle(state[S_INDEX(YAW)]);
  float delta = angle_utils::normalizeAngle(state[S_INDEX(STEER_ANGLE)] / params_p->steer_angle_scale);
  __sincosf(yaw_norm, &sin_yaw, &cos_yaw);
  cos_2_delta = __cosf(delta) * __cosf(delta);
#else
  sincosf(state[S_INDEX(YAW)], &sin_yaw, &cos_yaw);
  float delta = state[S_INDEX(STEER_ANGLE)] / params_p->steer_angle_scale;
  cos_2_delta = cosf(delta) * cosf(delta);
#endif

  int step, pi;
  mp1::getParallel1DIndex<mp1::Parallel1Dir::THREAD_Y>(pi, step);
  // A = df/dx + df/du * K
  for (int i = pi; i < this->UNCERTAINTY_DIM * this->UNCERTAINTY_DIM; i += step)
  {
    switch (i)
    {
      case mm::columnMajorIndex(U_INDEX(VEL_X), U_INDEX(VEL_X), this->UNCERTAINTY_DIM):
        A[i] = -params_p->c_v[0] - params_p->K_vel_x;
        break;
      case mm::columnMajorIndex(U_INDEX(VEL_X), U_INDEX(YAW), this->UNCERTAINTY_DIM):
        A[i] = 0.0f;
        break;
      case mm::columnMajorIndex(U_INDEX(VEL_X), U_INDEX(POS_X), this->UNCERTAINTY_DIM):
        A[i] = -params_p->K_x * cos_yaw;
        break;
      case mm::columnMajorIndex(U_INDEX(VEL_X), U_INDEX(POS_Y), this->UNCERTAINTY_DIM):
        A[i] = -params_p->K_x * sin_yaw;
        break;

      // yaw
      case mm::columnMajorIndex(U_INDEX(YAW), U_INDEX(VEL_X), this->UNCERTAINTY_DIM):
        A[i] = 0.0f;
        break;
      case mm::columnMajorIndex(U_INDEX(YAW), U_INDEX(YAW), this->UNCERTAINTY_DIM):
        A[i] = -fabsf(state[S_INDEX(VEL_X)]) * params_p->K_yaw / (params_p->wheel_base * cos_2_delta);
        break;
      case mm::columnMajorIndex(U_INDEX(YAW), U_INDEX(POS_X), this->UNCERTAINTY_DIM):
        A[i] = state[S_INDEX(VEL_X)] * params_p->K_y * sin_yaw / (params_p->wheel_base * cos_2_delta);
        break;
      case mm::columnMajorIndex(U_INDEX(YAW), U_INDEX(POS_Y), this->UNCERTAINTY_DIM):
        A[i] = -state[S_INDEX(VEL_X)] * params_p->K_y * cos_yaw / (params_p->wheel_base * cos_2_delta);
        break;
      // pos x
      case mm::columnMajorIndex(U_INDEX(POS_X), U_INDEX(VEL_X), this->UNCERTAINTY_DIM):
        A[i] = cos_yaw;
        break;
      case mm::columnMajorIndex(U_INDEX(POS_X), U_INDEX(YAW), this->UNCERTAINTY_DIM):
        A[i] = -sin_yaw * state[S_INDEX(VEL_X)] - cos_yaw * state[S_INDEX(VEL_Y)];
        break;
      case mm::columnMajorIndex(U_INDEX(POS_X), U_INDEX(POS_X), this->UNCERTAINTY_DIM):
        A[i] = 0.0f;
        break;
      case mm::columnMajorIndex(U_INDEX(POS_X), U_INDEX(POS_Y), this->UNCERTAINTY_DIM):
        A[i] = 0.0f;
        break;
      // pos y
      case mm::columnMajorIndex(U_INDEX(POS_Y), U_INDEX(VEL_X), this->UNCERTAINTY_DIM):
        A[i] = sin_yaw;
        break;
      case mm::columnMajorIndex(U_INDEX(POS_Y), U_INDEX(YAW), this->UNCERTAINTY_DIM):
        A[i] = cos_yaw * state[S_INDEX(VEL_X)] - sin_yaw * state[S_INDEX(VEL_Y)];
        break;
      case mm::columnMajorIndex(U_INDEX(POS_Y), U_INDEX(POS_Y), this->UNCERTAINTY_DIM):
        A[i] = 0.0f;
        break;
      case mm::columnMajorIndex(U_INDEX(POS_Y), U_INDEX(POS_X), this->UNCERTAINTY_DIM):
        A[i] = 0.0f;
        break;
    }
  }
  return true;
}
#endif
