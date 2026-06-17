//
// Created by Bogdan on 02/21/2024
//

#include <eigen3/Eigen/src/Geometry/Quaternion.h>
#include "racer_dubins_elevation_suspension_lstm.cuh"

#define TEMPLATE_TYPE template <class CLASS_T, class PARAMS_T>
#define TEMPLATE_NAME RacerDubinsElevationSuspensionImpl<CLASS_T, PARAMS_T>

TEMPLATE_TYPE
TEMPLATE_NAME::RacerDubinsElevationSuspensionImpl(int init_input_dim, int init_hidden_dim,
                                                  std::vector<int>& init_output_layers, int input_dim, int hidden_dim,
                                                  std::vector<int>& output_layers, int init_len, cudaStream_t stream)
  : PARENT_CLASS(init_input_dim, init_hidden_dim, init_output_layers, input_dim, hidden_dim, output_layers, init_len,
                 stream)
{
  normals_tex_helper_ = new TwoDTextureHelper<float4>(1, stream);
}

TEMPLATE_TYPE
TEMPLATE_NAME::RacerDubinsElevationSuspensionImpl(std::string path, cudaStream_t stream) : PARENT_CLASS(path, stream)
{
  normals_tex_helper_ = new TwoDTextureHelper<float4>(1, stream);
}

TEMPLATE_TYPE
void TEMPLATE_NAME::bindToStream(cudaStream_t stream)
{
  normals_tex_helper_->bindToStream(stream);
  PARENT_CLASS::bindToStream(stream);
}

TEMPLATE_TYPE
void TEMPLATE_NAME::paramsToDevice()
{
  normals_tex_helper_->copyToDevice();
  PARENT_CLASS::paramsToDevice();
}

TEMPLATE_TYPE
void TEMPLATE_NAME::GPUSetup()
{
  PARENT_CLASS::GPUSetup();

  normals_tex_helper_->GPUSetup();
  // makes sure that the device ptr sees the correct texture object
  HANDLE_ERROR(cudaMemcpyAsync(&(this->model_d_->normals_tex_helper_), &(normals_tex_helper_->ptr_d_),
                               sizeof(TwoDTextureHelper<float4>*), cudaMemcpyHostToDevice, this->stream_));
}

TEMPLATE_TYPE
void TEMPLATE_NAME::freeCudaMem()
{
  normals_tex_helper_->freeCudaMem();
  PARENT_CLASS::freeCudaMem();
}

TEMPLATE_TYPE
void TEMPLATE_NAME::computeSimpleSuspensionStep(Eigen::Ref<state_array> state, Eigen::Ref<state_array> state_der,
                                                Eigen::Ref<output_array> output)
{
  DYN_PARAMS_T* params_p = &(this->params_);

  // Calculate suspension-based state derivatives
  const float& x = state(S_INDEX(POS_X));
  const float& y = state(S_INDEX(POS_Y));
  const float& roll = state(S_INDEX(ROLL));
  const float& pitch = state(S_INDEX(PITCH));
  const float& yaw = state(S_INDEX(YAW));
  float3 wheel_positions_body[W_INDEX(NUM_WHEELS)];
  float3 wheel_positions_world[W_INDEX(NUM_WHEELS)];
  float3 wheel_positions_cg[W_INDEX(NUM_WHEELS)];
  wheel_positions_body[W_INDEX(FR)] = make_float3(2.981f, -0.737f, 0.f);
  wheel_positions_body[W_INDEX(FL)] = make_float3(2.981f, 0.737f, 0.0f);
  wheel_positions_body[W_INDEX(BR)] = make_float3(0.0f, 0.737f, 0.0f);
  wheel_positions_body[W_INDEX(BL)] = make_float3(0.0f, -0.737f, 0.f);

  float3 body_pose = make_float3(x, y, 0.0f);
  // rotation matrix representation
  // float3 rotation = make_float3(roll, pitch, yaw);
  Eigen::Matrix3f M;
  mppi::math::Euler2DCM_NWU(roll, pitch, yaw, M);
  float wheel_pos_z, wheel_vel_z;
  float wheel_height = 0.0f;
  float4 wheel_normal_world = make_float4(0.0f, 0.0f, 1.0f, 0.0f);
  int pi, step;

  state_der(S_INDEX(ROLL)) = state(S_INDEX(ROLL_RATE));
  state_der(S_INDEX(PITCH)) = state(S_INDEX(PITCH_RATE));
  state_der(S_INDEX(CG_POS_Z)) = state(S_INDEX(CG_VEL_I_Z));
  state_der(S_INDEX(CG_VEL_I_Z)) = 0.0f;
  state_der(S_INDEX(ROLL_RATE)) = 0.0f;
  state_der(S_INDEX(PITCH_RATE)) = 0.0f;
  output(O_INDEX(WHEEL_FORCE_UP_MAX)) = -std::numeric_limits<float>::max();
  output(O_INDEX(WHEEL_FORCE_FWD_MAX)) = -std::numeric_limits<float>::max();
  output(O_INDEX(WHEEL_FORCE_SIDE_MAX)) = -std::numeric_limits<float>::max();
  mppi::p1::getParallel1DIndex<mppi::p1::Parallel1Dir::THREAD_Y>(pi, step);

  float wheel_yaw, sin_wheel_yaw, cos_wheel_yaw;
  for (int i = pi; i < W_INDEX(NUM_WHEELS); i += step)
  {
    // Calculate wheel position in different frames
    wheel_positions_cg[i] = wheel_positions_body[i] - params_p->c_g;
    mppi::math::bodyOffsetToWorldPoseDCM(wheel_positions_body[i], body_pose, M, wheel_positions_world[i]);
    if (this->tex_helper_->checkTextureUse(0))
    {
      wheel_height = this->tex_helper_->queryTextureAtWorldPose(0, wheel_positions_world[i]);
      if (!isfinite(wheel_height))
      {
        wheel_height = state(S_INDEX(CG_POS_Z)) - params_p->wheel_radius;
      }
    }
    if (normals_tex_helper_->checkTextureUse(0))
    {
      wheel_normal_world = normals_tex_helper_->queryTextureAtWorldPose(0, wheel_positions_world[i]);
      if (!isfinite(wheel_normal_world.x) || !isfinite(wheel_normal_world.y) || !isfinite(wheel_normal_world.z))
      {
        wheel_normal_world = make_float4(0.0, 0.0, 1.0, 0.0);
      }
    }

    if (i == W_INDEX(FR) || i == W_INDEX(FL))
    {
      wheel_yaw = yaw + S_INDEX(STEER_ANGLE) / -9.1f;
    }
    else
    {
      wheel_yaw = yaw;
    }
    sincosf(wheel_yaw, &sin_wheel_yaw, &cos_wheel_yaw);

    // Calculate wheel heights, velocities, and forces
    wheel_pos_z = state(S_INDEX(CG_POS_Z)) + roll * wheel_positions_cg[i].y - pitch * wheel_positions_cg[i].x -
                  params_p->wheel_radius;
    wheel_vel_z = state(S_INDEX(CG_VEL_I_Z)) + state(S_INDEX(ROLL_RATE)) * wheel_positions_cg[i].y -
                  state(S_INDEX(PITCH_RATE)) * wheel_positions_cg[i].x;

    // h_dot = - V_I * N_I
    float h_dot = -(state[S_INDEX(VEL_X)] * cos_wheel_yaw * wheel_normal_world.x +
                    state[S_INDEX(VEL_X)] * sin_wheel_yaw * wheel_normal_world.y);

    float wheel_force = -params_p->spring_k * (wheel_pos_z - wheel_height) - params_p->drag_c * (wheel_vel_z - h_dot);
    float up_wheel_force = wheel_force;  // + params_p->mass * (9.81f / 4.0f);
    float fwd_wheel_force =
        wheel_force / wheel_normal_world.z *
        (wheel_normal_world.x * cos_wheel_yaw + wheel_normal_world.y * sin_wheel_yaw + wheel_normal_world.z * (-pitch));
    float side_wheel_force =
        wheel_force / wheel_normal_world.z *
        (-wheel_normal_world.x * sin_wheel_yaw + wheel_normal_world.y * cos_wheel_yaw + wheel_normal_world.z * roll);
    output(O_INDEX(WHEEL_FORCE_UP_MAX)) = fmaxf(output(O_INDEX(WHEEL_FORCE_UP_MAX)), up_wheel_force);
    output(O_INDEX(WHEEL_FORCE_FWD_MAX)) = fmaxf(output(O_INDEX(WHEEL_FORCE_FWD_MAX)), fabsf(fwd_wheel_force));
    output(O_INDEX(WHEEL_FORCE_SIDE_MAX)) = fmaxf(output(O_INDEX(WHEEL_FORCE_SIDE_MAX)), fabsf(side_wheel_force));
    // output(O_INDEX(WHEEL_FORCE_UP_FL) + i) = up_wheel_force;
    // output(O_INDEX(WHEEL_FORCE_FWD_FL) + i) = fwd_wheel_force;
    // output(O_INDEX(WHEEL_FORCE_SIDE_FL) + i) = side_wheel_force;
    state_der(S_INDEX(CG_VEL_I_Z)) += wheel_force / params_p->mass;
    state_der(S_INDEX(ROLL_RATE)) += wheel_force * wheel_positions_cg[i].y / params_p->I_xx;
    state_der(S_INDEX(PITCH_RATE)) += -wheel_force * wheel_positions_cg[i].x / params_p->I_yy;
  }

  if (output(O_INDEX(WHEEL_FORCE_UP_MAX)) == 0.0f)
  {
    std::cout << "got state: " << state.transpose() << std::endl;
  }
}

TEMPLATE_TYPE
void TEMPLATE_NAME::step(Eigen::Ref<state_array> state, Eigen::Ref<state_array> next_state,
                         Eigen::Ref<state_array> state_der, const Eigen::Ref<const control_array>& control,
                         Eigen::Ref<output_array> output, const float t, const float dt)
{
  this->computeParametricDelayDeriv(state, control, state_der);
  this->computeParametricAccelDeriv(state, control, state_der, dt);
  this->computeLSTMSteering(state, control, state_der);

  computeSimpleSuspensionStep(state, state_der, output);

  // Integrate using Euler Integration
  updateState(state, next_state, state_der, dt);
  SharedBlock sb;
  computeUncertaintyPropagation(state.data(), control.data(), state_der.data(), next_state.data(), dt, &this->params_,
                                &sb, nullptr);

  // float roll = state(S_INDEX(ROLL));
  // float pitch = state(S_INDEX(PITCH));
  // RACER::computeStaticSettling<typename DYN_PARAMS_T::OutputIndex, TwoDTextureHelper<float>>(
  //     this->tex_helper_, next_state(S_INDEX(YAW)), next_state(S_INDEX(POS_X)), next_state(S_INDEX(POS_Y)), roll,
  //     pitch, output.data());
  // next_state[S_INDEX(PITCH)] = pitch;
  // next_state[S_INDEX(ROLL)] = roll;

  this->setOutputs(state_der.data(), next_state.data(), output.data());
  // printf("CPU t: %3.0f, VEL_Z(t + 1): %f, VEL_Z(t): %f, VEl_Z'(t): %f\n", t, next_state(S_INDEX(CG_VEL_I_Z)),
  // state(S_INDEX(CG_VEL_I_Z)),
  //     state_der(S_INDEX(CG_VEL_I_Z)));
}

TEMPLATE_TYPE
__device__ void TEMPLATE_NAME::computeSimpleSuspensionStep(float* state, float* state_der, float* output,
                                                           DYN_PARAMS_T* params_p, float* theta_s)
{
  // computes the velocity dot
  int pi, step;
  mppi::p1::getParallel1DIndex<mppi::p1::Parallel1Dir::THREAD_Y>(pi, step);

  if (pi == 0)
  {
    state_der[S_INDEX(CG_VEL_I_Z)] = 0.0f;
    state_der[S_INDEX(ROLL_RATE)] = 0.0f;
    state_der[S_INDEX(PITCH_RATE)] = 0.0f;
    state_der[S_INDEX(ROLL)] = state[S_INDEX(ROLL_RATE)];
    state_der[S_INDEX(PITCH)] = state[S_INDEX(PITCH_RATE)];
    state_der[S_INDEX(CG_POS_Z)] = state[S_INDEX(CG_VEL_I_Z)];
    output[O_INDEX(WHEEL_FORCE_UP_MAX)] = 0.0f;
    output[O_INDEX(WHEEL_FORCE_FWD_MAX)] = 0.0f;
    output[O_INDEX(WHEEL_FORCE_SIDE_MAX)] = 0.0f;
  }
  const int grd_shift = this->SHARED_MEM_REQUEST_GRD_BYTES / sizeof(float);  // grid size to shift by
  const int blk_shift = this->SHARED_MEM_REQUEST_BLK_BYTES * (threadIdx.x + blockDim.x * threadIdx.z) /
                        sizeof(float);  // blk size to shift by
  // uses same memory as compute space for the uncertainty
  float* wheel_force_up = theta_s + grd_shift + blk_shift;
  float* wheel_force_fwd = theta_s + grd_shift + blk_shift + 4;
  float* wheel_force_side = theta_s + grd_shift + blk_shift + 8;
  __syncthreads();

  // Calculate suspension-based state derivatives
  const float& x = state[S_INDEX(POS_X)];
  const float& y = state[S_INDEX(POS_Y)];
  const float& roll = state[S_INDEX(ROLL)];
  const float& pitch = state[S_INDEX(PITCH)];
  const float& yaw = state[S_INDEX(YAW)];
  float3 wheel_positions_body;
  float3 wheel_positions_world;

  float3 body_pose = make_float3(x, y, 0.0f);
  float3 rotation = make_float3(roll, pitch, yaw);
  // rotation matrix representation
  // TODO Check if M needs to be in shared memory
  float M[3][3];
  mppi::math::Euler2DCM_NWU(roll, pitch, yaw, M);
  // mppi::math::Euler2DCM_NWU(rotation.x, rotation.y, rotation.z, M);
  float wheel_pos_z, wheel_vel_z;
  float wheel_height = 0.0f;
  float h_dot = 0.0f;
  float4 wheel_normal_world = make_float4(0.0f, 0.0f, 1.0f, 0.0f);
  float3 wheel_positions_cg;
  float wheel_yaw, cos_wheel_yaw, sin_wheel_yaw;

  __syncthreads();
  for (int i = pi; i < W_INDEX(NUM_WHEELS); i += step)
  {
    wheel_yaw = yaw;
    // get body frame wheel positions
    switch (i)
    {
      case W_INDEX(FR):
        wheel_positions_body = make_float3(2.981f, -0.737f, 0.f);
        wheel_yaw += S_INDEX(STEER_ANGLE) / -9.1f;
        break;
      case W_INDEX(FL):
        wheel_positions_body = make_float3(2.981f, 0.737f, 0.0f);
        wheel_yaw += S_INDEX(STEER_ANGLE) / -9.1f;
        break;
      case W_INDEX(BR):
        wheel_positions_body = make_float3(0.0f, 0.737f, 0.0f);
        break;
      case W_INDEX(BL):
        wheel_positions_body = make_float3(0.0f, -0.737f, 0.f);
        break;
      default:
        break;
    }
    __sincosf(wheel_yaw, &sin_wheel_yaw, &cos_wheel_yaw);

    // Calculate wheel position in different frames
    wheel_positions_cg = wheel_positions_body - params_p->c_g;
    mppi::math::bodyOffsetToWorldPoseDCM(wheel_positions_body, body_pose, M, wheel_positions_world);
    if (this->tex_helper_->checkTextureUse(0))
    {
      wheel_height = this->tex_helper_->queryTextureAtWorldPose(0, wheel_positions_world);
      if (!isfinite(wheel_height))
      {
        wheel_height = state[S_INDEX(CG_POS_Z)] - params_p->wheel_radius;
      }
    }
    if (normals_tex_helper_->checkTextureUse(0))
    {
      wheel_normal_world = normals_tex_helper_->queryTextureAtWorldPose(0, wheel_positions_world);
      if (!isfinite(wheel_normal_world.x) || !isfinite(wheel_normal_world.y) || !isfinite(wheel_normal_world.z))
      {
        wheel_normal_world = make_float4(0.0, 0.0, 1.0, 0.0);
      }
    }

    // Calculate wheel heights, velocities, and forces
    wheel_pos_z =
        state[S_INDEX(CG_POS_Z)] + roll * wheel_positions_cg.y - pitch * wheel_positions_cg.x - params_p->wheel_radius;
    wheel_vel_z = state[S_INDEX(CG_VEL_I_Z)] + state[S_INDEX(ROLL_RATE)] * wheel_positions_cg.y -
                  state[S_INDEX(PITCH_RATE)] * wheel_positions_cg.x;

    // h_dot = - V_I * N_I
    h_dot = -(state[S_INDEX(VEL_X)] * cos_wheel_yaw * wheel_normal_world.x +
              state[S_INDEX(VEL_X)] * sin_wheel_yaw * wheel_normal_world.y);

    float wheel_force = -params_p->spring_k * (wheel_pos_z - wheel_height) - params_p->drag_c * (wheel_vel_z - h_dot);
    float up_wheel_force = wheel_force;  // + params_p->mass * (9.81f / 4.0f);
    float fwd_wheel_force =
        wheel_force / wheel_normal_world.z *
        (wheel_normal_world.x * cos_wheel_yaw + wheel_normal_world.y * sin_wheel_yaw + wheel_normal_world.z * (-pitch));
    float side_wheel_force =
        wheel_force / wheel_normal_world.z *
        (-wheel_normal_world.x * sin_wheel_yaw + wheel_normal_world.y * cos_wheel_yaw + wheel_normal_world.z * roll);
    wheel_force_up[i] = up_wheel_force;
    wheel_force_fwd[i] = fabsf(fwd_wheel_force);
    wheel_force_side[i] = fabsf(side_wheel_force);
    // output[O_INDEX(WHEEL_FORCE_UP_FL) + i] = up_wheel_force;
    // output[O_INDEX(WHEEL_FORCE_FWD_FL) + i] = fwd_wheel_force;
    // output[O_INDEX(WHEEL_FORCE_SIDE_FL) + i] = side_wheel_force;
#if defined(__CUDA_ARCH__) && __CUDA_ARCH__ > 600
    // block-specific atomicAdd is only available on compute capabilities 6.x and greater
    atomicAdd_block(&state_der[S_INDEX(CG_VEL_I_Z)], wheel_force / params_p->mass);
    atomicAdd_block(&state_der[S_INDEX(ROLL_RATE)], wheel_force * wheel_positions_cg.y / params_p->I_xx);
    atomicAdd_block(&state_der[S_INDEX(PITCH_RATE)], -wheel_force * wheel_positions_cg.x / params_p->I_yy);
#elif defined(__CUDA_ARCH__)
    atomicAdd(&state_der[S_INDEX(CG_VEL_I_Z)], wheel_force / params_p->mass);
    atomicAdd(&state_der[S_INDEX(ROLL_RATE)], wheel_force * wheel_positions_cg.y / params_p->I_xx);
    atomicAdd(&state_der[S_INDEX(PITCH_RATE)], -wheel_force * wheel_positions_cg.x / params_p->I_yy);
#endif
  }
  __syncthreads();

  output[O_INDEX(WHEEL_FORCE_UP_MAX)] =
      fmaxf(wheel_force_up[0], fmaxf(wheel_force_up[1], fmaxf(wheel_force_up[2], wheel_force_up[3])));
  output[O_INDEX(WHEEL_FORCE_FWD_MAX)] =
      fmaxf(wheel_force_fwd[0], fmaxf(wheel_force_fwd[1], fmaxf(wheel_force_fwd[2], wheel_force_fwd[3])));
  output[O_INDEX(WHEEL_FORCE_SIDE_MAX)] =
      fmaxf(wheel_force_side[0], fmaxf(wheel_force_side[1], fmaxf(wheel_force_side[2], wheel_force_side[3])));
}

TEMPLATE_TYPE
__device__ void TEMPLATE_NAME::step(float* state, float* next_state, float* state_der, float* control, float* output,
                                    float* theta_s, const float t, const float dt)
{
  DYN_PARAMS_T* params_p;
  SharedBlock* sb;
  // if (GRANDPARENT_CLASS::SHARED_MEM_REQUEST_GRD_BYTES != 0)
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
  computeSimpleSuspensionStep(state, state_der, output, params_p, theta_s);

  updateState(state, next_state, state_der, dt);
  computeUncertaintyPropagation(state, control, state_der, next_state, dt, params_p, sb, theta_s);
  // if (pi == 0)
  // {
  //   float roll = state[S_INDEX(ROLL)];
  //   float pitch = state[S_INDEX(PITCH)];
  //   RACER::computeStaticSettling<DYN_PARAMS_T::OutputIndex, TwoDTextureHelper<float>>(
  //       this->tex_helper_, next_state[S_INDEX(YAW)], next_state[S_INDEX(POS_X)], next_state[S_INDEX(POS_Y)], roll,
  //       pitch, output);
  //   next_state[S_INDEX(PITCH)] = pitch;
  //   next_state[S_INDEX(ROLL)] = roll;
  // }
  __syncthreads();
  // if (threadIdx.x == 0 && blockIdx.x == 0 && threadIdx.y == 0)
  // {
  //   printf("GPU t: %3d, VEL_Z(t + 1): %f, VEL_Z(t): %f, VEl_Z'(t): %f\n", t, next_state[S_INDEX(CG_VEL_I_Z)],
  //   state[S_INDEX(CG_VEL_I_Z)],
  //       state_der[S_INDEX(CG_VEL_I_Z)]);
  // }
  this->setOutputs(state_der, next_state, output);
}

TEMPLATE_TYPE
__device__ void TEMPLATE_NAME::updateState(float* state, float* next_state, float* state_der, const float dt)
{
  int i;
  int tdy, step;
  mppi::p1::getParallel1DIndex<mppi::p1::Parallel1Dir::THREAD_Y>(tdy, step);
  // Add the state derivative time dt to the current state.
  for (i = tdy; i < S_INDEX(STEER_ANGLE_RATE); i += step)
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
  for (int i = 0; i < S_INDEX(STEER_ANGLE_RATE); i++)
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

TEMPLATE_TYPE
__host__ __device__ void TEMPLATE_NAME::setOutputs(const float* state_der, const float* next_state, float* output)
{
  // Setup output

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
      case O_INDEX(BASELINK_POS_I_Z):
        output[i] = next_state[S_INDEX(CG_POS_Z)] - next_state[S_INDEX(PITCH)] * (-this->params_.c_g.x);
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

TEMPLATE_TYPE
TEMPLATE_NAME::state_array TEMPLATE_NAME::stateFromMap(const std::map<std::string, float>& map)
{
  std::vector<std::string> needed_keys = { "VEL_X",      "VEL_Z", "POS_X", "POS_Y", "POS_Z",       "OMEGA_X",
                                           "OMEGA_Y",    "ROLL",  "PITCH", "YAW",   "STEER_ANGLE", "STEER_ANGLE_RATE",
                                           "BRAKE_STATE" };
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
#undef TEMPLATE_NAME
#undef TEMPLATE_TYPE
