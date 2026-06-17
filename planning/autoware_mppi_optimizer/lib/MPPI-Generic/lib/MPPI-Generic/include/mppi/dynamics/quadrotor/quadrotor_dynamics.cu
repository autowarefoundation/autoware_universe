#include <mppi/dynamics/quadrotor/quadrotor_dynamics.cuh>
#include <mppi/utils/math_utils.h>

QuadrotorDynamics::QuadrotorDynamics(std::array<float2, CONTROL_DIM> control_rngs, cudaStream_t stream)
  : PARENT_CLASS(control_rngs, stream)
{
  this->params_ = QuadrotorDynamicsParams();
  zero_control_[3] = mppi::math::GRAVITY;
}

QuadrotorDynamics::QuadrotorDynamics(cudaStream_t stream) : PARENT_CLASS(stream)
{
  this->params_ = QuadrotorDynamicsParams();
  float2 thrust_rng;
  thrust_rng.x = 0;
  thrust_rng.y = 36;  // TODO Figure out if this is a reasonable amount of thrust
  this->control_rngs_[3] = thrust_rng;
  this->zero_control_[3] = mppi::math::GRAVITY;
}

void QuadrotorDynamics::printState(float* state)
{
  int precision = 4;
  int total_char = precision + 4;
  printf("Pos     x: %*.*f, y: %*.*f, z: %*.*f\n", total_char, precision, state[0], total_char, precision, state[1],
         total_char, precision, state[2]);
  printf("Vel     x: %*.*f, y: %*.*f, z: %*.*f\n", total_char, precision, state[3], total_char, precision, state[4],
         total_char, precision, state[5]);
  printf("Quat    w: %*.*f, x: %*.*f, y: %*.*f, z: %*.*f\n", total_char, precision, state[6], total_char, precision,
         state[7], total_char, precision, state[8], total_char, precision, state[9]);
  printf("Ang Vel x: %*.*f, y: %*.*f, z: %*.*f\n", total_char, precision, state[10], total_char, precision, state[11],
         total_char, precision, state[12]);
}

bool QuadrotorDynamics::computeGrad(const Eigen::Ref<const state_array>& state,
                                    const Eigen::Ref<const control_array>& control, Eigen::Ref<dfdx> A,
                                    Eigen::Ref<dfdu> B)
{
  Eigen::Quaternionf q(state[6], state[7], state[8], state[9]);
  Eigen::Matrix3f dcm_lb;
  // dcm_lb = q.toRotationMatrix();
  mppi::math::Quat2DCM(q, dcm_lb);

  // I can do the math for everything except quaternions
  A.setZero();
  // x_d
  A.block<3, 3>(0, 3).setIdentity();
  // v_d TODO figure out derivative of v_d wrt q

  // q_d TODO figure out derivative of q_d wrt q

  // w_d
  Eigen::Vector3f tau_inv;
  tau_inv << 1 / this->params_.tau_roll, 1 / this->params_.tau_pitch, 1 / this->params_.tau_yaw;
  A.block<3, 3>(10, 10) = -1 * tau_inv.asDiagonal();

  B.setZero();
  // x_d is empty
  // v_d
  B.block<3, 1>(3, 3) = dcm_lb.col(2) / this->params_.mass;
  // q_d using omega2edot as reference
  B.block<4, 3>(6, 0) << -0.5 * q.x(), -0.5 * q.y(), -0.5 * q.z(), 0.5 * q.w(), -0.5 * q.z(), 0.5 * q.y(), 0.5 * q.z(),
      0.5 * q.w(), -0.5 * q.x(), -0.5 * q.y(), 0.5 * q.x(), 0.5 * q.w();

  // w_d
  B.block<3, 3>(10, 0) = tau_inv.asDiagonal();
  return false;
}

void QuadrotorDynamics::computeDynamics(const Eigen::Ref<const state_array>& state,
                                        const Eigen::Ref<const control_array>& control,
                                        Eigen::Ref<state_array> state_der)
{
  // Fill in
  state_der.block<3, 1>(0, 0);
  Eigen::Vector3f x_d, v_d, angular_speed_d, u_pqr;
  Eigen::Matrix<float, 3, 1> angular_speed, v;
  Eigen::Quaternionf q_d;
  Eigen::Matrix<float, 3, 1> tau_inv;
  float u_thrust = control[C_INDEX(THRUST)];

  // Assume quaterion is w, x, y, z in state array
  Eigen::Quaternionf q(state[6], state[7], state[8], state[9]);
  u_pqr << control[C_INDEX(ANG_RATE_X)], control[C_INDEX(ANG_RATE_Y)], control[C_INDEX(ANG_RATE_Z)];
  v = state.block<3, 1>(3, 0);
  angular_speed << state(10), state(11), state(12);
  tau_inv << 1 / this->params_.tau_roll, 1 / this->params_.tau_pitch, 1 / this->params_.tau_yaw;

  Eigen::Matrix3f dcm_lb = Eigen::Matrix3f::Identity();

  // x_d = v
  x_d = v;

  // v_d = Lvb * [0 0 T]' + g
  mppi::math::Quat2DCM(q, dcm_lb);
  v_d = (u_thrust / this->params_.mass) * dcm_lb.col(2);
  v_d(2) -= mppi::math::GRAVITY;

  // q_d = H(q) w
  mppi::math::omega2edot(angular_speed(0), angular_speed(1), angular_speed(2), q, q_d);

  // w_d = (u_pqr - w)/ tau
  // Note we assume that a low level controller makes angular velocity tracking
  // a first order system
  angular_speed_d = tau_inv.cwiseProduct(u_pqr - angular_speed);

  // Copy into state_deriv
  state_der.block<3, 1>(0, 0) = x_d;
  state_der.block<3, 1>(3, 0) = v_d;
  state_der(6) = q_d.w();
  state_der(7) = q_d.x();
  state_der(8) = q_d.y();
  state_der(9) = q_d.z();
  state_der.block<3, 1>(10, 0) = angular_speed_d;
}

void QuadrotorDynamics::updateState(const Eigen::Ref<const state_array> state, Eigen::Ref<state_array> next_state,
                                    Eigen::Ref<state_array> state_der, const float dt)
{
  PARENT_CLASS::updateState(state, next_state, state_der, dt);

  // Renormalize quaternion
  Eigen::Quaternionf q(next_state[6], next_state[7], next_state[8], next_state[9]);
  next_state.block<4, 1>(6, 0) /= q.norm() * copysign(1.0, q.w());
}

__device__ void QuadrotorDynamics::computeDynamics(float* state, float* control, float* state_der, float* theta)
{
  //  Fill in
  float* v = state + 3;
  float* q = state + 6;
  float* w = state + 10;

  // Derivatives
  float* x_d = state_der;
  float* v_d = state_der + 3;
  float* q_d = state_der + 6;
  float* w_d = state_der + 10;

  float* u_pqr = control;
  float u_thrust = control[C_INDEX(THRUST)];

  float dcm_lb[3][3];

  int i;

  // x_d = v
  for (i = threadIdx.y; i < 3; i += blockDim.y)
  {
    x_d[i] = v[i];
  }

  // v_d = Lvb * [0 0 T]' + g
  mppi::math::Quat2DCM(q, dcm_lb);
  for (i = threadIdx.y; i < 3; i += blockDim.y)
  {
    v_d[i] = (u_thrust / this->params_.mass) * dcm_lb[i][2];
  }
  __syncthreads();
  if (threadIdx.y == 0)
  {
    v_d[2] -= mppi::math::GRAVITY;
  }

  // q_d = H(q) w
  mppi::math::omega2edot(w[0], w[1], w[2], q, q_d);

  // w_d = (u - w) / tau
  w_d[0] = (u_pqr[0] - w[0]) / this->params_.tau_roll;
  w_d[1] = (u_pqr[1] - w[1]) / this->params_.tau_pitch;
  w_d[2] = (u_pqr[2] - w[2]) / this->params_.tau_yaw;
  __syncthreads();
}

__device__ void QuadrotorDynamics::updateState(float* state, float* next_state, float* state_der, float dt)
{
  PARENT_CLASS::updateState(state, next_state, state_der, dt);

  int i = 0;
  // renormalze quaternion
  float* q = next_state + 6;
  float q_norm = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  for (i = threadIdx.y; i < 4; i += blockDim.y)
  {
    q[i] /= q_norm * copysignf(1.0, q[0]);
  }
  // __syncthreads();
}

QuadrotorDynamics::state_array QuadrotorDynamics::stateFromMap(const std::map<std::string, float>& map)
{
  state_array s;
  s(S_INDEX(POS_X)) = map.at("POS_X");
  s(S_INDEX(POS_Y)) = map.at("POS_Y");
  s(S_INDEX(POS_Z)) = map.at("POS_Z");

  s(S_INDEX(VEL_X)) = map.at("VEL_X");
  s(S_INDEX(VEL_Y)) = map.at("VEL_Y");
  s(S_INDEX(VEL_Z)) = map.at("VEL_Z");

  s(S_INDEX(QUAT_X)) = map.at("Q_X");
  s(S_INDEX(QUAT_Y)) = map.at("Q_Y");
  s(S_INDEX(QUAT_Z)) = map.at("Q_Z");
  s(S_INDEX(QUAT_W)) = map.at("Q_W");

  s(S_INDEX(ANG_VEL_X)) = map.at("OMEGA_X");
  s(S_INDEX(ANG_VEL_Y)) = map.at("OMEGA_Y");
  s(S_INDEX(ANG_VEL_Z)) = map.at("OMEGA_Z");
  return QuadrotorDynamics::state_array();
}

QuadrotorDynamics::state_array QuadrotorDynamics::getZeroState() const
{
  state_array zero = state_array::Zero();
  zero[S_INDEX(QUAT_W)] = 1.0f;
  return zero;
}

__host__ __device__ void QuadrotorDynamics::getZeroState(float* state) const
{
  int p_index, step;
  mppi::p1::getParallel1DIndex<mppi::p1::Parallel1Dir::THREAD_Y>(p_index, step);
  for (int i = p_index; i < STATE_DIM; i += step)
  {
    if (i == S_INDEX(QUAT_W))
    {
      state[i] = 1.0f;
    }
    else
    {
      state[i] = 0.0f;
    }
  }
}
