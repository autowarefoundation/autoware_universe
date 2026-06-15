#include <mppi/dynamics/cartpole/cartpole_dynamics.cuh>
#include <mppi/utils/math_utils.h>

CartpoleDynamics::CartpoleDynamics(float cart_mass, float pole_mass, float pole_length, cudaStream_t stream)
  : Dynamics<CartpoleDynamics, CartpoleDynamicsParams>(stream)
{
  this->params_ = CartpoleDynamicsParams(cart_mass, pole_mass, pole_length);
}

bool CartpoleDynamics::computeGrad(const Eigen::Ref<const state_array>& state,
                                   const Eigen::Ref<const control_array>& control, Eigen::Ref<dfdx> A,
                                   Eigen::Ref<dfdu> B)
{
  float theta = state(2);
  float theta_dot = state(3);
  float force = control(0);

  A(0, 1) = 1.0;
  A(1, 2) =
      (this->params_.pole_mass * cosf(theta) * (this->params_.pole_length * SQ(theta_dot) + gravity_ * cosf(theta)) -
       gravity_ * this->params_.pole_mass * SQ(sinf(theta))) /
          (this->params_.cart_mass + this->params_.pole_mass * SQ(sinf(theta))) -
      (2 * this->params_.pole_mass * cosf(theta) * sinf(theta) *
       (force +
        this->params_.pole_mass * sinf(theta) * (this->params_.pole_length * SQ(theta_dot) + gravity_ * cosf(theta)))) /
          powf((this->params_.cart_mass + this->params_.pole_mass * SQ(sinf(theta))), 2.0);
  A(1, 3) = (2 * this->params_.pole_length * this->params_.pole_mass * theta_dot * sinf(theta)) /
            (this->params_.cart_mass + this->params_.pole_mass * SQ(sinf(theta)));
  A(2, 3) = 1.0;
  A(3, 2) =
      (force * sinf(theta) - gravity_ * cosf(theta) * (this->params_.pole_mass + this->params_.cart_mass) -
       this->params_.pole_length * this->params_.pole_mass * SQ(theta_dot) * SQ(cosf(theta)) +
       this->params_.pole_length * this->params_.pole_mass * SQ(theta_dot) * SQ(sinf(theta))) /
          (this->params_.pole_length * (this->params_.cart_mass + this->params_.pole_mass * SQ(sinf(theta)))) +
      (2 * this->params_.pole_mass * cosf(theta) * sinf(theta) *
       (this->params_.pole_length * this->params_.pole_mass * cosf(theta) * sinf(theta) * SQ(theta_dot) +
        force * cosf(theta) + gravity_ * sinf(theta) * (this->params_.pole_mass + this->params_.cart_mass))) /
          powf(this->params_.pole_length * (this->params_.cart_mass + this->params_.pole_mass * SQ(sinf(theta))), 2.0);
  A(3, 3) = -(2 * this->params_.pole_mass * theta_dot * cosf(theta) * sinf(theta)) /
            (this->params_.cart_mass + this->params_.pole_mass * SQ(sinf(theta)));

  B(1, 0) = 1 / (this->params_.cart_mass + this->params_.pole_mass * SQ(sinf(theta)));
  B(3, 0) = -cosf(theta) /
            (this->params_.pole_length * (this->params_.cart_mass + this->params_.pole_mass * SQ(sinf(theta))));
  return true;
}

void CartpoleDynamics::computeDynamics(const Eigen::Ref<const state_array>& state,
                                       const Eigen::Ref<const control_array>& control,
                                       Eigen::Ref<state_array> state_der)
{
  const float sin_theta = sinf(state(S_INDEX(THETA)));
  const float cos_theta = cosf(state(S_INDEX(THETA)));
  float theta_dot = state[S_INDEX(THETA_DOT)];
  float force = control[C_INDEX(FORCE)];
  const float m_c = this->params_.cart_mass;
  const float m_p = this->params_.pole_mass;
  const float l_p = this->params_.pole_length;

  state_der(S_INDEX(POS_X)) = state(S_INDEX(VEL_X));
  state_der(S_INDEX(VEL_X)) =
      (force + m_p * sin_theta * (l_p * SQ(theta_dot) + gravity_ * cos_theta)) / (m_c + m_p * SQ(sin_theta));
  state_der(S_INDEX(THETA)) = theta_dot;
  state_der(S_INDEX(THETA_DOT)) =
      (-force * cos_theta - m_p * l_p * SQ(theta_dot) * cos_theta * sin_theta - (m_c + m_p) * gravity_ * sin_theta) /
      (l_p * (m_c + m_p * SQ(sin_theta)));
}

void CartpoleDynamics::printState(const Eigen::Ref<const state_array>& state)
{
  printf("Cart position: %f; Cart velocity: %f; Pole angle: %f; Pole rate: %f \n", state(0), state(1), state(2),
         state(3));  // Needs to be completed
}

void CartpoleDynamics::printState(float* state)
{
  printf("Cart position: %f; Cart velocity: %f; Pole angle: %f; Pole rate: %f \n", state[0], state[1], state[2],
         state[3]);  // Needs to be completed
}

void CartpoleDynamics::printParams()
{
  printf("Cart mass: %f; Pole mass: %f; Pole length: %f \n", this->params_.cart_mass, this->params_.pole_mass,
         this->params_.pole_length);
}

__device__ void CartpoleDynamics::computeDynamics(float* state, float* control, float* state_der, float* theta_s)
{
  float theta = angle_utils::normalizeAngle(state[S_INDEX(THETA)]);
  const float sin_theta = __sinf(theta);
  const float cos_theta = __cosf(theta);
  float theta_dot = state[S_INDEX(THETA_DOT)];
  float force = control[C_INDEX(FORCE)];
  const float m_c = this->params_.cart_mass;
  const float m_p = this->params_.pole_mass;
  const float l_p = this->params_.pole_length;

  state_der[S_INDEX(POS_X)] = state[S_INDEX(VEL_X)];
  state_der[S_INDEX(VEL_X)] =
      (force + m_p * sin_theta * (l_p * SQ(theta_dot) + gravity_ * cos_theta)) / (m_c + m_p * SQ(sin_theta));
  state_der[S_INDEX(THETA)] = theta_dot;
  state_der[S_INDEX(THETA_DOT)] =
      (-force * cos_theta - m_p * l_p * SQ(theta_dot) * cos_theta * sin_theta - (m_c + m_p) * gravity_ * sin_theta) /
      (l_p * (m_c + m_p * SQ(sin_theta)));
}

Dynamics<CartpoleDynamics, CartpoleDynamicsParams>::state_array
CartpoleDynamics::stateFromMap(const std::map<std::string, float>& map)
{
  state_array s;
  s(S_INDEX(POS_X)) = map.at("POS_X");
  s(S_INDEX(VEL_X)) = map.at("VEL_X");
  s(S_INDEX(THETA)) = map.at("THETA");
  s(S_INDEX(THETA_DOT)) = map.at("THETA_DOT");
  return s;
}
