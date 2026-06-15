#include <mppi/dynamics/double_integrator/di_dynamics.cuh>

DoubleIntegratorDynamics::DoubleIntegratorDynamics(float system_noise, cudaStream_t stream)
  : Dynamics<DoubleIntegratorDynamics, DoubleIntegratorParams>(stream)
{
  this->params_ = DoubleIntegratorParams(system_noise);

  // Seed the RNG and initialize the system noise distribution
  std::random_device rd;
  gen.seed(rd());  // Seed the RNG with a random number
  setStateVariance(system_noise);
}

void DoubleIntegratorDynamics::computeDynamics(const Eigen::Ref<const state_array>& state,
                                               const Eigen::Ref<const control_array>& control,
                                               Eigen::Ref<state_array> state_der)
{
  state_der(0) = state(2);    // xdot;
  state_der(1) = state(3);    // ydot;
  state_der(2) = control(0);  // x_force;
  state_der(3) = control(1);  // y_force
}

bool DoubleIntegratorDynamics::computeGrad(const Eigen::Ref<const state_array>& state,
                                           const Eigen::Ref<const control_array>& control, Eigen::Ref<dfdx> A,
                                           Eigen::Ref<dfdu> B)
{
  A(0, 2) = 1;
  A(1, 3) = 1;

  B(2, 0) = 1;
  B(3, 1) = 1;
  return true;
}

void DoubleIntegratorDynamics::printState(float* state)
{
  printf("X position: %f; Y position: %f; X velocity: %f; Y velocity: %f \n", state[0], state[1], state[2], state[3]);
}

void DoubleIntegratorDynamics::printState(const float* state)
{
  printf("X position: %f; Y position: %f; X velocity: %f; Y velocity: %f \n", state[0], state[1], state[2], state[3]);
}

__device__ void DoubleIntegratorDynamics::computeDynamics(float* state, float* control, float* state_der,
                                                          float* theta_s)
{
  state_der[0] = state[2];    // xdot;
  state_der[1] = state[3];    // ydot;
  state_der[2] = control[0];  // x_force;
  state_der[3] = control[1];  // y_force
}

void DoubleIntegratorDynamics::setStateVariance(float system_variance)
{
  normal_distribution = std::normal_distribution<float>(0, sqrtf(system_variance));
}

void DoubleIntegratorDynamics::computeStateDisturbance(float dt, Eigen::Ref<state_array> state)
{
  // Generate system noise
  state_array system_noise;
  system_noise << 0.0, 0.0, normal_distribution(gen), normal_distribution(gen);
  state += system_noise * dt;
}

DoubleIntegratorDynamics::dfdu DoubleIntegratorDynamics::B(const Eigen::Ref<const state_array>& state)
{
  dfdu B = dfdu::Zero();
  B(2, 0) = 1;
  B(3, 1) = 1;
  return B;
}

Dynamics<DoubleIntegratorDynamics, DoubleIntegratorParams>::state_array
DoubleIntegratorDynamics::stateFromMap(const std::map<std::string, float>& map)
{
  state_array s;
  s(0) = map.at("POS_X");
  s(1) = map.at("POS_Y");
  s(2) = map.at("VEL_X");
  s(3) = map.at("VEL_Y");
  return s;
}
