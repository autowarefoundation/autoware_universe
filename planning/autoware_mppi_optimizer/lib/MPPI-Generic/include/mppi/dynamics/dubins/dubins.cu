#include <mppi/dynamics/dubins/dubins.cuh>

DubinsDynamics::DubinsDynamics(cudaStream_t stream) : Dynamics<DubinsDynamics, DubinsParams>(stream)
{
  this->params_ = DubinsParams();
}

void DubinsDynamics::computeDynamics(const Eigen::Ref<const state_array>& state,
                                     const Eigen::Ref<const control_array>& control, Eigen::Ref<state_array> state_der)
{
  state_der(S_INDEX(POS_X)) = control(C_INDEX(VEL)) * cos(state(S_INDEX(YAW)));
  state_der(S_INDEX(POS_Y)) = control(C_INDEX(VEL)) * sin(state(S_INDEX(YAW)));
  state_der(S_INDEX(YAW)) = control(C_INDEX(YAW_RATE));
}

bool DubinsDynamics::computeGrad(const Eigen::Ref<const state_array>& state,
                                 const Eigen::Ref<const control_array>& control, Eigen::Ref<dfdx> A, Eigen::Ref<dfdu> B)
{
  A(0, 2) = -control(C_INDEX(VEL)) * sin(state(S_INDEX(YAW)));
  A(1, 2) = control(C_INDEX(VEL)) * cos(state(S_INDEX(YAW)));

  B(0, 0) = cos(state(S_INDEX(YAW)));
  B(1, 0) = sin(state(S_INDEX(YAW)));
  B(2, 1) = 1;
  return true;
}

void DubinsDynamics::updateState(const Eigen::Ref<const state_array> state, Eigen::Ref<state_array> next_state,
                                 Eigen::Ref<state_array> state_der, const float dt)
{
  next_state = state + state_der * dt;
  next_state(S_INDEX(YAW)) = angle_utils::normalizeAngle(next_state(S_INDEX(YAW)));
}

DubinsDynamics::state_array DubinsDynamics::interpolateState(const Eigen::Ref<state_array> state_1,
                                                             const Eigen::Ref<state_array> state_2, const float alpha)
{
  state_array result = (1 - alpha) * state_1 + alpha * state_2;
  result(2) = angle_utils::interpolateEulerAngleLinear(state_1(2), state_2(2), alpha);
  return result;
}

__device__ void DubinsDynamics::updateState(float* state, float* next_state, float* state_der, const float dt)
{
  int i;
  int tdy = threadIdx.y;
  // Add the state derivative time dt to the current state.
  // printf("updateState thread %d, %d = %f, %f\n", threadIdx.x, threadIdx.y, state[S_INDEX(POS_X)],
  // state_der[S_INDEX(POS_X)]);
  for (i = tdy; i < STATE_DIM; i += blockDim.y)
  {
    next_state[i] = state[i] + state_der[i] * dt;
    if (i == S_INDEX(YAW))
    {
      next_state[i] = angle_utils::normalizeAngle(next_state[i]);
    }
  }
}

__device__ void DubinsDynamics::computeDynamics(float* state, float* control, float* state_der, float* theta_s)
{
  state_der[S_INDEX(POS_X)] = control[C_INDEX(VEL)] * cos(state[S_INDEX(YAW)]);
  state_der[S_INDEX(POS_Y)] = control[C_INDEX(VEL)] * sin(state[S_INDEX(YAW)]);
  state_der[S_INDEX(YAW)] = control[C_INDEX(YAW_RATE)];
}

Dynamics<DubinsDynamics, DubinsParams>::state_array
DubinsDynamics::stateFromMap(const std::map<std::string, float>& map)
{
  state_array s;
  s(S_INDEX(POS_X)) = map.at("POS_X");
  s(S_INDEX(POS_Y)) = map.at("POS_Y");
  s(S_INDEX(YAW)) = map.at("YAW");
  return s;
}
