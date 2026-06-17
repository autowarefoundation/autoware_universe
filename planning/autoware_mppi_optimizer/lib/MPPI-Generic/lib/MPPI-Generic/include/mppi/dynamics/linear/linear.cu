/**
 * @file linear.cu
 * @brief Linear Dynamics
 * @author Bogdan Vlahov
 * @version
 * @date 2024-08-16
 */
#include <mppi/dynamics/racer_dubins/racer_dubins.cuh>
#include <string>
#include <mppi/utils/math_utils.h>

namespace mm = mppi::matrix_multiplication;
namespace mp1 = mppi::p1;

#define LINEAR_TEMPLATE template <class CLASS_T, class PARAMS_T>
#define LINEAR_DYNAMICS LinearDynamicsImpl<CLASS_T, PARAMS_T>

LINEAR_TEMPLATE LINEAR_DYNAMICS::LinearDynamicsImpl(cudaStream_t stream) : PARENT_CLASS(stream)
{
  this->SHARED_MEM_REQUEST_GRD_BYTES = sizeof(PARAMS_T);
}

LINEAR_TEMPLATE LINEAR_DYNAMICS::LinearDynamicsImpl(PARAMS_T& params, cudaStream_t stream)
  : PARENT_CLASS(params, stream)
{
  this->SHARED_MEM_REQUEST_GRD_BYTES = sizeof(PARAMS_T);
}

LINEAR_TEMPLATE
bool LINEAR_DYNAMICS::computeGrad(const Eigen::Ref<const state_array>& state,
                                  const Eigen::Ref<const control_array>& control, Eigen::Ref<dfdx> A,
                                  Eigen::Ref<dfdu> B)
{
  A = this->params_.getA();
  B = this->params_.getB();
  return true;
}

LINEAR_TEMPLATE
LINEAR_DYNAMICS::state_array LINEAR_DYNAMICS::stateFromMap(const std::map<std::string, float>& map)
{
  state_array x = this->getZeroState();
  for (int i = 0; i < this->STATE_DIM; i++)
  {
    std::string state_name = "x_" + std::to_string(i);
    x(i, 0) = map.at(state_name);
  }
  return x;
}

LINEAR_TEMPLATE
void LINEAR_DYNAMICS::step(Eigen::Ref<state_array> state, Eigen::Ref<state_array> next_state,
                           Eigen::Ref<state_array> state_der, const Eigen::Ref<const control_array>& control,
                           Eigen::Ref<output_array> output, const float t, const float dt)
{
  state_der = this->params_.getA() * state + this->params_.getB() * control;
  next_state = state + state_der * dt;
  this->stateToOutput(next_state, output);
}

LINEAR_TEMPLATE
__device__ inline void LINEAR_DYNAMICS::step(float* state, float* next_state, float* state_der, float* control,
                                             float* output, float* theta_s, const float t, const float dt)
{
  PARAMS_T* params_p = &(this->params_);
  if (this->getGrdSharedSizeBytes() != 0)
  {
    params_p = (PARAMS_T*)theta_s;
  }
  float* A = params_p->A;
  float* B = params_p->B;

  const mp1::Parallel1Dir P_DIR = mp1::Parallel1Dir::THREAD_Y;
  mm::gemm1<this->STATE_DIM, this->STATE_DIM, 1, P_DIR>(A, state, state_der);
  mm::gemm1<this->STATE_DIM, this->CONTROL_DIM, 1, P_DIR>(B, control, state_der, 1.0f, 1.0f);
  // __syncthreads();
  int index, step;
  mp1::getParallel1DIndex<P_DIR>(index, step);
  for (int i = index; i < this->STATE_DIM; i += step)
  {
    next_state[i] = state[i] + state_der[i] * dt;
    output[i] = next_state[i];
  }
}

LINEAR_TEMPLATE
__device__ void LINEAR_DYNAMICS::initializeDynamics(float* state, float* control, float* output, float* theta_s,
                                                    float t_0, float dt)
{
  PARENT_CLASS::initializeDynamics(state, control, output, theta_s, t_0, dt);
  if (this->getGrdSharedSizeBytes() != 0)
  {
    PARAMS_T* shared = (PARAMS_T*)theta_s;
    *shared = this->params_;
  }
}

#undef LINEAR_TEMPLATE
#undef LINEAR_DYNAMICS
