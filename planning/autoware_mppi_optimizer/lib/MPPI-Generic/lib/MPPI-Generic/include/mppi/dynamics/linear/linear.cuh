/**
 * @file linear.cuh
 * @brief Linear Dynamics
 * @author Bogdan Vlahov
 * @version
 * @date 2024-08-16
 */
#pragma once

#include <mppi/dynamics/dynamics.cuh>

template <int STATE_DIM = 1, int CONTROL_DIM = 1>
struct LinearDynamicsParams : public DynamicsParams
{
public:
  using state_matrix = Eigen::Matrix<float, STATE_DIM, STATE_DIM>;
  using control_matrix = Eigen::Matrix<float, STATE_DIM, CONTROL_DIM>;
  enum class StateIndex : int
  {
    NUM_STATES = STATE_DIM,
  };

  enum class ControlIndex : int
  {
    NUM_CONTROLS = CONTROL_DIM,
  };
  enum class OutputIndex : int
  {
    NUM_OUTPUTS = STATE_DIM,
  };
  float A[STATE_DIM * STATE_DIM] = { 0.0f };
  float B[STATE_DIM * CONTROL_DIM] = { 0.0f };

  LinearDynamicsParams() = default;
  ~LinearDynamicsParams() = default;

  void setA(const Eigen::Ref<const state_matrix>& A_eigen)
  {
    memcpy(A, A_eigen.data(), sizeof(float) * STATE_DIM * STATE_DIM);
  }

  void setB(const Eigen::Ref<const control_matrix>& B_eigen)
  {
    memcpy(B, B_eigen.data(), sizeof(float) * STATE_DIM * CONTROL_DIM);
  }

  float* getA() const
  {
    return A;
  }
  float* getB() const
  {
    return B;
  }

  Eigen::Map<state_matrix> getA()
  {
    Eigen::Map<state_matrix> A_eigen(A);
    return A_eigen;
  }

  Eigen::Map<control_matrix> getB()
  {
    Eigen::Map<control_matrix> B_eigen(B);
    return B_eigen;
  }
};

using namespace MPPI_internal;

template <class CLASS_T, class PARAMS_T = LinearDynamicsParams<1, 1>>
class LinearDynamicsImpl : public Dynamics<CLASS_T, PARAMS_T>
{
public:
  using PARENT_CLASS = Dynamics<CLASS_T, PARAMS_T>;
  using state_array = typename PARENT_CLASS::state_array;
  using control_array = typename PARENT_CLASS::control_array;
  using output_array = typename PARENT_CLASS::output_array;
  using dfdx = typename PARENT_CLASS::dfdx;
  using dfdu = typename PARENT_CLASS::dfdu;
  using PARENT_CLASS::initializeDynamics;

  LinearDynamicsImpl(cudaStream_t stream = nullptr);
  LinearDynamicsImpl(PARAMS_T& params, cudaStream_t stream = nullptr);

  std::string getDynamicsModelName() const override
  {
    return "Linear Dynamics";
  }

  bool computeGrad(const Eigen::Ref<const state_array>& state, const Eigen::Ref<const control_array>& control,
                   Eigen::Ref<dfdx> A, Eigen::Ref<dfdu> B);

  state_array stateFromMap(const std::map<std::string, float>& map);

  void step(Eigen::Ref<state_array> state, Eigen::Ref<state_array> next_state, Eigen::Ref<state_array> state_der,
            const Eigen::Ref<const control_array>& control, Eigen::Ref<output_array> output, const float t,
            const float dt);

  void setA(const Eigen::Ref<const dfdx>& A_eigen, bool synchronize = false)
  {
    this->params_.setA(A_eigen);
    this->paramsToDevice(synchronize);
  }

  void setB(const Eigen::Ref<const dfdu>& B_eigen, bool synchronize = false)
  {
    this->params_.setB(B_eigen);
    this->paramsToDevice(synchronize);
  }

  __device__ inline void step(float* state, float* next_state, float* state_der, float* control, float* output,
                              float* theta_s, const float t, const float dt);

  __device__ void initializeDynamics(float* state, float* control, float* output, float* theta_s, float t_0, float dt);
};

template <int STATE_DIM = 1, int CONTROL_DIM = 1>
class LinearDynamics
  : public LinearDynamicsImpl<LinearDynamics<STATE_DIM, CONTROL_DIM>, LinearDynamicsParams<STATE_DIM, CONTROL_DIM>>
{
public:
  using PARENT_CLASS =
      LinearDynamicsImpl<LinearDynamics<STATE_DIM, CONTROL_DIM>, LinearDynamicsParams<STATE_DIM, CONTROL_DIM>>;
  using DYN_PARAMS_T = typename PARENT_CLASS::DYN_PARAMS_T;

  LinearDynamics(cudaStream_t stream = nullptr) : PARENT_CLASS(stream)
  {
  }
  LinearDynamics(DYN_PARAMS_T& params, cudaStream_t stream = nullptr) : PARENT_CLASS(params, stream)
  {
  }
};

#ifdef __CUDACC__
#include "linear.cu"
#endif
