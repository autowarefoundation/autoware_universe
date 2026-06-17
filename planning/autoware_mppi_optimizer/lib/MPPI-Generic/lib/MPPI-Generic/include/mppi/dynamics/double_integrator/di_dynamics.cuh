#pragma once

#ifndef DOUBLE_INTEGRATOR_CUH_
#define DOUBLE_INTEGRATOR_CUH_

#include <mppi/dynamics/dynamics.cuh>
#include <random>

struct DoubleIntegratorParams : public DynamicsParams
{
  enum class StateIndex : int
  {
    POS_X = 0,
    POS_Y,
    VEL_X,
    VEL_Y,
    NUM_STATES
  };
  enum class ControlIndex : int
  {
    ACCEL_X = 0,
    ACCEL_Y,
    NUM_CONTROLS
  };
  enum class OutputIndex : int
  {
    POS_X = 0,
    POS_Y,
    VEL_X,
    VEL_Y,
    NUM_OUTPUTS
  };
  float system_noise = 1;
  DoubleIntegratorParams(float noise) : system_noise(noise){};
  DoubleIntegratorParams() = default;
  ~DoubleIntegratorParams() = default;
};

using namespace MPPI_internal;

class DoubleIntegratorDynamics : public Dynamics<DoubleIntegratorDynamics, DoubleIntegratorParams>
{
public:
  //  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  DoubleIntegratorDynamics(float system_noise = 1, cudaStream_t stream = nullptr);

  std::string getDynamicsModelName() const override
  {
    return "2D Double Integrator Model";
  }

  void computeDynamics(const Eigen::Ref<const state_array>& state, const Eigen::Ref<const control_array>& control,
                       Eigen::Ref<state_array> state_der);

  bool computeGrad(const Eigen::Ref<const state_array>& state, const Eigen::Ref<const control_array>& control,
                   Eigen::Ref<dfdx> A, Eigen::Ref<dfdu> B);

  void computeStateDisturbance(float dt, Eigen::Ref<state_array> state);

  void setStateVariance(float system_variance = 1.0);

  void printState(float* state);
  void printState(const float* state);

  __device__ void computeDynamics(float* state, float* control, float* state_der, float* theta = nullptr);

  dfdu B(const Eigen::Ref<const state_array>& state);

  state_array stateFromMap(const std::map<std::string, float>& map);

private:
  // Random number generator for system noise
  std::mt19937 gen;  // Standard mersenne_twister_engine which will be seeded
  std::normal_distribution<float> normal_distribution;
};

#if __CUDACC__
#include "di_dynamics.cu"
#endif

#endif  //! DOUBLE_INTEGRATOR_CUH_
