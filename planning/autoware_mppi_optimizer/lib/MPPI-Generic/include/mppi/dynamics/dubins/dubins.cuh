#pragma once

#ifndef DUBINS_CUH_
#define DUBINS_CUH_

#include <mppi/dynamics/dynamics.cuh>
#include <mppi/utils/angle_utils.cuh>
#include <random>

struct DubinsParams : public DynamicsParams
{
  enum class StateIndex : int
  {
    POS_X = 0,
    POS_Y,
    YAW,
    NUM_STATES
  };

  enum class ControlIndex : int
  {
    VEL = 0,
    YAW_RATE,
    NUM_CONTROLS
  };

  enum class OutputIndex : int
  {
    POS_X = 0,
    POS_Y,
    YAW,
    NUM_OUTPUTS
  };
  DubinsParams() = default;
  ~DubinsParams() = default;
};

using namespace MPPI_internal;
/**
 * state: x, y, theta
 * control: forward velocity, angular velocity
 */
class DubinsDynamics : public Dynamics<DubinsDynamics, DubinsParams>
{
public:
  DubinsDynamics(cudaStream_t stream = nullptr);
  using PARENT_CLASS = Dynamics<DubinsDynamics, DubinsParams>;
  using PARENT_CLASS::updateState;  // needed as overloading updateState here hides all parent versions of updateState

  std::string getDynamicsModelName() const override
  {
    return "Dubins Model";
  }

  void computeDynamics(const Eigen::Ref<const state_array>& state, const Eigen::Ref<const control_array>& control,
                       Eigen::Ref<state_array> state_der);

  void updateState(const Eigen::Ref<const state_array> state, Eigen::Ref<state_array> next_state,
                   Eigen::Ref<state_array> state_der, const float dt);

  __device__ void updateState(float* state, float* next_state, float* state_der, const float dt);

  state_array interpolateState(const Eigen::Ref<state_array> state_1, const Eigen::Ref<state_array> state_2,
                               const float alpha);

  bool computeGrad(const Eigen::Ref<const state_array>& state, const Eigen::Ref<const control_array>& control,
                   Eigen::Ref<dfdx> A, Eigen::Ref<dfdu> B);

  __device__ void computeDynamics(float* state, float* control, float* state_der, float* theta = nullptr);

  state_array stateFromMap(const std::map<std::string, float>& map) override;

private:
};

#if __CUDACC__
#include "dubins.cu"
#endif

#endif  //! DUBINS_CUH_
