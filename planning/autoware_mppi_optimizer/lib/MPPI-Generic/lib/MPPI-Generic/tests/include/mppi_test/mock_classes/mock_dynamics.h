//
// Created by jason on 4/14/20.
//

#ifndef MPPIGENERIC_MOCK_DYNAMICS_H
#define MPPIGENERIC_MOCK_DYNAMICS_H

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <mppi/dynamics/dynamics.cuh>

// ===== mock dynamics ====
struct mockDynamicsParams : public DynamicsParams
{
  int test = 2;
  bool copy_everything = false;
  float buffer[(1 + 1) * 10] = { 0.0 };

  void updateBuffer(Eigen::Matrix<float, 2, 11> new_buffer)
  {
  }
};

class MockDynamics : public MPPI_internal::Dynamics<MockDynamics, mockDynamicsParams>
{
public:
  typedef Eigen::Matrix<float, CONTROL_DIM, 1> control_array;                 // Control at a time t
  typedef Eigen::Matrix<float, STATE_DIM, 1> state_array;                     // State at a time t
  typedef Eigen::Matrix<float, OUTPUT_DIM, 1> output_array;                   // Output at a time t
  typedef Eigen::Matrix<float, STATE_DIM, STATE_DIM> dfdx;                    // Jacobian wrt x
  typedef Eigen::Matrix<float, STATE_DIM, CONTROL_DIM> dfdu;                  // Jacobian wrt u
  typedef Eigen::Matrix<float, CONTROL_DIM, STATE_DIM> feedback_matrix;       // Feedback matrix
  typedef Eigen::Matrix<float, STATE_DIM, STATE_DIM + CONTROL_DIM> Jacobian;  // Jacobian of x and u

  typedef std::map<std::string, Eigen::VectorXf> buffer_trajectory;

  MOCK_METHOD1(bindToStream, void(cudaStream_t stream));
  MOCK_METHOD1(setParams, void(mockDynamicsParams params));
  MOCK_METHOD0(GPUSetup, void());
  MOCK_METHOD0(getControlRanges, std::array<float2, 1>());
  MOCK_METHOD0(getControlRangesRaw, void());
  MOCK_METHOD0(getParams, mockDynamicsParams());
  // MOCK_METHOD0(freeCudaMem, void());
  MOCK_METHOD0(paramsToDevice, void());
  MOCK_METHOD3(computeDynamics, void(const Eigen::Ref<const state_array>&, const Eigen::Ref<const control_array>&,
                                     Eigen::Ref<state_array>));
  MOCK_METHOD2(computeKinematics, void(Eigen::Ref<const state_array>&, Eigen::Ref<state_array>));
  MOCK_METHOD3(computeStateDeriv, void(const Eigen::Ref<const state_array>&, const Eigen::Ref<const control_array>&,
                                       Eigen::Ref<state_array>));
  MOCK_METHOD4(updateState,
               void(const Eigen::Ref<const state_array>, Eigen::Ref<state_array>, Eigen::Ref<state_array>, float));

  MOCK_METHOD7(step, void(Eigen::Ref<state_array>, Eigen::Ref<state_array>, Eigen::Ref<state_array>,
                          const Eigen::Ref<const control_array>&, Eigen::Ref<output_array>, float, float));
  MOCK_METHOD3(updateState, void(const Eigen::Ref<const state_array>, Eigen::Ref<state_array>, float));
  MOCK_METHOD2(enforceConstraints, void(Eigen::Ref<state_array>, Eigen::Ref<control_array>));
  MOCK_METHOD4(computeGrad, bool(const Eigen::Ref<const state_array>&, const Eigen::Ref<const control_array>,
                                 Eigen::Ref<dfdx>, Eigen::Ref<dfdu>));
  MOCK_METHOD1(updateFromBuffer, void(const buffer_trajectory& buffer));
  MOCK_METHOD1(stateFromMap, state_array(const std::map<std::string, float>&));
  MOCK_METHOD0(freeCudaMem, void());
};

#endif  // MPPIGENERIC_MOCK_DYNAMICS_H
