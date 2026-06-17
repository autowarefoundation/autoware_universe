//
// Created by jason on 4/14/20.
//

#ifndef MPPIGENERIC_MOCK_CONTROLLER_H
#define MPPIGENERIC_MOCK_CONTROLLER_H

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <mppi/controllers/controller.cuh>
#include <mppi_test/mock_classes/mock_classes.h>

// ===== mock controller ====
class MockController
  : public Controller<MockDynamics, MockCost, MockFeedback, MockSamplingDistribution, NUM_TIMESTEPS, 512>
{
public:
  MOCK_METHOD0(calculateSampledStateTrajectories, void());
  MOCK_METHOD0(resetControls, void());
  MOCK_METHOD1(computeFeedback, void(const Eigen::Ref<const state_array>& state));
  MOCK_METHOD1(slideControlSequence, void(int stride));
  MOCK_METHOD5(getCurrentControl, control_array(Eigen::Ref<state_array>, double, Eigen::Ref<state_array>,
                                                Eigen::Ref<control_trajectory>, TEMPLATED_FEEDBACK_STATE&));
  MOCK_METHOD2(computeControl, void(const Eigen::Ref<const state_array>& state, int optimization_stride));
  MOCK_METHOD(control_trajectory, getControlSeq, (), (const, override));
  MOCK_METHOD(state_trajectory, getTargetStateSeq, (), (const, override));
  // MOCK_METHOD(output_trajectory, getTargetOutputSeq, (), (const, override));
  MOCK_METHOD(TEMPLATED_FEEDBACK_STATE, getFeedbackState, (), (const, override));
  MOCK_METHOD(control_array, getFeedbackControl,
              (const Eigen::Ref<const state_array>&, const Eigen::Ref<const state_array>&, int), (override));
  MOCK_METHOD1(updateImportanceSampler, void(const Eigen::Ref<const control_trajectory>& nominal_control));
  MOCK_METHOD0(allocateCUDAMemory, void());
  MOCK_METHOD0(computeFeedbackPropagatedStateSeq, void());
  MOCK_METHOD0(smoothControlTrajectory, void());
  MOCK_METHOD1(computeStateTrajectory, void(const Eigen::Ref<const state_array>& x0));
  MOCK_METHOD1(setPercentageSampledControlTrajectories, void(float new_perc));
  MOCK_METHOD0(getSampledNoise, std::vector<float>());
  MOCK_METHOD0(getDt, float());
};
#endif  // MPPIGENERIC_MOCK_CONTROLLER_H
