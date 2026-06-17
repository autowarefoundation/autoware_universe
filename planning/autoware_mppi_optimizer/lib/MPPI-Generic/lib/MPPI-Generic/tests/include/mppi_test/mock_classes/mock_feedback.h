//
// Created by jason on 2/21/24.
//

#ifndef MPPIGENERIC_TESTS_INCLUDE_MPPI_TEST_MOCK_CLASSES_MOCK_FEEDBACK_H_
#define MPPIGENERIC_TESTS_INCLUDE_MPPI_TEST_MOCK_CLASSES_MOCK_FEEDBACK_H_

#include <mppi_test/mock_classes/mock_classes.h>
#include <mppi/feedback_controllers/feedback.cuh>

struct mockGPUFeedbackParams
{
};

class MockGPUFeedback : public GPUFeedbackController<MockGPUFeedback, MockDynamics, GPUState>
{
public:
  using DYN_T = MockDynamics;
  using FEEDBACK_STATE_T = GPUState;

  MockGPUFeedback(cudaStream_t stream) : GPUFeedbackController<MockGPUFeedback, MockDynamics, GPUState>(stream)
  {
  }
};

class MockFeedback : public FeedbackController<MockGPUFeedback, mockGPUFeedbackParams, NUM_TIMESTEPS>
{
public:
  MOCK_METHOD0(initTrackingController, void());
  MOCK_METHOD4(k_, control_array(const Eigen::Ref<const state_array>& x_act,
                                 const Eigen::Ref<const state_array>& x_goal, int t, GPUState& fb_state));
  MOCK_METHOD3(computeFeedback, void(const Eigen::Ref<const state_array>& init_state,
                                     const Eigen::Ref<const state_trajectory>& goal_traj,
                                     const Eigen::Ref<const control_trajectory>& control_traj));
  MOCK_METHOD0(freeCudaMem, void());
};

#endif  // MPPIGENERIC_TESTS_INCLUDE_MPPI_TEST_MOCK_CLASSES_MOCK_FEEDBACK_H_
