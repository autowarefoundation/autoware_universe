// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//
// Created by jason on 2/21/24.
//

#ifndef MPPI_TEST__MOCK_CLASSES__MOCK_FEEDBACK_H_
#define MPPI_TEST__MOCK_CLASSES__MOCK_FEEDBACK_H_

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

#endif  // MPPI_TEST__MOCK_CLASSES__MOCK_FEEDBACK_H_
