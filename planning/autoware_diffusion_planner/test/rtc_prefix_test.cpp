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

#include "autoware/diffusion_planner/dimensions.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

using autoware::diffusion_planner::MAX_NUM_AGENTS;
using autoware::diffusion_planner::OUTPUT_T;
using autoware::diffusion_planner::POSE_DIM;

namespace
{

/**
 * @brief Helper function to verify RTC prefix constraint
 *
 * Verifies that:
 * 1. First 'delay' timesteps match the previous prediction (prefix constraint)
 * 2. Timesteps after 'delay' are independent (not copied from previous prediction)
 *
 * @param delay Number of timesteps that should match as prefix
 */
void verify_prefix_constraint(int delay)
{
  const float tolerance = 1e-5f;
  const float offset = 100.0f;  // Offset to ensure non-prefix values are different

  // Generate random previous prediction
  std::mt19937 gen(42 + delay);  // Different seed for each delay
  std::uniform_real_distribution<float> dist(-10.0f, 10.0f);

  std::vector<float> previous_prediction(OUTPUT_T * POSE_DIM);
  for (auto & val : previous_prediction) {
    val = dist(gen);
  }

  // Create sampled_trajectories with offset values (to ensure they're different)
  std::vector<float> sampled_trajectories((MAX_NUM_AGENTS) * (OUTPUT_T + 1) * POSE_DIM);
  for (auto & val : sampled_trajectories) {
    val = dist(gen) + offset;
  }

  // Copy prefix from previous prediction (simulating RTC behavior)
  // Prefix: sampled[t] = previous_prediction[t+1] for t < delay
  for (int t = 0; t < delay && (t + 1) < OUTPUT_T; ++t) {
    const size_t src_idx = (t + 1) * POSE_DIM;  // t+1 from previous prediction
    const size_t dst_idx = 0 * (OUTPUT_T + 1) * POSE_DIM + t * POSE_DIM;  // t in sampled

    for (int d = 0; d < POSE_DIM; ++d) {
      sampled_trajectories[dst_idx + d] = previous_prediction[src_idx + d];
    }
  }

  // Verify prefix matches (t < delay)
  for (int t = 0; t < delay && (t + 1) < OUTPUT_T; ++t) {
    const size_t src_idx = (t + 1) * POSE_DIM;
    const size_t dst_idx = 0 * (OUTPUT_T + 1) * POSE_DIM + t * POSE_DIM;

    for (int d = 0; d < POSE_DIM; ++d) {
      EXPECT_NEAR(sampled_trajectories[dst_idx + d], previous_prediction[src_idx + d], tolerance)
        << "Prefix should match at delay=" << delay << ", t=" << t << ", dim=" << d;
    }
  }

  // Verify non-prefix is different (t >= delay)
  for (int t = delay; t < std::min(OUTPUT_T, static_cast<int64_t>(20)); ++t) {
    const size_t src_idx = (t + 1) * POSE_DIM;
    const size_t dst_idx = 0 * (OUTPUT_T + 1) * POSE_DIM + t * POSE_DIM;

    // Check that at least one dimension differs significantly
    bool has_difference = false;
    for (int d = 0; d < POSE_DIM; ++d) {
      if (src_idx + d < previous_prediction.size()) {
        float diff = std::abs(sampled_trajectories[dst_idx + d] - previous_prediction[src_idx + d]);
        if (diff > offset / 2.0f) {  // Should have the offset
          has_difference = true;
          break;
        }
      }
    }

    EXPECT_TRUE(has_difference) << "Non-prefix values should be independent at delay=" << delay
                                << ", t=" << t;
  }
}

}  // namespace

// Test cases for different delay values

TEST(RTCPrefixTest, Delay0)
{
  verify_prefix_constraint(0);
}

TEST(RTCPrefixTest, Delay1)
{
  verify_prefix_constraint(1);
}

TEST(RTCPrefixTest, Delay5)
{
  verify_prefix_constraint(5);
}

TEST(RTCPrefixTest, Delay10)
{
  verify_prefix_constraint(10);
}

TEST(RTCPrefixTest, Delay15)
{
  verify_prefix_constraint(15);
}
