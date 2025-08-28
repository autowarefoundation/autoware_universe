// Copyright 2025 TIER IV, Inc.
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

#include "autoware/simpl_prediction/processing/postprocessor.hpp"

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>

#include <gtest/gtest.h>

#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::simpl_prediction::test
{
using autoware::simpl_prediction::processing::PostProcessor;
using autoware_perception_msgs::msg::TrackedObject;
using geometry_msgs::msg::Point;

namespace
{
class TestPostProcessor : public ::testing::Test
{
protected:
  void SetUp() override
  {
    TrackedObject obj;
    obj.object_id.uuid = {{1, 2, 3, 4}};
    obj.kinematics.pose_with_covariance.pose.position =
      geometry_msgs::build<Point>().x(1.0).y(2.0).z(0.0);
    obj.kinematics.pose_with_covariance.pose.orientation.w = 1.0;  // identity rotation
    tracked_map_["1-2-3-4"] = obj;
  }

  PostProcessor::Header make_header()
  {
    return std_msgs::build<PostProcessor::Header>().stamp(rclcpp::Clock().now()).frame_id("map");
  }

  std::unordered_map<std::string, TrackedObject> tracked_map_;
};
}  // namespace

TEST_F(TestPostProcessor, OutputsValidPredictedObject)
{
  const size_t num_modes = 2;
  const size_t num_future = 3;
  const double score_threshold = 0.5;

  PostProcessor processor(num_modes, num_future, score_threshold);

  std::vector<float> scores = {0.6f, 0.4f};                   // one above, one below
  std::vector<float> trajectories = {0.1f, 0.1f, 0.0f, 0.0f,  // t=0, mode=0
                                     0.2f, 0.2f, 0.0f, 0.0f,  // t=1
                                     0.3f, 0.3f, 0.0f, 0.0f,  // t=2
                                     1.0f, 1.0f, 0.0f, 0.0f,  // mode=1 (won't be used)
                                     1.1f, 1.1f, 0.0f, 0.0f, 1.2f, 1.2f, 0.0f, 0.0f};

  std::vector<std::string> agent_ids = {"1-2-3-4"};

  auto msg = processor.process(scores, trajectories, agent_ids, make_header(), tracked_map_);

  ASSERT_EQ(msg.objects.size(), 1);
  const auto & obj = msg.objects.front();
  EXPECT_EQ(obj.kinematics.predicted_paths.size(), 1);
  EXPECT_NEAR(obj.kinematics.predicted_paths.front().confidence, 0.6, 1e-6);
  EXPECT_EQ(obj.kinematics.predicted_paths.front().path.size(), 4);  // 3 future + 1 current
}
}  // namespace autoware::simpl_prediction::test
