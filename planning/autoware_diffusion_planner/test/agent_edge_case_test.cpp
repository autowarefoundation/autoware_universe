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

#include "autoware/diffusion_planner/conversion/agent.hpp"

#include <Eigen/Dense>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>

#include <autoware_perception_msgs/msg/tracked_objects.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

namespace autoware::diffusion_planner::test
{

using autoware_perception_msgs::msg::TrackedObject;
using autoware_perception_msgs::msg::TrackedObjects;

class AgentEdgeCaseTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Create a basic tracked object
    tracked_object_.object_id = autoware_utils_uuid::generate_uuid();
    tracked_object_.kinematics.pose_with_covariance.pose.position.x = 1.0;
    tracked_object_.kinematics.pose_with_covariance.pose.position.y = 2.0;
    tracked_object_.kinematics.pose_with_covariance.pose.position.z = 0.0;
    tracked_object_.kinematics.pose_with_covariance.pose.orientation =
      autoware_utils::create_quaternion_from_yaw(0.0);

    tracked_object_.kinematics.twist_with_covariance.twist.linear.x = 3.0;
    tracked_object_.kinematics.twist_with_covariance.twist.linear.y = 4.0;

    tracked_object_.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
    tracked_object_.shape.dimensions.x = 5.0;
    tracked_object_.shape.dimensions.y = 2.0;
    tracked_object_.shape.dimensions.z = 1.5;

    tracked_object_.existence_probability = 0.9;

    // Add classification
    autoware_perception_msgs::msg::ObjectClassification classification;
    classification.label = autoware_perception_msgs::msg::ObjectClassification::CAR;
    classification.probability = 0.9;
    tracked_object_.classification.push_back(classification);
  }

  TrackedObject tracked_object_;

  // Feed the single fixture object through AgentData and return the surviving histories.
  std::vector<AgentHistory> run(const bool remap)
  {
    TrackedObjects objects;
    objects.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    objects.objects.push_back(tracked_object_);

    AgentData agent_data;
    agent_data.update_histories(objects, remap);
    return agent_data.transformed_and_trimmed_histories(Eigen::Matrix4d::Identity(), 10);
  }

  void set_label(const uint8_t label)
  {
    tracked_object_.classification.clear();
    autoware_perception_msgs::msg::ObjectClassification classification;
    classification.label = label;
    classification.probability = 0.9;
    tracked_object_.classification.push_back(classification);
  }
};

TEST_F(AgentEdgeCaseTest, UnknownObjectIsRemappedToPedestrian)
{
  set_label(autoware_perception_msgs::msg::ObjectClassification::UNKNOWN);

  const auto histories = run(true);

  ASSERT_EQ(histories.size(), 1u);
  const auto & state = histories.front().get_latest_state();
  EXPECT_EQ(state.label, AgentLabel::PEDESTRIAN);
  EXPECT_EQ(
    state.original_info.classification.front().label,
    autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN);

  const auto array = state.as_array();
  EXPECT_FLOAT_EQ(array[8], 0.0F);   // VEHICLE
  EXPECT_FLOAT_EQ(array[9], 1.0F);   // PEDESTRIAN
  EXPECT_FLOAT_EQ(array[10], 0.0F);  // BICYCLE

  // A BOX shape must keep its real extents.
  EXPECT_DOUBLE_EQ(state.original_info.shape.dimensions.x, 5.0);
  EXPECT_DOUBLE_EQ(state.original_info.shape.dimensions.y, 2.0);
}

TEST_F(AgentEdgeCaseTest, HazardObjectIsRemappedToPedestrian)
{
  set_label(autoware_perception_msgs::msg::ObjectClassification::HAZARD);

  const auto histories = run(true);

  ASSERT_EQ(histories.size(), 1u);
  const auto & state = histories.front().get_latest_state();
  EXPECT_EQ(state.label, AgentLabel::PEDESTRIAN);
  EXPECT_EQ(
    state.original_info.classification.front().label,
    autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN);
}

TEST_F(AgentEdgeCaseTest, UnknownPolygonObjectGetsDefaultBox)
{
  set_label(autoware_perception_msgs::msg::ObjectClassification::UNKNOWN);
  tracked_object_.shape.type = autoware_perception_msgs::msg::Shape::POLYGON;

  const auto histories = run(true);

  // Not dropped by the POLYGON filter, because the remap replaced the shape first.
  ASSERT_EQ(histories.size(), 1u);
  const auto & shape = histories.front().get_latest_state().original_info.shape;
  EXPECT_EQ(shape.type, autoware_perception_msgs::msg::Shape::BOUNDING_BOX);
  EXPECT_DOUBLE_EQ(shape.dimensions.x, 0.5);
  EXPECT_DOUBLE_EQ(shape.dimensions.y, 0.5);
  EXPECT_TRUE(shape.footprint.points.empty());
}

TEST_F(AgentEdgeCaseTest, UnsupportedObjectsAreIgnoredWhenRemapDisabled)
{
  for (const uint8_t label :
       {autoware_perception_msgs::msg::ObjectClassification::UNKNOWN,
        autoware_perception_msgs::msg::ObjectClassification::HAZARD}) {
    set_label(label);

    const auto histories = run(false);

    // get_model_label() maps both to AgentLabel::IGNORE, so they are dropped entirely.
    EXPECT_TRUE(histories.empty()) << "label " << static_cast<int>(label) << " was not ignored";
  }
}

TEST_F(AgentEdgeCaseTest, SupportedPolygonObjectIsStillSkipped)
{
  // Stays CAR, so the remap must not touch it and the POLYGON filter must still drop it.
  tracked_object_.shape.type = autoware_perception_msgs::msg::Shape::POLYGON;

  EXPECT_TRUE(run(true).empty());
}

TEST_F(AgentEdgeCaseTest, EmptyClassificationObjectIsStillIgnored)
{
  // getHighestProbLabel() returns UNKNOWN for an empty vector; there is no label to rewrite, so the
  // object must stay ignored rather than being silently promoted to PEDESTRIAN.
  tracked_object_.classification.clear();

  EXPECT_TRUE(run(true).empty());
}

TEST_F(AgentEdgeCaseTest, UnrecognizedFutureLabelIsRemapped)
{
  // Stands in for a class added to ObjectClassification later: get_model_label() sends it to
  // IGNORE, so a remap keyed on a fixed list of labels would drop it silently.
  set_label(200);

  const auto histories = run(true);

  ASSERT_EQ(histories.size(), 1u);
  EXPECT_EQ(histories.front().get_latest_state().label, AgentLabel::PEDESTRIAN);
}

TEST_F(AgentEdgeCaseTest, DeliberatelyIgnoredLabelsAreNotRemapped)
{
  for (const uint8_t label :
       {autoware_perception_msgs::msg::ObjectClassification::ANIMAL,
        autoware_perception_msgs::msg::ObjectClassification::OVER_DRIVABLE,
        autoware_perception_msgs::msg::ObjectClassification::UNDER_DRIVABLE}) {
    set_label(label);

    EXPECT_TRUE(run(true).empty()) << "label " << static_cast<int>(label) << " was remapped";
  }
}

}  // namespace autoware::diffusion_planner::test
