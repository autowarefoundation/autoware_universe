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

#include "./test_helper.hpp"
#include "autoware/behavior_path_bidirectional_traffic_module/bidirectional_lanelets.hpp"
#include "autoware/behavior_path_bidirectional_traffic_module/give_way.hpp"
#include "autoware/behavior_path_bidirectional_traffic_module/oncoming_car.hpp"
#include "autoware/trajectory/path_point_with_lane_id.hpp"

#include <autoware_internal_planning_msgs/msg/path_point_with_lane_id.hpp>
#include <autoware_perception_msgs/msg/predicted_object.hpp>

#include <gtest/gtest.h>

#include <cstdlib>
#include <iostream>
#include <vector>

namespace autoware::behavior_path_planner
{
class TestGiveWay : public ::testing::Test
{
public:
  std::vector<autoware::behavior_path_planner::ConnectedBidirectionalLanelets::SharedConstPtr>
    bidirectional_lanelets;

  autoware_internal_planning_msgs::msg::PathWithLaneId path;

  std::optional<geometry_msgs::msg::Pose> stop_pose_;

protected:
  void SetUp() override
  {
    bidirectional_lanelets = load_bidirectional_lanelets_from_map(
      "autoware_behavior_path_bidirectional_traffic_module", "lanelet2_map.osm");
    path = load_path("autoware_behavior_path_bidirectional_traffic_module", "test_data/path1.yaml");
  }
};
TEST_F(TestGiveWay, keepLeftTest)
{
  auto trajectory = experimental::trajectory::Trajectory<
                      autoware_internal_planning_msgs::msg::PathPointWithLaneId>::Builder{}
                      .build(path.points);
  if (!trajectory) {
    throw std::runtime_error("Failed to build trajectory in ConnectedBidirectionalLanelets");
  }

  EgoParameters ego_params(3.8, 1.1, 1.9);

  GiveWay give_way(
    bidirectional_lanelets[0], ego_params, BidirectionalTrafficModuleParameters{},
    [&](geometry_msgs::msg::Pose pose) { stop_pose_ = pose; });

  EXPECT_EQ(give_way.get_state_name(), "NoNeedToGiveWay");

  geometry_msgs::msg::Pose ego_pose;
  ego_pose.position.x = 722;
  ego_pose.position.y = 1308;
  ego_pose.position.z = 100;
  ego_pose.orientation.x = 0;
  ego_pose.orientation.y = 0;
  ego_pose.orientation.z = 0;
  ego_pose.orientation.w = 1;

  std::optional<ConnectedBidirectionalLanelets::SharedConstPtr> bidirectional_lane_where_ego_is =
    get_bidirectional_lanelets_where_ego_is(ego_pose, ego_params, bidirectional_lanelets);

  if (!bidirectional_lane_where_ego_is.has_value()) {
    std::cerr << "No bidirectional lane found for the given ego pose." << std::endl;
    // Fail the test or handle the error as needed
    return;
  }

  OncomingCars oncoming_cars(*bidirectional_lane_where_ego_is, 0.1);

  autoware_perception_msgs::msg::PredictedObject obj;

  obj.kinematics.initial_pose_with_covariance.pose.position.x = 737;
  obj.kinematics.initial_pose_with_covariance.pose.position.y = 1306;
  obj.kinematics.initial_pose_with_covariance.pose.position.z = 100;
  obj.kinematics.initial_pose_with_covariance.pose.orientation.x = 0;
  obj.kinematics.initial_pose_with_covariance.pose.orientation.y = 0;
  obj.kinematics.initial_pose_with_covariance.pose.orientation.z = 1;
  obj.kinematics.initial_pose_with_covariance.pose.orientation.w = 0;
  obj.kinematics.initial_twist_with_covariance.twist.linear.x = 3.0;
  autoware_perception_msgs::msg::ObjectClassification classification;
  classification.label = autoware_perception_msgs::msg::ObjectClassification::CAR;
  obj.classification.emplace_back(classification);

  autoware_perception_msgs::msg::PredictedObjects objs;
  objs.objects.push_back(obj);

  oncoming_cars.update(objs, ego_pose, rclcpp::Time(0));
  oncoming_cars.update(objs, ego_pose, rclcpp::Time(1e+9));
  oncoming_cars.update(objs, ego_pose, rclcpp::Time(2e+9));

  EXPECT_EQ(oncoming_cars.get_front_oncoming_car().has_value(), true);

  auto trj = give_way.modify_trajectory(*trajectory, oncoming_cars, ego_pose, 1.0);

  EXPECT_EQ(give_way.get_state_name(), "ApproachingToShift");

  EXPECT_EQ(stop_pose_.has_value(), true);

  trj = give_way.modify_trajectory(*trajectory, oncoming_cars, *stop_pose_, 1.0);

  EXPECT_EQ(give_way.get_state_name(), "ShiftingRoadside");

  trj = give_way.modify_trajectory(*trajectory, oncoming_cars, *stop_pose_, 0.0);

  EXPECT_EQ(give_way.get_state_name(), "WaitingForOncomingCarsToPass");

  obj.kinematics.initial_pose_with_covariance.pose.position.x = 657;
  obj.kinematics.initial_pose_with_covariance.pose.position.y = 1306;
  obj.kinematics.initial_pose_with_covariance.pose.position.z = 1306;

  objs.objects.clear();
  objs.objects.push_back(obj);

  oncoming_cars.update(objs, ego_pose, rclcpp::Time(3e+9));
  oncoming_cars.update(objs, ego_pose, rclcpp::Time(3e+9));

  EXPECT_EQ(oncoming_cars.get_front_oncoming_car().has_value(), false);

  trj = give_way.modify_trajectory(*trajectory, oncoming_cars, *stop_pose_, 0.0);

  EXPECT_EQ(give_way.get_state_name(), "BackToNormalLane");

  ego_pose.position.x = 815;
  ego_pose.position.y = 1330;
  ego_pose.position.z = 100;

  trj = give_way.modify_trajectory(*trajectory, oncoming_cars, ego_pose, 0.0);
  EXPECT_EQ(give_way.get_state_name(), "NoNeedToGiveWay");
}
}  // namespace autoware::behavior_path_planner
