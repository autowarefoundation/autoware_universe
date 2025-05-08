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

#include "autoware/behavior_path_planner_common/utils/parking_departure/geometric_parallel_parking.hpp"

#include <autoware_test_utils/autoware_test_utils.hpp>

#include <gtest/gtest.h>

#include <vector>

namespace autoware::behavior_path_planner
{

class TestGeometricParallelParking : public ::testing::Test
{

protected:
  GeometricParallelParking geometric_parallel_parking_;
  
  const double R_E_far_= 2.74/std::tan(0.7);
  std::shared_ptr<autoware::lane_departure_checker::LaneDepartureChecker> lane_departure_checker_;
  const double arc_path_interval_ = 0.05;

  std::vector<lanelet::ConstLanelet> return_lanelets(const double center_x, const double center_y,
    const double length_x, const double width_y) 
  {
    lanelet::LineString3d left_bound;
    lanelet::LineString3d right_bound;
    lanelet::LineString3d centerline;
    left_bound.push_back(lanelet::Point3d{lanelet::InvalId, center_x-length_x, center_y+width_y});
    left_bound.push_back(lanelet::Point3d{lanelet::InvalId, center_x         , center_y+width_y});
    left_bound.push_back(lanelet::Point3d{lanelet::InvalId, center_x+length_x, center_y+width_y});
    right_bound.push_back(lanelet::Point3d{lanelet::InvalId, center_x-length_x, center_y-width_y});
    right_bound.push_back(lanelet::Point3d{lanelet::InvalId, center_x         , center_y-width_y});
    right_bound.push_back(lanelet::Point3d{lanelet::InvalId, center_x+length_x, center_y-width_y});
    centerline.push_back(lanelet::Point3d{lanelet::InvalId, center_x         , center_y});
    centerline.push_back(lanelet::Point3d{lanelet::InvalId, center_x+length_x, center_y});
    centerline.push_back(lanelet::Point3d{lanelet::InvalId, center_x-length_x, center_y});
    lanelet::Lanelet road_lane{lanelet::InvalId, left_bound, right_bound};
    road_lane.setCenterline(centerline);
    std::vector<lanelet::ConstLanelet> road_lanes{road_lane};
    return road_lanes;
  }

};

TEST_F(TestGeometricParallelParking, GenerateValidClothoidPullOverPath)
{
    geometry_msgs::msg::Pose start_pose;
    geometry_msgs::msg::Pose goal_pose;

    const auto road_lanes = return_lanelets(0, 0, 10, 2);
    const auto right_shoulder_lanes = return_lanelets(0, -3, 10, 2);
    const auto left_shoulder_lanes = return_lanelets(0, 3, 10, 2);
    
    const double L_min = 1.0;
    const double R_E_far_= 2.74/std::tan(0.7);
    std::shared_ptr<autoware::lane_departure_checker::LaneDepartureChecker> lane_departure_checker_;
    const double arc_path_interval_ = 0.1;
    
    goal_pose.position.x = 3.0;
    goal_pose.position.y = 1.0;
    auto paths_forward_left = geometric_parallel_parking_.planOneTrialClothoid(start_pose, goal_pose, R_E_far_, L_min, 
        road_lanes, left_shoulder_lanes, true, true,
        0.0, 0.0, arc_path_interval_, lane_departure_checker_);

    goal_pose.position.x = 3.0;
    goal_pose.position.y = -1.0;
    auto paths_forward_right = geometric_parallel_parking_.planOneTrialClothoid(start_pose, goal_pose, R_E_far_, L_min, 
        road_lanes, right_shoulder_lanes, true, false,
        0.0, 0.0, arc_path_interval_, lane_departure_checker_);
    
    // Assert that a valid geometric pull out path is generated
    EXPECT_TRUE(paths_forward_left.size() > 0);
    EXPECT_TRUE(paths_forward_right.size() > 0);
}

}  // namespace autoware::behavior_path_planner
