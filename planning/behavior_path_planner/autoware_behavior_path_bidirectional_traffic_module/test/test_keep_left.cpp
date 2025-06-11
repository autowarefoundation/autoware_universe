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
#include "autoware/behavior_path_bidirectional_traffic_module/keep_left.hpp"
#include "autoware/trajectory/path_point_with_lane_id.hpp"
#include "autoware/trajectory/utils/closest.hpp"
#include "autoware/trajectory/utils/find_intervals.hpp"
#include "autoware_utils/geometry/geometry.hpp"

#include <autoware_internal_planning_msgs/msg/path_point_with_lane_id.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

namespace autoware::behavior_path_planner
{
class TestKeepLeft : public ::testing::Test
{
public:
  std::vector<autoware::behavior_path_planner::ConnectedBidirectionalLanelets::SharedConstPtr>
    bidirectional_lanelets;

  autoware_internal_planning_msgs::msg::PathWithLaneId path;

protected:
  void SetUp() override
  {
    bidirectional_lanelets = load_bidirectional_lanelets_from_map(
      "autoware_behavior_path_bidirectional_traffic_module", "lanelet2_map.osm");
    path = load_path("autoware_behavior_path_bidirectional_traffic_module", "test_data/path1.yaml");
  }
};
TEST_F(TestKeepLeft, keepLeftTest)
{
  auto trajectory = experimental::trajectory::Trajectory<
                      autoware_internal_planning_msgs::msg::PathPointWithLaneId>::Builder{}
                      .build(path.points);
  if (!trajectory) {
    throw std::runtime_error("Failed to build trajectory in ConnectedBidirectionalLanelets");
  }

  BidirectionalTrafficModuleParameters parameters;
  parameters.keep_left_distance_from_center_line = 0.1;

  auto shifted_trajectory = shift_trajectory_for_keep_left(
    *trajectory, bidirectional_lanelets, parameters, EgoParameters(3.8, 1.1, 1.9),
    [](std::string_view) {});

  std::vector<int> bidirectional_lanelet_ids = {37179, 42762, 39876};

  auto intervals = experimental::trajectory::find_intervals(
    *trajectory, [&](const autoware_internal_planning_msgs::msg::PathPointWithLaneId & point) {
      for (const auto & lanelet_id : bidirectional_lanelet_ids) {
        if (
          std::find(point.lane_ids.begin(), point.lane_ids.end(), lanelet_id) !=
          point.lane_ids.end()) {
          return true;
        }
      }
      return false;
    });

  EXPECT_EQ(intervals.size(), 1);

  auto bidirectional_lanelet_entering = trajectory->compute(intervals[0].start);
  auto bidirectional_lanelet_exiting = trajectory->compute(intervals[0].end);

  double shifted_trajectory_start_s =
    experimental::trajectory::closest(shifted_trajectory, bidirectional_lanelet_entering);
  double shifted_trajectory_end_s =
    experimental::trajectory::closest(shifted_trajectory, bidirectional_lanelet_exiting);

  auto shifted_trajectory_start = shifted_trajectory.compute(shifted_trajectory_start_s);

  auto shifted_trajectory_end = shifted_trajectory.compute(shifted_trajectory_end_s);

  double distance_enter =
    autoware_utils::calc_distance2d(bidirectional_lanelet_entering, shifted_trajectory_start);

  double distance_exit =
    autoware_utils::calc_distance2d(bidirectional_lanelet_exiting, shifted_trajectory_end);

  EXPECT_NEAR(distance_enter, parameters.keep_left_distance_from_center_line, 0.01);
  EXPECT_NEAR(distance_exit, parameters.keep_left_distance_from_center_line, 0.01);
}
}  // namespace autoware::behavior_path_planner
