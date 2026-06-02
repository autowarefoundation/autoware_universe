// Copyright 2021 TIER IV, Inc.
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

#ifndef AUTOWARE__MAP_BASED_PREDICTION__PREDICTOR_VEHICLE__MANEUVER_PREDICTION_HPP_
#define AUTOWARE__MAP_BASED_PREDICTION__PREDICTOR_VEHICLE__MANEUVER_PREDICTION_HPP_

#include "autoware/map_based_prediction/data_structure.hpp"

#include <autoware_utils/system/time_keeper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <lanelet2_routing/Forward.h>

#include <deque>
#include <memory>
#include <string>
#include <unordered_map>

namespace autoware::map_based_prediction
{

class ManeuverPredictor
{
public:
  struct Params
  {
    std::string lane_change_detection_method{"time_to_change_lane"};
    double dist_threshold_to_bound{1.0};
    double time_threshold_to_bound{5.0};
    double dist_ratio_threshold_to_left_bound{0.4};
    double dist_ratio_threshold_to_right_bound{-0.4};
    double diff_dist_threshold_to_left_bound{0.1};
    double diff_dist_threshold_to_right_bound{-0.1};
    int num_continuous_state_transition{3};
    double history_time_length{1.0};
  };

  explicit ManeuverPredictor(rclcpp::Node & node);

  void setTimeKeeper(std::shared_ptr<autoware_utils::TimeKeeper> time_keeper_ptr);
  void setRoutingGraph(std::shared_ptr<lanelet::routing::RoutingGraph> routing_graph_ptr);
  void setParams(const Params & params);

  Maneuver predictObjectManeuver(
    const std::string & object_id, const geometry_msgs::msg::Pose & object_pose,
    const LaneletData & current_lanelet_data, double object_detected_time,
    std::unordered_map<std::string, std::deque<RoadUser>> & history);

  ManeuverProbability calculateManeuverProbability(
    const Maneuver & predicted_maneuver, bool left_paths_exists, bool right_paths_exists,
    bool center_paths_exists) const;

private:
  Maneuver predictObjectManeuverByTimeToLaneChange(
    const std::string & object_id, const LaneletData & current_lanelet_data,
    double object_detected_time,
    const std::unordered_map<std::string, std::deque<RoadUser>> & history);
  Maneuver predictObjectManeuverByLatDiffDistance(
    const std::string & object_id, const geometry_msgs::msg::Pose & object_pose,
    const LaneletData & current_lanelet_data, double object_detected_time,
    const std::unordered_map<std::string, std::deque<RoadUser>> & history);
  double calcRightLateralOffset(
    const lanelet::ConstLineString2d & boundary_line, const geometry_msgs::msg::Pose & search_pose);
  double calcLeftLateralOffset(
    const lanelet::ConstLineString2d & boundary_line, const geometry_msgs::msg::Pose & search_pose);

  rclcpp::Node & node_;
  std::shared_ptr<autoware_utils::TimeKeeper> time_keeper_;
  std::shared_ptr<lanelet::routing::RoutingGraph> routing_graph_ptr_;
  Params params_;
};

}  // namespace autoware::map_based_prediction

#endif  // AUTOWARE__MAP_BASED_PREDICTION__PREDICTOR_VEHICLE__MANEUVER_PREDICTION_HPP_
