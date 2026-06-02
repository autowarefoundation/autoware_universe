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

#ifndef AUTOWARE__MAP_BASED_PREDICTION__PREDICTOR_VEHICLE__OBJECT_PROCESSING_HPP_
#define AUTOWARE__MAP_BASED_PREDICTION__PREDICTOR_VEHICLE__OBJECT_PROCESSING_HPP_

#include "autoware/map_based_prediction/data_structure.hpp"

#include <autoware_utils/system/time_keeper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/header.hpp>

#include <lanelet2_routing/Forward.h>

#include <deque>
#include <memory>
#include <string>
#include <unordered_map>

namespace autoware::map_based_prediction
{

class ObjectTracker
{
public:
  struct Params
  {
    double dist_threshold_for_searching_lanelet{3.0};
    double delta_yaw_threshold_for_searching_lanelet{0.785};
    double sigma_lateral_offset{0.5};
    double sigma_yaw_angle_deg{5.0};
    double cutoff_freq_of_velocity_lpf{0.1};
  };

  explicit ObjectTracker(rclcpp::Node & node);

  void setTimeKeeper(std::shared_ptr<autoware_utils::TimeKeeper> time_keeper_ptr);
  void setLaneletMap(std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr);
  void setRoutingGraph(std::shared_ptr<lanelet::routing::RoutingGraph> routing_graph_ptr);
  void setParams(const Params & params);

  void removeOldHistory(double current_time, double buffer_time);

  void updateObjectData(TrackedObject & object);
  LaneletsData getCurrentLanelets(const TrackedObject & object);
  void updateRoadUsersHistory(
    const std_msgs::msg::Header & header, const TrackedObject & object,
    const LaneletsData & current_lanelets_data);

  std::unordered_map<std::string, std::deque<RoadUser>> & getHistory()
  {
    return road_users_history_;
  }
  const std::unordered_map<std::string, std::deque<RoadUser>> & getHistory() const
  {
    return road_users_history_;
  }

private:
  rclcpp::Node & node_;
  std::shared_ptr<autoware_utils::TimeKeeper> time_keeper_;
  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr_;
  std::shared_ptr<lanelet::routing::RoutingGraph> routing_graph_ptr_;
  std::unordered_map<std::string, std::deque<RoadUser>> road_users_history_;
  Params params_;
};

}  // namespace autoware::map_based_prediction

#endif  // AUTOWARE__MAP_BASED_PREDICTION__PREDICTOR_VEHICLE__OBJECT_PROCESSING_HPP_
