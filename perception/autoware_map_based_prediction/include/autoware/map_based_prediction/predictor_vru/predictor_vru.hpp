// Copyright 2024 TIER IV, inc.
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

#ifndef AUTOWARE__MAP_BASED_PREDICTION__PREDICTOR_VRU__PREDICTOR_VRU_HPP_
#define AUTOWARE__MAP_BASED_PREDICTION__PREDICTOR_VRU__PREDICTOR_VRU_HPP_

#include "autoware/map_based_prediction/data_structure.hpp"
#include "autoware/map_based_prediction/path_generator/path_generator.hpp"
#include "autoware/map_based_prediction/predictor_vru/fence.hpp"
#include "autoware/map_based_prediction/predictor_vru/history.hpp"
#include "autoware/map_based_prediction/predictor_vru/traffic_signal.hpp"

#include <autoware_utils/system/time_keeper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_traffic_rules/TrafficRules.h>

#include <memory>
#include <string>
#include <vector>

namespace autoware::map_based_prediction
{

class PredictorVru
{
public:
  explicit PredictorVru(rclcpp::Node & node)
  : node_(node), traffic_signal_module_(node), history_manager_(node)
  {
  }
  ~PredictorVru() = default;

  void setParameters(
    bool match_lost_and_appeared_crosswalk_users, double min_crosswalk_user_velocity,
    double max_crosswalk_user_delta_yaw_threshold_for_lanelet,
    double max_crosswalk_user_on_road_distance, bool use_crosswalk_signal,
    double threshold_velocity_assumed_as_stopping,
    const std::vector<double> & distance_set_for_no_intention_to_walk,
    const std::vector<double> & timeout_set_for_no_intention_to_walk,
    double prediction_sampling_time_interval, double prediction_time_horizon,
    double crossing_intention_duration, double no_crossing_intention_duration)
  {
    min_crosswalk_user_velocity_ = min_crosswalk_user_velocity;
    max_crosswalk_user_delta_yaw_threshold_for_lanelet_ =
      max_crosswalk_user_delta_yaw_threshold_for_lanelet;
    max_crosswalk_user_on_road_distance_ = max_crosswalk_user_on_road_distance;
    use_crosswalk_signal_ = use_crosswalk_signal;
    prediction_time_horizon_ = prediction_time_horizon;

    traffic_signal_module_.setParams(
      threshold_velocity_assumed_as_stopping, distance_set_for_no_intention_to_walk,
      timeout_set_for_no_intention_to_walk);

    history_manager_.setParams(
      match_lost_and_appeared_crosswalk_users, crossing_intention_duration,
      no_crossing_intention_duration);

    path_generator_ = std::make_shared<PathGenerator>(
      prediction_sampling_time_interval, min_crosswalk_user_velocity);
  }

  void setLaneletMap(std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr);

  void setTimeKeeper(std::shared_ptr<autoware_utils::TimeKeeper> time_keeper_ptr)
  {
    time_keeper_ = time_keeper_ptr;
    traffic_signal_module_.setTimeKeeper(time_keeper_ptr);
    history_manager_.setTimeKeeper(time_keeper_ptr);
  }

  void setTrafficSignal(const TrafficLightGroupArray & traffic_signal_groups)
  {
    traffic_signal_module_.update(traffic_signal_groups);
  }

  void initialize() { history_manager_.initialize(); }

  void loadCurrentCrosswalkUsers(const TrackedObjects & objects);
  void removeOldKnownMatches(const double current_time, const double buffer_time);

  PredictedObject predict(const std_msgs::msg::Header & header, const TrackedObject & object);
  PredictedObjects retrieveUndetectedObjects();

private:
  rclcpp::Node & node_;
  std::shared_ptr<autoware_utils::TimeKeeper> time_keeper_;

  // Map data
  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr_;
  lanelet::ConstLanelets crosswalks_;
  std::shared_ptr<PathGenerator> path_generator_;

  // Sub-modules
  FenceModule fence_module_;
  TrafficSignalModule traffic_signal_module_;
  CrosswalkUserHistoryManager history_manager_;

  // Parameters
  double prediction_time_horizon_{0.0};
  double min_crosswalk_user_velocity_{0.0};
  double max_crosswalk_user_delta_yaw_threshold_for_lanelet_{0.0};
  double max_crosswalk_user_on_road_distance_{0.0};
  bool use_crosswalk_signal_{false};

  PredictedObject getPredictedObjectAsCrosswalkUser(const TrackedObject & object);
};

}  // namespace autoware::map_based_prediction

#endif  // AUTOWARE__MAP_BASED_PREDICTION__PREDICTOR_VRU__PREDICTOR_VRU_HPP_
