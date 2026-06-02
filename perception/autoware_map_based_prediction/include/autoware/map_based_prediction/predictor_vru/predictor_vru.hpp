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

#include <memory>
#include <vector>

namespace autoware::map_based_prediction
{

class PredictorVru
{
public:
  struct Params
  {
    // Path generation
    double prediction_time_horizon{10.0};
    double prediction_sampling_time_interval{0.5};
    // Movement thresholds
    double min_crosswalk_user_velocity{1.39};
    double max_crosswalk_user_delta_yaw_threshold_for_lanelet{0.785};
    double max_crosswalk_user_on_road_distance{2.0};
    // Signal interaction
    bool use_crosswalk_signal{true};
    // Sub-module params
    TrafficSignalModule::Params traffic_signal;
    CrosswalkUserHistoryManager::Params history;
  };

  explicit PredictorVru(rclcpp::Node & node)
  : node_(node), traffic_signal_module_(node), history_manager_(node)
  {
  }
  ~PredictorVru() = default;

  void setParams(const Params & params)
  {
    params_ = params;
    traffic_signal_module_.setParams(params.traffic_signal);
    history_manager_.setParams(params.history);
    path_generator_ = std::make_shared<PathGenerator>(
      params.prediction_sampling_time_interval, params.min_crosswalk_user_velocity);
  }

  const Params & getParams() const { return params_; }

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

  Params params_{};

  PredictedObject getPredictedObjectAsCrosswalkUser(const TrackedObject & object);
};

}  // namespace autoware::map_based_prediction

#endif  // AUTOWARE__MAP_BASED_PREDICTION__PREDICTOR_VRU__PREDICTOR_VRU_HPP_
