// Copyright 2021 Tier IV, Inc.
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

#ifndef AUTOWARE__MAP_BASED_PREDICTION__PREDICTOR_VEHICLE__PREDICTOR_VEHICLE_HPP_
#define AUTOWARE__MAP_BASED_PREDICTION__PREDICTOR_VEHICLE__PREDICTOR_VEHICLE_HPP_

#include "autoware/map_based_prediction/predictor_vehicle/debug.hpp"
#include "autoware/map_based_prediction/predictor_vehicle/maneuver_prediction.hpp"
#include "autoware/map_based_prediction/predictor_vehicle/object_processing.hpp"
#include "autoware/map_based_prediction/predictor_vehicle/path_processing.hpp"

#include <autoware_utils/system/time_keeper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <autoware_perception_msgs/msg/tracked_object.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/Forward.h>
#include <lanelet2_traffic_rules/TrafficRules.h>

#include <memory>
#include <optional>
#include <string>

namespace autoware::map_based_prediction
{
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::TrackedObject;

class PredictorVehicle
{
public:
  struct Params
  {
    // Lanelet search
    double dist_threshold_for_searching_lanelet{3.0};
    double delta_yaw_threshold_for_searching_lanelet{0.785};
    double sigma_lateral_offset{0.5};
    double sigma_yaw_angle_deg{5.0};
    bool consider_only_routable_neighbours{false};
    // Object history
    double history_time_length{1.0};
    double cutoff_freq_of_velocity_lpf{0.1};
    // Lane change detection
    std::string lane_change_detection_method{"time_to_change_lane"};
    double dist_threshold_to_bound{1.0};
    double time_threshold_to_bound{5.0};
    double dist_ratio_threshold_to_left_bound{0.4};
    double dist_ratio_threshold_to_right_bound{-0.4};
    double diff_dist_threshold_to_left_bound{0.1};
    double diff_dist_threshold_to_right_bound{-0.1};
    int num_continuous_state_transition{3};
    // Path generation
    double lateral_control_time_horizon{5.0};
    double prediction_time_horizon{15.0};
    double prediction_time_horizon_rate_for_validate_lane_length{0.8};
    double prediction_sampling_time_interval{0.5};
    double min_velocity_for_map_based_prediction{1.0};
    double reference_path_resolution{0.5};
    bool check_lateral_acceleration_constraints{true};
    double max_lateral_accel{0.5};
    double min_acceleration_before_curve{-2.5};
    bool use_vehicle_acceleration{false};
    double speed_limit_multiplier{1.5};
    double acceleration_exponential_half_life{2.5};
  };

  explicit PredictorVehicle(rclcpp::Node & node);
  ~PredictorVehicle() = default;

  void setParams(const Params & params);
  const Params & getParams() const { return params_; }

  void setLaneletMap(
    std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr,
    std::shared_ptr<lanelet::routing::RoutingGraph> routing_graph_ptr,
    std::shared_ptr<lanelet::traffic_rules::TrafficRules> traffic_rules_ptr);

  void setTimeKeeper(std::shared_ptr<autoware_utils::TimeKeeper> time_keeper_ptr);

  void removeOldHistory(double current_time, double buffer_time);

  std::optional<PredictedObject> predict(
    const std_msgs::msg::Header & header, const TrackedObject & object,
    double objects_detected_time, visualization_msgs::msg::MarkerArray & debug_markers);

private:
  rclcpp::Node & node_;
  std::shared_ptr<autoware_utils::TimeKeeper> time_keeper_;

  Params params_;

  // Sub-modules
  ObjectTracker object_tracker_;
  ManeuverPredictor maneuver_predictor_;
  PathProcessor path_processor_;
};

}  // namespace autoware::map_based_prediction

#endif  // AUTOWARE__MAP_BASED_PREDICTION__PREDICTOR_VEHICLE__PREDICTOR_VEHICLE_HPP_
