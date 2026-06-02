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

#include "autoware/map_based_prediction/predictor_vehicle/predictor_vehicle.hpp"

namespace autoware::map_based_prediction
{

PredictorVehicle::PredictorVehicle(rclcpp::Node & node)
: node_(node), object_tracker_(node), maneuver_predictor_(node), path_processor_(node)
{
  path_processor_.setManeuverPredictor(maneuver_predictor_);
  path_processor_.setObjectTracker(object_tracker_);
}

void PredictorVehicle::setParams(const Params & params)
{
  params_ = params;

  {
    ObjectTracker::Params ot_params;
    ot_params.dist_threshold_for_searching_lanelet = params.dist_threshold_for_searching_lanelet;
    ot_params.delta_yaw_threshold_for_searching_lanelet =
      params.delta_yaw_threshold_for_searching_lanelet;
    ot_params.sigma_lateral_offset = params.sigma_lateral_offset;
    ot_params.sigma_yaw_angle_deg = params.sigma_yaw_angle_deg;
    ot_params.cutoff_freq_of_velocity_lpf = params.cutoff_freq_of_velocity_lpf;
    object_tracker_.setParams(ot_params);
  }

  {
    ManeuverPredictor::Params mp_params;
    mp_params.lane_change_detection_method = params.lane_change_detection_method;
    mp_params.dist_threshold_to_bound = params.dist_threshold_to_bound;
    mp_params.time_threshold_to_bound = params.time_threshold_to_bound;
    mp_params.dist_ratio_threshold_to_left_bound = params.dist_ratio_threshold_to_left_bound;
    mp_params.dist_ratio_threshold_to_right_bound = params.dist_ratio_threshold_to_right_bound;
    mp_params.diff_dist_threshold_to_left_bound = params.diff_dist_threshold_to_left_bound;
    mp_params.diff_dist_threshold_to_right_bound = params.diff_dist_threshold_to_right_bound;
    mp_params.num_continuous_state_transition = params.num_continuous_state_transition;
    mp_params.history_time_length = params.history_time_length;
    maneuver_predictor_.setParams(mp_params);
  }

  {
    PathProcessor::Params pp_params;
    pp_params.lateral_control_time_horizon = params.lateral_control_time_horizon;
    pp_params.prediction_time_horizon = params.prediction_time_horizon;
    pp_params.prediction_time_horizon_rate_for_validate_lane_length =
      params.prediction_time_horizon_rate_for_validate_lane_length;
    pp_params.prediction_sampling_time_interval = params.prediction_sampling_time_interval;
    pp_params.min_velocity_for_map_based_prediction = params.min_velocity_for_map_based_prediction;
    pp_params.reference_path_resolution = params.reference_path_resolution;
    pp_params.check_lateral_acceleration_constraints = params.check_lateral_acceleration_constraints;
    pp_params.max_lateral_accel = params.max_lateral_accel;
    pp_params.min_acceleration_before_curve = params.min_acceleration_before_curve;
    pp_params.use_vehicle_acceleration = params.use_vehicle_acceleration;
    pp_params.speed_limit_multiplier = params.speed_limit_multiplier;
    pp_params.acceleration_exponential_half_life = params.acceleration_exponential_half_life;
    pp_params.consider_only_routable_neighbours = params.consider_only_routable_neighbours;
    path_processor_.setParams(pp_params);
  }
}

void PredictorVehicle::setLaneletMap(
  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr,
  std::shared_ptr<lanelet::routing::RoutingGraph> routing_graph_ptr,
  std::shared_ptr<lanelet::traffic_rules::TrafficRules> traffic_rules_ptr)
{
  object_tracker_.setLaneletMap(lanelet_map_ptr);
  object_tracker_.setRoutingGraph(routing_graph_ptr);
  maneuver_predictor_.setRoutingGraph(routing_graph_ptr);
  path_processor_.setLaneletMap(lanelet_map_ptr, routing_graph_ptr, traffic_rules_ptr);
}

void PredictorVehicle::setTimeKeeper(std::shared_ptr<autoware_utils::TimeKeeper> time_keeper_ptr)
{
  time_keeper_ = time_keeper_ptr;
  object_tracker_.setTimeKeeper(time_keeper_ptr);
  maneuver_predictor_.setTimeKeeper(time_keeper_ptr);
  path_processor_.setTimeKeeper(time_keeper_ptr);
}

void PredictorVehicle::removeOldHistory(double current_time, double buffer_time)
{
  object_tracker_.removeOldHistory(current_time, buffer_time);
}

std::optional<PredictedObject> PredictorVehicle::predict(
  const std_msgs::msg::Header & header, const TrackedObject & object,
  double objects_detected_time, visualization_msgs::msg::MarkerArray & debug_markers)
{
  return path_processor_.predict(header, object, objects_detected_time, debug_markers);
}

}  // namespace autoware::map_based_prediction
