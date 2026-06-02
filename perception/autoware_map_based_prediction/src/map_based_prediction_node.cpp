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

#include "map_based_prediction_node.hpp"

#include "autoware/map_based_prediction/params.hpp"

#include <autoware_utils/ros/update_param.hpp>

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace autoware::map_based_prediction
{

MapBasedPredictionNode::MapBasedPredictionNode(const rclcpp::NodeOptions & node_options)
: Node("map_based_prediction", node_options)
{
  // --- Prediction time horizons ---
  PredictionTimeHorizonParams time_horizon;
  time_horizon.vehicle = declare_parameter<double>("prediction_time_horizon.vehicle");
  time_horizon.pedestrian = declare_parameter<double>("prediction_time_horizon.pedestrian");
  time_horizon.unknown = declare_parameter<double>("prediction_time_horizon.unknown");

  const double prediction_sampling_dt = declare_parameter<double>("prediction_sampling_delta_time");

  // --- Vehicle predictor ---
  PredictorVehicle::Params vehicle_params;
  {
    auto & ot = vehicle_params.object_tracker;
    ot.dist_threshold_for_searching_lanelet =
      declare_parameter<double>("dist_threshold_for_searching_lanelet");
    ot.delta_yaw_threshold_for_searching_lanelet =
      declare_parameter<double>("delta_yaw_threshold_for_searching_lanelet");
    ot.sigma_lateral_offset = declare_parameter<double>("sigma_lateral_offset");
    ot.sigma_yaw_angle_deg = declare_parameter<double>("sigma_yaw_angle_deg");
    ot.cutoff_freq_of_velocity_lpf = declare_parameter<double>(
      "lane_change_detection.time_to_change_lane.cutoff_freq_of_velocity_for_lane_change_"
      "detection");
  }
  {
    auto & mp = vehicle_params.maneuver_predictor;
    mp.history_time_length = declare_parameter<double>("history_time_length");
    mp.lane_change_detection_method =
      declare_parameter<std::string>("lane_change_detection.method");
    mp.dist_threshold_to_bound = declare_parameter<double>(
      "lane_change_detection.time_to_change_lane.dist_threshold_for_lane_change_detection");
    mp.time_threshold_to_bound = declare_parameter<double>(
      "lane_change_detection.time_to_change_lane.time_threshold_for_lane_change_detection");
    mp.dist_ratio_threshold_to_left_bound = declare_parameter<double>(
      "lane_change_detection.lat_diff_distance.dist_ratio_threshold_to_left_bound");
    mp.dist_ratio_threshold_to_right_bound = declare_parameter<double>(
      "lane_change_detection.lat_diff_distance.dist_ratio_threshold_to_right_bound");
    mp.diff_dist_threshold_to_left_bound = declare_parameter<double>(
      "lane_change_detection.lat_diff_distance.diff_dist_threshold_to_left_bound");
    mp.diff_dist_threshold_to_right_bound = declare_parameter<double>(
      "lane_change_detection.lat_diff_distance.diff_dist_threshold_to_right_bound");
    mp.num_continuous_state_transition =
      declare_parameter<int>("lane_change_detection.num_continuous_state_transition");
  }
  {
    auto & pp = vehicle_params.path_processor;
    pp.prediction_time_horizon = time_horizon.vehicle;
    pp.prediction_sampling_time_interval = prediction_sampling_dt;
    pp.lateral_control_time_horizon = declare_parameter<double>("lateral_control_time_horizon");
    pp.min_velocity_for_map_based_prediction =
      declare_parameter<double>("min_velocity_for_map_based_prediction");
    pp.reference_path_resolution = declare_parameter<double>("reference_path_resolution");
    pp.prediction_time_horizon_rate_for_validate_lane_length =
      declare_parameter<double>("prediction_time_horizon_rate_for_validate_shoulder_lane_length");
    pp.check_lateral_acceleration_constraints =
      declare_parameter<bool>("check_lateral_acceleration_constraints");
    pp.max_lateral_accel = declare_parameter<double>("max_lateral_accel");
    pp.min_acceleration_before_curve = declare_parameter<double>("min_acceleration_before_curve");
    pp.use_vehicle_acceleration = declare_parameter<bool>("use_vehicle_acceleration");
    pp.speed_limit_multiplier = declare_parameter<double>("speed_limit_multiplier");
    pp.acceleration_exponential_half_life =
      declare_parameter<double>("acceleration_exponential_half_life");
    pp.consider_only_routable_neighbours =
      declare_parameter<bool>("lane_change_detection.consider_only_routable_neighbours");
  }
  state_.predictor_vehicle = std::make_shared<PredictorVehicle>(*this);
  state_.predictor_vehicle->setParams(vehicle_params);

  // --- VRU predictor ---
  state_.predictor_vru = std::make_shared<PredictorVru>(*this);
  {
    PredictorVru::Params vru_params;
    vru_params.prediction_time_horizon = time_horizon.pedestrian;
    vru_params.prediction_sampling_time_interval = prediction_sampling_dt;
    vru_params.min_crosswalk_user_velocity =
      declare_parameter<double>("min_crosswalk_user_velocity");
    vru_params.max_crosswalk_user_delta_yaw_threshold_for_lanelet =
      declare_parameter<double>("max_crosswalk_user_delta_yaw_threshold_for_lanelet");
    vru_params.max_crosswalk_user_on_road_distance =
      declare_parameter<double>("max_crosswalk_user_on_road_distance");
    vru_params.use_crosswalk_signal =
      declare_parameter<bool>("crosswalk_with_signal.use_crosswalk_signal");
    vru_params.traffic_signal.threshold_velocity_assumed_as_stopping =
      declare_parameter<double>("crosswalk_with_signal.threshold_velocity_assumed_as_stopping");
    vru_params.traffic_signal.distance_set_for_no_intention_to_walk =
      declare_parameter<std::vector<double>>(
        "crosswalk_with_signal.distance_set_for_no_intention_to_walk");
    vru_params.traffic_signal.timeout_set_for_no_intention_to_walk =
      declare_parameter<std::vector<double>>(
        "crosswalk_with_signal.timeout_set_for_no_intention_to_walk");
    vru_params.history.match_lost_and_appeared =
      declare_parameter<bool>("use_crosswalk_user_history.match_lost_and_appeared_users");
    vru_params.history.crossing_intention_duration =
      declare_parameter<double>("crossing_intention_duration");
    vru_params.history.no_crossing_intention_duration =
      declare_parameter<double>("no_crossing_intention_duration");
    state_.predictor_vru->setParams(vru_params);
  }

  // --- Path generator for unknown-class objects ---
  state_.path_generator = std::make_shared<PathGenerator>(prediction_sampling_dt);

  // --- Node params ---
  state_.params.object_buffer_time_length = declare_parameter<double>("object_buffer_time_length");
  state_.params.remember_lost_crosswalk_users =
    declare_parameter<bool>("use_crosswalk_user_history.remember_lost_users");
  state_.params.prediction_time_horizon_unknown = time_horizon.unknown;

  // --- Callbacks ---
  map_callback_ = std::make_unique<MapCallback>(this, state_);
  objects_callback_ = std::make_unique<ObjectsCallback>(this, state_);
  diagnostics_ = std::make_unique<Diagnostics>(this);

  // --- ROS publishers ---
  objects_callback_->setObjectsPublisher(
    this->create_publisher<PredictedObjects>("~/output/objects", rclcpp::QoS{1}));

  // --- Debug parameters ---
  const bool use_time_publisher = declare_parameter<bool>("publish_processing_time");
  const bool use_time_keeper = declare_parameter<bool>("publish_processing_time_detail");
  const bool use_debug_marker = declare_parameter<bool>("publish_debug_markers");

  if (use_time_publisher) {
    diagnostics_->setPublishedTimePublisher(
      std::make_unique<autoware_utils::PublishedTimePublisher>(this));
    diagnostics_->setProcessingTimePublisher(
      std::make_unique<autoware_utils::DebugPublisher>(this, "map_based_prediction"));
  }

  if (use_time_keeper) {
    detailed_processing_time_publisher_ =
      this->create_publisher<autoware_utils::ProcessingTimeDetail>(
        "~/debug/processing_time_detail_ms", 1);
    auto time_keeper = autoware_utils::TimeKeeper(detailed_processing_time_publisher_);
    state_.time_keeper = std::make_shared<autoware_utils::TimeKeeper>(time_keeper);
    state_.predictor_vehicle->setTimeKeeper(state_.time_keeper);
    state_.predictor_vru->setTimeKeeper(state_.time_keeper);
  }

  if (use_debug_marker) {
    objects_callback_->setDebugMarkersPublisher(
      this->create_publisher<visualization_msgs::msg::MarkerArray>("maneuver", rclcpp::QoS{1}));
  }

  // --- Diagnostics params + link ---
  {
    const double processing_time_tolerance_ms =
      declare_parameter<double>("processing_time_tolerance") * 1e3;
    const double processing_time_consecutive_excess_tolerance_ms =
      declare_parameter<double>("processing_time_consecutive_excess_tolerance") * 1e3;
    Diagnostics::Params diag_params;
    diag_params.processing_time_tolerance_ms = processing_time_tolerance_ms;
    diag_params.processing_time_consecutive_excess_tolerance_ms =
      processing_time_consecutive_excess_tolerance_ms;
    diagnostics_->setParams(diag_params);
    objects_callback_->setDiagnostics(diagnostics_.get());
  }

  // --- ROS subscriptions ---
  sub_objects_ = this->create_subscription<TrackedObjects>(
    "~/input/objects", 1,
    [this](const TrackedObjects::ConstSharedPtr msg) { objects_callback_->objectsCallback(msg); });
  sub_map_ = this->create_subscription<LaneletMapBin>(
    "/vector_map", rclcpp::QoS{1}.transient_local(),
    [this](const LaneletMapBin::ConstSharedPtr msg) { map_callback_->mapCallback(msg); });

  // --- Dynamic reconfigure ---
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&MapBasedPredictionNode::onParam, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult MapBasedPredictionNode::onParam(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils::update_param;

  auto vehicle_params = state_.predictor_vehicle->getParams();
  auto & pp = vehicle_params.path_processor;
  update_param(parameters, "max_lateral_accel", pp.max_lateral_accel);
  update_param(parameters, "min_acceleration_before_curve", pp.min_acceleration_before_curve);
  update_param(
    parameters, "check_lateral_acceleration_constraints",
    pp.check_lateral_acceleration_constraints);
  update_param(parameters, "use_vehicle_acceleration", pp.use_vehicle_acceleration);
  update_param(parameters, "speed_limit_multiplier", pp.speed_limit_multiplier);
  update_param(
    parameters, "acceleration_exponential_half_life", pp.acceleration_exponential_half_life);
  state_.predictor_vehicle->setParams(vehicle_params);

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

}  // namespace autoware::map_based_prediction

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::map_based_prediction::MapBasedPredictionNode)
