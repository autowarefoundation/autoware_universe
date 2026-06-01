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

#include "map_based_prediction_node.hpp"

#include <autoware_utils/ros/update_param.hpp>

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace autoware::map_based_prediction
{
using autoware_utils::ScopedTimeTrack;

MapBasedPredictionNode::MapBasedPredictionNode(const rclcpp::NodeOptions & node_options)
: Node("map_based_prediction", node_options)
{
  // --- Prediction time horizons ---
  prediction_time_horizon_.vehicle = declare_parameter<double>("prediction_time_horizon.vehicle");
  prediction_time_horizon_.pedestrian =
    declare_parameter<double>("prediction_time_horizon.pedestrian");
  prediction_time_horizon_.unknown = declare_parameter<double>("prediction_time_horizon.unknown");

  const double prediction_sampling_time_interval =
    declare_parameter<double>("prediction_sampling_delta_time");

  // --- Vehicle predictor parameters ---
  PredictorVehicle::Params vehicle_params;
  vehicle_params.prediction_time_horizon = prediction_time_horizon_.vehicle;
  vehicle_params.prediction_sampling_time_interval = prediction_sampling_time_interval;
  vehicle_params.lateral_control_time_horizon =
    declare_parameter<double>("lateral_control_time_horizon");
  vehicle_params.min_velocity_for_map_based_prediction =
    declare_parameter<double>("min_velocity_for_map_based_prediction");
  vehicle_params.dist_threshold_for_searching_lanelet =
    declare_parameter<double>("dist_threshold_for_searching_lanelet");
  vehicle_params.delta_yaw_threshold_for_searching_lanelet =
    declare_parameter<double>("delta_yaw_threshold_for_searching_lanelet");
  vehicle_params.sigma_lateral_offset = declare_parameter<double>("sigma_lateral_offset");
  vehicle_params.sigma_yaw_angle_deg = declare_parameter<double>("sigma_yaw_angle_deg");
  object_buffer_time_length_ = declare_parameter<double>("object_buffer_time_length");
  vehicle_params.history_time_length = declare_parameter<double>("history_time_length");
  vehicle_params.check_lateral_acceleration_constraints =
    declare_parameter<bool>("check_lateral_acceleration_constraints");
  vehicle_params.max_lateral_accel = declare_parameter<double>("max_lateral_accel");
  vehicle_params.min_acceleration_before_curve =
    declare_parameter<double>("min_acceleration_before_curve");

  {
    vehicle_params.lane_change_detection_method =
      declare_parameter<std::string>("lane_change_detection.method");
    vehicle_params.dist_threshold_to_bound = declare_parameter<double>(
      "lane_change_detection.time_to_change_lane.dist_threshold_for_lane_change_detection");
    vehicle_params.time_threshold_to_bound = declare_parameter<double>(
      "lane_change_detection.time_to_change_lane.time_threshold_for_lane_change_detection");
    vehicle_params.cutoff_freq_of_velocity_lpf = declare_parameter<double>(
      "lane_change_detection.time_to_change_lane.cutoff_freq_of_velocity_for_lane_change_"
      "detection");
    vehicle_params.dist_ratio_threshold_to_left_bound = declare_parameter<double>(
      "lane_change_detection.lat_diff_distance.dist_ratio_threshold_to_left_bound");
    vehicle_params.dist_ratio_threshold_to_right_bound = declare_parameter<double>(
      "lane_change_detection.lat_diff_distance.dist_ratio_threshold_to_right_bound");
    vehicle_params.diff_dist_threshold_to_left_bound = declare_parameter<double>(
      "lane_change_detection.lat_diff_distance.diff_dist_threshold_to_left_bound");
    vehicle_params.diff_dist_threshold_to_right_bound = declare_parameter<double>(
      "lane_change_detection.lat_diff_distance.diff_dist_threshold_to_right_bound");
    vehicle_params.num_continuous_state_transition =
      declare_parameter<int>("lane_change_detection.num_continuous_state_transition");
    vehicle_params.consider_only_routable_neighbours =
      declare_parameter<bool>("lane_change_detection.consider_only_routable_neighbours");
  }

  vehicle_params.reference_path_resolution = declare_parameter<double>("reference_path_resolution");
  vehicle_params.prediction_time_horizon_rate_for_validate_lane_length =
    declare_parameter<double>("prediction_time_horizon_rate_for_validate_shoulder_lane_length");
  vehicle_params.use_vehicle_acceleration = declare_parameter<bool>("use_vehicle_acceleration");
  vehicle_params.speed_limit_multiplier = declare_parameter<double>("speed_limit_multiplier");
  vehicle_params.acceleration_exponential_half_life =
    declare_parameter<double>("acceleration_exponential_half_life");

  predictor_vehicle_ = std::make_shared<PredictorVehicle>(*this);
  predictor_vehicle_->setParams(vehicle_params);

  // --- VRU predictor ---
  predictor_vru_ = std::make_unique<PredictorVru>(*this);

  remember_lost_crosswalk_users_ =
    declare_parameter<bool>("use_crosswalk_user_history.remember_lost_users");
  {
    bool match_lost_and_appeared_crosswalk_users =
      declare_parameter<bool>("use_crosswalk_user_history.match_lost_and_appeared_users");
    double min_crosswalk_user_velocity = declare_parameter<double>("min_crosswalk_user_velocity");
    double max_crosswalk_user_delta_yaw_threshold_for_lanelet =
      declare_parameter<double>("max_crosswalk_user_delta_yaw_threshold_for_lanelet");
    double max_crosswalk_user_on_road_distance =
      declare_parameter<double>("max_crosswalk_user_on_road_distance");
    bool use_crosswalk_signal =
      declare_parameter<bool>("crosswalk_with_signal.use_crosswalk_signal");
    double threshold_velocity_assumed_as_stopping =
      declare_parameter<double>("crosswalk_with_signal.threshold_velocity_assumed_as_stopping");
    double crossing_intention_duration = declare_parameter<double>("crossing_intention_duration");
    double no_crossing_intention_duration =
      declare_parameter<double>("no_crossing_intention_duration");
    std::vector<double> distance_set_for_no_intention_to_walk =
      declare_parameter<std::vector<double>>(
        "crosswalk_with_signal.distance_set_for_no_intention_to_walk");
    std::vector<double> timeout_set_for_no_intention_to_walk =
      declare_parameter<std::vector<double>>(
        "crosswalk_with_signal.timeout_set_for_no_intention_to_walk");
    predictor_vru_->setParameters(
      match_lost_and_appeared_crosswalk_users, min_crosswalk_user_velocity,
      max_crosswalk_user_delta_yaw_threshold_for_lanelet, max_crosswalk_user_on_road_distance,
      use_crosswalk_signal, threshold_velocity_assumed_as_stopping,
      distance_set_for_no_intention_to_walk, timeout_set_for_no_intention_to_walk,
      prediction_sampling_time_interval, prediction_time_horizon_.pedestrian,
      crossing_intention_duration, no_crossing_intention_duration);
  }

  // --- Path generator for unknown-class objects ---
  path_generator_ = std::make_shared<PathGenerator>(prediction_sampling_time_interval);

  // --- Debug parameters ---
  const bool use_time_publisher = declare_parameter<bool>("publish_processing_time");
  const bool use_time_keeper = declare_parameter<bool>("publish_processing_time_detail");
  const bool use_debug_marker = declare_parameter<bool>("publish_debug_markers");

  // --- ROS subscriptions ---
  sub_objects_ = this->create_subscription<TrackedObjects>(
    "~/input/objects", 1,
    std::bind(&MapBasedPredictionNode::objectsCallback, this, std::placeholders::_1));
  sub_map_ = this->create_subscription<LaneletMapBin>(
    "/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&MapBasedPredictionNode::mapCallback, this, std::placeholders::_1));

  // --- ROS publishers ---
  pub_objects_ = this->create_publisher<PredictedObjects>("~/output/objects", rclcpp::QoS{1});

  // --- StopWatch ---
  stop_watch_ptr_ = std::make_unique<autoware_utils::StopWatch<std::chrono::milliseconds>>();
  stop_watch_ptr_->tic("cyclic_time");
  stop_watch_ptr_->tic("processing_time");

  // --- Diagnostics ---
  {
    diagnostics_interface_ptr_ =
      std::make_unique<autoware_utils::DiagnosticsInterface>(this, "map_based_prediction");
    processing_time_tolerance_ms_ = declare_parameter<double>("processing_time_tolerance") * 1e3;
    processing_time_consecutive_excess_tolerance_ms_ =
      declare_parameter<double>("processing_time_consecutive_excess_tolerance") * 1e3;
  }

  // --- Optional debug publishers ---
  if (use_time_publisher) {
    processing_time_publisher_ =
      std::make_unique<autoware_utils::DebugPublisher>(this, "map_based_prediction");
    published_time_publisher_ = std::make_unique<autoware_utils::PublishedTimePublisher>(this);
  }

  if (use_time_keeper) {
    detailed_processing_time_publisher_ =
      this->create_publisher<autoware_utils::ProcessingTimeDetail>(
        "~/debug/processing_time_detail_ms", 1);
    auto time_keeper = autoware_utils::TimeKeeper(detailed_processing_time_publisher_);
    time_keeper_ = std::make_shared<autoware_utils::TimeKeeper>(time_keeper);
    predictor_vehicle_->setTimeKeeper(time_keeper_);
    predictor_vru_->setTimeKeeper(time_keeper_);
  }

  if (use_debug_marker) {
    pub_debug_markers_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>("maneuver", rclcpp::QoS{1});
  }

  // --- Dynamic reconfigure ---
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&MapBasedPredictionNode::onParam, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult MapBasedPredictionNode::onParam(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils::update_param;

  auto vehicle_params = predictor_vehicle_->getParams();
  update_param(parameters, "max_lateral_accel", vehicle_params.max_lateral_accel);
  update_param(
    parameters, "min_acceleration_before_curve", vehicle_params.min_acceleration_before_curve);
  update_param(
    parameters, "check_lateral_acceleration_constraints",
    vehicle_params.check_lateral_acceleration_constraints);
  update_param(parameters, "use_vehicle_acceleration", vehicle_params.use_vehicle_acceleration);
  update_param(parameters, "speed_limit_multiplier", vehicle_params.speed_limit_multiplier);
  update_param(
    parameters, "acceleration_exponential_half_life",
    vehicle_params.acceleration_exponential_half_life);
  predictor_vehicle_->setParams(vehicle_params);

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

void MapBasedPredictionNode::publish(
  const PredictedObjects & output, const visualization_msgs::msg::MarkerArray & debug_markers) const
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  pub_objects_->publish(output);
  if (published_time_publisher_)
    published_time_publisher_->publish_if_subscribed(pub_objects_, output.header.stamp);
  if (pub_debug_markers_) pub_debug_markers_->publish(debug_markers);
}

}  // namespace autoware::map_based_prediction

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::map_based_prediction::MapBasedPredictionNode)
