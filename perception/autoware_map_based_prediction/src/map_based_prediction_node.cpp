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
  prediction_time_horizon_.vehicle = declare_parameter<double>("prediction_time_horizon.vehicle");
  prediction_time_horizon_.pedestrian =
    declare_parameter<double>("prediction_time_horizon.pedestrian");
  prediction_time_horizon_.unknown = declare_parameter<double>("prediction_time_horizon.unknown");
  lateral_control_time_horizon_ =
    declare_parameter<double>("lateral_control_time_horizon");  // [s] for lateral control point
  prediction_sampling_time_interval_ = declare_parameter<double>("prediction_sampling_delta_time");
  min_velocity_for_map_based_prediction_ =
    declare_parameter<double>("min_velocity_for_map_based_prediction");

  dist_threshold_for_searching_lanelet_ =
    declare_parameter<double>("dist_threshold_for_searching_lanelet");
  delta_yaw_threshold_for_searching_lanelet_ =
    declare_parameter<double>("delta_yaw_threshold_for_searching_lanelet");
  sigma_lateral_offset_ = declare_parameter<double>("sigma_lateral_offset");
  sigma_yaw_angle_deg_ = declare_parameter<double>("sigma_yaw_angle_deg");
  object_buffer_time_length_ = declare_parameter<double>("object_buffer_time_length");
  history_time_length_ = declare_parameter<double>("history_time_length");

  check_lateral_acceleration_constraints_ =
    declare_parameter<bool>("check_lateral_acceleration_constraints");
  max_lateral_accel_ = declare_parameter<double>("max_lateral_accel");
  min_acceleration_before_curve_ = declare_parameter<double>("min_acceleration_before_curve");

  {  // lane change detection
    lane_change_detection_method_ = declare_parameter<std::string>("lane_change_detection.method");

    // lane change detection by time_to_change_lane
    dist_threshold_to_bound_ = declare_parameter<double>(
      "lane_change_detection.time_to_change_lane.dist_threshold_for_lane_change_detection");  // 1m
    time_threshold_to_bound_ = declare_parameter<double>(
      "lane_change_detection.time_to_change_lane.time_threshold_for_lane_change_detection");
    cutoff_freq_of_velocity_lpf_ = declare_parameter<double>(
      "lane_change_detection.time_to_change_lane.cutoff_freq_of_velocity_for_lane_change_"
      "detection");

    // lane change detection by lat_diff_distance
    dist_ratio_threshold_to_left_bound_ = declare_parameter<double>(
      "lane_change_detection.lat_diff_distance.dist_ratio_threshold_to_left_bound");
    dist_ratio_threshold_to_right_bound_ = declare_parameter<double>(
      "lane_change_detection.lat_diff_distance.dist_ratio_threshold_to_right_bound");
    diff_dist_threshold_to_left_bound_ = declare_parameter<double>(
      "lane_change_detection.lat_diff_distance.diff_dist_threshold_to_left_bound");
    diff_dist_threshold_to_right_bound_ = declare_parameter<double>(
      "lane_change_detection.lat_diff_distance.diff_dist_threshold_to_right_bound");

    num_continuous_state_transition_ =
      declare_parameter<int>("lane_change_detection.num_continuous_state_transition");

    consider_only_routable_neighbours_ =
      declare_parameter<bool>("lane_change_detection.consider_only_routable_neighbours");
  }
  reference_path_resolution_ = declare_parameter<double>("reference_path_resolution");
  prediction_time_horizon_rate_for_validate_lane_length_ =
    declare_parameter<double>("prediction_time_horizon_rate_for_validate_shoulder_lane_length");

  use_vehicle_acceleration_ = declare_parameter<bool>("use_vehicle_acceleration");
  speed_limit_multiplier_ = declare_parameter<double>("speed_limit_multiplier");
  acceleration_exponential_half_life_ =
    declare_parameter<double>("acceleration_exponential_half_life");

  // initialize VRU predictor
  predictor_vru_ = std::make_unique<PredictorVru>(*this);

  // VRU parameters
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
      prediction_sampling_time_interval_, prediction_time_horizon_.pedestrian,
      crossing_intention_duration, no_crossing_intention_duration);
  }

  // debug parameter
  bool use_time_publisher = declare_parameter<bool>("publish_processing_time");
  bool use_time_keeper = declare_parameter<bool>("publish_processing_time_detail");
  bool use_debug_marker = declare_parameter<bool>("publish_debug_markers");

  // initialize path generator
  path_generator_ = std::make_shared<PathGenerator>(prediction_sampling_time_interval_);

  path_generator_->setUseVehicleAcceleration(use_vehicle_acceleration_);
  path_generator_->setAccelerationHalfLife(acceleration_exponential_half_life_);

  // subscribers
  sub_objects_ = this->create_subscription<TrackedObjects>(
    "~/input/objects", 1,
    std::bind(&MapBasedPredictionNode::objectsCallback, this, std::placeholders::_1));
  sub_map_ = this->create_subscription<LaneletMapBin>(
    "/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&MapBasedPredictionNode::mapCallback, this, std::placeholders::_1));

  // publishers
  pub_objects_ = this->create_publisher<PredictedObjects>("~/output/objects", rclcpp::QoS{1});

  // stopwatch
  stop_watch_ptr_ = std::make_unique<autoware_utils::StopWatch<std::chrono::milliseconds>>();
  stop_watch_ptr_->tic("cyclic_time");
  stop_watch_ptr_->tic("processing_time");

  {  // diagnostics
    diagnostics_interface_ptr_ =
      std::make_unique<autoware_utils::DiagnosticsInterface>(this, "map_based_prediction");

    // [s] -> [ms]
    processing_time_tolerance_ms_ = declare_parameter<double>("processing_time_tolerance") * 1e3;
    processing_time_consecutive_excess_tolerance_ms_ =
      declare_parameter<double>("processing_time_consecutive_excess_tolerance") * 1e3;
  }

  // debug publishers
  if (use_time_publisher) {
    processing_time_publisher_ =
      std::make_unique<autoware_utils::DebugPublisher>(this, "map_based_prediction");
    published_time_publisher_ = std::make_unique<autoware_utils::PublishedTimePublisher>(this);
  }

  // debug time keeper
  if (use_time_keeper) {
    detailed_processing_time_publisher_ =
      this->create_publisher<autoware_utils::ProcessingTimeDetail>(
        "~/debug/processing_time_detail_ms", 1);
    auto time_keeper = autoware_utils::TimeKeeper(detailed_processing_time_publisher_);
    time_keeper_ = std::make_shared<autoware_utils::TimeKeeper>(time_keeper);
    path_generator_->setTimeKeeper(time_keeper_);
    predictor_vru_->setTimeKeeper(time_keeper_);
  }

  // debug marker
  if (use_debug_marker) {
    pub_debug_markers_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>("maneuver", rclcpp::QoS{1});
  }
  // dynamic reconfigure
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&MapBasedPredictionNode::onParam, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult MapBasedPredictionNode::onParam(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils::update_param;

  update_param(parameters, "max_lateral_accel", max_lateral_accel_);
  update_param(parameters, "min_acceleration_before_curve", min_acceleration_before_curve_);
  update_param(
    parameters, "check_lateral_acceleration_constraints", check_lateral_acceleration_constraints_);
  update_param(parameters, "use_vehicle_acceleration", use_vehicle_acceleration_);
  update_param(parameters, "speed_limit_multiplier", speed_limit_multiplier_);
  update_param(
    parameters, "acceleration_exponential_half_life", acceleration_exponential_half_life_);

  path_generator_->setUseVehicleAcceleration(use_vehicle_acceleration_);
  path_generator_->setAccelerationHalfLife(acceleration_exponential_half_life_);

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
