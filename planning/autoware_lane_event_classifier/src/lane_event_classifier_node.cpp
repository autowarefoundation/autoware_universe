// Copyright 2026 TIER IV, Inc.
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

#include "autoware_lane_event_classifier/lane_event_classifier_node.hpp"

#include <autoware/universe_utils/system/stop_watch.hpp>
#include <autoware_lane_event_classifier/geometry_utils.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::lane_event_classifier
{

LaneEventClassifierNode::LaneEventClassifierNode(const rclcpp::NodeOptions & node_options)
: Node("lane_event_classifier", node_options),
  param_listener_{this->get_node_parameters_interface()},
  vehicle_info_{autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo()},
  debug_{*this}
{
  params_ = param_listener_.get_params();
  build_classifiers();

  sub_map_ = create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
    "/map/vector_map", rclcpp::QoS{1}.transient_local(),
    [this](const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr & msg) {
      map_callback(msg);
    });

  pub_driving_factor_ = create_publisher<autoware_lane_event_classifier::msg::DrivingFactor>(
    "/planning/driving_factor", rclcpp::QoS{1});

  sub_trajectory_ = create_subscription<autoware_planning_msgs::msg::Trajectory>(
    "/planning/trajectory", rclcpp::QoS{1},
    [this](const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr & msg) {
      on_trajectory(msg);
    });
}

void LaneEventClassifierNode::build_classifiers()
{
  lane_following_checker_ = LaneFollowingChecker();

  // Classifiers are constructed here (no plugin/pluginlib): the node owns a vector of concrete
  // LaneEventClassifierBase implementations and iterates it in on_trajectory(). Each classifier's
  // real logic and its enable/config parameters are added in its own follow-up PR; today they are
  // no-op stubs so the loading and aggregation path is exercised end-to-end.
  classifiers_.clear();
  classifiers_.emplace_back(std::make_unique<LaneChangeClassifier>(true));
  classifiers_.emplace_back(std::make_unique<IntentionalCrossingClassifier>(true));
}

void LaneEventClassifierNode::map_callback(
  const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr & msg)
{
  // Stash the raw map. Converting it into a lanelet map / routing graph is the LaneTracker's job,
  // which is added in a follow-up PR.
  map_msg_ptr_ = msg;
}

tl::expected<void, std::string> LaneEventClassifierNode::take_data(
  const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr & trajectory_msg)
{
  input_.trajectory_ptr = trajectory_msg;

  const auto odometry_msg = sub_odometry_.take_data();
  if (!odometry_msg) {
    return tl::make_unexpected("odometry_msg not available");
  }
  input_.odometry_ptr = odometry_msg;

  auto objects_msg = sub_objects_.take_data();
  if (!objects_msg) {
    return tl::make_unexpected("perceived objects_msg not available");
  }
  input_.objects_ptr = objects_msg;

  if (!map_msg_ptr_) {
    return tl::make_unexpected("lanelet map not yet available");
  }

  auto route_msg = sub_route_.take_data();
  if (!route_msg) {
    return tl::make_unexpected("route_msg not available");
  }

  if (route_msg->segments.empty()) {
    return tl::make_unexpected("route_msg does not contain any segments");
  }

  if (!input_.route_ptr || route_msg->uuid != input_.route_ptr->uuid) {
    input_.route_ptr = route_msg;
  }

  if (!input_.vehicle_info_ptr) {
    input_.vehicle_info_ptr =
      std::make_unique<autoware::vehicle_info_utils::VehicleInfo>(vehicle_info_);
  }

  // Turn indicator is optional: keep the previous / default value when it is unavailable rather
  // than failing the cycle (the blinker confidence signal just stays inactive).
  if (const auto turn_indicators_msg = sub_turn_indicators_.take_data()) {
    input_.turn_indicator = turn_indicators_msg->report;
  }

  input_.footprint = compute_footprint(input_);

  return {};
}

void LaneEventClassifierNode::on_trajectory(
  const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr & trajectory_msg)
{
  if (param_listener_.is_old(params_)) {
    params_ = param_listener_.get_params();
    build_classifiers();
  }

  autoware::universe_utils::StopWatch<std::chrono::milliseconds> stop_watch;

  autoware_lane_event_classifier::msg::DrivingFactor out;
  out.header.stamp = trajectory_msg->header.stamp;
  out.header.frame_id = "map";

  const auto lane_event_inputs_updated = take_data(trajectory_msg);
  if (!lane_event_inputs_updated) {
    debug_.log_warn(lane_event_inputs_updated.error());
    out.driving_state.state = DrivingState::UNKNOWN;
    pub_driving_factor_->publish(out);
    return;
  }

  stop_watch.tic("lane_following");
  const auto lane_following_result = lane_following_checker_.evaluate();
  const double lane_following_time_ms = stop_watch.toc("lane_following");

  uint8_t current_state_val = DrivingState::LANE_FOLLOWING;
  bool is_any_event_active = false;
  std::vector<std::pair<std::string, double>> classifier_processing_times_ms;
  classifier_processing_times_ms.reserve(classifiers_.size());
  for (auto & classifier : classifiers_) {
    if (!classifier->is_enabled()) {
      continue;
    }
    stop_watch.tic(classifier->name());
    classifier->update(input_);
    classifier_processing_times_ms.emplace_back(
      classifier->name(), stop_watch.toc(classifier->name()));
    const uint8_t candidate_state = classifier->get_state();
    // Only a confirmed event counts; LANE_FOLLOWING or UNKNOWN from a classifier is no event.
    if (
      candidate_state == DrivingState::LANE_FOLLOWING || candidate_state == DrivingState::UNKNOWN) {
      continue;
    }
    if (!is_any_event_active) {
      current_state_val = candidate_state;  // first confirmed classifier wins (priority order)
    }
    is_any_event_active = true;
  }
  // No confirmed event: fall back to the gate. A departure with no classified event is UNKNOWN.
  if (!is_any_event_active && !lane_following_result.is_following) {
    current_state_val = DrivingState::UNKNOWN;
  }

  debug_.log_state(current_state_val, input_, lane_following_result, classifiers_);

  out.driving_state.state = current_state_val;
  pub_driving_factor_->publish(out);

  const double total_time_ms = stop_watch.toc();

  std::vector<std::pair<std::string, double>> section_times_ms;
  section_times_ms.reserve(classifier_processing_times_ms.size() + 1);
  section_times_ms.emplace_back("lane_following", lane_following_time_ms);
  section_times_ms.insert(
    section_times_ms.end(), classifier_processing_times_ms.begin(),
    classifier_processing_times_ms.end());
  debug_.publish_processing_time(trajectory_msg->header.stamp, total_time_ms, section_times_ms);
}
}  // namespace autoware::lane_event_classifier

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::lane_event_classifier::LaneEventClassifierNode)
