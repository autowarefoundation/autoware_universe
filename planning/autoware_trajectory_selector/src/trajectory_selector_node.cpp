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

#include "autoware/trajectory_selector/trajectory_selector_node.hpp"

#include <memory>
#include <string>

namespace autoware::trajectory_selector
{
TrajectorySelectorNode::TrajectorySelectorNode(const rclcpp::NodeOptions & node_options)
: Node{"trajectory_selector_node", node_options}
{
  subscribers();
  publishers();

  concatenator_ptr_ = std::make_unique<trajectory_concatenator::TrajectoryConcatenatorWrapper>(
    *this, get_node_parameters_interface());

  validator_ptr_ = std::make_unique<trajectory_validator::TrajectoryValidatorWrapper>(
    *this, get_node_parameters_interface(),
    autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo(), time_keeper_);

  constexpr int64_t timer_period_ms = 100;
  timer_ = rclcpp::create_timer(
    this, get_clock(), std::chrono::milliseconds(timer_period_ms),
    std::bind(&TrajectorySelectorNode::on_timer, this));
}

void TrajectorySelectorNode::subscribers()
{
  sub_map_ = create_subscription<LaneletMapBin>(
    "~/input/lanelet2_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&TrajectorySelectorNode::map_callback, this, std::placeholders::_1));

  sub_trajectories_generative_ = create_subscription<CandidateTrajectories>(
    "~/input/trajectories_generative", 1,
    std::bind(&TrajectorySelectorNode::on_trajectories, this, std::placeholders::_1));

  sub_trajectories_backup_ = create_subscription<CandidateTrajectories>(
    "~/input/trajectories_backup", 1,
    std::bind(&TrajectorySelectorNode::on_trajectories, this, std::placeholders::_1));
}

void TrajectorySelectorNode::publishers()
{
  pub_trajectories_ = create_publisher<CandidateTrajectories>("~/output/trajectories", 1);

  pub_processing_time_detail_ = create_publisher<autoware_utils_debug::ProcessingTimeDetail>(
    "~/debug/processing_time_detail_ms/trajectory_selector", 1);
  time_keeper_ = std::make_shared<autoware_utils_debug::TimeKeeper>(pub_processing_time_detail_);
}

void TrajectorySelectorNode::map_callback(const LaneletMapBin::ConstSharedPtr msg)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  lanelet_map_ptr_ = autoware::experimental::lanelet2_utils::remove_const(
    autoware::experimental::lanelet2_utils::from_autoware_map_msgs(*msg));
}

void TrajectorySelectorNode::on_trajectories(const CandidateTrajectories::ConstSharedPtr msg)
{
  concatenator_ptr_->add_candidate(*msg);
}

tl::expected<trajectory_validator::FilterContext, std::string>
TrajectorySelectorNode::take_validator_data()
{
  trajectory_validator::FilterContext context;

  context.odometry = sub_odometry_.take_data();
  if (!context.odometry) {
    return tl::make_unexpected("Failed to take odometry data");
  }

  context.predicted_objects = sub_objects_.take_data();
  if (!context.predicted_objects) {
    return tl::make_unexpected("Failed to take predicted objects data");
  }

  context.acceleration = sub_acceleration_.take_data();
  if (!context.acceleration) {
    return tl::make_unexpected("Failed to take acceleration data");
  }

  context.traffic_light_signals = sub_traffic_lights_.take_data();

  context.lanelet_map = lanelet_map_ptr_;
  if (!context.lanelet_map) {
    return tl::make_unexpected("Lanelet map is not available");
  }

  if (context.lanelet_map->laneletLayer.empty()) {
    return tl::make_unexpected("Lanelet map does not contain any lanelets");
  }

  return context;
}

void TrajectorySelectorNode::on_timer()
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  auto concatenated_trajectories = concatenator_ptr_->get_concatenated();

  if (concatenated_trajectories.candidate_trajectories.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000, "No concatenated trajectories received yet");
    return;
  }

  auto context_opt = take_validator_data();
  if (!context_opt) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "%s", context_opt.error().c_str());
    return;
  }

  auto validated_trajectories =
    validator_ptr_->validate_trajectories(concatenated_trajectories, context_opt.value());

  pub_trajectories_->publish(validated_trajectories);
}

}  // namespace autoware::trajectory_selector

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::trajectory_selector::TrajectorySelectorNode)
