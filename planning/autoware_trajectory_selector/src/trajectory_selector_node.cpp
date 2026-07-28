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

#include <autoware_utils_uuid/uuid_helper.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>

#include <memory>
#include <string>
#include <unordered_map>

namespace autoware::trajectory_selector
{
TrajectorySelectorNode::TrajectorySelectorNode(const rclcpp::NodeOptions & node_options)
: Node{"trajectory_selector_node", node_options},
  route_handler_ptr_{std::make_shared<route_handler::RouteHandler>()}
{
  subscribers();
  publishers();

  concatenator_ptr_ = std::make_unique<trajectory_concatenator::TrajectoryConcatenatorWrapper>(
    *this, get_node_parameters_interface());

  validator_ptr_ = std::make_unique<trajectory_validator::TrajectoryValidatorWrapper>(
    *this, get_node_parameters_interface(),
    autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo(), time_keeper_);

  ranker_ptr_ = std::make_unique<trajectory_ranker::TrajectoryRankerWrapper>(
    *this, get_node_parameters_interface(),
    autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo(), time_keeper_);

  selector_params_ = selector_params_listener_.get_params();
  selector_params_listener_.setUserCallback([&](const auto &) { update_parameters(); });
  update_fallback_timer();
}

void TrajectorySelectorNode::subscribers()
{
  sub_map_ = create_subscription<LaneletMapBin>(
    "~/input/lanelet2_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&TrajectorySelectorNode::map_callback, this, std::placeholders::_1));

  sub_route_ = create_subscription<LaneletRoute>(
    "~/input/route", rclcpp::QoS{1}.transient_local(),
    std::bind(&TrajectorySelectorNode::route_callback, this, std::placeholders::_1));

  sub_trajectories_generative_ = create_subscription<CandidateTrajectories>(
    "~/input/trajectories_generative", 1,
    std::bind(&TrajectorySelectorNode::on_anchor_trajectories, this, std::placeholders::_1));

  sub_trajectories_backup_ = create_subscription<CandidateTrajectories>(
    "~/input/trajectories_backup", 1,
    std::bind(&TrajectorySelectorNode::on_trajectories, this, std::placeholders::_1));
}

void TrajectorySelectorNode::publishers()
{
  pub_concatenated_trajectories_ =
    create_publisher<CandidateTrajectories>("~/output/concatenated_trajectories", 1);
  pub_validated_trajectories_ =
    create_publisher<CandidateTrajectories>("~/output/validated_trajectories", 1);
  pub_scored_trajectories_ =
    create_publisher<ScoredCandidateTrajectories>("~/output/scored_trajectories", 1);
  pub_processing_time_detail_ = create_publisher<autoware_utils_debug::ProcessingTimeDetail>(
    "~/debug/processing_time_detail_ms/trajectory_selector", 1);
  time_keeper_ = std::make_shared<autoware_utils_debug::TimeKeeper>(pub_processing_time_detail_);
}

void TrajectorySelectorNode::map_callback(
  const AUTOWARE_MESSAGE_CONST_SHARED_PTR(LaneletMapBin) & msg)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  lanelet_map_ptr_ = autoware::experimental::lanelet2_utils::remove_const(
    autoware::experimental::lanelet2_utils::from_autoware_map_msgs(*msg));
  if (msg != nullptr) route_handler_ptr_->setMap(*msg);
}

void TrajectorySelectorNode::route_callback(
  const AUTOWARE_MESSAGE_CONST_SHARED_PTR(LaneletRoute) & msg)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  route_ptr_ = std::make_shared<const LaneletRoute>(*msg);
  if (msg != nullptr) route_handler_ptr_->setRoute(*msg);
}

void TrajectorySelectorNode::on_anchor_trajectories(
  const AUTOWARE_MESSAGE_CONST_SHARED_PTR(CandidateTrajectories) & msg)
{
  concatenator_ptr_->add_candidate(*msg);
  process_trajectories();
  timer_->reset();
}
void TrajectorySelectorNode::on_trajectories(
  const AUTOWARE_MESSAGE_CONST_SHARED_PTR(CandidateTrajectories) & msg)
{
  concatenator_ptr_->add_candidate(*msg);
}

tl::expected<trajectory_validator::FilterContext, std::string>
TrajectorySelectorNode::take_validator_data()
{
  trajectory_validator::FilterContext context;

  context.odometry = sub_odometry_->take_data();
  if (!context.odometry) {
    return tl::make_unexpected("Failed to take odometry data");
  }

  context.predicted_objects = sub_objects_->take_data();
  if (!context.predicted_objects) {
    return tl::make_unexpected("Failed to take predicted objects data");
  }

  context.acceleration = sub_acceleration_->take_data();
  if (!context.acceleration) {
    return tl::make_unexpected("Failed to take acceleration data");
  }

  context.traffic_light_signals = sub_traffic_lights_->take_data();
  if (!context.traffic_light_signals) {
    // No traffic light input means no known signals, not an unavailable input.
    context.traffic_light_signals =
      std::make_shared<autoware_perception_msgs::msg::TrafficLightGroupArray>();
  }

  context.route = route_ptr_;

  context.lanelet_map = lanelet_map_ptr_;
  if (!context.lanelet_map) {
    return tl::make_unexpected("Lanelet map is not available");
  }

  if (context.lanelet_map->laneletLayer.empty()) {
    return tl::make_unexpected("Lanelet map does not contain any lanelets");
  }

  return context;
}

trajectory_ranker::RankerContext TrajectorySelectorNode::take_ranker_data(
  const CandidateTrajectories & candidate_trajectories)
{
  trajectory_ranker::RankerContext context;
  context.route_handler = route_handler_ptr_;
  context.odometry = sub_odometry_->take_data();
  context.generator_info = candidate_trajectories.generator_info;
  return context;
}

RiskLevel::_level_type get_trajectory_risk_level(
  const CandidateTrajectory & traj, const ValidationReports & validation_reports)
{
  auto itr = std::find_if(
    validation_reports.begin(), validation_reports.end(),
    [&](const auto & report) { return traj.generator_id == report.generator_id; });
  if (itr == validation_reports.end()) {
    return RiskLevel::FATAL;
  }
  return itr->risk.level;
}

std::optional<TrajectorySource> generator_name_prefix_to_source(
  const std::string & generator_name_prefix)
{
  static const std::unordered_map<std::string, TrajectorySource> generator_name_prefix_to_source = {
    {"DiffusionPlanner_", TrajectorySource::DIFFUSION_PLANNER},
    {"MinimumRuleBasedPlanner_Go", TrajectorySource::BACKUP_PLANNER_GO},
    {"MinimumRuleBasedPlanner_Stop", TrajectorySource::BACKUP_PLANNER_STOP},
  };
  if (generator_name_prefix_to_source.count(generator_name_prefix) == 0) {
    return std::nullopt;
  }
  return generator_name_prefix_to_source.at(generator_name_prefix);
}

trajectory_ranker::RankerInputTrajectories TrajectorySelectorNode::to_ranker_input_trajectories(
  const CandidateTrajectories & trajectories, const ValidationReports & validation_reports)
{
  // Create map from UUID to generator name
  std::unordered_map<std::string, std::string> uuid_to_name;
  uuid_to_name.reserve(trajectories.generator_info.size());
  for (const auto & info : trajectories.generator_info) {
    uuid_to_name[autoware_utils_uuid::to_hex_string(info.generator_id)] = info.generator_name.data;
  }

  auto get_source =
    [&](const std::string & name) -> std::optional<trajectory_ranker::TrajectorySource> {
    for (const auto & prefix : selector_params_.generator_name_prefixes) {
      if (name.rfind(prefix, 0) == 0) {
        return generator_name_prefix_to_source(prefix);
      }
    }
    return std::nullopt;
  };

  RankerInputTrajectories input_trajectories;
  input_trajectories.reserve(trajectories.candidate_trajectories.size());
  for (const auto & candidate : trajectories.candidate_trajectories) {
    const auto risk_level = get_trajectory_risk_level(candidate, validation_reports);
    auto name_it = uuid_to_name.find(autoware_utils_uuid::to_hex_string(candidate.generator_id));
    if (name_it == uuid_to_name.end()) continue;
    const auto source = get_source(name_it->second);
    if (!source) continue;
    input_trajectories.push_back(RankerInputTrajectory{candidate, risk_level, source.value()});
  }
  return input_trajectories;
}

void TrajectorySelectorNode::process_trajectories()
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto concatenated_trajectories = concatenator_ptr_->get_concatenated();

  if (concatenated_trajectories.candidate_trajectories.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000, "No concatenated trajectories received yet");
    return;
  }

  pub_concatenated_trajectories_->publish(concatenated_trajectories);

  auto context_opt = take_validator_data();
  if (!context_opt) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "%s", context_opt.error().c_str());
    return;
  }

  const auto validator_report =
    validator_ptr_->validate_trajectories(concatenated_trajectories, context_opt.value());
  const auto & valid_trajectories = validator_report.valid_trajectories;
  const auto & validation_reports = validator_report.validation_reports;

  pub_validated_trajectories_->publish(valid_trajectories);

  const auto input_trajectories =
    to_ranker_input_trajectories(valid_trajectories, validation_reports);

  const auto scored_trajectories =
    ranker_ptr_->rank_trajectories(input_trajectories, take_ranker_data(valid_trajectories));
  pub_scored_trajectories_->publish(scored_trajectories);
}

void TrajectorySelectorNode::update_parameters()
{
  if (!selector_params_listener_.is_old(selector_params_)) return;

  const auto new_params = selector_params_listener_.get_params();
  const auto is_new_fallback_timer_period =
    new_params.fallback_period_ms != selector_params_.fallback_period_ms;
  selector_params_ = new_params;
  if (is_new_fallback_timer_period) update_fallback_timer();

  RCLCPP_INFO(get_logger(), "Trajectory Selector parameters are updated.");
}
void TrajectorySelectorNode::update_fallback_timer()
{
  if (timer_) {
    timer_->cancel();
  }
  RCLCPP_INFO(
    get_logger(), "New concatenate_and_validate timer callback created with period %ld.",
    selector_params_.fallback_period_ms);
  timer_ = autoware::agnocast_wrapper::create_timer(
    this, get_clock(), std::chrono::milliseconds(selector_params_.fallback_period_ms),
    std::bind(&TrajectorySelectorNode::process_trajectories, this));
}
}  // namespace autoware::trajectory_selector

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::trajectory_selector::TrajectorySelectorNode)
