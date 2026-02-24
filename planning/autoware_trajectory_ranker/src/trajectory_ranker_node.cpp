// Copyright 2025 TIER IV, Inc.
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

#include "autoware/trajectory_ranker/trajectory_ranker_node.hpp"

#include "autoware/trajectory_ranker/evaluation.hpp"
#include "autoware/trajectory_ranker/utils.hpp"

#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <magic_enum.hpp>
#include <rclcpp/logging.hpp>

#include <autoware_internal_planning_msgs/msg/candidate_trajectory.hpp>
#include <autoware_internal_planning_msgs/msg/scored_candidate_trajectory.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::trajectory_ranker
{
TrajectoryRanker::TrajectoryRanker(const rclcpp::NodeOptions & options)
: Node("trajectory_ranker", options),
  listener_{std::make_unique<evaluation::ParamListener>(get_node_parameters_interface())},
  route_handler_{std::make_shared<RouteHandler>()},
  previous_points_{nullptr}
{
  max_history_size_ = static_cast<size_t>(listener_->get_params().trajectory_history_size);

  // Vehicle info for evaluator
  const auto vehicle_info = std::make_shared<vehicle_info_utils::VehicleInfo>(
    vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo());

  evaluator_ = std::make_shared<Evaluator>(route_handler_, vehicle_info, get_logger(), this);

  const auto metrics = listener_->get_params().metrics;
  for (size_t i = 0; i < metrics.name.size(); i++) {
    evaluator_->load_metric(metrics.name.at(i), i, listener_->get_params().resolution);
  }

  // Setup subscriptions
  sub_map_ = create_subscription<LaneletMapBin>(
    "~/input/vector_map", rclcpp::QoS{1}.transient_local(),
    [this](const LaneletMapBin::ConstSharedPtr msg) { route_handler_->setMap(*msg); });

  sub_route_ = create_subscription<LaneletRoute>(
    "~/input/route", rclcpp::QoS{1}.transient_local(),
    [this](const LaneletRoute::ConstSharedPtr msg) { route_handler_->setRoute(*msg); });

  sub_trajectories_ = create_subscription<CandidateTrajectories>(
    "~/input/candidate_trajectories", 1,
    [this](const CandidateTrajectories::ConstSharedPtr msg) { process(msg); });

  // Setup publishers
  pub_trajectories_ =
    create_publisher<ScoredCandidateTrajectories>("~/output/scored_trajectories", 1);

  // Debug publisher
  debug_processing_time_detail_pub_ = create_publisher<autoware_utils_debug::ProcessingTimeDetail>(
    "~/debug/processing_time_detail_ms/trajectory_ranker", 1);
  time_keeper_ =
    std::make_shared<autoware_utils_debug::TimeKeeper>(debug_processing_time_detail_pub_);
}

void TrajectoryRanker::process(const CandidateTrajectories::ConstSharedPtr msg)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  pub_trajectories_->publish(*score(msg));
}

ScoredCandidateTrajectories::ConstSharedPtr TrajectoryRanker::score(
  const CandidateTrajectories::ConstSharedPtr msg)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  ScoredCandidateTrajectories output;
  output.generator_info = msg->generator_info;

  if (!route_handler_->isHandlerReady()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Route handler is not ready. Returning empty output.");
    return std::make_shared<ScoredCandidateTrajectories>(output);
  }

  const auto odometry_ptr = std::const_pointer_cast<Odometry>(sub_odometry_.take_data());
  if (odometry_ptr == nullptr) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Odometry data is not available. Returning empty output.");
    return std::make_shared<ScoredCandidateTrajectories>(output);
  }

  const auto objects_ptr = std::const_pointer_cast<PredictedObjects>(sub_objects_.take_data());
  if (objects_ptr == nullptr) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Objects data is not available. Returning empty output.");
    return std::make_shared<ScoredCandidateTrajectories>(output);
  }

  const auto preferred_lanes =
    std::make_shared<lanelet::ConstLanelets>(route_handler_->getPreferredLanelets());

  std::vector<autoware_internal_planning_msgs::msg::ScoredCandidateTrajectory> trajectories;
  trajectories.reserve(msg->candidate_trajectories.size());

  evaluator_->clear();

  const auto params = parameters();

  // Process each candidate trajectory
  for (const auto & candidate : msg->candidate_trajectories) {
    auto sampled = utils::sampling(
      candidate.points, odometry_ptr->pose.pose, params->sample_num, params->resolution);
    auto sampled_points = std::make_shared<TrajectoryPoints>(std::move(sampled));
    auto original_points = std::make_shared<TrajectoryPoints>(candidate.points);

    // Create shared pointer to trajectory history for passing to CoreData
    auto trajectory_history_ptr = std::make_shared<std::deque<Trajectory>>(trajectory_history_);

    auto core_data = std::make_shared<CoreData>(
      original_points, sampled_points, previous_points_, objects_ptr, preferred_lanes,
      candidate.header, candidate.generator_id, trajectory_history_ptr);

    evaluator_->add(core_data);
  }

  const auto best_data = evaluator_->best(params);
  previous_points_ = best_data == nullptr ? nullptr : best_data->points();

  // Update trajectory history buffer with the best trajectory
  if (best_data != nullptr && best_data->points() != nullptr) {
    // Create Trajectory from best trajectory
    Trajectory best_trajectory;
    best_trajectory.header = best_data->header();
    best_trajectory.points = *best_data->original();

    trajectory_history_.push_back(best_trajectory);

    // Limit buffer size
    if (trajectory_history_.size() > max_history_size_) {
      trajectory_history_.pop_front();
    }
  }

  for (const auto & result : evaluator_->results()) {
    const auto candidate = autoware_internal_planning_msgs::build<
                             autoware_internal_planning_msgs::msg::CandidateTrajectory>()
                             .header(result->header())
                             .generator_id(result->uuid())
                             .points(*result->original());
    const auto scored_trajectory =
      autoware_internal_planning_msgs::build<
        autoware_internal_planning_msgs::msg::ScoredCandidateTrajectory>()
        .candidate_trajectory(candidate)
        .score(result->total());
    trajectories.push_back(scored_trajectory);
  }

  output.scored_candidate_trajectories = trajectories;
  return std::make_shared<ScoredCandidateTrajectories>(output);
}

std::shared_ptr<EvaluatorParameters> TrajectoryRanker::parameters() const
{
  const auto node_params = listener_->get_params();

  const auto parameters =
    std::make_shared<EvaluatorParameters>(node_params.metrics.name.size(), node_params.sample_num);

  // Convert double to float
  parameters->resolution = static_cast<float>(node_params.resolution);

  // Convert vector<double> to vector<float>
  parameters->score_weight.assign(node_params.score_weight.begin(), node_params.score_weight.end());

  // Convert time_decay_weight vectors
  auto convert_vector = [](const std::vector<double> & src) {
    std::vector<float> dst;
    dst.reserve(src.size());
    for (const auto & val : src) {
      dst.push_back(static_cast<float>(val));
    }
    return dst;
  };

  parameters->time_decay_weight.at(0) = convert_vector(node_params.time_decay_weight.s0);
  parameters->time_decay_weight.at(1) = convert_vector(node_params.time_decay_weight.s1);
  parameters->time_decay_weight.at(2) = convert_vector(node_params.time_decay_weight.s2);
  parameters->time_decay_weight.at(3) = convert_vector(node_params.time_decay_weight.s3);
  parameters->time_decay_weight.at(4) = convert_vector(node_params.time_decay_weight.s4);
  parameters->time_decay_weight.at(5) = convert_vector(node_params.time_decay_weight.s5);

  // Convert metrics_max_value
  parameters->metrics_max_value.assign(
    node_params.metrics.maximum.begin(), node_params.metrics.maximum.end());

  return parameters;
}

}  // namespace autoware::trajectory_ranker

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::trajectory_ranker::TrajectoryRanker)
