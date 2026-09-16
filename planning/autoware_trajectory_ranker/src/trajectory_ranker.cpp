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

#include "autoware/trajectory_ranker/trajectory_ranker.hpp"

#include "autoware/trajectory_ranker/utils.hpp"

#include <autoware_utils_uuid/uuid_helper.hpp>

#include <autoware_internal_planning_msgs/msg/candidate_trajectory.hpp>
#include <autoware_internal_planning_msgs/msg/scored_candidate_trajectory.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

namespace
{
using autoware::trajectory_ranker::TrajectorySource;
using autoware_trajectory_validator::msg::RiskLevel;
using std::string;
std::string to_risk_string(const RiskLevel::_level_type risk_level)
{
  static const std::unordered_map<RiskLevel::_level_type, std::string> risk_level_to_string = {
    {RiskLevel::SAFE, "safe"},
    {RiskLevel::LOW_CAUTION, "low_caution"},
    {RiskLevel::HIGH_CAUTION, "high_caution"},
    {RiskLevel::DANGER, "danger"},
    {RiskLevel::FATAL, "fatal"},
  };
  if (risk_level_to_string.count(risk_level) == 0) {
    return "";
  }
  return risk_level_to_string.at(risk_level);
}

std::string to_source_string(const TrajectorySource source)
{
  static const std::unordered_map<TrajectorySource, std::string> source_to_string = {
    {TrajectorySource::DIFFUSION_PLANNER, "diffusion_planner"},
    {TrajectorySource::BACKUP_PLANNER_GO, "backup_planner_go"},
    {TrajectorySource::BACKUP_PLANNER_STOP, "backup_planner_stop"},
  };
  if (source_to_string.count(source) == 0) {
    return "";
  }
  return source_to_string.at(source);
}
}  // namespace

namespace autoware::trajectory_ranker
{

tl::expected<RankerResult, std::string> TrajectoryRanker::process(
  const RankerInputTrajectories & input_trajectories, const RankerContext & context)
{
  ScoredTrajectories scored_trajectories;

  for (const auto & input_trajectory : input_trajectories) {
    scored_trajectories.emplace_back(ScoredTrajectory{input_trajectory});
  }

  evaluate_safety(scored_trajectories);
  evaluate_source(scored_trajectories);
  evaluate_quality(scored_trajectories, context);
  score_trajectories(scored_trajectories);

  RankerResult result;
  for (const auto & scored_trajectory : scored_trajectories) {
    ScoredCandidateTrajectory scored_candidate;
    scored_candidate.candidate_trajectory = scored_trajectory.input_trajectory.candidate_trajectory;
    scored_candidate.score = static_cast<float>(scored_trajectory.score);
    result.scored_trajectories.scored_candidate_trajectories.push_back(scored_candidate);
  }

  auto best_itr = std::max_element(
    scored_trajectories.begin(), scored_trajectories.end(),
    [](const auto & a, const auto & b) { return a.score < b.score; });
  if (best_itr == scored_trajectories.end()) {
    previous_points_ = nullptr;
    return tl::make_unexpected<std::string>("No best trajectory found.");
  }

  result.best_trajectory_info = *best_itr;
  update_trajectory_history(result.best_trajectory_info);

  previous_points_ = std::make_shared<TrajectoryPoints>(
    result.best_trajectory_info.input_trajectory.candidate_trajectory.points);

  for (const auto & traj : result.scored_trajectories.scored_candidate_trajectories) {
    auto it = std::find_if(
      context.generator_info.begin(), context.generator_info.end(), [&](const auto & info) {
        return traj.candidate_trajectory.generator_id.uuid == info.generator_id.uuid;
      });
    if (it != context.generator_info.end()) {
      result.scored_trajectories.generator_info.push_back(*it);
    }
  }

  return result;
}

void TrajectoryRanker::evaluate_safety(ScoredTrajectories & scored_trajectories) const
{
  if (!params_.safety.enable) return;

  auto get_safety_penalty = [&](const RiskLevel::_level_type risk_level) -> double {
    const auto risk_string = to_risk_string(risk_level);
    const auto idx =
      std::find(params_.safety.levels.begin(), params_.safety.levels.end(), risk_string);
    if (idx == params_.safety.levels.end()) return 1.0;

    size_t penalty_index = idx - params_.safety.levels.begin();
    if (penalty_index >= params_.safety.penalty.size()) return 1.0;
    return params_.safety.penalty.at(penalty_index);
  };

  for (auto & scored_trajectory : scored_trajectories) {
    const auto risk_level = scored_trajectory.input_trajectory.risk_level;
    scored_trajectory.safety_penalty = get_safety_penalty(risk_level);
  }
}

void TrajectoryRanker::evaluate_source(ScoredTrajectories & scored_trajectories) const
{
  if (!params_.source.enable) return;

  auto get_source_penalty = [&](const std::string & source) -> double {
    const auto idx = std::find(params_.source.levels.begin(), params_.source.levels.end(), source);
    if (idx == params_.source.levels.end()) return 1.0;

    size_t penalty_index = idx - params_.source.levels.begin();
    if (penalty_index >= params_.source.penalty.size()) return 1.0;
    return params_.source.penalty.at(penalty_index);
  };

  for (auto & scored_trajectory : scored_trajectories) {
    const auto source = scored_trajectory.input_trajectory.source;
    scored_trajectory.source_penalty = get_source_penalty(to_source_string(source));
  }
}

void TrajectoryRanker::evaluate_quality(
  ScoredTrajectories & scored_trajectories, const RankerContext & context) const
{
  if (!params_.evaluation.enable) return;

  if (
    !context.route_handler || !context.route_handler->isHandlerReady() ||
    context.odometry == nullptr) {
    return;
  }

  const auto preferred_lanes =
    std::make_shared<lanelet::ConstLanelets>(context.route_handler->getPreferredLanelets());

  evaluator_->clear();

  // Create shared pointer to trajectory history for passing to CoreData
  auto trajectory_history_ptr = std::make_shared<std::deque<Trajectory>>(trajectory_history_);

  // Process each candidate trajectory
  for (auto & scored_trajectory : scored_trajectories) {
    const auto & candidate = scored_trajectory.input_trajectory.candidate_trajectory;
    auto sampled = utils::sampling(
      candidate.points, context.odometry->pose.pose, params_.evaluation.sampling_number,
      params_.evaluation.sampling_resolution);
    auto sampled_points = std::make_shared<TrajectoryPoints>(std::move(sampled));
    auto original_points = std::make_shared<TrajectoryPoints>(candidate.points);

    auto core_data = std::make_shared<CoreData>(
      original_points, sampled_points, previous_points_, preferred_lanes, candidate.header,
      candidate.generator_id, trajectory_history_ptr, candidate.turn_indicators_command);

    auto quality_score = evaluator_->score(core_data);
    scored_trajectory.quality_penalty = 1.0 - quality_score;
  }
}

void TrajectoryRanker::score_trajectories(ScoredTrajectories & scored_trajectories) const
{
  const auto max_cost = params_.safety.scale + params_.source.scale + params_.evaluation.scale;
  if (max_cost < 1e-6) return;
  for (auto & scored_trajectory : scored_trajectories) {
    auto cost = params_.safety.scale * scored_trajectory.safety_penalty;
    cost += params_.source.scale * scored_trajectory.source_penalty;
    cost += params_.evaluation.scale * scored_trajectory.quality_penalty;
    cost /= max_cost;
    scored_trajectory.score = 1.0 - cost;
  }
}

void TrajectoryRanker::update_trajectory_history(const ScoredTrajectory & best_trajectory_info)
{
  Trajectory best_trajectory;
  best_trajectory.header = best_trajectory_info.input_trajectory.candidate_trajectory.header;
  best_trajectory.points = best_trajectory_info.input_trajectory.candidate_trajectory.points;
  trajectory_history_.push_back(best_trajectory);

  if (
    trajectory_history_.size() > static_cast<size_t>(params_.evaluation.trajectory_history_size)) {
    trajectory_history_.pop_front();
  }
}

}  // namespace autoware::trajectory_ranker
