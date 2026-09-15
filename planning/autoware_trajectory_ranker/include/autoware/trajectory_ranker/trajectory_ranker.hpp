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

#ifndef AUTOWARE__TRAJECTORY_RANKER__TRAJECTORY_RANKER_HPP_
#define AUTOWARE__TRAJECTORY_RANKER__TRAJECTORY_RANKER_HPP_

#include "autoware/trajectory_ranker/evaluation.hpp"
#include "autoware/trajectory_ranker/interface/metrics_interface.hpp"

#include <autoware_trajectory_ranker/autoware_trajectory_ranker_param.hpp>
#include <autoware_trajectory_validator/msg/validation_report.hpp>

#include <autoware_internal_planning_msgs/msg/candidate_trajectories.hpp>
#include <autoware_internal_planning_msgs/msg/scored_candidate_trajectories.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <deque>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::trajectory_ranker
{
using autoware_internal_planning_msgs::msg::CandidateTrajectories;
using autoware_internal_planning_msgs::msg::CandidateTrajectory;
using autoware_internal_planning_msgs::msg::GeneratorInfo;
using autoware_internal_planning_msgs::msg::ScoredCandidateTrajectories;
using autoware_internal_planning_msgs::msg::ScoredCandidateTrajectory;
using autoware_trajectory_validator::msg::ValidationReport;
using metrics::MetricInterface;

using ValidationReports = std::vector<ValidationReport>;

struct RankerContext
{
  nav_msgs::msg::Odometry::ConstSharedPtr odometry;
  std::shared_ptr<RouteHandler> route_handler;
  std::vector<GeneratorInfo> generator_info;
};

enum class TrajectorySource : uint8_t {
  DIFFUSION_PLANNER = 0,
  BACKUP_PLANNER_GO,
  BACKUP_PLANNER_STOP,
};

struct RankerInputTrajectory
{
  CandidateTrajectory candidate_trajectory;
  autoware_trajectory_validator::msg::RiskLevel::_level_type risk_level{};
  TrajectorySource source{};
};
using RankerInputTrajectories = std::vector<RankerInputTrajectory>;

struct ScoredTrajectory
{
  RankerInputTrajectory input_trajectory;
  double safety_penalty{};
  double source_penalty{};
  double quality_penalty{};
  double score{};
};
using ScoredTrajectories = std::vector<ScoredTrajectory>;

struct RankerResult
{
  ScoredCandidateTrajectories scored_trajectories;
  ScoredTrajectory best_trajectory_info;
};

class TrajectoryRanker
{
public:
  /**
   * @brief Constructs the ranker with the given evaluator and parameters.
   * @param evaluator Evaluator to use for ranking trajectories.
   * @param params Parameters for the ranker.
   */
  explicit TrajectoryRanker(
    const std::shared_ptr<Evaluator> & evaluator, const trajectory_ranker_params::Params & params)
  : evaluator_(evaluator), params_(params)
  {
  }

  tl::expected<RankerResult, std::string> process(
    const RankerInputTrajectories & input_trajectories, const RankerContext & context);

  void update_parameters(const trajectory_ranker_params::Params & params)
  {
    params_ = params;
    evaluator_->update_parameters(params.evaluation);
  }

private:
  void evaluate_safety(ScoredTrajectories & scored_trajectories) const;
  void evaluate_source(ScoredTrajectories & scored_trajectories) const;
  void evaluate_quality(
    ScoredTrajectories & scored_trajectories, const RankerContext & context) const;
  void score_trajectories(ScoredTrajectories & scored_trajectories) const;

  void update_trajectory_history(const ScoredTrajectory & best_trajectory_info);

  std::shared_ptr<Evaluator> evaluator_;

  std::shared_ptr<TrajectoryPoints> previous_points_;
  std::deque<autoware_planning_msgs::msg::Trajectory> trajectory_history_;

  trajectory_ranker_params::Params params_;
};

}  // namespace autoware::trajectory_ranker

#endif  // AUTOWARE__TRAJECTORY_RANKER__TRAJECTORY_RANKER_HPP_
