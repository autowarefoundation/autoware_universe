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

#ifndef AUTOWARE__TRAJECTORY_RANKER__DATA_STRUCTS_HPP_
#define AUTOWARE__TRAJECTORY_RANKER__DATA_STRUCTS_HPP_

#include <autoware_internal_planning_msgs/msg/candidate_trajectory.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <std_msgs/msg/header.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <deque>
#include <memory>
#include <string>
#include <vector>

namespace autoware::trajectory_ranker
{

using autoware_perception_msgs::msg::PredictedObjects;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using autoware_vehicle_msgs::msg::SteeringReport;
using std_msgs::msg::Header;
using unique_identifier_msgs::msg::UUID;

using TrajectoryPoints = std::vector<TrajectoryPoint>;

struct CoreData
{
  CoreData(
    const std::shared_ptr<TrajectoryPoints> & points,
    const std::shared_ptr<TrajectoryPoints> & previous_points,
    const std::shared_ptr<PredictedObjects> & objects,
    const std::shared_ptr<lanelet::ConstLanelets> & preferred_lanes, const std::string & tag)
  : original{points},
    points{points},
    previous_points{previous_points},
    objects{objects},
    preferred_lanes{preferred_lanes},
    tag{tag}
  {
  }

  CoreData(
    const std::shared_ptr<TrajectoryPoints> & original,
    const std::shared_ptr<TrajectoryPoints> & points,
    const std::shared_ptr<TrajectoryPoints> & previous_points,
    const std::shared_ptr<PredictedObjects> & objects,
    const std::shared_ptr<lanelet::ConstLanelets> & preferred_lanes, const Header & header,
    const UUID & generator_id,
    const std::shared_ptr<std::deque<Trajectory>> & trajectory_history = nullptr)
  : original{original},
    points{points},
    previous_points{previous_points},
    objects{objects},
    preferred_lanes{preferred_lanes},
    tag{"__anon"},
    header{header},
    generator_id{generator_id},
    trajectory_history{trajectory_history}
  {
  }

  std::shared_ptr<TrajectoryPoints> original;
  std::shared_ptr<TrajectoryPoints> points;
  std::shared_ptr<TrajectoryPoints> previous_points;
  std::shared_ptr<PredictedObjects> objects;
  std::shared_ptr<SteeringReport> steering;
  std::shared_ptr<lanelet::ConstLanelets> preferred_lanes;
  std::string tag;
  Header header;
  UUID generator_id;
  std::shared_ptr<std::deque<Trajectory>> trajectory_history;
};

struct EvaluatorParameters
{
  explicit EvaluatorParameters(const size_t metrics_num, const size_t sample_num)
  : sample_num{sample_num},
    time_decay_weight(metrics_num, std::vector<float>(sample_num, 0.0f)),
    score_weight(metrics_num, 0.0f),
    metrics_max_value(metrics_num, 0.0f)
  {
  }

  size_t sample_num;
  float resolution;
  std::vector<std::vector<float>> time_decay_weight;  // [metrics][samples]
  std::vector<float> score_weight;
  std::vector<float> metrics_max_value;
};

// Result of evaluation
struct EvaluationResult
{
  std::shared_ptr<CoreData> data;
  std::vector<float> scores;
  float total_score;

  explicit EvaluationResult(const std::shared_ptr<CoreData> & d) : data(d), total_score(0.0f) {}

  Header header() const { return data->header; }
  UUID uuid() const { return data->generator_id; }
  std::shared_ptr<TrajectoryPoints> original() const { return data->original; }
  std::shared_ptr<TrajectoryPoints> points() const { return data->points; }
  float total() const { return total_score; }
};

}  // namespace autoware::trajectory_ranker

#endif  // AUTOWARE__TRAJECTORY_RANKER__DATA_STRUCTS_HPP_
