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

#ifndef AUTOWARE__TRAJECTORY_RANKER__TRAJECTORY_RANKER_NODE_HPP_
#define AUTOWARE__TRAJECTORY_RANKER__TRAJECTORY_RANKER_NODE_HPP_

#include "autoware/trajectory_ranker/data_structs.hpp"
#include "autoware/trajectory_ranker/evaluation.hpp"

#include <autoware/route_handler/route_handler.hpp>
#include <autoware_trajectory_ranker/trajectory_ranker_parameters.hpp>
#include <autoware_utils_debug/time_keeper.hpp>
#include <autoware_utils_rclcpp/polling_subscriber.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/candidate_trajectories.hpp>
#include <autoware_internal_planning_msgs/msg/scored_candidate_trajectories.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>

namespace autoware::trajectory_ranker
{

using autoware_internal_planning_msgs::msg::CandidateTrajectories;
using autoware_internal_planning_msgs::msg::ScoredCandidateTrajectories;
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_planning_msgs::msg::LaneletRoute;
using nav_msgs::msg::Odometry;
using route_handler::RouteHandler;
using visualization_msgs::msg::MarkerArray;

class TrajectoryRanker : public rclcpp::Node
{
public:
  explicit TrajectoryRanker(const rclcpp::NodeOptions & options);

private:
  void process(const CandidateTrajectories::ConstSharedPtr msg);

  ScoredCandidateTrajectories::ConstSharedPtr score(
    const CandidateTrajectories::ConstSharedPtr msg);

  std::shared_ptr<EvaluatorParameters> parameters() const;

  autoware_utils_rclcpp::InterProcessPollingSubscriber<PredictedObjects> sub_objects_{
    this, "~/input/objects"};

  autoware_utils_rclcpp::InterProcessPollingSubscriber<Odometry> sub_odometry_{
    this, "~/input/odometry"};

  rclcpp::Subscription<LaneletMapBin>::SharedPtr sub_map_;
  rclcpp::Subscription<LaneletRoute>::SharedPtr sub_route_;
  rclcpp::Subscription<CandidateTrajectories>::SharedPtr sub_trajectories_;

  rclcpp::Publisher<ScoredCandidateTrajectories>::SharedPtr pub_trajectories_;

  std::unique_ptr<evaluation::ParamListener> listener_;
  std::shared_ptr<RouteHandler> route_handler_;
  std::shared_ptr<Evaluator> evaluator_;

  std::shared_ptr<TrajectoryPoints> previous_points_;

  rclcpp::Publisher<autoware_utils_debug::ProcessingTimeDetail>::SharedPtr
    debug_processing_time_detail_pub_;
  mutable std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_{nullptr};
};

}  // namespace autoware::trajectory_ranker

#endif  // AUTOWARE__TRAJECTORY_RANKER__TRAJECTORY_RANKER_NODE_HPP_
