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

#include "trajectory_smoother.hpp"

#include <memory>

namespace autoware::in_lane_mrm_planner
{

void TrajectorySmoother::initialize(rclcpp::Node * node, const Params & params)
{
  (void)params;

  const auto time_keeper = std::make_shared<autoware::path_smoother::TimeKeeper>();
  const autoware::path_smoother::EgoNearestParam ego_nearest_param(node);
  const autoware::path_smoother::CommonParam common_param(node);

  constexpr bool enable_debug_info = false;
  eb_ = std::make_shared<autoware::path_smoother::EBPathSmoother>(
    node, enable_debug_info, ego_nearest_param, common_param, time_keeper);
}

void TrajectorySmoother::smooth(
  TrajectoryPoints & points, const geometry_msgs::msg::Pose & ego_pose)
{
  if (!eb_ || points.size() < 2) {
    return;
  }

  points = eb_->smoothTrajectory(points, ego_pose);
  eb_->resetPreviousData();
}

}  // namespace autoware::in_lane_mrm_planner
