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

#ifndef TRAJECTORY_SMOOTHER_HPP_
#define TRAJECTORY_SMOOTHER_HPP_

#include "type_alias.hpp"

#include <autoware/path_smoother/common_structs.hpp>
#include <autoware/path_smoother/elastic_band.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <memory>

namespace autoware::in_lane_mrm_planner
{

// Thin wrapper around autoware_path_smoother::EBPathSmoother (Phase1 approach A).
// Node must load config/elastic_band_smoother.param.yaml for EB parameters.
class TrajectorySmoother
{
public:
  void initialize(rclcpp::Node * node, const Params & params);

  void smooth(TrajectoryPoints & points, const geometry_msgs::msg::Pose & ego_pose);

private:
  std::shared_ptr<autoware::path_smoother::EBPathSmoother> eb_;
};

}  // namespace autoware::in_lane_mrm_planner

#endif  // TRAJECTORY_SMOOTHER_HPP_
