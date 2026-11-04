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

#ifndef AUTOWARE__TRAJECTORY_FOLLOWER_NODE__TRAJECTORY_FOLLOWER_HPP_
#define AUTOWARE__TRAJECTORY_FOLLOWER_NODE__TRAJECTORY_FOLLOWER_HPP_

#include "autoware/trajectory_follower_base/input_data.hpp"
#include "autoware/trajectory_follower_base/lateral_controller_interface.hpp"
#include "autoware/trajectory_follower_base/longitudinal_controller_interface.hpp"
#include "autoware/trajectory_follower_base/sync_data.hpp"

#include <rclcpp/time.hpp>
#include <tl_expected/expected.hpp>

#include "autoware_control_msgs/msg/control.hpp"
#include "autoware_control_msgs/msg/control_horizon.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <memory>
#include <optional>
#include <string>

namespace autoware::motion::control::trajectory_follower_node
{
namespace trajectory_follower = ::autoware::motion::control::trajectory_follower;

struct TrajectoryFollowerConfig
{
  double timeout_thr_sec{0.0};
  bool enable_control_cmd_horizon{false};
};

struct TrajectoryFollowerResult
{
  autoware_control_msgs::msg::Control control_cmd;
  std::optional<autoware_planning_msgs::msg::Trajectory> predicted_trajectory{std::nullopt};
  std::optional<autoware_control_msgs::msg::ControlHorizon> control_cmd_horizon{std::nullopt};
  visualization_msgs::msg::MarkerArray debug_marker;
};

struct TrajectoryFollowerError
{
  enum class Level { info, error };

  Level level;
  std::string message;
};

class TrajectoryFollower
{
public:
  TrajectoryFollower(
    const TrajectoryFollowerConfig & config,
    std::unique_ptr<trajectory_follower::LateralControllerInterface> lateral_controller,
    std::unique_ptr<trajectory_follower::LongitudinalControllerInterface> longitudinal_controller);

  tl::expected<TrajectoryFollowerResult, TrajectoryFollowerError> run(
    const trajectory_follower::InputData & input_data, const rclcpp::Time & stamp);

private:
  TrajectoryFollowerConfig config_;
  std::unique_ptr<trajectory_follower::LateralControllerInterface> lateral_controller_;
  std::unique_ptr<trajectory_follower::LongitudinalControllerInterface> longitudinal_controller_;
  trajectory_follower::LateralSyncData previous_lateral_sync_data_;
};

}  // namespace autoware::motion::control::trajectory_follower_node

#endif  // AUTOWARE__TRAJECTORY_FOLLOWER_NODE__TRAJECTORY_FOLLOWER_HPP_
