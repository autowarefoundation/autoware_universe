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

#ifndef AUTOWARE__PLANNING_VALIDATOR__TYPES_HPP_
#define AUTOWARE__PLANNING_VALIDATOR__TYPES_HPP_

#include "autoware/planning_validator/debug_marker.hpp"
#include "autoware/planning_validator/utils.hpp"
#include "autoware_planning_validator/msg/planning_validator_status.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>

namespace autoware::planning_validator
{
using autoware::route_handler::RouteHandler;
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using autoware_planning_validator::msg::PlanningValidatorStatus;
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_updater::DiagnosticStatusWrapper;
using diagnostic_updater::Updater;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using nav_msgs::msg::Odometry;
using sensor_msgs::msg::PointCloud2;

enum class InvalidTrajectoryHandlingType : uint8_t {
  PUBLISH_AS_IT_IS,
  USE_PREVIOUS_RESULT,
  USE_PREVIOUS_RESULT_WITH_SOFT_STOP,
};

inline InvalidTrajectoryHandlingType get_handling_type(const int value)
{
  switch (value) {
    case 0:
      return InvalidTrajectoryHandlingType::PUBLISH_AS_IT_IS;
    case 1:
      return InvalidTrajectoryHandlingType::USE_PREVIOUS_RESULT;
    case 2:
      return InvalidTrajectoryHandlingType::USE_PREVIOUS_RESULT_WITH_SOFT_STOP;
    default:
      throw std::invalid_argument{"invalid_handling_type (" + std::to_string(value) + ")"};
  }
}

struct PlanningValidatorParams
{
  bool publish_diag = true;
  bool display_on_terminal = true;
  double soft_stop_deceleration{};
  double soft_stop_jerk_lim{};
  int diag_error_count_threshold{};
  InvalidTrajectoryHandlingType default_handling_type{};
};

struct PlanningValidatorData
{
  Trajectory::ConstSharedPtr current_trajectory;
  Trajectory::ConstSharedPtr resampled_current_trajectory;
  Trajectory::ConstSharedPtr last_valid_trajectory;

  std::optional<size_t> nearest_point_index;
  std::optional<size_t> nearest_segment_index;

  Odometry::ConstSharedPtr current_kinematics;
  AccelWithCovarianceStamped::ConstSharedPtr current_acceleration;
  PointCloud2::ConstSharedPtr obstacle_pointcloud;

  std::shared_ptr<RouteHandler> route_handler{std::make_shared<RouteHandler>()};

  bool is_ready(std::string & msg)
  {
    if (!current_trajectory) {
      msg = "current_trajectory";
      return false;
    }
    if (!current_kinematics) {
      msg = "current_kinematics";
      return false;
    }
    if (!current_acceleration) {
      msg = "current_acceleration";
      return false;
    }
    if (!obstacle_pointcloud) {
      msg = "obstacle_pointcloud";
      return false;
    }
    if (!route_handler->isHandlerReady()) {
      msg = "route/map";
      return false;
    }
    return true;
  }

  void set_current_trajectory(const Trajectory::ConstSharedPtr & msg)
  {
    current_trajectory.reset();
    resampled_current_trajectory.reset();
    if (!msg) return;
    current_trajectory = msg;
    constexpr double min_interval = 1.0;
    resampled_current_trajectory =
      std::make_shared<Trajectory>(resampleTrajectory(*current_trajectory, min_interval));
    set_nearest_trajectory_indices();
  }

  void set_nearest_trajectory_indices()
  {
    nearest_point_index.reset();
    nearest_segment_index.reset();
    if (!current_trajectory || !current_kinematics || current_trajectory->points.empty()) {
      return;
    }
    nearest_point_index = autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
      current_trajectory->points, current_kinematics->pose.pose);
    nearest_segment_index = autoware::motion_utils::findNearestSegmentIndex(
      current_trajectory->points, current_kinematics->pose.pose);
  }

  void set_route(const LaneletRoute::ConstSharedPtr & msg)
  {
    if (msg) {
      if (!msg->segments.empty()) {
        route_handler->setRoute(*msg);
      }
    }
  }

  void set_map(const LaneletMapBin::ConstSharedPtr & msg)
  {
    if (msg) {
      route_handler->setMap(*msg);
    }
  }
};

struct PlanningValidatorContext
{
  explicit PlanningValidatorContext(rclcpp::Node * node)
  : vehicle_info(autoware::vehicle_info_utils::VehicleInfoUtils(*node).getVehicleInfo()),
    tf_buffer{node->get_clock()},
    tf_listener{tf_buffer}
  {
    debug_pose_publisher = std::make_shared<PlanningValidatorDebugMarkerPublisher>(node);
    data = std::make_shared<PlanningValidatorData>();
    validation_status = std::make_shared<PlanningValidatorStatus>();
    diag_updater = std::make_shared<Updater>(node);
    init_validation_status();
  }

  autoware::vehicle_info_utils::VehicleInfo vehicle_info;

  PlanningValidatorParams params;

  std::shared_ptr<PlanningValidatorDebugMarkerPublisher> debug_pose_publisher = nullptr;
  std::shared_ptr<Updater> diag_updater = nullptr;
  std::shared_ptr<PlanningValidatorData> data = nullptr;
  std::shared_ptr<PlanningValidatorStatus> validation_status = nullptr;

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;

  void set_diag_id(const std::string & id)
  {
    if (diag_updater) {
      diag_updater->setHardwareID(id);
    }
  }

  void update_diag()
  {
    if (diag_updater) {
      diag_updater->force_update();
    }
  }

  void init_validation_status()
  {
    auto & s = validation_status;
    s->is_valid_size = true;
    s->is_valid_finite_value = true;
    s->is_valid_interval = true;
    s->is_valid_relative_angle = true;
    s->is_valid_curvature = true;
    s->is_valid_lateral_acc = true;
    s->is_valid_lateral_jerk = true;
    s->is_valid_longitudinal_max_acc = true;
    s->is_valid_longitudinal_min_acc = true;
    s->is_valid_steering = true;
    s->is_valid_steering_rate = true;
    s->is_valid_velocity_deviation = true;
    s->is_valid_distance_deviation = true;
    s->is_valid_longitudinal_distance_deviation = true;
    s->is_valid_forward_trajectory_length = true;
    s->is_valid_latency = true;
    s->is_valid_yaw_deviation = true;
    s->is_valid_trajectory_shift = true;
    s->is_valid_intersection_collision_check = true;
    s->is_valid_rear_collision_check = true;
  }

  void set_handling(const InvalidTrajectoryHandlingType handling_type)
  {
    if (inv_traj_handling > handling_type) return;
    inv_traj_handling = handling_type;
  }

  void reset_handling() { inv_traj_handling = InvalidTrajectoryHandlingType::PUBLISH_AS_IT_IS; }

  InvalidTrajectoryHandlingType get_handling() const { return inv_traj_handling; }

private:
  InvalidTrajectoryHandlingType inv_traj_handling;
};

}  // namespace autoware::planning_validator

#endif  // AUTOWARE__PLANNING_VALIDATOR__TYPES_HPP_
