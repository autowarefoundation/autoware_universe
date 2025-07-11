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
  STOP_PUBLISHING,
  USE_PREVIOUS_RESULT,
};

struct PlanningValidatorParams
{
  bool enable_soft_stop_on_prev_traj = true;
  bool publish_diag = true;
  bool display_on_terminal = true;
  double soft_stop_deceleration{};
  double soft_stop_jerk_lim{};
  int diag_error_count_threshold{};
  InvalidTrajectoryHandlingType inv_traj_handling_type{};
  InvalidTrajectoryHandlingType inv_traj_critical_handling_type{};
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

  void set_diag_status(
    DiagnosticStatusWrapper & stat, const bool & is_ok, const std::string & msg,
    const bool & is_critical = false)
  {
    if (is_ok) {
      stat.summary(DiagnosticStatus::OK, "validated.");
      return;
    }

    const bool only_warn = std::invoke([&]() {
      const auto handling_type =
        is_critical ? params.inv_traj_critical_handling_type : params.inv_traj_handling_type;
      if (handling_type != InvalidTrajectoryHandlingType::USE_PREVIOUS_RESULT) {
        return false;
      }
      return params.enable_soft_stop_on_prev_traj;
    });

    if (validation_status->invalid_count < params.diag_error_count_threshold || only_warn) {
      const auto warn_msg = msg + " (invalid count is less than error threshold: " +
                            std::to_string(validation_status->invalid_count) + " < " +
                            std::to_string(params.diag_error_count_threshold) + ")";
      stat.summary(DiagnosticStatus::WARN, warn_msg);
    } else {
      stat.summary(DiagnosticStatus::ERROR, msg);
    }
  }

  void add_diag(
    const std::string & name, const bool & status, const std::string & msg,
    const bool & is_critical = false)
  {
    if (diag_updater) {
      // Do not do implicit capture, need to capture msg by copy
      diag_updater->add(name, [this, &status, is_critical = is_critical, msg = msg](auto & stat) {
        set_diag_status(stat, status, msg, is_critical);
      });
    }
  }

  void update_diag()
  {
    if (diag_updater) {
      diag_updater->force_update();
    }
  }
};

}  // namespace autoware::planning_validator

#endif  // AUTOWARE__PLANNING_VALIDATOR__TYPES_HPP_
