// Copyright 2025 TIER IV, Inc. All rights reserved.
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

#include "obstacle_slow_down_module.hpp"

#include <autoware/motion_utils/distance/distance.hpp>
#include <autoware/motion_utils/marker/marker_helper.hpp>
#include <autoware/motion_utils/marker/virtual_wall_marker_creator.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/signal_processing/lowpass_filter_1d.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/ros/marker_helper.hpp>
#include <autoware_utils/ros/parameter.hpp>
#include <autoware_utils/ros/update_param.hpp>
#include <autoware_utils/ros/uuid_helper.hpp>

#include <algorithm>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::motion_velocity_planner
{
using autoware_utils::get_or_declare_parameter;

namespace
{
bool is_lower_considering_hysteresis(
  const double current_val, const bool was_low, const double high_val, const double low_val)
{
  if (was_low) {
    if (high_val < current_val) {
      return false;
    }
    return true;
  }
  if (current_val < low_val) {
    return true;
  }
  return false;
}

geometry_msgs::msg::Point to_geom_point(const autoware_utils::Point2d & point)
{
  geometry_msgs::msg::Point geom_point;
  geom_point.x = point.x();
  geom_point.y = point.y();
  return geom_point;
}

template <typename T>
std::optional<T> get_object_from_uuid(
  const std::vector<T> & objects, const std::string & target_uuid)
{
  const auto itr = std::find_if(objects.begin(), objects.end(), [&](const auto & object) {
    return object.uuid == target_uuid;
  });

  if (itr == objects.end()) {
    return std::nullopt;
  }
  return *itr;
}

// TODO(murooka) following two functions are copied from behavior_velocity_planner.
// These should be refactored.
double find_reach_time(
  const double jerk, const double accel, const double velocity, const double distance,
  const double t_min, const double t_max)
{
  const double j = jerk;
  const double a = accel;
  const double v = velocity;
  const double d = distance;
  const double min = t_min;
  const double max = t_max;
  auto f = [](const double t, const double j, const double a, const double v, const double d) {
    return j * t * t * t / 6.0 + a * t * t / 2.0 + v * t - d;
  };
  if (f(min, j, a, v, d) > 0 || f(max, j, a, v, d) < 0) {
    throw std::logic_error(
      "[motion_velocity_planner_common](find_reach_time): search range is invalid");
  }
  const double eps = 1e-5;
  const int warn_iter = 100;
  double lower = min;
  double upper = max;
  double t;
  int iter = 0;
  for (int i = 0;; i++) {
    t = 0.5 * (lower + upper);
    const double fx = f(t, j, a, v, d);
    // std::cout<<"fx: "<<fx<<" up: "<<upper<<" lo: "<<lower<<" t: "<<t<<std::endl;
    if (std::abs(fx) < eps) {
      break;
    } else if (fx > 0.0) {
      upper = t;
    } else {
      lower = t;
    }
    iter++;
    if (iter > warn_iter)
      std::cerr << "[motion_velocity_planner_common](find_reach_time): current iter is over warning"
                << std::endl;
  }
  return t;
}

double calc_deceleration_velocity_from_distance_to_target(
  const double max_slowdown_jerk, const double max_slowdown_accel, const double current_accel,
  const double current_velocity, const double distance_to_target)
{
  if (max_slowdown_jerk > 0 || max_slowdown_accel > 0) {
    throw std::logic_error("max_slowdown_jerk and max_slowdown_accel should be negative");
  }
  // case0: distance to target is behind ego
  if (distance_to_target <= 0) return current_velocity;
  auto ft = [](const double t, const double j, const double a, const double v, const double d) {
    return j * t * t * t / 6.0 + a * t * t / 2.0 + v * t - d;
  };
  auto vt = [](const double t, const double j, const double a, const double v) {
    return j * t * t / 2.0 + a * t + v;
  };
  const double j_max = max_slowdown_jerk;
  const double a0 = current_accel;
  const double a_max = max_slowdown_accel;
  const double v0 = current_velocity;
  const double l = distance_to_target;
  const double t_const_jerk = (a_max - a0) / j_max;
  const double d_const_jerk_stop = ft(t_const_jerk, j_max, a0, v0, 0.0);
  const double d_const_acc_stop = l - d_const_jerk_stop;

  if (d_const_acc_stop < 0) {
    // case0: distance to target is within constant jerk deceleration
    // use binary search instead of solving cubic equation
    const double t_jerk = find_reach_time(j_max, a0, v0, l, 0, t_const_jerk);
    const double velocity = vt(t_jerk, j_max, a0, v0);
    return velocity;
  } else {
    const double v1 = vt(t_const_jerk, j_max, a0, v0);
    const double discriminant_of_stop = 2.0 * a_max * d_const_acc_stop + v1 * v1;
    // case3: distance to target is farther than distance to stop
    if (discriminant_of_stop <= 0) {
      return 0.0;
    }
    // case2: distance to target is within constant accel deceleration
    // solve d = 0.5*a^2+v*t by t
    const double t_acc = (-v1 + std::sqrt(discriminant_of_stop)) / a_max;
    return vt(t_acc, 0.0, a_max, v1);
  }
  return current_velocity;
}

VelocityLimitClearCommand create_velocity_limit_clear_command(
  const rclcpp::Time & current_time, [[maybe_unused]] const std::string & module_name)
{
  VelocityLimitClearCommand msg;
  msg.stamp = current_time;
  msg.sender = "obstacle_slow_down";
  msg.command = true;
  return msg;
}

Float64Stamped create_float64_stamped(const rclcpp::Time & now, const float & data)
{
  Float64Stamped msg;
  msg.stamp = now;
  msg.data = data;
  return msg;
}
}  // namespace

void ObstacleSlowDownModule::init(rclcpp::Node & node, const std::string & module_name)
{
  module_name_ = module_name;
  clock_ = node.get_clock();
  logger_ = node.get_logger();

  // ros parameters
  common_param_ = CommonParam(node);
  slow_down_planning_param_ = SlowDownPlanningParam(node);
  obstacle_filtering_param_ = ObstacleFilteringParam(node);

  const double mask_lat_margin =
    get_or_declare_parameter<double>(node, "pointcloud.mask_lat_margin");

  if (mask_lat_margin < obstacle_filtering_param_.max_lat_margin) {
    throw std::invalid_argument("point-cloud mask narrower than stop margin");
  }

  objects_of_interest_marker_interface_ = std::make_unique<
    autoware::objects_of_interest_marker_interface::ObjectsOfInterestMarkerInterface>(
    &node, "motion_velocity_planner_common");

  // common publisher
  processing_time_publisher_ =
    node.create_publisher<Float64Stamped>("~/debug/obstacle_slow_down/processing_time_ms", 1);
  virtual_wall_publisher_ =
    node.create_publisher<MarkerArray>("~/obstacle_slow_down/virtual_walls", 1);
  debug_publisher_ = node.create_publisher<MarkerArray>("~/obstacle_slow_down/debug_markers", 1);

  // module publisher
  debug_slow_down_planning_info_pub_ =
    node.create_publisher<Float32MultiArrayStamped>("~/debug/obstacle_slow_down/planning_info", 1);
  processing_time_detail_pub_ = node.create_publisher<autoware_utils::ProcessingTimeDetail>(
    "~/debug/processing_time_detail_ms/obstacle_slow_down", 1);

  // interface publisher
  objects_of_interest_marker_interface_ = std::make_unique<
    autoware::objects_of_interest_marker_interface::ObjectsOfInterestMarkerInterface>(
    &node, "obstacle_slow_down");
  planning_factor_interface_ =
    std::make_unique<autoware::planning_factor_interface::PlanningFactorInterface>(
      &node, "obstacle_slow_down");

  // time keeper
  time_keeper_ = std::make_shared<autoware_utils::TimeKeeper>(processing_time_detail_pub_);
}

void ObstacleSlowDownModule::update_parameters(
  [[maybe_unused]] const std::vector<rclcpp::Parameter> & parameters)
{
}

std::vector<autoware::motion_velocity_planner::SlowDownPointData>
ObstacleSlowDownModule::convert_point_cloud_to_slow_down_points(
  const PlannerData::Pointcloud & pointcloud, const std::vector<TrajectoryPoint> & traj_points,
  const std::vector<Polygon2d> & decimated_traj_polys_with_lat_margin,
  const VehicleInfo & vehicle_info, const size_t ego_idx)
{
  if (pointcloud.pointcloud.empty()) {
    return {};
  }

  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto & p = obstacle_filtering_param_;

  std::vector<autoware::motion_velocity_planner::SlowDownPointData> slow_down_points;

  const PointCloud::Ptr filtered_points_ptr =
    pointcloud.get_filtered_pointcloud_ptr(traj_points, vehicle_info);
  const std::vector<pcl::PointIndices> clusters =
    pointcloud.get_cluster_indices(traj_points, vehicle_info);

  // 3. convert clusters to obstacles
  for (const auto & cluster_indices : clusters) {
    double ego_to_slow_down_front_collision_distance = std::numeric_limits<double>::max();
    double ego_to_slow_down_back_collision_distance = std::numeric_limits<double>::min();
    double lat_dist_from_obstacle_to_traj = std::numeric_limits<double>::max();
    std::optional<geometry_msgs::msg::Point> slow_down_front_collision_point = std::nullopt;
    std::optional<geometry_msgs::msg::Point> slow_down_back_collision_point = std::nullopt;

    for (const auto & index : cluster_indices.indices) {
      const auto obstacle_point = autoware::motion_velocity_planner::utils::to_geometry_point(
        filtered_points_ptr->points[index]);
      // 1. brief filtering - filters out point-cloud points that are far from the trajectory
      // laterally The lateral distance of the obstacle-point to trajectory is measured below
      const auto current_lat_dist_from_obstacle_to_traj =
        autoware::motion_utils::calcLateralOffset(traj_points, obstacle_point);
      // The minimum lateral distance to the trajectory polygon is estimated by assuming that the
      // ego-vehicle is fully perpendicular to the trajectory, in the very worst case
      const auto min_lat_dist_to_traj_poly =
        std::abs(current_lat_dist_from_obstacle_to_traj) - vehicle_info.max_longitudinal_offset_m;
      // The trajectory polygon is ignored if the minimum lateral distance is more than maximum
      // lateral margin
      if (min_lat_dist_to_traj_poly >= p.max_lat_margin) {
        continue;
      }

      // precise filtering
      const double precise_min_lat_dist_to_traj_poly =
        utils::get_dist_to_traj_poly(obstacle_point, decimated_traj_polys_with_lat_margin);
      if (precise_min_lat_dist_to_traj_poly > 0.0) {
        continue;
      }

      const auto current_ego_to_obstacle_distance =
        autoware::motion_velocity_planner::utils::calc_distance_to_front_object(
          traj_points, ego_idx, obstacle_point);
      if (!current_ego_to_obstacle_distance) {
        continue;
      }

      lat_dist_from_obstacle_to_traj =
        std::min(lat_dist_from_obstacle_to_traj, current_lat_dist_from_obstacle_to_traj);

      if (*current_ego_to_obstacle_distance < ego_to_slow_down_front_collision_distance) {
        slow_down_front_collision_point = obstacle_point;
        ego_to_slow_down_front_collision_distance = *current_ego_to_obstacle_distance;
      } else if (*current_ego_to_obstacle_distance > ego_to_slow_down_back_collision_distance) {
        slow_down_back_collision_point = obstacle_point;
        ego_to_slow_down_back_collision_distance = *current_ego_to_obstacle_distance;
      }
    }

    if (slow_down_front_collision_point) {
      slow_down_points.emplace_back(
        slow_down_front_collision_point, slow_down_back_collision_point,
        lat_dist_from_obstacle_to_traj);
    }
  }

  return slow_down_points;
}

VelocityPlanningResult ObstacleSlowDownModule::plan(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & raw_trajectory_points,
  [[maybe_unused]] const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> &
    smoothed_trajectory_points,
  const std::shared_ptr<const PlannerData> planner_data)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  stop_watch_.tic();
  debug_data_ptr_ = std::make_shared<DebugData>();
  decimated_traj_polys_ = std::nullopt;

  // calculate collision points with trajectory with lateral stop margin
  // NOTE: For additional margin, hysteresis is not divided by two.
  const auto & p = obstacle_filtering_param_;
  const auto & tp = planner_data->trajectory_polygon_collision_check;

  const auto decimated_traj_points = utils::decimate_trajectory_points_from_ego(
    raw_trajectory_points, planner_data->current_odometry.pose.pose,
    planner_data->ego_nearest_dist_threshold, planner_data->ego_nearest_yaw_threshold,
    planner_data->trajectory_polygon_collision_check.decimate_trajectory_step_length, 0.0);

  const auto decimated_traj_polys_with_lat_margin = polygon_utils::create_one_step_polygons(
    decimated_traj_points, planner_data->vehicle_info_, planner_data->current_odometry.pose.pose,
    p.max_lat_margin + p.lat_hysteresis_margin, tp.enable_to_consider_current_pose,
    tp.time_to_convergence, tp.decimate_trajectory_step_length);
  debug_data_ptr_->decimated_traj_polys = decimated_traj_polys_with_lat_margin;

  auto slow_down_obstacles_for_predicted_object = filter_slow_down_obstacle_for_predicted_object(
    planner_data->current_odometry, planner_data->ego_nearest_dist_threshold,
    planner_data->ego_nearest_yaw_threshold, decimated_traj_polys_with_lat_margin,
    raw_trajectory_points, planner_data->objects,
    rclcpp::Time(planner_data->predicted_objects_header.stamp), planner_data->vehicle_info_,
    planner_data->trajectory_polygon_collision_check);

  auto slow_down_obstacles_for_point_cloud = filter_slow_down_obstacle_for_point_cloud(
    raw_trajectory_points, decimated_traj_polys_with_lat_margin, planner_data->no_ground_pointcloud,
    planner_data->vehicle_info_,
    planner_data->find_index(raw_trajectory_points, planner_data->current_odometry.pose.pose));

  const auto slow_down_obstacles = autoware::motion_velocity_planner::utils::concat_vectors(
    std::move(slow_down_obstacles_for_predicted_object),
    std::move(slow_down_obstacles_for_point_cloud));

  VelocityPlanningResult result;
  result.slowdown_intervals = plan_slow_down(
    planner_data, raw_trajectory_points, slow_down_obstacles, result.velocity_limit,
    planner_data->vehicle_info_);

  // clear velocity limit if necessary
  if (result.velocity_limit) {
    need_to_clear_velocity_limit_ = true;
  } else {
    if (need_to_clear_velocity_limit_) {
      // clear velocity limit
      result.velocity_limit_clear_command =
        create_velocity_limit_clear_command(clock_->now(), module_name_);
      need_to_clear_velocity_limit_ = false;
    }
  }

  publish_debug_info();

  return result;
}

std::string ObstacleSlowDownModule::get_module_name() const
{
  return module_name_;
}

std::vector<SlowDownObstacle>
ObstacleSlowDownModule::filter_slow_down_obstacle_for_predicted_object(
  const Odometry & odometry, const double ego_nearest_dist_threshold,
  const double ego_nearest_yaw_threshold,
  const std::vector<Polygon2d> & decimated_traj_polys_with_lat_margin,
  const std::vector<TrajectoryPoint> & traj_points,
  const std::vector<std::shared_ptr<PlannerData::Object>> & objects,
  const rclcpp::Time & predicted_objects_stamp, const VehicleInfo & vehicle_info,
  const TrajectoryPolygonCollisionCheck & trajectory_polygon_collision_check)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto & current_pose = odometry.pose.pose;

  // slow down
  slow_down_condition_counter_.reset_current_uuids();
  std::vector<SlowDownObstacle> slow_down_obstacles;
  for (const auto & object : objects) {
    // 1. rough filtering
    // 1.1. Check if the obstacle is in front of the ego.
    const double lon_dist_from_ego_to_obj =
      object->get_dist_from_ego_longitudinal(traj_points, current_pose.position);
    if (lon_dist_from_ego_to_obj < 0.0) {
      continue;
    }

    // 1.2. Check if the rough lateral distance is smaller than the threshold.
    const double min_lat_dist_to_traj_poly =
      utils::calc_possible_min_dist_from_obj_to_traj_poly(object, traj_points, vehicle_info);
    if (obstacle_filtering_param_.max_lat_margin < min_lat_dist_to_traj_poly) {
      continue;
    }

    // 2. precise filtering
    const auto & decimated_traj_polys = get_decimated_traj_polys(
      traj_points, current_pose, vehicle_info, ego_nearest_dist_threshold,
      ego_nearest_yaw_threshold, trajectory_polygon_collision_check);
    const double dist_from_obj_poly_to_traj_poly =
      object->get_dist_to_traj_poly(decimated_traj_polys);
    const auto slow_down_obstacle = create_slow_down_obstacle_for_predicted_object(
      traj_points, decimated_traj_polys_with_lat_margin, object, predicted_objects_stamp,
      dist_from_obj_poly_to_traj_poly);
    if (slow_down_obstacle) {
      slow_down_obstacles.push_back(*slow_down_obstacle);
      continue;
    }
  }
  slow_down_condition_counter_.remove_counter_unless_updated();
  prev_slow_down_object_obstacles_ = slow_down_obstacles;

  RCLCPP_DEBUG(
    logger_, "The number of output obstacles of filter_slow_down_obstacles is %ld",
    slow_down_obstacles.size());
  return slow_down_obstacles;
}

std::vector<SlowDownObstacle> ObstacleSlowDownModule::filter_slow_down_obstacle_for_point_cloud(
  const std::vector<TrajectoryPoint> & traj_points,
  const std::vector<Polygon2d> & decimated_traj_polys_with_lat_margin,
  const PlannerData::Pointcloud & point_cloud, const VehicleInfo & vehicle_info, size_t ego_idx)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  if (!obstacle_filtering_param_.use_pointcloud) {
    return std::vector<SlowDownObstacle>{};
  }

  // Get Objects
  const std::vector<autoware::motion_velocity_planner::SlowDownPointData> slow_down_points_data =
    convert_point_cloud_to_slow_down_points(
      point_cloud, traj_points, decimated_traj_polys_with_lat_margin, vehicle_info, ego_idx);

  // slow down
  std::vector<SlowDownObstacle> slow_down_obstacles;
  for (const auto & slow_down_point_data : slow_down_points_data) {
    if (!slow_down_point_data.front) {
      continue;
    }
    const auto & front_collision_point = *slow_down_point_data.front;
    const auto & back_collision_point = slow_down_point_data.back.value_or(front_collision_point);

    const auto slow_down_obstacle = create_slow_down_obstacle_for_point_cloud(
      rclcpp::Time(point_cloud.pointcloud.header.stamp), front_collision_point,
      back_collision_point, slow_down_point_data.lat_dist_to_traj);
    slow_down_obstacles.push_back(slow_down_obstacle);
  }

  RCLCPP_DEBUG(
    logger_, "The number of output obstacles of filter_slow_down_obstacles is %ld",
    slow_down_obstacles.size());
  return slow_down_obstacles;
}

std::optional<SlowDownObstacle>
ObstacleSlowDownModule::create_slow_down_obstacle_for_predicted_object(
  const std::vector<TrajectoryPoint> & traj_points,
  const std::vector<Polygon2d> & decimated_traj_polys_with_lat_margin,
  const std::shared_ptr<PlannerData::Object> object, const rclcpp::Time & predicted_objects_stamp,
  const double dist_from_obj_poly_to_traj_poly)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto & p = obstacle_filtering_param_;

  const auto & obj_uuid = object->predicted_object.object_id;
  const auto & obj_uuid_str = autoware_utils::to_hex_string(obj_uuid);
  const auto & obj_label = object->predicted_object.classification.at(0).label;
  slow_down_condition_counter_.add_current_uuid(obj_uuid_str);

  const bool is_prev_obstacle_slow_down =
    utils::get_obstacle_from_uuid(prev_slow_down_object_obstacles_, obj_uuid_str).has_value();

  if (!is_slow_down_obstacle(obj_label)) {
    return std::nullopt;
  }

  if (dist_from_obj_poly_to_traj_poly <= p.min_lat_margin) {
    return std::nullopt;
  }

  // check lateral distance considering hysteresis
  const bool is_lat_dist_low = is_lower_considering_hysteresis(
    dist_from_obj_poly_to_traj_poly, is_prev_obstacle_slow_down,
    p.max_lat_margin + p.lat_hysteresis_margin / 2.0,
    p.max_lat_margin - p.lat_hysteresis_margin / 2.0);

  const bool is_slow_down_required = [&]() {
    if (is_prev_obstacle_slow_down) {
      // check if exiting slow down
      if (!is_lat_dist_low) {
        const int count = slow_down_condition_counter_.decrease_counter(obj_uuid_str);
        if (count <= -p.successive_num_to_exit_slow_down_condition) {
          slow_down_condition_counter_.reset(obj_uuid_str);
          return false;
        }
      }
      return true;
    }
    // check if entering slow down
    if (is_lat_dist_low) {
      const int count = slow_down_condition_counter_.increase_counter(obj_uuid_str);
      if (p.successive_num_to_entry_slow_down_condition <= count) {
        slow_down_condition_counter_.reset(obj_uuid_str);
        return true;
      }
    }
    return false;
  }();
  if (!is_slow_down_required) {
    RCLCPP_DEBUG(
      logger_, "[SlowDown] Ignore obstacle (%s) since it's far from trajectory. (%f [m])",
      obj_uuid_str.substr(0, 4).c_str(), dist_from_obj_poly_to_traj_poly);
    return std::nullopt;
  }

  const auto obstacle_poly = autoware_utils::to_polygon2d(
    object->predicted_object.kinematics.initial_pose_with_covariance.pose,
    object->predicted_object.shape);

  std::vector<Polygon2d> front_collision_polygons;
  size_t front_seg_idx = 0;
  std::vector<Polygon2d> back_collision_polygons;
  size_t back_seg_idx = 0;
  for (size_t i = 0; i < decimated_traj_polys_with_lat_margin.size(); ++i) {
    std::vector<Polygon2d> collision_polygons;
    bg::intersection(decimated_traj_polys_with_lat_margin.at(i), obstacle_poly, collision_polygons);

    if (!collision_polygons.empty()) {
      if (front_collision_polygons.empty()) {
        front_collision_polygons = collision_polygons;
        front_seg_idx = i == 0 ? i : i - 1;
      }
      back_collision_polygons = collision_polygons;
      back_seg_idx = i == 0 ? i : i - 1;
    } else {
      if (!back_collision_polygons.empty()) {
        break;  // for efficient calculation
      }
    }
  }

  if (front_collision_polygons.empty() || back_collision_polygons.empty()) {
    RCLCPP_DEBUG(
      logger_, "[SlowDown] Ignore obstacle (%s) since there is no collision point",
      obj_uuid_str.substr(0, 4).c_str());
    return std::nullopt;
  }

  // calculate front collision point
  double front_min_dist = std::numeric_limits<double>::max();
  geometry_msgs::msg::Point front_collision_point;
  for (const auto & collision_poly : front_collision_polygons) {
    for (const auto & collision_point : collision_poly.outer()) {
      const auto collision_geom_point = to_geom_point(collision_point);
      const double dist = autoware::motion_utils::calcLongitudinalOffsetToSegment(
        traj_points, front_seg_idx, collision_geom_point);
      if (dist < front_min_dist) {
        front_min_dist = dist;
        front_collision_point = collision_geom_point;
      }
    }
  }

  // calculate back collision point
  double back_max_dist = -std::numeric_limits<double>::max();
  geometry_msgs::msg::Point back_collision_point = front_collision_point;
  for (const auto & collision_poly : back_collision_polygons) {
    for (const auto & collision_point : collision_poly.outer()) {
      const auto collision_geom_point = to_geom_point(collision_point);
      const double dist = autoware::motion_utils::calcLongitudinalOffsetToSegment(
        traj_points, back_seg_idx, collision_geom_point);
      if (back_max_dist < dist) {
        back_max_dist = dist;
        back_collision_point = collision_geom_point;
      }
    }
  }

  return SlowDownObstacle{
    obj_uuid_str,
    predicted_objects_stamp,
    object->predicted_object.classification.at(0),
    object->get_predicted_current_pose(clock_->now(), predicted_objects_stamp),
    object->get_lon_vel_relative_to_traj(traj_points),
    object->get_lat_vel_relative_to_traj(traj_points),
    dist_from_obj_poly_to_traj_poly,
    front_collision_point,
    back_collision_point};
}

SlowDownObstacle ObstacleSlowDownModule::create_slow_down_obstacle_for_point_cloud(
  const rclcpp::Time & stamp, const geometry_msgs::msg::Point & front_collision_point,
  const geometry_msgs::msg::Point & back_collision_point, const double lat_dist_to_traj)
{
  const unique_identifier_msgs::msg::UUID obj_uuid;
  const auto & obj_uuid_str = autoware_utils::to_hex_string(obj_uuid);

  ObjectClassification unknown_object_classification;
  unknown_object_classification.label = ObjectClassification::UNKNOWN;
  unknown_object_classification.probability = 1.0;

  const geometry_msgs::msg::Pose unconfigured_pose;

  const double unconfigured_lon_velocity = 0.;
  const double unconfigured_lat_velocity = 0.;

  return SlowDownObstacle{
    obj_uuid_str,
    stamp,
    unknown_object_classification,
    unconfigured_pose,
    unconfigured_lon_velocity,
    unconfigured_lat_velocity,
    lat_dist_to_traj,
    front_collision_point,
    back_collision_point};
}

std::vector<SlowdownInterval> ObstacleSlowDownModule::plan_slow_down(
  const std::shared_ptr<const PlannerData> planner_data,
  const std::vector<TrajectoryPoint> & traj_points, const std::vector<SlowDownObstacle> & obstacles,
  [[maybe_unused]] std::optional<VelocityLimit> & velocity_limit, const VehicleInfo & vehicle_info)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  auto slow_down_traj_points = traj_points;
  slow_down_debug_multi_array_ = Float32MultiArrayStamped();

  const double dist_to_ego = [&]() {
    const size_t ego_seg_idx = planner_data->find_segment_index(
      slow_down_traj_points, planner_data->current_odometry.pose.pose);
    return autoware::motion_utils::calcSignedArcLength(
      slow_down_traj_points, 0, planner_data->current_odometry.pose.pose.position, ego_seg_idx);
  }();
  const double abs_ego_offset = planner_data->is_driving_forward
                                  ? std::abs(vehicle_info.max_longitudinal_offset_m)
                                  : std::abs(vehicle_info.min_longitudinal_offset_m);

  // define function to insert slow down velocity to trajectory
  const auto insert_point_in_trajectory = [&](const double lon_dist) -> std::optional<size_t> {
    const auto inserted_idx =
      autoware::motion_utils::insertTargetPoint(0, lon_dist, slow_down_traj_points);
    if (inserted_idx) {
      if (inserted_idx.value() + 1 <= slow_down_traj_points.size() - 1) {
        // zero-order hold for velocity interpolation
        slow_down_traj_points.at(inserted_idx.value()).longitudinal_velocity_mps =
          slow_down_traj_points.at(inserted_idx.value() + 1).longitudinal_velocity_mps;
      }
      return inserted_idx.value();
    }
    return std::nullopt;
  };

  std::vector<SlowdownInterval> slowdown_intervals;
  std::vector<SlowDownOutput> new_prev_slow_down_output;
  for (size_t i = 0; i < obstacles.size(); ++i) {
    const auto & obstacle = obstacles.at(i);
    const auto prev_output = get_object_from_uuid(prev_slow_down_output_, obstacle.uuid);

    const bool is_obstacle_moving = [&]() -> bool {
      const auto & p = slow_down_planning_param_;
      const auto object_vel_norm = std::hypot(obstacle.velocity, obstacle.lat_velocity);
      if (!prev_output) {
        return object_vel_norm > p.moving_object_speed_threshold;
      }
      if (prev_output->is_obstacle_moving) {
        return object_vel_norm > p.moving_object_speed_threshold - p.moving_object_hysteresis_range;
      }
      return object_vel_norm > p.moving_object_speed_threshold + p.moving_object_hysteresis_range;
    }();

    // calculate slow down start distance, and insert slow down velocity
    const auto dist_vec_to_slow_down = calculate_distance_to_slow_down_with_constraints(
      planner_data, slow_down_traj_points, obstacle, prev_output, dist_to_ego, vehicle_info,
      is_obstacle_moving);
    if (!dist_vec_to_slow_down) {
      RCLCPP_DEBUG(
        logger_, "[SlowDown] Ignore obstacle (%s) since distance to slow down is not valid",
        obstacle.uuid.c_str());
      continue;
    }
    const auto dist_to_slow_down_start = std::get<0>(*dist_vec_to_slow_down);
    const auto dist_to_slow_down_end = std::get<1>(*dist_vec_to_slow_down);
    const auto feasible_slow_down_vel = std::get<2>(*dist_vec_to_slow_down);

    // calculate slow down end distance, and insert slow down velocity
    // NOTE: slow_down_start_idx will not be wrong since inserted back point is after inserted
    // front point.
    const auto slow_down_start_idx = insert_point_in_trajectory(dist_to_slow_down_start);
    const auto slow_down_end_idx = dist_to_slow_down_start < dist_to_slow_down_end
                                     ? insert_point_in_trajectory(dist_to_slow_down_end)
                                     : std::nullopt;
    if (!slow_down_end_idx) {
      continue;
    }

    // calculate slow down velocity
    const double stable_slow_down_vel = [&]() {
      if (prev_output) {
        return autoware::signal_processing::lowpassFilter(
          feasible_slow_down_vel, prev_output->target_vel,
          slow_down_planning_param_.lpf_gain_slow_down_vel);
      }
      return feasible_slow_down_vel;
    }();

    // insert slow down velocity between slow start and end
    slowdown_intervals.push_back(SlowdownInterval{
      slow_down_traj_points.at(slow_down_start_idx ? *slow_down_start_idx : 0).pose.position,
      slow_down_traj_points.at(*slow_down_end_idx).pose.position, stable_slow_down_vel});

    // add debug data
    slow_down_debug_multi_array_.data.push_back(obstacle.dist_to_traj_poly);
    slow_down_debug_multi_array_.data.push_back(dist_to_slow_down_start);
    slow_down_debug_multi_array_.data.push_back(dist_to_slow_down_end);
    slow_down_debug_multi_array_.data.push_back(feasible_slow_down_vel);
    slow_down_debug_multi_array_.data.push_back(stable_slow_down_vel);
    slow_down_debug_multi_array_.data.push_back(slow_down_start_idx ? *slow_down_start_idx : -1.0);
    slow_down_debug_multi_array_.data.push_back(*slow_down_end_idx);

    // add virtual wall
    if (slow_down_start_idx && slow_down_end_idx) {
      const size_t ego_idx =
        planner_data->find_index(slow_down_traj_points, planner_data->current_odometry.pose.pose);
      const size_t slow_down_wall_idx = [&]() {
        if (ego_idx < *slow_down_start_idx) return *slow_down_start_idx;
        if (ego_idx < *slow_down_end_idx) return ego_idx;
        return *slow_down_end_idx;
      }();

      const auto markers = autoware::motion_utils::createSlowDownVirtualWallMarker(
        slow_down_traj_points.at(slow_down_wall_idx).pose, "obstacle slow down", clock_->now(), i,
        abs_ego_offset, "", planner_data->is_driving_forward);
      autoware_utils::append_marker_array(markers, &debug_data_ptr_->slow_down_wall_marker);

      // update planning factor
      planning_factor_interface_->add(
        slow_down_traj_points, planner_data->current_odometry.pose.pose,
        slow_down_traj_points.at(*slow_down_start_idx).pose,
        slow_down_traj_points.at(*slow_down_end_idx).pose, PlanningFactor::SLOW_DOWN,
        SafetyFactorArray{}, planner_data->is_driving_forward, stable_slow_down_vel);
    }

    // add debug virtual wall
    if (slow_down_start_idx) {
      const auto markers = autoware::motion_utils::createSlowDownVirtualWallMarker(
        slow_down_traj_points.at(*slow_down_start_idx).pose, "obstacle slow down start",
        clock_->now(), i * 2, abs_ego_offset, "", planner_data->is_driving_forward);
      autoware_utils::append_marker_array(markers, &debug_data_ptr_->slow_down_debug_wall_marker);
    }
    if (slow_down_end_idx) {
      const auto markers = autoware::motion_utils::createSlowDownVirtualWallMarker(
        slow_down_traj_points.at(*slow_down_end_idx).pose, "obstacle slow down end", clock_->now(),
        i * 2 + 1, abs_ego_offset, "", planner_data->is_driving_forward);
      autoware_utils::append_marker_array(markers, &debug_data_ptr_->slow_down_debug_wall_marker);
    }

    // Add debug data
    debug_data_ptr_->obstacles_to_slow_down.push_back(obstacle);

    // update prev_slow_down_output_
    new_prev_slow_down_output.push_back(SlowDownOutput{
      obstacle.uuid, slow_down_traj_points, slow_down_start_idx, slow_down_end_idx,
      stable_slow_down_vel, feasible_slow_down_vel, obstacle.dist_to_traj_poly,
      is_obstacle_moving});
  }

  // update prev_slow_down_output_
  prev_slow_down_output_ = new_prev_slow_down_output;

  return slowdown_intervals;
}

Float32MultiArrayStamped ObstacleSlowDownModule::get_slow_down_planning_debug_message(
  const rclcpp::Time & current_time)
{
  slow_down_debug_multi_array_.stamp = current_time;
  return slow_down_debug_multi_array_;
}

void ObstacleSlowDownModule::publish_debug_info()
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  // 1. debug marker
  MarkerArray debug_marker;

  // 1.1. obstacles
  for (size_t i = 0; i < debug_data_ptr_->obstacles_to_slow_down.size(); ++i) {
    // obstacle
    const auto obstacle_marker = utils::get_object_marker(
      debug_data_ptr_->obstacles_to_slow_down.at(i).pose, i, "obstacles", 0.7, 0.7, 0.0);
    debug_marker.markers.push_back(obstacle_marker);

    // collision points
    auto front_collision_point_marker = autoware_utils::create_default_marker(
      "map", clock_->now(), "collision_points", i * 2 + 0, Marker::SPHERE,
      autoware_utils::create_marker_scale(0.25, 0.25, 0.25),
      autoware_utils::create_marker_color(1.0, 0.0, 0.0, 0.999));
    front_collision_point_marker.pose.position =
      debug_data_ptr_->obstacles_to_slow_down.at(i).front_collision_point;
    auto back_collision_point_marker = autoware_utils::create_default_marker(
      "map", clock_->now(), "collision_points", i * 2 + 1, Marker::SPHERE,
      autoware_utils::create_marker_scale(0.25, 0.25, 0.25),
      autoware_utils::create_marker_color(1.0, 0.0, 0.0, 0.999));
    back_collision_point_marker.pose.position =
      debug_data_ptr_->obstacles_to_slow_down.at(i).back_collision_point;

    debug_marker.markers.push_back(front_collision_point_marker);
    debug_marker.markers.push_back(back_collision_point_marker);
  }

  // 1.2. slow down debug wall marker
  autoware_utils::append_marker_array(debug_data_ptr_->slow_down_debug_wall_marker, &debug_marker);

  // 1.3. detection area
  auto decimated_traj_polys_marker = autoware_utils::create_default_marker(
    "map", clock_->now(), "detection_area", 0, Marker::LINE_LIST,
    autoware_utils::create_marker_scale(0.01, 0.0, 0.0),
    autoware_utils::create_marker_color(0.0, 1.0, 0.0, 0.999));
  for (const auto & decimated_traj_poly : debug_data_ptr_->decimated_traj_polys) {
    for (size_t dp_idx = 0; dp_idx < decimated_traj_poly.outer().size(); ++dp_idx) {
      const auto & current_point = decimated_traj_poly.outer().at(dp_idx);
      const auto & next_point =
        decimated_traj_poly.outer().at((dp_idx + 1) % decimated_traj_poly.outer().size());

      decimated_traj_polys_marker.points.push_back(
        autoware_utils::create_point(current_point.x(), current_point.y(), 0.0));
      decimated_traj_polys_marker.points.push_back(
        autoware_utils::create_point(next_point.x(), next_point.y(), 0.0));
    }
  }
  debug_marker.markers.push_back(decimated_traj_polys_marker);

  debug_publisher_->publish(debug_marker);

  // 2. virtual wall
  virtual_wall_publisher_->publish(debug_data_ptr_->slow_down_wall_marker);

  // 3. slow down planning info
  const auto slow_down_debug_msg = get_slow_down_planning_debug_message(clock_->now());
  debug_slow_down_planning_info_pub_->publish(slow_down_debug_msg);

  // 4. objects of interest
  objects_of_interest_marker_interface_->publishMarkerArray();

  // 5. processing time
  processing_time_publisher_->publish(create_float64_stamped(clock_->now(), stop_watch_.toc()));

  // 6. planning factor
  planning_factor_interface_->publish();
}

bool ObstacleSlowDownModule::is_slow_down_obstacle(const uint8_t label) const
{
  const auto & types = obstacle_filtering_param_.object_types;
  return std::find(types.begin(), types.end(), label) != types.end();
}

std::optional<std::tuple<double, double, double>>
ObstacleSlowDownModule::calculate_distance_to_slow_down_with_constraints(
  const std::shared_ptr<const PlannerData> planner_data,
  const std::vector<TrajectoryPoint> & traj_points, const SlowDownObstacle & obstacle,
  const std::optional<SlowDownOutput> & prev_output, const double dist_to_ego,
  const VehicleInfo & vehicle_info, const bool is_obstacle_moving) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const double abs_ego_offset = planner_data->is_driving_forward
                                  ? std::abs(vehicle_info.max_longitudinal_offset_m)
                                  : std::abs(vehicle_info.min_longitudinal_offset_m);
  const double obstacle_vel = obstacle.velocity;
  // calculate slow down velocity
  const double slow_down_vel =
    calculate_slow_down_velocity(obstacle, prev_output, is_obstacle_moving);

  // calculate distance to collision points
  const double dist_to_front_collision =
    autoware::motion_utils::calcSignedArcLength(traj_points, 0, obstacle.front_collision_point);
  const double dist_to_back_collision =
    autoware::motion_utils::calcSignedArcLength(traj_points, 0, obstacle.back_collision_point);

  // calculate offset distance to first collision considering relative velocity
  const double relative_vel =
    planner_data->current_odometry.twist.twist.linear.x - obstacle.velocity;
  const double offset_dist_to_collision = [&]() {
    if (dist_to_front_collision < dist_to_ego + abs_ego_offset) {
      return 0.0;
    }

    // NOTE: This min_relative_vel forces the relative velocity positive if the ego velocity is
    // lower than the obstacle velocity. Without this, the slow down feature will flicker where
    // the ego velocity is very close to the obstacle velocity.
    constexpr double min_relative_vel = 1.0;
    const double time_to_collision = (dist_to_front_collision - dist_to_ego - abs_ego_offset) /
                                     std::max(min_relative_vel, relative_vel);

    constexpr double time_to_collision_margin = 1.0;
    const double cropped_time_to_collision =
      std::max(0.0, time_to_collision - time_to_collision_margin);
    return obstacle_vel * cropped_time_to_collision;
  }();

  // calculate distance during deceleration, slow down preparation, and slow down
  const double min_slow_down_prepare_dist = 3.0;
  const double slow_down_prepare_dist = std::max(
    min_slow_down_prepare_dist,
    slow_down_vel * slow_down_planning_param_.time_margin_on_target_velocity);
  const double deceleration_dist = offset_dist_to_collision + dist_to_front_collision -
                                   abs_ego_offset - dist_to_ego - slow_down_prepare_dist;
  const double slow_down_dist =
    dist_to_back_collision - dist_to_front_collision + slow_down_prepare_dist;

  // calculate distance to start/end slow down
  const double dist_to_slow_down_start = dist_to_ego + deceleration_dist;
  const double dist_to_slow_down_end = dist_to_ego + deceleration_dist + slow_down_dist;
  if (100.0 < dist_to_slow_down_start) {
    // NOTE: distance to slow down is too far.
    return std::nullopt;
  }

  // apply low-pass filter to distance to start/end slow down
  const auto apply_lowpass_filter = [&](const double dist_to_slow_down, const auto prev_point) {
    if (prev_output && prev_point) {
      const size_t seg_idx =
        autoware::motion_utils::findNearestSegmentIndex(traj_points, prev_point->position);
      const double prev_dist_to_slow_down =
        autoware::motion_utils::calcSignedArcLength(traj_points, 0, prev_point->position, seg_idx);
      return autoware::signal_processing::lowpassFilter(
        dist_to_slow_down, prev_dist_to_slow_down,
        slow_down_planning_param_.lpf_gain_dist_to_slow_down);
    }
    return dist_to_slow_down;
  };
  const double filtered_dist_to_slow_down_start =
    apply_lowpass_filter(dist_to_slow_down_start, prev_output->start_point);
  const double filtered_dist_to_slow_down_end =
    apply_lowpass_filter(dist_to_slow_down_end, prev_output->end_point);

  // calculate velocity considering constraints
  const double feasible_slow_down_vel = [&]() {
    if (deceleration_dist < 0) {
      if (prev_output) {
        return prev_output->target_vel;
      }
      return std::max(planner_data->current_odometry.twist.twist.linear.x, slow_down_vel);
    }
    if (planner_data->current_odometry.twist.twist.linear.x < slow_down_vel) {
      return slow_down_vel;
    }

    const double one_shot_feasible_slow_down_vel = [&]() {
      if (planner_data->current_acceleration.accel.accel.linear.x < common_param_.min_accel) {
        const double squared_vel =
          std::pow(planner_data->current_odometry.twist.twist.linear.x, 2) +
          2 * deceleration_dist * common_param_.min_accel;
        if (squared_vel < 0) {
          return slow_down_vel;
        }
        return std::max(std::sqrt(squared_vel), slow_down_vel);
      }
      // TODO(murooka) Calculate more precisely. Final acceleration should be zero.
      const double min_feasible_slow_down_vel = calc_deceleration_velocity_from_distance_to_target(
        slow_down_planning_param_.slow_down_min_jerk, slow_down_planning_param_.slow_down_min_acc,
        planner_data->current_acceleration.accel.accel.linear.x,
        planner_data->current_odometry.twist.twist.linear.x, deceleration_dist);
      return min_feasible_slow_down_vel;
    }();
    if (prev_output) {
      // NOTE: If longitudinal controllability is not good, one_shot_slow_down_vel may be getting
      // larger since we use actual ego's velocity and acceleration for its calculation.
      //       Suppress one_shot_slow_down_vel getting larger here.
      const double feasible_slow_down_vel =
        std::min(one_shot_feasible_slow_down_vel, prev_output->feasible_target_vel);
      return std::max(slow_down_vel, feasible_slow_down_vel);
    }
    return std::max(slow_down_vel, one_shot_feasible_slow_down_vel);
  }();

  return std::make_tuple(
    filtered_dist_to_slow_down_start, filtered_dist_to_slow_down_end, feasible_slow_down_vel);
}

double ObstacleSlowDownModule::calculate_slow_down_velocity(
  const SlowDownObstacle & obstacle, const std::optional<SlowDownOutput> & prev_output,
  const bool is_obstacle_moving) const
{
  const auto & p = slow_down_planning_param_.get_object_param_by_label(
    obstacle.classification, is_obstacle_moving);
  const double stable_dist_from_obj_poly_to_traj_poly = [&]() {
    if (prev_output) {
      return autoware::signal_processing::lowpassFilter(
        obstacle.dist_to_traj_poly, prev_output->dist_from_obj_poly_to_traj_poly,
        slow_down_planning_param_.lpf_gain_lat_dist);
    }
    return obstacle.dist_to_traj_poly;
  }();

  const double ratio = std::clamp(
    (std::abs(stable_dist_from_obj_poly_to_traj_poly) - p.min_lat_margin) /
      (p.max_lat_margin - p.min_lat_margin),
    0.0, 1.0);
  const double slow_down_vel =
    p.min_ego_velocity + ratio * (p.max_ego_velocity - p.min_ego_velocity);

  return slow_down_vel;
}

std::vector<Polygon2d> ObstacleSlowDownModule::get_decimated_traj_polys(
  const std::vector<TrajectoryPoint> & traj_points, const geometry_msgs::msg::Pose & current_pose,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const double ego_nearest_dist_threshold, const double ego_nearest_yaw_threshold,
  const TrajectoryPolygonCollisionCheck & trajectory_polygon_collision_check) const
{
  if (!decimated_traj_polys_) {
    const auto & p = trajectory_polygon_collision_check;
    const auto decimated_traj_points = utils::decimate_trajectory_points_from_ego(
      traj_points, current_pose, ego_nearest_dist_threshold, ego_nearest_yaw_threshold,
      p.decimate_trajectory_step_length, p.goal_extended_trajectory_length);
    decimated_traj_polys_ = polygon_utils::create_one_step_polygons(
      decimated_traj_points, vehicle_info, current_pose, 0.0, p.enable_to_consider_current_pose,
      p.time_to_convergence, p.decimate_trajectory_step_length);
  }
  return *decimated_traj_polys_;
}

}  // namespace autoware::motion_velocity_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::motion_velocity_planner::ObstacleSlowDownModule,
  autoware::motion_velocity_planner::PluginModuleInterface)
