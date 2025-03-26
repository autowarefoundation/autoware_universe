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

#include "obstacle_cruise_module.hpp"

#include "autoware_utils/ros/uuid_helper.hpp"

#include <autoware/motion_utils/distance/distance.hpp>
#include <autoware/motion_utils/marker/virtual_wall_marker_creator.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/ros/marker_helper.hpp>
#include <autoware_utils/ros/parameter.hpp>
#include <autoware_utils/ros/update_param.hpp>

#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::motion_velocity_planner
{
namespace
{
double calc_diff_angle_against_trajectory(
  const std::vector<TrajectoryPoint> & traj_points, const geometry_msgs::msg::Pose & target_pose)
{
  const size_t nearest_idx =
    autoware::motion_utils::findNearestIndex(traj_points, target_pose.position);
  const double traj_yaw = tf2::getYaw(traj_points.at(nearest_idx).pose.orientation);

  const double target_yaw = tf2::getYaw(target_pose.orientation);

  const double diff_yaw = autoware_utils::normalize_radian(target_yaw - traj_yaw);
  return diff_yaw;
}

std::vector<PredictedPath> resample_highest_confidence_predicted_paths(
  const std::vector<PredictedPath> & predicted_paths, const double time_interval,
  const double time_horizon, const size_t num_paths)
{
  std::vector<PredictedPath> sorted_paths = predicted_paths;

  // Sort paths by descending confidence
  std::sort(
    sorted_paths.begin(), sorted_paths.end(),
    [](const PredictedPath & a, const PredictedPath & b) { return a.confidence > b.confidence; });

  std::vector<PredictedPath> selected_paths;
  size_t path_count = 0;

  // Select paths that meet the confidence thresholds
  for (const auto & path : sorted_paths) {
    if (path_count < num_paths) {
      selected_paths.push_back(path);
      ++path_count;
    }
  }

  // Resample each selected path
  std::vector<PredictedPath> resampled_paths;
  for (const auto & path : selected_paths) {
    if (path.path.size() < 2) {
      continue;
    }
    resampled_paths.push_back(
      autoware::object_recognition_utils::resamplePredictedPath(path, time_interval, time_horizon));
  }

  return resampled_paths;
}

VelocityLimitClearCommand create_velocity_limit_clear_command(
  const rclcpp::Time & current_time, [[maybe_unused]] const std::string & module_name)
{
  VelocityLimitClearCommand msg;
  msg.stamp = current_time;
  msg.sender = "obstacle_cruise";
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

void ObstacleCruiseModule::init(rclcpp::Node & node, const std::string & module_name)
{
  module_name_ = module_name;
  clock_ = node.get_clock();
  logger_ = node.get_logger();

  // ros parameters
  planning_algorithm_ =
    get_or_declare_parameter<std::string>(node, "obstacle_cruise.option.planning_algorithm");
  common_param_ = CommonParam(node);
  cruise_planning_param_ = CruisePlanningParam(node);
  obstacle_filtering_param_ = ObstacleFilteringParam(node);

  // common publisher
  processing_time_publisher_ =
    node.create_publisher<Float64Stamped>("~/debug/obstacle_cruise/processing_time_ms", 1);
  virtual_wall_publisher_ =
    node.create_publisher<MarkerArray>("~/obstacle_cruise/virtual_walls", 1);
  debug_publisher_ = node.create_publisher<MarkerArray>("~/obstacle_cruise/debug_markers", 1);

  // module publisher
  metrics_pub_ = node.create_publisher<MetricArray>("~/cruise/metrics", 10);
  debug_cruise_planning_info_pub_ =
    node.create_publisher<Float32MultiArrayStamped>("~/debug/cruise_planning_info", 1);
  processing_time_detail_pub_ = node.create_publisher<autoware_utils::ProcessingTimeDetail>(
    "~/debug/processing_time_detail_ms/obstacle_cruise", 1);

  // interface
  objects_of_interest_marker_interface_ = std::make_unique<
    autoware::objects_of_interest_marker_interface::ObjectsOfInterestMarkerInterface>(
    &node, "obstacle_cruise");
  planning_factor_interface_ =
    std::make_unique<autoware::planning_factor_interface::PlanningFactorInterface>(
      &node, "obstacle_cruise");

  // time keeper
  time_keeper_ = std::make_shared<autoware_utils::TimeKeeper>(processing_time_detail_pub_);

  // cruise planner
  cruise_planner_ = create_cruise_planner(node);
}

void ObstacleCruiseModule::update_parameters(const std::vector<rclcpp::Parameter> & parameters)
{
  cruise_planner_->update_parameters(parameters);
}

VelocityPlanningResult ObstacleCruiseModule::plan(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & raw_trajectory_points,
  [[maybe_unused]] const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> &
    smoothed_trajectory_points,
  const std::shared_ptr<const PlannerData> planner_data)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  // 1. init variables
  stop_watch_.tic();
  debug_data_ptr_ = std::make_shared<DebugData>();
  metrics_manager_.init();

  // filter obstacles of predicted objects
  const auto cruise_obstacles = filter_cruise_obstacle_for_predicted_object(
    planner_data->current_odometry, planner_data->ego_nearest_dist_threshold,
    planner_data->ego_nearest_yaw_threshold, raw_trajectory_points, planner_data->objects,
    rclcpp::Time(planner_data->predicted_objects_header.stamp), planner_data->is_driving_forward,
    planner_data->vehicle_info_, planner_data->trajectory_polygon_collision_check);

  // plan cruise
  VelocityPlanningResult result;
  [[maybe_unused]] const auto cruise_traj_points = cruise_planner_->plan_cruise(
    planner_data, raw_trajectory_points, cruise_obstacles, debug_data_ptr_,
    planning_factor_interface_, result.velocity_limit);
  metrics_manager_.calculate_metrics("PlannerInterface", "cruise");

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

std::string ObstacleCruiseModule::get_module_name() const
{
  return module_name_;
}

std::unique_ptr<CruisePlannerInterface> ObstacleCruiseModule::create_cruise_planner(
  rclcpp::Node & node) const
{
  if (planning_algorithm_ == "pid_base") {
    return std::make_unique<PIDBasedPlanner>(node, common_param_, cruise_planning_param_);
  } else if (planning_algorithm_ == "optimization_base") {
    return std::make_unique<OptimizationBasedPlanner>(node, common_param_, cruise_planning_param_);
  }
  throw std::logic_error("Designated algorithm is not supported.");
}

std::vector<CruiseObstacle> ObstacleCruiseModule::filter_cruise_obstacle_for_predicted_object(
  const Odometry & odometry, const double ego_nearest_dist_threshold,
  const double ego_nearest_yaw_threshold, const std::vector<TrajectoryPoint> & traj_points,
  const std::vector<std::shared_ptr<PlannerData::Object>> & objects,
  const rclcpp::Time & predicted_objects_stamp, const bool is_driving_forward,
  const VehicleInfo & vehicle_info,
  const TrajectoryPolygonCollisionCheck & trajectory_polygon_collision_check)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto & current_pose = odometry.pose.pose;

  const auto & p = trajectory_polygon_collision_check;
  const auto decimated_traj_points = utils::decimate_trajectory_points_from_ego(
    traj_points, current_pose, ego_nearest_dist_threshold, ego_nearest_yaw_threshold,
    p.decimate_trajectory_step_length, 0.0);
  const auto decimated_traj_polys = polygon_utils::create_one_step_polygons(
    decimated_traj_points, vehicle_info, current_pose, 0.0, p.enable_to_consider_current_pose,
    p.time_to_convergence, p.decimate_trajectory_step_length);
  debug_data_ptr_->decimated_traj_polys = decimated_traj_polys;

  // cruise
  std::vector<CruiseObstacle> cruise_obstacles;
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

    // 2. precise filtering for cruise
    const auto cruise_obstacle = create_cruise_obstacle(
      odometry, traj_points, decimated_traj_points, decimated_traj_polys, object,
      predicted_objects_stamp, object->get_dist_to_traj_poly(decimated_traj_polys),
      is_driving_forward, vehicle_info, trajectory_polygon_collision_check);
    if (cruise_obstacle) {
      cruise_obstacles.push_back(*cruise_obstacle);
      continue;
    }
  }

  // 3. precise filtering for yield cruise
  if (obstacle_filtering_param_.enable_yield) {
    const auto yield_obstacles = find_yield_cruise_obstacles(
      odometry, objects, predicted_objects_stamp, traj_points, vehicle_info);
    if (yield_obstacles) {
      for (const auto & y : yield_obstacles.value()) {
        // Check if there is no member with the same UUID in cruise_obstacles
        auto it = std::find_if(
          cruise_obstacles.begin(), cruise_obstacles.end(),
          [&y](const auto & c) { return y.uuid == c.uuid; });

        // If no matching UUID found, insert yield obstacle into cruise_obstacles
        if (it == cruise_obstacles.end()) {
          cruise_obstacles.push_back(y);
        }
      }
    }
  }

  prev_cruise_object_obstacles_ = cruise_obstacles;

  return cruise_obstacles;
}

void ObstacleCruiseModule::publish_debug_info()
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  // 1. debug marker
  MarkerArray debug_marker;

  // 1.1. obstacles
  std::vector<geometry_msgs::msg::Point> stop_collision_points;
  for (size_t i = 0; i < debug_data_ptr_->obstacles_to_cruise.size(); ++i) {
    // obstacle
    const auto obstacle_marker = utils::get_object_marker(
      debug_data_ptr_->obstacles_to_cruise.at(i).pose, i, "obstacles", 1.0, 0.6, 0.1);
    debug_marker.markers.push_back(obstacle_marker);

    // collision points
    for (size_t j = 0; j < debug_data_ptr_->obstacles_to_cruise.at(i).collision_points.size();
         ++j) {
      stop_collision_points.push_back(
        debug_data_ptr_->obstacles_to_cruise.at(i).collision_points.at(j).point);
    }
  }

  // 1.2. collision points
  for (size_t i = 0; i < stop_collision_points.size(); ++i) {
    auto collision_point_marker = autoware_utils::create_default_marker(
      "map", clock_->now(), "collision_points", i, Marker::SPHERE,
      autoware_utils::create_marker_scale(0.25, 0.25, 0.25),
      autoware_utils::create_marker_color(1.0, 0.0, 0.0, 0.999));
    collision_point_marker.pose.position = stop_collision_points.at(i);
    debug_marker.markers.push_back(collision_point_marker);
  }

  // 1.3. intentionally ignored obstacles
  for (size_t i = 0; i < debug_data_ptr_->intentionally_ignored_obstacles.size(); ++i) {
    const auto marker = utils::get_object_marker(
      debug_data_ptr_->intentionally_ignored_obstacles.at(i)
        ->predicted_object.kinematics.initial_pose_with_covariance.pose,
      i, "intentionally_ignored_obstacles", 0.0, 1.0, 0.0);
    debug_marker.markers.push_back(marker);
  }

  // 1.4. detection area
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
  virtual_wall_publisher_->publish(debug_data_ptr_->cruise_wall_marker);

  // 3. cruise planning info
  const auto cruise_debug_msg = cruise_planner_->get_cruise_planning_debug_message(clock_->now());
  debug_cruise_planning_info_pub_->publish(cruise_debug_msg);

  // 4. objects of interest
  objects_of_interest_marker_interface_->publishMarkerArray();

  // 5. metrics
  const auto metrics_msg = metrics_manager_.create_metric_array(clock_->now());
  metrics_pub_->publish(metrics_msg);

  // 6. processing time
  processing_time_publisher_->publish(create_float64_stamped(clock_->now(), stop_watch_.toc()));

  // 7. planning factor
  planning_factor_interface_->publish();
}

std::optional<CruiseObstacle> ObstacleCruiseModule::create_cruise_obstacle(
  const Odometry & odometry, const std::vector<TrajectoryPoint> & traj_points,
  const std::vector<TrajectoryPoint> & decimated_traj_points,
  const std::vector<Polygon2d> & decimated_traj_polys,
  const std::shared_ptr<PlannerData::Object> object, const rclcpp::Time & predicted_objects_stamp,
  const double dist_from_obj_poly_to_traj_poly, const bool is_driving_forward,
  const VehicleInfo & vehicle_info,
  const TrajectoryPolygonCollisionCheck & trajectory_polygon_collision_check) const
{
  const auto & obj_uuid = object->predicted_object.object_id;
  const auto & obj_uuid_str = autoware_utils::to_hex_string(obj_uuid);

  // NOTE: When driving backward, Stop will be planned instead of cruise.
  //       When the obstacle is crossing the ego's trajectory, cruise can be ignored.
  if (
    !is_cruise_obstacle(object->predicted_object.classification.at(0).label) ||
    !is_driving_forward) {
    return std::nullopt;
  }

  if (object->get_lon_vel_relative_to_traj(traj_points) < 0.0) {
    RCLCPP_DEBUG(
      logger_, "[Cruise] Ignore obstacle (%s) since it's driving in opposite direction.",
      obj_uuid_str.substr(0, 4).c_str());
    return std::nullopt;
  }

  if (obstacle_filtering_param_.max_lat_margin < dist_from_obj_poly_to_traj_poly) {
    const auto time_to_traj = dist_from_obj_poly_to_traj_poly /
                              std::max(1e-6, object->get_lat_vel_relative_to_traj(traj_points));
    if (time_to_traj > obstacle_filtering_param_.max_lateral_time_margin) {
      RCLCPP_DEBUG(
        logger_, "[Cruise] Ignore obstacle (%s) since it's far from trajectory.",
        obj_uuid_str.substr(0, 4).c_str());
      return std::nullopt;
    }
  }

  if (is_obstacle_crossing(traj_points, object)) {
    RCLCPP_DEBUG(
      logger_, "[Cruise] Ignore obstacle (%s) since it's crossing the ego's trajectory..",
      obj_uuid_str.substr(0, 4).c_str());
    return std::nullopt;
  }

  const auto collision_points = [&]() -> std::optional<std::vector<polygon_utils::PointWithStamp>> {
    constexpr double epsilon = 1e-6;
    if (dist_from_obj_poly_to_traj_poly < epsilon) {
      // obstacle is inside the trajectory
      return create_collision_points_for_inside_cruise_obstacle(
        traj_points, decimated_traj_points, decimated_traj_polys, object, predicted_objects_stamp,
        is_driving_forward, vehicle_info, trajectory_polygon_collision_check);
    }
    // obstacle is outside the trajectory
    // If the ego is stopping, do not plan cruise for outside obstacles. Stop will be planned.
    if (odometry.twist.twist.linear.x < 0.1) {
      return std::nullopt;
    }
    return create_collision_points_for_outside_cruise_obstacle(
      traj_points, decimated_traj_points, decimated_traj_polys, object, predicted_objects_stamp,
      is_driving_forward, vehicle_info, trajectory_polygon_collision_check);
  }();
  if (!collision_points) {
    return std::nullopt;
  }

  return CruiseObstacle{
    obj_uuid_str,
    predicted_objects_stamp,
    object->get_predicted_pose(clock_->now(), predicted_objects_stamp),
    object->get_lon_vel_relative_to_traj(traj_points),
    object->get_lat_vel_relative_to_traj(traj_points),
    *collision_points};
}

std::optional<std::vector<CruiseObstacle>> ObstacleCruiseModule::find_yield_cruise_obstacles(
  const Odometry & odometry, const std::vector<std::shared_ptr<PlannerData::Object>> & objects,
  const rclcpp::Time & predicted_objects_stamp, const std::vector<TrajectoryPoint> & traj_points,
  const VehicleInfo & vehicle_info)
{
  const auto & current_pose = odometry.pose.pose;

  if (objects.empty() || traj_points.empty()) return std::nullopt;

  std::vector<std::shared_ptr<PlannerData::Object>> stopped_objects;
  std::vector<std::shared_ptr<PlannerData::Object>> moving_objects;

  std::for_each(objects.begin(), objects.end(), [&](const auto & o) {
    const bool is_moving =
      std::hypot(
        o->predicted_object.kinematics.initial_twist_with_covariance.twist.linear.x,
        o->predicted_object.kinematics.initial_twist_with_covariance.twist.linear.y) >
      obstacle_filtering_param_.stopped_obstacle_velocity_threshold;
    if (is_moving) {
      const bool is_within_lat_dist_threshold =
        std::abs(o->get_dist_to_traj_lateral(traj_points)) <
        obstacle_filtering_param_.yield_lat_distance_threshold;
      if (is_within_lat_dist_threshold) moving_objects.push_back(o);
      return;
    }
    // lat threshold is larger for stopped obstacles
    const bool is_within_lat_dist_threshold =
      std::abs(o->get_dist_to_traj_lateral(traj_points)) <
      obstacle_filtering_param_.yield_lat_distance_threshold +
        obstacle_filtering_param_.max_lat_dist_between_obstacles;
    if (is_within_lat_dist_threshold) stopped_objects.push_back(o);
    return;
  });

  if (stopped_objects.empty() || moving_objects.empty()) return std::nullopt;

  std::sort(stopped_objects.begin(), stopped_objects.end(), [&](const auto & o1, const auto & o2) {
    return o1->get_dist_from_ego_longitudinal(traj_points, current_pose.position) <
           o2->get_dist_from_ego_longitudinal(traj_points, current_pose.position);
  });

  std::sort(moving_objects.begin(), moving_objects.end(), [&](const auto & o1, const auto & o2) {
    return o1->get_dist_from_ego_longitudinal(traj_points, current_pose.position) <
           o2->get_dist_from_ego_longitudinal(traj_points, current_pose.position);
  });

  std::vector<CruiseObstacle> yield_obstacles;
  for (const auto & moving_object : moving_objects) {
    for (const auto & stopped_object : stopped_objects) {
      const bool is_moving_obs_behind_of_stopped_obs =
        moving_object->get_dist_from_ego_longitudinal(traj_points, current_pose.position) <
        stopped_object->get_dist_from_ego_longitudinal(traj_points, current_pose.position);
      const bool is_moving_obs_ahead_of_ego_front =
        moving_object->get_dist_from_ego_longitudinal(traj_points, current_pose.position) >
        vehicle_info.vehicle_length_m;

      if (!is_moving_obs_ahead_of_ego_front || !is_moving_obs_behind_of_stopped_obs) continue;

      const double lateral_distance_between_obstacles = std::abs(
        moving_object->get_dist_to_traj_lateral(traj_points) -
        stopped_object->get_dist_to_traj_lateral(traj_points));

      const double longitudinal_distance_between_obstacles = std::abs(
        moving_object->get_dist_from_ego_longitudinal(traj_points, current_pose.position) -
        stopped_object->get_dist_from_ego_longitudinal(traj_points, current_pose.position));

      const double moving_object_speed = std::hypot(
        moving_object->predicted_object.kinematics.initial_twist_with_covariance.twist.linear.x,
        moving_object->predicted_object.kinematics.initial_twist_with_covariance.twist.linear.y);

      const bool are_obstacles_aligned = lateral_distance_between_obstacles <
                                         obstacle_filtering_param_.max_lat_dist_between_obstacles;
      const bool obstacles_collide_within_threshold_time =
        longitudinal_distance_between_obstacles / moving_object_speed <
        obstacle_filtering_param_.max_obstacles_collision_time;
      if (are_obstacles_aligned && obstacles_collide_within_threshold_time) {
        const auto yield_obstacle = create_yield_cruise_obstacle(
          moving_object, stopped_object, predicted_objects_stamp, traj_points);
        if (yield_obstacle) {
          yield_obstacles.push_back(*yield_obstacle);
          using autoware::objects_of_interest_marker_interface::ColorName;
          objects_of_interest_marker_interface_->insertObjectData(
            stopped_object->predicted_object.kinematics.initial_pose_with_covariance.pose,
            stopped_object->predicted_object.shape, ColorName::RED);
        }
      }
    }
  }
  if (yield_obstacles.empty()) return std::nullopt;
  return yield_obstacles;
}

std::optional<std::vector<polygon_utils::PointWithStamp>>
ObstacleCruiseModule::create_collision_points_for_inside_cruise_obstacle(
  const std::vector<TrajectoryPoint> & traj_points,
  const std::vector<TrajectoryPoint> & decimated_traj_points,
  const std::vector<Polygon2d> & decimated_traj_polys,
  const std::shared_ptr<PlannerData::Object> object, const rclcpp::Time & predicted_objects_stamp,
  const bool is_driving_forward, const VehicleInfo & vehicle_info,
  const TrajectoryPolygonCollisionCheck & trajectory_polygon_collision_check) const
{
  const auto & obj_uuid = object->predicted_object.object_id;
  const auto & obj_uuid_str = autoware_utils::to_hex_string(obj_uuid);

  // check label
  if (!is_inside_cruise_obstacle(object->predicted_object.classification.at(0).label)) {
    RCLCPP_DEBUG(
      logger_, "[Cruise] Ignore inside obstacle (%s) since its type is not designated.",
      obj_uuid_str.substr(0, 4).c_str());
    return std::nullopt;
  }

  {  // consider hysteresis
    // const bool is_prev_obstacle_stop = get_obstacle_from_uuid(prev_stop_obstacles_,
    // obstacle.uuid).has_value();
    const bool is_prev_obstacle_cruise =
      utils::get_obstacle_from_uuid(prev_cruise_object_obstacles_, obj_uuid_str).has_value();

    if (is_prev_obstacle_cruise) {
      if (
        object->get_lon_vel_relative_to_traj(traj_points) <
        obstacle_filtering_param_.obstacle_velocity_threshold_from_cruise) {
        return std::nullopt;
      }
      // NOTE: else is keeping cruise
    } else {
      // If previous obstacle is stop or does not exist.
      if (
        object->get_lon_vel_relative_to_traj(traj_points) <
        obstacle_filtering_param_.obstacle_velocity_threshold_to_cruise) {
        return std::nullopt;
      }
      // NOTE: else is cruise from stop
    }
  }

  // Get highest confidence predicted path
  constexpr double prediction_resampling_time_interval = 0.1;
  constexpr double prediction_resampling_time_horizon = 10.0;
  std::vector<PredictedPath> predicted_paths;
  for (const auto & path : object->predicted_object.kinematics.predicted_paths) {
    predicted_paths.push_back(path);
  }
  const auto resampled_predicted_paths = resample_highest_confidence_predicted_paths(
    predicted_paths, prediction_resampling_time_interval, prediction_resampling_time_horizon, 1);

  if (resampled_predicted_paths.empty()) {
    return std::nullopt;
  }

  // calculate nearest collision point
  std::vector<size_t> collision_index;
  const auto collision_points = polygon_utils::get_collision_points(
    decimated_traj_points, decimated_traj_polys, predicted_objects_stamp,
    resampled_predicted_paths.front(), object->predicted_object.shape, clock_->now(),
    is_driving_forward, collision_index,
    utils::calc_object_possible_max_dist_from_center(object->predicted_object.shape) +
      trajectory_polygon_collision_check.decimate_trajectory_step_length +
      std::hypot(
        vehicle_info.vehicle_length_m,
        vehicle_info.vehicle_width_m * 0.5 + obstacle_filtering_param_.max_lat_margin));
  return collision_points;
}

std::optional<std::vector<polygon_utils::PointWithStamp>>
ObstacleCruiseModule::create_collision_points_for_outside_cruise_obstacle(
  const std::vector<TrajectoryPoint> & traj_points,
  const std::vector<TrajectoryPoint> & decimated_traj_points,
  const std::vector<Polygon2d> & decimated_traj_polys,
  const std::shared_ptr<PlannerData::Object> object, const rclcpp::Time & predicted_objects_stamp,
  const bool is_driving_forward, const VehicleInfo & vehicle_info,
  const TrajectoryPolygonCollisionCheck & trajectory_polygon_collision_check) const
{
  const auto & obj_uuid = object->predicted_object.object_id;
  const auto & obj_uuid_str = autoware_utils::to_hex_string(obj_uuid);

  // check label
  if (!is_outside_cruise_obstacle(object->predicted_object.classification.at(0).label)) {
    RCLCPP_DEBUG(
      logger_, "[Cruise] Ignore outside obstacle (%s) since its type is not designated.",
      obj_uuid_str.substr(0, 4).c_str());
    return std::nullopt;
  }

  if (
    std::hypot(
      object->predicted_object.kinematics.initial_twist_with_covariance.twist.linear.x,
      object->predicted_object.kinematics.initial_twist_with_covariance.twist.linear.y) <
    obstacle_filtering_param_.outside_obstacle_velocity_threshold) {
    RCLCPP_DEBUG(
      logger_, "[Cruise] Ignore outside obstacle (%s) since the obstacle velocity is low.",
      obj_uuid_str.substr(0, 4).c_str());
    return std::nullopt;
  }

  // Get the highest confidence predicted paths
  constexpr double prediction_resampling_time_interval = 0.1;
  constexpr double prediction_resampling_time_horizon = 10.0;
  std::vector<PredictedPath> predicted_paths;
  for (const auto & path : object->predicted_object.kinematics.predicted_paths) {
    predicted_paths.push_back(path);
  }

  const auto resampled_predicted_paths = resample_highest_confidence_predicted_paths(
    predicted_paths, prediction_resampling_time_interval, prediction_resampling_time_horizon,
    obstacle_filtering_param_.num_of_predicted_paths_for_outside_cruise_obstacle);

  // calculate collision condition for cruise
  std::vector<size_t> collision_index;
  const auto get_collision_points = [&]() -> std::vector<polygon_utils::PointWithStamp> {
    for (const auto & predicted_path : resampled_predicted_paths) {
      const auto collision_points = polygon_utils::get_collision_points(
        decimated_traj_points, decimated_traj_polys, predicted_objects_stamp, predicted_path,
        object->predicted_object.shape, clock_->now(), is_driving_forward, collision_index,
        utils::calc_object_possible_max_dist_from_center(object->predicted_object.shape) +
          trajectory_polygon_collision_check.decimate_trajectory_step_length +
          std::hypot(
            vehicle_info.vehicle_length_m,
            vehicle_info.vehicle_width_m * 0.5 + obstacle_filtering_param_.max_lat_margin),
        obstacle_filtering_param_.max_prediction_time_for_collision_check);
      if (!collision_points.empty()) {
        return collision_points;
      }
    }
    return {};
  };

  const auto collision_points = get_collision_points();

  if (collision_points.empty()) {
    // Ignore vehicle obstacles outside the trajectory without collision
    RCLCPP_DEBUG(
      logger_, "[Cruise] Ignore outside obstacle (%s) since there are no collision points.",
      obj_uuid_str.substr(0, 4).c_str());
    debug_data_ptr_->intentionally_ignored_obstacles.push_back(object);
    return std::nullopt;
  }

  const double overlap_time =
    (rclcpp::Time(collision_points.back().stamp) - rclcpp::Time(collision_points.front().stamp))
      .seconds();
  if (overlap_time < obstacle_filtering_param_.ego_obstacle_overlap_time_threshold) {
    // Ignore vehicle obstacles outside the trajectory, whose predicted path
    // overlaps the ego trajectory in a certain time.
    RCLCPP_DEBUG(
      logger_, "[Cruise] Ignore outside obstacle (%s) since it will not collide with the ego.",
      obj_uuid_str.substr(0, 4).c_str());
    debug_data_ptr_->intentionally_ignored_obstacles.push_back(object);
    return std::nullopt;
  }

  // Ignore obstacles behind the ego vehicle.
  // Note: Only using isFrontObstacle(), behind obstacles cannot be filtered
  // properly when the trajectory is crossing or overlapping.
  const size_t first_collision_index = collision_index.front();
  if (!is_front_collide_obstacle(traj_points, object, first_collision_index)) {
    return std::nullopt;
  }
  return collision_points;
}

std::optional<CruiseObstacle> ObstacleCruiseModule::create_yield_cruise_obstacle(
  const std::shared_ptr<PlannerData::Object> object,
  const std::shared_ptr<PlannerData::Object> stopped_object,
  const rclcpp::Time & predicted_objects_stamp, const std::vector<TrajectoryPoint> & traj_points)
{
  if (traj_points.empty()) return std::nullopt;
  // check label

  const auto & obj_uuid = object->predicted_object.object_id;
  const auto & obj_uuid_str = autoware_utils::to_hex_string(obj_uuid);

  if (!is_outside_cruise_obstacle(object->predicted_object.classification.at(0).label)) {
    RCLCPP_DEBUG(
      logger_, "[Cruise] Ignore yield obstacle (%s) since its type is not designated.",
      obj_uuid_str.substr(0, 4).c_str());
    return std::nullopt;
  }

  if (!is_side_stopped_obstacle(stopped_object->predicted_object.classification.at(0).label)) {
    RCLCPP_DEBUG(
      logger_,
      "[Cruise] Ignore yield obstacle (%s) since the corresponding stopped object type is not "
      "designated as side_stopped.",
      obj_uuid_str.substr(0, 4).c_str());
    return std::nullopt;
  }

  if (
    std::hypot(
      object->predicted_object.kinematics.initial_twist_with_covariance.twist.linear.x,
      object->predicted_object.kinematics.initial_twist_with_covariance.twist.linear.y) <
    obstacle_filtering_param_.outside_obstacle_velocity_threshold) {
    RCLCPP_DEBUG(
      logger_, "[Cruise] Ignore yield obstacle (%s) since the obstacle velocity is low.",
      obj_uuid_str.substr(0, 4).c_str());
    return std::nullopt;
  }

  if (is_obstacle_crossing(traj_points, object)) {
    RCLCPP_DEBUG(
      logger_, "[Cruise] Ignore yield obstacle (%s) since it's crossing the ego's trajectory..",
      obj_uuid_str.substr(0, 4).c_str());
    return std::nullopt;
  }

  const auto collision_points = [&]() -> std::optional<std::vector<polygon_utils::PointWithStamp>> {
    const auto obstacle_idx = autoware::motion_utils::findNearestIndex(
      traj_points, object->predicted_object.kinematics.initial_pose_with_covariance.pose);
    if (!obstacle_idx) return std::nullopt;
    const auto collision_traj_point = traj_points.at(obstacle_idx.value());
    const auto object_time = clock_->now() + traj_points.at(obstacle_idx.value()).time_from_start;

    polygon_utils::PointWithStamp collision_traj_point_with_stamp;
    collision_traj_point_with_stamp.stamp = object_time;
    collision_traj_point_with_stamp.point.x = collision_traj_point.pose.position.x;
    collision_traj_point_with_stamp.point.y = collision_traj_point.pose.position.y;
    collision_traj_point_with_stamp.point.z = collision_traj_point.pose.position.z;
    std::vector<polygon_utils::PointWithStamp> collision_points_vector{
      collision_traj_point_with_stamp};
    return collision_points_vector;
  }();

  if (!collision_points) return std::nullopt;
  // check if obstacle is driving on the opposite direction
  if (object->get_lon_vel_relative_to_traj(traj_points) < 0.0) return std::nullopt;
  return CruiseObstacle{
    obj_uuid_str,
    predicted_objects_stamp,
    object->get_predicted_pose(clock_->now(), predicted_objects_stamp),
    object->get_lon_vel_relative_to_traj(traj_points),
    object->get_lat_vel_relative_to_traj(traj_points),
    collision_points.value(),
    true};
}

bool ObstacleCruiseModule::is_inside_cruise_obstacle(const uint8_t label) const
{
  const auto & types = obstacle_filtering_param_.inside_object_types;
  return std::find(types.begin(), types.end(), label) != types.end();
}

bool ObstacleCruiseModule::is_outside_cruise_obstacle(const uint8_t label) const
{
  const auto & types = obstacle_filtering_param_.outside_object_types;
  return std::find(types.begin(), types.end(), label) != types.end();
}

bool ObstacleCruiseModule::is_side_stopped_obstacle(const uint8_t label) const
{
  const auto & types = obstacle_filtering_param_.side_stopped_object_types;
  return std::find(types.begin(), types.end(), label) != types.end();
}

bool ObstacleCruiseModule::is_cruise_obstacle(const uint8_t label) const
{
  return is_inside_cruise_obstacle(label) || is_outside_cruise_obstacle(label);
}

bool ObstacleCruiseModule::is_front_collide_obstacle(
  const std::vector<TrajectoryPoint> & traj_points,
  const std::shared_ptr<PlannerData::Object> object, const size_t first_collision_idx) const
{
  const auto obstacle_idx = autoware::motion_utils::findNearestIndex(
    traj_points, object->predicted_object.kinematics.initial_pose_with_covariance.pose.position);

  const double obstacle_to_col_points_distance =
    autoware::motion_utils::calcSignedArcLength(traj_points, obstacle_idx, first_collision_idx);
  const double object_possible_max_dist_from_center =
    utils::calc_object_possible_max_dist_from_center(object->predicted_object.shape);

  // If the obstacle is far in front of the collision point, the obstacle is behind the ego.
  return obstacle_to_col_points_distance > -object_possible_max_dist_from_center;
}

bool ObstacleCruiseModule::is_obstacle_crossing(
  const std::vector<TrajectoryPoint> & traj_points,
  const std::shared_ptr<PlannerData::Object> object) const
{
  const double diff_angle = calc_diff_angle_against_trajectory(
    traj_points, object->predicted_object.kinematics.initial_pose_with_covariance.pose);

  // NOTE: Currently predicted objects does not have orientation availability even
  // though sometimes orientation is not available.
  const bool is_obstacle_crossing_trajectory =
    obstacle_filtering_param_.crossing_obstacle_traj_angle_threshold < std::abs(diff_angle) &&
    obstacle_filtering_param_.crossing_obstacle_traj_angle_threshold < M_PI - std::abs(diff_angle);
  if (!is_obstacle_crossing_trajectory) {
    return false;
  }

  // Only obstacles crossing the ego's trajectory with high speed are considered.
  return true;
}
}  // namespace autoware::motion_velocity_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::motion_velocity_planner::ObstacleCruiseModule,
  autoware::motion_velocity_planner::PluginModuleInterface)
