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

#include "rear_collision_checker.hpp"

#include "utils.hpp"

#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware/signal_processing/lowpass_filter_1d.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_lanelet2_extension/visualization/visualization.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/ros/parameter.hpp>
#include <autoware_utils/ros/update_param.hpp>
#include <autoware_utils/transform/transforms.hpp>
#include <magic_enum.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <autoware_internal_planning_msgs/msg/planning_factor.hpp>
#include <autoware_internal_planning_msgs/msg/safety_factor_array.hpp>

#include <pcl/filters/crop_hull.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace autoware::planning_validator
{
using autoware_internal_planning_msgs::msg::PlanningFactor;
using autoware_internal_planning_msgs::msg::SafetyFactor;
using autoware_internal_planning_msgs::msg::SafetyFactorArray;
using autoware_utils::get_or_declare_parameter;

void RearCollisionChecker::init(
  rclcpp::Node & node, const std::string & name,
  const std::shared_ptr<PlanningValidatorContext> & context)
{
  module_name_ = name;

  clock_ = node.get_clock();

  logger_ = node.get_logger();

  context_ = context;

  last_safe_time_ = clock_->now();

  last_unsafe_time_ = clock_->now();

  param_listener_ = std::make_unique<rear_collision_checker_node::ParamListener>(
    node.get_node_parameters_interface());

  pub_cluster_pointcloud_ =
    node.create_publisher<PointCloud2>("~/rear_collision_checker/debug/cluster_points", 1);

  pub_voxel_pointcloud_ =
    node.create_publisher<PointCloud2>("~/rear_collision_checker/debug/voxel_points", 1);

  pub_string_ = node.create_publisher<StringStamped>("~/rear_collision_checker/debug/state", 1);

  pub_debug_processing_time_detail_ = node.create_publisher<autoware_utils::ProcessingTimeDetail>(
    "~/rear_collision_checker/debug/processing_time_detail_ms", 1);

  time_keeper_ = std::make_shared<autoware_utils::TimeKeeper>(pub_debug_processing_time_detail_);

  planning_factor_interface_ =
    std::make_unique<autoware::planning_factor_interface::PlanningFactorInterface>(
      &node, "rear_collision_checker");

  setup_diag();
}

void RearCollisionChecker::validate(bool & is_critical)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  const auto start_time = clock_->now();

  DebugData debug_data;

  context_->validation_status->is_valid_rear_collision_check = is_safe(debug_data);

  is_critical = false;

  post_process();

  debug_data.processing_time_detail_ms = (clock_->now() - start_time).seconds() * 1e3;

  publish_marker(debug_data);
  publish_planning_factor(debug_data);
}

void RearCollisionChecker::setup_diag()
{
  context_->add_diag(
    "rear_collision_check", context_->validation_status->is_valid_rear_collision_check,
    "obstacle detected behind the vehicle");
}

void RearCollisionChecker::fill_rss_distance(PointCloudObjects & objects) const
{
  const auto p = param_listener_->get_params();
  const auto & max_deceleration_ego = p.common.ego.max_deceleration;
  const auto & current_velocity = context_->data->current_kinematics->twist.twist.linear.x;

  for (auto & object : objects) {
    if (object.is_vru) {
      const auto & delay_object = p.common.vulnerable_road_user.reaction_time;
      const auto & max_deceleration_object = p.common.vulnerable_road_user.max_deceleration;
      const auto stop_distance_object =
        delay_object * object.velocity +
        0.5 * std::pow(object.velocity, 2.0) / std::abs(max_deceleration_object);
      const auto stop_distance_ego =
        0.5 * std::pow(current_velocity, 2.0) / std::abs(max_deceleration_ego);

      object.rss_distance = stop_distance_object - stop_distance_ego;
      object.safe = object.rss_distance < object.relative_distance_with_delay_compensation;
      object.ignore = object.moving_time < p.common.filter.moving_time ||
                      object.velocity > p.common.vulnerable_road_user.max_velocity;
    } else {
      const auto & delay_object = p.common.vehicle.reaction_time;
      const auto & max_deceleration_object = p.common.vehicle.max_deceleration;
      const auto stop_distance_object =
        delay_object * object.velocity +
        0.5 * std::pow(object.velocity, 2.0) / std::abs(max_deceleration_object);
      const auto stop_distance_ego =
        0.5 * std::pow(current_velocity, 2.0) / std::abs(max_deceleration_ego);

      object.rss_distance = stop_distance_object - stop_distance_ego;
      object.safe = object.rss_distance < object.relative_distance_with_delay_compensation;
      object.ignore = object.moving_time < p.common.filter.moving_time ||
                      object.velocity > p.common.vehicle.max_velocity;
    }
  }
}

void RearCollisionChecker::fill_velocity(PointCloudObject & pointcloud_object)
{
  const auto p = param_listener_->get_params();

  const auto update_history = [this](const auto & pointcloud_object) {
    if (history_.count(pointcloud_object.furthest_lane.id()) == 0) {
      history_.emplace(pointcloud_object.furthest_lane.id(), pointcloud_object);
    } else {
      history_.at(pointcloud_object.furthest_lane.id()) = pointcloud_object;
    }
  };

  const auto fill_velocity = [&p, this](auto & pointcloud_object, const auto & previous_data) {
    const auto dx = previous_data.relative_distance - pointcloud_object.relative_distance;
    const auto dt = (pointcloud_object.last_update_time - previous_data.last_update_time).seconds();

    if (dt < 1e-6) {
      pointcloud_object.velocity = previous_data.velocity;
      pointcloud_object.tracking_duration = previous_data.tracking_duration;
      pointcloud_object.relative_distance_with_delay_compensation =
        pointcloud_object.relative_distance -
        pointcloud_object.velocity * p.common.pointcloud.latency;
      return;
    }

    const auto raw_velocity = dx / dt + context_->data->current_kinematics->twist.twist.linear.x;
    const auto is_reliable =
      previous_data.tracking_duration > p.common.pointcloud.velocity_estimation.observation_time;

    if (
      is_reliable && std::abs(raw_velocity - previous_data.velocity) / dt >
                       p.common.pointcloud.velocity_estimation.max_acceleration) {
      // closest point may jumped. don't use the data.
      pointcloud_object.velocity = previous_data.velocity;
      pointcloud_object.tracking_duration = previous_data.tracking_duration;
      pointcloud_object.detail = "the estimated velocity may be an outlier.";
    } else {
      // keep tracking.
      pointcloud_object.velocity =
        autoware::signal_processing::lowpassFilter(raw_velocity, previous_data.velocity, 0.5);
      pointcloud_object.tracking_duration = previous_data.tracking_duration + dt;
    }

    pointcloud_object.relative_distance_with_delay_compensation =
      pointcloud_object.relative_distance -
      pointcloud_object.velocity * p.common.pointcloud.latency;
  };

  const auto fill_moving_time = [&p, this](auto & pointcloud_object, const auto & previous_data) {
    const auto dt = (pointcloud_object.last_update_time - previous_data.last_update_time).seconds();

    if (pointcloud_object.velocity > p.common.filter.min_velocity) {
      pointcloud_object.moving_time = previous_data.moving_time + dt;
      pointcloud_object.last_stop_time = previous_data.last_stop_time;
    } else {
      pointcloud_object.moving_time = 0.0;
      pointcloud_object.last_stop_time = clock_->now();
    }
  };

  if (history_.count(pointcloud_object.furthest_lane.id()) == 0) {
    const auto previous_lanes =
      context_->data->route_handler->getPreviousLanelets(pointcloud_object.furthest_lane);
    for (const auto & previous_lane : previous_lanes) {
      if (history_.count(previous_lane.id()) != 0) {
        fill_velocity(pointcloud_object, history_.at(previous_lane.id()));
        fill_moving_time(pointcloud_object, history_.at(previous_lane.id()));
        update_history(pointcloud_object);
        return;
      }
    }

    update_history(pointcloud_object);
    return;
  }

  fill_velocity(pointcloud_object, history_.at(pointcloud_object.furthest_lane.id()));
  fill_moving_time(pointcloud_object, history_.at(pointcloud_object.furthest_lane.id()));
  update_history(pointcloud_object);
}

auto RearCollisionChecker::filter_pointcloud(DebugData & debug) const -> PointCloud::Ptr
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto p = param_listener_->get_params().common.pointcloud;

  auto output = std::make_shared<PointCloud>();

  if (context_->data->obstacle_pointcloud->data.empty()) {
    return output;
  }

  {
    pcl::fromROSMsg(*context_->data->obstacle_pointcloud, *output);
    debug.pointcloud_nums.push_back(output->size());
  }

  {
    autoware_utils::ScopedTimeTrack scoped_time_track("crop_x", *time_keeper_);

    pcl::PassThrough<pcl::PointXYZ> filter;
    filter.setInputCloud(output);
    filter.setFilterFieldName("x");
    filter.setFilterLimits(p.crop_box_filter.x.min, p.crop_box_filter.x.max);
    filter.filter(*output);
    debug.pointcloud_nums.push_back(output->size());
  }

  if (output->empty()) {
    return output;
  }

  {
    autoware_utils::ScopedTimeTrack scoped_time_track("crop_z", *time_keeper_);

    pcl::PassThrough<pcl::PointXYZ> filter;
    filter.setInputCloud(output);
    filter.setFilterFieldName("z");
    filter.setFilterLimits(
      p.crop_box_filter.z.min, context_->vehicle_info.vehicle_height_m + p.crop_box_filter.z.max);
    filter.filter(*output);
    debug.pointcloud_nums.push_back(output->size());
  }

  if (output->empty()) {
    return output;
  }

  {
    autoware_utils::ScopedTimeTrack scoped_time_track("transform", *time_keeper_);

    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      transform_stamped = context_->tf_buffer.lookupTransform(
        "map", context_->data->obstacle_pointcloud->header.frame_id,
        context_->data->obstacle_pointcloud->header.stamp, rclcpp::Duration::from_seconds(0.1));
    } catch (tf2::TransformException & e) {
      RCLCPP_WARN(logger_, "no transform found for no_ground_pointcloud: %s", e.what());
    }

    Eigen::Affine3f isometry = tf2::transformToEigen(transform_stamped.transform).cast<float>();
    autoware_utils::transform_pointcloud(*output, *output, isometry);
    debug.pointcloud_nums.push_back(output->size());
  }

  {
    autoware_utils::ScopedTimeTrack scoped_time_track("voxel_grid_filter", *time_keeper_);

    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(output);
    filter.setLeafSize(p.voxel_grid_filter.x, p.voxel_grid_filter.y, p.voxel_grid_filter.z);
    filter.filter(*output);
    debug.pointcloud_nums.push_back(output->size());
  }

  {
    const auto obstacle_pointcloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*output, *obstacle_pointcloud);
    obstacle_pointcloud->header.stamp = context_->data->obstacle_pointcloud->header.stamp;
    obstacle_pointcloud->header.frame_id = "map";
    debug.voxel_points = obstacle_pointcloud;
  }

  return output;
}

auto RearCollisionChecker::get_clustered_pointcloud(
  const PointCloud::Ptr in, DebugData & debug) const -> PointCloud::Ptr
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto parameter = param_listener_->get_params().common.pointcloud;
  const auto base_link_z = context_->data->current_kinematics->pose.pose.position.z;

  const auto points_belonging_to_cluster_hulls = std::make_shared<PointCloud>();
  // eliminate noisy points by only considering points belonging to clusters of at least a certain
  // size
  if (in->empty()) return std::make_shared<PointCloud>();
  const std::vector<pcl::PointIndices> cluster_indices = std::invoke([&]() {
    std::vector<pcl::PointIndices> cluster_idx;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(in);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(parameter.clustering.cluster_tolerance);
    ec.setMinClusterSize(parameter.clustering.min_cluster_size);
    ec.setMaxClusterSize(parameter.clustering.max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(in);
    ec.extract(cluster_idx);
    return cluster_idx;
  });
  for (const auto & indices : cluster_indices) {
    PointCloud::Ptr cluster(new PointCloud);
    bool cluster_surpasses_threshold_height{false};
    for (const auto & index : indices.indices) {
      const auto & point = (*in)[index];
      cluster_surpasses_threshold_height |=
        (point.z - base_link_z) > parameter.clustering.min_cluster_height;
      cluster->push_back(point);
    }
    if (!cluster_surpasses_threshold_height) continue;
    // Make a 2d convex hull for the objects
    pcl::ConvexHull<pcl::PointXYZ> hull;
    hull.setDimension(2);
    hull.setInputCloud(cluster);
    PointCloud::Ptr surface_hull(new PointCloud);
    hull.reconstruct(*surface_hull);
    autoware_utils::Polygon3d hull_polygon;
    for (const auto & p : *surface_hull) {
      points_belonging_to_cluster_hulls->push_back(p);
      const auto point = autoware_utils::Point3d{p.x, p.y, p.z};
      boost::geometry::append(hull_polygon.outer(), point);
    }
    debug.hull_polygons.push_back(hull_polygon);
  }

  {
    const auto obstacle_pointcloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*points_belonging_to_cluster_hulls, *obstacle_pointcloud);
    obstacle_pointcloud->header.stamp = context_->data->obstacle_pointcloud->header.stamp;
    obstacle_pointcloud->header.frame_id = "map";
    debug.cluster_points = obstacle_pointcloud;
  }

  return points_belonging_to_cluster_hulls;
}

auto RearCollisionChecker::get_pointcloud_object(
  const rclcpp::Time & now, const PointCloud::Ptr & pointcloud_ptr,
  const DetectionAreas & detection_areas, DebugData & debug) -> std::optional<PointCloudObject>
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  std::optional<PointCloudObject> opt_object = std::nullopt;
  for (const auto & [polygon, lanes] : detection_areas) {
    const auto pointcloud =
      *get_clustered_pointcloud(utils::get_obstacle_points({polygon}, *pointcloud_ptr), debug);

    const auto path = context_->data->route_handler->getCenterLinePath(
      lanes, 0.0, std::numeric_limits<double>::max());
    const auto resampled_path = autoware::motion_utils::resamplePath(path, 2.0);

    for (const auto & point : pointcloud) {
      const auto p_geom = autoware_utils::create_point(point.x, point.y, point.z);
      const size_t src_seg_idx =
        autoware::motion_utils::findNearestSegmentIndex(resampled_path.points, p_geom);
      const double signed_length_src_offset =
        autoware::motion_utils::calcLongitudinalOffsetToSegment(
          resampled_path.points, src_seg_idx, p_geom);

      const double obj_arc_length =
        autoware::motion_utils::calcSignedArcLength(
          resampled_path.points, src_seg_idx, resampled_path.points.size() - 1) -
        signed_length_src_offset;
      const auto pose_on_center_line = autoware::motion_utils::calcLongitudinalOffsetPose(
        resampled_path.points, src_seg_idx, signed_length_src_offset);

      if (!pose_on_center_line.has_value()) {
        continue;
      }

      if (!opt_object.has_value()) {
        PointCloudObject object;
        object.last_update_time = now;
        object.last_stop_time = now;
        object.pose = pose_on_center_line.value();
        object.furthest_lane = lanes.back();
        object.tracking_duration = 0.0;
        object.absolute_distance = obj_arc_length;
        object.velocity = 0.0;
        object.moving_time = 0.0;
        opt_object = object;
      } else if (opt_object.value().absolute_distance > obj_arc_length) {
        opt_object.value().last_update_time = now;
        opt_object.value().last_stop_time = now;
        opt_object.value().pose = pose_on_center_line.value();
        opt_object.value().furthest_lane = lanes.back();
        opt_object.value().tracking_duration = 0.0;
        opt_object.value().absolute_distance = obj_arc_length;
        opt_object.value().velocity = 0.0;
        opt_object.value().moving_time = 0.0;
      }
    }
  }

  return opt_object;
}

auto RearCollisionChecker::get_pointcloud_objects(
  const lanelet::ConstLanelets & current_lanes, const Behavior & shift_behavior,
  const Behavior & turn_behavior, DebugData & debug) -> PointCloudObjects
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto p = param_listener_->get_params();

  PointCloudObjects objects{};

  const auto obstacle_pointcloud = filter_pointcloud(debug);

  const auto max_deceleration_ego = p.common.ego.max_deceleration;
  const auto current_velocity = context_->data->current_kinematics->twist.twist.linear.x;

  {
    const auto delay_object = p.common.vulnerable_road_user.reaction_time;
    const auto max_deceleration_object = p.common.vulnerable_road_user.max_deceleration;

    const auto stop_distance_object = delay_object * p.common.vulnerable_road_user.max_velocity +
                                      0.5 *
                                        std::pow(p.common.vulnerable_road_user.max_velocity, 2.0) /
                                        std::abs(max_deceleration_object);
    const auto stop_distance_ego =
      0.5 * std::pow(current_velocity, 2.0) / std::abs(max_deceleration_ego);

    const auto forward_distance = context_->vehicle_info.max_longitudinal_offset_m;
    const auto backward_distance = p.common.pointcloud.range.buffer -
                                   context_->vehicle_info.min_longitudinal_offset_m +
                                   std::max(0.0, stop_distance_object - stop_distance_ego);

    const auto objects_at_blind_spot = get_pointcloud_objects_at_blind_spot(
      current_lanes, turn_behavior, forward_distance, backward_distance, obstacle_pointcloud,
      debug);
    objects.insert(objects.end(), objects_at_blind_spot.begin(), objects_at_blind_spot.end());
  }

  {
    const auto delay_object = p.common.vehicle.reaction_time;
    const auto max_deceleration_object = p.common.vehicle.max_deceleration;

    const auto stop_distance_object =
      delay_object * p.common.vehicle.max_velocity +
      0.5 * std::pow(p.common.vehicle.max_velocity, 2.0) / std::abs(max_deceleration_object);
    const auto stop_distance_ego =
      0.5 * std::pow(current_velocity, 2.0) / std::abs(max_deceleration_ego);

    const auto forward_distance = context_->vehicle_info.max_longitudinal_offset_m;
    const auto backward_distance = p.common.pointcloud.range.buffer -
                                   context_->vehicle_info.min_longitudinal_offset_m +
                                   std::max(0.0, stop_distance_object - stop_distance_ego);

    const auto objects_on_adjacent_lane = get_pointcloud_objects_on_adjacent_lane(
      current_lanes, shift_behavior, forward_distance, backward_distance, obstacle_pointcloud,
      debug);
    objects.insert(objects.end(), objects_on_adjacent_lane.begin(), objects_on_adjacent_lane.end());
  }

  return objects;
}

auto RearCollisionChecker::get_pointcloud_objects_on_adjacent_lane(
  const lanelet::ConstLanelets & current_lanes, const Behavior & shift_behavior,
  const double forward_distance, const double backward_distance,
  const PointCloud::Ptr & obstacle_pointcloud, DebugData & debug) -> PointCloudObjects
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto p = param_listener_->get_params();

  PointCloudObjects objects{};

  if (shift_behavior == Behavior::NONE) {
    return objects;
  }

  const auto ego_coordinate_on_arc =
    lanelet::utils::getArcCoordinates(current_lanes, context_->data->current_kinematics->pose.pose);

  lanelet::ConstLanelets connected_adjacent_lanes{};

  double length = 0.0;
  for (const auto & lane : current_lanes) {
    const auto current_lane_length = lanelet::utils::getLaneletLength2d(lane);

    length += current_lane_length;

    const auto ego_to_furthest_point = length - ego_coordinate_on_arc.length;
    const auto residual_distance = ego_to_furthest_point - forward_distance;

    const auto opt_adjacent_lane = [&lane, &shift_behavior, this]() {
      const auto is_right = shift_behavior == Behavior::SHIFT_RIGHT;
      return is_right ? context_->data->route_handler->getRightLanelet(lane, true, true)
                      : context_->data->route_handler->getLeftLanelet(lane, true, true);
    }();

    if (opt_adjacent_lane.has_value()) {
      connected_adjacent_lanes.push_back(opt_adjacent_lane.value());
    }

    if (!connected_adjacent_lanes.empty() && residual_distance > 0.0) {
      auto detection_areas = utils::get_previous_polygons_with_lane_recursively(
        current_lanes, connected_adjacent_lanes, residual_distance,
        residual_distance + forward_distance + backward_distance, context_->data->route_handler,
        p.common.adjacent_lane.offset.left, p.common.adjacent_lane.offset.right);

      utils::cut_by_lanelets(current_lanes, detection_areas);

      {
        debug.detection_areas.insert(
          debug.detection_areas.end(), detection_areas.begin(), detection_areas.end());
      }

      time_keeper_->start_track("get_pointcloud_object");
      auto opt_pointcloud_object = get_pointcloud_object(
        context_->data->obstacle_pointcloud->header.stamp, obstacle_pointcloud, detection_areas,
        debug);
      time_keeper_->end_track("get_pointcloud_object");

      if (!opt_pointcloud_object.has_value()) {
        return objects;
      }

      opt_pointcloud_object.value().is_vru = false;

      opt_pointcloud_object.value().relative_distance =
        opt_pointcloud_object.value().absolute_distance - ego_to_furthest_point -
        std::abs(context_->vehicle_info.min_longitudinal_offset_m);

      if (
        opt_pointcloud_object.value().relative_distance <
        p.common.pointcloud.range.dead_zone - context_->vehicle_info.max_longitudinal_offset_m) {
        return objects;
      }

      fill_velocity(opt_pointcloud_object.value());

      objects.push_back(opt_pointcloud_object.value());

      return objects;
    }

    if (!connected_adjacent_lanes.empty() && !opt_adjacent_lane.has_value()) {
      auto detection_areas = utils::get_previous_polygons_with_lane_recursively(
        current_lanes, connected_adjacent_lanes, 0.0,
        ego_to_furthest_point - current_lane_length + backward_distance,
        context_->data->route_handler, p.common.adjacent_lane.offset.left,
        p.common.adjacent_lane.offset.right);

      utils::cut_by_lanelets(current_lanes, detection_areas);

      {
        debug.detection_areas.insert(
          debug.detection_areas.end(), detection_areas.begin(), detection_areas.end());
      }

      connected_adjacent_lanes.clear();

      time_keeper_->start_track("get_pointcloud_object");
      auto opt_pointcloud_object = get_pointcloud_object(
        context_->data->obstacle_pointcloud->header.stamp, obstacle_pointcloud, detection_areas,
        debug);
      time_keeper_->end_track("get_pointcloud_object");

      if (!opt_pointcloud_object.has_value()) {
        continue;
      }

      opt_pointcloud_object.value().is_vru = false;

      opt_pointcloud_object.value().relative_distance =
        opt_pointcloud_object.value().absolute_distance - ego_to_furthest_point -
        std::abs(context_->vehicle_info.min_longitudinal_offset_m);

      if (
        opt_pointcloud_object.value().relative_distance <
        p.common.pointcloud.range.dead_zone - context_->vehicle_info.max_longitudinal_offset_m) {
        return objects;
      }

      fill_velocity(opt_pointcloud_object.value());

      objects.push_back(opt_pointcloud_object.value());
    }
  }

  return objects;
}

auto RearCollisionChecker::get_pointcloud_objects_at_blind_spot(
  const lanelet::ConstLanelets & current_lanes, const Behavior & turn_behavior,
  const double forward_distance, const double backward_distance,
  const PointCloud::Ptr & obstacle_pointcloud, DebugData & debug) -> PointCloudObjects
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto p = param_listener_->get_params();

  PointCloudObjects objects{};

  if (turn_behavior == Behavior::NONE) {
    return objects;
  }

  const auto half_lanes = [&current_lanes, &turn_behavior, &p, this]() {
    const auto is_right = turn_behavior == Behavior::TURN_RIGHT;
    lanelet::ConstLanelets ret{};
    for (const auto & lane : current_lanes) {
      ret.push_back(utils::generate_half_lanelet(
        lane, is_right,
        0.5 * context_->vehicle_info.vehicle_width_m + p.common.blind_spot.offset.inner,
        p.common.blind_spot.offset.outer));
    }
    return ret;
  }();
  const auto detection_polygon = utils::generate_detection_polygon(
    half_lanes, context_->data->current_kinematics->pose.pose, forward_distance, backward_distance);

  DetectionAreas detection_areas{};
  detection_areas.emplace_back(detection_polygon, half_lanes);

  {
    debug.detection_areas.insert(
      debug.detection_areas.end(), detection_areas.begin(), detection_areas.end());
  }

  time_keeper_->start_track("get_pointcloud_object");
  auto opt_pointcloud_object = get_pointcloud_object(
    context_->data->obstacle_pointcloud->header.stamp, obstacle_pointcloud, detection_areas, debug);
  time_keeper_->end_track("get_pointcloud_object");

  if (!opt_pointcloud_object.has_value()) {
    return objects;
  }

  opt_pointcloud_object.value().is_vru = true;

  const auto ego_coordinate_on_arc =
    lanelet::utils::getArcCoordinates(current_lanes, context_->data->current_kinematics->pose.pose);

  const auto ego_to_furthest_point =
    lanelet::utils::getLaneletLength2d(half_lanes) - ego_coordinate_on_arc.length;

  opt_pointcloud_object.value().relative_distance =
    opt_pointcloud_object.value().absolute_distance - ego_to_furthest_point -
    std::abs(context_->vehicle_info.min_longitudinal_offset_m);

  if (
    opt_pointcloud_object.value().relative_distance <
    p.common.pointcloud.range.dead_zone - context_->vehicle_info.max_longitudinal_offset_m) {
    return objects;
  }

  fill_velocity(opt_pointcloud_object.value());

  objects.push_back(opt_pointcloud_object.value());

  return objects;
}

bool RearCollisionChecker::is_safe(const PointCloudObjects & objects, DebugData & debug) const
{
  autoware_utils::ScopedTimeTrack st("is_safe_pointcloud", *time_keeper_);

  const auto p = param_listener_->get_params();

  {
    debug.pointcloud_objects = objects;
  }

  for (const auto & object : objects) {
    if (object.tracking_duration < p.common.pointcloud.velocity_estimation.observation_time) {
      continue;
    }

    if (object.ignore) {
      continue;
    }

    if (object.safe) {
      continue;
    }

    return false;
  }

  return true;
}

bool RearCollisionChecker::is_safe(DebugData & debug)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto p = param_listener_->get_params();
  const auto now = clock_->now();

  {
    const auto obstacle_pointcloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
    obstacle_pointcloud->header.stamp = context_->data->obstacle_pointcloud->header.stamp;
    obstacle_pointcloud->header.frame_id = "map";
    debug.cluster_points = obstacle_pointcloud;
  }

  constexpr double forward = 100.0;
  constexpr double backward = 100.0;
  const auto current_lanes = utils::get_current_lanes(context_, forward, backward);
  const auto combine_lanelet = lanelet::utils::combineLaneletsShape(current_lanes);

  if (current_lanes.empty()) {
    debug.text = "failed to identify the current driving lane.";
    return true;
  }

  const auto is_unsafe_holding = (now - last_unsafe_time_).seconds() < p.common.off_time_buffer;
  const auto turn_behavior =
    utils::check_turn_behavior(current_lanes, is_unsafe_holding, context_, p, debug);
  const auto shift_behavior =
    utils::check_shift_behavior(current_lanes, is_unsafe_holding, context_, p, debug);

  {
    debug.current_lanes = current_lanes;
    debug.turn_behavior = turn_behavior;
    debug.shift_behavior = shift_behavior;
  }

  if (turn_behavior == Behavior::NONE && shift_behavior == Behavior::NONE) {
    return true;
  }

  {
    debug.is_active = true;
  }

  PredictedObjects objects_on_target_lane;
  PointCloudObjects pointcloud_objects{};

  {
    time_keeper_->start_track("pointcloud_base");
    pointcloud_objects =
      get_pointcloud_objects(current_lanes, shift_behavior, turn_behavior, debug);
    fill_rss_distance(pointcloud_objects);
    time_keeper_->end_track("pointcloud_base");
  }

  {
    if (is_safe(pointcloud_objects, debug)) {
      if ((now - last_unsafe_time_).seconds() > p.common.off_time_buffer) {
        last_safe_time_ = now;
        return true;
      }
    } else if ((now - last_safe_time_).seconds() < p.common.on_time_buffer) {
      RCLCPP_WARN(logger_, "[RCC] Momentary collision risk detected.");
      return true;
    } else {
      last_unsafe_time_ = now;
    }

    {
      RCLCPP_ERROR(logger_, "[RCC] Continuous collision risk detected.");
      debug.text = "continuous collision risk detected.";
    }
  }

  return false;
}

void RearCollisionChecker::post_process()
{
  auto itr = history_.begin();
  while (itr != history_.end()) {
    if ((clock_->now() - itr->second.last_update_time).seconds() > 1.0) {
      itr = history_.erase(itr);
    } else {
      itr++;
    }
  }
}

void RearCollisionChecker::publish_marker(const DebugData & debug) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  MarkerArray msg;

  const auto add = [&msg](const MarkerArray & added) {
    autoware_utils::append_marker_array(added, &msg);
  };

  {
    add(utils::create_line_marker_array(
      debug.reachable_line, "reachable_line",
      autoware_utils::create_marker_color(1.0, 0.67, 0.0, 0.999)));
    add(utils::create_line_marker_array(
      debug.stoppable_line, "stoppable_line",
      autoware_utils::create_marker_color(1.0, 0.0, 0.42, 0.999)));
    add(lanelet::visualization::laneletsAsTriangleMarkerArray(
      "detection_lanes", debug.get_detection_lanes(),
      autoware_utils::create_marker_color(1.0, 0.0, 0.42, 0.2)));
    add(lanelet::visualization::laneletsAsTriangleMarkerArray(
      "current_lanes", debug.current_lanes,
      autoware_utils::create_marker_color(0.16, 1.0, 0.69, 0.2)));
    add(utils::create_pointcloud_object_marker_array(
      debug.pointcloud_objects, "pointcloud_objects", param_listener_->get_params()));
    add(utils::create_polygon_marker_array(
      debug.get_detection_polygons(), "detection_areas",
      autoware_utils::create_marker_color(1.0, 0.0, 0.42, 0.999)));
    add(utils::create_polygon_marker_array(
      debug.hull_polygons, "hull_polygons",
      autoware_utils::create_marker_color(0.0, 0.0, 1.0, 0.999)));

    std::for_each(msg.markers.begin(), msg.markers.end(), [](auto & marker) {
      marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    });

    context_->debug_pose_publisher->pushMarkers(msg);
  }

  if (debug.cluster_points) {
    pub_cluster_pointcloud_->publish(*debug.cluster_points);
  }

  if (debug.voxel_points) {
    pub_voxel_pointcloud_->publish(*debug.voxel_points);
  }

  {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2) << std::boolalpha;
    ss << "ACTIVE:" << debug.is_active << "\n";
    ss << "SAFE:" << debug.is_safe << "\n";
    ss << "TURN:" << magic_enum::enum_name(debug.turn_behavior) << "\n";
    ss << "SHIFT:" << magic_enum::enum_name(debug.shift_behavior) << "\n";
    ss << "INFO:" << debug.text << "\n";
    ss << "TRACKING OBJECTS:" << debug.pointcloud_objects.size() << "\n";
    ss << "PC NUM:";
    for (const auto num : debug.pointcloud_nums) {
      ss << num << "->";
    }
    ss << "\n";
    ss << "PROCESSING TIME:" << debug.processing_time_detail_ms << "[ms]\n";

    StringStamped string_stamp;
    string_stamp.stamp = clock_->now();
    string_stamp.data = ss.str();
    pub_string_->publish(string_stamp);
  }
}

void RearCollisionChecker::publish_planning_factor(const DebugData & debug) const
{
  if (debug.is_safe) return;

  SafetyFactorArray factor_array;
  factor_array.is_safe = false;
  factor_array.detail = "possible collision with rear object";
  factor_array.header.stamp = clock_->now();

  SafetyFactor factor;
  factor.type = SafetyFactor::POINTCLOUD;
  for (const auto & obj : debug.pointcloud_objects) {
    factor.is_safe = obj.safe;
    factor.points.push_back(obj.pose.position);
    factor_array.factors.push_back(factor);
  }

  const auto & traj_points = context_->data->current_trajectory->points;
  const auto & ego_pose = context_->data->current_kinematics->pose.pose;
  planning_factor_interface_->add(
    traj_points, ego_pose, ego_pose, PlanningFactor::STOP, factor_array);
  planning_factor_interface_->publish();
}

}  // namespace autoware::planning_validator

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::planning_validator::RearCollisionChecker, autoware::planning_validator::PluginInterface)
