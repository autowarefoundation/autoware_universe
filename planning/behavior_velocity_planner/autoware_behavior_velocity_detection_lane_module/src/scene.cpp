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

#include "scene.hpp"

#include "autoware/signal_processing/lowpass_filter_1d.hpp"
#include "utils.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/arc_lane_util.hpp>
#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_lanelet2_extension/visualization/visualization.hpp>

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
#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner
{
using autoware::motion_utils::calcLongitudinalOffsetPose;
using autoware::motion_utils::calcSignedArcLength;

DetectionLaneModule::DetectionLaneModule(
  const int64_t module_id, const lanelet::ConstLanelet & passing_lane,
  const lanelet::ConstLineString3d & stop_line, const lanelet::ConstLanelets & detection_lanes,
  const double ttc_threshold, const detection_lane::Params::DetectionLane & planner_param,
  const rclcpp::Logger & logger, const rclcpp::Clock::SharedPtr clock,
  const std::shared_ptr<autoware_utils::TimeKeeper> time_keeper,
  const std::shared_ptr<planning_factor_interface::PlanningFactorInterface>
    planning_factor_interface)
: SceneModuleInterfaceWithRTC(module_id, logger, clock, time_keeper, planning_factor_interface),
  turn_direction_{passing_lane.attributeOr("turn_direction", "else")},
  stop_line_{stop_line},
  ttc_threshold_{ttc_threshold},
  planner_param_(planner_param)
{
  for (const auto & detection_lane : detection_lanes) {
    std::vector<autoware_utils::Point2d> intersects;
    boost::geometry::intersection(
      lanelet::utils::to2D(detection_lane.centerline()).basicLineString(),
      lanelet::utils::to2D(passing_lane.centerline()).basicLineString(), intersects);

    if (intersects.size() != 1) {
      RCLCPP_ERROR(logger, "something wrong.");
    }

    geometry_msgs::msg::Pose conflict_point;
    conflict_point.position =
      autoware_utils::create_point(intersects.front().x(), intersects.front().y(), 0.0);

    lanelet::ConstLanelets lanelets{detection_lane};

    const auto conflict_point_coordinate =
      lanelet::utils::getArcCoordinates(lanelets, conflict_point).length;

    const auto expand_lanelets = lanelet::utils::getExpandedLanelets(
      lanelets, planner_param_.pointcloud.offset.left,
      -1.0 * planner_param_.pointcloud.offset.right);

    const auto polygon =
      lanelet::utils::getPolygonFromArcLength(
        expand_lanelets, conflict_point_coordinate - planner_param_.pointcloud.range.max,
        conflict_point_coordinate - planner_param_.pointcloud.range.min)
        .basicPolygon();

    detection_areas_info_.emplace_back(lanelets, polygon, conflict_point_coordinate);
  }

  const auto traffic_light_regulatory_elements =
    passing_lane.regulatoryElementsAs<lanelet::autoware::AutowareTrafficLight>();
  if (!traffic_light_regulatory_elements.empty()) {
    traffic_light_regulatory_element_id_ = traffic_light_regulatory_elements.front()->id();
  }
}

bool DetectionLaneModule::modifyPathVelocity([[maybe_unused]] PathWithLaneId * path)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  debug_data_ = DebugData();

  post_process();

  const auto original_path = *path;

  const auto obstacle_pointcloud = filter_pointcloud(debug_data_);
  auto pointcloud_objects = get_pointcloud_object(
    planner_data_->current_odometry->header.stamp, obstacle_pointcloud, detection_areas_info_,
    debug_data_);

  fill_ttc(pointcloud_objects);

  debug_data_.pointcloud_objects = pointcloud_objects;

  // Get self pose
  const auto & self_pose = planner_data_->current_odometry->pose;
  const size_t current_seg_idx = findEgoSegmentIndex(path->points);

  // Get stop point
  const auto extended_stop_line = planning_utils::extendLine(
    stop_line_[0], stop_line_[1], planner_data_->stop_line_extend_length);
  const auto stop_point = arc_lane_utils::createTargetPoint(
    original_path, extended_stop_line, planner_param_.stop.margin,
    planner_data_->vehicle_info_.max_longitudinal_offset_m);
  if (!stop_point) {
    return true;
  }

  const auto & stop_point_idx = stop_point->first;
  const auto & stop_pose = stop_point->second;
  const size_t stop_line_seg_idx = planning_utils::calcSegmentIndexFromPointIndex(
    path->points, stop_pose.position, stop_point_idx);

  auto modified_stop_pose = stop_pose;
  size_t modified_stop_line_seg_idx = stop_line_seg_idx;

  const auto is_stopped = planner_data_->isVehicleStopped(0.0);
  const auto stop_dist = calcSignedArcLength(
    path->points, self_pose.position, current_seg_idx, stop_pose.position, stop_line_seg_idx);

  // Don't re-approach when the ego stops closer to the stop point than hold_stop_margin_distance
  if (is_stopped && stop_dist < planner_param_.hold.margin) {
    const auto ego_pos_on_path =
      calcLongitudinalOffsetPose(original_path.points, self_pose.position, 0.0);

    if (!ego_pos_on_path) {
      return false;
    }

    modified_stop_pose = ego_pos_on_path.value();
    modified_stop_line_seg_idx = current_seg_idx;
  }

  setDistance(stop_dist);

  // Check state
  setSafe(is_safe(pointcloud_objects) || is_prioritized_lane());

  if (isActivated()) {
    return true;
  }

  // Force ignore objects after dead_line
  if (planner_param_.dead_line.enable) {
    // Use '-' for margin because it's the backward distance from stop line
    const auto dead_line_point = arc_lane_utils::createTargetPoint(
      original_path, extended_stop_line, -planner_param_.dead_line.margin,
      planner_data_->vehicle_info_.max_longitudinal_offset_m);

    if (dead_line_point) {
      const size_t dead_line_point_idx = dead_line_point->first;
      const auto & dead_line_pose = dead_line_point->second;

      const size_t dead_line_seg_idx = planning_utils::calcSegmentIndexFromPointIndex(
        path->points, dead_line_pose.position, dead_line_point_idx);

      debug_data_.dead_line_poses.push_back(dead_line_pose);

      const double dist_from_ego_to_dead_line = calcSignedArcLength(
        original_path.points, self_pose.position, current_seg_idx, dead_line_pose.position,
        dead_line_seg_idx);
      if (dist_from_ego_to_dead_line < 0.0) {
        RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000, "exceed dead line.");
        setSafe(true);
        return true;
      }
    }
  }

  // Ignore objects if braking distance is not enough
  if (!planner_param_.stop.use_sudden_stop) {
    const auto v_now = planner_data_->current_velocity->twist.linear.x;
    const auto a_now = planner_data_->current_acceleration->accel.accel.linear.x;
    const double pass_judge_line_distance = planning_utils::calcJudgeLineDistWithAccLimit(
      v_now, a_now, planner_data_->delay_response_time);
    constexpr double buffer = 3.0;
    if (stop_dist + buffer < pass_judge_line_distance) {
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000, "avoid sudden stop.");
      setSafe(true);
      return true;
    }
  }

  planning_utils::insertStopPoint(modified_stop_pose.position, modified_stop_line_seg_idx, *path);

  debug_data_.stop_poses.push_back(stop_point->second);

  return true;
}

auto DetectionLaneModule::filter_pointcloud([[maybe_unused]] DebugData & debug) const
  -> PointCloud::Ptr
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto p = planner_param_.pointcloud;

  auto output = std::make_shared<PointCloud>(*planner_data_->no_ground_pointcloud);

  {
    debug.pointcloud_nums.push_back(output->size());
  }

  {
    output =
      filter_by_range(output, planner_data_->current_odometry->pose.position, p.range.min, true);
    debug.pointcloud_nums.push_back(output->size());
  }

  // {
  //   autoware_utils::ScopedTimeTrack st("voxel_grid_filter", *time_keeper_);

  //   pcl::VoxelGrid<pcl::PointXYZ> filter;
  //   filter.setInputCloud(output);
  //   filter.setLeafSize(p.voxel_grid_filter.x, p.voxel_grid_filter.y, p.voxel_grid_filter.z);
  //   filter.filter(*output);
  //   debug.pointcloud_nums.push_back(output->size());
  // }

  return output;
}

auto DetectionLaneModule::get_pointcloud_object(
  const rclcpp::Time & now, const PointCloud::Ptr & pointcloud_ptr,
  const DetectionAreasInfo & detection_areas_info, DebugData & debug) -> PointCloudObjects
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  PointCloudObjects ret{};
  for (const auto & info : detection_areas_info) {
    const auto pointcloud =
      *get_clustered_pointcloud(get_obstacle_points({info.polygon}, *pointcloud_ptr), debug);

    const auto path = planner_data_->route_handler_->getCenterLinePath(
      info.lanelets, 0.0, std::numeric_limits<double>::max());
    // const auto resampled_path = behavior_path_planner::utils::resamplePathWithSpline(path, 2.0);
    const auto resampled_path = path;

    std::optional<PointCloudObject> opt_object = std::nullopt;
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
        object.pose = pose_on_center_line.value();
        object.furthest_lane = info.lanelets.back();
        object.tracking_duration = 0.0;
        object.distance = obj_arc_length;
        object.velocity = 0.0;
        opt_object = object;
      } else if (opt_object.value().distance > obj_arc_length) {
        opt_object.value().last_update_time = now;
        opt_object.value().pose = pose_on_center_line.value();
        opt_object.value().furthest_lane = info.lanelets.back();
        opt_object.value().tracking_duration = 0.0;
        opt_object.value().distance = obj_arc_length;
        opt_object.value().velocity = 0.0;
      }
    }
    if (opt_object.has_value()) {
      opt_object.value().distance -=
        lanelet::utils::getLaneletLength2d(info.lanelets) - info.conflict_point_coordinate;
      fill_velocity(opt_object.value());
      ret.push_back(opt_object.value());
    }
  }

  return ret;
}

auto DetectionLaneModule::get_clustered_pointcloud(
  const PointCloud::Ptr in, DebugData & debug) const -> PointCloud::Ptr
{
  const auto p = planner_param_.pointcloud;

  const auto points_belonging_to_cluster_hulls = std::make_shared<PointCloud>();
  // eliminate noisy points by only considering points belonging to clusters of at least a certain
  // size
  if (in->empty()) return std::make_shared<PointCloud>();
  const std::vector<pcl::PointIndices> cluster_indices = std::invoke([&]() {
    std::vector<pcl::PointIndices> cluster_idx;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(in);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(p.clustering.cluster_tolerance);
    ec.setMinClusterSize(p.clustering.min_cluster_size);
    ec.setMaxClusterSize(p.clustering.max_cluster_size);
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
      cluster_surpasses_threshold_height = (cluster_surpasses_threshold_height)
                                             ? cluster_surpasses_threshold_height
                                             : (point.z > p.clustering.min_cluster_height);
      cluster->push_back(point);
    }
    if (!cluster_surpasses_threshold_height) continue;
    // Make a 2d convex hull for the objects
    pcl::ConvexHull<pcl::PointXYZ> hull;
    hull.setDimension(2);
    hull.setInputCloud(cluster);
    std::vector<pcl::Vertices> polygons;
    PointCloud::Ptr surface_hull(new PointCloud);
    hull.reconstruct(*surface_hull, polygons);
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
    obstacle_pointcloud->header.stamp = planner_data_->current_odometry->header.stamp;
    obstacle_pointcloud->header.frame_id = "map";
    debug.obstacle_pointcloud = obstacle_pointcloud;
  }

  return points_belonging_to_cluster_hulls;
}

void DetectionLaneModule::fill_ttc(PointCloudObjects & objects) const
{
  for (auto & object : objects) {
    object.ttc = object.distance / std::max(planner_param_.filter.min_velocity, object.velocity);
    object.safe = object.ttc > ttc_threshold_;
    object.ignore = object.velocity < planner_param_.filter.min_velocity;
  }
}

void DetectionLaneModule::fill_velocity(PointCloudObject & pointcloud_object)
{
  const auto update_history = [this](const auto & pointcloud_object) {
    if (history_.count(pointcloud_object.furthest_lane.id()) == 0) {
      history_.emplace(pointcloud_object.furthest_lane.id(), pointcloud_object);
    } else {
      history_.at(pointcloud_object.furthest_lane.id()) = pointcloud_object;
    }
  };

  const auto fill_velocity = [this](auto & pointcloud_object, const auto & previous_data) {
    const auto dx = previous_data.distance - pointcloud_object.distance;
    const auto dt = (pointcloud_object.last_update_time - previous_data.last_update_time).seconds();

    if (dt < 1e-6) {
      pointcloud_object.velocity = previous_data.velocity;
      pointcloud_object.tracking_duration = previous_data.tracking_duration + dt;
      pointcloud_object.distance_with_delay_compensation =
        pointcloud_object.distance - pointcloud_object.velocity * planner_param_.pointcloud.latency;
      return;
    }

    constexpr double assumed_acceleration = 30.0;
    const auto raw_velocity = dx / dt;
    const auto is_reliable = previous_data.tracking_duration >
                             planner_param_.pointcloud.velocity_estimation.observation_time;

    if (
      is_reliable && std::abs(raw_velocity - previous_data.velocity) / dt > assumed_acceleration) {
      // closest point may jumped. reset tracking history.
      pointcloud_object.velocity = 0.0;
      pointcloud_object.tracking_duration = 0.0;
    } else {
      // keep tracking.
      pointcloud_object.velocity =
        autoware::signal_processing::lowpassFilter(raw_velocity, previous_data.velocity, 0.5);
      pointcloud_object.tracking_duration = previous_data.tracking_duration + dt;
    }

    pointcloud_object.distance_with_delay_compensation =
      pointcloud_object.distance - pointcloud_object.velocity * planner_param_.pointcloud.latency;
  };

  if (history_.count(pointcloud_object.furthest_lane.id()) == 0) {
    update_history(pointcloud_object);
    return;
  }

  fill_velocity(pointcloud_object, history_.at(pointcloud_object.furthest_lane.id()));
  update_history(pointcloud_object);
}

bool DetectionLaneModule::is_prioritized_lane() const
{
  const auto signal = planner_data_->getTrafficSignal(
    traffic_light_regulatory_element_id_, true /* this module keeps last observation*/);

  if (!signal.has_value()) {
    return false;
  }

  constexpr double threshold = 1.0;
  if ((clock_->now() - signal.value().stamp).seconds() > threshold) {
    return false;
  }

  if (turn_direction_ == "right") {
    return is_green_arrow(
      signal.value().signal, autoware_perception_msgs::msg::TrafficLightElement::RIGHT_ARROW);
  }

  if (turn_direction_ == "left") {
    return is_green_arrow(
      signal.value().signal, autoware_perception_msgs::msg::TrafficLightElement::LEFT_ARROW);
  }

  return false;
}

bool DetectionLaneModule::is_safe(const PointCloudObjects & pointcloud_objects)
{
  const auto is_safe = std::all_of(
    pointcloud_objects.begin(), pointcloud_objects.end(),
    [](const auto & object) { return object.ignore || object.safe; });

  const auto now = clock_->now();
  if (is_safe) {
    last_safe_time_ = now;
    if ((now - last_unsafe_time_).seconds() > planner_param_.stop.off_time_buffer) {
      return true;
    }
  } else {
    last_unsafe_time_ = now;
    if ((now - last_safe_time_).seconds() < planner_param_.stop.on_time_buffer) {
      return true;
    }
  }

  return false;
}

auto DetectionLaneModule::createDebugMarkerArray() -> visualization_msgs::msg::MarkerArray
{
  visualization_msgs::msg::MarkerArray msg;

  const auto add = [&msg](const visualization_msgs::msg::MarkerArray & added) {
    autoware_utils::append_marker_array(added, &msg);
  };

  add(lanelet::visualization::laneletsAsTriangleMarkerArray(
    "detection_lanes", get_detection_lanes(),
    autoware_utils::create_marker_color(1.0, 0.0, 0.42, 0.1)));

  add(create_polygon_marker_array(
    get_detection_polygons(), "detection_areas",
    autoware_utils::create_marker_color(1.0, 0.0, 0.42, 0.999)));

  add(create_polygon_marker_array(
    debug_data_.hull_polygons, "hull_polygons",
    autoware_utils::create_marker_color(0.0, 0.0, 1.0, 0.999)));

  add(create_pointcloud_object_marker_array(debug_data_.pointcloud_objects, "pointcloud_objects"));

  std::for_each(msg.markers.begin(), msg.markers.end(), [](auto & marker) {
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
  });

  return msg;
}

auto DetectionLaneModule::createVirtualWalls() -> autoware::motion_utils::VirtualWalls
{
  autoware::motion_utils::VirtualWalls virtual_walls;
  autoware::motion_utils::VirtualWall wall;
  wall.text = "detection_lane";

  wall.style = autoware::motion_utils::VirtualWallType::stop;
  for (const auto & p : debug_data_.stop_poses) {
    wall.pose = autoware_utils::calc_offset_pose(
      p, planner_data_->vehicle_info_.max_longitudinal_offset_m, 0.0, 0.0);
    virtual_walls.push_back(wall);
  }

  wall.style = autoware::motion_utils::VirtualWallType::deadline;
  for (const auto & p : debug_data_.dead_line_poses) {
    wall.pose = autoware_utils::calc_offset_pose(
      p, planner_data_->vehicle_info_.max_longitudinal_offset_m, 0.0, 0.0);
    virtual_walls.push_back(wall);
  }
  return virtual_walls;
}
}  // namespace autoware::behavior_velocity_planner
