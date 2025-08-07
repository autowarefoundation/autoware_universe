// Copyright 2024 TIER IV, Inc. All rights reserved.
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

#include "out_of_lane_module.hpp"

#include "calculate_slowdown_points.hpp"
#include "debug.hpp"
#include "filter_predicted_objects.hpp"
#include "footprint.hpp"
#include "lanelets_selection.hpp"
#include "out_of_lane_collisions.hpp"
#include "types.hpp"

#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/motion_velocity_planner_common/planner_data.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <autoware/traffic_light_utils/traffic_light_utils.hpp>
#include <autoware_utils/geometry/boost_geometry.hpp>
#include <autoware_utils/ros/parameter.hpp>
#include <autoware_utils/ros/published_time_publisher.hpp>
#include <autoware_utils/ros/update_param.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>
#include <rclcpp/logging.hpp>

#include <autoware_internal_planning_msgs/msg/safety_factor.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <unique_identifier_msgs/msg/detail/uuid__struct.hpp>

#include <boost/geometry/algorithms/envelope.hpp>
#include <boost/geometry/algorithms/intersects.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>
#include <boost/uuid/uuid.hpp>

#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include <algorithm>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::motion_velocity_planner
{

using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

void OutOfLaneModule::init(rclcpp::Node & node, const std::string & module_name)
{
  module_name_ = module_name;
  logger_ = node.get_logger().get_child(module_name_);
  clock_ = node.get_clock();
  init_parameters(node);

  planning_factor_interface_ =
    std::make_unique<autoware::planning_factor_interface::PlanningFactorInterface>(
      &node, "out_of_lane");

  debug_publisher_ =
    node.create_publisher<visualization_msgs::msg::MarkerArray>("~/" + ns_ + "/debug_markers", 1);
  virtual_wall_publisher_ =
    node.create_publisher<visualization_msgs::msg::MarkerArray>("~/" + ns_ + "/virtual_walls", 1);
  processing_diag_publisher_ = std::make_shared<autoware_utils::ProcessingTimePublisher>(
    &node, "~/debug/" + ns_ + "/processing_time_ms_diag");
  processing_time_publisher_ =
    node.create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "~/debug/" + ns_ + "/processing_time_ms", 1);
}
void OutOfLaneModule::init_parameters(rclcpp::Node & node)
{
  using autoware_utils::get_or_declare_parameter;
  auto & pp = params_;

  pp.mode = get_or_declare_parameter<std::string>(node, ns_ + ".mode");
  pp.skip_if_already_overlapping =
    get_or_declare_parameter<bool>(node, ns_ + ".skip_if_already_overlapping");
  pp.max_arc_length = get_or_declare_parameter<double>(node, ns_ + ".max_arc_length");

  pp.time_threshold = get_or_declare_parameter<double>(node, ns_ + ".threshold.time_threshold");
  pp.ttc_threshold = get_or_declare_parameter<double>(node, ns_ + ".ttc.threshold");
  pp.ttc_release_threshold = get_or_declare_parameter<double>(node, ns_ + ".ttc.release_threshold");

  pp.objects_min_vel = get_or_declare_parameter<double>(node, ns_ + ".objects.minimum_velocity");
  pp.objects_min_confidence =
    get_or_declare_parameter<double>(node, ns_ + ".objects.predicted_path_min_confidence");
  pp.objects_cut_predicted_paths_beyond_red_lights =
    get_or_declare_parameter<bool>(node, ns_ + ".objects.cut_predicted_paths_beyond_red_lights");
  pp.objects_ignore_behind_ego =
    get_or_declare_parameter<bool>(node, ns_ + ".objects.ignore_behind_ego");
  pp.validate_predicted_paths_on_lanelets =
    get_or_declare_parameter<bool>(node, ns_ + ".objects.validate_predicted_paths_on_lanelets");
  pp.objects_extra_width = get_or_declare_parameter<double>(node, ns_ + ".objects.extra_width");

  pp.precision = get_or_declare_parameter<double>(node, ns_ + ".action.precision");
  pp.use_map_stop_lines = get_or_declare_parameter<bool>(node, ns_ + ".action.use_map_stop_lines");
  pp.min_on_duration = get_or_declare_parameter<double>(node, ns_ + ".action.min_on_duration");
  pp.min_off_duration = get_or_declare_parameter<double>(node, ns_ + ".action.min_off_duration");
  pp.update_distance_th =
    get_or_declare_parameter<double>(node, ns_ + ".action.update_distance_th");
  pp.lon_dist_buffer =
    get_or_declare_parameter<double>(node, ns_ + ".action.longitudinal_distance_buffer");
  pp.lat_dist_buffer =
    get_or_declare_parameter<double>(node, ns_ + ".action.lateral_distance_buffer");
  pp.slow_velocity = get_or_declare_parameter<double>(node, ns_ + ".action.slowdown.velocity");
  pp.stop_dist_threshold =
    get_or_declare_parameter<double>(node, ns_ + ".action.stop.distance_threshold");

  pp.extra_front_offset = get_or_declare_parameter<double>(node, ns_ + ".ego.extra_front_offset");
  pp.extra_rear_offset = get_or_declare_parameter<double>(node, ns_ + ".ego.extra_rear_offset");
  pp.extra_left_offset = get_or_declare_parameter<double>(node, ns_ + ".ego.extra_left_offset");
  pp.extra_right_offset = get_or_declare_parameter<double>(node, ns_ + ".ego.extra_right_offset");
  const auto vehicle_info = vehicle_info_utils::VehicleInfoUtils(node).getVehicleInfo();
  pp.front_offset = vehicle_info.max_longitudinal_offset_m;
  pp.rear_offset = vehicle_info.min_longitudinal_offset_m;
  pp.left_offset = vehicle_info.max_lateral_offset_m;
  pp.right_offset = vehicle_info.min_lateral_offset_m;
}

void OutOfLaneModule::update_parameters(const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils::update_param;
  auto & pp = params_;
  update_param(parameters, ns_ + ".mode", pp.mode);
  update_param(parameters, ns_ + ".skip_if_already_overlapping", pp.skip_if_already_overlapping);
  update_param(parameters, ns_ + ".max_arc_length", pp.max_arc_length);

  update_param(parameters, ns_ + ".threshold.time_threshold", pp.time_threshold);
  update_param(parameters, ns_ + ".ttc.threshold", pp.ttc_threshold);
  update_param(parameters, ns_ + ".ttc.release_threshold", pp.ttc_release_threshold);

  update_param(parameters, ns_ + ".objects.minimum_velocity", pp.objects_min_vel);
  update_param(
    parameters, ns_ + ".objects.predicted_path_min_confidence", pp.objects_min_confidence);
  update_param(
    parameters, ns_ + ".objects.cut_predicted_paths_beyond_red_lights",
    pp.objects_cut_predicted_paths_beyond_red_lights);
  update_param(parameters, ns_ + ".objects.ignore_behind_ego", pp.objects_ignore_behind_ego);
  update_param(
    parameters, ns_ + ".objects.validate_predicted_paths_on_lanelets",
    pp.validate_predicted_paths_on_lanelets);
  update_param(parameters, ns_ + ".objects.extra_width", pp.objects_extra_width);

  update_param(parameters, ns_ + ".action.precision", pp.precision);
  update_param(parameters, ns_ + ".action.use_map_stop_lines", pp.use_map_stop_lines);
  update_param(parameters, ns_ + ".action.min_on_duration", pp.min_on_duration);
  update_param(parameters, ns_ + ".action.min_off_duration", pp.min_off_duration);
  update_param(parameters, ns_ + ".action.longitudinal_distance_buffer", pp.lon_dist_buffer);
  update_param(parameters, ns_ + ".action.lateral_distance_buffer", pp.lat_dist_buffer);
  update_param(parameters, ns_ + ".action.slowdown.velocity", pp.slow_velocity);
  update_param(parameters, ns_ + ".action.stop.distance_threshold", pp.stop_dist_threshold);

  update_param(parameters, ns_ + ".ego.extra_front_offset", pp.extra_front_offset);
  update_param(parameters, ns_ + ".ego.extra_rear_offset", pp.extra_rear_offset);
  update_param(parameters, ns_ + ".ego.extra_left_offset", pp.extra_left_offset);
  update_param(parameters, ns_ + ".ego.extra_right_offset", pp.extra_right_offset);
}

void OutOfLaneModule::limit_trajectory_size(
  out_of_lane::EgoData & ego_data,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & smoothed_trajectory_points,
  const double max_arc_length)
{
  ego_data.first_trajectory_idx =
    motion_utils::findNearestSegmentIndex(smoothed_trajectory_points, ego_data.pose.position);
  ego_data.longitudinal_offset_to_first_trajectory_index =
    motion_utils::calcLongitudinalOffsetToSegment(
      smoothed_trajectory_points, ego_data.first_trajectory_idx, ego_data.pose.position);
  auto l = -ego_data.longitudinal_offset_to_first_trajectory_index;
  ego_data.trajectory_points.push_back(smoothed_trajectory_points[ego_data.first_trajectory_idx]);
  for (auto i = ego_data.first_trajectory_idx + 1; i < smoothed_trajectory_points.size(); ++i) {
    l += autoware_utils::calc_distance2d(
      smoothed_trajectory_points[i - 1], smoothed_trajectory_points[i]);
    if (l >= max_arc_length) {
      break;
    }
    ego_data.trajectory_points.push_back(smoothed_trajectory_points[i]);
  }
}

void prepare_stop_lines_rtree(
  out_of_lane::EgoData & ego_data, const PlannerData & planner_data, const double search_distance)
{
  std::vector<out_of_lane::StopLineNode> rtree_nodes;
  const auto bbox = lanelet::BoundingBox2d(
    lanelet::BasicPoint2d{
      ego_data.pose.position.x - search_distance, ego_data.pose.position.y - search_distance},
    lanelet::BasicPoint2d{
      ego_data.pose.position.x + search_distance, ego_data.pose.position.y + search_distance});
  out_of_lane::StopLineNode stop_line_node;
  for (const auto & ll :
       planner_data.route_handler->getLaneletMapPtr()->laneletLayer.search(bbox)) {
    for (const auto & element : ll.regulatoryElementsAs<lanelet::TrafficLight>()) {
      const auto traffic_signal_stamped = planner_data.get_traffic_signal(element->id());
      if (
        traffic_signal_stamped.has_value() && element->stopLine().has_value() &&
        autoware::traffic_light_utils::isTrafficSignalStop(
          ll, traffic_signal_stamped.value().signal)) {
        stop_line_node.second.stop_line.clear();
        for (const auto & p : element->stopLine()->basicLineString()) {
          stop_line_node.second.stop_line.emplace_back(p.x(), p.y());
        }
        // use a longer stop line to also cut predicted paths that slightly go around the stop line
        const auto diff =
          stop_line_node.second.stop_line.back() - stop_line_node.second.stop_line.front();
        stop_line_node.second.stop_line.front() -= diff * 0.5;
        stop_line_node.second.stop_line.back() += diff * 0.5;
        stop_line_node.second.lanelets = planner_data.route_handler->getPreviousLanelets(ll);
        stop_line_node.first =
          boost::geometry::return_envelope<autoware_utils::Box2d>(stop_line_node.second.stop_line);
        rtree_nodes.push_back(stop_line_node);
      }
    }
  }
  ego_data.stop_lines_rtree = {rtree_nodes.begin(), rtree_nodes.end()};
}

out_of_lane::OutOfLaneData prepare_out_of_lane_data(const out_of_lane::EgoData & ego_data)
{
  out_of_lane::OutOfLaneData out_of_lane_data;
  out_of_lane_data.outside_points = out_of_lane::calculate_out_of_lane_points(ego_data);
  out_of_lane::prepare_out_of_lane_areas_rtree(out_of_lane_data);
  return out_of_lane_data;
}

std::optional<geometry_msgs::msg::Pose> OutOfLaneModule::calculate_slowdown_pose(
  const out_of_lane::EgoData & ego_data, const out_of_lane::OutOfLaneData & out_of_lane_data)
{
  // points are ordered by trajectory index so the first one has the smallest index and arc length
  const auto point_to_avoid_it = std::find_if(
    out_of_lane_data.outside_points.cbegin(), out_of_lane_data.outside_points.cend(),
    [&](const auto & p) { return p.to_avoid; });
  const auto has_point_to_avoid = (point_to_avoid_it != out_of_lane_data.outside_points.cend());
  const auto slowdown_pose =
    has_point_to_avoid ? out_of_lane::calculate_slowdown_pose(ego_data, *point_to_avoid_it, params_)
                       : std::nullopt;

  const auto log_cannot_stop = [&]() {
    if (has_point_to_avoid && !slowdown_pose) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 1000, "Could not insert slowdown point because of deceleration limits");
    }
  };

  update_slowdown_pose_buffer(ego_data, slowdown_pose);

  if (slowdown_pose_buffer_.empty()) {
    log_cannot_stop();
    return {};
  }

  // get nearest active slowdown pose
  auto min_arc_length = std::numeric_limits<double>::max();
  std::optional<out_of_lane::SlowdownPose> nearest_slowdown_pose = {};
  for (const auto & sp : slowdown_pose_buffer_) {
    if (sp.arc_length > min_arc_length || !sp.is_active) continue;
    nearest_slowdown_pose = sp;
    min_arc_length = sp.arc_length;
  }

  if (!nearest_slowdown_pose) {
    log_cannot_stop();
    return {};
  }

  return motion_utils::calcInterpolatedPose(
    ego_data.trajectory_points, nearest_slowdown_pose->arc_length);
}

void OutOfLaneModule::update_slowdown_pose_buffer(
  const out_of_lane::EgoData & ego_data,
  const std::optional<geometry_msgs::msg::Pose> & slowdown_pose)
{
  const double slowdown_pose_arc_length =
    slowdown_pose
      ? motion_utils::calcSignedArcLength(ego_data.trajectory_points, 0LU, slowdown_pose->position)
      : std::numeric_limits<double>::max();

  // remove no longer valid slowdown poses in the buffer:
  //  slowdown poses that are active but have exceeded the duration threshold since last detection
  //  slowdown poses that are invalid and not near the new slowdown pose
  std::vector<out_of_lane::SlowdownPose> valid_poses;
  for (auto & sp : slowdown_pose_buffer_) {
    const auto sp_duration = (clock_->now() - sp.start_time).seconds();
    if (sp.is_active && sp_duration > params_.min_off_duration) continue;
    if (!sp.is_active && !slowdown_pose) continue;

    sp.arc_length =
      motion_utils::calcSignedArcLength(ego_data.trajectory_points, 0LU, sp.pose.position);
    if (
      !sp.is_active && abs(sp.arc_length - slowdown_pose_arc_length) > params_.update_distance_th) {
      continue;
    }

    if (!sp.is_active && (clock_->now() - sp.start_time).seconds() > params_.min_on_duration) {
      sp.is_active = true;
      sp.start_time = clock_->now();
    }
    valid_poses.push_back(sp);
  }

  slowdown_pose_buffer_ = valid_poses;

  if (!slowdown_pose) return;

  static constexpr double eps = 1e-3;
  if (slowdown_pose_buffer_.empty()) {
    slowdown_pose_buffer_.emplace_back(
      slowdown_pose_arc_length, clock_->now(), *slowdown_pose, params_.min_on_duration < eps);
    return;
  }

  auto nearest_prev_pose_it = slowdown_pose_buffer_.end();
  auto min_relative_dist = std::numeric_limits<double>::max();
  for (auto it = slowdown_pose_buffer_.begin(); it < slowdown_pose_buffer_.end(); ++it) {
    const auto rel_dist = it->arc_length - slowdown_pose_arc_length;
    if (std::abs(rel_dist) < params_.update_distance_th && rel_dist < min_relative_dist) {
      nearest_prev_pose_it = it;
      min_relative_dist = rel_dist;
    }
  }

  if (nearest_prev_pose_it == slowdown_pose_buffer_.end()) {
    slowdown_pose_buffer_.emplace_back(
      slowdown_pose_arc_length, clock_->now(), *slowdown_pose, params_.min_on_duration < eps);
    return;
  }

  if (min_relative_dist > 0) {
    nearest_prev_pose_it->pose = *slowdown_pose;
    nearest_prev_pose_it->arc_length = slowdown_pose_arc_length;
  }

  if (nearest_prev_pose_it->is_active) {
    nearest_prev_pose_it->start_time = clock_->now();
  }
}

void OutOfLaneModule::update_result(
  VelocityPlanningResult & result, const std::optional<geometry_msgs::msg::Pose> & slowdown_pose,
  const out_of_lane::EgoData & ego_data, const out_of_lane::OutOfLaneData & out_of_lane_data)
{
  if (!slowdown_pose) {
    return;
  }
  const auto arc_length =
    motion_utils::calcSignedArcLength(ego_data.trajectory_points, 0UL, slowdown_pose->position) -
    ego_data.longitudinal_offset_to_first_trajectory_index;
  const auto slowdown_velocity =
    arc_length <= params_.stop_dist_threshold ? 0.0 : params_.slow_velocity;
  previous_slowdown_pose_ = slowdown_pose;
  if (slowdown_velocity == 0.0) {
    result.stop_points.push_back(slowdown_pose->position);
  } else {
    result.slowdown_intervals.emplace_back(
      slowdown_pose->position, slowdown_pose->position, slowdown_velocity);
  }
  virtual_wall_marker_creator.add_virtual_walls(
    out_of_lane::debug::create_virtual_walls(*slowdown_pose, slowdown_velocity == 0.0, params_));
  virtual_wall_publisher_->publish(virtual_wall_marker_creator.create_markers(clock_->now()));

  SafetyFactorArray safety_factors;
  safety_factors.header.stamp = clock_->now();
  safety_factors.header.frame_id = "map";

  const auto avoided_point_it = std::find_if(
    out_of_lane_data.outside_points.cbegin(), out_of_lane_data.outside_points.cend(),
    [&](const auto & p) { return p.to_avoid; });
  std::unordered_map<std::string, autoware_internal_planning_msgs::msg::SafetyFactor>
    factor_per_object;
  if (avoided_point_it != out_of_lane_data.outside_points.cend()) {
    for (const auto & collision : avoided_point_it->collision_times) {
      const auto is_possible_collision =
        collision.collision_time >= avoided_point_it->min_object_arrival_time &&
        collision.collision_time <= avoided_point_it->max_object_arrival_time;
      if (is_possible_collision) {
        const auto uuid = autoware_utils_uuid::to_hex_string(collision.object_uuid);
        if (factor_per_object.count(uuid) == 0) {
          autoware_internal_planning_msgs::msg::SafetyFactor sf;
          sf.is_safe = false;
          sf.object_id = collision.object_uuid;
          sf.type = autoware_internal_planning_msgs::msg::SafetyFactor::OBJECT;
          sf.ttc_begin = static_cast<float>(*avoided_point_it->ttc);
          sf.ttc_end = static_cast<float>(*avoided_point_it->ttc);
          factor_per_object[uuid] = sf;
        } else {
          auto & sf = factor_per_object[uuid];
          sf.ttc_begin = std::min(sf.ttc_begin, static_cast<float>(*avoided_point_it->ttc));
          sf.ttc_end = std::max(sf.ttc_end, static_cast<float>(*avoided_point_it->ttc));
        }
      }
    }
  }
  for (const auto & [_, safety_factor] : factor_per_object) {
    safety_factors.factors.push_back(safety_factor);
  }

  const auto planning_factor =
    slowdown_velocity == 0.0 ? PlanningFactor::STOP : PlanningFactor::SLOW_DOWN;
  planning_factor_interface_->add(
    ego_data.trajectory_points, ego_data.pose, *slowdown_pose, planning_factor, safety_factors);
}

VelocityPlanningResult OutOfLaneModule::plan(
  [[maybe_unused]] const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> &
    raw_trajectory_points,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & smoothed_trajectory_points,
  const std::shared_ptr<const PlannerData> planner_data)
{
  VelocityPlanningResult result;
  autoware_utils::StopWatch<std::chrono::microseconds> stopwatch;
  stopwatch.tic();

  stopwatch.tic("preprocessing");
  out_of_lane::EgoData ego_data;
  ego_data.pose = planner_data->current_odometry.pose.pose;
  limit_trajectory_size(ego_data, smoothed_trajectory_points, params_.max_arc_length);
  out_of_lane::calculate_min_stop_and_slowdown_distances(
    ego_data, *planner_data, previous_slowdown_pose_);
  prepare_stop_lines_rtree(ego_data, *planner_data, params_.max_arc_length);
  ego_data.map_stop_points = planner_data->calculate_map_stop_points(ego_data.trajectory_points);
  const auto preprocessing_us = stopwatch.toc("preprocessing");

  stopwatch.tic("calculate_trajectory_footprints");
  ego_data.current_footprint =
    out_of_lane::calculate_current_ego_footprint(ego_data, params_, true);
  ego_data.trajectory_footprints = out_of_lane::calculate_trajectory_footprints(ego_data, params_);
  const auto calculate_trajectory_footprints_us = stopwatch.toc("calculate_trajectory_footprints");

  stopwatch.tic("calculate_lanelets");
  out_of_lane::calculate_out_lanelet_rtree(ego_data, *planner_data->route_handler, params_);
  const auto calculate_lanelets_us = stopwatch.toc("calculate_lanelets");

  stopwatch.tic("calculate_out_of_lane_areas");
  auto out_of_lane_data = prepare_out_of_lane_data(ego_data);
  const auto calculate_out_of_lane_areas_us = stopwatch.toc("calculate_out_of_lane_areas");

  stopwatch.tic("filter_predicted_objects");
  const auto objects = out_of_lane::filter_predicted_objects(*planner_data, ego_data, params_);
  const auto filter_predicted_objects_us = stopwatch.toc("filter_predicted_objects");

  stopwatch.tic("calculate_time_collisions");
  out_of_lane::calculate_objects_time_collisions(
    out_of_lane_data, objects.objects, *planner_data->route_handler, params_);
  const auto calculate_time_collisions_us = stopwatch.toc("calculate_time_collisions");

  stopwatch.tic("calculate_times");
  const auto is_stopping = previous_slowdown_pose_.has_value();
  out_of_lane::calculate_collisions_to_avoid(
    out_of_lane_data, ego_data.trajectory_points, params_, is_stopping);
  const auto calculate_times_us = stopwatch.toc("calculate_times");

  const auto is_already_overlapping =
    params_.skip_if_already_overlapping &&
    std::find_if(ego_data.out_lanelets.begin(), ego_data.out_lanelets.end(), [&](const auto & ll) {
      return !boost::geometry::disjoint(ll.polygon2d().basicPolygon(), ego_data.current_footprint);
    }) != ego_data.out_lanelets.end();
  if (is_already_overlapping) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000, "Ego is already out of lane, skipping the module\n");
    debug_publisher_->publish(
      out_of_lane::debug::create_debug_marker_array(
        ego_data, out_of_lane_data, objects, debug_data_));
    return result;
  }

  stopwatch.tic("calculate_slowdown_point");
  const auto slowdown_pose = calculate_slowdown_pose(ego_data, out_of_lane_data);
  const auto calculate_slowdown_point_us = stopwatch.toc("calculate_slowdown_point");

  update_result(result, slowdown_pose, ego_data, out_of_lane_data);
  stopwatch.tic("gen_debug");
  const auto markers =
    out_of_lane::debug::create_debug_marker_array(ego_data, out_of_lane_data, objects, debug_data_);
  const auto markers_us = stopwatch.toc("gen_debug");
  stopwatch.tic("pub");
  debug_publisher_->publish(markers);
  const auto pub_markers_us = stopwatch.toc("pub");
  const auto total_time_us = stopwatch.toc();
  std::map<std::string, double> processing_times;
  processing_times["preprocessing"] = preprocessing_us / 1000;
  processing_times["calculate_lanelets"] = calculate_lanelets_us / 1000;
  processing_times["calculate_trajectory_footprints"] = calculate_trajectory_footprints_us / 1000;
  processing_times["calculate_out_of_lane_areas"] = calculate_out_of_lane_areas_us / 1000;
  processing_times["filter_pred_objects"] = filter_predicted_objects_us / 1000;
  processing_times["calculate_time_collisions"] = calculate_time_collisions_us / 1000;
  processing_times["calculate_times"] = calculate_times_us / 1000;
  processing_times["calculate_slowdown_point"] = calculate_slowdown_point_us / 1000;
  processing_times["generate_markers"] = markers_us / 1000;
  processing_times["publish_markers"] = pub_markers_us / 1000;
  processing_times["Total"] = total_time_us / 1000;
  processing_diag_publisher_->publish(processing_times);
  autoware_internal_debug_msgs::msg::Float64Stamped processing_time_msg;
  processing_time_msg.stamp = clock_->now();
  processing_time_msg.data = processing_times["Total"];
  processing_time_publisher_->publish(processing_time_msg);
  return result;
}

}  // namespace autoware::motion_velocity_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::motion_velocity_planner::OutOfLaneModule,
  autoware::motion_velocity_planner::PluginModuleInterface)
