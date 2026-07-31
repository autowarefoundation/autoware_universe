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

#include "autoware/trajectory_validator/filters/traffic_rule/crosswalk_filter.hpp"

#include <autoware/motion_utils/distance/distance.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/object_recognition_utils/object_classification.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/Forward.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/crosswalk.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_utils/geometry/boost_polygon_utils.hpp>
#include <autoware_utils/ros/marker_helper.hpp>
#include <autoware_utils_geometry/boost_geometry.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/intersection.hpp>

#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/Polygon.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace
{
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_utils_geometry::Line2d;

ObjectClassification::_label_type to_classification_label(const std::string & label_str)
{
  static const std::unordered_map<std::string, ObjectClassification::_label_type>
    string_to_classification_map = {
      {"unknown", ObjectClassification::UNKNOWN}, {"car", ObjectClassification::CAR},
      {"truck", ObjectClassification::TRUCK},     {"bus", ObjectClassification::BUS},
      {"trailer", ObjectClassification::TRAILER}, {"motorcycle", ObjectClassification::MOTORCYCLE},
      {"bicycle", ObjectClassification::BICYCLE}, {"pedestrian", ObjectClassification::PEDESTRIAN},
      {"animal", ObjectClassification::ANIMAL},   {"hazard", ObjectClassification::HAZARD}};

  const auto it = string_to_classification_map.find(label_str);
  if (it == string_to_classification_map.end()) {
    return ObjectClassification::UNKNOWN;
  }
  return it->second;
}

bool is_signaled_crosswalk(const lanelet::CrosswalkConstPtr & crosswalk_reg_elem)
{
  const auto & crosswalk_lanelet = crosswalk_reg_elem->crosswalkLanelet();
  return !crosswalk_lanelet.regulatoryElementsAs<const lanelet::TrafficLight>().empty();
}

std::vector<lanelet::CrosswalkConstPtr> collect_crosswalk_reg_elems_from_route(
  const lanelet::LaneletMap & lanelet_map, const autoware_planning_msgs::msg::LaneletRoute & route)
{
  lanelet::ConstLanelets route_lanelets;
  for (const auto & segment : route.segments) {
    const auto ll = lanelet_map.laneletLayer.get(segment.preferred_primitive.id);
    route_lanelets.push_back(ll);
  }

  std::vector<lanelet::CrosswalkConstPtr> crosswalks_on_route;
  for (const auto & cw : lanelet::utils::query::crosswalks(route_lanelets)) {
    if (is_signaled_crosswalk(cw)) {
      continue;
    }
    crosswalks_on_route.push_back(cw);
  }
  return crosswalks_on_route;
}

lanelet::BasicPoint2d extrude_bound_end(
  const lanelet::BasicSegment2d & segment, const double extra_distance)
{
  const auto direction_vector = (segment.second - segment.first).normalized();
  return segment.second + extra_distance * direction_vector;
}

/// @brief Build sidewalk-side waiting areas as end caps extruded from the crosswalk lanelet ends.
/// Longitudinal extrusion follows the pedestrian direction; lateral expansion widens each end
/// along the crosswalk end edge so pedestrians standing beside / in front of the entry are covered.
lanelet::BasicPolygons2d get_detection_areas(
  const lanelet::CrosswalkConstPtr & crosswalk, const double lon_margin, const double lat_margin)
{
  if (lon_margin <= 1e-3) return lanelet::BasicPolygons2d{};

  const auto & crosswalk_lanelet = crosswalk->crosswalkLanelet();
  const auto left = crosswalk_lanelet.leftBound2d();
  const auto right = crosswalk_lanelet.rightBound2d();
  if (left.size() < 2 || right.size() < 2) {
    return lanelet::BasicPolygons2d{};
  }

  const std::vector<std::function<lanelet::BasicSegment2d(const lanelet::ConstLineString2d &)>>
    segment_getters = {
      [](const auto & ls) -> lanelet::BasicSegment2d {
        return {ls[1].basicPoint2d(), ls[0].basicPoint2d()};
      },
      [](const auto & ls) -> lanelet::BasicSegment2d {
        return {ls[ls.size() - 2].basicPoint2d(), ls[ls.size() - 1].basicPoint2d()};
      }};

  lanelet::BasicPolygons2d detection_areas;
  for (const auto & segment_getter : segment_getters) {
    const auto left_segment = segment_getter(left);
    const auto right_segment = segment_getter(right);
    const auto target_left = extrude_bound_end(left_segment, lon_margin);
    const auto target_right = extrude_bound_end(right_segment, lon_margin);

    // Widen the end cap along the end edge (left <-> right).
    const auto end_edge = right_segment.second - left_segment.second;
    const auto end_edge_length = end_edge.norm();
    if (end_edge_length <= 1e-3) {
      continue;
    }
    const auto lateral_dir = end_edge / end_edge_length;

    const auto left_near = left_segment.second - lat_margin * lateral_dir;
    const auto right_near = right_segment.second + lat_margin * lateral_dir;
    const auto left_far = target_left - lat_margin * lateral_dir;
    const auto right_far = target_right + lat_margin * lateral_dir;

    detection_areas.emplace_back(
      lanelet::BasicPolygon2d{left_near, left_far, right_far, right_near});
  }
  return detection_areas;
}

using autoware::trajectory_validator::plugin::traffic_rule::CrosswalkOnTrajectory;
std::vector<CrosswalkOnTrajectory> filter_crosswalks_intersecting_trajectory(
  const std::vector<lanelet::CrosswalkConstPtr> & crosswalks_on_route,
  const lanelet::BasicLineString2d & trajectory_ls)
{
  std::vector<CrosswalkOnTrajectory> crosswalks_on_trajectory;

  if (trajectory_ls.size() < 2 || crosswalks_on_route.empty()) {
    return crosswalks_on_trajectory;
  }

  std::unordered_set<lanelet::Id> seen_crosswalk_ids;
  auto checked_length = 0.0;

  auto process_cw =
    [&](const lanelet::CrosswalkConstPtr & cw, const lanelet::BasicLineString2d & traj_seg) {
      auto stop_lines = cw->stopLines();
      if (stop_lines.empty()) return;

      auto distance_to_stop_line = std::numeric_limits<double>::max();
      std::optional<lanelet::BasicPoints2d> stop_line;
      for (const auto & sl : stop_lines) {
        const auto stop_line_2d = lanelet::utils::to2D(sl).basicLineString();
        lanelet::BasicPoints2d intersection_points;
        boost::geometry::intersection(traj_seg, stop_line_2d, intersection_points);
        if (!intersection_points.empty()) {
          const auto distance = checked_length + static_cast<double>(boost::geometry::distance(
                                                   traj_seg.front(), intersection_points.front()));
          if (distance < distance_to_stop_line) {
            distance_to_stop_line = distance;
            stop_line = stop_line_2d;
          }
        }
      }
      if (stop_line) {
        crosswalks_on_trajectory.emplace_back(cw, distance_to_stop_line, *stop_line);
        seen_crosswalk_ids.insert(cw->id());
      }
    };

  for (size_t i = 0; i + 1 < trajectory_ls.size(); ++i) {
    const lanelet::BasicLineString2d segment{trajectory_ls[i], trajectory_ls[i + 1]};

    for (const auto & cw : crosswalks_on_route) {
      if (seen_crosswalk_ids.count(cw->id())) continue;
      process_cw(cw, segment);
    }
    checked_length += static_cast<double>(boost::geometry::length(segment));
  }

  return crosswalks_on_trajectory;
}

}  // namespace

namespace autoware::trajectory_validator::plugin::traffic_rule
{

CrosswalkFilter::CrosswalkFilter() : ValidatorInterface("crosswalk_filter")
{
}

void CrosswalkFilter::update_parameters(const validator::Params & params)
{
  params_ = params.crosswalk;
  object_types_.clear();
  for (const auto & object_type_string : params_.object_types) {
    object_types_.insert(to_classification_label(object_type_string));
  }
}

void CrosswalkFilter::set_vehicle_info(const VehicleInfo & vehicle_info)
{
  ValidatorInterface::set_vehicle_info(vehicle_info);
}

CrosswalkFilter::result_t CrosswalkFilter::is_feasible(
  const CandidateTrajectory & candidate_trajectory, const FilterContext & context)
{
  if (!context.lanelet_map) {
    return tl::make_unexpected("Lanelet map is not available in the context.");
  }

  if (!context.route) {
    return tl::make_unexpected("Route is not available in the context.");
  }

  const auto target_crosswalks = get_target_crosswalks(candidate_trajectory.points, context);

  std::vector<MetricReport> metrics;
  if (target_crosswalks.empty()) return ValidationResult{true, std::move(metrics)};

  update_target_objects(context, target_crosswalks);

  std::unordered_set<lanelet::Id> obstructing_crosswalk_ids;
  const bool feasible =
    std::none_of(target_crosswalks.begin(), target_crosswalks.end(), [&](const auto & cw) {
      if (!is_obstructing_crosswalk(candidate_trajectory.points, cw)) return false;
      obstructing_crosswalk_ids.insert(cw.crosswalk_info.crosswalk->id());
      return true;
    });

  update_debug_data(
    candidate_trajectory.points, target_crosswalks, obstructing_crosswalk_ids,
    context.odometry->header.stamp, context.odometry->pose.pose.position.z);

  RiskLevel risk_level;
  risk_level.level = feasible ? RiskLevel::SAFE : RiskLevel::DANGER;
  metrics.push_back(
    autoware_trajectory_validator::build<MetricReport>()
      .validator_name(get_name())
      .validator_category(category())
      .metric_name("check_crosswalk_obstruction")
      .metric_value(0.0)
      .risk(risk_level));

  return ValidationResult{feasible, std::move(metrics)};
}

std::vector<TargetCrosswalk> CrosswalkFilter::get_target_crosswalks(
  const TrajectoryPoints & traj_points, const FilterContext & context)
{
  std::vector<TargetCrosswalk> target_crosswalks;

  if (traj_points.size() < 2) return target_crosswalks;

  const auto crosswalks_on_route =
    collect_crosswalk_reg_elems_from_route(*context.lanelet_map, *context.route);

  if (crosswalks_on_route.empty()) return target_crosswalks;

  const double current_vel = context.odometry->twist.twist.linear.x;
  const double current_acc = context.acceleration->accel.accel.linear.x;
  const auto decel_limit = 1.0;
  const auto jerk_limit = 1.0;

  auto stop_distance = autoware::motion_utils::calculate_stop_distance(
    current_vel, current_acc, decel_limit, jerk_limit);
  const auto lookahead_distance_m = stop_distance
                                      ? *stop_distance + params_.arrived_distance_threshold
                                      : std::numeric_limits<double>::max();

  lanelet::BasicLineString2d trajectory_ls;
  double length = 0.0;

  for (const auto & p : traj_points) {
    // skip points behind ego
    if (rclcpp::Duration(p.time_from_start).seconds() < 0.0) {
      continue;
    }

    const lanelet::BasicPoint2d lanelet_p(p.pose.position.x, p.pose.position.y);
    if (!trajectory_ls.empty()) {
      length += lanelet::geometry::distance2d(trajectory_ls.back(), lanelet_p);
    }
    trajectory_ls.emplace_back(lanelet_p);

    // skip points beyond the first stop, or skip once we reach the maximum length
    if (p.longitudinal_velocity_mps <= 1e-6 || length > lookahead_distance_m) {
      break;
    }
  }

  auto longitudinal_offset_m =
    vehicle_info_ptr_->max_longitudinal_offset_m - params_.overshoot_tolerance;

  const auto trajectory_footprint_length = length + longitudinal_offset_m;
  if (length < params_.arrived_distance_threshold) {
    longitudinal_offset_m += (params_.arrived_distance_threshold - length);
  }

  if (longitudinal_offset_m > 0.0) {
    // extend the trajectory linestring by longitudinal offset
    const auto offset_pose = autoware_utils_geometry::calc_offset_pose(
      traj_points.back().pose, longitudinal_offset_m, 0.0, 0.0);
    const lanelet::BasicPoint2d offset_point(offset_pose.position.x, offset_pose.position.y);
    trajectory_ls.emplace_back(offset_point);
  }

  const auto intersecting_crosswalks =
    filter_crosswalks_intersecting_trajectory(crosswalks_on_route, trajectory_ls);

  if (intersecting_crosswalks.empty()) return target_crosswalks;

  for (const auto & cw : intersecting_crosswalks) {
    auto crosswalk_polygon = cw.crosswalk->crosswalkLanelet().polygon2d().basicPolygon();
    const bool is_crossing = trajectory_footprint_length >= cw.arc_length_to_stop_line_m;
    target_crosswalks.emplace_back(
      cw, crosswalk_polygon,
      get_detection_areas(cw.crosswalk, params_.lon_detection_margin, params_.lat_detection_margin),
      is_crossing);
  }

  return target_crosswalks;
}

void CrosswalkFilter::update_target_objects(
  const FilterContext & context, const TargetCrosswalks & target_crosswalks)
{
  std::unordered_set<lanelet::Id> target_crosswalk_ids;
  target_crosswalk_ids.reserve(target_crosswalks.size());
  for (const auto & cw : target_crosswalks) {
    target_crosswalk_ids.insert(cw.crosswalk_info.crosswalk->id());
  }
  for (auto it = crosswalk_objects_map_.begin(); it != crosswalk_objects_map_.end();) {
    if (target_crosswalk_ids.count(it->first) == 0) {
      it = crosswalk_objects_map_.erase(it);
    } else {
      ++it;
    }
  }

  auto objects = context.predicted_objects->objects;

  const auto current_time = rclcpp::Time(context.odometry->header.stamp);

  objects.erase(
    std::remove_if(
      objects.begin(), objects.end(),
      [&](const auto & object) {
        auto label = object.classification.empty()
                       ? ObjectClassification::UNKNOWN
                       : object_recognition_utils::getHighestProbLabel(object.classification);
        return object_types_.count(label) == 0;
      }),
    objects.end());

  const auto ego_vel = context.odometry->twist.twist.linear.x;
  const auto ego_longitudinal_offset = vehicle_info_ptr_->max_longitudinal_offset_m;

  auto is_stopped_at_crosswalk = [&](const TargetCrosswalk & cw) {
    const auto dist_to_cw = cw.crosswalk_info.arc_length_to_stop_line_m;
    if (dist_to_cw - ego_longitudinal_offset > params_.arrived_distance_threshold) return false;
    return ego_vel < 0.1;
  };

  auto update_object = [&](const PredictedObject & obj, const auto cw_id, bool stopped_at_cw) {
    if (crosswalk_objects_map_.count(cw_id) == 0) {
      crosswalk_objects_map_[cw_id] = TargetObjects{};
    }
    auto & cw_objects = crosswalk_objects_map_[cw_id];
    const auto it = std::find_if(cw_objects.begin(), cw_objects.end(), [&](const auto & cw_object) {
      return cw_object.matches(obj, params_.distance_hysteresis_th);
    });
    if (it == cw_objects.end()) {
      cw_objects.emplace_back(obj, current_time, current_time);
      return;
    }
    it->object = obj;
    it->last_seen_time = current_time;
    if (it->ignore) return;
    it->first_seen_time = stopped_at_cw ? it->first_seen_time : current_time;
    it->ignore = (it->last_seen_time - it->first_seen_time).seconds() > params_.stop_duration;
  };

  auto clear_old_objects = [&](const auto cw_id) {
    if (crosswalk_objects_map_.count(cw_id) == 0) return;
    auto & cw_objects = crosswalk_objects_map_[cw_id];
    cw_objects.erase(
      std::remove_if(
        cw_objects.begin(), cw_objects.end(),
        [&](const auto & cw_object) {
          return (current_time - cw_object.last_seen_time).seconds() > params_.object_clear_time_th;
        }),
      cw_objects.end());
  };

  auto is_inside_detection_areas =
    [&](const PredictedObject & obj, const lanelet::BasicPolygons2d & detection_areas) {
      const auto obj_position = obj.kinematics.initial_pose_with_covariance.pose.position;
      lanelet::BasicPoint2d obj_point(obj_position.x, obj_position.y);
      return std::any_of(detection_areas.begin(), detection_areas.end(), [&](const auto & area) {
        return lanelet::geometry::distance2d(area, obj_point) < 1e-3;
      });
    };

  for (const auto & cw : target_crosswalks) {
    auto detection_areas = cw.detection_areas;
    bool is_ego_stopped_at_cw = is_stopped_at_crosswalk(cw);
    for (const auto & object : objects) {
      if (!is_inside_detection_areas(object, detection_areas)) continue;
      update_object(object, cw.crosswalk_info.crosswalk->id(), is_ego_stopped_at_cw);
    }
    clear_old_objects(cw.crosswalk_info.crosswalk->id());
  }
}

bool CrosswalkFilter::is_obstructing_crosswalk(
  const TrajectoryPoints & traj_points, const TargetCrosswalk & target_crosswalk) const
{
  if (!target_crosswalk.is_crossing) return false;

  constexpr double zero_vel_threshold = 0.1;
  const auto start_move_it = std::find_if(
    traj_points.begin(), traj_points.end(),
    [&](const auto & p) { return p.longitudinal_velocity_mps > zero_vel_threshold; });

  // skip check for non-moving trajectory
  if (start_move_it == traj_points.end()) return false;

  if (crosswalk_objects_map_.count(target_crosswalk.crosswalk_info.crosswalk->id()) == 0)
    return false;

  const auto & cw_objects =
    crosswalk_objects_map_.at(target_crosswalk.crosswalk_info.crosswalk->id());
  if (cw_objects.empty()) return false;

  auto min_duration_obj = cw_objects.front();
  const auto required_waiting_time = [&]() {
    auto min_object_duration = params_.stop_duration;
    for (const auto & obj : cw_objects) {
      if (obj.ignore) continue;
      const auto duration = rclcpp::Duration(obj.last_seen_time - obj.first_seen_time).seconds();
      if (duration >= min_object_duration) continue;
      min_object_duration = duration;
      min_duration_obj = obj;
    }
    return params_.stop_duration - min_object_duration;
  }();

  if (required_waiting_time < 1e-3) return false;

  if (!params_.use_trajectory_time) return true;

  // check if stopping duration is sufficient
  const auto start_move_time = rclcpp::Duration(start_move_it->time_from_start).seconds();
  return start_move_time < required_waiting_time;
}

void CrosswalkFilter::update_debug_data(
  const TrajectoryPoints & traj_points, const std::vector<TargetCrosswalk> & target_crosswalks,
  const std::unordered_set<lanelet::Id> & obstructing_crosswalk_ids,
  const rclcpp::Time & current_time, const double z)
{
  using visualization_msgs::msg::Marker;
  debug_markers_.markers.clear();

  auto add_polygon_marker = [&](
                              const auto & polygon, const std::string & ns, const int id,
                              const std_msgs::msg::ColorRGBA & color, const double scale = 0.1) {
    visualization_msgs::msg::Marker marker = autoware_utils::create_default_marker(
      "map", current_time, ns, id, Marker::LINE_STRIP,
      autoware_utils::create_marker_scale(scale, scale, scale), color);
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);

    for (const auto & p : polygon) {
      marker.points.push_back(autoware_utils::create_marker_position(p.x(), p.y(), z));
    }
    if (!marker.points.empty()) {
      marker.points.push_back(marker.points.front());
    }
    debug_markers_.markers.push_back(marker);
  };

  auto add_multi_polygon_marker = [&](
                                    const auto & polygons, const std::string & ns, const int id,
                                    const std_msgs::msg::ColorRGBA & color,
                                    const double scale = 0.1) {
    visualization_msgs::msg::Marker marker = autoware_utils::create_default_marker(
      "map", current_time, ns, id, Marker::LINE_LIST,
      autoware_utils::create_marker_scale(scale, scale, scale), color);
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    for (const auto & polygon : polygons) {
      if (polygon.empty()) {
        continue;
      }
      boost::geometry::for_each_segment(polygon, [&](const auto & s) {
        const auto & [p1, p2] = s;
        marker.points.push_back(autoware_utils::create_marker_position(p1.x(), p1.y(), z));
        marker.points.push_back(autoware_utils::create_marker_position(p2.x(), p2.y(), z));
      });
    }
    debug_markers_.markers.push_back(marker);
  };

  auto add_line_marker = [&](
                           const lanelet::BasicLineString2d & line, const std::string & ns,
                           const int id, const std_msgs::msg::ColorRGBA & color,
                           const double scale = 0.15) {
    if (line.empty()) return;
    visualization_msgs::msg::Marker marker = autoware_utils::create_default_marker(
      "map", current_time, ns, id, Marker::LINE_STRIP,
      autoware_utils::create_marker_scale(scale, scale, scale), color);
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    for (const auto & p : line) {
      marker.points.push_back(autoware_utils::create_marker_position(p.x(), p.y(), z));
    }
    debug_markers_.markers.push_back(marker);
  };

  auto add_text_marker = [&](
                           const std::string & text, const auto & pose, const std::string & ns,
                           const int id, const std_msgs::msg::ColorRGBA & color,
                           const double scale = 0.2) {
    visualization_msgs::msg::Marker marker = autoware_utils::create_default_marker(
      "map", current_time, ns, id, Marker::TEXT_VIEW_FACING,
      autoware_utils::create_marker_scale(scale, scale, scale), color);
    marker.pose = pose;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    marker.text = text;
    debug_markers_.markers.push_back(marker);
  };

  const auto magenta = autoware_utils::create_marker_color(1.0, 0.0, 1.0, 1.0);
  const auto yellow = autoware_utils::create_marker_color(1.0, 1.0, 0.0, 1.0);
  const auto green = autoware_utils::create_marker_color(0.0, 1.0, 0.0, 1.0);
  const auto red = autoware_utils::create_marker_color(1.0, 0.0, 0.0, 1.0);
  const auto white = autoware_utils::create_marker_color(1.0, 1.0, 1.0, 1.0);

  double remaining_time{};
  auto add_objects_marker = [&](const auto & cw) {
    if (crosswalk_objects_map_.count(cw.crosswalk_info.crosswalk->id()) == 0) return;
    int obj_id = 0;
    remaining_time = 0.0;
    auto min_obj_duration = params_.stop_duration;
    for (const auto & cw_object : crosswalk_objects_map_[cw.crosswalk_info.crosswalk->id()]) {
      const auto obj_duration = (cw_object.last_seen_time - cw_object.first_seen_time).seconds();
      const auto color = cw_object.ignore ? green : red;
      auto obj_polygon = autoware_utils_geometry::to_polygon2d(
        cw_object.object.kinematics.initial_pose_with_covariance.pose, cw_object.object.shape);
      add_polygon_marker(obj_polygon.outer(), "target_objects", obj_id, color);
      if (!cw_object.ignore) {
        add_text_marker(
          std::to_string(obj_duration),
          cw_object.object.kinematics.initial_pose_with_covariance.pose, "target_objects_duration",
          obj_id, white);
        min_obj_duration = std::min(min_obj_duration, obj_duration);
      }
      obj_id++;
    }
    remaining_time = params_.stop_duration - min_obj_duration;
  };

  auto get_arrival_linestring = [&](const TargetCrosswalk & cw) {
    auto arc_length_to_arrival =
      cw.crosswalk_info.arc_length_to_stop_line_m - params_.arrived_distance_threshold;
    auto arrival_pose =
      motion_utils::calcLongitudinalOffsetPose(traj_points, 0, arc_length_to_arrival);
    if (!arrival_pose) return lanelet::BasicLineString2d{};
    const auto left_offset_pose =
      autoware_utils_geometry::calc_offset_pose(*arrival_pose, 0.0, -1.0, 0.0);
    const auto right_offset_pose =
      autoware_utils_geometry::calc_offset_pose(*arrival_pose, 0.0, 1.0, 0.0);
    lanelet::BasicLineString2d arrival_ls;
    arrival_ls.emplace_back(
      lanelet::BasicPoint2d(left_offset_pose.position.x, left_offset_pose.position.y));
    arrival_ls.emplace_back(
      lanelet::BasicPoint2d(right_offset_pose.position.x, right_offset_pose.position.y));
    return arrival_ls;
  };

  int id = 0;
  for (const auto & cw : target_crosswalks) {
    bool is_obstructing = obstructing_crosswalk_ids.count(cw.crosswalk_info.crosswalk->id()) > 0;
    add_polygon_marker(cw.crosswalk_polygon, "target_crosswalks", id, magenta);
    add_multi_polygon_marker(
      cw.detection_areas, "detection_areas", id, is_obstructing ? red : yellow);
    add_line_marker(
      cw.crosswalk_info.stop_line, "target_stop_lines", id, is_obstructing ? red : magenta);
    add_objects_marker(cw);
    if (is_obstructing) {
      geometry_msgs::msg::Pose stop_line_pose;
      stop_line_pose.position.x = cw.crosswalk_info.stop_line.front().x();
      stop_line_pose.position.y = cw.crosswalk_info.stop_line.front().y();
      stop_line_pose.position.z = z;
      const std::string remaining_time_str =
        "remaining waiting time:" + std::to_string(remaining_time) + " s";
      add_text_marker(remaining_time_str, stop_line_pose, "remaining_time", id, white, 0.3);
      add_line_marker(get_arrival_linestring(cw), "arrival_linestring", id, yellow);
    }
    id++;
  }
}

}  // namespace autoware::trajectory_validator::plugin::traffic_rule

#include <pluginlib/class_list_macros.hpp>
namespace traffic_rule = autoware::trajectory_validator::plugin::traffic_rule;
PLUGINLIB_EXPORT_CLASS(
  traffic_rule::CrosswalkFilter, autoware::trajectory_validator::plugin::ValidatorInterface)
