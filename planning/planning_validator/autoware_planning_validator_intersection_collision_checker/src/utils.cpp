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

#include "autoware/planning_validator_intersection_collision_checker/utils.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils/geometry/boost_geometry.hpp>
#include <autoware_utils/ros/marker_helper.hpp>

#include <std_msgs/msg/color_rgba.hpp>

#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/disjoint.hpp>
#include <boost/geometry/algorithms/intersection.hpp>

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>

namespace autoware::planning_validator::collision_checker_utils
{

namespace
{
bool contains_lanelet(const lanelet::ConstLanelets & lanelets, const lanelet::Id id)
{
  return std::find_if(lanelets.begin(), lanelets.end(), [&](const auto & l) {
           return l.id() == id;
         }) != lanelets.end();
}

bool is_turn_lanelet(const lanelet::ConstLanelet & ll)
{
  return ll.hasAttribute("turn_direction") && ll.attribute("turn_direction") != "straight";
}
}  // namespace

TrajectoryPoints trim_trajectory_points(
  const TrajectoryPoints & trajectory_points, const geometry_msgs::msg::Pose & start_pose)
{
  const auto nearest_idx =
    autoware::motion_utils::findNearestIndex(trajectory_points, start_pose.position);
  return TrajectoryPoints(trajectory_points.begin() + nearest_idx, trajectory_points.end());
}

void set_trajectory_lanelets(
  const TrajectoryPoints & trajectory_points, const RouteHandler & route_handler,
  const geometry_msgs::msg::Pose & ego_pose, EgoLanelets & lanelets)
{
  lanelet::ConstLanelet closest_lanelet;
  if (!route_handler.getClosestLaneletWithinRoute(ego_pose, &closest_lanelet)) {
    throw std::logic_error("failed to get closest lanelet within route");
  }

  const auto forward_trajectory_length = autoware::motion_utils::calcSignedArcLength(
    trajectory_points, ego_pose.position, trajectory_points.size() - 1);

  lanelets.trajectory_lanelets =
    route_handler.getLaneletSequence(closest_lanelet, ego_pose, 0.0, forward_trajectory_length);

  lanelet::ConstLanelets prev_lanelets{closest_lanelet};
  if (is_turn_lanelet(closest_lanelet)) {
    while (route_handler.getPreviousLaneletsWithinRoute(prev_lanelets.front(), &prev_lanelets) &&
           is_turn_lanelet(prev_lanelets.front())) {
      lanelets.trajectory_lanelets.insert(
        lanelets.trajectory_lanelets.begin(), prev_lanelets.front());
    }
  }

  if (route_handler.getPreviousLaneletsWithinRoute(closest_lanelet, &prev_lanelets)) {
    lanelets.connected_lanelets.push_back(prev_lanelets.front());
    for (const auto & connected_ll : route_handler.getNextLanelets(prev_lanelets.front())) {
      lanelets.connected_lanelets.push_back(connected_ll);
    }
  }

  bool lock_turn_lanelets = false;
  auto set_turn_lanelet = [&](const lanelet::ConstLanelet & ll) {
    if (lock_turn_lanelets) return;
    if (!is_turn_lanelet(ll)) {
      if (!lanelets.turn_lanelets.empty()) {
        lock_turn_lanelets = true;
      }
    } else {
      lanelets.turn_lanelets.push_back(ll);
    }
  };

  for (const auto & ll : lanelets.trajectory_lanelets) {
    if (!lock_turn_lanelets) set_turn_lanelet(ll);
    // Add connected lanelets to the list
    for (const auto & connected_ll : route_handler.getNextLanelets(ll)) {
      lanelets.connected_lanelets.push_back(connected_ll);
    }
  }
}

std::optional<std::pair<size_t, size_t>> get_overlap_index(
  const lanelet::ConstLanelet & ll, const TrajectoryPoints & trajectory_points,
  const autoware_utils::LineString2d & trajectory_ls)
{
  autoware_utils::MultiLineString2d overlap_lines;
  autoware_utils::Polygon2d ll_polygon;
  boost::geometry::convert(ll.polygon2d().basicPolygon(), ll_polygon);
  boost::geometry::correct(ll_polygon);
  boost::geometry::intersection(ll_polygon, trajectory_ls, overlap_lines);
  if (overlap_lines.empty()) return {};

  const auto overlap_line = overlap_lines.front();

  const auto nearest_idx_front = autoware::motion_utils::findNearestIndex(
    trajectory_points,
    geometry_msgs::msg::Point().set__x(overlap_line.front()[0]).set__y(overlap_line.front()[1]));
  const auto nearest_idx_back = autoware::motion_utils::findNearestIndex(
    trajectory_points,
    geometry_msgs::msg::Point().set__x(overlap_line.back()[0]).set__y(overlap_line.back()[1]));

  if (nearest_idx_back > nearest_idx_front) return {{nearest_idx_front, nearest_idx_back}};

  return {{nearest_idx_back, nearest_idx_front}};
}

bool is_crossing_lane(
  const lanelet::ConstLanelet & ll, const lanelet::ConstLanelet & ref_ll, const double angle_th)
{
  const auto ref_center_line = ref_ll.centerline2d();
  if (ref_center_line.size() < 2) return false;
  const auto & front_it = ref_center_line.begin();
  const Eigen::Vector2d ref_direction(
    (front_it->basicPoint2d() - std::next(front_it)->basicPoint2d()).normalized());

  const auto center_line = ll.centerline2d();
  if (center_line.size() < 2) return false;
  const Eigen::Vector2d direction(
    (center_line.front().basicPoint2d() - center_line.back().basicPoint2d()).normalized());

  const auto threshold = std::abs(cos(angle_th));
  return std::abs(ref_direction.dot(direction)) < threshold;
}

lanelet::ConstLanelets extend_lanelet(
  const RouteHandler & route_handler, const lanelet::ConstLanelet & ll,
  const geometry_msgs::msg::Pose & ref_point, const double distance_th)
{
  lanelet::ConstLanelets extended_lanelets{ll};
  auto current_arc_length = lanelet::utils::getArcCoordinates(extended_lanelets, ref_point).length;
  if (current_arc_length >= distance_th) return extended_lanelets;

  lanelet::ConstLanelets prev_lanelets = {ll};
  while (current_arc_length < distance_th) {
    prev_lanelets = route_handler.getPreviousLanelets(prev_lanelets.front());
    if (prev_lanelets.empty()) break;  // No more previous lanelets to extend
    extended_lanelets.push_back(prev_lanelets.front());
    current_arc_length += lanelet::utils::getLaneletLength2d(prev_lanelets.front());
  }
  std::reverse(extended_lanelets.begin(), extended_lanelets.end());
  return extended_lanelets;
}

void set_right_turn_target_lanelets(
  const EgoTrajectory & ego_traj, const RouteHandler & route_handler,
  const intersection_collision_checker_node::Params & params, const EgoLanelets & lanelets,
  TargetLaneletsMap & target_lanelets, const double time_horizon)
{
  if (lanelets.turn_lanelets.empty()) return;
  const std::string turn_direction =
    lanelets.turn_lanelets.front().attributeOr("turn_direction", "else");
  if (turn_direction != "right") return;

  const auto & p = params.icc_parameters;
  autoware_utils::LineString2d trajectory_ls;
  for (const auto & traj_p : ego_traj.front_traj) {
    trajectory_ls.emplace_back(traj_p.pose.position.x, traj_p.pose.position.y);
  }

  auto is_road = [](const lanelet::ConstLanelet & ll) {
    return ll.hasAttribute(lanelet::AttributeName::Subtype) &&
           ll.attribute(lanelet::AttributeName::Subtype).value() ==
             lanelet::AttributeValueString::Road;
  };

  auto ignore_lanelet = [&](const lanelet::ConstLanelet & ll) {
    if (is_turn_lanelet(ll)) {
      return !p.right_turn.check_turning_lanes;
    }
    if (lanelets.turn_lanelets.empty()) return false;
    if (is_crossing_lane(ll, lanelets.turn_lanelets.front(), p.right_turn.crossing_lane_angle_th)) {
      return !p.right_turn.check_crossing_lanes;
    }
    return false;
  };

  auto extend =
    [&](const lanelet::ConstLanelet & ll, const geometry_msgs::msg::Pose & overlap_point) {
      return extend_lanelet(route_handler, ll, overlap_point, p.detection_range);
    };

  const auto combined_turn_lls = lanelet::utils::combineLaneletsShape(lanelets.turn_lanelets);
  const auto lanelet_map_ptr = route_handler.getLaneletMapPtr();
  const auto candidates =
    lanelet_map_ptr->laneletLayer.search(boost::geometry::return_envelope<lanelet::BoundingBox2d>(
      combined_turn_lls.centerline2d().basicLineString()));
  for (const auto & ll : candidates) {
    const auto id = ll.id();
    if (
      !is_road(ll) || ignore_lanelet(ll) || contains_lanelet(lanelets.trajectory_lanelets, id) ||
      contains_lanelet(lanelets.connected_lanelets, id))
      continue;

    const auto overlap_index = get_overlap_index(ll, ego_traj.front_traj, trajectory_ls);
    if (!overlap_index || overlap_index->first < ego_traj.front_index) continue;
    const auto mid_idx = (overlap_index->first + overlap_index->second) / 2;
    const auto overlap_point =
      lanelet::utils::getClosestCenterPose(ll, ego_traj.front_traj[mid_idx].pose.position);
    std::pair<double, double> overlap_time;
    overlap_time.first =
      rclcpp::Duration(ego_traj.front_traj[overlap_index->first].time_from_start).seconds();
    overlap_time.second =
      rclcpp::Duration(ego_traj.back_traj[overlap_index->second].time_from_start).seconds();
    if (overlap_time.first > time_horizon) continue;
    const auto & it = target_lanelets.find(id);
    if (it != target_lanelets.end()) {
      it->second.ego_overlap_time = overlap_time;
      it->second.is_active = true;
    } else {
      target_lanelets[id] =
        TargetLanelet(id, extend(ll, overlap_point), overlap_point, overlap_time);
    }
  }
}

void set_left_turn_target_lanelets(
  const EgoTrajectory & ego_traj, const RouteHandler & route_handler,
  const intersection_collision_checker_node::Params & params, const EgoLanelets & lanelets,
  TargetLaneletsMap & target_lanelets, const double time_horizon)
{
  if (lanelets.turn_lanelets.empty()) return;
  const std::string turn_direction =
    lanelets.turn_lanelets.front().attributeOr("turn_direction", "else");
  if (turn_direction != "left") return;

  const auto last_turn_ll = lanelets.turn_lanelets.back();
  lanelet::ConstLanelet next_lanelet;
  if (!route_handler.getNextLaneletWithinRoute(last_turn_ll, &next_lanelet)) return;

  autoware_utils::LineString2d trajectory_ls;
  for (const auto & p : ego_traj.front_traj) {
    trajectory_ls.emplace_back(p.pose.position.x, p.pose.position.y);
  }

  const auto & p = params.icc_parameters;

  auto ignore_turning = [&p](const lanelet::ConstLanelet & ll) {
    if (!ll.hasAttribute("turn_direction")) return false;
    if (ll.attribute("turn_direction") == "straight") return false;
    return !p.left_turn.check_turning_lanes;
  };

  auto extend =
    [&](const lanelet::ConstLanelet & ll, const geometry_msgs::msg::Pose & overlap_point) {
      return extend_lanelet(route_handler, ll, overlap_point, p.detection_range);
    };

  const auto turn_lanelet_id = last_turn_ll.id();
  for (const auto & ll : route_handler.getPreviousLanelets(next_lanelet)) {
    const auto id = ll.id();
    if (id == turn_lanelet_id || ignore_turning(ll)) continue;
    const auto overlap_index = get_overlap_index(ll, ego_traj.front_traj, trajectory_ls);
    if (!overlap_index || overlap_index->first < ego_traj.front_index) continue;
    const auto overlap_point = lanelet::utils::getClosestCenterPose(
      ll, ego_traj.front_traj[overlap_index->first].pose.position);
    std::pair<double, double> overlap_time;
    overlap_time.first =
      rclcpp::Duration(ego_traj.front_traj[overlap_index->first].time_from_start).seconds();
    overlap_time.second =
      rclcpp::Duration(ego_traj.back_traj[overlap_index->first].time_from_start).seconds();
    if (overlap_time.first > time_horizon) continue;
    const auto & it = target_lanelets.find(id);
    if (it != target_lanelets.end()) {
      it->second.ego_overlap_time = overlap_time;
      it->second.is_active = true;
    } else {
      target_lanelets[id] =
        TargetLanelet(id, extend(ll, overlap_point), overlap_point, overlap_time);
    }
  }
}

Marker create_polygon_marker(
  const lanelet::BasicPolygon3d & polygon, const std::string & ns, const size_t id,
  const std_msgs::msg::ColorRGBA & color)
{
  visualization_msgs::msg::Marker marker = autoware_utils::create_default_marker(
    "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns, id, Marker::LINE_STRIP,
    autoware_utils::create_marker_scale(0.1, 0.1, 0.1), color);
  marker.lifetime = rclcpp::Duration::from_seconds(0.2);

  for (const auto & p : polygon) {
    marker.points.push_back(autoware_utils::create_point(p.x(), p.y(), p.z()));
  }
  if (!marker.points.empty()) {
    marker.points.push_back(marker.points.front());
  }

  return marker;
}

Marker create_point_marker(
  const geometry_msgs::msg::Point & position, const std::string & ns, const size_t id,
  const std_msgs::msg::ColorRGBA & color, const double scale = 0.3, const bool is_cube = false)
{
  const auto marker_type = is_cube ? Marker::CUBE : Marker::SPHERE;
  Marker marker = autoware_utils::create_default_marker(
    "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns, id, marker_type,
    autoware_utils::create_marker_scale(scale, scale, scale), color);
  marker.lifetime = rclcpp::Duration::from_seconds(0.2);
  marker.pose.position = position;

  return marker;
}

Marker create_text_marker(
  const std::string & text, const geometry_msgs::msg::Pose & pose, const std::string & ns,
  const size_t id, const std_msgs::msg::ColorRGBA & color)
{
  Marker marker = autoware_utils::create_default_marker(
    "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns, id, Marker::TEXT_VIEW_FACING,
    autoware_utils::create_marker_scale(0.5, 0.5, 0.5), color);
  marker.lifetime = rclcpp::Duration::from_seconds(0.2);
  marker.pose = pose;
  marker.text = text;
  return marker;
}

MarkerArray get_lanelets_marker_array(const DebugData & debug_data)
{
  MarkerArray marker_array;

  const auto red = autoware_utils::create_marker_color(1.0, 0.0, 0.0, 0.9);
  const auto green = autoware_utils::create_marker_color(0.0, 1.0, 0.0, 0.9);
  const auto blue = autoware_utils::create_marker_color(0.0, 0.0, 1.0, 0.9);
  const auto white = autoware_utils::create_marker_color(1.0, 1.0, 1.0, 0.7);

  {  // trajectory lanelets
    for (const auto & ll : debug_data.ego_lanelets.trajectory_lanelets) {
      marker_array.markers.push_back(create_polygon_marker(
        ll.polygon3d().basicPolygon(), "ICC_trajectory_lanelets", ll.id(), green));
      if (ll.hasAttribute("turn_direction") && ll.attribute("turn_direction") != "straight") {
        marker_array.markers.push_back(create_polygon_marker(
          ll.polygon3d().basicPolygon(), "ICC_turn_direction_lanelets", ll.id(), red));
      }
    }
  }

  auto add_text_marker = [&](const TargetLanelet & target_ll, const std::string & ns) {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << "Lane ID:" << target_ll.id;
    ss << "\nTimeToReach:" << target_ll.ego_overlap_time.first << "[s]";
    ss << "\nTimeToLeave:" << target_ll.ego_overlap_time.second << "[s]";
    auto pose = target_ll.overlap_point;
    pose.position.z += 1.0;
    marker_array.markers.push_back(create_text_marker(ss.str(), pose, ns, target_ll.id, white));
  };

  {  // target lanelets
    lanelet::BasicPolygons2d ll_polygons;
    for (const auto & target_ll : debug_data.target_lanelets) {
      const auto combine_ll = lanelet::utils::combineLaneletsShape(target_ll.lanelets);
      marker_array.markers.push_back(create_polygon_marker(
        combine_ll.polygon3d().basicPolygon(), "ICC_target_lanelets", target_ll.id, blue));
      marker_array.markers.push_back(create_point_marker(
        target_ll.overlap_point.position, "ICC_target_lanelets_op", target_ll.id, blue));
      add_text_marker(target_ll, "ICC_target_lanelets_text");
    }
  }

  return marker_array;
}

MarkerArray get_objects_marker_array(const DebugData & debug_data)
{
  MarkerArray marker_array;

  const auto red = autoware_utils::create_marker_color(1.0, 0.0, 0.0, 0.9);
  const auto green = autoware_utils::create_marker_color(0.0, 1.0, 0.0, 0.9);
  const auto white = autoware_utils::create_marker_color(1.0, 1.0, 1.0, 0.7);

  auto add_text_marker = [&](const PCDObject & object) {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << "TrackDuration:" << object.track_duration << "[s]\n";
    ss << "MovingTime:" << object.moving_time << "[s]\n";
    ss << "DistToOverlap(RAW):" << object.distance_to_overlap << "[m]\n";
    ss << "DistToOverlap(w/DC):" << object.delay_compensated_distance_to_overlap << "[m]\n";
    ss << "Velocity:" << object.velocity << "[m/s]\n";
    ss << "TTC:" << object.ttc << "[s]";
    marker_array.markers.push_back(create_text_marker(
      ss.str(), object.pose, "ICC_pcd_objects_text", object.overlap_lanelet_id, white));
  };

  auto add_line_segment_marker = [&](const PCDObject & object) {
    Marker marker = autoware_utils::create_default_marker(
      "map", rclcpp::Clock{RCL_ROS_TIME}.now(), "ICC_pcd_objects_cp", object.overlap_lanelet_id,
      Marker::LINE_LIST, autoware_utils::create_marker_scale(0.1, 0.1, 0.1), red);
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    marker.points.push_back(object.pose.position);
    marker.points.push_back(object.overlap_point);
    marker_array.markers.push_back(marker);
  };

  for (const auto & pcd_obj : debug_data.pcd_objects) {
    if (!pcd_obj.is_reliable) continue;
    if (pcd_obj.is_safe) {
      marker_array.markers.push_back(create_point_marker(
        pcd_obj.pose.position, "ICC_pcd_objects", pcd_obj.overlap_lanelet_id, green));
    } else {
      marker_array.markers.push_back(create_point_marker(
        pcd_obj.pose.position, "ICC_pcd_objects", pcd_obj.overlap_lanelet_id, red, 0.5, true));
      add_line_segment_marker(pcd_obj);
    }
    add_text_marker(pcd_obj);
  }

  return marker_array;
}

}  // namespace autoware::planning_validator::collision_checker_utils
