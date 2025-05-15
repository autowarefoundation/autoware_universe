// Copyright 2023 TIER IV, Inc.
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

#include "autoware/path_optimizer/debug_marker.hpp"

#include "autoware/path_optimizer/mpt_optimizer.hpp"

#include "visualization_msgs/msg/marker_array.hpp"

#include <string>
#include <vector>

namespace autoware::path_optimizer
{
using autoware_utils::append_marker_array;
using autoware_utils::create_default_marker;
using autoware_utils::create_marker_color;
using autoware_utils::create_marker_scale;

namespace
{
MarkerArray getFootprintsMarkerArray(
  const std::vector<TrajectoryPoint> & mpt_traj,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const size_t sampling_num)
{
  auto marker = create_default_marker(
    "map", rclcpp::Clock().now(), "mpt_footprints", 0, Marker::LINE_STRIP,
    create_marker_scale(0.05, 0.0, 0.0), create_marker_color(0.99, 0.99, 0.2, 0.99));
  marker.lifetime = rclcpp::Duration::from_seconds(1.5);

  MarkerArray marker_array;
  for (size_t i = 0; i < mpt_traj.size(); ++i) {
    if (i % sampling_num != 0) {
      continue;
    }

    marker.id = i;
    marker.points.clear();

    const auto & traj_point = mpt_traj.at(i);

    const double base_to_right = (vehicle_info.wheel_tread_m / 2.0) + vehicle_info.right_overhang_m;
    const double base_to_left = (vehicle_info.wheel_tread_m / 2.0) + vehicle_info.left_overhang_m;
    const double base_to_front = vehicle_info.vehicle_length_m - vehicle_info.rear_overhang_m;
    const double base_to_rear = vehicle_info.rear_overhang_m;

    marker.points.push_back(
      autoware_utils::calc_offset_pose(traj_point.pose, base_to_front, base_to_left, 0.0).position);
    marker.points.push_back(
      autoware_utils::calc_offset_pose(traj_point.pose, base_to_front, -base_to_right, 0.0)
        .position);
    marker.points.push_back(
      autoware_utils::calc_offset_pose(traj_point.pose, -base_to_rear, -base_to_right, 0.0)
        .position);
    marker.points.push_back(
      autoware_utils::calc_offset_pose(traj_point.pose, -base_to_rear, base_to_left, 0.0).position);
    marker.points.push_back(marker.points.front());

    marker_array.markers.push_back(marker);
  }
  return marker_array;
}

MarkerArray getBoundsWidthMarkerArray(
  const std::vector<ReferencePoint> & ref_points,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const size_t sampling_num)
{
  const auto current_time = rclcpp::Clock().now();
  MarkerArray marker_array;

  if (ref_points.empty()) return marker_array;

  // create lower bound marker
  auto lb_marker = create_default_marker(
    "map", rclcpp::Clock().now(), "", 0, Marker::LINE_LIST, create_marker_scale(0.05, 0.0, 0.0),
    create_marker_color(0.5, 0.99, 0.2, 0.8));
  lb_marker.lifetime = rclcpp::Duration::from_seconds(1.5);

  // create upper bound marker
  auto ub_marker = create_default_marker(
    "map", rclcpp::Clock().now(), "", 1, Marker::LINE_LIST, create_marker_scale(0.05, 0.0, 0.0),
    create_marker_color(0.99, 0.5, 0.2, 0.8));
  ub_marker.lifetime = rclcpp::Duration::from_seconds(1.5);

  for (size_t bound_idx = 0; bound_idx < ref_points.at(0).bounds_on_constraints.size();
       ++bound_idx) {
    const std::string ns = "bounds" + std::to_string(bound_idx);

    {  // lower bound
      lb_marker.points.clear();
      lb_marker.ns = ns;

      for (size_t i = 0; i < ref_points.size(); i++) {
        if (i % sampling_num != 0) {
          continue;
        }

        const geometry_msgs::msg::Pose & pose = ref_points.at(i).pose_on_constraints.at(bound_idx);
        const double base_to_right =
          (vehicle_info.wheel_tread_m / 2.0) + vehicle_info.right_overhang_m;
        const double lb_y =
          ref_points.at(i).bounds_on_constraints.at(bound_idx).lower_bound - base_to_right;
        const auto lb = autoware_utils::calc_offset_pose(pose, 0.0, lb_y, 0.0).position;

        lb_marker.points.push_back(pose.position);
        lb_marker.points.push_back(lb);
      }
      marker_array.markers.push_back(lb_marker);
    }

    {  // upper bound
      ub_marker.points.clear();
      ub_marker.ns = ns;

      for (size_t i = 0; i < ref_points.size(); i++) {
        if (i % sampling_num != 0) {
          continue;
        }

        const geometry_msgs::msg::Pose & pose = ref_points.at(i).pose_on_constraints.at(bound_idx);
        const double base_to_left =
          (vehicle_info.wheel_tread_m / 2.0) + vehicle_info.left_overhang_m;
        const double ub_y =
          ref_points.at(i).bounds_on_constraints.at(bound_idx).upper_bound + base_to_left;
        const auto ub = autoware_utils::calc_offset_pose(pose, 0.0, ub_y, 0.0).position;

        ub_marker.points.push_back(pose.position);
        ub_marker.points.push_back(ub);
      }
      marker_array.markers.push_back(ub_marker);
    }
  }

  return marker_array;
}

MarkerArray getBoundsLineMarkerArray(
  const std::vector<ReferencePoint> & ref_points,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info)
{
  MarkerArray marker_array;

  if (ref_points.empty()) return marker_array;

  auto ub_marker = create_default_marker(
    "map", rclcpp::Clock().now(), "left_bounds", 0, Marker::LINE_STRIP,
    create_marker_scale(0.05, 0.0, 0.0), create_marker_color(0.0, 1.0, 1.0, 0.8));
  ub_marker.lifetime = rclcpp::Duration::from_seconds(1.5);

  auto lb_marker = create_default_marker(
    "map", rclcpp::Clock().now(), "right_bounds", 0, Marker::LINE_STRIP,
    create_marker_scale(0.05, 0.0, 0.0), create_marker_color(0.0, 1.0, 1.0, 0.8));
  lb_marker.lifetime = rclcpp::Duration::from_seconds(1.5);

  for (size_t i = 0; i < ref_points.size(); i++) {
    const geometry_msgs::msg::Pose & pose = ref_points.at(i).pose;
    const double base_to_right = (vehicle_info.wheel_tread_m / 2.0) + vehicle_info.right_overhang_m;
    const double base_to_left = (vehicle_info.wheel_tread_m / 2.0) + vehicle_info.left_overhang_m;
    const double ub_y = ref_points.at(i).bounds.upper_bound + base_to_left;
    const auto ub = autoware_utils::calc_offset_pose(pose, 0.0, ub_y, 0.0).position;
    ub_marker.points.push_back(ub);

    const double lb_y = ref_points.at(i).bounds.lower_bound - base_to_right;
    const auto lb = autoware_utils::calc_offset_pose(pose, 0.0, lb_y, 0.0).position;
    lb_marker.points.push_back(lb);
  }
  marker_array.markers.push_back(ub_marker);
  marker_array.markers.push_back(lb_marker);

  return marker_array;
}

MarkerArray getVehicleCircleLinesMarkerArray(
  const std::vector<ReferencePoint> & ref_points,
  const std::vector<double> & vehicle_circle_longitudinal_offsets,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const size_t sampling_num,
  const std::string & ns)
{
  const auto current_time = rclcpp::Clock().now();
  MarkerArray msg;

  for (size_t i = 0; i < ref_points.size(); ++i) {
    if (i % sampling_num != 0) {
      continue;
    }
    const auto & ref_point = ref_points.at(i);

    auto marker = create_default_marker(
      "map", rclcpp::Clock().now(), ns, i, Marker::LINE_LIST, create_marker_scale(0.1, 0, 0),
      create_marker_color(0.99, 0.99, 0.2, 0.25));
    marker.lifetime = rclcpp::Duration::from_seconds(1.5);

    const double lat_dev = ref_point.optimized_kinematic_state.lat;
    const double yaw_dev = ref_point.optimized_kinematic_state.yaw;

    // apply lateral and yaw deviation
    auto pose_with_deviation = autoware_utils::calc_offset_pose(ref_point.pose, 0.0, lat_dev, 0.0);
    pose_with_deviation.orientation =
      autoware_utils::create_quaternion_from_yaw(ref_point.getYaw() + yaw_dev);

    for (const double d : vehicle_circle_longitudinal_offsets) {
      // apply longitudinal offset
      auto base_pose = autoware_utils::calc_offset_pose(pose_with_deviation, d, 0.0, 0.0);
      base_pose.orientation =
        autoware_utils::create_quaternion_from_yaw(ref_point.getYaw() + ref_point.alpha);
      const double base_to_right =
        (vehicle_info.wheel_tread_m / 2.0) + vehicle_info.right_overhang_m;
      const double base_to_left = (vehicle_info.wheel_tread_m / 2.0) + vehicle_info.left_overhang_m;
      const auto ub = autoware_utils::calc_offset_pose(base_pose, 0.0, base_to_left, 0.0).position;
      const auto lb =
        autoware_utils::calc_offset_pose(base_pose, 0.0, -base_to_right, 0.0).position;

      marker.points.push_back(ub);
      marker.points.push_back(lb);
    }
    msg.markers.push_back(marker);
  }

  return msg;
}

MarkerArray getCurrentVehicleCirclesMarkerArray(
  const geometry_msgs::msg::Pose & ego_pose,
  const std::vector<double> & vehicle_circle_longitudinal_offsets,
  const std::vector<double> & vehicle_circle_radiuses, const std::string & ns, const double r,
  const double g, const double b)
{
  MarkerArray msg;

  size_t id = 0;
  for (size_t v_idx = 0; v_idx < vehicle_circle_longitudinal_offsets.size(); ++v_idx) {
    const double offset = vehicle_circle_longitudinal_offsets.at(v_idx);

    auto marker = create_default_marker(
      "map", rclcpp::Clock().now(), ns, id, Marker::LINE_STRIP, create_marker_scale(0.05, 0.0, 0.0),
      create_marker_color(r, g, b, 0.8));
    marker.lifetime = rclcpp::Duration::from_seconds(1.5);
    marker.pose = autoware_utils::calc_offset_pose(ego_pose, offset, 0.0, 0.0);

    constexpr size_t circle_dividing_num = 16;
    for (size_t e_idx = 0; e_idx < circle_dividing_num + 1; ++e_idx) {
      geometry_msgs::msg::Point edge_pos;

      const double edge_angle =
        static_cast<double>(e_idx) / static_cast<double>(circle_dividing_num) * 2.0 * M_PI;
      edge_pos.x = vehicle_circle_radiuses.at(v_idx) * std::cos(edge_angle);
      edge_pos.y = vehicle_circle_radiuses.at(v_idx) * std::sin(edge_angle);

      marker.points.push_back(edge_pos);
    }

    msg.markers.push_back(marker);
    ++id;
  }
  return msg;
}

MarkerArray getVehicleCirclesMarkerArray(
  const std::vector<TrajectoryPoint> & mpt_traj_points,
  const std::vector<double> & vehicle_circle_longitudinal_offsets,
  const std::vector<double> & vehicle_circle_radiuses, const size_t sampling_num,
  const std::string & ns, const double r, const double g, const double b)
{
  MarkerArray msg;

  size_t id = 0;
  for (size_t i = 0; i < mpt_traj_points.size(); ++i) {
    if (i % sampling_num != 0) {
      continue;
    }
    const auto & mpt_traj_point = mpt_traj_points.at(i);

    for (size_t v_idx = 0; v_idx < vehicle_circle_longitudinal_offsets.size(); ++v_idx) {
      const double offset = vehicle_circle_longitudinal_offsets.at(v_idx);

      auto marker = create_default_marker(
        "map", rclcpp::Clock().now(), ns, id, Marker::LINE_STRIP,
        create_marker_scale(0.05, 0.0, 0.0), create_marker_color(r, g, b, 0.8));
      marker.lifetime = rclcpp::Duration::from_seconds(1.5);
      marker.pose = autoware_utils::calc_offset_pose(mpt_traj_point.pose, offset, 0.0, 0.0);

      constexpr size_t circle_dividing_num = 16;
      for (size_t e_idx = 0; e_idx < circle_dividing_num + 1; ++e_idx) {
        geometry_msgs::msg::Point edge_pos;

        const double edge_angle =
          static_cast<double>(e_idx) / static_cast<double>(circle_dividing_num) * 2.0 * M_PI;
        edge_pos.x = vehicle_circle_radiuses.at(v_idx) * std::cos(edge_angle);
        edge_pos.y = vehicle_circle_radiuses.at(v_idx) * std::sin(edge_angle);

        marker.points.push_back(edge_pos);
      }

      msg.markers.push_back(marker);
      ++id;
    }
  }
  return msg;
}

visualization_msgs::msg::MarkerArray getPointsTextMarkerArray(
  const std::vector<ReferencePoint> & ref_points)
{
  if (ref_points.empty()) {
    return visualization_msgs::msg::MarkerArray{};
  }

  auto marker = create_default_marker(
    "map", rclcpp::Clock().now(), "text", 0, visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
    create_marker_scale(0.0, 0.0, 0.15), create_marker_color(1.0, 1.0, 0.0, 0.99));
  marker.lifetime = rclcpp::Duration::from_seconds(1.5);

  visualization_msgs::msg::MarkerArray msg;
  for (size_t i = 0; i < ref_points.size(); i++) {
    marker.id = i;
    // marker.text = std::to_string(tf2::getYaw(ref_points[i].pose.orientation)) + "\n" +
    // std::to_string(ref_points[i].delta_arc_length); marker.text =
    // std::to_string(ref_points[i].alpha) + "\n" + std::to_string(ref_points[i].beta); marker.text
    // marker.text = std::to_string(ref_points.at(i).curvature);
    marker.text = std::to_string(ref_points.at(i).curvature) + " \n" +
                  std::to_string(ref_points.at(i).optimized_input);
    marker.pose.position = ref_points.at(i).pose.position;
    msg.markers.push_back(marker);
  }

  return msg;
}

visualization_msgs::msg::MarkerArray getFootprintByDrivableAreaMarkerArray(
  const geometry_msgs::msg::Pose & stop_pose,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const std::string & ns,
  const double r, const double g, const double b)
{
  visualization_msgs::msg::MarkerArray msg;

  auto marker = create_default_marker(
    "map", rclcpp::Clock().now(), ns, 1, visualization_msgs::msg::Marker::LINE_STRIP,
    create_marker_scale(0.05, 0.0, 0.0), create_marker_color(r, g, b, 1.0));
  marker.lifetime = rclcpp::Duration::from_seconds(1.5);

  const double base_to_right = (vehicle_info.wheel_tread_m / 2.0) + vehicle_info.right_overhang_m;
  const double base_to_left = (vehicle_info.wheel_tread_m / 2.0) + vehicle_info.left_overhang_m;
  const double base_to_front = vehicle_info.vehicle_length_m - vehicle_info.rear_overhang_m;
  const double base_to_rear = vehicle_info.rear_overhang_m;

  marker.points.push_back(
    autoware_utils::calc_offset_pose(stop_pose, base_to_front, base_to_left, 0.0).position);
  marker.points.push_back(
    autoware_utils::calc_offset_pose(stop_pose, base_to_front, -base_to_right, 0.0).position);
  marker.points.push_back(
    autoware_utils::calc_offset_pose(stop_pose, -base_to_rear, -base_to_right, 0.0).position);
  marker.points.push_back(
    autoware_utils::calc_offset_pose(stop_pose, -base_to_rear, base_to_left, 0.0).position);
  marker.points.push_back(marker.points.front());

  msg.markers.push_back(marker);

  return msg;
}

}  // namespace

MarkerArray getDebugMarker(
  const DebugData & debug_data, const std::vector<TrajectoryPoint> & optimized_points,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const bool publish_extra_marker)
{
  MarkerArray marker_array;

  // bounds line
  append_marker_array(getBoundsLineMarkerArray(debug_data.ref_points, vehicle_info), &marker_array);

  // bounds width
  append_marker_array(
    getBoundsWidthMarkerArray(
      debug_data.ref_points, vehicle_info, debug_data.mpt_visualize_sampling_num),
    &marker_array);

  // current vehicle circles
  append_marker_array(
    getCurrentVehicleCirclesMarkerArray(
      debug_data.ego_pose, debug_data.vehicle_circle_longitudinal_offsets,
      debug_data.vehicle_circle_radiuses, "current_vehicle_circles", 1.0, 0.3, 0.3),
    &marker_array);

  // NOTE: Default debug marker is limited for less calculation time
  //       Circles visualization is comparatively heavy.
  if (publish_extra_marker) {
    // vehicle circles
    append_marker_array(
      getVehicleCirclesMarkerArray(
        optimized_points, debug_data.vehicle_circle_longitudinal_offsets,
        debug_data.vehicle_circle_radiuses, debug_data.mpt_visualize_sampling_num,
        "vehicle_circles", 1.0, 0.3, 0.3),
      &marker_array);

    // mpt footprints
    append_marker_array(
      getFootprintsMarkerArray(
        optimized_points, vehicle_info, debug_data.mpt_visualize_sampling_num),
      &marker_array);

    // vehicle circle line
    append_marker_array(
      getVehicleCircleLinesMarkerArray(
        debug_data.ref_points, debug_data.vehicle_circle_longitudinal_offsets, vehicle_info,
        debug_data.mpt_visualize_sampling_num, "vehicle_circle_lines"),
      &marker_array);

    // footprint by drivable area
    if (debug_data.stop_pose_by_drivable_area) {
      append_marker_array(
        getFootprintByDrivableAreaMarkerArray(
          *debug_data.stop_pose_by_drivable_area, vehicle_info, "footprint_by_drivable_area", 1.0,
          0.0, 0.0),
        &marker_array);
    }

    // debug text
    append_marker_array(getPointsTextMarkerArray(debug_data.ref_points), &marker_array);
  }

  return marker_array;
}
}  // namespace autoware::path_optimizer
