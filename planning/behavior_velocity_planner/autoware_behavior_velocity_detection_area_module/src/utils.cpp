// Copyright 2024 Tier IV, Inc.
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

#include "utils.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/arc_lane_util.hpp>
#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware/object_recognition_utils/object_classification.hpp>
#include <autoware/trajectory/utils/crossed.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/geometry/Polygon.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace
{
// calc smallest enclosing circle with average O(N) algorithm
// reference:
// https://erickimphotography.com/blog/wp-content/uploads/2018/09/Computational-Geometry-Algorithms-and-Applications-3rd-Ed.pdf
std::pair<lanelet::BasicPoint2d, double> get_smallest_enclosing_circle(
  const lanelet::ConstPolygon2d & poly)
{
  // The `eps` is used to avoid precision bugs in circle inclusion checks.
  // If the value of `eps` is too small, this function doesn't work well. More than 1e-10 is
  // recommended.
  const double eps = 1e-5;
  lanelet::BasicPoint2d center(0.0, 0.0);
  double radius_squared = 0.0;

  auto cross = [](const lanelet::BasicPoint2d & p1, const lanelet::BasicPoint2d & p2) -> double {
    return p1.x() * p2.y() - p1.y() * p2.x();
  };

  auto make_circle_3 = [&](
                         const lanelet::BasicPoint2d & p1, const lanelet::BasicPoint2d & p2,
                         const lanelet::BasicPoint2d & p3) -> void {
    // reference for circumcenter vector https://en.wikipedia.org/wiki/Circumscribed_circle
    const double a = (p2 - p3).squaredNorm();
    const double b = (p3 - p1).squaredNorm();
    const double c = (p1 - p2).squaredNorm();
    const double s = cross(p2 - p1, p3 - p1);
    if (std::abs(s) < eps) return;
    center = (a * (b + c - a) * p1 + b * (c + a - b) * p2 + c * (a + b - c) * p3) / (4 * s * s);
    radius_squared = (center - p1).squaredNorm() + eps;
  };

  auto make_circle_2 =
    [&](const lanelet::BasicPoint2d & p1, const lanelet::BasicPoint2d & p2) -> void {
    center = (p1 + p2) * 0.5;
    radius_squared = (center - p1).squaredNorm() + eps;
  };

  auto in_circle = [&](const lanelet::BasicPoint2d & p) -> bool {
    return (center - p).squaredNorm() <= radius_squared;
  };

  // mini disc
  for (size_t i = 1; i < poly.size(); i++) {
    const auto p1 = poly[i].basicPoint2d();
    if (in_circle(p1)) continue;

    // mini disc with point
    const auto p0 = poly[0].basicPoint2d();
    make_circle_2(p0, p1);
    for (size_t j = 0; j < i; j++) {
      const auto p2 = poly[j].basicPoint2d();
      if (in_circle(p2)) continue;

      // mini disc with two points
      make_circle_2(p1, p2);
      for (size_t k = 0; k < j; k++) {
        const auto p3 = poly[k].basicPoint2d();
        if (in_circle(p3)) continue;

        // mini disc with tree points
        make_circle_3(p1, p2, p3);
      }
    }
  }

  return std::make_pair(center, radius_squared);
}
}  // namespace

namespace autoware::behavior_velocity_planner::detection_area
{
autoware_utils::LineString2d get_stop_line(
  const lanelet::autoware::DetectionArea & detection_area,
  const std::vector<geometry_msgs::msg::Point> & left_bound,
  const std::vector<geometry_msgs::msg::Point> & right_bound)
{
  const auto stop_line = detection_area.stopLine();
  return planning_utils::extendSegmentToBounds(
    lanelet::utils::to2D(stop_line).basicLineString(), left_bound, right_bound);
}

std::optional<double> get_stop_point(
  const Trajectory & path, const autoware_utils::LineString2d & stop_line, const double margin,
  const double vehicle_offset, const lanelet::Ids & lane_ids)
{
  const auto collision_points = experimental::trajectory::crossed_with_constraint(
    path, stop_line, [&](const autoware_internal_planning_msgs::msg::PathPointWithLaneId & p) {
      return lane_ids.empty() ||
             std::any_of(p.lane_ids.begin(), p.lane_ids.end(), [&](const lanelet::Id id) {
               return std::find(lane_ids.begin(), lane_ids.end(), id) != lane_ids.end();
             });
    });

  if (collision_points.empty()) {
    return std::nullopt;
  }

  return collision_points.front() - margin - vehicle_offset;
}

autoware_utils::LineString2d get_stop_line_geometry2d(
  const lanelet::autoware::DetectionArea & detection_area,
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path)
{
  const auto stop_line = detection_area.stopLine();
  return planning_utils::extendSegmentToBounds(
    lanelet::utils::to2D(stop_line).basicLineString(), path.left_bound, path.right_bound);
}

std::vector<geometry_msgs::msg::Point> get_obstacle_points(
  const lanelet::ConstPolygons3d & detection_areas, const pcl::PointCloud<pcl::PointXYZ> & points)
{
  std::vector<geometry_msgs::msg::Point> obstacle_points;
  for (const auto & detection_area : detection_areas) {
    const auto poly = lanelet::utils::to2D(detection_area);
    const auto circle = get_smallest_enclosing_circle(poly);
    for (const auto p : points) {
      const double squared_dist = (circle.first.x() - p.x) * (circle.first.x() - p.x) +
                                  (circle.first.y() - p.y) * (circle.first.y() - p.y);
      if (squared_dist <= circle.second) {
        const lanelet::BasicPoint2d point(p.x, p.y);
        if (lanelet::geometry::within(point, poly.basicPolygon())) {
          obstacle_points.push_back(autoware_utils::create_point(p.x, p.y, p.z));
          // get all obstacle point becomes high computation cost so skip if any point is found
          break;
        }
      }
    }
  }
  return obstacle_points;
}

bool can_clear_stop_state(
  const std::shared_ptr<const rclcpp::Time> & last_obstacle_found_time, const rclcpp::Time & now,
  const double state_clear_time)
{
  // vehicle can clear stop state if the obstacle has never appeared in detection area
  if (!last_obstacle_found_time) {
    return true;
  }

  // vehicle can clear stop state if the certain time has passed since the obstacle disappeared
  const auto elapsed_time = now - *last_obstacle_found_time;
  if (elapsed_time.seconds() >= state_clear_time) {
    return true;
  }

  // rollback in simulation mode
  if (elapsed_time.seconds() < 0.0) {
    return true;
  }

  return false;
}

bool has_enough_braking_distance(
  const double self_s, const double line_point_s, const double pass_judge_line_distance,
  const double current_velocity)
{
  // prevent from being judged as not having enough distance when the current velocity is zero
  // and the vehicle crosses the stop line
  if (current_velocity < 1e-3) {
    return true;
  }

  return line_point_s - self_s > pass_judge_line_distance;
}

bool has_enough_braking_distance(
  const geometry_msgs::msg::Pose & self_pose, const geometry_msgs::msg::Pose & line_pose,
  const double pass_judge_line_distance, const double current_velocity)
{
  // prevent from being judged as not having enough distance when the current velocity is zero
  // and the vehicle crosses the stop line
  if (current_velocity < 1e-3) {
    return true;
  }

  return arc_lane_utils::calcSignedDistance(self_pose, line_pose.position) >
         pass_judge_line_distance;
}

double feasible_stop_distance_by_max_acceleration(
  const double current_velocity, const double max_acceleration)
{
  return current_velocity * current_velocity / (2.0 * max_acceleration);
}

std::optional<autoware_perception_msgs::msg::PredictedObject> get_detected_object(
  const lanelet::ConstPolygons3d & detection_areas,
  const autoware_perception_msgs::msg::PredictedObjects & predicted_objects,
  const DetectionAreaModule::PlannerParam::TargetFiltering & target_filtering)
{
  for (const auto & object : predicted_objects.objects) {
    if (!is_target_object(object.classification, target_filtering)) {
      continue;
    }

    // Get object position
    const auto & position = object.kinematics.initial_pose_with_covariance.pose.position;

    // Check if the object is within any detection area
    for (const auto & detection_area : detection_areas) {
      const lanelet::BasicPoint2d obj_point(position.x, position.y);
      const auto detection_area_2d = lanelet::utils::to2D(detection_area);
      if (lanelet::geometry::within(obj_point, detection_area_2d.basicPolygon())) {
        return object;  // Return the detected object
      }
    }
  }

  return std::nullopt;  // No object detected
}

bool is_target_object(
  const std::vector<autoware_perception_msgs::msg::ObjectClassification> & classifications,
  const DetectionAreaModule::PlannerParam::TargetFiltering & target_filtering)
{
  using autoware_perception_msgs::msg::ObjectClassification;

  // Check all classifications (use the one with the highest probability)
  if (classifications.empty()) {
    return false;
  }

  const auto label = autoware::object_recognition_utils::getHighestProbLabel(classifications);

  if (label == ObjectClassification::UNKNOWN && target_filtering.unknown) {
    return true;
  }
  if (label == ObjectClassification::CAR && target_filtering.car) {
    return true;
  }
  if (label == ObjectClassification::TRUCK && target_filtering.truck) {
    return true;
  }
  if (label == ObjectClassification::BUS && target_filtering.bus) {
    return true;
  }
  if (label == ObjectClassification::TRAILER && target_filtering.trailer) {
    return true;
  }
  if (label == ObjectClassification::MOTORCYCLE && target_filtering.motorcycle) {
    return true;
  }
  if (label == ObjectClassification::BICYCLE && target_filtering.bicycle) {
    return true;
  }
  if (label == ObjectClassification::PEDESTRIAN && target_filtering.pedestrian) {
    return true;
  }
  if (label == ObjectClassification::ANIMAL && target_filtering.animal) {
    return true;
  }
  if (label == ObjectClassification::HAZARD && target_filtering.hazard) {
    return true;
  }
  if (label == ObjectClassification::OVER_DRIVABLE && target_filtering.over_drivable) {
    return true;
  }
  if (label == ObjectClassification::UNDER_DRIVABLE && target_filtering.under_drivable) {
    return true;
  }

  return false;
}

std::string object_label_to_string(const uint8_t label)
{
  using autoware_perception_msgs::msg::ObjectClassification;

  switch (label) {
    case ObjectClassification::UNKNOWN:
      return "unknown";
    case ObjectClassification::CAR:
      return "car";
    case ObjectClassification::TRUCK:
      return "truck";
    case ObjectClassification::BUS:
      return "bus";
    case ObjectClassification::TRAILER:
      return "trailer";
    case ObjectClassification::MOTORCYCLE:
      return "motorcycle";
    case ObjectClassification::BICYCLE:
      return "bicycle";
    case ObjectClassification::PEDESTRIAN:
      return "pedestrian";
    case ObjectClassification::ANIMAL:
      return "animal";
    case ObjectClassification::HAZARD:
      return "hazard";
    case ObjectClassification::OVER_DRIVABLE:
      return "over_drivable";
    case ObjectClassification::UNDER_DRIVABLE:
      return "under_drivable";
    default:
      return "unrecognized";
  }
}

}  // namespace autoware::behavior_velocity_planner::detection_area
