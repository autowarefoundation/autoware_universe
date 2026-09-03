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

#ifndef AUTOWARE__TRAJECTORY_VALIDATOR__FILTERS__TRAFFIC_RULE__CROSSWALK_FILTER_HPP_
#define AUTOWARE__TRAJECTORY_VALIDATOR__FILTERS__TRAFFIC_RULE__CROSSWALK_FILTER_HPP_

#include "autoware/trajectory_validator/validator_interface.hpp"

#include <autoware_lanelet2_extension/regulatory_elements/Forward.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/crosswalk.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>

#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace autoware::trajectory_validator::plugin::traffic_rule
{
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_utils_uuid::to_hex_string;

struct CrosswalkOnTrajectory
{
  lanelet::CrosswalkConstPtr crosswalk;
  double arc_length_to_stop_line_m{0.0};
  lanelet::BasicLineString2d stop_line;

  CrosswalkOnTrajectory(
    lanelet::CrosswalkConstPtr crosswalk, double arc_length_to_stop_line_m,
    lanelet::BasicLineString2d stop_line)
  : crosswalk(crosswalk), arc_length_to_stop_line_m(arc_length_to_stop_line_m), stop_line(stop_line)
  {
  }
};

struct TargetCrosswalk
{
  CrosswalkOnTrajectory crosswalk_info;
  lanelet::BasicPolygon2d crosswalk_polygon;
  lanelet::BasicPolygons2d detection_areas;
  bool is_crossing{false};

  TargetCrosswalk(
    const CrosswalkOnTrajectory & crosswalk_info, const lanelet::BasicPolygon2d & crosswalk_polygon,
    const lanelet::BasicPolygons2d & detection_areas, const bool is_crossing)
  : crosswalk_info(crosswalk_info),
    crosswalk_polygon(crosswalk_polygon),
    detection_areas(detection_areas),
    is_crossing(is_crossing)
  {
  }
};
using TargetCrosswalks = std::vector<TargetCrosswalk>;

struct TargetObject
{
  PredictedObject object;
  rclcpp::Time first_seen_time;
  rclcpp::Time last_seen_time;
  ObjectClassification::_label_type type;
  bool ignore{false};

  TargetObject(
    const PredictedObject & object, const rclcpp::Time & first_seen_time,
    const rclcpp::Time & last_seen_time)
  : object(object),
    first_seen_time(first_seen_time),
    last_seen_time(last_seen_time),
    type(
      object.classification.empty() ? ObjectClassification::UNKNOWN
                                    : object.classification.front().label)
  {
  }

  bool operator==(const TargetObject & other) const
  {
    return to_hex_string(object.object_id) == to_hex_string(other.object.object_id);
  }

  bool operator==(const PredictedObject & other) const
  {
    return to_hex_string(object.object_id) == to_hex_string(other.object_id);
  }

  [[nodiscard]] bool matches(const TargetObject & other, const double distance_th) const
  {
    if (*this == other) return true;
    if (type != other.type) return false;
    const auto distance = autoware_utils_geometry::calc_distance2d(
      object.kinematics.initial_pose_with_covariance.pose.position,
      other.object.kinematics.initial_pose_with_covariance.pose.position);
    return distance < distance_th;
  }

  [[nodiscard]] bool matches(const PredictedObject & other, const double distance_th = 1e-3) const
  {
    if (*this == other) return true;
    const auto other_type = other.classification.empty() ? ObjectClassification::UNKNOWN
                                                         : other.classification.front().label;
    if (type != other_type) return false;
    const auto distance = autoware_utils_geometry::calc_distance2d(
      object.kinematics.initial_pose_with_covariance.pose.position,
      other.kinematics.initial_pose_with_covariance.pose.position);
    return distance < distance_th;
  }
};
using TargetObjects = std::vector<TargetObject>;

class CrosswalkFilter : public ValidatorInterface
{
public:
  CrosswalkFilter();

  result_t is_feasible(
    const CandidateTrajectory & candidate_trajectory, const FilterContext & context) final;

  void update_parameters(const validator::Params & params) final;

  void set_vehicle_info(const VehicleInfo & vehicle_info) final;

private:
  validator::Params::Crosswalk params_;
  std::unordered_map<lanelet::Id, TargetObjects> crosswalk_objects_map_;
  std::unordered_set<ObjectClassification::_label_type> object_types_;

  TargetCrosswalks get_target_crosswalks(
    const TrajectoryPoints & traj_points, const FilterContext & context);

  void update_target_objects(
    const FilterContext & context, const TargetCrosswalks & target_crosswalks);

  bool is_obstructing_crosswalk(
    const TrajectoryPoints & traj_points, const TargetCrosswalk & target_crosswalk) const;

  void update_debug_data(
    const TrajectoryPoints & traj_points, const TargetCrosswalks & target_crosswalks,
    const std::unordered_set<lanelet::Id> & obstructing_crosswalk_ids,
    const rclcpp::Time & current_time, const double z);
};

}  // namespace autoware::trajectory_validator::plugin::traffic_rule

#endif  // AUTOWARE__TRAJECTORY_VALIDATOR__FILTERS__TRAFFIC_RULE__CROSSWALK_FILTER_HPP_
