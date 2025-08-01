// Copyright 2022 TIER IV, Inc.
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

#ifndef AUTOWARE__BEHAVIOR_PATH_START_PLANNER_MODULE__DATA_STRUCTS_HPP_
#define AUTOWARE__BEHAVIOR_PATH_START_PLANNER_MODULE__DATA_STRUCTS_HPP_

#include "autoware/behavior_path_planner_common/utils/parking_departure/geometric_parallel_parking.hpp"
#include "autoware/behavior_path_planner_common/utils/path_safety_checker/path_safety_checker_parameters.hpp"

#include <autoware/freespace_planning_algorithms/abstract_algorithm.hpp>
#include <autoware/freespace_planning_algorithms/astar_search.hpp>
#include <autoware/freespace_planning_algorithms/rrtstar.hpp>
#include <magic_enum.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <cmath>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>
#include <vector>
namespace autoware::behavior_path_planner
{

using geometry_msgs::msg::Pose;

using autoware::behavior_path_planner::utils::path_safety_checker::CollisionCheckDebugMap;
using autoware::behavior_path_planner::utils::path_safety_checker::PoseWithVelocityStamped;
using autoware::behavior_path_planner::utils::path_safety_checker::TargetObjectsOnLane;
using autoware_perception_msgs::msg::PredictedObjects;

using autoware::freespace_planning_algorithms::AstarParam;
using autoware::freespace_planning_algorithms::PlannerCommonParam;
using autoware::freespace_planning_algorithms::RRTStarParam;

enum class PlannerType {
  NONE = 0,
  SHIFT = 1,
  GEOMETRIC = 2,
  CLOTHOID = 3,
  STOP = 4,
  FREESPACE = 5,
};

struct PlannerDebugData
{
public:
  PlannerType planner_type;
  double backward_distance{0.0};
  double required_margin{0.0};
  std::vector<std::string> conditions_evaluation;

  static std::string double_to_str(double value, int precision = 1)
  {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << value;
    return oss.str();
  }

  static std::string to_planner_type_name(PlannerType pt)
  {
    // Adding whitespace for column width alignment in RViz display
    switch (pt) {
      case PlannerType::NONE:
        return "NONE                  ";
      case PlannerType::SHIFT:
        return "SHIFT               ";
      case PlannerType::GEOMETRIC:
        return "GEOMETRIC   ";
      case PlannerType::CLOTHOID:
        return "CLOTHOID    ";
      case PlannerType::STOP:
        return "STOP                  ";
      case PlannerType::FREESPACE:
        return "FREESPACE   ";
      default:
        return "UNKNOWN";
    }
  }
};

/**
 * @brief Structure representing a pose-based arc segment
 */
struct ArcSegment
{
  // Geometric parameters of the arc
  geometry_msgs::msg::Point center;     // Center point of the arc
  double radius;                        // Radius [m]
  geometry_msgs::msg::Pose start_pose;  // Start pose
  geometry_msgs::msg::Pose end_pose;    // End pose
  bool is_clockwise;                    // Whether clockwise or not

  ArcSegment() : radius(0.0), is_clockwise(true) { center.x = center.y = center.z = 0.0; }

  /**
   * @brief Calculate start angle
   * @return Start angle [rad]
   */
  double calculateStartAngle() const
  {
    return std::atan2(start_pose.position.y - center.y, start_pose.position.x - center.x);
  }

  /**
   * @brief Calculate end angle
   * @return End angle [rad]
   */
  double calculateEndAngle() const
  {
    return std::atan2(end_pose.position.y - center.y, end_pose.position.x - center.x);
  }

  /**
   * @brief Calculate arc length
   * @return Arc length [m]
   */
  double calculateArcLength() const
  {
    double start_angle = calculateStartAngle();
    double end_angle = calculateEndAngle();
    double angle_diff = std::abs(end_angle - start_angle);

    // Adjust angle difference if it exceeds 2π
    if (angle_diff > 2.0 * M_PI) {
      angle_diff = 2.0 * M_PI - std::fmod(angle_diff, 2.0 * M_PI);
    }
    return radius * angle_diff;
  }

  /**
   * @brief Get curvature (constant for arc)
   * @return Curvature [1/m]
   */
  double getCurvature() const { return (radius > 0.0) ? (1.0 / radius) : 0.0; }

  /**
   * @brief Calculate position at specified angle
   * @param angle Angle [rad]
   * @return Position
   */
  geometry_msgs::msg::Point getPointAtAngle(double angle) const
  {
    geometry_msgs::msg::Point point;
    point.x = center.x + radius * std::cos(angle);
    point.y = center.y + radius * std::sin(angle);
    point.z = center.z;
    return point;
  }

  /**
   * @brief Get start position
   * @return Start position
   */
  geometry_msgs::msg::Point getStartPoint() const { return start_pose.position; }

  /**
   * @brief Get end position
   * @return End position
   */
  geometry_msgs::msg::Point getEndPoint() const { return end_pose.position; }

  /**
   * @brief Calculate pose at specified angle
   * @param angle Angle [rad]
   * @return Pose
   */
  geometry_msgs::msg::Pose getPoseAtAngle(double angle) const
  {
    geometry_msgs::msg::Pose pose;

    // Calculate position
    pose.position = getPointAtAngle(angle);

    // Calculate tangent direction (arc progression direction)
    double tangent_angle = angle + (is_clockwise ? -M_PI / 2 : M_PI / 2);

    // Set quaternion
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = std::sin(tangent_angle / 2.0);
    pose.orientation.w = std::cos(tangent_angle / 2.0);

    return pose;
  }

  /**
   * @brief Get start pose
   * @return Start pose
   */
  geometry_msgs::msg::Pose getStartPose() const { return start_pose; }

  /**
   * @brief Get end pose
   * @return End pose
   */
  geometry_msgs::msg::Pose getEndPose() const { return end_pose; }
};

/**
 * @brief Composite arc path consisting of multiple arc segments
 */
struct CompositeArcPath
{
  std::vector<ArcSegment> segments;  // Array of arc segments

  CompositeArcPath() = default;
};

/**
 * @brief Structure to hold relative pose information in vehicle coordinate system
 */
struct RelativePoseInfo
{
  double longitudinal_distance_vehicle;  // Longitudinal distance in vehicle coordinate [m]
  double lateral_distance_vehicle;  // Lateral distance in vehicle coordinate [m] (positive: left,
                                    // negative: right)
  double angle_diff;  // Angle difference [rad] (positive: counter-clockwise/left turn, negative:
                      // clockwise/right turn)
};

/**
 * @brief Clothoid segment structure for smooth path transitions
 */
struct ClothoidSegment
{
  enum Type { CLOTHOID_ENTRY, CIRCULAR_ARC, CLOTHOID_EXIT };

  Type type;
  double A;           // Clothoid parameter
  double L;           // Arc length
  double radius;      // Radius (for circular arc segment)
  double angle;       // Angle (for circular arc segment)
  bool is_clockwise;  // Rotation direction
  std::string description;

  explicit ClothoidSegment(Type t, double a = 0.0, double l = 0.0)
  : type(t), A(a), L(l), radius(0.0), angle(0.0), is_clockwise(true)
  {
  }
};

struct StartPlannerDebugData
{
  // filtered objects
  PredictedObjects filtered_objects;
  TargetObjectsOnLane target_objects_on_lane;
  std::vector<PoseWithVelocityStamped> ego_predicted_path;
  // collision check debug map
  CollisionCheckDebugMap collision_check;
  lanelet::ConstLanelets departure_check_lanes;

  Pose refined_start_pose;
  std::vector<Pose> start_pose_candidates;
  size_t selected_start_pose_candidate_index;
  double margin_for_start_pose_candidate;

  // for isPreventingRearVehicleFromPassingThrough
  std::optional<Pose> estimated_stop_pose;
};

struct StartPlannerParameters
{
  static StartPlannerParameters init(rclcpp::Node & node);
  double th_arrived_distance{0.0};
  double th_stopped_velocity{0.0};
  double th_stopped_time{0.0};
  double prepare_time_before_start{0.0};
  double th_distance_to_middle_of_the_road{0.0};
  bool skip_rear_vehicle_check{false};
  double extra_width_margin_for_rear_obstacle{0.0};
  std::vector<double> collision_check_margins{};
  double collision_check_margin_from_front_object{0.0};
  double th_moving_object_velocity{0.0};
  autoware::behavior_path_planner::utils::path_safety_checker::ObjectTypesToCheck
    object_types_to_check_for_path_generation{};
  double center_line_path_interval{0.0};
  double lane_departure_check_expansion_margin{0.0};

  // shift pull out
  bool enable_shift_pull_out{false};
  bool check_shift_path_lane_departure{false};
  bool allow_check_shift_path_lane_departure_override{false};
  double shift_collision_check_distance_from_end{0.0};
  double minimum_shift_pull_out_distance{0.0};
  int lateral_acceleration_sampling_num{0};
  double lateral_jerk{0.0};
  double maximum_lateral_acc{0.0};
  double minimum_lateral_acc{0.0};
  double maximum_curvature{0.0};  // maximum curvature considered in the path generation
  double end_pose_curvature_threshold{0.0};
  double maximum_longitudinal_deviation{0.0};
  // geometric pull out
  bool enable_geometric_pull_out{false};
  double geometric_collision_check_distance_from_end{0.0};
  bool divide_pull_out_path{false};
  bool enable_clothoid_fallback{false};
  // Enable clothoid path search when no path is found with collision margins
  ParallelParkingParameters parallel_parking_parameters{};

  // clothoid pull out
  double clothoid_initial_velocity{0.0};
  double clothoid_acceleration{0.0};
  std::vector<double> clothoid_max_steer_angles_deg{};
  double clothoid_max_steer_angle_rate_deg_per_sec{0.0};
  bool check_clothoid_path_lane_departure{true};  // enable lane departure check for clothoid path

  // search start pose backward
  std::string search_priority;  // "efficient_path" or "short_back_distance"
  bool enable_back{false};
  double backward_velocity{0.0};
  double max_back_distance{0.0};
  double backward_search_resolution{0.0};
  double backward_path_update_duration{0.0};
  double ignore_distance_from_lane_end{0.0};
  // freespace planner
  bool enable_freespace_planner{false};
  std::string freespace_planner_algorithm;
  double end_pose_search_start_distance{0.0};
  double end_pose_search_end_distance{0.0};
  double end_pose_search_interval{0.0};
  double freespace_planner_velocity{0.0};
  double vehicle_shape_margin{0.0};
  PlannerCommonParam freespace_planner_common_parameters;
  AstarParam astar_parameters;
  RRTStarParam rrt_star_parameters;

  // stop condition
  double maximum_deceleration_for_stop{0.0};
  double maximum_jerk_for_stop{0.0};

  // hysteresis parameter
  double hysteresis_factor_expand_rate{0.0};

  // path safety checker
  utils::path_safety_checker::EgoPredictedPathParams ego_predicted_path_params{};
  utils::path_safety_checker::ObjectsFilteringParams objects_filtering_params{};
  utils::path_safety_checker::SafetyCheckParams safety_check_params{};

  // surround moving obstacle check
  double search_radius{0.0};
  double th_moving_obstacle_velocity{0.0};
  autoware::behavior_path_planner::utils::path_safety_checker::ObjectTypesToCheck
    surround_moving_obstacles_type_to_check{};

  bool print_debug_info{false};
};

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_START_PLANNER_MODULE__DATA_STRUCTS_HPP_
