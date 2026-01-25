// Copyright 2025 Tier IV, Inc.
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

#ifndef AUTOWARE__PLANNING_EVALUATOR__OBSTACLE_METRICS_CALCULATOR_HPP_
#define AUTOWARE__PLANNING_EVALUATOR__OBSTACLE_METRICS_CALCULATOR_HPP_

#include "autoware/planning_evaluator/metrics/metric.hpp"

#include <autoware_utils/geometry/boost_polygon_utils.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/math/accumulator.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include "autoware_perception_msgs/msg/predicted_objects.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <boost/geometry.hpp>

#include <tf2/utils.h>

#include <array>
#include <cmath>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace planning_diagnostics
{
using autoware::vehicle_info_utils::VehicleInfo;
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::Shape;
using autoware_planning_msgs::msg::Trajectory;
using autoware_utils::Accumulator;
using autoware_utils::calc_distance2d;
using autoware_utils::Point2d;
using autoware_utils::Polygon2d;
using autoware_utils::Segment2d;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
namespace bg = boost::geometry;

/**
 * @brief Data structure to store ego trajectory point
 */
struct EgoTrajectoryPoint
{
  double velocity_mps = 0.0;
  double time_from_start_s = 0.0;
  double distance_from_start_m = 0.0;

  Pose pose;
  std::optional<Polygon2d> polygon;

  EgoTrajectoryPoint() = default;

  /**
   * @brief Constructor from pose, velocity, time from start and distance from start
   * @param [in] p pose
   * @param [in] velocity_mps velocity
   * @param [in] time_from_start_s time from start
   * @param [in] distance_from_start_m distance from start
   */
  EgoTrajectoryPoint(
    const Pose & pose, double velocity_mps, double time_from_start_s, double distance_from_start_m)
  {
    this->velocity_mps = velocity_mps;
    this->time_from_start_s = time_from_start_s;
    this->distance_from_start_m = distance_from_start_m;

    this->pose = pose;
  }

  void setPolygon(const VehicleInfo & ego_vehicle_info)
  {
    if (!polygon.has_value()) {
      const autoware_utils::LinearRing2d ego_footprint = autoware_utils::transform_vector(
        ego_vehicle_info.createFootprint(), autoware_utils::pose2transform(pose));
      Polygon2d ego_polygon;
      ego_polygon.outer() = ego_footprint;
      bg::correct(ego_polygon);
      polygon = std::make_optional(ego_polygon);
    }
  }
};

/**
 * @brief Data structure to store obstacle trajectory point
 */
struct ObstacleTrajectoryPoint
{
  bool is_overlapping_with_ego_trajectory = false;
  bool is_collision_with_ego_trajectory = false;
  size_t first_overlapping_ego_trajectory_index = std::numeric_limits<size_t>::max();
  size_t last_overlapping_ego_trajectory_index = 0;

  double velocity_mps = 0.0;
  double time_from_start_s = 0.0;
  double distance_from_start_m = 0.0;

  Pose pose;
  std::optional<Polygon2d> polygon;

  ObstacleTrajectoryPoint() = default;

  /**
   * @brief Constructor from predicted obstacle
   */
  ObstacleTrajectoryPoint(
    const Pose & pose, double velocity_mps, double time_from_start_s, double distance_from_start_m)
  {
    this->is_overlapping_with_ego_trajectory = false;
    this->is_collision_with_ego_trajectory = false;
    this->first_overlapping_ego_trajectory_index = std::numeric_limits<size_t>::max();
    this->last_overlapping_ego_trajectory_index = 0;
    this->velocity_mps = velocity_mps;
    this->time_from_start_s = time_from_start_s;
    this->distance_from_start_m = distance_from_start_m;

    this->pose = pose;
  }

  /**
   * @brief Constructor from reference point and time from reference
   * @param [in] reference_point reference point
   * @param [in] time_from_reference_s time from reference to the current point
   */
  ObstacleTrajectoryPoint(
    const ObstacleTrajectoryPoint & reference_point, double time_from_reference_s)
  {
    this->is_overlapping_with_ego_trajectory = false;
    this->is_collision_with_ego_trajectory = false;
    this->first_overlapping_ego_trajectory_index = std::numeric_limits<size_t>::max();
    this->last_overlapping_ego_trajectory_index = 0;

    this->velocity_mps = reference_point.velocity_mps;
    this->time_from_start_s = reference_point.time_from_start_s + time_from_reference_s;
    this->distance_from_start_m = reference_point.distance_from_start_m;

    this->pose = reference_point.pose;

    const double distance_from_reference_m = time_from_reference_s * reference_point.velocity_mps;
    this->distance_from_start_m += distance_from_reference_m;

    const double yaw = tf2::getYaw(reference_point.pose.orientation);
    this->pose.position.x += distance_from_reference_m * std::cos(yaw);
    this->pose.position.y += distance_from_reference_m * std::sin(yaw);
  }

  void setPolygon(const Shape & obstacle_shape)
  {
    if (!polygon.has_value()) {
      polygon = std::make_optional(autoware_utils::to_polygon2d(pose, obstacle_shape));
    }
  }
};

/**
 * @brief Calculator for obstacle metrics calculator parameters
 * @details This struct contains the parameters for the obstacle metrics calculator
 * @param [in] worst_only if true, only calculate the worst case metrics
 * @param [in] use_ego_traj_vel if true, use the ego trajectory velocity for obstacle metrics
 * calculation
 * @param [in] collision_thr_m the distance threshold to consider a collision occurs between object
 * footprints and ego trajectory footprints
 * @param [in] stop_velocity_mps the velocity threshold to consider the object or ego static, used
 * for speed up the calculation.
 * @param [in] limit_min_accel the minimum acceleration limit for ego velocity, to avoid abnormal
 * stop points in the ego trajectory.
 * @param [in] min_time_interval_s the minimum time interval to resample the ego trajectory
 * @param [in] min_spatial_interval_m the minimum spatial interval to resample the ego trajectory
 */
class ObstacleMetricsCalculator
{
public:
  struct Parameters
  {
    bool worst_only = true;

    // for metrics calculation
    bool use_ego_traj_vel = false;
    double collision_thr_m = 0.0;
    double stop_velocity_mps = 0.2777;
    double limit_min_accel = -2.5;

    // for trajectory resampling
    double min_time_interval_s = 0.05;
    double min_spatial_interval_m = 0.1;
  } parameters;

  ObstacleMetricsCalculator()
  {
    ego_trajectory_points_.reserve(100);
    obstacle_trajectory_points_.reserve(100);

    for (const auto metric : obstacle_metric_types) {
      metrics_need_[metric] = false;
      obstacle_metrics_[metric].reserve(100);
    }
  }

  static constexpr std::array<Metric, 4> obstacle_metric_types = {
    Metric::obstacle_distance, Metric::obstacle_ttc, Metric::obstacle_pet, Metric::obstacle_drac};

  /**
   * @brief set vehicle info
   * @param [in] vehicle_info input vehicle info
   */
  void setVehicleInfo(const VehicleInfo & vehicle_info);

  /**
   * @brief set the predicted objects used to calculate obstacle metrics
   * @param [in] objects predicted objects
   */
  void setPredictedObjects(const PredictedObjects & objects);

  /**
   * @brief set the ego odometry
   * @param [in] ego_odometry ego odometry
   */
  void setEgoPose(const nav_msgs::msg::Odometry & ego_odometry);

  /**
   * @brief set the trajectory used to calculate obstacle metrics
   * @param [in] traj input trajectory
   */
  void setTrajectory(const Trajectory & traj);

  /**
   * @brief calculate all needed obstacle metrics for each obstacle and store them
   * @return list of pairs containing obstacle UUID/`worst` and corresponding metric accumulator
   *         The first element of pair can be obstacle UUID string or "worst" for worst case
   *         The list length can be 0 if no calculable obstacles exist
   */
  void calculateMetrics();

  /**
   * @brief clear all data and metrics
   */
  void clearData();

  /**
   * @brief get the stored metric for the given Metric type
   * @param [in] metric Metric enum value
   * @return list of pairs containing obstacle UUID/`worst` and corresponding metric accumulator
   *         The first element of pair can be obstacle UUID string or "worst" for worst case
   *         The list length can be 0 if no calculable obstacles exist
   */
  std::vector<std::pair<std::string, Accumulator<double>>> getMetric(const Metric metric) const;

  /**
   * @brief set whether a metric needs to be calculated
   * @param [in] metric Metric enum value
   * @param [in] need true if the metric needs to be calculated, false otherwise
   */
  void setMetricNeed(const Metric metric, bool need);

  /**
   * @brief Collect worst case metrics from all obstacles and insert as "worst"
   * @details For each metric, finds the worst case from all obstacles
   *          and inserts it with the name "worst" into obstacle_metrics_，
   *          if `parameters.worst_only` is true, remove the original metrics from obstacle_metrics_
   *          and only keep the "worst" metric.
   */
  void CollectWorstMetrics();

private:
  /**
   * @brief Check if all required data is ready for metric calculation
   * @return true if data is ready, false otherwise
   */
  bool isDataReady() const;

  /**
   * @brief Preprocess ego trajectory: trim the trajectory from ego pose and resample them.
   * @details This function:
   *          1. Finds the closest trajectory point to ego pose and trims past points
   *          2. Resamples trajectory based on parameters.min_time_interval and
   * parameters.min_spatial_interval
   *          3. Adds ego trajectory points to ego_trajectory_points_
   *          4. calculate the max reachable distance of ego base on the trajectory
   *          Note: The points after the first stop point has `time_from_start_s` =
   * std::numeric_limits<double>::infinity()
   */
  void PreprocessEgoTrajectory();

  /**
   * @brief Process obstacles trajectory and calculate metrics
   * @details For each obstacle, this function:
   *          1. Checks if the obstacle trajectory is no overlapping with ego trajectory, if so,
   * skip some metrics calculation.
   *          2. Creates obstacle trajectory points based on the ego trajectory points'
   * time_from_start_s.
   *          3. Calculates metrics for each obstacle trajectory point and stores them into
   * obstacle_metrics_.
   */
  void ProcessObstaclesTrajectory();

  // input data
  std::optional<PredictedObjects> predicted_objects_;
  std::optional<nav_msgs::msg::Odometry> ego_odometry_;
  std::optional<VehicleInfo> vehicle_info_;
  std::optional<Trajectory> trajectory_;

  // intermediate data
  std::vector<EgoTrajectoryPoint> ego_trajectory_points_;
  std::vector<ObstacleTrajectoryPoint> obstacle_trajectory_points_;
  double ego_max_reachable_distance_ = 0.0;

  // output data
  std::unordered_map<Metric, bool> metrics_need_;
  std::unordered_map<Metric, std::vector<std::pair<std::string, Accumulator<double>>>>
    obstacle_metrics_;
};

}  // namespace planning_diagnostics

#endif  // AUTOWARE__PLANNING_EVALUATOR__OBSTACLE_METRICS_CALCULATOR_HPP_
