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

#ifndef AUTOWARE__CONTROL_EVALUATOR__CONTROL_EVALUATOR_NODE_HPP_
#define AUTOWARE__CONTROL_EVALUATOR__CONTROL_EVALUATOR_NODE_HPP_

#include "autoware/control_evaluator/metrics/deviation_metrics.hpp"
#include "autoware/control_evaluator/metrics/metric.hpp"
#include "autoware_utils/math/accumulator.hpp"

#include <autoware/route_handler/route_handler.hpp>
#include <autoware_utils/geometry/boost_geometry.hpp>
#include <autoware_utils/ros/polling_subscriber.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <autoware_utils_math/accumulator.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include "autoware_vehicle_msgs/msg/steering_report.hpp"
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_internal_planning_msgs/msg/planning_factor.hpp>
#include <autoware_internal_planning_msgs/msg/planning_factor_array.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tier4_metric_msgs/msg/metric.hpp>
#include <tier4_metric_msgs/msg/metric_array.hpp>

#include <deque>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace control_diagnostics
{
using autoware::vehicle_info_utils::VehicleInfo;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_planning_msgs::msg::Trajectory;
using autoware_utils::Accumulator;
using autoware_utils::LineString2d;
using autoware_utils::Point2d;
using autoware_vehicle_msgs::msg::SteeringReport;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using nav_msgs::msg::Odometry;
using LaneletMapBin = autoware_map_msgs::msg::LaneletMapBin;
using autoware_planning_msgs::msg::LaneletRoute;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using MetricMsg = tier4_metric_msgs::msg::Metric;
using MetricArrayMsg = tier4_metric_msgs::msg::MetricArray;
using autoware_internal_planning_msgs::msg::PathWithLaneId;
using autoware_internal_planning_msgs::msg::PlanningFactor;
using autoware_internal_planning_msgs::msg::PlanningFactorArray;

/**
 * @brief Node for control evaluation
 */
class ControlEvaluatorNode : public rclcpp::Node
{
public:
  explicit ControlEvaluatorNode(const rclcpp::NodeOptions & node_options);
  ~ControlEvaluatorNode() override;

  void AddMetricMsg(
    const Metric & metric, const double & metric_value, const bool & accumulate_metric = true);
  void AddLateralDeviationMetricMsg(const Trajectory & traj, const Point & ego_point);
  void AddYawDeviationMetricMsg(const Trajectory & traj, const Pose & ego_pose);
  void AddGoalDeviationMetricMsg(const Odometry & odom);
  void AddObjectMetricMsg(const Odometry & odom, const PredictedObjects & objects);
  void AddBoundaryDistanceMetricMsg(const PathWithLaneId & behavior_path, const Pose & ego_pose);
  void AddUncrossableBoundaryDistanceMetricMsg(const Pose & ego_pose);

  void AddLaneletInfoMsg(const Pose & ego_pose);
  void AddKinematicStateMetricMsg(
    const Odometry & odom, const AccelWithCovarianceStamped & accel_stamped);
  void AddSteeringMetricMsg(const SteeringReport & steering_report);
  void AddStopDeviationMetricMsg();
  void onTimer();

private:
  autoware_utils::InterProcessPollingSubscriber<Odometry> odometry_sub_{this, "~/input/odometry"};
  autoware_utils::InterProcessPollingSubscriber<AccelWithCovarianceStamped> accel_sub_{
    this, "~/input/acceleration"};
  autoware_utils::InterProcessPollingSubscriber<Trajectory> traj_sub_{this, "~/input/trajectory"};
  autoware_utils::InterProcessPollingSubscriber<
    LaneletRoute, autoware_utils::polling_policy::Newest>
    route_subscriber_{this, "~/input/route", rclcpp::QoS{1}.transient_local()};
  autoware_utils::InterProcessPollingSubscriber<
    LaneletMapBin, autoware_utils::polling_policy::Newest>
    vector_map_subscriber_{this, "~/input/vector_map", rclcpp::QoS{1}.transient_local()};
  autoware_utils::InterProcessPollingSubscriber<PathWithLaneId> behavior_path_subscriber_{
    this, "~/input/behavior_path"};
  autoware_utils::InterProcessPollingSubscriber<SteeringReport> steering_sub_{
    this, "~/input/steering_status"};
  autoware_utils::InterProcessPollingSubscriber<PredictedObjects> objects_sub_{
    this, "~/input/objects"};
  std::unordered_map<
    std::string, autoware_utils::InterProcessPollingSubscriber<PlanningFactorArray>>
    planning_factors_sub_;
  std::unordered_map<std::string, Accumulator<double>> stop_deviation_accumulators_;
  std::unordered_map<std::string, Accumulator<double>> stop_deviation_abs_accumulators_;
  std::unordered_set<std::string> stop_deviation_modules_;

  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float64Stamped>::SharedPtr
    processing_time_pub_;
  rclcpp::Publisher<MetricArrayMsg>::SharedPtr metrics_pub_;

  // update Route Handler
  void getRouteData();

  // Parameters
  bool output_metrics_;
  double distance_filter_thr_m_;

  // Metric
  const std::vector<Metric> metrics_ = {
    // collect all metrics
    Metric::velocity,
    Metric::acceleration,
    Metric::jerk,
    Metric::lateral_deviation,
    Metric::lateral_deviation_abs,
    Metric::yaw_deviation,
    Metric::yaw_deviation_abs,
    Metric::goal_longitudinal_deviation,
    Metric::goal_longitudinal_deviation_abs,
    Metric::goal_lateral_deviation,
    Metric::goal_lateral_deviation_abs,
    Metric::goal_yaw_deviation,
    Metric::goal_yaw_deviation_abs,
    Metric::left_boundary_distance,
    Metric::right_boundary_distance,
    Metric::steering_angle,
    Metric::steering_angle_abs,
    Metric::steering_rate,
    Metric::steering_acceleration,
    Metric::stop_deviation,
    Metric::stop_deviation_abs,
    Metric::closest_object_distance};

  std::array<Accumulator<double>, static_cast<size_t>(Metric::SIZE)>
    metric_accumulators_;  // 3(min, max, mean) * metric_size

  MetricArrayMsg metrics_msg_;
  VehicleInfo vehicle_info_;
  autoware::route_handler::RouteHandler route_handler_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::optional<AccelWithCovarianceStamped> prev_acc_stamped_{std::nullopt};
  std::optional<double> prev_steering_angle_{std::nullopt};
  std::optional<double> prev_steering_rate_{std::nullopt};
  std::optional<double> prev_steering_angle_timestamp_{std::nullopt};

  // for output_metrics_only_moving
  float ego_speed_{0.0};
  std::array<bool, static_cast<size_t>(Metric::SIZE)> is_output_metrics_only_moving{};
};
}  // namespace control_diagnostics

#endif  // AUTOWARE__CONTROL_EVALUATOR__CONTROL_EVALUATOR_NODE_HPP_
