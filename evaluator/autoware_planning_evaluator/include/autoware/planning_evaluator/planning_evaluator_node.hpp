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

#ifndef AUTOWARE__PLANNING_EVALUATOR__PLANNING_EVALUATOR_NODE_HPP_
#define AUTOWARE__PLANNING_EVALUATOR__PLANNING_EVALUATOR_NODE_HPP_

#include "autoware/planning_evaluator/metrics/metric.hpp"
#include "autoware/planning_evaluator/metrics/output_metric.hpp"
#include "autoware/planning_evaluator/metrics_accumulator.hpp"
#include "autoware/planning_evaluator/metrics_calculator.hpp"
#include "autoware/planning_evaluator/obstacle_metrics_calculator.hpp"
#include "rclcpp/rclcpp.hpp"

#include <autoware/agnocast_wrapper/node.hpp>
#include <autoware/agnocast_wrapper/polling_subscriber.hpp>
#include <autoware/agnocast_wrapper/tf2.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <autoware_utils/math/accumulator.hpp>
#include <autoware_utils/ros/polling_subscriber.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <autoware_internal_planning_msgs/msg/planning_factor.hpp>
#include <autoware_internal_planning_msgs/msg/planning_factor_array.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/pose_with_uuid_stamped.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <autoware_vehicle_msgs/msg/detail/steering_report__struct.hpp>
#include <autoware_vehicle_msgs/msg/detail/turn_indicators_report__struct.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tier4_metric_msgs/msg/metric.hpp>
#include <tier4_metric_msgs/msg/metric_array.hpp>

#include <array>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
namespace planning_diagnostics
{
using autoware::vehicle_info_utils::VehicleInfo;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_planning_msgs::msg::PoseWithUuidStamped;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using autoware_utils::Accumulator;
using autoware_vehicle_msgs::msg::SteeringReport;
using autoware_vehicle_msgs::msg::TurnIndicatorsReport;
using MetricMsg = tier4_metric_msgs::msg::Metric;
using MetricArrayMsg = tier4_metric_msgs::msg::MetricArray;
using nav_msgs::msg::Odometry;
using LaneletMapBin = autoware_map_msgs::msg::LaneletMapBin;
using autoware_internal_planning_msgs::msg::PlanningFactor;
using autoware_internal_planning_msgs::msg::PlanningFactorArray;
using autoware_planning_msgs::msg::LaneletRoute;
using geometry_msgs::msg::AccelWithCovarianceStamped;
/**
 * @brief Node for planning evaluation
 */
class PlanningEvaluatorNode : public autoware::agnocast_wrapper::Node
{
public:
  explicit PlanningEvaluatorNode(const rclcpp::NodeOptions & node_options);
  ~PlanningEvaluatorNode() override;

  /**
   * @brief callback on receiving an odometry
   * @param [in] odometry_msg received odometry message
   */
  void onOdometry(const std::shared_ptr<const Odometry> odometry_msg);

  /**parameters
   * @brief callback on receiving a trajectory
   * @param [in] traj_msg received trajectory message
   */
  void onTrajectory(
    const std::shared_ptr<const Trajectory> traj_msg,
    const std::shared_ptr<const Odometry> ego_state_ptr);

  /**
   * @brief callback on receiving a reference trajectory
   * @param [in] traj_msg received trajectory message
   */
  void onReferenceTrajectory(const std::shared_ptr<const Trajectory> traj_msg);

  /**
   * @brief callback on receiving a dynamic objects array
   * @param [in] objects_msg received dynamic object array message
   */
  void onObjects(const std::shared_ptr<const PredictedObjects> objects_msg);

  /**
   * @brief callback on receiving a modified goal
   * @param [in] modified_goal_msg received modified goal message
   */
  void onModifiedGoal(
    const std::shared_ptr<const PoseWithUuidStamped> modified_goal_msg,
    const std::shared_ptr<const Odometry> ego_state_ptr);

  /**
   * @brief callback on receiving an steering status message
   * @param [in] steering_msg received steering status message
   */
  void onSteering(const std::shared_ptr<const SteeringReport> steering_msg);

  /**
   * @brief callback on receiving a turn indicators message
   * @param [in] blinker_msg received turn indicators message
   */
  void onBlinker(const std::shared_ptr<const TurnIndicatorsReport> blinker_msg);

  /**
   * @brief callback on receiving a planning factors
   * @param [in] planning_factors received planning factor message
   * @param [in] module_name module name of the planning factor
   */
  void onPlanningFactors(
    const std::shared_ptr<const PlanningFactorArray> planning_factors,
    const std::string & module_name);

  /**
   * @brief add the given metric statistic
   */
  void AddMetricMsg(const Metric & metric, const Accumulator<double> & metric_stat);

  /**
   * @brief add current ego lane info
   */
  void AddLaneletMetricMsg(const std::shared_ptr<const Odometry> ego_state_ptr);

  /**
   * @brief add current ego kinematic state
   */
  void AddKinematicStateMetricMsg(
    const AccelWithCovarianceStamped & accel_stamped,
    const std::shared_ptr<const Odometry> ego_state_ptr);

  /**
   * @brief add stop count metric for a specific planning factor module
   */
  void AddStopCountMetricMsg(
    const std::shared_ptr<const PlanningFactorArray> & planning_factors,
    const std::shared_ptr<const Odometry> ego_state_ptr, const std::string & module_name);

  /**
   * @brief add obstacle metric for a specific object
   * @param [in] metric the obstacle metric type
   * @param [in] metric_stat the metric statistics
   * @param [in] object_name the object identifier (e.g., "worst" or object UUID)
   */
  void AddObstacleMsg(
    const Metric & metric, const Accumulator<double> & metric_stat,
    const std::string & object_name);

private:
  static bool isFinite(const TrajectoryPoint & p);

  /**
   * @brief update route handler data
   */
  void getRouteData();

  /**
   * @brief fetch data and publish diagnostics
   */
  void onTimer();

  // ROS subscribers
  autoware::agnocast_wrapper::polling::PollingSubscriber<Trajectory>::SharedPtr traj_sub_ =
    autoware::agnocast_wrapper::polling::create_polling_subscriber<Trajectory>(
      this, "~/input/trajectory");
  autoware::agnocast_wrapper::polling::PollingSubscriber<Trajectory>::SharedPtr ref_sub_ =
    autoware::agnocast_wrapper::polling::create_polling_subscriber<Trajectory>(
      this, "~/input/reference_trajectory");
  autoware::agnocast_wrapper::polling::PollingSubscriber<PredictedObjects>::SharedPtr objects_sub_ =
    autoware::agnocast_wrapper::polling::create_polling_subscriber<PredictedObjects>(
      this, "~/input/objects");
  autoware::agnocast_wrapper::polling::PollingSubscriber<PoseWithUuidStamped>::SharedPtr
    modified_goal_sub_ =
      autoware::agnocast_wrapper::polling::create_polling_subscriber<PoseWithUuidStamped>(
        this, "~/input/modified_goal");
  autoware::agnocast_wrapper::polling::PollingSubscriber<Odometry>::SharedPtr odometry_sub_ =
    autoware::agnocast_wrapper::polling::create_polling_subscriber<Odometry>(
      this, "~/input/odometry");
  autoware::agnocast_wrapper::polling::PollingSubscriber<LaneletRoute>::SharedPtr
    route_subscriber_ =
      autoware::agnocast_wrapper::polling::create_polling_subscriber<LaneletRoute>(
        this, "~/input/route", rclcpp::QoS{1}.transient_local());
  autoware::agnocast_wrapper::polling::PollingSubscriber<LaneletMapBin>::SharedPtr
    vector_map_subscriber_ =
      autoware::agnocast_wrapper::polling::create_polling_subscriber<LaneletMapBin>(
        this, "~/input/vector_map", rclcpp::QoS{1}.transient_local());
  autoware::agnocast_wrapper::polling::PollingSubscriber<AccelWithCovarianceStamped>::SharedPtr
    accel_sub_ =
      autoware::agnocast_wrapper::polling::create_polling_subscriber<AccelWithCovarianceStamped>(
        this, "~/input/acceleration");
  autoware::agnocast_wrapper::polling::PollingSubscriber<SteeringReport>::SharedPtr steering_sub_ =
    autoware::agnocast_wrapper::polling::create_polling_subscriber<SteeringReport>(
      this, "~/input/steering_status");
  autoware::agnocast_wrapper::polling::PollingSubscriber<TurnIndicatorsReport>::SharedPtr
    blinker_sub_ =
      autoware::agnocast_wrapper::polling::create_polling_subscriber<TurnIndicatorsReport>(
        this, "~/input/turn_indicators_status");

  std::unordered_map<
    std::string,
    autoware::agnocast_wrapper::polling::PollingSubscriber<PlanningFactorArray>::SharedPtr>
    planning_factors_sub_;
  std::unordered_set<std::string> stop_decision_modules_;

  // ROS publishers
  AUTOWARE_PUBLISHER_PTR(autoware_internal_debug_msgs::msg::Float64Stamped) processing_time_pub_;
  AUTOWARE_PUBLISHER_PTR(MetricArrayMsg) metrics_pub_;
  std::shared_ptr<autoware::agnocast_wrapper::TransformListener> transform_listener_{nullptr};
  std::unique_ptr<autoware::agnocast_wrapper::Buffer> tf_buffer_;
  autoware::route_handler::RouteHandler route_handler_;

  // Message to publish
  MetricArrayMsg metrics_msg_;

  // Parameters
  bool output_metrics_;
  std::string ego_frame_str_;

  // Calculator and accumulator
  MetricsCalculator metrics_calculator_;
  ObstacleMetricsCalculator obstacle_metrics_calculator_;
  MetricsAccumulator metrics_accumulator_;

  // Metrics for publishing

  std::unordered_set<Metric> metrics_for_publish_;
  std::unordered_set<OutputMetric> metrics_for_output_;

  AUTOWARE_TIMER_PTR timer_;
  VehicleInfo vehicle_info_;
  std::optional<AccelWithCovarianceStamped> prev_acc_stamped_{std::nullopt};
};
}  // namespace planning_diagnostics

#endif  // AUTOWARE__PLANNING_EVALUATOR__PLANNING_EVALUATOR_NODE_HPP_
