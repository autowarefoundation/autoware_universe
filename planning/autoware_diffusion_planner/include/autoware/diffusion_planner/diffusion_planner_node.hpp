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

#ifndef AUTOWARE__DIFFUSION_PLANNER__DIFFUSION_PLANNER_NODE_HPP_
#define AUTOWARE__DIFFUSION_PLANNER__DIFFUSION_PLANNER_NODE_HPP_

#include "autoware/diffusion_planner/diffusion_planner_core.hpp"
#include "autoware/diffusion_planner/utils/planning_factor_utils.hpp"

#include <autoware/agnocast_wrapper/node.hpp>
#include <autoware/agnocast_wrapper/polling_subscriber.hpp>
#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/planning_factor_interface/planning_factor_interface.hpp>
#include <autoware/vehicle_info_utils/vehicle_info.hpp>
#include <autoware_utils/ros/update_param.hpp>
#include <autoware_utils/system/time_keeper.hpp>
#include <autoware_utils_diagnostics/diagnostics_interface.hpp>
#include <autoware_utils_system/stop_watch.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <autoware_internal_debug_msgs/msg/string_stamped.hpp>
#include <autoware_internal_planning_msgs/msg/candidate_trajectories.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::diffusion_planner
{
using autoware_internal_planning_msgs::msg::CandidateTrajectories;
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_planning_msgs::msg::Trajectory;
using autoware_vehicle_msgs::msg::TurnIndicatorsCommand;
using HADMapBin = autoware_map_msgs::msg::LaneletMapBin;
using autoware::vehicle_info_utils::VehicleInfo;
using autoware_internal_planning_msgs::msg::PlanningFactor;
using DiagnosticsInterface =
  autoware_utils_diagnostics::BasicDiagnosticsInterface<autoware::agnocast_wrapper::Node>;
using geometry_msgs::msg::Pose;
using rcl_interfaces::msg::SetParametersResult;
using std_srvs::srv::SetBool;
using unique_identifier_msgs::msg::UUID;
using visualization_msgs::msg::MarkerArray;

struct DiffusionPlannerDebugParams
{
  bool publish_debug_route{true};
  bool publish_debug_map{false};
  bool publish_debug_linestrings{true};
};

struct DiffusionPlannerPlanningFactorParams
{
  bool enable_stop{false};
  bool enable_slowdown{false};
  PlanningFactorDetectionConfig detection_config;
};

/**
 * @class DiffusionPlanner
 * @brief Main class for the diffusion-based trajectory planner node in Autoware.
 *
 * Handles parameter setup, map and route processing, ONNX model inference, and publishing of
 * planned trajectories and debug information.
 *
 * @note This class integrates with ROS 2, ONNX Runtime, and Autoware-specific utilities for
 * autonomous vehicle trajectory planning.
 *
 * @section Responsibilities
 * - Parameter management and dynamic reconfiguration
 * - Map and route data handling
 * - Preprocessing and normalization of input data for inference
 * - Running inference using ONNX models
 * - Postprocessing and publishing of predicted trajectories and debug markers
 * - Managing subscriptions and publishers for required topics
 *
 * @section Members
 * @brief
 * - set_up_params: Initialize and declare node parameters.
 * - load_model: Load TensorRT model via core.
 * - on_timer: Timer callback for periodic processing and publishing.
 * - on_map: Callback for receiving and processing map data.
 * - publish_debug_markers: Publish visualization markers for debugging.
 * - publish_first_traffic_light_on_route: Publish first traffic light for debug.
 * - on_parameter: Callback for dynamic parameter updates.
 *
 * @section Internal State
 * @brief
 * - core_: Core logic instance handling inference and state management.
 * - params_, debug_params_: Node and debug parameters.
 * - ROS 2 node elements: timer_, publishers, subscriptions, and time_keeper_.
 * - generator_uuid_: Unique identifier for the planner instance.
 * - vehicle_info_: Vehicle-specific parameters.
 */
class DiffusionPlanner : public autoware::agnocast_wrapper::Node
{
public:
  explicit DiffusionPlanner(const rclcpp::NodeOptions & options);
  ~DiffusionPlanner();

private:
  /**
   * @brief Initialize and declare node parameters.
   */
  void set_up_params();

  /**
   * @brief Load TensorRT model and normalization statistics.
   *
   * Updates the normalization_map_ and diffusion_planner_inference_ member variables.
   *
   * @throws std::runtime_error if args_path or model paths are invalid, if the
   *         model version is incompatible, or if TensorRT engine setup fails.
   */
  void load_model();

  /**
   * @brief Timer callback for periodic processing and publishing.
   */
  void on_timer();

  /**
   * @brief Callback for receiving and processing map data.
   * @param map_msg The received map message.
   */
  void on_map(const AUTOWARE_MESSAGE_CONST_SHARED_PTR(HADMapBin) & map_msg);

  /**
   * @brief Publish visualization markers for debugging.
   * @param input_data_map Input data used for inference.
   * @param ego_to_map_transform Transform from ego to map frame for visualization.
   */
  void publish_debug_markers(
    const InputDataMap & input_data_map, const Eigen::Matrix4d & ego_to_map_transform,
    const rclcpp::Time & timestamp) const;

  /**
   * @brief Publish the first traffic light on the route (from ego forward) for debug.
   * @param frame_context Context of the current frame (ego pose).
   */
  void publish_first_traffic_light_on_route(const FrameContext & frame_context) const;

  /**
   * @brief Publish the ego pose snapped onto the previous trajectory and the interpolation time
   *        used to compute it. Does nothing when the frame did not snap the ego pose.
   * @param frame_context Context of the current frame.
   * @param timestamp Timestamp of the current frame.
   */
  void publish_snapped_pose(
    const FrameContext & frame_context, const rclcpp::Time & timestamp) const;

  /**
   * @brief Publish planning factors (stop/slowdown) derived from the trajectory.
   * @param trajectory The planned trajectory.
   */
  void publish_planning_factor(const Trajectory & trajectory);

  /**
   * @brief Publish guidance triggered status as a debug message.
   * @param guidance_triggered Map of guidance name to triggered flags per batch.
   * @param timestamp Timestamp of the current frame.
   */
  void publish_guidance_status(
    const std::unordered_map<std::string, std::vector<bool>> & guidance_triggered,
    const rclcpp::Time & timestamp);

  /**
   * @brief Callback for dynamic parameter updates.
   * @param parameters Updated parameters.
   * @return Result of parameter update.
   */
  SetParametersResult on_parameter(const std::vector<rclcpp::Parameter> & parameters);

  /**
   * @brief Enable or disable start guidance.
   */
  void on_set_start_guidance_enabled(
    const AUTOWARE_SERVER_REQUEST_PTR(SetBool) & request,
    const AUTOWARE_SERVER_RESPONSE_PTR(SetBool) & response);

  /**
   * @brief Enable or disable stop guidance.
   */
  void on_set_stop_guidance_enabled(
    const AUTOWARE_SERVER_REQUEST_PTR(SetBool) & request,
    const AUTOWARE_SERVER_RESPONSE_PTR(SetBool) & response);

  /**
   * @brief Enable or disable centerline guidance.
   */
  void on_set_centerline_guidance_enabled(
    const AUTOWARE_SERVER_REQUEST_PTR(SetBool) & request,
    const AUTOWARE_SERVER_RESPONSE_PTR(SetBool) & response);

  // Core logic instance
  std::unique_ptr<DiffusionPlannerCore> core_;

  // Node parameters
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  DiffusionPlannerParams params_;
  DiffusionPlannerDebugParams debug_params_;

  // Node elements
  AUTOWARE_TIMER_PTR timer_;
  AUTOWARE_PUBLISHER_PTR(autoware_utils::ProcessingTimeDetail) debug_processing_time_detail_pub_;
  AUTOWARE_PUBLISHER_PTR(autoware_internal_debug_msgs::msg::Float64Stamped)
  debug_processing_time_pub_;
  AUTOWARE_PUBLISHER_PTR(Trajectory) pub_trajectory_;
  AUTOWARE_PUBLISHER_PTR(CandidateTrajectories) pub_trajectories_;
  AUTOWARE_PUBLISHER_PTR(PredictedObjects) pub_objects_;
  AUTOWARE_PUBLISHER_PTR(MarkerArray) pub_lane_marker_;
  AUTOWARE_PUBLISHER_PTR(MarkerArray) pub_linestring_marker_;
  AUTOWARE_PUBLISHER_PTR(MarkerArray) pub_route_marker_;
  AUTOWARE_PUBLISHER_PTR(TurnIndicatorsCommand) pub_turn_indicators_;
  AUTOWARE_PUBLISHER_PTR(autoware_perception_msgs::msg::TrafficLightGroup)
  pub_traffic_signal_;
  AUTOWARE_PUBLISHER_PTR(geometry_msgs::msg::PoseStamped) pub_snapped_pose_;
  AUTOWARE_PUBLISHER_PTR(autoware_internal_debug_msgs::msg::Float64Stamped)
  pub_snap_interpolation_time_;
  AUTOWARE_PUBLISHER_PTR(std_msgs::msg::Float64) pub_inference_time_;
  AUTOWARE_PUBLISHER_PTR(std_msgs::msg::Float32MultiArray) pub_denoising_steps_;
  AUTOWARE_PUBLISHER_PTR(autoware_internal_debug_msgs::msg::StringStamped)
  pub_guidance_status_;
  AUTOWARE_SERVICE_PTR(SetBool) set_start_guidance_enabled_service_;
  AUTOWARE_SERVICE_PTR(SetBool) set_stop_guidance_enabled_service_;
  AUTOWARE_SERVICE_PTR(SetBool) set_centerline_guidance_enabled_service_;
  mutable std::shared_ptr<autoware_utils::TimeKeeper> time_keeper_{nullptr};
  autoware::agnocast_wrapper::polling::PollingSubscriber<Odometry>::SharedPtr
    sub_current_odometry_ =
      autoware::agnocast_wrapper::polling::create_polling_subscriber<Odometry>(
        this, "~/input/odometry");
  autoware::agnocast_wrapper::polling::PollingSubscriber<AccelWithCovarianceStamped>::SharedPtr
    sub_current_acceleration_ =
      autoware::agnocast_wrapper::polling::create_polling_subscriber<AccelWithCovarianceStamped>(
        this, "~/input/acceleration");
  autoware::agnocast_wrapper::polling::PollingSubscriber<TrackedObjects>::SharedPtr
    sub_tracked_objects_ =
      autoware::agnocast_wrapper::polling::create_polling_subscriber<TrackedObjects>(
        this, "~/input/tracked_objects");
  autoware::agnocast_wrapper::polling::PollingSubscriber<
    autoware_perception_msgs::msg::TrafficLightGroupArray,
    autoware::agnocast_wrapper::polling::polling_policy::All>::SharedPtr sub_traffic_signals_ =
    autoware::agnocast_wrapper::polling::create_polling_subscriber<
      autoware_perception_msgs::msg::TrafficLightGroupArray,
      autoware::agnocast_wrapper::polling::polling_policy::All>(
      this, "~/input/traffic_signals", rclcpp::QoS{10});
  autoware::agnocast_wrapper::polling::PollingSubscriber<TurnIndicatorsReport>::SharedPtr
    sub_turn_indicators_ =
      autoware::agnocast_wrapper::polling::create_polling_subscriber<TurnIndicatorsReport>(
        this, "~/input/turn_indicators");
  autoware::agnocast_wrapper::polling::PollingSubscriber<
    LaneletRoute, autoware::agnocast_wrapper::polling::polling_policy::Newest>::SharedPtr
    route_subscriber_ = autoware::agnocast_wrapper::polling::create_polling_subscriber<
      LaneletRoute, autoware::agnocast_wrapper::polling::polling_policy::Newest>(
      this, "~/input/route", rclcpp::QoS{1}.transient_local());
  AUTOWARE_SUBSCRIPTION_PTR(HADMapBin) sub_map_;
  UUID generator_uuid_;
  VehicleInfo vehicle_info_;

  std::unique_ptr<DiagnosticsInterface> diagnostics_inference_;
  std::shared_ptr<const lanelet::LaneletMap> lanelet_map_ptr_{nullptr};

  std::unique_ptr<autoware_utils_system::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_;

  std::unique_ptr<
    autoware::planning_factor_interface::PlanningFactorInterfaceT<autoware::agnocast_wrapper::Node>>
    planning_factor_interface_;
  DiffusionPlannerPlanningFactorParams planning_factor_params_;
};

}  // namespace autoware::diffusion_planner
#endif  // AUTOWARE__DIFFUSION_PLANNER__DIFFUSION_PLANNER_NODE_HPP_
