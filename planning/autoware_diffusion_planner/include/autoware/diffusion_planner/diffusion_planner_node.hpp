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

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/vehicle_info_utils/vehicle_info.hpp>
#include <autoware_utils/ros/polling_subscriber.hpp>
#include <autoware_utils/ros/update_param.hpp>
#include <autoware_utils/system/time_keeper.hpp>
#include <autoware_utils_diagnostics/diagnostics_interface.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>

#include <autoware_internal_planning_msgs/msg/candidate_trajectories.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <string>
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
using autoware_utils_diagnostics::DiagnosticsInterface;
using geometry_msgs::msg::Pose;
using rcl_interfaces::msg::SetParametersResult;
using unique_identifier_msgs::msg::UUID;
using visualization_msgs::msg::MarkerArray;

struct DiffusionPlannerDebugParams
{
  bool publish_debug_route{true};
  bool publish_debug_map{false};
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
class DiffusionPlanner : public rclcpp::Node
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
   * Updates the normalization_map_ and tensorrt_inference_ member variables.
   *
   * @throws std::runtime_error if args_path or model_path are invalid, if the
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
  void on_map(const HADMapBin::ConstSharedPtr map_msg);

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
   * @brief Callback for dynamic parameter updates.
   * @param parameters Updated parameters.
   * @return Result of parameter update.
   */
  SetParametersResult on_parameter(const std::vector<rclcpp::Parameter> & parameters);

  // Core logic instance
  std::unique_ptr<DiffusionPlannerCore> core_;

  // Node parameters
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  DiffusionPlannerParams params_;
  DiffusionPlannerDebugParams debug_params_;

  // Node elements
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<autoware_utils::ProcessingTimeDetail>::SharedPtr
    debug_processing_time_detail_pub_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_trajectory_{nullptr};
  rclcpp::Publisher<CandidateTrajectories>::SharedPtr pub_trajectories_{nullptr};
  rclcpp::Publisher<PredictedObjects>::SharedPtr pub_objects_{nullptr};
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_lane_marker_{nullptr};
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_route_marker_{nullptr};
  rclcpp::Publisher<TurnIndicatorsCommand>::SharedPtr pub_turn_indicators_{nullptr};
  rclcpp::Publisher<autoware_perception_msgs::msg::TrafficLightGroup>::SharedPtr
    pub_traffic_signal_{nullptr};
  mutable std::shared_ptr<autoware_utils::TimeKeeper> time_keeper_{nullptr};
  autoware_utils::InterProcessPollingSubscriber<Odometry> sub_current_odometry_{
    this, "~/input/odometry"};
  autoware_utils::InterProcessPollingSubscriber<AccelWithCovarianceStamped>
    sub_current_acceleration_{this, "~/input/acceleration"};
  autoware_utils::InterProcessPollingSubscriber<TrackedObjects> sub_tracked_objects_{
    this, "~/input/tracked_objects"};
  autoware_utils::InterProcessPollingSubscriber<
    autoware_perception_msgs::msg::TrafficLightGroupArray, autoware_utils::polling_policy::All>
    sub_traffic_signals_{this, "~/input/traffic_signals", rclcpp::QoS{10}};
  autoware_utils::InterProcessPollingSubscriber<TurnIndicatorsReport> sub_turn_indicators_{
    this, "~/input/turn_indicators"};
  autoware_utils::InterProcessPollingSubscriber<
    LaneletRoute, autoware_utils::polling_policy::Newest>
    route_subscriber_{this, "~/input/route", rclcpp::QoS{1}.transient_local()};
  autoware_utils::InterProcessPollingSubscriber<
    LaneletMapBin, autoware_utils::polling_policy::Newest>
    vector_map_subscriber_{this, "~/input/vector_map", rclcpp::QoS{1}.transient_local()};
  rclcpp::Subscription<HADMapBin>::SharedPtr sub_map_;
  UUID generator_uuid_;
  VehicleInfo vehicle_info_;

  std::unique_ptr<DiagnosticsInterface> diagnostics_inference_;
};

}  // namespace autoware::diffusion_planner
#endif  // AUTOWARE__DIFFUSION_PLANNER__DIFFUSION_PLANNER_NODE_HPP_
