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

#include "autoware/diffusion_planner/conversion/agent.hpp"
#include "autoware/diffusion_planner/conversion/lanelet.hpp"
#include "autoware/diffusion_planner/inference/tensorrt_inference.hpp"
#include "autoware/diffusion_planner/postprocessing/turn_indicator_manager.hpp"
#include "autoware/diffusion_planner/preprocessing/lane_segments.hpp"
#include "autoware/diffusion_planner/preprocessing/traffic_signals.hpp"
#include "autoware/diffusion_planner/utils/arg_reader.hpp"

#include <Eigen/Dense>
#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/vehicle_info_utils/vehicle_info.hpp>
#include <autoware_utils/ros/polling_subscriber.hpp>
#include <autoware_utils/ros/update_param.hpp>
#include <autoware_utils/system/time_keeper.hpp>
#include <autoware_utils_diagnostics/diagnostics_interface.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>

#include <autoware_internal_planning_msgs/msg/candidate_trajectories.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_perception_msgs/msg/traffic_signal.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <Eigen/src/Core/Matrix.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRules.h>

#include <deque>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::diffusion_planner
{
using autoware::diffusion_planner::AgentData;
using autoware_internal_planning_msgs::msg::CandidateTrajectories;
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::TrackedObjects;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using autoware_vehicle_msgs::msg::TurnIndicatorsCommand;
using autoware_vehicle_msgs::msg::TurnIndicatorsReport;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using nav_msgs::msg::Odometry;
using HADMapBin = autoware_map_msgs::msg::LaneletMapBin;
using InputDataMap = std::unordered_map<std::string, std::vector<float>>;
using autoware::vehicle_info_utils::VehicleInfo;
using autoware_utils_diagnostics::DiagnosticsInterface;
using builtin_interfaces::msg::Duration;
using builtin_interfaces::msg::Time;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using preprocess::TrafficSignalStamped;
using rcl_interfaces::msg::SetParametersResult;
using std_msgs::msg::ColorRGBA;
using unique_identifier_msgs::msg::UUID;
using utils::NormalizationMap;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

struct FrameContext
{
  nav_msgs::msg::Odometry ego_kinematic_state;
  geometry_msgs::msg::AccelWithCovarianceStamped ego_acceleration;
  Eigen::Matrix4d ego_to_map_transform;
  std::vector<AgentHistory> ego_centric_neighbor_histories;
  rclcpp::Time frame_time;
};

struct DiffusionPlannerParams
{
  std::string model_path;
  std::string args_path;
  std::string plugins_path;
  bool build_only;
  double planning_frequency_hz;
  bool ignore_neighbors;
  bool ignore_unknown_neighbors;
  bool predict_neighbor_trajectory;
  double traffic_light_group_msg_timeout_seconds;
  int batch_size;
  std::vector<double> temperature_list;
  int64_t velocity_smoothing_window;
  double stopping_threshold;
  float turn_indicator_keep_offset;
  double turn_indicator_hold_duration;
  bool shift_x;
};

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
 * - on_timer: Timer callback for periodic processing and publishing.
 * - on_map: Callback for receiving and processing map data.
 * - publish_debug_markers: Publish visualization markers for debugging.
 * - publish_predictions: Publish model predictions.
 * - on_parameter: Callback for dynamic parameter updates.
 * - create_frame_context: Prepare frame context for inference.
 * - create_input_data: Build model input tensors from frame context.
 *
 * @section Internal State
 * @brief
 * - agent_data_: Optional input data for inference.
 * - params_, debug_params_, normalization_map_: Node and debug parameters, normalization info.
 * - Lanelet map and routing members: route_ptr_, lane_segment_context_.
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
   * @brief Publish model predictions.
   * @param predictions Output from the model.
   * @param frame_context Context of the current frame.
   */
  void publish_predictions(
    const std::vector<float> & predictions, const FrameContext & frame_context,
    const rclcpp::Time & timestamp) const;

  /**
   * @brief Callback for dynamic parameter updates.
   * @param parameters Updated parameters.
   * @return Result of parameter update.
   */
  SetParametersResult on_parameter(const std::vector<rclcpp::Parameter> & parameters);

  /**
   * @brief Prepare input data for inference.
   * @return FrameContext containing preprocessed data.
   */
  std::optional<FrameContext> create_frame_context();

  /**
   * @brief Build model input tensors from frame context.
   * @param frame_context Preprocessed frame context.
   * @return Map of input data for the model.
   */
  InputDataMap create_input_data(const FrameContext & frame_context);

  // Inference engine
  std::unique_ptr<TensorrtInference> tensorrt_inference_{nullptr};

  // history data
  std::deque<nav_msgs::msg::Odometry> ego_history_;
  std::deque<TurnIndicatorsReport> turn_indicators_history_;
  AgentData agent_data_;
  std::map<lanelet::Id, TrafficSignalStamped> traffic_light_id_map_;

  // Node parameters
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  DiffusionPlannerParams params_;
  DiffusionPlannerDebugParams debug_params_;
  NormalizationMap normalization_map_;

  // Lanelet map
  LaneletRoute::ConstSharedPtr route_ptr_;
  std::unique_ptr<preprocess::LaneSegmentContext> lane_segment_context_;

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
  postprocess::TurnIndicatorManager turn_indicator_manager_{
    rclcpp::Duration::from_seconds(0.0), 0.0f};
};

}  // namespace autoware::diffusion_planner
#endif  // AUTOWARE__DIFFUSION_PLANNER__DIFFUSION_PLANNER_NODE_HPP_
