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
#include "autoware/diffusion_planner/preprocessing/lane_segments.hpp"
#include "autoware/diffusion_planner/preprocessing/traffic_signals.hpp"
#include "autoware/diffusion_planner/utils/arg_reader.hpp"

#include <Eigen/Dense>
#include <autoware/cuda_utils/cuda_unique_ptr.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <autoware/tensorrt_common/tensorrt_common.hpp>
#include <autoware/tensorrt_common/tensorrt_conv_calib.hpp>
#include <autoware/tensorrt_common/utils.hpp>
#include <autoware/vehicle_info_utils/vehicle_info.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_utils/ros/polling_subscriber.hpp>
#include <autoware_utils/ros/update_param.hpp>
#include <autoware_utils/system/time_keeper.hpp>
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
using geometry_msgs::msg::AccelWithCovarianceStamped;
using nav_msgs::msg::Odometry;
using HADMapBin = autoware_map_msgs::msg::LaneletMapBin;
using InputDataMap = std::unordered_map<std::string, std::vector<float>>;
using autoware::route_handler::RouteHandler;
using autoware::vehicle_info_utils::VehicleInfo;
using builtin_interfaces::msg::Duration;
using builtin_interfaces::msg::Time;
using geometry_msgs::msg::Point;
using preprocess::ColLaneIDMaps;
using preprocess::TrafficSignalStamped;
using rcl_interfaces::msg::SetParametersResult;
using std_msgs::msg::ColorRGBA;
using unique_identifier_msgs::msg::UUID;
using utils::NormalizationMap;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;
// TensorRT
using autoware::cuda_utils::CudaUniquePtr;
using autoware::tensorrt_common::TrtConvCalib;

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
  bool update_traffic_light_group_info;
  bool keep_last_traffic_light_group_info;
  double traffic_light_group_msg_timeout_seconds;
};
struct DiffusionPlannerDebugParams
{
  bool publish_debug_route{false};
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
 * - load_model: Load ONNX model from file.
 * - publish_debug_markers: Publish visualization markers for debugging.
 * - publish_predictions: Publish model predictions.
 * - do_inference: Run inference on input data and return predictions.
 * - on_parameter: Callback for dynamic parameter updates.
 * - create_input_data: Prepare input data for inference.
 * - get_ego_centric_agent_data: Extract ego-centric agent data from tracked objects.
 * - create_trajectory: Convert predictions to a trajectory in map coordinates.
 * - create_ego_agent_past: Create a representation of the ego agent's past trajectory.
 *
 * @section Internal State
 * @brief
 * - route_handler_: Handles route-related operations.
 * - transforms_: Stores transformation matrices between map and ego frames.
 * - ego_kinematic_state_: Current odometry state of the ego vehicle.
 * - ONNX Runtime members: env_, session_options_, session_, allocator_, cuda_options_.
 * - agent_data_: Optional input data for inference.
 * - params_, debug_params_, normalization_map_: Node and debug parameters, normalization info.
 * - Lanelet map and routing members: route_ptr_, lanelet_map_ptr_, routing_graph_ptr_,
 * traffic_rules_ptr_, lanelet_converter_ptr_, lane_segments_, map_lane_segments_matrix_,
 * col_id_mapping_, is_map_loaded_.
 * - ROS 2 node elements: timer_, publishers, subscriptions, and time_keeper_.
 * - generator_uuid_: Unique identifier for the planner instance.
 * - vehicle_info_: Vehicle-specific parameters.
 */
class DiffusionPlanner : public rclcpp::Node
{
public:
  explicit DiffusionPlanner(const rclcpp::NodeOptions & options);
  ~DiffusionPlanner();
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
   * @brief Init TensorRT pointers.
   */
  void init_pointers();

  /**
   * @brief Load the TensorRT engine from the specified model path.
   * This function sets up the TensorRT engine with the required input and output shapes.
   * It also initializes the TrtConvCalib for calibration if needed.
   * @param model_path Path to the TensorRT model file.
   */
  void load_engine(const std::string & model_path);

  /**
   * @brief Publish visualization markers for debugging.
   * @param input_data_map Input data used for inference.
   */
  void publish_debug_markers(InputDataMap & input_data_map) const;

  /**
   * @brief Publish model predictions.
   * @param predictions Output from the model.
   */
  void publish_predictions(const std::vector<float> & predictions) const;

  /**
   * @brief Run inference on input data output is stored on member output_d_.
   * @param input_data_map Input data for the model.
   */
  std::vector<float> do_inference_trt(InputDataMap & input_data_map);

  /**
   * @brief Callback for dynamic parameter updates.
   * @param parameters Updated parameters.
   * @return Result of parameter update.
   */
  SetParametersResult on_parameter(const std::vector<rclcpp::Parameter> & parameters);

  /**
   * @brief Prepare input data for inference.
   * @return Map of input data for the model.
   */
  InputDataMap create_input_data();

  // preprocessing
  std::shared_ptr<RouteHandler> route_handler_{std::make_shared<RouteHandler>()};
  std::pair<Eigen::Matrix4f, Eigen::Matrix4f> transforms_;
  AgentData get_ego_centric_agent_data(
    const TrackedObjects & objects, const Eigen::Matrix4f & map_to_ego_transform);

  /**
   * @brief Create ego agent past tensor from ego history.
   * @param map_to_ego_transform Transformation matrix from map to ego frame.
   * @return Vector of float values representing ego agent past.
   */
  std::vector<float> create_ego_agent_past(const Eigen::Matrix4f & map_to_ego_transform);

  // current state
  Odometry ego_kinematic_state_;

  // ego history for ego_agent_past
  std::deque<Odometry> ego_history_;

  // TensorRT
  std::unique_ptr<TrtConvCalib> trt_common_;
  std::unique_ptr<autoware::tensorrt_common::TrtCommon> network_trt_ptr_{nullptr};
  // For float inputs and output
  CudaUniquePtr<float[]> ego_history_d_;
  CudaUniquePtr<float[]> ego_current_state_d_;
  CudaUniquePtr<float[]> neighbor_agents_past_d_;
  CudaUniquePtr<float[]> static_objects_d_;
  CudaUniquePtr<float[]> lanes_d_;
  CudaUniquePtr<bool[]> lanes_has_speed_limit_d_;
  CudaUniquePtr<float[]> lanes_speed_limit_d_;
  CudaUniquePtr<float[]> route_lanes_d_;
  CudaUniquePtr<bool[]> route_lanes_has_speed_limit_d_;
  CudaUniquePtr<float[]> route_lanes_speed_limit_d_;
  CudaUniquePtr<float[]> goal_pose_d_;
  CudaUniquePtr<float[]> ego_shape_d_;
  CudaUniquePtr<float[]> output_d_;                // shape: [1, 11, 80, 4]
  CudaUniquePtr<float[]> turn_indicator_logit_d_;  // shape: [1, 4]
  cudaStream_t stream_{nullptr};

  // Model input data
  std::optional<AgentData> agent_data_{std::nullopt};

  // Node parameters
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  DiffusionPlannerParams params_;
  DiffusionPlannerDebugParams debug_params_;
  NormalizationMap normalization_map_;

  // Lanelet map
  LaneletRoute::ConstSharedPtr route_ptr_;
  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr_;
  std::shared_ptr<lanelet::routing::RoutingGraph> routing_graph_ptr_;
  std::shared_ptr<lanelet::traffic_rules::TrafficRules> traffic_rules_ptr_;
  std::map<lanelet::Id, TrafficSignalStamped> traffic_light_id_map_;
  std::unique_ptr<LaneletConverter> lanelet_converter_ptr_;
  std::vector<LaneSegment> lane_segments_;
  Eigen::MatrixXf map_lane_segments_matrix_;
  ColLaneIDMaps col_id_mapping_;
  bool is_map_loaded_{false};

  // Node elements
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<autoware_utils::ProcessingTimeDetail>::SharedPtr
    debug_processing_time_detail_pub_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_trajectory_{nullptr};
  rclcpp::Publisher<CandidateTrajectories>::SharedPtr pub_trajectories_{nullptr};
  rclcpp::Publisher<PredictedObjects>::SharedPtr pub_objects_{nullptr};
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_lane_marker_{nullptr};
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_route_marker_{nullptr};
  mutable std::shared_ptr<autoware_utils::TimeKeeper> time_keeper_{nullptr};
  autoware_utils::InterProcessPollingSubscriber<Odometry> sub_current_odometry_{
    this, "~/input/odometry"};
  autoware_utils::InterProcessPollingSubscriber<AccelWithCovarianceStamped>
    sub_current_acceleration_{this, "~/input/acceleration"};
  autoware_utils::InterProcessPollingSubscriber<TrackedObjects> sub_tracked_objects_{
    this, "~/input/tracked_objects"};
  autoware_utils::InterProcessPollingSubscriber<
    autoware_perception_msgs::msg::TrafficLightGroupArray>
    sub_traffic_signals_{this, "~/input/traffic_signals"};
  autoware_utils::InterProcessPollingSubscriber<
    LaneletRoute, autoware_utils::polling_policy::Newest>
    route_subscriber_{this, "~/input/route", rclcpp::QoS{1}.transient_local()};
  autoware_utils::InterProcessPollingSubscriber<
    LaneletMapBin, autoware_utils::polling_policy::Newest>
    vector_map_subscriber_{this, "~/input/vector_map", rclcpp::QoS{1}.transient_local()};
  rclcpp::Subscription<HADMapBin>::SharedPtr sub_map_;
  UUID generator_uuid_;
  VehicleInfo vehicle_info_;
};

}  // namespace autoware::diffusion_planner
#endif  // AUTOWARE__DIFFUSION_PLANNER__DIFFUSION_PLANNER_NODE_HPP_
