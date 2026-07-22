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

#ifndef AUTOWARE__TENSORRT_ONEPLANNER__ONEPLANNER_NODE_HPP_
#define AUTOWARE__TENSORRT_ONEPLANNER__ONEPLANNER_NODE_HPP_

#include "autoware/tensorrt_oneplanner/bev_encoder.hpp"
#include "autoware/tensorrt_oneplanner/oneplanner_core.hpp"
#include "autoware/tensorrt_oneplanner/planner_inference.hpp"

#include <autoware/vehicle_info_utils/vehicle_info.hpp>
#include <autoware_utils/ros/polling_subscriber.hpp>
#include <autoware_utils_diagnostics/diagnostics_interface.hpp>
#include <autoware_utils_system/stop_watch.hpp>
#include <cuda_blackboard/cuda_adaptation.hpp>
#include <cuda_blackboard/cuda_blackboard_subscriber.hpp>
#include <cuda_blackboard/cuda_pointcloud2.hpp>
#include <cuda_blackboard/negotiated_types.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <autoware_internal_planning_msgs/msg/candidate_trajectories.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>
#include <vector>

namespace autoware::tensorrt_oneplanner
{

using autoware::vehicle_info_utils::VehicleInfo;
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_utils_diagnostics::DiagnosticsInterface;
using unique_identifier_msgs::msg::UUID;

/**
 * @class OnePlannerNode
 * @brief One-stage end-to-end planner node (OnePlanner).
 *
 * Combines a BEVFusion LiDAR encoder and a diffusion-planner head as two TensorRT
 * engines chained on-device:
 *
 *   pointcloud --> voxelization --> BEV encoder engine --> bev_feature_map (device)
 *                                                              |
 *   odometry / route / map / turn indicators --> planner engine +--> trajectory
 *
 * Inference is driven by the LiDAR pointcloud callback (sensor rate). Unlike the
 * diffusion planner there is no tracked-objects input: obstacle awareness comes from
 * the BEV feature map.
 */
class OnePlannerNode : public rclcpp::Node
{
public:
  explicit OnePlannerNode(const rclcpp::NodeOptions & options);

private:
  void set_up_params();
  void load_models();
  void on_pointcloud(const std::shared_ptr<const cuda_blackboard::CudaPointCloud2> & msg_ptr);
  void on_map(const LaneletMapBin::ConstSharedPtr map_msg);

  /// @brief Throttled diagnostic dump of the (normalized) planner inputs and the raw
  ///        model-output trajectory speed. Only called when `debug_tensor_logging_` is set.
  void log_debug_tensors(
    const InputDataMap & input_data_map, const std::vector<float> & prediction,
    const autoware_planning_msgs::msg::Trajectory & published_trajectory,
    const nav_msgs::msg::Odometry & ego_kinematic_state, int64_t num_input_points);

  // Model / engine parameters
  BevEncoderParams bev_params_;
  std::string planner_onnx_path_;
  std::string args_path_;
  std::string plugins_path_;

  OnePlannerParams params_;

  // When true, log the (normalized) planner-input tensors and the raw model-output
  // trajectory speed every frame (throttled). Off by default; used to diagnose the
  // "speed capped at ~2 m/s" behavior by separating an input-pipeline fault from
  // genuine (possibly out-of-distribution) model output. See CLAUDE.md.
  bool debug_tensor_logging_{false};

  // Processing
  std::unique_ptr<OnePlannerCore> core_;
  std::unique_ptr<BevEncoder> bev_encoder_;
  std::unique_ptr<PlannerInference> planner_inference_;
  cudaStream_t stream_{nullptr};

  // Node elements
  std::unique_ptr<cuda_blackboard::CudaBlackboardSubscriber<cuda_blackboard::CudaPointCloud2>>
    cloud_sub_;
  rclcpp::Subscription<LaneletMapBin>::SharedPtr sub_map_;
  autoware_utils::InterProcessPollingSubscriber<nav_msgs::msg::Odometry> sub_current_odometry_{
    this, "~/input/odometry"};
  autoware_utils::InterProcessPollingSubscriber<geometry_msgs::msg::AccelWithCovarianceStamped>
    sub_current_acceleration_{this, "~/input/acceleration"};
  autoware_utils::InterProcessPollingSubscriber<
    autoware_perception_msgs::msg::TrafficLightGroupArray, autoware_utils::polling_policy::All>
    sub_traffic_signals_{this, "~/input/traffic_signals", rclcpp::QoS{10}};
  autoware_utils::InterProcessPollingSubscriber<autoware_vehicle_msgs::msg::TurnIndicatorsReport>
    sub_turn_indicators_{this, "~/input/turn_indicators"};
  autoware_utils::InterProcessPollingSubscriber<
    autoware_planning_msgs::msg::LaneletRoute, autoware_utils::polling_policy::Newest>
    route_subscriber_{this, "~/input/route", rclcpp::QoS{1}.transient_local()};

  rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr pub_trajectory_;
  rclcpp::Publisher<autoware_internal_planning_msgs::msg::CandidateTrajectories>::SharedPtr
    pub_trajectories_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::TurnIndicatorsCommand>::SharedPtr
    pub_turn_indicators_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float64Stamped>::SharedPtr
    debug_processing_time_pub_;

  std::unique_ptr<DiagnosticsInterface> diagnostics_inference_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_{tf_buffer_};

  UUID generator_uuid_;
  VehicleInfo vehicle_info_;
};

}  // namespace autoware::tensorrt_oneplanner

#endif  // AUTOWARE__TENSORRT_ONEPLANNER__ONEPLANNER_NODE_HPP_
