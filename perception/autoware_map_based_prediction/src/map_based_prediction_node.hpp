// Copyright 2021 Tier IV, Inc.
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

#ifndef AUTOWARE_MAP_BASED_PREDICTION__MAP_BASED_PREDICTION_NODE_HPP_
#define AUTOWARE_MAP_BASED_PREDICTION__MAP_BASED_PREDICTION_NODE_HPP_

#include "autoware/map_based_prediction/data_structure.hpp"
#include "autoware/map_based_prediction/path_generator/path_generator.hpp"
#include "autoware/map_based_prediction/predictor_vehicle/predictor_vehicle.hpp"
#include "autoware/map_based_prediction/predictor_vru/predictor_vru.hpp"

#include <autoware_utils/ros/debug_publisher.hpp>
#include <autoware_utils/ros/diagnostics_interface.hpp>
#include <autoware_utils/ros/polling_subscriber.hpp>
#include <autoware_utils/ros/published_time_publisher.hpp>
#include <autoware_utils/ros/transform_listener.hpp>
#include <autoware_utils/ros/update_param.hpp>
#include <autoware_utils/system/time_keeper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <memory>
#include <optional>
#include <vector>

namespace autoware::map_based_prediction
{

class MapBasedPredictionNode : public rclcpp::Node
{
public:
  explicit MapBasedPredictionNode(const rclcpp::NodeOptions & node_options);

private:
  // ROS communication
  rclcpp::Publisher<PredictedObjects>::SharedPtr pub_objects_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_debug_markers_;
  rclcpp::Subscription<TrackedObjects>::SharedPtr sub_objects_;
  rclcpp::Subscription<LaneletMapBin>::SharedPtr sub_map_;
  autoware_utils::InterProcessPollingSubscriber<TrafficLightGroupArray> sub_traffic_signals_{
    this, "/traffic_signals"};

  // Timing / debug
  std::unique_ptr<autoware_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_;
  std::unique_ptr<autoware_utils::DebugPublisher> processing_time_publisher_;
  std::unique_ptr<autoware_utils::PublishedTimePublisher> published_time_publisher_;
  rclcpp::Publisher<autoware_utils::ProcessingTimeDetail>::SharedPtr
    detailed_processing_time_publisher_;
  std::shared_ptr<autoware_utils::TimeKeeper> time_keeper_;

  // Map (kept for guard check and VRU label override in callback)
  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr_;

  // Parameter update
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rcl_interfaces::msg::SetParametersResult onParam(
    const std::vector<rclcpp::Parameter> & parameters);

  // Transform listener
  autoware_utils::TransformListener transform_listener_{this};

  // Predictors
  std::shared_ptr<PredictorVehicle> predictor_vehicle_;
  std::shared_ptr<PredictorVru> predictor_vru_;

  // Path generator for unknown-class objects
  std::shared_ptr<PathGenerator> path_generator_;

  // Diagnostics
  std::unique_ptr<autoware_utils::DiagnosticsInterface> diagnostics_interface_ptr_;
  double processing_time_tolerance_ms_;
  double processing_time_consecutive_excess_tolerance_ms_;
  std::optional<rclcpp::Time> last_in_time_processing_timestamp_;

  // Parameters shared with callback logic
  double object_buffer_time_length_;
  bool remember_lost_crosswalk_users_;
  PredictionTimeHorizon prediction_time_horizon_;  // .unknown used for unknown-class objects

  // Callbacks
  void mapCallback(const LaneletMapBin::ConstSharedPtr msg);
  void trafficSignalsCallback(const TrafficLightGroupArray::ConstSharedPtr msg);
  void objectsCallback(const TrackedObjects::ConstSharedPtr in_objects);

  // Diagnostics
  void updateDiagnostics(const rclcpp::Time & timestamp, double processing_time_ms);

  // Output
  void publish(
    const PredictedObjects & output,
    const visualization_msgs::msg::MarkerArray & debug_markers) const;
};

}  // namespace autoware::map_based_prediction

#endif  // AUTOWARE_MAP_BASED_PREDICTION__MAP_BASED_PREDICTION_NODE_HPP_
