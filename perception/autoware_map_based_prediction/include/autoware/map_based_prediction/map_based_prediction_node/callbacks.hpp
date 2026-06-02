// Copyright 2021 TIER IV, Inc.
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

#ifndef AUTOWARE__MAP_BASED_PREDICTION__MAP_BASED_PREDICTION_NODE__CALLBACKS_HPP_
#define AUTOWARE__MAP_BASED_PREDICTION__MAP_BASED_PREDICTION_NODE__CALLBACKS_HPP_

#include "autoware/map_based_prediction/data_structure.hpp"
#include "autoware/map_based_prediction/params.hpp"
#include "autoware/map_based_prediction/path_generator/path_generator.hpp"
#include "autoware/map_based_prediction/predictor_vehicle/predictor_vehicle.hpp"
#include "autoware/map_based_prediction/predictor_vru/predictor_vru.hpp"

#include <autoware_utils/ros/polling_subscriber.hpp>
#include <autoware_utils/ros/transform_listener.hpp>
#include <autoware_utils/system/time_keeper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include <chrono>
#include <memory>

namespace autoware::map_based_prediction
{

class Diagnostics;

struct NodeState
{
  NodeParams params;
  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr;
  std::shared_ptr<PredictorVehicle> predictor_vehicle;
  std::shared_ptr<PredictorVru> predictor_vru;
  std::shared_ptr<PathGenerator> path_generator;
  std::shared_ptr<autoware_utils::TimeKeeper> time_keeper;
};

class MapCallback
{
public:
  explicit MapCallback(rclcpp::Node * node, NodeState & state);

  void mapCallback(const LaneletMapBin::ConstSharedPtr msg);

private:
  rclcpp::Node * node_;
  NodeState & state_;
};

class ObjectsCallback
{
public:
  explicit ObjectsCallback(rclcpp::Node * node, NodeState & state);

  void setObjectsPublisher(rclcpp::Publisher<PredictedObjects>::SharedPtr pub_objects);
  void setDebugMarkersPublisher(
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_debug_markers);
  void setDiagnostics(Diagnostics * diagnostics);

  void objectsCallback(const TrackedObjects::ConstSharedPtr in_objects);

private:
  rclcpp::Node * node_;
  NodeState & state_;

  autoware_utils::InterProcessPollingSubscriber<TrafficLightGroupArray> sub_traffic_signals_;
  autoware_utils::TransformListener transform_listener_;
  std::unique_ptr<autoware_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_;

  rclcpp::Publisher<PredictedObjects>::SharedPtr pub_objects_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_debug_markers_;

  Diagnostics * diagnostics_{};

  void trafficSignalsCallback(const TrafficLightGroupArray::ConstSharedPtr msg);
  void publish(
    const PredictedObjects & output,
    const visualization_msgs::msg::MarkerArray & debug_markers) const;
};

}  // namespace autoware::map_based_prediction

#endif  // AUTOWARE__MAP_BASED_PREDICTION__MAP_BASED_PREDICTION_NODE__CALLBACKS_HPP_
