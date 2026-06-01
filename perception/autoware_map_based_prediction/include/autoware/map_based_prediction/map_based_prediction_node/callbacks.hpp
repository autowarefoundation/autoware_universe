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

#ifndef AUTOWARE__MAP_BASED_PREDICTION__MAP_BASED_PREDICTION_NODE__CALLBACKS_HPP_
#define AUTOWARE__MAP_BASED_PREDICTION__MAP_BASED_PREDICTION_NODE__CALLBACKS_HPP_

#include "autoware/map_based_prediction/data_structure.hpp"
#include "autoware/map_based_prediction/path_generator/path_generator.hpp"
#include "autoware/map_based_prediction/predictor_vehicle/predictor_vehicle.hpp"
#include "autoware/map_based_prediction/predictor_vru/predictor_vru.hpp"

#include <autoware_utils/ros/debug_publisher.hpp>
#include <autoware_utils/ros/diagnostics_interface.hpp>
#include <autoware_utils/ros/polling_subscriber.hpp>
#include <autoware_utils/ros/published_time_publisher.hpp>
#include <autoware_utils/ros/transform_listener.hpp>
#include <autoware_utils/system/time_keeper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include <chrono>
#include <memory>
#include <optional>

namespace autoware::map_based_prediction
{

class Callbacks
{
public:
  struct Params
  {
    double object_buffer_time_length{};
    bool remember_lost_crosswalk_users{};
    double prediction_time_horizon_unknown{};
  };

  explicit Callbacks(rclcpp::Node * node);

  void setParams(const Params & params) { params_ = params; }

  void setPredictors(
    std::shared_ptr<PredictorVehicle> predictor_vehicle,
    std::shared_ptr<PredictorVru> predictor_vru, std::shared_ptr<PathGenerator> path_generator);

  void setObjectsPublisher(rclcpp::Publisher<PredictedObjects>::SharedPtr pub_objects);
  void setDebugMarkersPublisher(
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_debug_markers);
  void setPublishedTimePublisher(std::unique_ptr<autoware_utils::PublishedTimePublisher> publisher);
  void setProcessingTimePublisher(std::unique_ptr<autoware_utils::DebugPublisher> publisher);

  void setDiagnostics(
    std::unique_ptr<autoware_utils::DiagnosticsInterface> diagnostics_interface_ptr,
    double processing_time_tolerance_ms, double processing_time_consecutive_excess_tolerance_ms);

  void setTimeKeeper(std::shared_ptr<autoware_utils::TimeKeeper> time_keeper);

  void mapCallback(const LaneletMapBin::ConstSharedPtr msg);
  void trafficSignalsCallback(const TrafficLightGroupArray::ConstSharedPtr msg);
  void objectsCallback(const TrackedObjects::ConstSharedPtr in_objects);

private:
  rclcpp::Node * node_;
  autoware_utils::InterProcessPollingSubscriber<TrafficLightGroupArray> sub_traffic_signals_;
  autoware_utils::TransformListener transform_listener_;
  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr_;

  Params params_{};

  std::shared_ptr<PredictorVehicle> predictor_vehicle_;
  std::shared_ptr<PredictorVru> predictor_vru_;
  std::shared_ptr<PathGenerator> path_generator_;

  std::shared_ptr<autoware_utils::TimeKeeper> time_keeper_;
  std::unique_ptr<autoware_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_;

  rclcpp::Publisher<PredictedObjects>::SharedPtr pub_objects_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_debug_markers_;
  std::unique_ptr<autoware_utils::PublishedTimePublisher> published_time_publisher_;
  std::unique_ptr<autoware_utils::DebugPublisher> processing_time_publisher_;

  std::unique_ptr<autoware_utils::DiagnosticsInterface> diagnostics_interface_ptr_;
  double processing_time_tolerance_ms_{};
  double processing_time_consecutive_excess_tolerance_ms_{};
  std::optional<rclcpp::Time> last_in_time_processing_timestamp_;

  void publish(
    const PredictedObjects & output,
    const visualization_msgs::msg::MarkerArray & debug_markers) const;
  void updateDiagnostics(const rclcpp::Time & timestamp, double processing_time_ms);
};

}  // namespace autoware::map_based_prediction

#endif  // AUTOWARE__MAP_BASED_PREDICTION__MAP_BASED_PREDICTION_NODE__CALLBACKS_HPP_
