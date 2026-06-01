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

#include "autoware/map_based_prediction/map_based_prediction_node/callbacks.hpp"

#include "autoware/map_based_prediction/utils.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/lanelet2_utils/geometry.hpp>
#include <autoware/object_recognition_utils/object_recognition_utils.hpp>
#include <autoware_utils/autoware_utils.hpp>
#include <autoware_utils/ros/uuid_helper.hpp>

#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <chrono>
#include <memory>
#include <string>

namespace autoware::map_based_prediction
{
using autoware_utils::ScopedTimeTrack;

Callbacks::Callbacks(rclcpp::Node * node)
: node_(node), sub_traffic_signals_(node, "/traffic_signals"), transform_listener_(node)
{
  stop_watch_ptr_ = std::make_unique<autoware_utils::StopWatch<std::chrono::milliseconds>>();
  stop_watch_ptr_->tic("cyclic_time");
  stop_watch_ptr_->tic("processing_time");
}

void Callbacks::setPredictors(
  std::shared_ptr<PredictorVehicle> predictor_vehicle, std::shared_ptr<PredictorVru> predictor_vru,
  std::shared_ptr<PathGenerator> path_generator)
{
  predictor_vehicle_ = std::move(predictor_vehicle);
  predictor_vru_ = std::move(predictor_vru);
  path_generator_ = std::move(path_generator);
}

void Callbacks::setObjectsPublisher(rclcpp::Publisher<PredictedObjects>::SharedPtr pub_objects)
{
  pub_objects_ = std::move(pub_objects);
}

void Callbacks::setDebugMarkersPublisher(
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_debug_markers)
{
  pub_debug_markers_ = std::move(pub_debug_markers);
}

void Callbacks::setPublishedTimePublisher(
  std::unique_ptr<autoware_utils::PublishedTimePublisher> publisher)
{
  published_time_publisher_ = std::move(publisher);
}

void Callbacks::setProcessingTimePublisher(
  std::unique_ptr<autoware_utils::DebugPublisher> publisher)
{
  processing_time_publisher_ = std::move(publisher);
}

void Callbacks::setDiagnostics(
  std::unique_ptr<autoware_utils::DiagnosticsInterface> diagnostics_interface_ptr,
  double processing_time_tolerance_ms, double processing_time_consecutive_excess_tolerance_ms)
{
  diagnostics_interface_ptr_ = std::move(diagnostics_interface_ptr);
  processing_time_tolerance_ms_ = processing_time_tolerance_ms;
  processing_time_consecutive_excess_tolerance_ms_ =
    processing_time_consecutive_excess_tolerance_ms;
}

void Callbacks::setTimeKeeper(std::shared_ptr<autoware_utils::TimeKeeper> time_keeper)
{
  time_keeper_ = std::move(time_keeper);
}

void Callbacks::mapCallback(const LaneletMapBin::ConstSharedPtr msg)
{
  RCLCPP_DEBUG(node_->get_logger(), "[Map Based Prediction]: Start loading lanelet");

  lanelet_map_ptr_ = autoware::experimental::lanelet2_utils::remove_const(
    autoware::experimental::lanelet2_utils::from_autoware_map_msgs(*msg));

  auto routing_graph_and_traffic_rules =
    autoware::experimental::lanelet2_utils::instantiate_routing_graph_and_traffic_rules(
      lanelet_map_ptr_);

  auto routing_graph_ptr =
    autoware::experimental::lanelet2_utils::remove_const(routing_graph_and_traffic_rules.first);
  auto traffic_rules_ptr = routing_graph_and_traffic_rules.second;

  predictor_vehicle_->setLaneletMap(lanelet_map_ptr_, routing_graph_ptr, traffic_rules_ptr);
  predictor_vru_->setLaneletMap(lanelet_map_ptr_);

  RCLCPP_DEBUG(node_->get_logger(), "[Map Based Prediction]: Map is loaded");
}

void Callbacks::trafficSignalsCallback(const TrafficLightGroupArray::ConstSharedPtr msg)
{
  predictor_vru_->setTrafficSignal(*msg);
}

void Callbacks::objectsCallback(const TrackedObjects::ConstSharedPtr in_objects)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  stop_watch_ptr_->toc("processing_time", true);

  {
    const auto msg = sub_traffic_signals_.take_data();
    if (msg) trafficSignalsCallback(msg);
  }

  if (!lanelet_map_ptr_) return;

  geometry_msgs::msg::TransformStamped::ConstSharedPtr world2map_transform;
  bool is_object_not_in_map_frame = in_objects->header.frame_id != "map";
  if (is_object_not_in_map_frame) {
    world2map_transform = transform_listener_.get_transform(
      "map", in_objects->header.frame_id, in_objects->header.stamp,
      rclcpp::Duration::from_seconds(0.1));
    if (!world2map_transform) return;
  }

  const double objects_detected_time = rclcpp::Time(in_objects->header.stamp).seconds();

  predictor_vehicle_->removeOldHistory(objects_detected_time, params_.object_buffer_time_length);
  predictor_vru_->removeOldKnownMatches(objects_detected_time, params_.object_buffer_time_length);

  PredictedObjects output;
  output.header = in_objects->header;
  output.header.frame_id = "map";

  visualization_msgs::msg::MarkerArray debug_markers;

  predictor_vru_->loadCurrentCrosswalkUsers(*in_objects);

  for (const auto & object : in_objects->objects) {
    TrackedObject transformed_object = object;

    if (is_object_not_in_map_frame) {
      geometry_msgs::msg::PoseStamped pose_in_map;
      geometry_msgs::msg::PoseStamped pose_orig;
      pose_orig.pose = object.kinematics.pose_with_covariance.pose;
      tf2::doTransform(pose_orig, pose_in_map, *world2map_transform);
      transformed_object.kinematics.pose_with_covariance.pose = pose_in_map.pose;
    }

    const auto & label_ =
      autoware::object_recognition_utils::getHighestProbLabel(transformed_object.classification);
    const auto label = utils::changeVRULabelForPrediction(label_, object, lanelet_map_ptr_);

    switch (label) {
      case ObjectClassification::PEDESTRIAN:
      case ObjectClassification::BICYCLE: {
        output.objects.emplace_back(predictor_vru_->predict(output.header, transformed_object));
        break;
      }
      case ObjectClassification::CAR:
      case ObjectClassification::BUS:
      case ObjectClassification::TRAILER:
      case ObjectClassification::MOTORCYCLE:
      case ObjectClassification::TRUCK: {
        const auto predicted_object_opt = predictor_vehicle_->predict(
          output.header, transformed_object, objects_detected_time, debug_markers);
        if (predicted_object_opt) output.objects.push_back(predicted_object_opt.value());
        break;
      }
      default: {
        auto predicted_unknown_object = utils::convertToPredictedObject(transformed_object);
        PredictedPath predicted_path = path_generator_->generatePathForNonVehicleObject(
          transformed_object, params_.prediction_time_horizon_unknown);
        predicted_path.confidence = 1.0;
        predicted_unknown_object.kinematics.predicted_paths.push_back(predicted_path);
        output.objects.push_back(predicted_unknown_object);
        break;
      }
    }
  }

  if (params_.remember_lost_crosswalk_users) {
    PredictedObjects retrieved_objects = predictor_vru_->retrieveUndetectedObjects();
    output.objects.insert(
      output.objects.end(), retrieved_objects.objects.begin(), retrieved_objects.objects.end());
  }

  publish(output, debug_markers);

  const auto processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
  const auto cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);

  updateDiagnostics(output.header.stamp, processing_time_ms);

  if (processing_time_publisher_) {
    processing_time_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/cyclic_time_ms", cyclic_time_ms);
    processing_time_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time_ms", processing_time_ms);
  }
}

void Callbacks::publish(
  const PredictedObjects & output, const visualization_msgs::msg::MarkerArray & debug_markers) const
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  pub_objects_->publish(output);
  if (published_time_publisher_)
    published_time_publisher_->publish_if_subscribed(pub_objects_, output.header.stamp);
  if (pub_debug_markers_) pub_debug_markers_->publish(debug_markers);
}

}  // namespace autoware::map_based_prediction
