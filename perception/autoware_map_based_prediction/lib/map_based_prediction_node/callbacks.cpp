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

#include "autoware/map_based_prediction/map_based_prediction_node/callbacks.hpp"

#include "autoware/map_based_prediction/map_based_prediction_node/diagnostics.hpp"
#include "autoware/map_based_prediction/utils.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/lanelet2_utils/geometry.hpp>
#include <autoware/object_recognition_utils/object_recognition_utils.hpp>
#include <autoware_utils/autoware_utils.hpp>
#include <autoware_utils/ros/uuid_helper.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::map_based_prediction
{
using autoware_utils::ScopedTimeTrack;

namespace
{

void appendRelevanceMarker(
  const TrackedObject & object, const Relevance relevance, const std_msgs::msg::Header & header,
  visualization_msgs::msg::MarkerArray & debug_markers)
{
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = "relevance";
  marker.id = static_cast<int32_t>(debug_markers.markers.size());
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose = object.kinematics.pose_with_covariance.pose;
  marker.pose.position.z += 2.0;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color.a = 0.8;
  if (relevance == Relevance::HIGH) {
    marker.color.g = 1.0;
  } else {
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
  }
  marker.lifetime = rclcpp::Duration::from_seconds(0.2);
  debug_markers.markers.push_back(marker);
}

}  // namespace

// ---------------------------------------------------------------------------
// MapCallback
// ---------------------------------------------------------------------------

MapCallback::MapCallback(autoware::agnocast_wrapper::Node * node, NodeState & state)
: node_(node), state_(state)
{
}

void MapCallback::mapCallback(const AUTOWARE_MESSAGE_CONST_SHARED_PTR(LaneletMapBin) & msg)
{
  RCLCPP_DEBUG(node_->get_logger(), "[Map Based Prediction]: Start loading lanelet");

  state_.lanelet_map_ptr = autoware::experimental::lanelet2_utils::remove_const(
    autoware::experimental::lanelet2_utils::from_autoware_map_msgs(*msg));

  auto routing_graph_and_traffic_rules =
    autoware::experimental::lanelet2_utils::instantiate_routing_graph_and_traffic_rules(
      state_.lanelet_map_ptr);

  auto routing_graph_ptr =
    autoware::experimental::lanelet2_utils::remove_const(routing_graph_and_traffic_rules.first);
  auto traffic_rules_ptr = routing_graph_and_traffic_rules.second;

  state_.predictor_vehicle->setLaneletMap(
    state_.lanelet_map_ptr, routing_graph_ptr, traffic_rules_ptr);
  state_.predictor_vru->setLaneletMap(state_.lanelet_map_ptr);

  RCLCPP_DEBUG(node_->get_logger(), "[Map Based Prediction]: Map is loaded");
}

// ---------------------------------------------------------------------------
// ObjectsCallback
// ---------------------------------------------------------------------------

ObjectsCallback::ObjectsCallback(autoware::agnocast_wrapper::Node * node, NodeState & state)
: state_(state), transform_listener_(node)
{
  sub_traffic_signals_ =
    node->create_polling_subscriber<TrafficLightGroupArray>("/traffic_signals", rclcpp::QoS{1});
  sub_ego_trajectory_ =
    node->create_polling_subscriber<Trajectory>("~/input/ego_trajectory", rclcpp::QoS{1});
  sub_ego_odometry_ =
    node->create_polling_subscriber<Odometry>("~/input/ego_odometry", rclcpp::QoS{1});
  stop_watch_ptr_ = std::make_unique<autoware_utils::StopWatch<std::chrono::milliseconds>>();
  stop_watch_ptr_->tic("cyclic_time");
  stop_watch_ptr_->tic("processing_time");
}

void ObjectsCallback::setObjectsPublisher(AUTOWARE_PUBLISHER_PTR(PredictedObjects) pub_objects)
{
  pub_objects_ = std::move(pub_objects);
}

void ObjectsCallback::setDebugMarkersPublisher(
  AUTOWARE_PUBLISHER_PTR(visualization_msgs::msg::MarkerArray) pub_debug_markers)
{
  pub_debug_markers_ = std::move(pub_debug_markers);
}

void ObjectsCallback::setDiagnostics(Diagnostics * diagnostics)
{
  diagnostics_ = diagnostics;
}

void ObjectsCallback::trafficSignalsCallback(
  const AUTOWARE_MESSAGE_CONST_SHARED_PTR(TrafficLightGroupArray) & msg)
{
  state_.predictor_vru->setTrafficSignal(*msg);
}

void ObjectsCallback::updateRelevanceClassifier(const double objects_detected_time)
{
  if (!state_.relevance_classifier || !state_.relevance_classifier->getParams().enable) {
    return;
  }

  const auto trajectory_msg = sub_ego_trajectory_->take_data();
  const auto odometry_msg = sub_ego_odometry_->take_data();
  if (trajectory_msg && odometry_msg) {
    std::vector<geometry_msgs::msg::Point> trajectory_points;
    trajectory_points.reserve(trajectory_msg->points.size());
    for (const auto & point : trajectory_msg->points) {
      trajectory_points.push_back(point.pose.position);
    }
    state_.relevance_classifier->setEgoData(
      trajectory_points, odometry_msg->pose.pose.position,
      rclcpp::Time(trajectory_msg->header.stamp).seconds());
  }
  state_.relevance_classifier->removeOldHistory(
    objects_detected_time, state_.params.object_buffer_time_length);
}

PredictedObject ObjectsCallback::predictLowFidelityVehicle(const TrackedObject & object) const
{
  auto predicted_object = utils::convertToPredictedObject(object);
  PredictedPath predicted_path = state_.path_generator->generatePathForNonVehicleObject(
    object, state_.relevance_classifier->getParams().low_fidelity_time_horizon);
  predicted_path.confidence = 1.0;
  predicted_object.kinematics.predicted_paths.push_back(predicted_path);
  return predicted_object;
}

void ObjectsCallback::objectsCallback(
  const AUTOWARE_MESSAGE_CONST_SHARED_PTR(TrackedObjects) & in_objects)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (state_.time_keeper) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *state_.time_keeper);

  stop_watch_ptr_->toc("processing_time", true);

  {
    const auto msg = sub_traffic_signals_->take_data();
    if (msg) trafficSignalsCallback(msg);
  }

  if (!state_.lanelet_map_ptr) return;

  geometry_msgs::msg::TransformStamped::ConstSharedPtr world2map_transform;
  const bool is_object_not_in_map_frame = in_objects->header.frame_id != "map";
  if (is_object_not_in_map_frame) {
    world2map_transform = transform_listener_.get_transform(
      "map", in_objects->header.frame_id, in_objects->header.stamp,
      rclcpp::Duration::from_seconds(0.1));
    if (!world2map_transform) return;
  }

  const double objects_detected_time = rclcpp::Time(in_objects->header.stamp).seconds();

  updateRelevanceClassifier(objects_detected_time);

  state_.predictor_vehicle->removeOldHistory(
    objects_detected_time, state_.params.object_buffer_time_length);
  state_.predictor_vru->removeOldKnownMatches(
    objects_detected_time, state_.params.object_buffer_time_length);

  auto output_msg = ALLOCATE_OUTPUT_MESSAGE_UNIQUE(pub_objects_);
  PredictedObjects & output = *output_msg;
  output.header = in_objects->header;
  output.header.frame_id = "map";

  visualization_msgs::msg::MarkerArray debug_markers;

  state_.predictor_vru->loadCurrentCrosswalkUsers(*in_objects);

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
    const auto label = utils::changeVRULabelForPrediction(label_, object, state_.lanelet_map_ptr);

    switch (label) {
      case ObjectClassification::PEDESTRIAN:
      case ObjectClassification::BICYCLE: {
        output.objects.emplace_back(
          state_.predictor_vru->predict(output.header, transformed_object));
        break;
      }
      case ObjectClassification::CAR:
      case ObjectClassification::BUS:
      case ObjectClassification::TRAILER:
      case ObjectClassification::MOTORCYCLE:
      case ObjectClassification::TRUCK: {
        // Planning-aware fidelity allocation: vehicles with low relevance to the ego
        // planned trajectory receive a lightweight constant-velocity prediction instead
        // of the full lanelet-based multi-mode prediction. VRU classes are always
        // predicted with full fidelity.
        const auto relevance =
          state_.relevance_classifier
            ? state_.relevance_classifier->classify(transformed_object, objects_detected_time)
            : Relevance::HIGH;
        if (
          pub_debug_markers_ && state_.relevance_classifier &&
          state_.relevance_classifier->getParams().enable) {
          appendRelevanceMarker(transformed_object, relevance, output.header, debug_markers);
        }
        if (relevance == Relevance::LOW) {
          output.objects.push_back(predictLowFidelityVehicle(transformed_object));
          break;
        }
        const auto predicted_object_opt = state_.predictor_vehicle->predict(
          output.header, transformed_object, objects_detected_time,
          pub_debug_markers_ ? &debug_markers : nullptr);
        if (predicted_object_opt) output.objects.push_back(predicted_object_opt.value());
        break;
      }
      default: {
        auto predicted_unknown_object = utils::convertToPredictedObject(transformed_object);
        PredictedPath predicted_path = state_.path_generator->generatePathForNonVehicleObject(
          transformed_object, state_.params.prediction_time_horizon_unknown);
        predicted_path.confidence = 1.0;
        predicted_unknown_object.kinematics.predicted_paths.push_back(predicted_path);
        output.objects.push_back(predicted_unknown_object);
        break;
      }
    }
  }

  if (state_.params.remember_lost_crosswalk_users) {
    PredictedObjects retrieved_objects = state_.predictor_vru->retrieveUndetectedObjects();
    output.objects.insert(
      output.objects.end(), retrieved_objects.objects.begin(), retrieved_objects.objects.end());
  }

  publish(std::move(output_msg), debug_markers);

  const auto processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
  const auto cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);

  if (diagnostics_) diagnostics_->update(output.header.stamp, processing_time_ms, cyclic_time_ms);
}

void ObjectsCallback::publish(
  AUTOWARE_MESSAGE_UNIQUE_PTR(PredictedObjects) output,
  const visualization_msgs::msg::MarkerArray & debug_markers) const
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (state_.time_keeper) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *state_.time_keeper);

  const auto stamp = output->header.stamp;
  pub_objects_->publish(std::move(output));
  if (diagnostics_) diagnostics_->publishIfSubscribed<PredictedObjects>(pub_objects_, stamp);
  if (pub_debug_markers_) {
    auto debug_markers_msg = ALLOCATE_OUTPUT_MESSAGE_UNIQUE(pub_debug_markers_);
    *debug_markers_msg = debug_markers;
    pub_debug_markers_->publish(std::move(debug_markers_msg));
  }
}

}  // namespace autoware::map_based_prediction
