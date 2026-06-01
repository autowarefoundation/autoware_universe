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
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>

#include <chrono>
#include <memory>
#include <string>

namespace autoware::map_based_prediction
{
using autoware_utils::ScopedTimeTrack;

void MapBasedPredictionNode::mapCallback(const LaneletMapBin::ConstSharedPtr msg)
{
  RCLCPP_DEBUG(get_logger(), "[Map Based Prediction]: Start loading lanelet");
  lanelet_map_ptr_ = autoware::experimental::lanelet2_utils::remove_const(
    autoware::experimental::lanelet2_utils::from_autoware_map_msgs(*msg));

  auto routing_graph_and_traffic_rules =
    autoware::experimental::lanelet2_utils::instantiate_routing_graph_and_traffic_rules(
      lanelet_map_ptr_);

  routing_graph_ptr_ =
    autoware::experimental::lanelet2_utils::remove_const(routing_graph_and_traffic_rules.first);
  traffic_rules_ptr_ = routing_graph_and_traffic_rules.second;

  lru_cache_of_convert_path_type_.clear();  // clear cache
  RCLCPP_DEBUG(get_logger(), "[Map Based Prediction]: Map is loaded");

  predictor_vru_->setLaneletMap(lanelet_map_ptr_);
}

void MapBasedPredictionNode::trafficSignalsCallback(
  const TrafficLightGroupArray::ConstSharedPtr msg)
{
  // load traffic signals to the predictor
  predictor_vru_->setTrafficSignal(*msg);
}

void MapBasedPredictionNode::objectsCallback(const TrackedObjects::ConstSharedPtr in_objects)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  stop_watch_ptr_->toc("processing_time", true);

  // take traffic_signal
  {
    const auto msg = sub_traffic_signals_.take_data();
    if (msg) {
      trafficSignalsCallback(msg);
    }
  }

  // Guard for map pointer and frame transformation
  if (!lanelet_map_ptr_) {
    return;
  }

  // get world to map transform
  geometry_msgs::msg::TransformStamped::ConstSharedPtr world2map_transform;
  bool is_object_not_in_map_frame = in_objects->header.frame_id != "map";
  if (is_object_not_in_map_frame) {
    world2map_transform = transform_listener_.get_transform(
      "map",                        // target
      in_objects->header.frame_id,  // src
      in_objects->header.stamp, rclcpp::Duration::from_seconds(0.1));
    if (!world2map_transform) return;
  }

  // Get objects detected time
  const double objects_detected_time = rclcpp::Time(in_objects->header.stamp).seconds();

  // Remove old objects information in object history
  // road users
  utils::removeOldObjectsHistory(
    objects_detected_time, object_buffer_time_length_, road_users_history_);
  // crosswalk users
  predictor_vru_->removeOldKnownMatches(objects_detected_time, object_buffer_time_length_);

  // result output
  PredictedObjects output;
  output.header = in_objects->header;
  output.header.frame_id = "map";

  // result debug
  visualization_msgs::msg::MarkerArray debug_markers;

  // get current crosswalk users for later prediction
  predictor_vru_->loadCurrentCrosswalkUsers(*in_objects);

  // for each object
  for (const auto & object : in_objects->objects) {
    TrackedObject transformed_object = object;

    // transform object frame if it's based on map frame
    if (is_object_not_in_map_frame) {
      geometry_msgs::msg::PoseStamped pose_in_map;
      geometry_msgs::msg::PoseStamped pose_orig;
      pose_orig.pose = object.kinematics.pose_with_covariance.pose;
      tf2::doTransform(pose_orig, pose_in_map, *world2map_transform);
      transformed_object.kinematics.pose_with_covariance.pose = pose_in_map.pose;
    }

    // get the maximum probability label from the classification array
    const auto & label_ =
      autoware::object_recognition_utils::getHighestProbLabel(transformed_object.classification);
    // overwrite the label for VRU in specific cases
    const auto label = utils::changeVRULabelForPrediction(label_, object, lanelet_map_ptr_);

    switch (label) {
      case ObjectClassification::PEDESTRIAN:
      case ObjectClassification::BICYCLE: {
        // Run pedestrian/bicycle prediction
        const auto predicted_vru =
          getPredictionForNonVehicleObject(output.header, transformed_object);
        output.objects.emplace_back(predicted_vru);
        break;
      }
      case ObjectClassification::CAR:
      case ObjectClassification::BUS:
      case ObjectClassification::TRAILER:
      case ObjectClassification::MOTORCYCLE:
      case ObjectClassification::TRUCK: {
        const auto predicted_object_opt = getPredictionForVehicleObject(
          output.header, transformed_object, objects_detected_time, debug_markers);
        if (predicted_object_opt) {
          output.objects.push_back(predicted_object_opt.value());
        }
        break;
      }
      default: {
        auto predicted_unknown_object = utils::convertToPredictedObject(transformed_object);
        PredictedPath predicted_path = path_generator_->generatePathForNonVehicleObject(
          transformed_object, prediction_time_horizon_.unknown);
        predicted_path.confidence = 1.0;

        predicted_unknown_object.kinematics.predicted_paths.push_back(predicted_path);
        output.objects.push_back(predicted_unknown_object);
        break;
      }
    }
  }

  // process lost crosswalk users to tackle unstable detection
  if (remember_lost_crosswalk_users_) {
    PredictedObjects retrieved_objects = predictor_vru_->retrieveUndetectedObjects();
    output.objects.insert(
      output.objects.end(), retrieved_objects.objects.begin(), retrieved_objects.objects.end());
  }

  // Publish Results
  publish(output, debug_markers);

  // Processing time
  const auto processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
  const auto cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);

  // Diagnostics
  updateDiagnostics(output.header.stamp, processing_time_ms);

  // Publish Processing Time
  if (processing_time_publisher_) {
    processing_time_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/cyclic_time_ms", cyclic_time_ms);
    processing_time_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time_ms", processing_time_ms);
  }
}

}  // namespace autoware::map_based_prediction
