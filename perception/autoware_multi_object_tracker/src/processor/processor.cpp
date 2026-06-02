// Copyright 2024 TIER IV, Inc.
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

#include "processor.hpp"

#include "autoware/multi_object_tracker/object_model/object_model.hpp"
#include "autoware/multi_object_tracker/object_model/shapes.hpp"
#include "autoware/multi_object_tracker/tracker/tracker.hpp"
#include "autoware/multi_object_tracker/types.hpp"

#include <tf2/transform_datatypes.hpp>

#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <cstdint>
#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::multi_object_tracker
{
using autoware_utils_debug::ScopedTimeTrack;

TrackerProcessor::TrackerProcessor(
  const TrackerCreationConfig & creation_config,
  const TrackerAssociationConfig & association_config,
  const TrackerOverlapManagerConfig & tracker_overlap_manager_config,
  const std::vector<types::InputChannel> & channels_config, const rclcpp::Logger & logger,
  rclcpp::Clock::SharedPtr clock)
: creation_config_(creation_config),
  channels_config_(channels_config),
  logger_(logger),
  clock_(std::move(clock))
{
  association_manager_ = std::make_unique<AssociationManager>(association_config, channels_config);
  tracker_overlap_manager_ =
    std::make_unique<TrackerOverlapManager>(tracker_overlap_manager_config);
}

std::optional<geometry_msgs::msg::Pose> TrackerProcessor::getEgoPose() const
{
  return ego_pose_ ? std::make_optional(ego_pose_->pose) : std::nullopt;
}

void TrackerProcessor::updateEgoPose(
  const std::optional<geometry_msgs::msg::PoseStamped> & ego_pose_stamped)
{
  ego_pose_ = ego_pose_stamped;
}

void TrackerProcessor::predictTrackers(const rclcpp::Time & time)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  for (auto itr = list_tracker_.begin(); itr != list_tracker_.end(); ++itr) {
    (*itr)->predict(time);
  }
}

types::AssociationResult TrackerProcessor::associate(
  const types::DynamicObjectList & detected_objects) const
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  return association_manager_->associate(detected_objects, list_tracker_, ego_pose_);
}

void TrackerProcessor::update(const types::AssociatedObjects & associated_objects)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  const auto & detected_objects = associated_objects.objects;
  const auto & association_result = associated_objects.association;

  int tracker_idx = 0;
  const auto & time = detected_objects.header.stamp;
  for (auto tracker_itr = list_tracker_.begin(); tracker_itr != list_tracker_.end();
       ++tracker_itr, ++tracker_idx) {
    bool found = false;
    size_t measurement_idx = 0;
    unique_identifier_msgs::msg::UUID tracker_uuid = (*tracker_itr)->getUUID();

    if (association_result.tracker_to_measurement.count(tracker_uuid)) {
      unique_identifier_msgs::msg::UUID measurement_uuid =
        association_result.tracker_to_measurement.at(tracker_uuid);
      const auto idx = detected_objects.getObjectIndexByUuid(measurement_uuid);
      if (idx) {
        measurement_idx = *idx;
        found = true;
      }
    }

    if (found) {
      const auto & associated_object = detected_objects.objects.at(measurement_idx);
      const types::InputChannel channel_info = channels_config_[associated_object.channel_index];
      const bool has_significant_shape_change = association_result.wasShapeChanged(tracker_uuid);
      (*(tracker_itr))
        ->updateWithMeasurement(
          associated_object, time, channel_info, has_significant_shape_change);
    } else {
      (*(tracker_itr))->updateWithoutMeasurement(time);
    }
  }
}

void TrackerProcessor::spawn(const types::AssociatedObjects & associated_objects)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  const auto & detected_objects = associated_objects.objects;
  const auto & association_result = associated_objects.association;

  const auto channel_config = channels_config_[detected_objects.channel_index];
  if (!channel_config.is_spawn_enabled) {
    return;
  }

  const auto & time = detected_objects.header.stamp;
  for (size_t i = 0; i < detected_objects.objects.size(); ++i) {
    const auto & new_object = detected_objects.objects.at(i);
    if (association_result.measurement_to_tracker.count(new_object.uuid)) {
      continue;
    }
    std::shared_ptr<Tracker> tracker = createNewTracker(new_object, time);
    if (!tracker) continue;  // null combo: (shape, label) not accepted

    if (channel_config.trust_existence_probability) {
      tracker->initializeExistenceProbabilities(
        new_object.channel_index, new_object.existence_probability);
    } else {
      tracker->initializeExistenceProbabilities(
        new_object.channel_index, types::default_existence_probability);
    }

    list_tracker_.push_back(tracker);
  }
}

std::shared_ptr<Tracker> TrackerProcessor::createNewTracker(
  const types::DynamicObject & object, const rclcpp::Time & time) const
{
  const classes::Label label = classes::getHighestProbLabel(object.classification);
  const ShapeLabelKey key{types::toShapeType(object.shape.type), label};

  const auto tracker_type_opt = get_map_value_if_exists(creation_config_.shape_tracker_map, key);

  if (tracker_type_opt) {
    switch (tracker_type_opt->get()) {
      case types::TrackerType::MULTIPLE_VEHICLE:
        return std::make_shared<MultipleVehicleTracker>(time, object);
      case types::TrackerType::GENERAL_VEHICLE:
        return std::make_shared<VehicleTracker>(object_model::general_vehicle, time, object);
      case types::TrackerType::PEDESTRIAN_AND_BICYCLE:
        return std::make_shared<PedestrianAndBicycleTracker>(time, object);
      case types::TrackerType::NORMAL_VEHICLE:
        return std::make_shared<VehicleTracker>(object_model::normal_vehicle, time, object);
      case types::TrackerType::PEDESTRIAN:
        return std::make_shared<PedestrianTracker>(time, object);
      case types::TrackerType::BICYCLE:
        return std::make_shared<VehicleTracker>(object_model::bicycle, time, object);
      case types::TrackerType::BIG_VEHICLE:
        return std::make_shared<VehicleTracker>(object_model::big_vehicle, time, object);
      case types::TrackerType::POLYGON:
        return std::make_shared<PolygonTracker>(
          time, object, creation_config_.enable_unknown_object_velocity_estimation,
          creation_config_.enable_unknown_object_motion_output);
      default:
        return std::make_shared<PolygonTracker>(
          time, object, creation_config_.enable_unknown_object_velocity_estimation,
          creation_config_.enable_unknown_object_motion_output);
    }
  }

  if (creation_config_.explicit_null_combos.count(key)) {
    return nullptr;  // create: "null" — explicitly not accepted, silently skip
  }

  // implicitly omitted — not listed in tracker_assignment; error periodically
  RCLCPP_ERROR_THROTTLE(
    logger_, *clock_, 1000,
    "Received detection with unspecified tracker_assignment combination: shape=%s label=%s. "
    "Add an explicit entry (or create: \"null\") to suppress this error.",
    types::toString(key.first).c_str(), classes::toString(key.second).c_str());
  return nullptr;
}

void TrackerProcessor::prune(const rclcpp::Time & time)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  if (time.nanoseconds() - last_prune_time_.nanoseconds() < 2000 /*2ms*/) {
    return;
  }

  removeOldTracker(time);
  tracker_overlap_manager_->merge(list_tracker_, time, adaptive_threshold_cache_, getEgoPose());

  last_prune_time_ = time;
}

void TrackerProcessor::removeOldTracker(const rclcpp::Time & time)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  for (auto itr = list_tracker_.begin(); itr != list_tracker_.end(); ++itr) {
    if ((*itr)->isExpired(time, adaptive_threshold_cache_, getEgoPose())) {
      auto erase_itr = itr;
      --itr;
      list_tracker_.erase(erase_itr);
    }
  }
}

void TrackerProcessor::getTrackedObjects(
  const rclcpp::Time & time, autoware_perception_msgs::msg::TrackedObjects & tracked_objects) const
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  tracked_objects.header.stamp = time;
  types::DynamicObject tracked_object;
  for (const auto & tracker : list_tracker_) {
    if (!tracker->isConfident(adaptive_threshold_cache_, getEgoPose(), std::nullopt)) continue;
    constexpr bool to_publish = true;
    if (tracker->getTrackedObject(time, tracked_object, to_publish)) {
      tracked_object.existence_probability = tracker->getTotalExistenceProbability();
      tracked_object.classification = tracker->getClassification();
      tracked_objects.objects.push_back(types::toTrackedObjectMsg(tracked_object));
    }
  }
}

void TrackerProcessor::getTentativeObjects(
  const rclcpp::Time & time,
  autoware_perception_msgs::msg::TrackedObjects & tentative_objects) const
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  tentative_objects.header.stamp = time;
  types::DynamicObject tracked_object;
  for (const auto & tracker : list_tracker_) {
    if (tracker->isConfident(adaptive_threshold_cache_, getEgoPose(), std::nullopt)) continue;
    constexpr bool to_publish = false;
    if (tracker->getTrackedObject(time, tracked_object, to_publish)) {
      tentative_objects.objects.push_back(types::toTrackedObjectMsg(tracked_object));
    }
  }
}

void TrackerProcessor::getMergedObjects(
  const rclcpp::Time & time, const geometry_msgs::msg::Transform & tf_base_to_world,
  autoware_perception_msgs::msg::DetectedObjects & merged_objects) const
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  merged_objects.header.stamp = time;
  merged_objects.objects.clear();
  merged_objects.objects.reserve(list_tracker_.size());
  types::DynamicObject tracked_object;
  for (const auto & tracker : list_tracker_) {
    constexpr bool to_publish = false;
    if (tracker->getTrackedObject(time, tracked_object, to_publish)) {
      merged_objects.objects.push_back(types::toDetectedObjectMsg(tracked_object));
    }
  }

  // Transform poses from world frame to ego frame using the inverse of tf_base_to_world
  tf2::Transform tf2_base_to_world;
  tf2::fromMsg(tf_base_to_world, tf2_base_to_world);
  geometry_msgs::msg::TransformStamped ts;
  ts.transform = tf2::toMsg(tf2_base_to_world.inverse());

  for (auto & obj : merged_objects.objects) {
    tf2::doTransform(
      obj.kinematics.pose_with_covariance.pose, obj.kinematics.pose_with_covariance.pose, ts);
  }
}

void TrackerProcessor::setTimeKeeper(
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_ptr)
{
  time_keeper_ = std::move(time_keeper_ptr);
  association_manager_->setTimeKeeper(time_keeper_);
}

}  // namespace autoware::multi_object_tracker
