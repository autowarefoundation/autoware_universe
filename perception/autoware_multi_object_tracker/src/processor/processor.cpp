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
#include "autoware/multi_object_tracker/object_model/types.hpp"
#include "autoware/multi_object_tracker/tracker/tracker.hpp"

#include <autoware/object_recognition_utils/object_recognition_utils.hpp>

#include <autoware_perception_msgs/msg/tracked_objects.hpp>

#include <iterator>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::multi_object_tracker
{
using autoware_utils::ScopedTimeTrack;
using Label = autoware_perception_msgs::msg::ObjectClassification;
using LabelType = autoware_perception_msgs::msg::ObjectClassification::_label_type;

TrackerProcessor::TrackerProcessor(
  const TrackerProcessorConfig & config, const AssociatorConfig & associator_config,
  const std::vector<types::InputChannel> & channels_config)
: config_(config), channels_config_(channels_config)
{
  association_ = std::make_unique<DataAssociation>(associator_config);
}

void TrackerProcessor::predict(const rclcpp::Time & time)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  for (auto itr = list_tracker_.begin(); itr != list_tracker_.end(); ++itr) {
    (*itr)->predict(time);
  }
}

void TrackerProcessor::associate(
  const types::DynamicObjectList & detected_objects,
  std::unordered_map<int, int> & direct_assignment,
  std::unordered_map<int, int> & reverse_assignment) const
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  const auto & tracker_list = list_tracker_;
  // global nearest neighbor
  Eigen::MatrixXd score_matrix = association_->calcScoreMatrix(
    detected_objects, tracker_list);  // row : tracker, col : measurement
  association_->assign(score_matrix, direct_assignment, reverse_assignment);
}

void TrackerProcessor::update(
  const types::DynamicObjectList & detected_objects,
  const std::unordered_map<int, int> & direct_assignment)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  int tracker_idx = 0;
  const auto & time = detected_objects.header.stamp;
  for (auto tracker_itr = list_tracker_.begin(); tracker_itr != list_tracker_.end();
       ++tracker_itr, ++tracker_idx) {
    if (direct_assignment.find(tracker_idx) != direct_assignment.end()) {
      // found
      const auto & associated_object =
        detected_objects.objects.at(direct_assignment.find(tracker_idx)->second);
      const types::InputChannel channel_info = channels_config_[associated_object.channel_index];
      (*(tracker_itr))->updateWithMeasurement(associated_object, time, channel_info);

    } else {
      // not found
      (*(tracker_itr))->updateWithoutMeasurement(time);
    }
  }
}

void TrackerProcessor::spawn(
  const types::DynamicObjectList & detected_objects,
  const std::unordered_map<int, int> & reverse_assignment)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  const auto channel_config = channels_config_[detected_objects.channel_index];
  // If spawn is disabled, return
  if (!channel_config.is_spawn_enabled) {
    return;
  }

  // Spawn new trackers for the objects that are not associated
  const auto & time = detected_objects.header.stamp;
  for (size_t i = 0; i < detected_objects.objects.size(); ++i) {
    if (reverse_assignment.find(i) != reverse_assignment.end()) {  // found
      continue;
    }
    const auto & new_object = detected_objects.objects.at(i);
    std::shared_ptr<Tracker> tracker = createNewTracker(new_object, time);

    // Initialize existence probabilities
    if (channel_config.trust_existence_probability) {
      tracker->initializeExistenceProbabilities(
        new_object.channel_index, new_object.existence_probability);
    } else {
      tracker->initializeExistenceProbabilities(
        new_object.channel_index, types::default_existence_probability);
    }

    // Update the tracker with the new object
    list_tracker_.push_back(tracker);
  }
}

std::shared_ptr<Tracker> TrackerProcessor::createNewTracker(
  const types::DynamicObject & object, const rclcpp::Time & time) const
{
  const LabelType label =
    autoware::object_recognition_utils::getHighestProbLabel(object.classification);
  if (config_.tracker_map.count(label) != 0) {
    const auto tracker = config_.tracker_map.at(label);
    if (tracker == "bicycle_tracker")
      return std::make_shared<VehicleTracker>(object_model::bicycle, time, object);
    if (tracker == "big_vehicle_tracker")
      return std::make_shared<VehicleTracker>(object_model::big_vehicle, time, object);
    if (tracker == "multi_vehicle_tracker")
      return std::make_shared<MultipleVehicleTracker>(time, object);
    if (tracker == "normal_vehicle_tracker")
      return std::make_shared<VehicleTracker>(object_model::normal_vehicle, time, object);
    if (tracker == "pass_through_tracker")
      return std::make_shared<PassThroughTracker>(time, object);
    if (tracker == "pedestrian_and_bicycle_tracker")
      return std::make_shared<PedestrianAndBicycleTracker>(time, object);
    if (tracker == "pedestrian_tracker") return std::make_shared<PedestrianTracker>(time, object);
  }
  return std::make_shared<UnknownTracker>(time, object);
}

void TrackerProcessor::prune(const rclcpp::Time & time)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  // Check tracker lifetime: if the tracker is old, delete it
  removeOldTracker(time);
  // Check tracker overlap: if the tracker is overlapped, delete the one with lower IOU
  mergeOverlappedTracker(time);
}

void TrackerProcessor::removeOldTracker(const rclcpp::Time & time)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  // Check elapsed time from last update
  for (auto itr = list_tracker_.begin(); itr != list_tracker_.end(); ++itr) {
    // If the tracker is expired, delete it
    if ((*itr)->isExpired(time)) {
      auto erase_itr = itr;
      --itr;
      list_tracker_.erase(erase_itr);
    }
  }
}

// This function removes overlapped trackers based on distance and IoU criteria
void TrackerProcessor::mergeOverlappedTracker(const rclcpp::Time & time)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  // Create sorted list with non-UNKNOWN objects first, then by measurement count
  list_tracker_.sort(
    [&time](const std::shared_ptr<Tracker> & a, const std::shared_ptr<Tracker> & b) {
      bool a_unknown = (a->getHighestProbLabel() == Label::UNKNOWN);
      bool b_unknown = (b->getHighestProbLabel() == Label::UNKNOWN);
      if (a_unknown != b_unknown) {
        return b_unknown;  // Put non-UNKNOWN objects first
      }
      if (a->getTotalMeasurementCount() != b->getTotalMeasurementCount()) {
        return a->getTotalMeasurementCount() >
               b->getTotalMeasurementCount();  // Then sort by measurement count
      }
      return a->getElapsedTimeFromLastUpdate(time) <
             b->getElapsedTimeFromLastUpdate(time);  // Finally sort by elapsed time (smaller first)
    });

  // Iterate through the list of trackers
  for (auto itr1 = list_tracker_.begin(); itr1 != list_tracker_.end(); ++itr1) {
    types::DynamicObject object1;
    if (!(*itr1)->getTrackedObject(time, object1)) continue;
    // Compare the current tracker with the remaining trackers
    for (auto itr2 = std::next(itr1); itr2 != list_tracker_.end();) {
      types::DynamicObject object2;
      if (!(*itr2)->getTrackedObject(time, object2)) {
        ++itr2;
        continue;
      }
      // Calculate the distance between the two objects
      const double distance = std::hypot(
        object1.pose.position.x - object2.pose.position.x,
        object1.pose.position.y - object2.pose.position.y);
      const auto & label1 = !(*itr1)->getHighestProbLabel();
      const auto & label2 = !(*itr2)->getHighestProbLabel();
      const double max_dist_matrix_value = config_.max_dist_matrix(
        label2, label1);  // Get the maximum distance threshold for the labels

      // If the distance is too large, skip
      if (distance > max_dist_matrix_value) {
        ++itr2;
        continue;
      }

      // Check the Intersection over Union (IoU) between the two objects
      constexpr double min_union_iou_area = 1e-2;
      const auto iou = shapes::get2dIoU(object2, object1, min_union_iou_area);

      // check if object2 should be removed
      if (canMergeOverlappedTarget(*(*itr2), *(*itr1), time, iou)) {
        // add existence probability to the tracker 1
        (*itr1)->updateTotalExistenceProbability((*itr2)->getTotalExistenceProbability());
        (*itr1)->mergeExistenceProbabilities((*itr2)->getExistenceProbabilityVector());

        // Remove from original list_tracker
        itr2 = list_tracker_.erase(itr2);
      } else {
        ++itr2;  // Move to the next tracker
      }
    }
  }
}

bool TrackerProcessor::canMergeOverlappedTarget(
  const Tracker & target, const Tracker & other, const rclcpp::Time & time, const double iou) const
{
  // if the other is not confident, do not remove the target
  if (!other.isConfident(time)) {
    return false;
  }

  // 1. compare known class probability
  const float target_known_prob = target.getKnownObjectProbability();
  const float other_known_prob = other.getKnownObjectProbability();
  constexpr float min_known_prob = 0.2;

  // the target class is known
  if (target_known_prob >= min_known_prob) {
    // if other class is unknown, do not remove target
    if (other_known_prob < min_known_prob) {
      return false;
    }
    // both are known class, check the IoU
    if (iou > config_.min_known_object_removal_iou) {
      // compare probability vector, prioritize lower index of the probability vector
      std::vector<float> target_existence_prob = target.getExistenceProbabilityVector();
      std::vector<float> other_existence_prob = other.getExistenceProbabilityVector();
      constexpr float prob_buffer = 0.4;
      for (size_t i = 0; i < target_existence_prob.size(); ++i) {
        if (target_existence_prob[i] + prob_buffer < other_existence_prob[i]) {
          // if a channel probability has a large difference in higher index, remove the target
          return true;
        }
      }

      // if there is no big difference in the probability per channel, compare the covariance size
      return target.getPositionCovarianceDeterminant() > other.getPositionCovarianceDeterminant();
    }
  }
  // 2. the target class is unknown, check the IoU
  if (iou > config_.min_unknown_object_removal_iou) {
    if (other_known_prob < min_known_prob) {
      // both are unknown, remove the larger uncertainty one
      return target.getPositionCovarianceDeterminant() > other.getPositionCovarianceDeterminant();
    }
    // if the other class is known, remove the target
    return true;
  }
  return false;
}

void TrackerProcessor::getTrackedObjects(
  const rclcpp::Time & time, autoware_perception_msgs::msg::TrackedObjects & tracked_objects) const
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  tracked_objects.header.stamp = time;
  types::DynamicObject tracked_object;
  for (const auto & tracker : list_tracker_) {
    // check if the tracker is confident, if not, skip
    if (!tracker->isConfident(time)) continue;
    // Get the tracked object, extrapolated to the given time
    if (tracker->getTrackedObject(time, tracked_object)) {
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
    // check if the tracker is confident, if so, skip
    if (tracker->isConfident(time)) continue;
    // Get the tracked object, extrapolated to the given time
    if (tracker->getTrackedObject(time, tracked_object)) {
      tentative_objects.objects.push_back(types::toTrackedObjectMsg(tracked_object));
    }
  }
}

void TrackerProcessor::setTimeKeeper(std::shared_ptr<autoware_utils::TimeKeeper> time_keeper_ptr)
{
  time_keeper_ = std::move(time_keeper_ptr);
}

}  // namespace autoware::multi_object_tracker
