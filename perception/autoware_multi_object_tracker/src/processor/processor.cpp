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

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <algorithm>
#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
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

void TrackerProcessor::predict(
  const rclcpp::Time & time, const std::optional<geometry_msgs::msg::Pose> & ego_pose)
{
  ego_pose_ = ego_pose;

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
  return std::make_shared<UnknownTracker>(
    time, object, config_.enable_unknown_object_velocity_estimation,
    config_.enable_unknown_object_motion_output);
}

void TrackerProcessor::prune(const rclcpp::Time & time)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  if (time.nanoseconds() - last_prune_time_.nanoseconds() < 2000 /*2ms*/) {
    // prune is called too frequently, skip
    return;
  }

  // Check tracker lifetime: if the tracker is old, delete it
  removeOldTracker(time);
  // Check tracker overlap: if the tracker is overlapped, delete the one with lower IOU
  mergeOverlappedTracker(time);

  // update last prune time
  last_prune_time_ = time;
}

void TrackerProcessor::removeOldTracker(const rclcpp::Time & time)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  // Check elapsed time from last update
  for (auto itr = list_tracker_.begin(); itr != list_tracker_.end(); ++itr) {
    // If the tracker is expired, delete it
    if ((*itr)->isExpired(time, adaptive_threshold_cache_, ego_pose_)) {
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

  // Pre-filter valid trackers and cache their data
  struct TrackerData
  {
    std::shared_ptr<Tracker> tracker;
    types::DynamicObject object;
    uint8_t label;
    double measurement_count;
    double elapsed_time;
    bool is_unknown;
    bool is_valid;

    explicit TrackerData(const std::shared_ptr<Tracker> & t)
    : tracker(t),
      object(),
      label(0),
      measurement_count(0.0),
      elapsed_time(0.0),
      is_unknown(false),
      is_valid(false)
    {
    }
  };

  std::vector<TrackerData> valid_trackers;
  valid_trackers.reserve(list_tracker_.size());

  // First pass: collect valid trackers and their data
  for (const auto & tracker : list_tracker_) {
    TrackerData data(tracker);

    // Get tracked object and basic data
    if (!tracker->getTrackedObject(time, data.object)) {
      continue;
    }

    data.label = tracker->getHighestProbLabel();
    data.is_unknown = (data.label == Label::UNKNOWN);
    data.measurement_count = tracker->getTotalMeasurementCount();
    data.elapsed_time = tracker->getElapsedTimeFromLastUpdate(time);
    data.is_valid = true;

    valid_trackers.push_back(std::move(data));
  }

  // Sort valid trackers by priority
  std::sort(
    valid_trackers.begin(), valid_trackers.end(), [](const TrackerData & a, const TrackerData & b) {
      if (a.is_unknown != b.is_unknown) {
        return b.is_unknown;  // Non-unknown first
      }
      if (a.measurement_count != b.measurement_count) {
        return a.measurement_count > b.measurement_count;
      }
      return a.elapsed_time < b.elapsed_time;
    });

  // search distance per label
  size_t label_size = config_.max_dist_matrix.cols();
  std::vector<double> search_distance_sq_per_label(label_size, 0.0);
  for (size_t i = 0; i < label_size; ++i) {
    for (size_t j = 0; j < label_size; ++j) {
      search_distance_sq_per_label[i] =
        std::max(search_distance_sq_per_label[i], config_.max_dist_matrix(j, i));
    }
  }

  // Build spatial index for quick neighbor lookup
  using Point = boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian>;
  using Value = std::pair<Point, size_t>;  // Point and index into valid_trackers
  boost::geometry::index::rtree<Value, boost::geometry::index::quadratic<16>> rtree;

  // Insert valid trackers into R-tree
  std::vector<ValueType> rtree_points;
  rtree_points.reserve(valid_trackers.size());
  for (size_t i = 0; i < valid_trackers.size(); ++i) {
    const auto & data = valid_trackers[i];
    if (!data.is_valid) continue;

    Point p(data.object.pose.position.x, data.object.pose.position.y);
    rtree_points.push_back(std::make_pair(p, i));
  }
  rtree.insert(rtree_points.begin(), rtree_points.end());

  // Vector to store indices of trackers to remove
  std::vector<size_t> to_remove;
  to_remove.reserve(valid_trackers.size() / 4);  // Reasonable initial capacity

  // Second pass: merge overlapping trackers
  for (size_t i = 0; i < valid_trackers.size(); ++i) {
    auto & data1 = valid_trackers[i];
    if (!data1.is_valid || !data1.tracker->isConfident(time, adaptive_threshold_cache_, ego_pose_))
      continue;

    // Find nearby trackers using R-tree
    std::vector<Value> nearby;
    nearby.reserve(16);  // Reasonable initial capacity

    Point p1(data1.object.pose.position.x, data1.object.pose.position.y);
    double max_search_dist_sq = search_distance_sq_per_label[data1.label];

    // Query R-tree with circle
    rtree.query(
      boost::geometry::index::satisfies([&](const Value & v) {
        if (v.second <= i) return false;  // Skip already processed and self

        const double dx = boost::geometry::get<0>(v.first) - data1.object.pose.position.x;
        const double dy = boost::geometry::get<1>(v.first) - data1.object.pose.position.y;
        return dx * dx + dy * dy <= max_search_dist_sq;
      }),
      std::back_inserter(nearby));

    // Process nearby trackers
    for (const auto & [p2, idx2] : nearby) {
      auto & data2 = valid_trackers[idx2];
      if (!data2.is_valid) continue;

      // Calculate IoU only if necessary
      constexpr double min_union_iou_area = 1e-2;
      const auto iou = shapes::get2dIoU(data2.object, data1.object, min_union_iou_area);

      // Skip if IoU is too small
      if (iou < 1e-6) continue;

      if (canMergeOverlappedTarget(*data2.tracker, *data1.tracker, time, iou)) {
        // Merge tracker2 into tracker1
        data1.tracker->updateTotalExistenceProbability(
          data2.tracker->getTotalExistenceProbability());
        data1.tracker->mergeExistenceProbabilities(data2.tracker->getExistenceProbabilityVector());

        // Mark tracker2 for removal
        data2.is_valid = false;
        to_remove.push_back(idx2);
      }
    }
  }

  // Final pass: remove merged trackers efficiently using batch removal
  std::unordered_set<std::shared_ptr<Tracker>> trackers_to_remove;
  trackers_to_remove.reserve(to_remove.size());

  // Collect all trackers to remove in a set for O(1) lookup
  for (const auto idx : to_remove) {
    trackers_to_remove.insert(valid_trackers[idx].tracker);
  }

  // Remove all marked trackers in a single pass
  list_tracker_.remove_if([&trackers_to_remove](const std::shared_ptr<Tracker> & tracker) {
    return trackers_to_remove.count(tracker) > 0;
  });
}

bool TrackerProcessor::canMergeOverlappedTarget(
  const Tracker & target, const Tracker & other, const rclcpp::Time & time, const double iou) const
{
  // if the other is not confident, do not remove the target
  if (!other.isConfident(time, adaptive_threshold_cache_, ego_pose_)) {
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
    if (!tracker->isConfident(time, adaptive_threshold_cache_, ego_pose_)) continue;
    // Get the tracked object, extrapolated to the given time
    constexpr bool to_publish = true;
    if (tracker->getTrackedObject(time, tracked_object, to_publish)) {
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
    if (tracker->isConfident(time, adaptive_threshold_cache_, ego_pose_)) continue;
    // Get the tracked object, extrapolated to the given time
    constexpr bool to_publish = false;
    if (tracker->getTrackedObject(time, tracked_object, to_publish)) {
      tentative_objects.objects.push_back(types::toTrackedObjectMsg(tracked_object));
    }
  }
}

void TrackerProcessor::setTimeKeeper(std::shared_ptr<autoware_utils::TimeKeeper> time_keeper_ptr)
{
  time_keeper_ = std::move(time_keeper_ptr);
  association_->setTimeKeeper(time_keeper_);
}

}  // namespace autoware::multi_object_tracker
