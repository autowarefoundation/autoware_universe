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

#ifndef PROCESSOR__PROCESSOR_HPP_
#define PROCESSOR__PROCESSOR_HPP_

#include "autoware/multi_object_tracker/association/association.hpp"
#include "autoware/multi_object_tracker/object_model/types.hpp"
#include "autoware/multi_object_tracker/tracker/model/tracker_base.hpp"
#include "autoware/multi_object_tracker/tracker/util/adaptive_threshold_cache.hpp"

#include <autoware_utils/system/time_keeper.hpp>
#include <rclcpp/rclcpp.hpp>

#include "autoware_perception_msgs/msg/detected_objects.hpp"
#include "autoware_perception_msgs/msg/tracked_objects.hpp"

#include <list>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::multi_object_tracker
{
using LabelType = autoware_perception_msgs::msg::ObjectClassification::_label_type;

struct TrackerProcessorConfig
{
  std::map<LabelType, std::string> tracker_map;
  float tracker_lifetime;                              // [s]
  float min_known_object_removal_iou;                  // ratio [0, 1]
  float min_unknown_object_removal_iou;                // ratio [0, 1]
  std::map<LabelType, int> confident_count_threshold;  // [count]
  Eigen::MatrixXd max_dist_matrix;
  bool enable_unknown_object_velocity_estimation;
  bool enable_unknown_object_motion_output;
};

class TrackerProcessor
{
public:
  TrackerProcessor(
    const TrackerProcessorConfig & config, const AssociatorConfig & associator_config,
    const std::vector<types::InputChannel> & channels_config);

  const std::list<std::shared_ptr<Tracker>> & getListTracker() const { return list_tracker_; }
  // tracker processes
  void predict(const rclcpp::Time & time, const std::optional<geometry_msgs::msg::Pose> & ego_pose);
  void associate(
    const types::DynamicObjectList & detected_objects,
    std::unordered_map<int, int> & direct_assignment,
    std::unordered_map<int, int> & reverse_assignment) const;
  void update(
    const types::DynamicObjectList & detected_objects,
    const std::unordered_map<int, int> & direct_assignment);
  void spawn(
    const types::DynamicObjectList & detected_objects,
    const std::unordered_map<int, int> & reverse_assignment);
  void prune(const rclcpp::Time & time);

  // output processes
  void getTrackedObjects(
    const rclcpp::Time & time,
    autoware_perception_msgs::msg::TrackedObjects & tracked_objects) const;
  void getTentativeObjects(
    const rclcpp::Time & time,
    autoware_perception_msgs::msg::TrackedObjects & tentative_objects) const;

  void setTimeKeeper(std::shared_ptr<autoware_utils::TimeKeeper> time_keeper_ptr);

private:
  const TrackerProcessorConfig config_;
  const std::vector<types::InputChannel> & channels_config_;

  std::unique_ptr<DataAssociation> association_;

  mutable rclcpp::Time last_prune_time_;

  std::list<std::shared_ptr<Tracker>> list_tracker_;
  void removeOldTracker(const rclcpp::Time & time);
  void mergeOverlappedTracker(const rclcpp::Time & time);
  bool canMergeOverlappedTarget(
    const Tracker & target, const Tracker & other, const rclcpp::Time & time,
    const double iou) const;
  std::shared_ptr<Tracker> createNewTracker(
    const types::DynamicObject & object, const rclcpp::Time & time) const;

  std::shared_ptr<autoware_utils::TimeKeeper> time_keeper_;
  std::optional<geometry_msgs::msg::Pose> ego_pose_;
  AdaptiveThresholdCache adaptive_threshold_cache_;
};

}  // namespace autoware::multi_object_tracker

#endif  // PROCESSOR__PROCESSOR_HPP_
