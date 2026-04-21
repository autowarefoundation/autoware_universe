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

#include "autoware/multi_object_tracker/association/adaptive_threshold_cache.hpp"
#include "autoware/multi_object_tracker/association/association_manager.hpp"
#include "autoware/multi_object_tracker/association/tracker_overlap_manager.hpp"
#include "autoware/multi_object_tracker/tracker/model/tracker_base.hpp"
#include "autoware/multi_object_tracker/types.hpp"

#include <autoware_utils_debug/time_keeper.hpp>
#include <rclcpp/rclcpp.hpp>

#include "autoware_perception_msgs/msg/detected_objects.hpp"
#include "autoware_perception_msgs/msg/tracked_objects.hpp"

#include <list>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::multi_object_tracker
{
class TrackerProcessor
{
public:
  TrackerProcessor(
    const TrackerCreationConfig & creation_config, const AssociatorConfig & associator_config,
    const TrackerOverlapManagerConfig & tracker_overlap_manager_config,
    const std::vector<types::InputChannel> & channels_config);

  const std::list<std::shared_ptr<Tracker>> & getListTracker() const { return list_tracker_; }

  // Tracker processes
  void predict(const rclcpp::Time & time, const std::optional<geometry_msgs::msg::Pose> & ego_pose);
  types::AssociationResult associate(const types::DynamicObjectList & detected_objects) const;
  void update(const types::AssociatedObjects & associated_objects);
  void spawn(const types::AssociatedObjects & associated_objects);
  void prune(const rclcpp::Time & time);

  // Output processes
  void getTrackedObjects(
    const rclcpp::Time & time,
    autoware_perception_msgs::msg::TrackedObjects & tracked_objects) const;
  void getTentativeObjects(
    const rclcpp::Time & time,
    autoware_perception_msgs::msg::TrackedObjects & tentative_objects) const;
  void getMergedObjects(
    const rclcpp::Time & time, const geometry_msgs::msg::Transform & tf_base_to_world,
    autoware_perception_msgs::msg::DetectedObjects & merged_objects) const;

  void setTimeKeeper(std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_ptr);

private:
  const TrackerCreationConfig creation_config_;
  const std::vector<types::InputChannel> & channels_config_;

  std::unique_ptr<AssociationManager> association_manager_;
  std::unique_ptr<TrackerOverlapManager> tracker_overlap_manager_;

  mutable rclcpp::Time last_prune_time_;

  std::list<std::shared_ptr<Tracker>> list_tracker_;
  void removeOldTracker(const rclcpp::Time & time);
  std::shared_ptr<Tracker> createNewTracker(
    const types::DynamicObject & object, const rclcpp::Time & time) const;

  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_;
  std::optional<geometry_msgs::msg::Pose> ego_pose_;
  AdaptiveThresholdCache adaptive_threshold_cache_;
};

}  // namespace autoware::multi_object_tracker

#endif  // PROCESSOR__PROCESSOR_HPP_
