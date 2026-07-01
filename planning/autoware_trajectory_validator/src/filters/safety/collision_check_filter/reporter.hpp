// Copyright 2026 TIER IV, Inc.
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

#ifndef FILTERS__SAFETY__COLLISION_CHECK_FILTER__REPORTER_HPP_
#define FILTERS__SAFETY__COLLISION_CHECK_FILTER__REPORTER_HPP_

#include "types.hpp"

#include <autoware_utils_geometry/geometry.hpp>
#include <rclcpp/time.hpp>

#include <autoware_internal_planning_msgs/msg/planning_factor_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety::reporter
{
using autoware_utils_geometry::Polygon2d;
using PoseTrajectory = std::vector<geometry_msgs::msg::Pose>;

class ContinuousDetectionTimes
{
public:
  void clear();

  template <typename Detections, typename KeyFunc>
  void update(const rclcpp::Time & current_time, const Detections & detections, KeyFunc key_func)
  {
    current_time_ = current_time;

    std::unordered_set<std::string> active_keys{};
    for (const auto & detection : detections) {
      const auto key = key_func(detection);
      active_keys.insert(key);
      detection_start_times_.try_emplace(key, current_time);
    }

    for (auto it = detection_start_times_.begin(); it != detection_start_times_.end();) {
      if (!active_keys.count(it->first)) {
        it = detection_start_times_.erase(it);
      } else {
        ++it;
      }
    }
  }

  double get_time(const std::string & key) const;

private:
  std::optional<rclcpp::Time> current_time_;
  std::unordered_map<std::string, rclcpp::Time> detection_start_times_;
};

void add_debug_markers(
  visualization_msgs::msg::MarkerArray & debug_markers, const rclcpp::Time & stamp,
  const std::string & ns, const std::string & trajectory_id, const PoseTrajectory & ego_trajectory,
  const PoseTrajectory & object_trajectory, const Polygon2d & ego_hull,
  const Polygon2d & object_hull);

void add_error_text_marker(
  visualization_msgs::msg::MarkerArray & debug_markers, const rclcpp::Time & stamp,
  const geometry_msgs::msg::Pose & ego_pose, const std::string & error_msg);

void append_text_marker_message(std::string & text, const std::string & message);

void log_collision_messages(const RiskLevel::_level_type level, const std::string & messages);

autoware_internal_planning_msgs::msg::PlanningFactorArray process_collision_artifacts(
  const nav_msgs::msg::Odometry & odometry, const PetArtifact & pet_artifact,
  ContinuousDetectionTimes & pet_continuous_times, const DracArtifact & drac_artifact,
  ContinuousDetectionTimes & drac_continuous_times, const RssArtifact & rss_artifact,
  ContinuousDetectionTimes & rss_continuous_times,
  visualization_msgs::msg::MarkerArray & debug_markers, double time_resolution);
}  // namespace autoware::trajectory_validator::plugin::safety::reporter

#endif  // FILTERS__SAFETY__COLLISION_CHECK_FILTER__REPORTER_HPP_
