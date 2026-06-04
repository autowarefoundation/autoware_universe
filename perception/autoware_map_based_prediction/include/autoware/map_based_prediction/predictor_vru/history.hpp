// Copyright 2024 TIER IV, inc.
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

#ifndef AUTOWARE__MAP_BASED_PREDICTION__PREDICTOR_VRU__HISTORY_HPP_
#define AUTOWARE__MAP_BASED_PREDICTION__PREDICTOR_VRU__HISTORY_HPP_

#include "autoware/map_based_prediction/data_structure.hpp"

#include <Eigen/Core>
#include <autoware_utils/system/time_keeper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/tracked_objects.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <deque>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace autoware::map_based_prediction
{

class CrosswalkUserHistoryManager
{
public:
  struct Params
  {
    bool match_lost_and_appeared{false};
    double crossing_intention_duration{0.3};
    double no_crossing_intention_duration{1.0};
  };

  explicit CrosswalkUserHistoryManager(rclcpp::Node & node) : node_(node) {}

  void setTimeKeeper(std::shared_ptr<autoware_utils::TimeKeeper> time_keeper)
  {
    time_keeper_ = std::move(time_keeper);
  }

  void setParams(const Params & params)
  {
    match_lost_and_appeared_ = params.match_lost_and_appeared;
    crossing_intention_duration_ = params.crossing_intention_duration;
    no_crossing_intention_duration_ = params.no_crossing_intention_duration;
  }

  void initialize();

  void loadCurrentUsers(
    const TrackedObjects & objects, std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr);

  void removeOldHistory(double current_time, double buffer_time);

  void updateHistory(
    const std_msgs::msg::Header & header, const TrackedObject & object,
    const std::string & object_id);

  std::string tryMatchToDisappeared(const std::string & object_id);

  bool hasPotentialToReachWithHistory(
    const TrackedObject & object, const Eigen::Vector2d & center_point,
    const Eigen::Vector2d & right_point, const Eigen::Vector2d & left_point, double time_horizon,
    double min_object_vel, double max_crosswalk_user_delta_yaw_threshold_for_lanelet,
    bool is_crossing);

  const std::unordered_map<std::string, TrackedObject> & currentUsers() const
  {
    return current_crosswalk_users_;
  }
  std::unordered_map<std::string, TrackedObject> & currentUsers()
  {
    return current_crosswalk_users_;
  }

  const std::unordered_set<std::string> & predictedIds() const
  {
    return predicted_crosswalk_users_ids_;
  }
  std::unordered_set<std::string> & predictedIds() { return predicted_crosswalk_users_ids_; }

  const std::unordered_map<std::string, std::deque<CrosswalkUser>> & history() const
  {
    return crosswalk_users_history_;
  }

private:
  rclcpp::Node & node_;
  std::shared_ptr<autoware_utils::TimeKeeper> time_keeper_;

  std::unordered_map<std::string, TrackedObject> current_crosswalk_users_;
  std::unordered_set<std::string> predicted_crosswalk_users_ids_;
  std::unordered_map<std::string, std::deque<CrosswalkUser>> crosswalk_users_history_;
  std::unordered_map<std::string, std::string> known_matches_;

  bool match_lost_and_appeared_{false};
  double crossing_intention_duration_{0.0};
  double no_crossing_intention_duration_{0.0};
};

}  // namespace autoware::map_based_prediction

#endif  // AUTOWARE__MAP_BASED_PREDICTION__PREDICTOR_VRU__HISTORY_HPP_
