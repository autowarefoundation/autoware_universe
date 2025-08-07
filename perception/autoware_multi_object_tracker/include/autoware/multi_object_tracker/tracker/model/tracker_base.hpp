// Copyright 2020 Tier IV, Inc.
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
//
//
// Author: v1.0 Yukihiro Saito
//

#ifndef AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__MODEL__TRACKER_BASE_HPP_
#define AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__MODEL__TRACKER_BASE_HPP_

#define EIGEN_MPL2_ONLY
#include "autoware/multi_object_tracker/object_model/object_model.hpp"
#include "autoware/multi_object_tracker/object_model/types.hpp"
#include "autoware/multi_object_tracker/tracker/util/adaptive_threshold_cache.hpp"

#include <Eigen/Core>
#include <autoware/object_recognition_utils/object_recognition_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/detected_object.hpp>
#include <autoware_perception_msgs/msg/tracked_object.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <optional>
#include <string>
#include <vector>

namespace autoware::multi_object_tracker
{

enum class TrackerType {
  PASS_THROUGH = 0,
  PEDESTRIAN_AND_BICYCLE = 10,
  PEDESTRIAN = 11,
  BICYCLE = 12,
  MULTIPLE_VEHICLE = 20,
  NORMAL_VEHICLE = 21,
  BIG_VEHICLE = 22,
  VEHICLE = 23,
  UNKNOWN = 30,
};

class Tracker
{
private:
  // existence states
  int no_measurement_count_;
  int total_no_measurement_count_;
  int total_measurement_count_;
  rclcpp::Time last_update_with_measurement_time_;
  std::vector<float> existence_probabilities_;
  float total_existence_probability_;
  std::vector<autoware_perception_msgs::msg::ObjectClassification> classification_;

  // cache
  mutable rclcpp::Time cached_time_;
  mutable types::DynamicObject cached_object_;
  mutable int cached_measurement_count_;

public:
  Tracker(const rclcpp::Time & time, const types::DynamicObject & object);
  virtual ~Tracker() = default;

  // tracker probabilities
  void initializeExistenceProbabilities(
    const uint & channel_index, const float & existence_probability);
  std::vector<float> getExistenceProbabilityVector() const { return existence_probabilities_; }
  std::vector<autoware_perception_msgs::msg::ObjectClassification> getClassification() const
  {
    return classification_;
  }
  float getTotalExistenceProbability() const { return total_existence_probability_; }
  void updateTotalExistenceProbability(const float & existence_probability);
  void mergeExistenceProbabilities(std::vector<float> existence_probabilities);

  // object update
  bool updateWithMeasurement(
    const types::DynamicObject & object, const rclcpp::Time & measurement_time,
    const types::InputChannel & channel_info);
  bool updateWithoutMeasurement(const rclcpp::Time & now);
  void updateClassification(
    const std::vector<autoware_perception_msgs::msg::ObjectClassification> & classification);
  void setObjectShape(const autoware_perception_msgs::msg::Shape & shape)
  {
    object_.shape = shape;
    object_.area = types::getArea(shape);
  }

  // object life management
  uint getChannelIndex() const;
  void getPositionCovarianceEigenSq(
    const rclcpp::Time & time, double & major_axis_sq, double & minor_axis_sq) const;
  bool isConfident(
    const rclcpp::Time & time, const AdaptiveThresholdCache & cache,
    const std::optional<geometry_msgs::msg::Pose> & ego_pose) const;
  bool isExpired(
    const rclcpp::Time & time, const AdaptiveThresholdCache & cache,
    const std::optional<geometry_msgs::msg::Pose> & ego_pose) const;
  float getKnownObjectProbability() const;
  double getPositionCovarianceDeterminant() const;
  virtual TrackerType getTrackerType() const { return tracker_type_; }
  int getTrackerPriority() const { return static_cast<int>(getTrackerType()); }

  std::uint8_t getHighestProbLabel() const
  {
    return autoware::object_recognition_utils::getHighestProbLabel(object_.classification);
  }

  // existence states
  int getNoMeasurementCount() const { return no_measurement_count_; }
  int getTotalNoMeasurementCount() const { return total_no_measurement_count_; }
  int getTotalMeasurementCount() const { return total_measurement_count_; }
  double getElapsedTimeFromLastUpdate(const rclcpp::Time & current_time) const
  {
    return (current_time - last_update_with_measurement_time_).seconds();
  }
  rclcpp::Time getLatestMeasurementTime() const { return last_update_with_measurement_time_; }

  std::string getUuidString() const
  {
    const auto uuid_msg = object_.uuid;
    std::stringstream ss;
    constexpr size_t UUID_SIZE = 16;
    ss << std::hex << std::setfill('0');
    for (size_t i = 0; i < UUID_SIZE; ++i) {
      ss << std::setw(2) << static_cast<int>(uuid_msg.uuid[i]);
    }
    return ss.str();
  }

protected:
  types::DynamicObject object_;
  TrackerType tracker_type_{TrackerType::UNKNOWN};

  void updateCache(const types::DynamicObject & object, const rclcpp::Time & time) const
  {
    cached_time_ = time;
    cached_object_ = object;
    cached_measurement_count_ = total_measurement_count_ + total_no_measurement_count_;
  }

  bool getCachedObject(const rclcpp::Time & time, types::DynamicObject & object) const
  {
    if (
      cached_time_.nanoseconds() == time.nanoseconds() &&
      cached_measurement_count_ == total_measurement_count_ + total_no_measurement_count_) {
      object = cached_object_;
      return true;
    }
    return false;
  }

  void removeCache() const
  {
    cached_time_ = rclcpp::Time();
    cached_object_ = types::DynamicObject();
    cached_measurement_count_ = -1;
  }

  void limitObjectExtension(const object_model::ObjectModel object_model);

  // virtual functions
  virtual bool measure(
    const types::DynamicObject & object, const rclcpp::Time & time,
    const types::InputChannel & channel_info) = 0;

public:
  virtual bool getTrackedObject(
    const rclcpp::Time & time, types::DynamicObject & object,
    const bool to_publish = false) const = 0;
  virtual bool predict(const rclcpp::Time & time) = 0;
  double getBEVArea() const;
  double getDistanceSqToEgo(const std::optional<geometry_msgs::msg::Pose> & ego_pose) const;
  double computeAdaptiveThreshold(
    double base_threshold, double fallback_threshold, const AdaptiveThresholdCache & cache,
    const std::optional<geometry_msgs::msg::Pose> & ego_pose) const;
};

}  // namespace autoware::multi_object_tracker

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__MODEL__TRACKER_BASE_HPP_
