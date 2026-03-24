// Copyright 2020 TIER IV, Inc.
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

#ifndef AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__MODEL__TRACKER_BASE_HPP_
#define AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__MODEL__TRACKER_BASE_HPP_

#define EIGEN_MPL2_ONLY
#include "autoware/multi_object_tracker/association/adaptive_threshold_cache.hpp"
#include "autoware/multi_object_tracker/object_model/classes.hpp"
#include "autoware/multi_object_tracker/object_model/object_model.hpp"
#include "autoware/multi_object_tracker/object_model/types.hpp"
#include "autoware/multi_object_tracker/object_model/uuid.hpp"
#include "autoware/multi_object_tracker/tracker/shape_model/unstable_shape_filter.hpp"

#include <Eigen/Core>
#include <autoware_utils_geometry/msg/covariance.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/detected_object.hpp>
#include <autoware_perception_msgs/msg/tracked_object.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <boost/circular_buffer.hpp>

#include <optional>
#include <string>
#include <array>
#include <vector>

namespace autoware::multi_object_tracker
{

enum class TrackerType {
  PASS_THROUGH = 0,
  PEDESTRIAN_AND_BICYCLE = 10,
  PEDESTRIAN = 11,
  BICYCLE = 12,
  MULTIPLE_VEHICLE = 20,
  GENERAL_VEHICLE = 21,
  NORMAL_VEHICLE = 22,
  BIG_VEHICLE = 23,
  VEHICLE = 24,
  POLYGON = 30,
};

inline constexpr std::array<TrackerType, 10> ALL_TRACKER_TYPES = {
  TrackerType::PASS_THROUGH,
  TrackerType::PEDESTRIAN_AND_BICYCLE,
  TrackerType::PEDESTRIAN,
  TrackerType::BICYCLE,
  TrackerType::MULTIPLE_VEHICLE,
  TrackerType::GENERAL_VEHICLE,
  TrackerType::NORMAL_VEHICLE,
  TrackerType::BIG_VEHICLE,
  TrackerType::VEHICLE,
  TrackerType::POLYGON};

inline const std::array<TrackerType, 10> & allTrackerTypes() { return ALL_TRACKER_TYPES; }

inline std::string toString(const TrackerType tracker_type)
{
  switch (tracker_type) {
    case TrackerType::PASS_THROUGH:
      return "pass_through_tracker";
    case TrackerType::PEDESTRIAN_AND_BICYCLE:
      return "pedestrian_and_bicycle_tracker";
    case TrackerType::PEDESTRIAN:
      return "pedestrian_tracker";
    case TrackerType::BICYCLE:
      return "bicycle_tracker";
    case TrackerType::MULTIPLE_VEHICLE:
      return "multi_vehicle_tracker";
    case TrackerType::GENERAL_VEHICLE:
      return "general_vehicle_tracker";
    case TrackerType::NORMAL_VEHICLE:
      return "normal_vehicle_tracker";
    case TrackerType::BIG_VEHICLE:
      return "big_vehicle_tracker";
    case TrackerType::VEHICLE:
      return "vehicle_tracker";
    case TrackerType::POLYGON:
      return "polygon_tracker";
    default:
      return "polygon_tracker";
  }
}

inline std::optional<TrackerType> toTrackerType(const std::string & tracker_name)
{
  if (tracker_name == "pass_through_tracker") return TrackerType::PASS_THROUGH;
  if (tracker_name == "pedestrian_and_bicycle_tracker") {
    return TrackerType::PEDESTRIAN_AND_BICYCLE;
  }
  if (tracker_name == "pedestrian_tracker") return TrackerType::PEDESTRIAN;
  if (tracker_name == "bicycle_tracker") return TrackerType::BICYCLE;
  if (tracker_name == "multi_vehicle_tracker") return TrackerType::MULTIPLE_VEHICLE;
  if (tracker_name == "general_vehicle_tracker") return TrackerType::GENERAL_VEHICLE;
  if (tracker_name == "normal_vehicle_tracker") return TrackerType::NORMAL_VEHICLE;
  if (tracker_name == "big_vehicle_tracker") return TrackerType::BIG_VEHICLE;
  if (tracker_name == "vehicle_tracker") return TrackerType::VEHICLE;
  if (tracker_name == "polygon_tracker") return TrackerType::POLYGON;
  return std::nullopt;
}

class Tracker
{
private:
  // existence states
  int no_measurement_count_;
  int total_no_measurement_count_;
  int total_measurement_count_;
  rclcpp::Time last_update_with_measurement_time_;
  std::vector<types::ExistenceProbability> existence_probabilities_;
  float total_existence_probability_;
  std::vector<object_model::Classification> classification_;

  // conditioned update configs
  // EMA/ema below are abbreviation for exponential moving average
  static constexpr double EMA_ALPHA_WEAK = 0.05;
  static constexpr double EMA_ALPHA_STRONG = 0.2;
  static constexpr double SHAPE_VARIATION_THRESHOLD = 0.1;
  static constexpr size_t STABLE_STREAK_THRESHOLD = 4;

  UnstableShapeFilter unstable_shape_filter_{
    EMA_ALPHA_WEAK, EMA_ALPHA_STRONG, SHAPE_VARIATION_THRESHOLD, STABLE_STREAK_THRESHOLD};

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
  std::vector<types::ExistenceProbability> getExistenceProbabilityVector() const
  {
    return existence_probabilities_;
  }
  std::vector<object_model::Classification> getClassification() const { return classification_; }
  float getTotalExistenceProbability() const { return total_existence_probability_; }
  void updateTotalExistenceProbability(const float & existence_probability);
  void mergeExistenceProbabilities(
    std::vector<types::ExistenceProbability> existence_probabilities);

  // object update
  bool updateWithMeasurement(
    const types::DynamicObject & object, const rclcpp::Time & measurement_time,
    const types::InputChannel & channel_info, bool has_significant_shape_change = false);
  bool updateWithoutMeasurement(const rclcpp::Time & now);
  void updateClassification(const std::vector<object_model::Classification> & classification);
  virtual void setObjectShape(const autoware_perception_msgs::msg::Shape & shape)
  {
    object_.shape = shape;
    object_.area = types::getArea(shape);
  }

  // object life management
  uint getChannelIndex() const;
  void getPositionCovarianceEigenSq(
    const rclcpp::Time & time, double & major_axis_sq, double & minor_axis_sq) const;
  bool isConfident(
    const AdaptiveThresholdCache & cache, const std::optional<geometry_msgs::msg::Pose> & ego_pose,
    const std::optional<rclcpp::Time> & time) const;
  bool isExpired(
    const rclcpp::Time & time, const AdaptiveThresholdCache & cache,
    const std::optional<geometry_msgs::msg::Pose> & ego_pose) const;
  float getKnownObjectProbability() const;
  double getPositionCovarianceDeterminant() const;
  virtual TrackerType getTrackerType() const { return tracker_type_; }
  int getTrackerPriority() const { return static_cast<int>(getTrackerType()); }

  object_model::Label getHighestProbLabel() const
  {
    return object_model::getHighestProbLabel(object_.classification);
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

  unique_identifier_msgs::msg::UUID getUUID() const { return object_.uuid; }

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

  double getBEVArea() const;
  double getDistanceSqToEgo(const std::optional<geometry_msgs::msg::Pose> & ego_pose) const;
  double computeAdaptiveThreshold(
    double base_threshold, double fallback_threshold, const AdaptiveThresholdCache & cache,
    const std::optional<geometry_msgs::msg::Pose> & ego_pose) const;
  bool createPseudoMeasurement(
    const types::DynamicObject & meas, types::DynamicObject & pred,
    const autoware_perception_msgs::msg::Shape & tracker_shape,
    const bool enlarge_covariance = false);

protected:
  types::DynamicObject object_;
  TrackerType tracker_type_{TrackerType::POLYGON};

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

  virtual bool conditionedUpdate(
    const types::DynamicObject & measurement, const types::DynamicObject & prediction,
    const autoware_perception_msgs::msg::Shape & tracker_shape,
    const rclcpp::Time & measurement_time, const types::InputChannel & channel_info);

public:
  virtual bool getTrackedObject(
    const rclcpp::Time & time, types::DynamicObject & object,
    const bool to_publish = false) const = 0;
  virtual bool predict(const rclcpp::Time & time) = 0;

  virtual void setOrientationAvailability(
    const types::OrientationAvailability & orientation_availability)
  {
    object_.kinematics.orientation_availability = orientation_availability;
  }
};

}  // namespace autoware::multi_object_tracker

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__MODEL__TRACKER_BASE_HPP_
