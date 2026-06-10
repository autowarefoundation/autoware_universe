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

#ifndef AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__TRACKERS__TRACKER_BASE_HPP_
#define AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__TRACKERS__TRACKER_BASE_HPP_

#define EIGEN_MPL2_ONLY
#include "autoware/multi_object_tracker/association/adaptive_threshold_cache.hpp"
#include "autoware/multi_object_tracker/object_model/classes.hpp"
#include "autoware/multi_object_tracker/object_model/object_model.hpp"
#include "autoware/multi_object_tracker/object_model/uuid.hpp"
#include "autoware/multi_object_tracker/tracker/shape_model/shape_model_base.hpp"
#include "autoware/multi_object_tracker/tracker/shape_model/unstable_shape_filter.hpp"
#include "autoware/multi_object_tracker/types.hpp"

#include <Eigen/Core>
#include <autoware_utils_geometry/msg/covariance.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/detected_object.hpp>
#include <autoware_perception_msgs/msg/tracked_object.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <boost/circular_buffer.hpp>

#include <array>
#include <optional>
#include <string>
#include <vector>

namespace autoware::multi_object_tracker
{

enum class UpdatePath { NORMAL, TRY_EXTENSION, CONDITIONED };

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
  std::vector<classes::Classification> classification_;

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
  std::vector<classes::Classification> getClassification() const { return classification_; }
  float getTotalExistenceProbability() const { return total_existence_probability_; }
  void updateTotalExistenceProbability(const float & existence_probability);
  void mergeExistenceProbabilities(
    std::vector<types::ExistenceProbability> existence_probabilities);

  // object update
  bool updateWithMeasurement(
    const types::DynamicObject & object, const rclcpp::Time & measurement_time,
    const types::InputChannel & channel_info, bool has_significant_shape_change = false);
  bool updateWithoutMeasurement(const rclcpp::Time & now);
  void updateClassification(const std::vector<classes::Classification> & classification);

  // Returns the shape model that owns this tracker's extension (single source of truth).
  // Composite trackers return the active inner tracker's model (selected by highest-prob label).
  virtual ShapeModelBase & getShapeModel() = 0;
  virtual const ShapeModelBase & getShapeModel() const = 0;

  // Assemble this tracker's shape (and area) into the output object. Replaces the per-tracker
  // shape_model_.exportTo() calls so the base owns shape composition (used by getBEVArea() and the
  // getTrackedObject() implementations). to_publish enables publish-time shape conversion.
  virtual void assembleShapeTo(types::DynamicObject & output, bool to_publish) const = 0;

  // Apply a shape (e.g. from overlap-merge) by routing it to the shape model.
  // VehicleTracker overrides to also sync the motion-model length; composite trackers override to
  // forward to both inner trackers.
  virtual void setObjectShape(const autoware_perception_msgs::msg::Shape & shape)
  {
    getShapeModel().setShape(shape, getLatestMeasurementTime());
  }
  virtual void mergeFootprintFrom(
    const geometry_msgs::msg::Polygon & footprint, const geometry_msgs::msg::Pose & src_pose,
    const geometry_msgs::msg::Pose & dst_pose)
  {
    getShapeModel().mergeFrom(footprint, src_pose, dst_pose, getLatestMeasurementTime());
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
  virtual types::TrackerType getTrackerType() const { return tracker_type_; }
  int getTrackerPriority() const { return static_cast<int>(getTrackerType()); }

  classes::Label getHighestProbLabel() const
  {
    return classes::getHighestProbLabel(classification_);
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

  unique_identifier_msgs::msg::UUID getUUID() const { return uuid_; }

  std::string getUuidString() const
  {
    const auto uuid_msg = uuid_;
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

protected:
  // Committed (non-shape) tracker state. Shape is owned by the shape model (getShapeModel());
  // kinematics (pose/twist) are owned by the motion model and supplied at getTrackedObject() time.
  // These members hold the snapshot at time_, updated by commitState().
  unique_identifier_msgs::msg::UUID uuid_;
  uint channel_index_{0};
  float existence_probability_{types::default_existence_probability};
  types::ObjectKinematics kinematics_{};
  geometry_msgs::msg::Pose pose_;
  std::array<double, 36> pose_covariance_{};
  geometry_msgs::msg::Twist twist_;
  std::array<double, 36> twist_covariance_{};
  bool trust_extension_{false};
  rclcpp::Time time_;

  types::TrackerType tracker_type_{types::TrackerType::POLYGON};

  // Fill the persistent (non-kinematic, non-shape) fields of `object` from the committed state.
  // getTrackedObject() overlays motion (pose/twist via the motion model) and shape
  // (assembleShapeTo()) on top of this; the low-pass z carried in pose_ is preserved.
  void populatePersistentFields(types::DynamicObject & object) const
  {
    object.uuid = uuid_;
    object.channel_index = channel_index_;
    object.existence_probability = existence_probability_;
    object.existence_probabilities = existence_probabilities_;
    object.classification = classification_;
    object.kinematics = kinematics_;
    object.pose = pose_;
    object.pose_covariance = pose_covariance_;
    object.twist = twist_;
    object.twist_covariance = twist_covariance_;
    object.trust_extension = trust_extension_;
  }

  // Snapshot the tracked state at `time` into the committed members (replaces the old
  // getTrackedObject(time, object_) commit). Preserves the low-pass z already written to pose_.
  void commitState(const rclcpp::Time & time)
  {
    types::DynamicObject committed;
    if (!getTrackedObject(time, committed)) return;
    pose_ = committed.pose;
    pose_covariance_ = committed.pose_covariance;
    twist_ = committed.twist;
    twist_covariance_ = committed.twist_covariance;
    kinematics_ = committed.kinematics;
    time_ = time;
  }

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

  // virtual functions
  virtual bool measure(
    const types::DynamicObject & object, const rclcpp::Time & time,
    const types::InputChannel & channel_info) = 0;

  virtual bool conditionedUpdate(
    const types::DynamicObject & measurement, const types::DynamicObject & prediction,
    const autoware_perception_msgs::msg::Shape & tracker_shape,
    const rclcpp::Time & measurement_time, const types::InputChannel & channel_info);

  // Selects the update path for a given measurement.
  // NORMAL      — standard Kalman update (with optional shape-filter history accumulation)
  // TRY_EXTENSION — attempt extension update via shape filter; fall back to CONDITIONED if unstable
  // CONDITIONED — edge-aligned / weak conditioned update; shape management is bypassed entirely
  virtual UpdatePath selectUpdatePath(bool trust_extension, bool has_significant_shape_change) const
  {
    (void)trust_extension;
    (void)has_significant_shape_change;
    return UpdatePath::NORMAL;
  }

public:
  virtual bool getTrackedObject(
    const rclcpp::Time & time, types::DynamicObject & object,
    const bool to_publish = false) const = 0;
  virtual bool predict(const rclcpp::Time & time) = 0;

  virtual void setEgoPose(const std::optional<geometry_msgs::msg::Point> & ego_pos)
  {
    getShapeModel().setEgoPose(ego_pos);
  }

  virtual void setOrientationAvailability(
    const types::OrientationAvailability & orientation_availability)
  {
    kinematics_.orientation_availability = orientation_availability;
  }
};

}  // namespace autoware::multi_object_tracker

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__TRACKERS__TRACKER_BASE_HPP_
