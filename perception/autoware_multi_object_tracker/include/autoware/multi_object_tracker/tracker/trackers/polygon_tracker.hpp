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

#ifndef AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__TRACKERS__POLYGON_TRACKER_HPP_
#define AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__TRACKERS__POLYGON_TRACKER_HPP_

#include "autoware/multi_object_tracker/configurations.hpp"
#include "autoware/multi_object_tracker/object_model/object_model.hpp"
#include "autoware/multi_object_tracker/tracker/motion_model/cv_motion_model.hpp"
#include "autoware/multi_object_tracker/tracker/motion_model/static_motion_model.hpp"
#include "autoware/multi_object_tracker/tracker/shape_model/polygon_shape_model.hpp"
#include "autoware/multi_object_tracker/tracker/trackers/tracker_base.hpp"
#include "autoware/multi_object_tracker/types.hpp"

namespace autoware::multi_object_tracker
{

class PolygonTracker : public Tracker
{
private:
  rclcpp::Logger logger_;

  object_model::ObjectModel object_model_ = object_model::unknown;

  CVMotionModel motion_model_;
  StaticMotionModel static_motion_model_;

  bool enable_velocity_estimation_;
  // Per-label motion-output gate, evaluated at publish time against the track's current label.
  LabelBoolMap enable_motion_output_;

  geometry_msgs::msg::Pose last_pose_;

  PolygonShapeModel shape_model_;

  bool updateKinematics(const types::DynamicObject & object);

  // Continuously attenuate the published velocity by its direction-projected uncertainty
  // (mirrors VehicleTracker): zero when too uncertain, unchanged when large enough, rescaling
  // the 2D twist so the velocity direction is preserved.
  void suppressUncertainVelocity(types::DynamicObject & object) const;

public:
  PolygonTracker(
    const rclcpp::Time & time, const types::DynamicObject & object,
    const PolygonTrackerConfig & config);

  bool predict(const rclcpp::Time & time) override;
  bool measure(
    const types::DynamicObject & object, const rclcpp::Time & time,
    const types::InputChannel & channel_info) override;
  bool getTrackedObject(
    const rclcpp::Time & time, types::DynamicObject & object,
    const bool to_publish = false) const override;

  bool getMotionState(
    const rclcpp::Time & time, geometry_msgs::msg::Pose & pose, std::array<double, 36> & pose_cov,
    geometry_msgs::msg::Twist & twist, std::array<double, 36> & twist_cov) const override;
  rclcpp::Time getStateTime() const override
  {
    return enable_velocity_estimation_ ? motion_model_.getLastPredictionTime()
                                       : static_motion_model_.getLastPredictionTime();
  }

  ShapeModelBase & getShapeModel() override { return shape_model_; }
  const ShapeModelBase & getShapeModel() const override { return shape_model_; }
  void assembleShapeTo(types::DynamicObject & output, bool /*to_publish*/) const override
  {
    shape_model_.exportTo(output);
  }
};

}  // namespace autoware::multi_object_tracker

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__TRACKERS__POLYGON_TRACKER_HPP_
