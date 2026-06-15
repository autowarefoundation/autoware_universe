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

#ifndef AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__TRACKERS__PEDESTRIAN_TRACKER_HPP_
#define AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__TRACKERS__PEDESTRIAN_TRACKER_HPP_

#include "autoware/multi_object_tracker/object_model/object_model.hpp"
#include "autoware/multi_object_tracker/tracker/motion_model/ctrv_motion_model.hpp"
#include "autoware/multi_object_tracker/tracker/shape_model/pedestrian_shape_model.hpp"
#include "autoware/multi_object_tracker/tracker/trackers/tracker_base.hpp"
#include "autoware/multi_object_tracker/types.hpp"

namespace autoware::multi_object_tracker
{

class PedestrianTracker : public Tracker
{
private:
  rclcpp::Logger logger_;

  object_model::ObjectModel object_model_ = object_model::pedestrian;

  // cspell: ignore CTRV
  CTRVMotionModel motion_model_;
  using IDX = CTRVMotionModel::IDX;

  PedestrianShapeModel shape_model_;

  bool updateKinematics(const types::DynamicObject & object);

public:
  PedestrianTracker(const rclcpp::Time & time, const types::DynamicObject & object);

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
  rclcpp::Time getStateTime() const override { return motion_model_.getLastUpdateTime(); }

  ShapeModelBase & getShapeModel() override { return shape_model_; }
  const ShapeModelBase & getShapeModel() const override { return shape_model_; }
  void assembleShapeTo(types::DynamicObject & output, bool /*to_publish*/) const override
  {
    shape_model_.exportTo(output);
  }
};

}  // namespace autoware::multi_object_tracker

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__TRACKERS__PEDESTRIAN_TRACKER_HPP_
