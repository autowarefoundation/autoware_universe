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

#include "autoware/multi_object_tracker/tracker/shape_model/vehicle_extend_manager.hpp"

#include "autoware/multi_object_tracker/object_model/shapes.hpp"

#include <algorithm>

namespace autoware::multi_object_tracker
{

using Shape = autoware_perception_msgs::msg::Shape;

VehicleExtendManager::VehicleExtendManager(const object_model::ObjectModel & object_model)
: last_footprint_update_time_(rclcpp::Time(0, 0, RCL_ROS_TIME)), object_model_(object_model)
{
}

void VehicleExtendManager::init(const types::DynamicObject & object)
{
  if (object.shape.type == Shape::BOUNDING_BOX) {
    width_ = object.shape.dimensions.y;
    height_ = object.shape.dimensions.z;
  } else {
    width_ = object_model_.init_size.width;
    height_ = object_model_.init_size.height;
  }
  width_ =
    std::clamp(width_, object_model_.size_limit.width_min, object_model_.size_limit.width_max);
  height_ =
    std::clamp(height_, object_model_.size_limit.height_min, object_model_.size_limit.height_max);

  footprint_.points.clear();
  footprint_valid_ = false;
}

void VehicleExtendManager::updateShape(const types::DynamicObject & object, bool can_update)
{
  if (!can_update) return;

  constexpr double size_max_multiplier = 1.5;
  constexpr double size_min_multiplier = 0.25;
  if (
    object.shape.dimensions.x > object_model_.size_limit.length_max * size_max_multiplier ||
    object.shape.dimensions.x < object_model_.size_limit.length_min * size_min_multiplier ||
    object.shape.dimensions.y > object_model_.size_limit.width_max * size_max_multiplier ||
    object.shape.dimensions.y < object_model_.size_limit.width_min * size_min_multiplier) {
    return;
  }

  constexpr double gain = 0.4;
  constexpr double gain_inv = 1.0 - gain;
  width_ = gain_inv * width_ + gain * object.shape.dimensions.y;
  height_ = gain_inv * height_ + gain * object.shape.dimensions.z;

  width_ =
    std::clamp(width_, object_model_.size_limit.width_min, object_model_.size_limit.width_max);
  height_ =
    std::clamp(height_, object_model_.size_limit.height_min, object_model_.size_limit.height_max);
}

void VehicleExtendManager::updateFootprint(
  const types::DynamicObject & object, const rclcpp::Time & time,
  const std::optional<geometry_msgs::msg::Pose> & tracker_pose)
{
  const bool has_poly =
    !object.shape.footprint.points.empty() &&
    (object.shape.type == Shape::BOUNDING_BOX || object.shape.type == Shape::POLYGON);
  if (!has_poly) return;

  if (!tracker_pose) {
    // Constructor path: detection pose == tracker initial pose, no transform needed
    footprint_ = object.shape.footprint;
  } else {
    footprint_ = shapes::transformFootprint(object.shape.footprint, object.pose, *tracker_pose);
  }
  footprint_valid_ = true;
  last_footprint_update_time_ = time;
}

void VehicleExtendManager::updateHeight(double z_measurement)
{
  if (z_measurement <= 0.0) return;
  constexpr double gain = 0.4;
  height_ = (1.0 - gain) * height_ + gain * z_measurement;
  height_ =
    std::clamp(height_, object_model_.size_limit.height_min, object_model_.size_limit.height_max);
}

std::optional<double> VehicleExtendManager::setShape(
  const autoware_perception_msgs::msg::Shape & shape, const rclcpp::Time & latest_measurement_time)
{
  width_ = std::clamp(
    shape.dimensions.y, object_model_.size_limit.width_min, object_model_.size_limit.width_max);
  height_ = std::clamp(
    shape.dimensions.z, object_model_.size_limit.height_min, object_model_.size_limit.height_max);

  if (!shape.footprint.points.empty()) {
    footprint_ = shape.footprint;
    footprint_valid_ = true;
    last_footprint_update_time_ = latest_measurement_time;
  }

  if (shape.type == Shape::BOUNDING_BOX) {
    return shape.dimensions.x;
  }
  return std::nullopt;
}

void VehicleExtendManager::mergeFrom(
  const geometry_msgs::msg::Polygon & footprint, const geometry_msgs::msg::Pose & src_pose,
  const geometry_msgs::msg::Pose & winner_pose, const rclcpp::Time & latest_measurement_time)
{
  if (footprint.points.empty()) return;
  const auto transformed = shapes::transformFootprint(footprint, src_pose, winner_pose);
  footprint_ = shapes::unionFootprints(footprint_, transformed);
  footprint_valid_ = true;
  last_footprint_update_time_ = latest_measurement_time;
}

void VehicleExtendManager::exportTo(types::DynamicObject & output, double vehicle_length) const
{
  output.shape.type = Shape::BOUNDING_BOX;
  output.shape.dimensions.x = vehicle_length;
  output.shape.dimensions.y = width_;
  output.shape.dimensions.z = height_;

  const bool footprint_fresh =
    footprint_valid_ && (output.time - last_footprint_update_time_).seconds() < FOOTPRINT_TIMEOUT_S;
  if (footprint_fresh) {
    output.shape.footprint = footprint_;
  } else {
    output.shape.footprint.points.clear();
  }

  output.area = types::getArea(output.shape);
}

void VehicleExtendManager::flipFootprintXY()
{
  for (auto & pt : footprint_.points) {
    pt.x = -pt.x;
    pt.y = -pt.y;
  }
}

}  // namespace autoware::multi_object_tracker
