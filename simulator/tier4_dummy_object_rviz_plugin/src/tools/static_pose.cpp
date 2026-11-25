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

#include "static_pose.hpp"

#include <autoware/object_recognition_utils/pointcloud_classification.hpp>
#include <autoware/point_types/types.hpp>
#include <magic_enum.hpp>
#include <rviz_common/display_context.hpp>

#include <algorithm>
#include <random>
#include <string>

namespace rviz_plugins
{

namespace
{
using autoware::point_types::PointCloudClassification;

int to_int(PointCloudClassification classification)
{
  return static_cast<int>(classification);
}
}  // namespace

StaticInitialPoseTool::StaticInitialPoseTool()
{
  shortcut_key_ = 's';

  enable_interactive_property_ = new rviz_common::properties::BoolProperty(
    "Interactive", false, "Enable/Disable interactive action by manipulating mouse.",
    getPropertyContainer());
  enable_interactive_property_->hide();
  predicted_property_ = new rviz_common::properties::BoolProperty(
    "Predicted", false, "This property is unused for static pose.", getPropertyContainer());
  predicted_property_->hide();
  property_frame_ = new rviz_common::properties::TfFrameProperty(
    "Target Frame", rviz_common::properties::TfFrameProperty::FIXED_FRAME_STRING,
    "The TF frame where the point cloud is output.", getPropertyContainer(), nullptr, true);
  topic_property_ = new rviz_common::properties::StringProperty(
    "Static Area Topic", "/simulation/dummy_perception_publisher/static_area",
    "The topic on which to publish dummy static area info.", getPropertyContainer(),
    SLOT(updateTopic()), this);
  class_property_ = new rviz_common::properties::EnumProperty(
    "Point Class", "VEGETATION", "PointXYZCPE class_id for generated points.",
    getPropertyContainer());
  for (auto c : magic_enum::enum_values<PointCloudClassification>()) {
    if (autoware::object_recognition_utils::is_object_compatible(c)) {
      continue;
    }
    const auto name = magic_enum::enum_name(c);
    class_property_->addOption(
      QString::fromUtf8(name.data(), static_cast<int>(name.size())), to_int(c));
  }
  std_dev_x_ = new rviz_common::properties::FloatProperty(
    "X std deviation", 0.03, "X standard deviation for initial pose [m]", getPropertyContainer());
  std_dev_y_ = new rviz_common::properties::FloatProperty(
    "Y std deviation", 0.03, "Y standard deviation for initial pose [m]", getPropertyContainer());
  std_dev_z_ = new rviz_common::properties::FloatProperty(
    "Z std deviation", 0.03, "Z standard deviation for initial pose [m]", getPropertyContainer());
  std_dev_theta_ = new rviz_common::properties::FloatProperty(
    "Theta std deviation", 5.0 * M_PI / 180.0, "Theta standard deviation for initial pose [rad]",
    getPropertyContainer());
  position_z_ = new rviz_common::properties::FloatProperty(
    "Z position", 0.0, "Z position for initial pose [m]", getPropertyContainer());
  length_ = new rviz_common::properties::FloatProperty(
    "Length", 0.8, "Length of the static area [m]", getPropertyContainer());
  width_ = new rviz_common::properties::FloatProperty(
    "Width", 0.8, "Width of the static area [m]", getPropertyContainer());
  height_ = new rviz_common::properties::FloatProperty(
    "Height", 2.0, "Height of the static area [m]", getPropertyContainer());

  // Unused for static pose
  velocity_ = new rviz_common::properties::FloatProperty(
    "Velocity", 0.0, "This property is unused for static pose.", getPropertyContainer());
  accel_ = new rviz_common::properties::FloatProperty(
    "Acceleration", 0.0, "This property is unused for static pose.", getPropertyContainer());
  max_velocity_ = new rviz_common::properties::FloatProperty(
    "Max velocity", 0.0, "This property is unused for static pose.", getPropertyContainer());
  min_velocity_ = new rviz_common::properties::FloatProperty(
    "Min velocity", 0.0, "This property is unused for static pose.", getPropertyContainer());
  velocity_->hide();
  accel_->hide();
  max_velocity_->hide();
  min_velocity_->hide();

  std_dev_x_->setMin(0);
  std_dev_y_->setMin(0);
  std_dev_z_->setMin(0);
  std_dev_theta_->setMin(0);
  length_->setMin(0.01);
  width_->setMin(0.01);
  height_->setMin(0.01);
}

void StaticInitialPoseTool::onInitialize()
{
  rviz_plugins::InteractiveObjectTool::onInitialize();
  setName("2D Dummy Static Area");
  updateTopic();
}

SimulatedObject StaticInitialPoseTool::createObjectMsg() const
{
  SimulatedObject object{};
  std::string fixed_frame = context_->getFixedFrame().toStdString();

  object.header.frame_id = fixed_frame;
  object.header.stamp = clock_->now();

  object.classification.label = static_cast<uint8_t>(class_property_->getOptionInt());
  object.classification.probability = 1.0;

  object.shape.type = Shape::BOUNDING_BOX;
  object.shape.dimensions.x = length_->getFloat();
  object.shape.dimensions.y = width_->getFloat();
  object.shape.dimensions.z = height_->getFloat();

  object.initial_state.pose_covariance.pose.position.z = position_z_->getFloat();
  object.initial_state.pose_covariance.covariance[0] =
    std_dev_x_->getFloat() * std_dev_x_->getFloat();
  object.initial_state.pose_covariance.covariance[7] =
    std_dev_y_->getFloat() * std_dev_y_->getFloat();
  object.initial_state.pose_covariance.covariance[14] =
    std_dev_z_->getFloat() * std_dev_z_->getFloat();
  object.initial_state.pose_covariance.covariance[35] =
    std_dev_theta_->getFloat() * std_dev_theta_->getFloat();

  std::mt19937 gen(std::random_device{}());
  std::independent_bits_engine<std::mt19937, 8, uint8_t> bit_eng(gen);
  std::generate(object.id.uuid.begin(), object.id.uuid.end(), bit_eng);

  return object;
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::StaticInitialPoseTool, rviz_common::Tool)
