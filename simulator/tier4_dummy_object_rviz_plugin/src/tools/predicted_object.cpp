// Copyright 2025 Tier IV, Inc.
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

#include "predicted_object.hpp"

#include <string>
#include <vector>

namespace rviz_plugins
{
PredictedCarPoseTool::PredictedCarPoseTool()
{
  shortcut_key_ = 'c';

  topic_property_ = new rviz_common::properties::StringProperty(
    "Topic", "/planning/dummy_objects",
    "The topic on which to publish dummy object info.", getPropertyContainer(),
    SLOT(updateTopic()), this);
  std_dev_x_ = new rviz_common::properties::FloatProperty(
    "X std deviation", 0.03, "X standard deviation for initial pose [m]",
    getPropertyContainer());
  std_dev_y_ = new rviz_common::properties::FloatProperty(
    "Y std deviation", 0.03, "Y standard deviation for initial pose [m]",
    getPropertyContainer());
  std_dev_z_ = new rviz_common::properties::FloatProperty(
    "Z std deviation", 0.03, "Z standard deviation for initial pose [m]",
    getPropertyContainer());
  std_dev_theta_ = new rviz_common::properties::FloatProperty(
    "Theta std deviation", 5.0 * M_PI / 180.0, "Theta standard deviation for initial pose [rad]",
    getPropertyContainer());
  position_z_ = new rviz_common::properties::FloatProperty(
    "Z position", 0.0, "Z position for initial pose [m]", getPropertyContainer());
  velocity_ = new rviz_common::properties::FloatProperty(
    "Velocity", 0.0, "velocity [m/s]", getPropertyContainer());
  max_velocity_ = new rviz_common::properties::FloatProperty(
    "Max velocity", 33.3, "Max velocity [m/s]", getPropertyContainer());
  min_velocity_ = new rviz_common::properties::FloatProperty(
    "Min velocity", -33.3, "Min velocity [m/s]", getPropertyContainer());
  accel_ = new rviz_common::properties::FloatProperty(
    "Acceleration", 0.0, "Acceleration [m/s^2]", getPropertyContainer());
  width_ = new rviz_common::properties::FloatProperty(
    "Width", 1.8, "Width [m]", getPropertyContainer());
  length_ = new rviz_common::properties::FloatProperty(
    "Length", 4.0, "Length [m]", getPropertyContainer());
  height_ = new rviz_common::properties::FloatProperty(
    "Height", 2.0, "Height [m]", getPropertyContainer());
  std_dev_x_->setMin(0);
  std_dev_y_->setMin(0);
  std_dev_z_->setMin(0);
  std_dev_theta_->setMin(0);
  position_z_->setMin(-100);
  position_z_->setMax(100);

  enable_interactive_property_ = new rviz_common::properties::BoolProperty(
    "Interactive", false, "Enable/Disable interactive action by manipulating mouse.",
    getPropertyContainer());
  predicted_property_ = new rviz_common::properties::BoolProperty(
    "Predicted", true, "Enable/Disable predicted object mode using trajectory predictions.",
    getPropertyContainer());
  predicted_property_->setHidden(true);  // Always true for predicted tools
  property_frame_ = new rviz_common::properties::TfFrameProperty(
    "Target Frame", rviz_common::properties::TfFrameProperty::FIXED_FRAME_STRING,
    "The TF frame where the point cloud is output.", getPropertyContainer(), nullptr, true);
}

void PredictedCarPoseTool::onInitialize()
{
  InteractiveObjectTool::onInitialize();
  setName("2D Predicted Car");
  setIcon(loadPixmap("package://rviz_default_plugins/icons/classes/Pose.png"));
}

DummyObject PredictedCarPoseTool::createObjectMsg() const
{
  DummyObject object;
  object.header.frame_id = property_frame_->getFrameStd();
  object.header.stamp = clock_->now();

  std::vector<double> initial_pose_covariance(36, 0.0);
  std::vector<double> initial_twist_covariance(36, 0.0);
  std::vector<double> initial_accel_covariance(36, 0.0);

  initial_pose_covariance[0] = std_dev_x_->getFloat() * std_dev_x_->getFloat();
  initial_pose_covariance[7] = std_dev_y_->getFloat() * std_dev_y_->getFloat();
  initial_pose_covariance[14] = std_dev_z_->getFloat() * std_dev_z_->getFloat();
  initial_pose_covariance[35] = std_dev_theta_->getFloat() * std_dev_theta_->getFloat();
  object.initial_state.pose_covariance.covariance = initial_pose_covariance;
  object.initial_state.twist_covariance.covariance = initial_twist_covariance;
  object.initial_state.accel_covariance.covariance = initial_accel_covariance;

  object.action = DummyObject::PREDICT;  // Always predict for predicted tools
  object.classification.label = ObjectClassification::CAR;
  object.shape.type = Shape::BOUNDING_BOX;
  object.shape.dimensions.x = length_->getFloat();
  object.shape.dimensions.y = width_->getFloat();
  object.shape.dimensions.z = height_->getFloat();

  return object;
}

PredictedPedestrianPoseTool::PredictedPedestrianPoseTool()
{
  shortcut_key_ = 'p';

  topic_property_ = new rviz_common::properties::StringProperty(
    "Topic", "/planning/dummy_objects",
    "The topic on which to publish dummy object info.", getPropertyContainer(),
    SLOT(updateTopic()), this);
  std_dev_x_ = new rviz_common::properties::FloatProperty(
    "X std deviation", 0.03, "X standard deviation for initial pose [m]",
    getPropertyContainer());
  std_dev_y_ = new rviz_common::properties::FloatProperty(
    "Y std deviation", 0.03, "Y standard deviation for initial pose [m]",
    getPropertyContainer());
  std_dev_z_ = new rviz_common::properties::FloatProperty(
    "Z std deviation", 0.03, "Z standard deviation for initial pose [m]",
    getPropertyContainer());
  std_dev_theta_ = new rviz_common::properties::FloatProperty(
    "Theta std deviation", 5.0 * M_PI / 180.0, "Theta standard deviation for initial pose [rad]",
    getPropertyContainer());
  position_z_ = new rviz_common::properties::FloatProperty(
    "Z position", 0.0, "Z position for initial pose [m]", getPropertyContainer());
  velocity_ = new rviz_common::properties::FloatProperty(
    "Velocity", 0.0, "velocity [m/s]", getPropertyContainer());
  max_velocity_ = new rviz_common::properties::FloatProperty(
    "Max velocity", 10.0, "Max velocity [m/s]", getPropertyContainer());
  min_velocity_ = new rviz_common::properties::FloatProperty(
    "Min velocity", -10.0, "Min velocity [m/s]", getPropertyContainer());
  accel_ = new rviz_common::properties::FloatProperty(
    "Acceleration", 0.0, "Acceleration [m/s^2]", getPropertyContainer());
  width_ = new rviz_common::properties::FloatProperty(
    "Width", 0.6, "Width [m]", getPropertyContainer());
  length_ = new rviz_common::properties::FloatProperty(
    "Length", 0.6, "Length [m]", getPropertyContainer());
  height_ = new rviz_common::properties::FloatProperty(
    "Height", 1.8, "Height [m]", getPropertyContainer());
  std_dev_x_->setMin(0);
  std_dev_y_->setMin(0);
  std_dev_z_->setMin(0);
  std_dev_theta_->setMin(0);
  position_z_->setMin(-100);
  position_z_->setMax(100);

  enable_interactive_property_ = new rviz_common::properties::BoolProperty(
    "Interactive", false, "Enable/Disable interactive action by manipulating mouse.",
    getPropertyContainer());
  predicted_property_ = new rviz_common::properties::BoolProperty(
    "Predicted", true, "Enable/Disable predicted object mode using trajectory predictions.",
    getPropertyContainer());
  predicted_property_->setHidden(true);  // Always true for predicted tools
  property_frame_ = new rviz_common::properties::TfFrameProperty(
    "Target Frame", rviz_common::properties::TfFrameProperty::FIXED_FRAME_STRING,
    "The TF frame where the point cloud is output.", getPropertyContainer(), nullptr, true);
}

void PredictedPedestrianPoseTool::onInitialize()
{
  InteractiveObjectTool::onInitialize();
  setName("2D Predicted Pedestrian");
  setIcon(loadPixmap("package://rviz_default_plugins/icons/classes/Pose.png"));
}

DummyObject PredictedPedestrianPoseTool::createObjectMsg() const
{
  DummyObject object;
  object.header.frame_id = property_frame_->getFrameStd();
  object.header.stamp = clock_->now();

  std::vector<double> initial_pose_covariance(36, 0.0);
  std::vector<double> initial_twist_covariance(36, 0.0);
  std::vector<double> initial_accel_covariance(36, 0.0);

  initial_pose_covariance[0] = std_dev_x_->getFloat() * std_dev_x_->getFloat();
  initial_pose_covariance[7] = std_dev_y_->getFloat() * std_dev_y_->getFloat();
  initial_pose_covariance[14] = std_dev_z_->getFloat() * std_dev_z_->getFloat();
  initial_pose_covariance[35] = std_dev_theta_->getFloat() * std_dev_theta_->getFloat();
  object.initial_state.pose_covariance.covariance = initial_pose_covariance;
  object.initial_state.twist_covariance.covariance = initial_twist_covariance;
  object.initial_state.accel_covariance.covariance = initial_accel_covariance;

  object.action = DummyObject::PREDICT;  // Always predict for predicted tools
  object.classification.label = ObjectClassification::PEDESTRIAN;
  object.shape.type = Shape::BOUNDING_BOX;
  object.shape.dimensions.x = length_->getFloat();
  object.shape.dimensions.y = width_->getFloat();
  object.shape.dimensions.z = height_->getFloat();

  return object;
}

PredictedBusPoseTool::PredictedBusPoseTool()
{
  shortcut_key_ = 'b';

  topic_property_ = new rviz_common::properties::StringProperty(
    "Topic", "/planning/dummy_objects",
    "The topic on which to publish dummy object info.", getPropertyContainer(),
    SLOT(updateTopic()), this);
  std_dev_x_ = new rviz_common::properties::FloatProperty(
    "X std deviation", 0.03, "X standard deviation for initial pose [m]",
    getPropertyContainer());
  std_dev_y_ = new rviz_common::properties::FloatProperty(
    "Y std deviation", 0.03, "Y standard deviation for initial pose [m]",
    getPropertyContainer());
  std_dev_z_ = new rviz_common::properties::FloatProperty(
    "Z std deviation", 0.03, "Z standard deviation for initial pose [m]",
    getPropertyContainer());
  std_dev_theta_ = new rviz_common::properties::FloatProperty(
    "Theta std deviation", 5.0 * M_PI / 180.0, "Theta standard deviation for initial pose [rad]",
    getPropertyContainer());
  position_z_ = new rviz_common::properties::FloatProperty(
    "Z position", 0.0, "Z position for initial pose [m]", getPropertyContainer());
  velocity_ = new rviz_common::properties::FloatProperty(
    "Velocity", 0.0, "velocity [m/s]", getPropertyContainer());
  max_velocity_ = new rviz_common::properties::FloatProperty(
    "Max velocity", 33.3, "Max velocity [m/s]", getPropertyContainer());
  min_velocity_ = new rviz_common::properties::FloatProperty(
    "Min velocity", -33.3, "Min velocity [m/s]", getPropertyContainer());
  accel_ = new rviz_common::properties::FloatProperty(
    "Acceleration", 0.0, "Acceleration [m/s^2]", getPropertyContainer());
  width_ = new rviz_common::properties::FloatProperty(
    "Width", 2.5, "Width [m]", getPropertyContainer());
  length_ = new rviz_common::properties::FloatProperty(
    "Length", 10.0, "Length [m]", getPropertyContainer());
  height_ = new rviz_common::properties::FloatProperty(
    "Height", 3.5, "Height [m]", getPropertyContainer());
  std_dev_x_->setMin(0);
  std_dev_y_->setMin(0);
  std_dev_z_->setMin(0);
  std_dev_theta_->setMin(0);
  position_z_->setMin(-100);
  position_z_->setMax(100);

  enable_interactive_property_ = new rviz_common::properties::BoolProperty(
    "Interactive", false, "Enable/Disable interactive action by manipulating mouse.",
    getPropertyContainer());
  predicted_property_ = new rviz_common::properties::BoolProperty(
    "Predicted", true, "Enable/Disable predicted object mode using trajectory predictions.",
    getPropertyContainer());
  predicted_property_->setHidden(true);  // Always true for predicted tools
  property_frame_ = new rviz_common::properties::TfFrameProperty(
    "Target Frame", rviz_common::properties::TfFrameProperty::FIXED_FRAME_STRING,
    "The TF frame where the point cloud is output.", getPropertyContainer(), nullptr, true);
}

void PredictedBusPoseTool::onInitialize()
{
  InteractiveObjectTool::onInitialize();
  setName("2D Predicted Bus");
  setIcon(loadPixmap("package://rviz_default_plugins/icons/classes/Pose.png"));
}

DummyObject PredictedBusPoseTool::createObjectMsg() const
{
  DummyObject object;
  object.header.frame_id = property_frame_->getFrameStd();
  object.header.stamp = clock_->now();

  std::vector<double> initial_pose_covariance(36, 0.0);
  std::vector<double> initial_twist_covariance(36, 0.0);
  std::vector<double> initial_accel_covariance(36, 0.0);

  initial_pose_covariance[0] = std_dev_x_->getFloat() * std_dev_x_->getFloat();
  initial_pose_covariance[7] = std_dev_y_->getFloat() * std_dev_y_->getFloat();
  initial_pose_covariance[14] = std_dev_z_->getFloat() * std_dev_z_->getFloat();
  initial_pose_covariance[35] = std_dev_theta_->getFloat() * std_dev_theta_->getFloat();
  object.initial_state.pose_covariance.covariance = initial_pose_covariance;
  object.initial_state.twist_covariance.covariance = initial_twist_covariance;
  object.initial_state.accel_covariance.covariance = initial_accel_covariance;

  object.action = DummyObject::PREDICT;  // Always predict for predicted tools
  object.classification.label = ObjectClassification::BUS;
  object.shape.type = Shape::BOUNDING_BOX;
  object.shape.dimensions.x = length_->getFloat();
  object.shape.dimensions.y = width_->getFloat();
  object.shape.dimensions.z = height_->getFloat();

  return object;
}

PredictedBikePoseTool::PredictedBikePoseTool()
{
  shortcut_key_ = 'k';

  topic_property_ = new rviz_common::properties::StringProperty(
    "Topic", "/planning/dummy_objects",
    "The topic on which to publish dummy object info.", getPropertyContainer(),
    SLOT(updateTopic()), this);
  std_dev_x_ = new rviz_common::properties::FloatProperty(
    "X std deviation", 0.03, "X standard deviation for initial pose [m]",
    getPropertyContainer());
  std_dev_y_ = new rviz_common::properties::FloatProperty(
    "Y std deviation", 0.03, "Y standard deviation for initial pose [m]",
    getPropertyContainer());
  std_dev_z_ = new rviz_common::properties::FloatProperty(
    "Z std deviation", 0.03, "Z standard deviation for initial pose [m]",
    getPropertyContainer());
  std_dev_theta_ = new rviz_common::properties::FloatProperty(
    "Theta std deviation", 5.0 * M_PI / 180.0, "Theta standard deviation for initial pose [rad]",
    getPropertyContainer());
  position_z_ = new rviz_common::properties::FloatProperty(
    "Z position", 0.0, "Z position for initial pose [m]", getPropertyContainer());
  velocity_ = new rviz_common::properties::FloatProperty(
    "Velocity", 0.0, "velocity [m/s]", getPropertyContainer());
  max_velocity_ = new rviz_common::properties::FloatProperty(
    "Max velocity", 16.66, "Max velocity [m/s]", getPropertyContainer());
  min_velocity_ = new rviz_common::properties::FloatProperty(
    "Min velocity", -16.66, "Min velocity [m/s]", getPropertyContainer());
  accel_ = new rviz_common::properties::FloatProperty(
    "Acceleration", 0.0, "Acceleration [m/s^2]", getPropertyContainer());
  width_ = new rviz_common::properties::FloatProperty(
    "Width", 0.8, "Width [m]", getPropertyContainer());
  length_ = new rviz_common::properties::FloatProperty(
    "Length", 2.0, "Length [m]", getPropertyContainer());
  height_ = new rviz_common::properties::FloatProperty(
    "Height", 1.5, "Height [m]", getPropertyContainer());
  std_dev_x_->setMin(0);
  std_dev_y_->setMin(0);
  std_dev_z_->setMin(0);
  std_dev_theta_->setMin(0);
  position_z_->setMin(-100);
  position_z_->setMax(100);

  enable_interactive_property_ = new rviz_common::properties::BoolProperty(
    "Interactive", false, "Enable/Disable interactive action by manipulating mouse.",
    getPropertyContainer());
  predicted_property_ = new rviz_common::properties::BoolProperty(
    "Predicted", true, "Enable/Disable predicted object mode using trajectory predictions.",
    getPropertyContainer());
  predicted_property_->setHidden(true);  // Always true for predicted tools
  label_ = new rviz_common::properties::EnumProperty(
    "Type", "bicycle", "Bike type", getPropertyContainer());
  label_->addOption("bicycle", ObjectClassification::BICYCLE);
  label_->addOption("motorcycle", ObjectClassification::MOTORCYCLE);
  property_frame_ = new rviz_common::properties::TfFrameProperty(
    "Target Frame", rviz_common::properties::TfFrameProperty::FIXED_FRAME_STRING,
    "The TF frame where the point cloud is output.", getPropertyContainer(), nullptr, true);
}

void PredictedBikePoseTool::onInitialize()
{
  InteractiveObjectTool::onInitialize();
  setName("2D Predicted Bike");
  setIcon(loadPixmap("package://rviz_default_plugins/icons/classes/Pose.png"));
}

DummyObject PredictedBikePoseTool::createObjectMsg() const
{
  DummyObject object;
  object.header.frame_id = property_frame_->getFrameStd();
  object.header.stamp = clock_->now();

  std::vector<double> initial_pose_covariance(36, 0.0);
  std::vector<double> initial_twist_covariance(36, 0.0);
  std::vector<double> initial_accel_covariance(36, 0.0);

  initial_pose_covariance[0] = std_dev_x_->getFloat() * std_dev_x_->getFloat();
  initial_pose_covariance[7] = std_dev_y_->getFloat() * std_dev_y_->getFloat();
  initial_pose_covariance[14] = std_dev_z_->getFloat() * std_dev_z_->getFloat();
  initial_pose_covariance[35] = std_dev_theta_->getFloat() * std_dev_theta_->getFloat();
  object.initial_state.pose_covariance.covariance = initial_pose_covariance;
  object.initial_state.twist_covariance.covariance = initial_twist_covariance;
  object.initial_state.accel_covariance.covariance = initial_accel_covariance;

  object.action = DummyObject::PREDICT;  // Always predict for predicted tools
  object.classification.label = label_->getOptionInt();
  object.shape.type = Shape::BOUNDING_BOX;
  object.shape.dimensions.x = length_->getFloat();
  object.shape.dimensions.y = width_->getFloat();
  object.shape.dimensions.z = height_->getFloat();

  return object;
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::PredictedCarPoseTool, rviz_common::Tool)
PLUGINLIB_EXPORT_CLASS(rviz_plugins::PredictedPedestrianPoseTool, rviz_common::Tool)
PLUGINLIB_EXPORT_CLASS(rviz_plugins::PredictedBusPoseTool, rviz_common::Tool)
PLUGINLIB_EXPORT_CLASS(rviz_plugins::PredictedBikePoseTool, rviz_common::Tool)