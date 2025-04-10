// Copyright 2023 TIER IV, Inc.
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

#ifndef TIER4_PLANNING_RVIZ_PLUGIN__PATH__DISPLAY_HPP_
#define TIER4_PLANNING_RVIZ_PLUGIN__PATH__DISPLAY_HPP_

#include "tier4_planning_rviz_plugin/path/display_base.hpp"

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_planning_msgs/msg/detail/trajectory__struct.hpp>
#include <autoware_planning_msgs/msg/path.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>

#include <memory>
#include <utility>
#include <vector>

namespace
{
bool validateBoundFloats(const std::vector<geometry_msgs::msg::Point> & bound)
{
  for (auto && point : bound) {
    if (!rviz_common::validateFloats(point)) {
      return false;
    }
  }
  return true;
}

template <typename T>
bool validateBoundFloats(const typename T::ConstSharedPtr & msg_ptr)
{
  return validateBoundFloats(msg_ptr->left_bound) && validateBoundFloats(msg_ptr->right_bound);
}

void visualizeBound(
  const std::vector<geometry_msgs::msg::Point> & bound, const Ogre::ColourValue & color,
  const float width, Ogre::ManualObject * bound_object)
{
  if (bound.size() < 2) {
    return;
  }

  bound_object->estimateVertexCount(bound.size() * 2);
  bound_object->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_STRIP);

  // calculate normal vector of bound and widths depending on the normal vector
  std::vector<float> normal_vector_angles;
  std::vector<float> adaptive_widths;
  for (size_t i = 0; i < bound.size(); ++i) {
    const auto [normal_vector_angle, adaptive_width] = [&]() -> std::pair<float, float> {
      if (i == 0) {
        return std::make_pair(
          autoware_utils::calc_azimuth_angle(bound.at(i), bound.at(i + 1)) + M_PI_2, width);
      }
      if (i == bound.size() - 1) {
        return std::make_pair(
          autoware_utils::calc_azimuth_angle(bound.at(i - 1), bound.at(i)) + M_PI_2, width);
      }
      const auto & prev_p = bound.at(i - 1);
      const auto & curr_p = bound.at(i);
      const auto & next_p = bound.at(i + 1);

      const float curr_to_prev_angle = autoware_utils::calc_azimuth_angle(curr_p, prev_p);
      const float curr_to_next_angle = autoware_utils::calc_azimuth_angle(curr_p, next_p);
      const float normal_vector_angle = (curr_to_prev_angle + curr_to_next_angle) / 2.0;

      const float diff_angle =
        autoware_utils::normalize_radian(normal_vector_angle - curr_to_next_angle);
      if (diff_angle <= 1e-7 && diff_angle >= -1e-7) {
        return std::make_pair(normal_vector_angle, width);
      }

      return std::make_pair(normal_vector_angle, width / std::sin(diff_angle));
    }();

    normal_vector_angles.push_back(normal_vector_angle);
    adaptive_widths.push_back(adaptive_width);
  }

  // calculate triangle
  for (size_t i = 0; i < bound.size(); ++i) {
    const float normal_vector_angle = normal_vector_angles.at(i);
    const float adaptive_width = adaptive_widths.at(i);

    const auto x_offset = static_cast<float>(adaptive_width * 0.5 * std::cos(normal_vector_angle));
    const auto y_offset = static_cast<float>(adaptive_width * 0.5 * std::sin(normal_vector_angle));
    auto target_lp = bound.at(i);
    target_lp.x = target_lp.x + x_offset;
    target_lp.y = target_lp.y + y_offset;
    target_lp.z = target_lp.z;
    bound_object->position(target_lp.x, target_lp.y, target_lp.z);
    bound_object->colour(color);
    auto target_rp = bound.at(i);
    target_rp.x = target_rp.x - x_offset;
    target_rp.y = target_rp.y - y_offset;
    target_rp.z = target_rp.z;
    bound_object->position(target_rp.x, target_rp.y, target_rp.z);
    bound_object->colour(color);
  }
  bound_object->end();
}
}  // namespace

namespace rviz_plugins
{
template <class T>
class AutowarePathWithDrivableAreaDisplay : public AutowarePathBaseDisplay<T>
{
public:
  AutowarePathWithDrivableAreaDisplay()
  : property_drivable_area_view_{"View Drivable Area", true, "", this},
    property_drivable_area_alpha_{"Alpha", 0.999, "", &property_drivable_area_view_},
    property_drivable_area_color_{"Color", QColor(0, 148, 205), "", &property_drivable_area_view_},
    property_drivable_area_width_{"Width", 0.3f, "", &property_drivable_area_view_}
  {
    property_drivable_area_alpha_.setMin(0.0);
    property_drivable_area_alpha_.setMax(1.0);
    property_drivable_area_width_.setMin(0.001);
  }

  ~AutowarePathWithDrivableAreaDisplay()
  {
    if (this->initialized()) {
      this->scene_manager_->destroyManualObject(left_bound_object_);
      this->scene_manager_->destroyManualObject(right_bound_object_);
    }
  }
  void onInitialize() override
  {
    AutowarePathBaseDisplay<T>::onInitialize();
    left_bound_object_ = this->scene_manager_->createManualObject();
    right_bound_object_ = this->scene_manager_->createManualObject();
    left_bound_object_->setDynamic(true);
    right_bound_object_->setDynamic(true);
    this->scene_node_->attachObject(left_bound_object_);
    this->scene_node_->attachObject(right_bound_object_);
  }

  void reset() override
  {
    AutowarePathBaseDisplay<T>::reset();
    left_bound_object_->clear();
    right_bound_object_->clear();
  }

protected:
  void visualizeDrivableArea(const typename T::ConstSharedPtr msg_ptr) override
  {
    left_bound_object_->clear();
    right_bound_object_->clear();

    if (!validateBoundFloats<T>(msg_ptr)) {
      this->setStatus(
        rviz_common::properties::StatusProperty::Error, "Topic",
        "Message drivable area contained invalid floating point values (nans or infs)");
      return;
    }

    if (property_drivable_area_view_.getBool()) {
      Ogre::ColourValue color =
        rviz_common::properties::qtToOgre(property_drivable_area_color_.getColor());
      color.a = property_drivable_area_alpha_.getFloat();
      const auto line_width = property_drivable_area_width_.getFloat();
      visualizeBound(msg_ptr->left_bound, color, line_width, left_bound_object_);
      visualizeBound(msg_ptr->right_bound, color, line_width, right_bound_object_);
    }
  }

private:
  Ogre::ManualObject * left_bound_object_{nullptr};
  Ogre::ManualObject * right_bound_object_{nullptr};

  rviz_common::properties::BoolProperty property_drivable_area_view_;
  rviz_common::properties::FloatProperty property_drivable_area_alpha_;
  rviz_common::properties::ColorProperty property_drivable_area_color_;
  rviz_common::properties::FloatProperty property_drivable_area_width_;
};

class AutowarePathWithLaneIdDisplay
: public AutowarePathWithDrivableAreaDisplay<autoware_internal_planning_msgs::msg::PathWithLaneId>
{
  Q_OBJECT
public:
  AutowarePathWithLaneIdDisplay();
  ~AutowarePathWithLaneIdDisplay();

private:
  void preProcessMessageDetail() override;
  void preVisualizePathFootprintDetail(
    const autoware_internal_planning_msgs::msg::PathWithLaneId::ConstSharedPtr msg_ptr) override;
  void visualizePathFootprintDetail(
    const autoware_internal_planning_msgs::msg::PathWithLaneId::ConstSharedPtr msg_ptr,
    const size_t p_idx) override;

  rviz_common::properties::BoolProperty property_lane_id_view_;
  rviz_common::properties::FloatProperty property_lane_id_scale_;

  using LaneIdObject =
    std::pair<std::unique_ptr<Ogre::SceneNode>, std::unique_ptr<rviz_rendering::MovableText>>;
  std::vector<LaneIdObject> lane_id_obj_ptrs_;
};

class AutowarePathDisplay
: public AutowarePathWithDrivableAreaDisplay<autoware_planning_msgs::msg::Path>
{
  Q_OBJECT
private:
  void preProcessMessageDetail() override;
};

class AutowareTrajectoryDisplay
: public AutowarePathBaseDisplay<autoware_planning_msgs::msg::Trajectory>
{
  Q_OBJECT
  std::vector<rviz_rendering::MovableText *> time_texts_;
  std::vector<Ogre::SceneNode *> time_text_nodes_;
  rviz_common::properties::BoolProperty property_time_text_view_;
  rviz_common::properties::FloatProperty property_time_text_scale_;

public:
  // time text
  AutowareTrajectoryDisplay()
  : property_time_text_view_{"View Text Time", false, "", this},
    property_time_text_scale_{"Scale", 0.3, "", &property_time_text_view_}
  {
  }

  ~AutowareTrajectoryDisplay() override
  {
    if (this->initialized()) {
      for (auto node : time_text_nodes_) {
        node->removeAndDestroyAllChildren();
        node->detachAllObjects();
        this->scene_manager_->destroySceneNode(node);
      }
    }
  }

  void reset() override
  {
    AutowarePathBaseDisplay<autoware_planning_msgs::msg::Trajectory>::reset();
    for (size_t i = 0; i < time_texts_.size(); i++) {
      Ogre::SceneNode * node = time_text_nodes_.at(i);
      node->detachAllObjects();
      node->removeAndDestroyAllChildren();
      this->scene_manager_->destroySceneNode(node);
    }
    time_text_nodes_.clear();
    time_texts_.clear();
  }

  void visualizeTrajectory(
    const typename autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg_ptr) override
  {
    if (msg_ptr->points.size() > time_texts_.size()) {
      for (size_t i = time_texts_.size(); i < msg_ptr->points.size(); i++) {
        Ogre::SceneNode * node = this->scene_node_->createChildSceneNode();
        auto * text = new rviz_rendering::MovableText("not initialized", "Liberation Sans", 0.1);
        text->setVisible(false);
        text->setTextAlignment(
          rviz_rendering::MovableText::H_CENTER, rviz_rendering::MovableText::V_ABOVE);
        node->attachObject(text);
        time_texts_.push_back(text);
        time_text_nodes_.push_back(node);
      }
    } else if (msg_ptr->points.size() < time_texts_.size()) {
      for (size_t i = time_texts_.size() - 1; i >= msg_ptr->points.size(); i--) {
        Ogre::SceneNode * node = time_text_nodes_.at(i);
        node->detachAllObjects();
        node->removeAndDestroyAllChildren();
        this->scene_manager_->destroySceneNode(node);

        rviz_rendering::MovableText * text = time_texts_.at(i);
        delete text;
      }
      time_texts_.resize(msg_ptr->points.size());
      time_text_nodes_.resize(msg_ptr->points.size());
    }
    for (size_t point_idx = 0; point_idx < msg_ptr->points.size(); point_idx++) {
      const auto & trajectory_point = msg_ptr->points.at(point_idx);
      const auto & pose = autoware_utils::get_pose(trajectory_point);
      const auto & time = trajectory_point.time_from_start;
      // time text
      if (property_time_text_view_.getBool()) {
        Ogre::Vector3 position;
        position.x = pose.position.x;
        position.y = pose.position.y;
        position.z = pose.position.z;
        Ogre::SceneNode * node = time_text_nodes_.at(point_idx);
        node->setPosition(position);

        rviz_rendering::MovableText * text = time_texts_.at(point_idx);
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2) << rclcpp::Duration(time).seconds();
        text->setCaption(ss.str());
        text->setCharacterHeight(property_time_text_scale_.getFloat());
        text->setVisible(true);
      } else {
        rviz_rendering::MovableText * text = time_texts_.at(point_idx);
        text->setVisible(false);
      }
    }
  }

private:
  void preProcessMessageDetail() override;
};
}  // namespace rviz_plugins

#endif  // TIER4_PLANNING_RVIZ_PLUGIN__PATH__DISPLAY_HPP_
