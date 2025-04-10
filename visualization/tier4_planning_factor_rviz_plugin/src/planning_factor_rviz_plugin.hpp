// Copyright 2024 TIER IV, Inc.
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

#ifndef PLANNING_FACTOR_RVIZ_PLUGIN_HPP_
#define PLANNING_FACTOR_RVIZ_PLUGIN_HPP_

#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_default_plugins/displays/marker/marker_common.hpp>
#include <rviz_default_plugins/displays/marker_array/marker_array_display.hpp>

#include <autoware_internal_planning_msgs/msg/planning_factor_array.hpp>

#include <string>

namespace autoware::rviz_plugins
{

using RosTopicDisplay =
  rviz_common::RosTopicDisplay<autoware_internal_planning_msgs::msg::PlanningFactorArray>;

class PlanningFactorRvizPlugin
: public rviz_common::RosTopicDisplay<autoware_internal_planning_msgs::msg::PlanningFactorArray>
{
public:
  PlanningFactorRvizPlugin()
  : marker_common_{this},
    baselink2front_{"Baselink To Front", 0.0, "Length between base link to front.", this},
    topic_name_{"planning_factors"}
  {
  }

  void onInitialize() override
  {
    RosTopicDisplay::RTDClass::onInitialize();
    marker_common_.initialize(this->context_, this->scene_node_);
    QString message_type = QString::fromStdString(
      rosidl_generator_traits::name<autoware_internal_planning_msgs::msg::PlanningFactorArray>());
    this->topic_property_->setMessageType(message_type);
    this->topic_property_->setValue(topic_name_.c_str());
    this->topic_property_->setDescription("Topic to subscribe to.");

    const auto vehicle_info =
      autoware::vehicle_info_utils::VehicleInfoUtils(*rviz_ros_node_.lock()->get_raw_node())
        .getVehicleInfo();
    baselink2front_.setValue(vehicle_info.max_longitudinal_offset_m);
  }

  void load(const rviz_common::Config & config) override
  {
    RosTopicDisplay::Display::load(config);
    marker_common_.load(config);
  }

  void update(float wall_dt, float ros_dt) override { marker_common_.update(wall_dt, ros_dt); }

  void reset() override
  {
    RosTopicDisplay::reset();
    marker_common_.clearMarkers();
  }

  void clear_markers() { marker_common_.clearMarkers(); }

  void add_marker(visualization_msgs::msg::Marker::ConstSharedPtr marker_ptr)
  {
    marker_common_.addMessage(marker_ptr);
  }

  void add_marker(visualization_msgs::msg::MarkerArray::ConstSharedPtr markers_ptr)
  {
    marker_common_.addMessage(markers_ptr);
  }

  void delete_marker(rviz_default_plugins::displays::MarkerID marker_id)
  {
    marker_common_.deleteMarker(marker_id);
  }

private:
  void processMessage(
    const autoware_internal_planning_msgs::msg::PlanningFactorArray::ConstSharedPtr msg) override;

  rviz_default_plugins::displays::MarkerCommon marker_common_;

  rviz_common::properties::FloatProperty baselink2front_;

  std::string topic_name_;
};
}  // namespace autoware::rviz_plugins

#endif  // PLANNING_FACTOR_RVIZ_PLUGIN_HPP_
