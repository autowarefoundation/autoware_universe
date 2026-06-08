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

#include "autoware/map_loader/plugins/vector_map_tf_generator_plugin.hpp"

#include "autoware/map_loader/map_loader_plugin_base.hpp"

#include <autoware_utils/ros/parameter.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <tf2_ros/static_transform_broadcaster.h>

#include <memory>
#include <string>

namespace autoware::map_loader::plugin
{

class VectorMapTfGeneratorPlugin : public MapLoaderPluginBase
{
public:
  void initialize(
    const std::string & /*name*/, rclcpp::Node * node, MapLoaderData & /*data*/) override
  {
    node_ = node;
    using autoware_utils_rclcpp::get_or_declare_parameter;
    map_frame_ = get_or_declare_parameter<std::string>(*node, "map_frame");
    viewer_frame_ = get_or_declare_parameter<std::string>(*node, "viewer_frame");
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);
  }

  void on_startup(MapLoaderData & data) override
  {
    if (!data.vector_map_msg.has_value()) {
      RCLCPP_WARN(
        get_node_ptr()->get_logger(), "Vector map is not loaded; skipping TF generation.");
      return;
    }

    const auto transform = autoware::map_tf_generator::create_viewer_transform(
      data.vector_map_msg.value(), map_frame_, viewer_frame_, node_->now());
    static_broadcaster_->sendTransform(transform);

    RCLCPP_INFO_STREAM(
      get_node_ptr()->get_logger(),
      "Broadcast static tf. map_frame:" << map_frame_ << ", viewer_frame:" << viewer_frame_);
  }

private:
  rclcpp::Node * node_{nullptr};
  std::string map_frame_;
  std::string viewer_frame_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
};

}  // namespace autoware::map_loader::plugin

PLUGINLIB_EXPORT_CLASS(
  autoware::map_loader::plugin::VectorMapTfGeneratorPlugin,
  autoware::map_loader::plugin::MapLoaderPluginBase)
