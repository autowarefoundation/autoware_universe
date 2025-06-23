// Copyright 2020 Tier IV, Inc. All rights reserved.
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

#include "livox_tag_filter_node.hpp"

#include "autoware/point_types/types.hpp"

#include <pcl_conversions/pcl_conversions.h>

#include <memory>
#include <utility>
#include <vector>

struct EIGEN_ALIGN16 LivoxPoint
{
  float x;          // cppcheck-suppress unusedStructMember
  float y;          // cppcheck-suppress unusedStructMember
  float z;          // cppcheck-suppress unusedStructMember
  float intensity;  // cppcheck-suppress unusedStructMember
  std::uint8_t tag;
  std::uint8_t line;
};

POINT_CLOUD_REGISTER_POINT_STRUCT(
  LivoxPoint, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                std::uint8_t, tag, tag)(std::uint8_t, line, line))

namespace autoware::livox_tag_filter
{
using autoware::point_types::PointXYZIRC;
LivoxTagFilterNode::LivoxTagFilterNode(const rclcpp::NodeOptions & node_options)
: Node("livox_tag_filter", node_options)
{
  // Parameter
  ignore_tags_ = this->declare_parameter<std::vector<std::int64_t>>("ignore_tags");

  // Subscriber
  using std::placeholders::_1;
  sub_pointcloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "input", rclcpp::SensorDataQoS(), std::bind(&LivoxTagFilterNode::onPointCloud, this, _1));

  {
    // Publisher
    rclcpp::PublisherOptions pub_options;
    pub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
    pub_pointcloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "output", rclcpp::SensorDataQoS(), pub_options);
  }
}

void LivoxTagFilterNode::onPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  pcl::PointCloud<LivoxPoint> points;
  pcl::fromROSMsg(*msg, points);

  const auto isIgnored = [&](const LivoxPoint & p) {
    for (const auto & ignore_tag : ignore_tags_) {
      if (p.tag == ignore_tag) {
        return true;
      }
    }

    return false;
  };

  pcl::PointCloud<LivoxPoint> tag_filtered_points;
  for (const auto & p : points) {
    if (isIgnored(p)) {
      continue;
    }

    tag_filtered_points.push_back(p);
  }
  // convert pointcloud type to xyzirc
  pcl::PointCloud<PointXYZIRC>::Ptr tag_filtered_points_xyzirc =
    std::make_shared<pcl::PointCloud<PointXYZIRC>>();

  tag_filtered_points_xyzirc->resize(tag_filtered_points.size());
  for (size_t i = 0; i < tag_filtered_points.size(); ++i) {
    const auto & p = tag_filtered_points[i];
    (*tag_filtered_points_xyzirc)[i].x = p.x;
    (*tag_filtered_points_xyzirc)[i].y = p.y;
    (*tag_filtered_points_xyzirc)[i].z = p.z;
    (*tag_filtered_points_xyzirc)[i].intensity = p.intensity;
    (*tag_filtered_points_xyzirc)[i].return_type = 0;
    (*tag_filtered_points_xyzirc)[i].channel = 0;
  }

  // Publish ROS message
  auto tag_filtered_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(*tag_filtered_points_xyzirc, *tag_filtered_msg_ptr);
  tag_filtered_msg_ptr->header = msg->header;

  pub_pointcloud_->publish(std::move(tag_filtered_msg_ptr));
}

}  // namespace autoware::livox_tag_filter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::livox_tag_filter::LivoxTagFilterNode)
