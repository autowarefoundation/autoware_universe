// Copyright 2025 TIER IV, Inc.
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

#pragma once

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace autoware::pointcloud_preprocessor
{

struct PointCloud2Traits
{
  using PointCloudMessage = sensor_msgs::msg::PointCloud2;
  using PublisherType = rclcpp::Publisher<PointCloudMessage>;
  using SubscriberType = rclcpp::Subscription<PointCloudMessage>;
};

}  // namespace autoware::pointcloud_preprocessor
