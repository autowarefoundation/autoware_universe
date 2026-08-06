// Copyright 2026 The Autoware Contributors
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
#ifndef OBSTACLE_GRID_EXTRACTOR_NODE_HPP_
#define OBSTACLE_GRID_EXTRACTOR_NODE_HPP_

#include "autoware/obstacle_grid_extractor/obstacle_grid_extractor.hpp"

#include <rclcpp/rclcpp.hpp>

#include <grid_map_msgs/msg/grid_map.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>

namespace autoware::obstacle_grid_extractor
{
class ObstacleGridExtractorNode : public rclcpp::Node
{
public:
  explicit ObstacleGridExtractorNode(const rclcpp::NodeOptions & options);

private:
  void onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  std::unique_ptr<ObstacleGridExtractor> extractor_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};
}  // namespace autoware::obstacle_grid_extractor
#endif  // OBSTACLE_GRID_EXTRACTOR_NODE_HPP_
