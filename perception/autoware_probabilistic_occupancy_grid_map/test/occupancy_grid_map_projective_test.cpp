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

#include "autoware/probabilistic_occupancy_grid_map/costmap_2d/occupancy_grid_map_projective.hpp"

#include "autoware/probabilistic_occupancy_grid_map/cost_value/cost_value.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cuda_runtime.h>
#include <gtest/gtest.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <memory>
#include <vector>

// create pointcloud function
sensor_msgs::msg::PointCloud2::SharedPtr createPointCloud(const std::vector<pcl::PointXYZ> & points)
{
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl_cloud.points.assign(points.begin(), points.end());
  pcl_cloud.width = points.size();
  pcl_cloud.height = 1;
  auto ros_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(pcl_cloud, *ros_cloud);
  return ros_cloud;
}

// create pose function
geometry_msgs::msg::Pose createPose(const double x, const double y, const double z)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.w = 1.0;
  return pose;
}

// Test the shadow of an obstacle point at or above the scan origin height
TEST(OccupancyGridMapProjectiveBlindSpotTest, TestShadowOfObstacleAboveScanOrigin)
{
  using autoware::occupancy_grid_map::cost_value::FREE_SPACE;
  using autoware::occupancy_grid_map::cost_value::LETHAL_OBSTACLE;
  using autoware::occupancy_grid_map::cost_value::NO_INFORMATION;
  using autoware::occupancy_grid_map::costmap_2d::OccupancyGridMapProjectiveBlindSpot;

  rclcpp::init(0, nullptr);
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {{"OccupancyGridMapProjectiveBlindSpot.projection_dz_threshold", 0.01},
     {"OccupancyGridMapProjectiveBlindSpot.obstacle_separation_threshold", 1.0}});
  auto node = std::make_shared<rclcpp::Node>("occupancy_grid_map_projective_test", options);

  cudaStream_t stream;
  ASSERT_EQ(cudaStreamCreate(&stream), cudaSuccess);

  // 50 m x 50 m map around the robot at the origin, with the scan origin 2 m above the robot
  OccupancyGridMapProjectiveBlindSpot map(true, 100, 100, 0.5);
  map.initRosParam(*node->get_node_parameters_interface());
  map.setCudaStream(stream);
  map.resetMaps();
  map.updateOrigin(-25.0, -25.0);

  // points on the x axis: the ground up to 9 m, an obstacle 2.5 m high at 10 m, and a raw point
  // 5 m high at 20 m that is seen above the obstacle
  std::vector<pcl::PointXYZ> raw_points;
  for (float x = 1.0f; x < 9.5f; x += 0.5f) {
    raw_points.emplace_back(x, 0.0f, 0.0f);
  }
  raw_points.emplace_back(10.0f, 0.0f, 2.5f);
  raw_points.emplace_back(20.0f, 0.0f, 5.0f);
  const std::vector<pcl::PointXYZ> obstacle_points = {pcl::PointXYZ(10.0f, 0.0f, 2.5f)};

  CudaPointCloud2 raw_pointcloud;
  CudaPointCloud2 obstacle_pointcloud;
  raw_pointcloud.stream = stream;
  obstacle_pointcloud.stream = stream;
  raw_pointcloud.fromROSMsgAsync(createPointCloud(raw_points));
  obstacle_pointcloud.fromROSMsgAsync(createPointCloud(obstacle_points));

  map.updateWithPointCloud(
    raw_pointcloud, obstacle_pointcloud, createPose(0.0, 0.0, 0.0), createPose(0.0, 0.0, 2.0));
  map.copyDeviceCostmapToHost();

  const auto cost_at = [&](const double x) {
    unsigned int mx = 0;
    unsigned int my = 0;
    EXPECT_TRUE(map.worldToMap(x, 0.0, mx, my));
    return map.getCost(mx, my);
  };

  // in front of the obstacle: free
  for (double x = 1.25; x < 9.0; x += 0.5) {
    EXPECT_EQ(cost_at(x), FREE_SPACE) << "x = " << x;
  }
  // the obstacle
  EXPECT_EQ(cost_at(10.25), LETHAL_OBSTACLE);
  // behind the obstacle up to the map boundary: no information, although the raw point at 20 m
  // makes the cells up to it free first
  for (double x = 11.25; x < 24.5; x += 0.5) {
    EXPECT_EQ(cost_at(x), NO_INFORMATION) << "x = " << x;
  }

  cudaStreamDestroy(stream);
  rclcpp::shutdown();
}
