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

#ifndef AUTOWARE__LOCALIZED_POSITION_VALIDATOR__NODE_HPP_
#define AUTOWARE__LOCALIZED_POSITION_VALIDATOR__NODE_HPP_

#include "position_validator.hpp"

#include <autoware_utils_debug/debug_publisher.hpp>
#include <autoware_utils_system/stop_watch.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_map_msgs/srv/get_partial_point_cloud_map.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tier4_localization_msgs/msg/localized_position_validator_prediction.hpp>

#include <mutex>

namespace autoware::localized_position_validator
{
using ValidatorPredictionMsg = tier4_localization_msgs::msg::LocalizedPositionValidatorPrediction;

struct MapQueriedPos{
  double x;
  double y;
};

class LocalizedPositionValidatorNode : public rclcpp::Node
{
public:
  explicit LocalizedPositionValidatorNode(const rclcpp::NodeOptions & node_options);
  ~LocalizedPositionValidatorNode();

private:
  void update_belief_with_observation(const int & z);
  void update_belief_with_confidence(const float & valid_confidence);
  void callback_pointcloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_pointcloud_msg_ptr);
  void callback_map(const sensor_msgs::msg::PointCloud2::ConstSharedPtr map_pointcloud_msg_ptr);
  void callback_odometry(const nav_msgs::msg::Odometry::ConstSharedPtr odometry_msg_ptr);
  void process_map(
    rclcpp::Client<autoware_map_msgs::srv::GetPartialPointCloudMap>::SharedFuture response);
  void on_map_response(
    rclcpp::Client<autoware_map_msgs::srv::GetPartialPointCloudMap>::SharedFuture response);
  void get_partial_point_cloud_map(const float & x, const float & y);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Client<autoware_map_msgs::srv::GetPartialPointCloudMap>::SharedPtr pcd_map_cli_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  rclcpp::Publisher<ValidatorPredictionMsg>::SharedPtr inference_result_pub_;

  // debug
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_input_point_pub_;

  pcl::PointCloud<pcl::PointXYZ> pcd_map_;
  nav_msgs::msg::Odometry vcl_odom_;

  MapQueriedPos map_queried_pos_;
  float map_fetch_range_;
  double map_update_threshold_dist_;

  rclcpp::Duration max_time_diff_{std::chrono::milliseconds(300)};

  bool partial_load_ = false;
  bool map_arrived_ = false;
  bool vcl_odom_arrived_ = false;
  bool is_map_request_in_progress_ = false;

  float pcd_min_x_range_, pcd_max_x_range_;
  float pcd_min_y_range_, pcd_max_y_range_;
  float pcd_min_z_range_, pcd_max_z_range_;
  float map_downsample_res_;

  float p_z_given_s_[2][2]; // [observation][state]
  float belief_valid_ = 0.5;
  bool use_soft_bayesian_update_;

  int profile_points_min_;
  int profile_points_max_;

  int predicted_class_;
  float confidence_;

  std::unique_ptr<PositionValidatorTRT> validator_ptr_;

  std::mutex mutex_map_;
  std::vector<std::thread> worker_threads_;

  // debugger
  bool publish_input_points_;
  std::unique_ptr<autoware_utils_system::StopWatch<std::chrono::milliseconds>>
    stop_watch_ptr_{nullptr};
  std::unique_ptr<autoware_utils_debug::DebugPublisher> debug_publisher_ptr_{nullptr};
};

}  // namespace autoware::localized_position_validator

#endif  // AUTOWARE__LOCALIZED_POSITION_VALIDATOR__NODE_HPP_
