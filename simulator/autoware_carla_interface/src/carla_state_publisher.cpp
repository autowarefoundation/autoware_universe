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

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2_ros/transform_broadcaster.h>

#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

namespace autoware::carla_interface
{

class CarlaStatePublisher : public rclcpp::Node
{
public:
  explicit CarlaStatePublisher(const rclcpp::NodeOptions & options)
  : rclcpp::Node("carla_state_publisher", options)
  {
    frame_id_ = this->declare_parameter<std::string>("frame_id", "map");
    child_frame_id_ = this->declare_parameter<std::string>("child_frame_id", "base_link");
    publish_tf_ = this->declare_parameter<bool>("publish_tf", true);
    max_time_diff_sec_ = this->declare_parameter<double>("max_time_diff_sec", 0.2);

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "~/input/pose_with_covariance", rclcpp::QoS(10),
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_pose_ = msg;
        }
        publish_odometry_if_ready();
      });

    twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      "~/input/twist_with_covariance", rclcpp::QoS(50),
      [this](const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr msg) {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_twist_ = msg;
        }
        publish_odometry_if_ready();
      });

    odom_pub_ =
      this->create_publisher<nav_msgs::msg::Odometry>("~/output/odometry", rclcpp::QoS(10));

    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    RCLCPP_INFO(
      this->get_logger(), "carla_state_publisher initialized (frame_id=%s, child_frame_id=%s)",
      frame_id_.c_str(), child_frame_id_.c_str());
  }

private:
  using PoseMsg = geometry_msgs::msg::PoseWithCovarianceStamped;
  using TwistMsg = geometry_msgs::msg::TwistWithCovarianceStamped;

  void publish_odometry_if_ready()
  {
    PoseMsg::ConstSharedPtr pose;
    TwistMsg::ConstSharedPtr twist;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      pose = latest_pose_;
      twist = latest_twist_;
    }

    if (!pose || !twist) {
      return;
    }

    const rclcpp::Time pose_stamp{pose->header.stamp};
    const rclcpp::Time twist_stamp{twist->header.stamp};
    const double time_diff = std::fabs((pose_stamp - twist_stamp).seconds());
    if (time_diff > max_time_diff_sec_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Pose/Twist timestamp difference %.3f sec exceeds threshold %.3f sec - skipping odometry "
        "publish",
        time_diff, max_time_diff_sec_);
      return;
    }

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = pose->header.stamp;
    odom_msg.header.frame_id = frame_id_;
    odom_msg.child_frame_id = child_frame_id_;
    odom_msg.pose = pose->pose;
    odom_msg.twist = twist->twist;

    odom_pub_->publish(odom_msg);

    if (publish_tf_ && tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header = odom_msg.header;
      tf_msg.child_frame_id = child_frame_id_;
      tf_msg.transform.translation.x = pose->pose.pose.position.x;
      tf_msg.transform.translation.y = pose->pose.pose.position.y;
      tf_msg.transform.translation.z = pose->pose.pose.position.z;
      tf_msg.transform.rotation = pose->pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf_msg);
    }
  }

  std::string frame_id_;
  std::string child_frame_id_;
  bool publish_tf_{true};
  double max_time_diff_sec_{0.2};

  rclcpp::Subscription<PoseMsg>::SharedPtr pose_sub_;
  rclcpp::Subscription<TwistMsg>::SharedPtr twist_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::mutex mutex_;
  PoseMsg::ConstSharedPtr latest_pose_{nullptr};
  TwistMsg::ConstSharedPtr latest_twist_{nullptr};
};

}  // namespace autoware::carla_interface

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::carla_interface::CarlaStatePublisher)
