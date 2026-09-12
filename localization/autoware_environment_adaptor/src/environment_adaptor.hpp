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

#ifndef ENVIRONMENT_ADAPTOR_HPP_
#define ENVIRONMENT_ADAPTOR_HPP_

#include "environment_classifier.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <autoware_internal_debug_msgs/msg/int32_stamped.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

#include <array>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>

namespace autoware::environment_adaptor
{

enum class ClassificationWarn { None, MapNotReady, PoseNotReceived };

class EnvironmentAdaptor : public rclcpp::Node
{
public:
  explicit EnvironmentAdaptor(const rclcpp::NodeOptions & options);

private:
  void on_map(const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg);
  void on_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);
  void on_twist(const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr msg);

  EnvironmentClassifier::AreaClassification classify_at_latest_pose(
    ClassificationWarn * warn_out = nullptr);

  void load_classifier_params();
  void load_covariance_params();
  bool try_read_covariance_param(const std::string & name, std::array<double, 36> & dest);
  std::optional<std::array<double, 36>> get_body_covariance_for_env_id(int32_t env_id) const;

  struct CovarianceParam
  {
    std::unordered_map<int32_t, std::array<double, 36>> environment_covariance_map;
  } covariance_param_;

  EnvironmentClassifier classifier_;
  std::mutex mutex_;
  bool pose_received_{false};
  geometry_msgs::msg::Point latest_pose_position_;

  rclcpp::Subscription<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr sub_map_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr sub_twist_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_twist_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Int32Stamped>::SharedPtr pub_env_id_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float64Stamped>::SharedPtr
    pub_longitudinal_scale_factor_;
};

}  // namespace autoware::environment_adaptor

#endif  // ENVIRONMENT_ADAPTOR_HPP_
