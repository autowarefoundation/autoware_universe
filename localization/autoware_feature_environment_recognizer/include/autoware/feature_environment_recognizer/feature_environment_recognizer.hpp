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

#ifndef AUTOWARE__FEATURE_ENVIRONMENT_RECOGNIZER__FEATURE_ENVIRONMENT_RECOGNIZER_HPP_
#define AUTOWARE__FEATURE_ENVIRONMENT_RECOGNIZER__FEATURE_ENVIRONMENT_RECOGNIZER_HPP_

#include <autoware/lanelet2_utils/nn_search.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_feature_environment_recognizer/msg/feature_environment.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::feature_environment_recognizer
{

class FeatureEnvironmentRecognizer : public rclcpp::Node
{
public:
  explicit FeatureEnvironmentRecognizer(const rclcpp::NodeOptions & options);

private:
  // Callbacks
  void on_map(const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg);
  void on_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);

  // Core functions
  std::optional<lanelet::ConstLanelet> get_current_lanelet(
    const geometry_msgs::msg::Pose & pose) const;
  int32_t classify_environment(const lanelet::Id lanelet_id) const;
  void publish_environment(
    const int32_t environment_id, const std_msgs::msg::Header & header);

  // Parameters
  struct Param
  {
    // Lanelet ID lists for each environment type
    // Environment ID definitions:
    //   -1: Invalid (lanelet not found or map not ready)
    //   0: Normal environment (default)
    //   1: Uniform road (e.g., tunnel straight sections, uniform shape roads)
    //   2: Feature-poor road (roads with few features for map matching)
    // Key: environment_id (int32), Value: set of lanelet IDs
    std::unordered_map<int32_t, std::set<lanelet::Id>> environment_lanelet_ids;
    // Default environment ID when lanelet is not found in any list (0: Normal environment)
    int32_t default_environment_id{0};
    // Distance threshold for lanelet search
    double search_distance_threshold{10.0};
    // Yaw threshold for lanelet search (rad)
    double search_yaw_threshold{M_PI / 4.0};
  } param_;

  // State
  lanelet::LaneletMapPtr lanelet_map_ptr_;
  std::mutex mutex_;
  bool is_map_ready_{false};

  // Subscribers
  rclcpp::Subscription<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr sub_map_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_pose_;

  // Publishers
  rclcpp::Publisher<autoware_feature_environment_recognizer::msg::FeatureEnvironment>::SharedPtr
    pub_environment_;

  // Parameter callback
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  rcl_interfaces::msg::SetParametersResult on_set_param(
    const std::vector<rclcpp::Parameter> & parameters);
};

}  // namespace autoware::feature_environment_recognizer

#endif  // AUTOWARE__FEATURE_ENVIRONMENT_RECOGNIZER__FEATURE_ENVIRONMENT_RECOGNIZER_HPP_
