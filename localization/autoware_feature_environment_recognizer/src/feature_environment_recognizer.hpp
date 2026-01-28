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

#ifndef FEATURE_ENVIRONMENT_RECOGNIZER_HPP_
#define FEATURE_ENVIRONMENT_RECOGNIZER_HPP_

#include <autoware/lanelet2_utils/nn_search.hpp>
#include <autoware_feature_environment_recognizer/msg/feature_environment.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/header.hpp>

#include <boost/geometry/geometry.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <map>
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
  int32_t classify_environment(const geometry_msgs::msg::Point & point) const;
  void publish_environment(const int32_t environment_id, const std_msgs::msg::Header & header);

  // Parameters
  struct Param
  {
    // Area subtype to environment ID mapping
    // Environment ID definitions:
    //   -1: Invalid (area not found or map not ready)
    //   0: Normal environment (default)
    //   1: Uniform road (e.g., tunnel straight sections, uniform shape roads)
    //   2: Feature-poor road (roads with few features for map matching)
    // Key: area subtype (string), Value: environment_id (int32)
    std::unordered_map<std::string, int32_t> area_subtype_to_environment_id;
    // Default environment ID when area is not found in any list (0: Normal environment)
    int32_t default_environment_id{0};
  } param_;

  // State
  lanelet::LaneletMapPtr lanelet_map_ptr_;
  std::mutex mutex_;
  bool is_map_ready_{false};

  // Area data structure for point-in-polygon check
  using BoostPoint = boost::geometry::model::d2::point_xy<double>;
  using BoostPolygon = boost::geometry::model::polygon<BoostPoint>;
  std::multimap<std::string, BoostPolygon> area_polygons_;

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

#endif  // FEATURE_ENVIRONMENT_RECOGNIZER_HPP_
