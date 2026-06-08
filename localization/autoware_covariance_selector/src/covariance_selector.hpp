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

#ifndef COVARIANCE_SELECTOR_HPP_
#define COVARIANCE_SELECTOR_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/int32_stamped.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <Eigen/Core>
#include <boost/geometry/geometry.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <array>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

namespace autoware::covariance_selector
{

class CovarianceSelector : public rclcpp::Node
{
public:
  explicit CovarianceSelector(const rclcpp::NodeOptions & options);

private:
  void on_map(const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg);
  void on_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);

  int32_t classify_environment(const geometry_msgs::msg::Point & point) const;

  static std::array<double, 36> rotate_covariance(
    const std::array<double, 36> & src, const Eigen::Matrix3d & rotation);

  struct Param
  {
    std::unordered_map<std::string, int32_t> area_subtype_to_environment_id;
    int32_t default_environment_id{0};
    // environment_id → 36-element covariance array
    std::unordered_map<int32_t, std::array<double, 36>> environment_covariance_map;
    std::array<double, 36> default_covariance{};
  } param_;

  lanelet::LaneletMapPtr lanelet_map_ptr_;
  std::mutex mutex_;
  bool is_map_ready_{false};

  using BoostPoint = boost::geometry::model::d2::point_xy<double>;
  using BoostPolygon = boost::geometry::model::polygon<BoostPoint>;
  std::multimap<std::string, BoostPolygon> area_polygons_;

  rclcpp::Subscription<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr sub_map_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Int32Stamped>::SharedPtr pub_env_id_;
};

}  // namespace autoware::covariance_selector

#endif  // COVARIANCE_SELECTOR_HPP_
