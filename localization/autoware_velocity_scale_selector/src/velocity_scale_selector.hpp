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

#ifndef VELOCITY_SCALE_SELECTOR_HPP_
#define VELOCITY_SCALE_SELECTOR_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <autoware_internal_debug_msgs/msg/int32_stamped.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

#include <boost/geometry/geometry.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::velocity_scale_selector
{

class VelocityScaleSelector : public rclcpp::Node
{
public:
  explicit VelocityScaleSelector(const rclcpp::NodeOptions & options);

private:
  using BoostPoint = boost::geometry::model::d2::point_xy<double>;
  using BoostPolygon = boost::geometry::model::polygon<BoostPoint>;

  struct EnvironmentArea
  {
    BoostPolygon polygon;
    std::string subtype;
    std::optional<double> map_longitudinal_scale_factor;
  };

  struct AreaClassification
  {
    int32_t environment_id{0};
    double longitudinal_scale_factor{1.0};
  };

  void on_map(const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg);
  void on_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);
  void on_twist(const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr msg);

  AreaClassification classify_area(const geometry_msgs::msg::Point & point) const;
  double get_longitudinal_scale_factor_for_env_id(int32_t env_id) const;
  int32_t get_environment_id_for_subtype(const std::string & subtype) const;

  struct Param
  {
    std::unordered_map<std::string, int32_t> area_subtype_to_environment_id;
    int32_t default_environment_id{0};
    double default_longitudinal_scale_factor{1.0};
    std::string map_longitudinal_scale_factor_attribute{"longitudinal_scale_factor"};
    std::unordered_map<int32_t, double> environment_longitudinal_scale_factor_map;
  } param_;

  lanelet::LaneletMapPtr lanelet_map_ptr_;
  std::mutex mutex_;
  bool is_map_ready_{false};
  bool pose_received_{false};
  geometry_msgs::msg::Point latest_pose_position_;

  std::vector<EnvironmentArea> areas_;

  rclcpp::Subscription<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr sub_map_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr sub_twist_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_twist_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Int32Stamped>::SharedPtr pub_env_id_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float64Stamped>::SharedPtr
    pub_longitudinal_scale_factor_;
};

}  // namespace autoware::velocity_scale_selector

#endif  // VELOCITY_SCALE_SELECTOR_HPP_
