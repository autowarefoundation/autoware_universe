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

#include "covariance_selector.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>

#include <boost/geometry/geometry.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <memory>
#include <string>

namespace autoware::covariance_selector
{

CovarianceSelector::CovarianceSelector(const rclcpp::NodeOptions & options)
: Node(
    "covariance_selector",
    rclcpp::NodeOptions(options).automatically_declare_parameters_from_overrides(true))
{
  // --- environment classification parameters (same format as feature_environment_recognizer) ---
  if (!this->has_parameter("default_environment_id")) {
    this->declare_parameter<int32_t>("default_environment_id", 0);
  }
  param_.default_environment_id = this->get_parameter("default_environment_id").as_int();

  const auto param_names = this->list_parameters({}, 0);
  for (const auto & name : param_names.names) {
    if (name.find("area_subtype_") == 0) {
      const std::string rest = name.substr(std::string("area_subtype_").length());
      const size_t dot = rest.find('.');
      if (dot != std::string::npos &&
          this->get_parameter(name).get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        const std::string subtype = rest.substr(0, dot);
        const int32_t env_id = this->get_parameter(name).as_int();
        param_.area_subtype_to_environment_id[subtype] = env_id;
        RCLCPP_INFO(
          this->get_logger(), "area_subtype '%s' -> environment_id %d", subtype.c_str(), env_id);
      }
    }
  }

  // --- per-environment covariance parameters ---
  // Format: environment_<id>_output_pose_covariance (36-element double array)
  if (!this->has_parameter("default_output_pose_covariance")) {
    this->declare_parameter<std::vector<double>>(
      "default_output_pose_covariance", std::vector<double>{});
  }
  const auto default_cov_vec =
    this->get_parameter("default_output_pose_covariance").as_double_array();
  if (default_cov_vec.size() == 36) {
    for (size_t i = 0; i < 36; ++i) param_.default_covariance[i] = default_cov_vec[i];
  } else {
    RCLCPP_WARN(
      this->get_logger(),
      "default_output_pose_covariance has %zu elements (expected 36); using zero matrix",
      default_cov_vec.size());
  }

  for (const auto & name : param_names.names) {
    const std::string prefix = "environment_";
    const std::string suffix = "_output_pose_covariance";
    if (name.find(prefix) == 0 && name.size() > prefix.size() + suffix.size()) {
      const std::string middle = name.substr(prefix.size(), name.size() - prefix.size() - suffix.size());
      if (name.substr(name.size() - suffix.size()) == suffix) {
        try {
          const int32_t env_id = std::stoi(middle);
          const auto cov_vec = this->get_parameter(name).as_double_array();
          if (cov_vec.size() == 36) {
            std::array<double, 36> arr{};
            for (size_t i = 0; i < 36; ++i) arr[i] = cov_vec[i];
            param_.environment_covariance_map[env_id] = arr;
            RCLCPP_INFO(
              this->get_logger(), "Loaded covariance for environment_id %d", env_id);
          } else {
            RCLCPP_WARN(
              this->get_logger(),
              "Parameter '%s' has %zu elements (expected 36); skipping", name.c_str(),
              cov_vec.size());
          }
        } catch (const std::exception &) {
          // not an integer id in the name, skip
        }
      }
    }
  }

  // --- subscriptions & publications ---
  sub_map_ = this->create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
    "~/input/lanelet2_map", rclcpp::QoS(10).durability(rclcpp::DurabilityPolicy::TransientLocal),
    std::bind(&CovarianceSelector::on_map, this, std::placeholders::_1));

  sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "~/input/pose_with_covariance", rclcpp::QoS(10),
    std::bind(&CovarianceSelector::on_pose, this, std::placeholders::_1));

  pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "~/output/pose_with_covariance", 10);

  pub_env_id_ = this->create_publisher<autoware_internal_debug_msgs::msg::Int32Stamped>(
    "~/debug/environment_id", 10);

  RCLCPP_INFO(this->get_logger(), "CovarianceSelector initialized");
}

void CovarianceSelector::on_map(
  const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  lanelet_map_ptr_ = autoware::experimental::lanelet2_utils::remove_const(
    autoware::experimental::lanelet2_utils::from_autoware_map_msgs(*msg));
  is_map_ready_ = true;

  area_polygons_.clear();
  constexpr auto kAreaType = "feature_environment_specify";
  for (const auto & polygon : lanelet_map_ptr_->polygonLayer) {
    if (std::string{polygon.attributeOr(lanelet::AttributeName::Type, "none")} != kAreaType) {
      continue;
    }
    const std::string subtype{polygon.attributeOr(lanelet::AttributeName::Subtype, "none")};

    BoostPolygon boost_poly;
    for (const lanelet::ConstPoint3d & p : polygon) {
      boost_poly.outer().push_back(BoostPoint(p.x(), p.y()));
    }
    if (!boost_poly.outer().empty()) {
      boost_poly.outer().push_back(boost_poly.outer().front());
    }
    boost::geometry::correct(boost_poly);
    area_polygons_.emplace(subtype, boost_poly);
  }

  RCLCPP_INFO(
    this->get_logger(), "Map loaded: %zu lanelets, %zu covariance-selector areas",
    lanelet_map_ptr_->laneletLayer.size(), area_polygons_.size());
}

void CovarianceSelector::on_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
  int32_t env_id = param_.default_environment_id;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_map_ready_) {
      env_id = classify_environment(msg->pose.pose.position);
    } else {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000, "Lanelet map not ready yet");
    }
  }

  const auto it = param_.environment_covariance_map.find(env_id);
  const auto & cov =
    (it != param_.environment_covariance_map.end()) ? it->second : param_.default_covariance;

  auto out = *msg;
  for (size_t i = 0; i < 36; ++i) out.pose.covariance[i] = cov[i];
  pub_pose_->publish(out);

  autoware_internal_debug_msgs::msg::Int32Stamped env_msg;
  env_msg.stamp = msg->header.stamp;
  env_msg.data = env_id;
  pub_env_id_->publish(env_msg);
}

int32_t CovarianceSelector::classify_environment(
  const geometry_msgs::msg::Point & point) const
{
  const BoostPoint bp(point.x, point.y);
  for (const auto & [subtype, polygon] : area_polygons_) {
    if (boost::geometry::within(bp, polygon)) {
      const auto it = param_.area_subtype_to_environment_id.find(subtype);
      if (it != param_.area_subtype_to_environment_id.end()) {
        return it->second;
      }
    }
  }
  return param_.default_environment_id;
}

}  // namespace autoware::covariance_selector

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::covariance_selector::CovarianceSelector)
