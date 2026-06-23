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

#include "velocity_scale_selector.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>

#include <boost/geometry/geometry.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <memory>
#include <string>

namespace autoware::velocity_scale_selector
{

VelocityScaleSelector::VelocityScaleSelector(const rclcpp::NodeOptions & options)
: Node(
    "velocity_scale_selector",
    rclcpp::NodeOptions(options).automatically_declare_parameters_from_overrides(true))
{
  // --- environment classification parameters (same format as covariance_selector) ---
  if (!this->has_parameter("default_environment_id")) {
    this->declare_parameter<int32_t>("default_environment_id", 0);
  }
  param_.default_environment_id = this->get_parameter("default_environment_id").as_int();

  if (!this->has_parameter("default_longitudinal_scale_factor")) {
    this->declare_parameter<double>("default_longitudinal_scale_factor", 1.0);
  }
  param_.default_longitudinal_scale_factor =
    this->get_parameter("default_longitudinal_scale_factor").as_double();

  if (!this->has_parameter("map_longitudinal_scale_factor_attribute")) {
    this->declare_parameter<std::string>("map_longitudinal_scale_factor_attribute", "longitudinal_scale_factor");
  }
  param_.map_longitudinal_scale_factor_attribute =
    this->get_parameter("map_longitudinal_scale_factor_attribute").as_string();

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

  // --- per-environment longitudinal scale factor parameters (fallback when map attribute is absent) ---
  // Format: environment_<id>_longitudinal_scale_factor
  const std::string env_prefix = "environment_";
  const std::string env_suffix = "_longitudinal_scale_factor";
  for (const auto & name : param_names.names) {
    if (name.find(env_prefix) == 0 && name.size() > env_prefix.size() + env_suffix.size()) {
      const std::string middle =
        name.substr(env_prefix.size(), name.size() - env_prefix.size() - env_suffix.size());
      if (name.substr(name.size() - env_suffix.size()) == env_suffix) {
        try {
          const int32_t env_id = std::stoi(middle);
          const double factor = this->get_parameter(name).as_double();
          param_.environment_longitudinal_scale_factor_map[env_id] = factor;
          RCLCPP_INFO(
            this->get_logger(), "Loaded longitudinal_scale_factor for environment_id %d: %f",
            env_id, factor);
        } catch (const std::exception &) {
          // not an integer id in the name, skip
        }
      }
    }
  }

  // --- subscriptions & publications ---
  sub_map_ = this->create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
    "~/input/lanelet2_map", rclcpp::QoS(10).durability(rclcpp::DurabilityPolicy::TransientLocal),
    std::bind(&VelocityScaleSelector::on_map, this, std::placeholders::_1));

  sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "~/input/pose_with_covariance", rclcpp::QoS(10),
    std::bind(&VelocityScaleSelector::on_pose, this, std::placeholders::_1));

  sub_twist_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "~/input/twist_with_covariance", rclcpp::QoS(10),
    std::bind(&VelocityScaleSelector::on_twist, this, std::placeholders::_1));

  pub_twist_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "~/output/twist_with_covariance", 10);

  pub_env_id_ = this->create_publisher<autoware_internal_debug_msgs::msg::Int32Stamped>(
    "~/debug/environment_id", 10);

  pub_longitudinal_scale_factor_ =
    this->create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "~/debug/longitudinal_scale_factor", 10);

  RCLCPP_INFO(this->get_logger(), "VelocityScaleSelector initialized");
}

void VelocityScaleSelector::on_map(
  const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  lanelet_map_ptr_ = autoware::experimental::lanelet2_utils::remove_const(
    autoware::experimental::lanelet2_utils::from_autoware_map_msgs(*msg));
  is_map_ready_ = true;

  areas_.clear();
  constexpr auto kAreaType = "feature_environment_specify";
  size_t areas_with_map_factor = 0;
  for (const auto & polygon : lanelet_map_ptr_->polygonLayer) {
    if (std::string{polygon.attributeOr(lanelet::AttributeName::Type, "none")} != kAreaType) {
      continue;
    }

    EnvironmentArea area;
    area.subtype = polygon.attributeOr(lanelet::AttributeName::Subtype, "none");

    for (const lanelet::ConstPoint3d & p : polygon) {
      area.polygon.outer().push_back(BoostPoint(p.x(), p.y()));
    }
    if (!area.polygon.outer().empty()) {
      area.polygon.outer().push_back(area.polygon.outer().front());
    }
    boost::geometry::correct(area.polygon);

    const std::string factor_str =
      polygon.attributeOr(param_.map_longitudinal_scale_factor_attribute, "");
    if (!factor_str.empty()) {
      try {
        area.map_longitudinal_scale_factor = std::stod(factor_str);
        ++areas_with_map_factor;
        RCLCPP_INFO(
          this->get_logger(),
          "Area subtype '%s' has map %s = %f", area.subtype.c_str(),
          param_.map_longitudinal_scale_factor_attribute.c_str(),
          area.map_longitudinal_scale_factor.value());
      } catch (const std::exception &) {
        RCLCPP_WARN(
          this->get_logger(), "Invalid %s '%s' on area subtype '%s'; using parameter fallback",
          param_.map_longitudinal_scale_factor_attribute.c_str(), factor_str.c_str(),
          area.subtype.c_str());
      }
    }

    areas_.push_back(area);
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Map loaded: %zu lanelets, %zu velocity-scale-selector areas (%zu with map longitudinal_scale_factor)",
    lanelet_map_ptr_->laneletLayer.size(), areas_.size(), areas_with_map_factor);
}

void VelocityScaleSelector::on_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_pose_position_ = msg->pose.pose.position;
  pose_received_ = true;
}

void VelocityScaleSelector::on_twist(
  const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr msg)
{
  AreaClassification classification;
  classification.environment_id = param_.default_environment_id;
  classification.longitudinal_scale_factor = param_.default_longitudinal_scale_factor;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!pose_received_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Pose not received yet; using default longitudinal_scale_factor");
    } else if (!is_map_ready_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000, "Lanelet map not ready yet");
    } else {
      classification = classify_area(latest_pose_position_);
    }
  }

  auto out = *msg;
  out.twist.twist.linear.x *= classification.longitudinal_scale_factor;
  pub_twist_->publish(out);

  autoware_internal_debug_msgs::msg::Int32Stamped env_msg;
  env_msg.stamp = msg->header.stamp;
  env_msg.data = classification.environment_id;
  pub_env_id_->publish(env_msg);

  autoware_internal_debug_msgs::msg::Float64Stamped factor_msg;
  factor_msg.stamp = msg->header.stamp;
  factor_msg.data = classification.longitudinal_scale_factor;
  pub_longitudinal_scale_factor_->publish(factor_msg);
}

VelocityScaleSelector::AreaClassification VelocityScaleSelector::classify_area(
  const geometry_msgs::msg::Point & point) const
{
  const BoostPoint bp(point.x, point.y);
  for (const auto & area : areas_) {
    if (!boost::geometry::within(bp, area.polygon)) {
      continue;
    }

    AreaClassification result;
    result.environment_id = get_environment_id_for_subtype(area.subtype);
    if (area.map_longitudinal_scale_factor.has_value()) {
      result.longitudinal_scale_factor = area.map_longitudinal_scale_factor.value();
    } else {
      result.longitudinal_scale_factor =
        get_longitudinal_scale_factor_for_env_id(result.environment_id);
    }
    return result;
  }

  AreaClassification result;
  result.environment_id = param_.default_environment_id;
  result.longitudinal_scale_factor = param_.default_longitudinal_scale_factor;
  return result;
}

int32_t VelocityScaleSelector::get_environment_id_for_subtype(const std::string & subtype) const
{
  const auto it = param_.area_subtype_to_environment_id.find(subtype);
  if (it != param_.area_subtype_to_environment_id.end()) {
    return it->second;
  }
  return param_.default_environment_id;
}

double VelocityScaleSelector::get_longitudinal_scale_factor_for_env_id(int32_t env_id) const
{
  const auto it = param_.environment_longitudinal_scale_factor_map.find(env_id);
  if (it != param_.environment_longitudinal_scale_factor_map.end()) {
    return it->second;
  }
  return param_.default_longitudinal_scale_factor;
}

}  // namespace autoware::velocity_scale_selector

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::velocity_scale_selector::VelocityScaleSelector)
