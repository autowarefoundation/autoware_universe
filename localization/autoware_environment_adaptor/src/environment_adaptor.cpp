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

#include "environment_adaptor.hpp"

#include "covariance_utils.hpp"

#include <Eigen/Geometry>

#include <memory>
#include <string>

namespace autoware::environment_adaptor
{

EnvironmentAdaptor::EnvironmentAdaptor(const rclcpp::NodeOptions & options)
: Node(
    "environment_adaptor", rclcpp::NodeOptions(options)
                             .allow_undeclared_parameters(true)
                             .automatically_declare_parameters_from_overrides(true))
{
  load_classifier_params();
  load_covariance_params();

  sub_map_ = this->create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
    "~/input/lanelet2_map", rclcpp::QoS(10).durability(rclcpp::DurabilityPolicy::TransientLocal),
    std::bind(&EnvironmentAdaptor::on_map, this, std::placeholders::_1));

  sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "~/input/pose_with_covariance", rclcpp::QoS(10),
    std::bind(&EnvironmentAdaptor::on_pose, this, std::placeholders::_1));

  sub_twist_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "~/input/twist_with_covariance", rclcpp::QoS(10),
    std::bind(&EnvironmentAdaptor::on_twist, this, std::placeholders::_1));

  pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "~/output/pose_with_covariance", 10);

  pub_twist_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "~/output/twist_with_covariance", 10);

  pub_env_id_ = this->create_publisher<autoware_internal_debug_msgs::msg::Int32Stamped>(
    "~/debug/environment_id", 10);

  pub_longitudinal_scale_factor_ =
    this->create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "~/debug/longitudinal_scale_factor", 10);

  RCLCPP_INFO(this->get_logger(), "EnvironmentAdaptor initialized");
}

void EnvironmentAdaptor::load_classifier_params()
{
  EnvironmentClassifier::Param param;

  if (!this->has_parameter("default_environment_id")) {
    this->declare_parameter<int32_t>("default_environment_id", 0);
  }
  param.default_environment_id = this->get_parameter("default_environment_id").as_int();

  if (!this->has_parameter("default_longitudinal_scale_factor")) {
    this->declare_parameter<double>("default_longitudinal_scale_factor", 1.0);
  }
  param.default_longitudinal_scale_factor =
    this->get_parameter("default_longitudinal_scale_factor").as_double();

  if (!this->has_parameter("map_longitudinal_scale_factor_attribute")) {
    this->declare_parameter<std::string>(
      "map_longitudinal_scale_factor_attribute", "longitudinal_scale_factor");
  }
  param.map_longitudinal_scale_factor_attribute =
    this->get_parameter("map_longitudinal_scale_factor_attribute").as_string();

  const auto param_names = this->list_parameters({}, 0);
  for (const auto & name : param_names.names) {
    if (name.find("area_subtype_") == 0) {
      const std::string rest = name.substr(std::string("area_subtype_").length());
      const size_t dot = rest.find('.');
      if (
        dot != std::string::npos &&
        this->get_parameter(name).get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        const std::string subtype = rest.substr(0, dot);
        const int32_t env_id = this->get_parameter(name).as_int();
        param.area_subtype_to_environment_id[subtype] = env_id;
        RCLCPP_INFO(
          this->get_logger(), "area_subtype '%s' -> environment_id %d", subtype.c_str(), env_id);
      }
    }
  }

  classifier_.set_param(param);
}

void EnvironmentAdaptor::load_covariance_params()
{
  const auto param_names = this->list_parameters({}, 0);
  const std::string prefix = "environment_";
  const std::string suffix = "_output_pose_covariance";
  for (const auto & name : param_names.names) {
    if (name.find(prefix) == 0 && name.size() > prefix.size() + suffix.size()) {
      const std::string middle =
        name.substr(prefix.size(), name.size() - prefix.size() - suffix.size());
      if (name.substr(name.size() - suffix.size()) == suffix) {
        try {
          const int32_t env_id = std::stoi(middle);
          std::array<double, 36> arr{};
          if (try_read_covariance_param(name, arr)) {
            covariance_param_.environment_covariance_map[env_id] = arr;
            RCLCPP_INFO(this->get_logger(), "Loaded covariance for environment_id %d", env_id);
          }
        } catch (const std::exception &) {
          // not an integer id in the name, skip
        }
      }
    }
  }
}

bool EnvironmentAdaptor::try_read_covariance_param(
  const std::string & name, std::array<double, 36> & dest)
{
  if (!this->has_parameter(name)) {
    return false;
  }

  const auto param = this->get_parameter(name);
  if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
    RCLCPP_WARN(
      this->get_logger(), "Parameter '%s' has unexpected type; expected double array",
      name.c_str());
    return false;
  }

  const auto cov_vec = param.as_double_array();
  if (cov_vec.size() != 36) {
    RCLCPP_WARN(
      this->get_logger(), "Parameter '%s' has %zu elements (expected 36)", name.c_str(),
      cov_vec.size());
    return false;
  }

  for (size_t i = 0; i < 36; ++i) {
    dest[i] = cov_vec[i];
  }
  return true;
}

std::optional<std::array<double, 36>> EnvironmentAdaptor::get_body_covariance_for_env_id(
  int32_t env_id) const
{
  const auto it = covariance_param_.environment_covariance_map.find(env_id);
  if (it != covariance_param_.environment_covariance_map.end()) {
    return it->second;
  }
  return std::nullopt;
}

void EnvironmentAdaptor::on_map(const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  classifier_.load_map(*msg);
  RCLCPP_INFO(this->get_logger(), "Lanelet map loaded for environment classification");
}

void EnvironmentAdaptor::on_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
  EnvironmentClassifier::AreaClassification classification;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_pose_position_ = msg->pose.pose.position;
    pose_received_ = true;

    if (!classifier_.is_map_ready()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000, "Lanelet map not ready yet");
      classification.environment_id = classifier_.classify(latest_pose_position_).environment_id;
    } else {
      classification = classifier_.classify(latest_pose_position_);
    }
  }

  auto out = *msg;
  const auto body_cov = get_body_covariance_for_env_id(classification.environment_id);
  if (body_cov) {
    const auto & q_msg = msg->pose.pose.orientation;
    const Eigen::Quaterniond q(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
    const auto map_cov = rotate_covariance(body_cov.value(), q.normalized().toRotationMatrix());
    for (size_t i = 0; i < 36; ++i) out.pose.covariance[i] = map_cov[i];
  }
  pub_pose_->publish(out);

  autoware_internal_debug_msgs::msg::Int32Stamped env_msg;
  env_msg.stamp = msg->header.stamp;
  env_msg.data = classification.environment_id;
  pub_env_id_->publish(env_msg);
}

void EnvironmentAdaptor::on_twist(
  const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr msg)
{
  EnvironmentClassifier::AreaClassification classification;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!pose_received_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Pose not received yet; using default longitudinal_scale_factor");
      classification = classifier_.classify(latest_pose_position_);
    } else if (!classifier_.is_map_ready()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000, "Lanelet map not ready yet");
      classification = classifier_.classify(latest_pose_position_);
    } else {
      classification = classifier_.classify(latest_pose_position_);
    }
  }

  auto out = *msg;
  out.twist.twist.linear.x *= classification.longitudinal_scale_factor;
  pub_twist_->publish(out);

  autoware_internal_debug_msgs::msg::Float64Stamped factor_msg;
  factor_msg.stamp = msg->header.stamp;
  factor_msg.data = classification.longitudinal_scale_factor;
  pub_longitudinal_scale_factor_->publish(factor_msg);
}

}  // namespace autoware::environment_adaptor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::environment_adaptor::EnvironmentAdaptor)
