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

#include "autoware/feature_environment_recognizer/feature_environment_recognizer.hpp"

#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <algorithm>
#include <string>
#include <vector>

namespace autoware::feature_environment_recognizer
{

FeatureEnvironmentRecognizer::FeatureEnvironmentRecognizer(const rclcpp::NodeOptions & options)
: Node("feature_environment_recognizer", options)
{
  // Declare parameters
  this->declare_parameter<int32_t>("default_environment_id", 0);
  this->declare_parameter<double>("search_distance_threshold", 10.0);
  this->declare_parameter<double>("search_yaw_threshold", M_PI / 4.0);

  // Load parameters
  param_.default_environment_id = this->get_parameter("default_environment_id").as_int();
  param_.search_distance_threshold = this->get_parameter("search_distance_threshold").as_double();
  param_.search_yaw_threshold = this->get_parameter("search_yaw_threshold").as_double();

  // Load environment-lanelet ID mappings from parameters
  // Environment ID definitions:
  //   -1: Invalid (lanelet not found or map not ready)
  //   0: Normal environment (default)
  //   1: Uniform road (e.g., tunnel straight sections, uniform shape roads)
  //   2: Feature-poor road (roads with few features for map matching)
  // Format: environment_id_<number>.lanelet_ids = [id1, id2, id3, ...]
  const auto param_names = this->list_parameters({}, 0);
  for (const auto & param_name : param_names.names) {
    if (param_name.find("environment_id_") == 0) {
      // Extract environment ID from parameter name (e.g., "environment_id_1" -> 1)
      const size_t prefix_len = std::string("environment_id_").length();
      const std::string id_str = param_name.substr(prefix_len);
      const size_t dot_pos = id_str.find('.');
      if (dot_pos != std::string::npos) {
        const std::string env_id_str = id_str.substr(0, dot_pos);
        try {
          const int32_t env_id = std::stoi(env_id_str);
          const std::string lanelet_ids_param_name = param_name;
          if (this->has_parameter(lanelet_ids_param_name)) {
            const auto lanelet_ids_param = this->get_parameter(lanelet_ids_param_name);
            if (lanelet_ids_param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
              const auto lanelet_ids = lanelet_ids_param.as_integer_array();
              std::set<lanelet::Id> lanelet_id_set;
              for (const auto & id : lanelet_ids) {
                lanelet_id_set.insert(static_cast<lanelet::Id>(id));
              }
              param_.environment_lanelet_ids[env_id] = lanelet_id_set;
              RCLCPP_INFO(
                this->get_logger(), "Loaded environment_id %d with %zu lanelet IDs", env_id,
                lanelet_id_set.size());
            }
          }
        } catch (const std::exception & e) {
          RCLCPP_WARN(
            this->get_logger(), "Failed to parse environment ID from parameter: %s", param_name.c_str());
        }
      }
    }
  }

  // Create subscribers
  sub_map_ = this->create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
    "~/input/lanelet2_map", rclcpp::QoS(10).durability(rclcpp::DurabilityPolicy::TransientLocal),
    std::bind(&FeatureEnvironmentRecognizer::on_map, this, std::placeholders::_1));

  sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "~/input/pose", rclcpp::QoS(10),
    std::bind(&FeatureEnvironmentRecognizer::on_pose, this, std::placeholders::_1));

  // Create publishers
  pub_environment_ = this->create_publisher<autoware_feature_environment_recognizer::msg::FeatureEnvironment>(
    "~/output/environment", 10);

  // Set parameter callback
  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&FeatureEnvironmentRecognizer::on_set_param, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "FeatureEnvironmentRecognizer initialized");
}

void FeatureEnvironmentRecognizer::on_map(
  const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*msg, lanelet_map_ptr_);
  is_map_ready_ = true;

  RCLCPP_INFO(
    this->get_logger(), "Received lanelet map with %zu lanelets",
    lanelet_map_ptr_->laneletLayer.size());
}

void FeatureEnvironmentRecognizer::on_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
  if (!is_map_ready_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000, "Lanelet map not ready yet");
    // Publish invalid environment ID (-1) when map is not ready
    publish_environment(-1, msg->header);
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  const auto current_lanelet = get_current_lanelet(msg->pose.pose);
  if (!current_lanelet.has_value()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Could not find lanelet for current pose");
    // Publish invalid environment ID (-1) when lanelet is not found
    publish_environment(-1, msg->header);
    return;
  }

  const int32_t environment_id = classify_environment(current_lanelet.value().id());
  publish_environment(environment_id, msg->header);
}

std::optional<lanelet::ConstLanelet> FeatureEnvironmentRecognizer::get_current_lanelet(
  const geometry_msgs::msg::Pose & pose) const
{
  if (!lanelet_map_ptr_) {
    return std::nullopt;
  }

  // First, try to get road lanelets at the position
  const auto road_lanelets = autoware::experimental::lanelet2_utils::get_road_lanelets_at(
    lanelet_map_ptr_, pose.position.x, pose.position.y);

  if (!road_lanelets.empty()) {
    // Use get_closest_lanelet to select the best match based on yaw
    const auto closest_lanelet =
      autoware::experimental::lanelet2_utils::get_closest_lanelet(road_lanelets, pose);
    if (closest_lanelet.has_value()) {
      return closest_lanelet;
    }
  }

  // If no road lanelet found, try with constraints
  // Get all lanelets in the map
  lanelet::ConstLanelets all_lanelets;
  for (const auto & lanelet : lanelet_map_ptr_->laneletLayer) {
    all_lanelets.push_back(lanelet);
  }

  const auto closest_lanelet_with_constraint =
    autoware::experimental::lanelet2_utils::get_closest_lanelet_within_constraint(
      all_lanelets, pose, param_.search_distance_threshold, param_.search_yaw_threshold);

  return closest_lanelet_with_constraint;
}

int32_t FeatureEnvironmentRecognizer::classify_environment(const lanelet::Id lanelet_id) const
{
  // Check if the lanelet ID is in any of the environment lists
  for (const auto & [env_id, lanelet_ids] : param_.environment_lanelet_ids) {
    if (lanelet_ids.find(lanelet_id) != lanelet_ids.end()) {
      return env_id;
    }
  }

  // If not found in any list, return default environment ID
  return param_.default_environment_id;
}

void FeatureEnvironmentRecognizer::publish_environment(
  const int32_t environment_id, const std_msgs::msg::Header & header)
{
  autoware_feature_environment_recognizer::msg::FeatureEnvironment msg;
  msg.header = header;
  msg.environment_id = environment_id;
  msg.confidence = 1.0;  // Currently not used, set to 1.0 as default
  pub_environment_->publish(msg);
}

rcl_interfaces::msg::SetParametersResult FeatureEnvironmentRecognizer::on_set_param(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  std::lock_guard<std::mutex> lock(mutex_);

  for (const auto & param : parameters) {
    if (param.get_name() == "default_environment_id") {
      param_.default_environment_id = param.as_int();
      RCLCPP_INFO(
        this->get_logger(), "Updated default_environment_id to %d", param_.default_environment_id);
    } else if (param.get_name() == "search_distance_threshold") {
      param_.search_distance_threshold = param.as_double();
      RCLCPP_INFO(
        this->get_logger(), "Updated search_distance_threshold to %f",
        param_.search_distance_threshold);
    } else if (param.get_name() == "search_yaw_threshold") {
      param_.search_yaw_threshold = param.as_double();
      RCLCPP_INFO(
        this->get_logger(), "Updated search_yaw_threshold to %f", param_.search_yaw_threshold);
    }
  }

  return result;
}

}  // namespace autoware::feature_environment_recognizer

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::feature_environment_recognizer::FeatureEnvironmentRecognizer)
