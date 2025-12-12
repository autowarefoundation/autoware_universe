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

#include "feature_environment_recognizer.hpp"

#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <boost/geometry/geometry.hpp>

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

  // Load parameters
  param_.default_environment_id = this->get_parameter("default_environment_id").as_int();

  // Load area subtype to environment ID mappings from parameters
  // Environment ID definitions:
  //   -1: Invalid (area not found or map not ready)
  //   0: Normal environment (default)
  //   1: Uniform road (e.g., tunnel straight sections, uniform shape roads)
  //   2: Feature-poor road (roads with few features for map matching)
  // Format: area_subtype_<subtype_name>.environment_id = <environment_id>
  const auto param_names = this->list_parameters({}, 0);
  for (const auto & param_name : param_names.names) {
    if (param_name.find("area_subtype_") == 0) {
      // Extract area subtype from parameter name (e.g., "area_subtype_uniform_road" -> "uniform_road")
      const size_t prefix_len = std::string("area_subtype_").length();
      const std::string subtype_str = param_name.substr(prefix_len);
      const size_t dot_pos = subtype_str.find('.');
      if (dot_pos != std::string::npos) {
        const std::string subtype_name = subtype_str.substr(0, dot_pos);
        const std::string env_id_param_name = param_name;
        if (this->has_parameter(env_id_param_name)) {
          const auto env_id_param = this->get_parameter(env_id_param_name);
          if (env_id_param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
            const int32_t env_id = env_id_param.as_int();
            param_.area_subtype_to_environment_id[subtype_name] = env_id;
            RCLCPP_INFO(
              this->get_logger(), "Loaded area subtype '%s' -> environment_id %d", subtype_name.c_str(),
              env_id);
          }
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
  pub_environment_ =
    this->create_publisher<autoware_feature_environment_recognizer::msg::FeatureEnvironment>(
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

  // Load areas from polygon layer
  area_polygons_.clear();
  const std::string feature_environment_area_type = "feature_environment_specify";
  const auto & polygon_layer = lanelet_map_ptr_->polygonLayer;
  RCLCPP_DEBUG_STREAM(this->get_logger(), "Polygon layer size: " << polygon_layer.size());

  for (const auto & polygon : polygon_layer) {
    const std::string type{polygon.attributeOr(lanelet::AttributeName::Type, "none")};
    RCLCPP_DEBUG_STREAM(this->get_logger(), "polygon type: " << type);
    if (feature_environment_area_type != type) {
      continue;
    }

    const std::string subtype{polygon.attributeOr(lanelet::AttributeName::Subtype, "none")};
    RCLCPP_DEBUG_STREAM(this->get_logger(), "polygon subtype: " << subtype);

    // Create boost polygon from lanelet polygon
    BoostPolygon boost_poly;
    for (const lanelet::ConstPoint3d & p : polygon) {
      boost_poly.outer().push_back(BoostPoint(p.x(), p.y()));
    }
    // Close the polygon
    if (!boost_poly.outer().empty()) {
      boost_poly.outer().push_back(boost_poly.outer().front());
    }

    area_polygons_.emplace(subtype, boost_poly);
    RCLCPP_INFO(
      this->get_logger(), "Loaded area with subtype '%s' (%zu vertices)", subtype.c_str(),
      boost_poly.outer().size());
  }

  RCLCPP_INFO(
    this->get_logger(), "Received lanelet map with %zu lanelets and %zu feature environment areas",
    lanelet_map_ptr_->laneletLayer.size(), area_polygons_.size());
}

void FeatureEnvironmentRecognizer::on_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
  if (!is_map_ready_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Lanelet map not ready yet");
    // Publish invalid environment ID (-1) when map is not ready
    publish_environment(-1, msg->header);
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  const int32_t environment_id = classify_environment(msg->pose.pose.position);
  publish_environment(environment_id, msg->header);
}

int32_t FeatureEnvironmentRecognizer::classify_environment(const geometry_msgs::msg::Point & point) const
{
  const BoostPoint boost_point(point.x, point.y);

  // Check if the point is within any area polygon
  for (const auto & [subtype, polygon] : area_polygons_) {
    if (boost::geometry::within(boost_point, polygon)) {
      // Point is within this area, check if we have a mapping for this subtype
      const auto it = param_.area_subtype_to_environment_id.find(subtype);
      if (it != param_.area_subtype_to_environment_id.end()) {
        RCLCPP_DEBUG(
          this->get_logger(), "Point is within area subtype '%s' -> environment_id %d",
          subtype.c_str(), it->second);
        return it->second;
      } else {
        RCLCPP_DEBUG(
          this->get_logger(), "Point is within area subtype '%s' but no environment_id mapping found",
          subtype.c_str());
      }
    }
  }

  // If not found in any area, return default environment ID
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
    }
  }

  return result;
}

}  // namespace autoware::feature_environment_recognizer

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::feature_environment_recognizer::FeatureEnvironmentRecognizer)
