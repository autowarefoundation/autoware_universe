// Copyright 2025 The Autoware Contributors
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

#include "radar_objects_adapter.hpp"

#include <autoware/universe_utils/geometry/geometry.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware
{

float mask_cov_value(double value)
{
  return static_cast<float>(
    value == autoware_sensing_msgs::msg::RadarObject::INVALID_COV_VALUE ? 0.0 : value);
}

RadarObjectsAdapter::RadarObjectsAdapter(const rclcpp::NodeOptions & options)
: Node("radar_objects_adapter", options)
{
  radar_objects_sub_ = this->create_subscription<autoware_sensing_msgs::msg::RadarObjects>(
    "~/input/objects", rclcpp::SensorDataQoS(),
    std::bind(&RadarObjectsAdapter::objects_callback, this, std::placeholders::_1));

  radar_info_sub_ = this->create_subscription<autoware_sensing_msgs::msg::RadarInfo>(
    "~/input/radar_info", rclcpp::SensorDataQoS(),
    std::bind(&RadarObjectsAdapter::radar_info_callback, this, std::placeholders::_1));

  detections_pub_ = create_publisher<autoware_perception_msgs::msg::DetectedObjects>(
    "~/output/detections", rclcpp::QoS(10).reliable().transient_local());

  tracks_pub_ = create_publisher<autoware_perception_msgs::msg::TrackedObjects>(
    "~/output/tracks", rclcpp::QoS(10).reliable().transient_local());

  default_position_z_ = this->declare_parameter<float>("default_position_z");
  default_velocity_z_ = this->declare_parameter<float>("default_velocity_z");
  default_acceleration_z_ = this->declare_parameter<float>("default_acceleration_z");

  default_size_x_ = this->declare_parameter<float>("default_size_x");
  default_size_y_ = this->declare_parameter<float>("default_size_y");
  default_size_z_ = this->declare_parameter<float>("default_size_z");

  required_attributes_ = {
    "existence_probability", "position_x",     "position_y", "velocity_x", "velocity_y",
    "acceleration_x",        "acceleration_y", "orientation"};

  std::size_t hash_code = std::hash<std::string>{}(radar_objects_sub_->get_topic_name());

  for (std::size_t i = 0; i < sizeof(std::size_t); ++i) {
    topic_hash_code_[i] = static_cast<std::uint8_t>((hash_code >> (i * 8)) & 0xFF);
  }
}

void RadarObjectsAdapter::radar_cov_to_detection_pose_cov(
  const std::array<float, 6> & radar_pose_cov, const double orientation_std,
  std::array<double, 36> & pose_cov)
{
  using DETECTION_COV_IDX = autoware::universe_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;
  using RADAR_COV_IDX = autoware::universe_utils::xyz_upper_covariance_index::XYZ_UPPER_COV_IDX;

  pose_cov[DETECTION_COV_IDX::X_X] = mask_cov_value(radar_pose_cov[RADAR_COV_IDX::X_X]);
  pose_cov[DETECTION_COV_IDX::X_Y] = mask_cov_value(radar_pose_cov[RADAR_COV_IDX::X_Y]);
  pose_cov[DETECTION_COV_IDX::Y_X] = mask_cov_value(radar_pose_cov[RADAR_COV_IDX::X_Y]);
  pose_cov[DETECTION_COV_IDX::Y_Y] = mask_cov_value(radar_pose_cov[RADAR_COV_IDX::Y_Y]);

  if (orientation_std_available_) {
    pose_cov[DETECTION_COV_IDX::YAW_YAW] = static_cast<double>(orientation_std * orientation_std);
  }
}

void RadarObjectsAdapter::radar_cov_to_detection_twist_cov(
  const std::array<float, 6> & radar_twist_cov, const float yaw, const float yaw_rate_std,
  std::array<double, 36> & twist_cov)
{
  using DETECTION_COV_IDX = autoware::universe_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;
  using RADAR_COV_IDX = autoware::universe_utils::xyz_upper_covariance_index::XYZ_UPPER_COV_IDX;

  const float c = std::cos(yaw);
  const float s = std::sin(yaw);

  const float xx = mask_cov_value(radar_twist_cov[RADAR_COV_IDX::X_X]);
  const float xy = mask_cov_value(radar_twist_cov[RADAR_COV_IDX::X_Y]);
  const float yy = mask_cov_value(radar_twist_cov[RADAR_COV_IDX::Y_Y]);

  twist_cov[DETECTION_COV_IDX::X_X] =
    static_cast<double>(xx * c * c + yy * s * s + 2.f * xy * s * c);

  twist_cov[DETECTION_COV_IDX::X_Y] = static_cast<double>((yy - xx) * s * c + xy * (c * c - s * s));
  twist_cov[DETECTION_COV_IDX::Y_X] = twist_cov[DETECTION_COV_IDX::X_Y];

  twist_cov[DETECTION_COV_IDX::Y_Y] =
    static_cast<double>(xx * s * s + yy * c * c - 2.f * xy * s * c);

  twist_cov[DETECTION_COV_IDX::Y_Z] = 0.0;

  if (orientation_rate_std_available_) {
    twist_cov[DETECTION_COV_IDX::YAW_YAW] = static_cast<double>(yaw_rate_std * yaw_rate_std);
  }
}

void RadarObjectsAdapter::radar_cov_to_detection_acceleration_cov(
  const std::array<float, 6> & radar_acceleration_cov, const float yaw,
  std::array<double, 36> & acceleration_cov)
{
  using DETECTION_COV_IDX = autoware::universe_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;
  using RADAR_COV_IDX = autoware::universe_utils::xyz_upper_covariance_index::XYZ_UPPER_COV_IDX;

  const float c = std::cos(yaw);
  const float s = std::sin(yaw);

  const float xx = mask_cov_value(radar_acceleration_cov[RADAR_COV_IDX::X_X]);
  const float xy = mask_cov_value(radar_acceleration_cov[RADAR_COV_IDX::X_Y]);
  const float yy = mask_cov_value(radar_acceleration_cov[RADAR_COV_IDX::Y_Y]);

  acceleration_cov[DETECTION_COV_IDX::X_X] =
    static_cast<double>(xx * c * c + yy * s * s + 2.f * xy * s * c);

  acceleration_cov[DETECTION_COV_IDX::X_Y] =
    static_cast<double>((yy - xx) * s * c + xy * (c * c - s * s));
  acceleration_cov[DETECTION_COV_IDX::Y_X] = acceleration_cov[DETECTION_COV_IDX::X_Y];

  acceleration_cov[DETECTION_COV_IDX::Y_Y] =
    static_cast<double>(xx * s * s + yy * c * c - 2.f * xy * s * c);

  acceleration_cov[DETECTION_COV_IDX::Y_Z] = 0.0;
}

void RadarObjectsAdapter::objects_callback(
  const autoware_sensing_msgs::msg::RadarObjects & objects_msg)
{
  if (!valid_radar_info_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 10000,
      "A Valid radar info message has not been received. Cannot convert radar objects.");
    return;
  }

  // publish both detections and tracks
  this->parse_as_detections(objects_msg);
  this->parse_as_tracks(objects_msg);
}

template <typename ObjectType>
void RadarObjectsAdapter::populate_common_fields(
  const autoware_sensing_msgs::msg::RadarObject & input_object, ObjectType & output_object,
  const float yaw)
{
  output_object.existence_probability = input_object.existence_probability;

  auto & output_shape = output_object.shape;
  output_shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  output_shape.dimensions.x = size_x_available_ ? input_object.size.x : default_size_x_;
  output_shape.dimensions.y = size_y_available_ ? input_object.size.y : default_size_y_;
  output_shape.dimensions.z = size_z_available_ ? input_object.size.z : default_size_z_;

  auto & output_pose = output_object.kinematics.pose_with_covariance.pose;
  output_pose.position.x = input_object.position.x;
  output_pose.position.y = input_object.position.y;
  output_pose.position.z = position_z_available_ ? input_object.position.z : default_position_z_;
  output_pose.orientation = autoware::universe_utils::createQuaternionFromYaw(yaw);

  radar_cov_to_detection_pose_cov(
    input_object.position_covariance, input_object.orientation_std,
    output_object.kinematics.pose_with_covariance.covariance);

  auto & output_twist = output_object.kinematics.twist_with_covariance.twist;
  output_twist.linear.x =
    std::cos(yaw) * input_object.velocity.x + std::sin(yaw) * input_object.velocity.y;
  output_twist.linear.y =
    -std::sin(yaw) * input_object.velocity.x + std::cos(yaw) * input_object.velocity.y;
  output_twist.linear.z = velocity_z_available_ ? input_object.velocity.z : default_velocity_z_;
  output_twist.angular.z = input_object.orientation_rate;

  radar_cov_to_detection_twist_cov(
    input_object.velocity_covariance, yaw, input_object.orientation_rate_std,
    output_object.kinematics.twist_with_covariance.covariance);

  // Additional fields for TrackedObject
  if constexpr (std::is_same_v<ObjectType, autoware_perception_msgs::msg::TrackedObject>) {
    auto & output_acceleration = output_object.kinematics.acceleration_with_covariance.accel;
    output_acceleration.linear.x =
      std::cos(yaw) * input_object.acceleration.x + std::sin(yaw) * input_object.acceleration.y;
    output_acceleration.linear.y =
      -std::sin(yaw) * input_object.acceleration.x + std::cos(yaw) * input_object.acceleration.y;
    output_acceleration.linear.z =
      acceleration_z_available_ ? input_object.acceleration.z : default_acceleration_z_;

    radar_cov_to_detection_acceleration_cov(
      input_object.acceleration_covariance, yaw,
      output_object.kinematics.acceleration_with_covariance.covariance);
  }
}

void RadarObjectsAdapter::populate_classifications(
  const std::vector<autoware_sensing_msgs::msg::RadarClassification> & input_classifications,
  std::vector<autoware_perception_msgs::msg::ObjectClassification> & output_classifications)
{
  using RadarClassification = autoware_sensing_msgs::msg::RadarClassification;
  using ObjectClassification = autoware_perception_msgs::msg::ObjectClassification;

  for (const auto & input_classification : input_classifications) {
    ObjectClassification output_classification;

    switch (input_classification.label) {
      case RadarClassification::UNKNOWN:
        output_classification.label = ObjectClassification::UNKNOWN;
        break;
      case RadarClassification::CAR:
        output_classification.label = ObjectClassification::CAR;
        break;
      case RadarClassification::TRUCK:
        output_classification.label = ObjectClassification::TRUCK;
        break;
      case RadarClassification::MOTORCYCLE:
        output_classification.label = ObjectClassification::MOTORCYCLE;
        break;
      case RadarClassification::BICYCLE:
        output_classification.label = ObjectClassification::BICYCLE;
        break;
      case RadarClassification::PEDESTRIAN:
        output_classification.label = ObjectClassification::PEDESTRIAN;
        break;
      case RadarClassification::ANIMAL:
        output_classification.label = ObjectClassification::ANIMAL;
        break;
      case RadarClassification::HAZARD:
        output_classification.label = ObjectClassification::UNKNOWN;
        break;
      default:
        continue;
    }

    output_classification.probability = input_classification.probability;
    output_classifications.push_back(output_classification);
  }
}

void RadarObjectsAdapter::parse_as_detections(
  const autoware_sensing_msgs::msg::RadarObjects & input_msg)
{
  auto output_msg_ptr = std::make_unique<autoware_perception_msgs::msg::DetectedObjects>();
  auto & output_msg = *output_msg_ptr;

  output_msg.header = input_msg.header;
  output_msg.objects.reserve(input_msg.objects.size());

  for (const auto & input_object : input_msg.objects) {
    autoware_perception_msgs::msg::DetectedObject output_object;

    const auto & yaw = input_object.orientation;
    populate_common_fields(input_object, output_object, yaw);
    populate_classifications(input_object.classifications, output_object.classification);

    output_msg.objects.push_back(output_object);
  }

  detections_pub_->publish(std::move(output_msg_ptr));
}

void RadarObjectsAdapter::parse_as_tracks(
  const autoware_sensing_msgs::msg::RadarObjects & input_msg)
{
  auto output_msg_ptr = std::make_unique<autoware_perception_msgs::msg::TrackedObjects>();
  auto & output_msg = *output_msg_ptr;

  output_msg.header = input_msg.header;
  output_msg.objects.reserve(input_msg.objects.size());

  for (const auto & input_object : input_msg.objects) {
    autoware_perception_msgs::msg::TrackedObject output_object;

    output_object.object_id.uuid[0] = static_cast<uint8_t>((input_object.object_id >> 0) & 0xFF);
    output_object.object_id.uuid[1] = static_cast<uint8_t>((input_object.object_id >> 8) & 0xFF);
    output_object.object_id.uuid[2] = static_cast<uint8_t>((input_object.object_id >> 16) & 0xFF);
    output_object.object_id.uuid[3] = static_cast<uint8_t>((input_object.object_id >> 24) & 0xFF);

    for (std::size_t i = 4; i < output_object.object_id.uuid.size(); ++i) {
      if (i - 4 < topic_hash_code_.size()) {
        output_object.object_id.uuid[i] = topic_hash_code_[i - 4];
      } else {
        output_object.object_id.uuid[i] = 0;
      }
    }

    const auto & yaw = input_object.orientation;
    populate_common_fields(input_object, output_object, yaw);
    populate_classifications(input_object.classifications, output_object.classification);

    output_msg.objects.push_back(output_object);
  }

  tracks_pub_->publish(std::move(output_msg_ptr));
}

void RadarObjectsAdapter::radar_info_callback(
  const autoware_sensing_msgs::msg::RadarInfo & radar_info_msg)
{
  for (const auto & field_info : radar_info_msg.object_fields_info) {
    field_info_map_[field_info.field_name.data] = field_info;
  }

  valid_radar_info_ = std::all_of(
    required_attributes_.begin(), required_attributes_.end(),
    [this](const std::string & attribute) {
      return field_info_map_.find(attribute) != field_info_map_.end();
    });

  if (!valid_radar_info_) {
    RCLCPP_ERROR_ONCE(
      get_logger(),
      "Radar info message is not valid. Some required attributes are missing. This radar may not "
      "be compatible with autoware");

    for (const auto & attribute : required_attributes_) {
      if (field_info_map_.find(attribute) == field_info_map_.end()) {
        RCLCPP_ERROR_ONCE(get_logger(), "\tMissing attribute: %s", attribute.c_str());
      }
    }
    return;
  }

  position_z_available_ = field_info_map_.count("position_z") > 0;
  velocity_z_available_ = field_info_map_.count("velocity_z") > 0;
  size_x_available_ = field_info_map_.count("size_x") > 0;
  size_y_available_ = field_info_map_.count("size_y") > 0;
  size_z_available_ = field_info_map_.count("size_z") > 0;

  orientation_std_available_ = field_info_map_.count("orientation_std") > 0;
  orientation_rate_available_ = field_info_map_.count("orientation_rate") > 0;
  orientation_rate_std_available_ = field_info_map_.count("orientation_rate_std") > 0;

  if (!position_z_available_) {
    RCLCPP_WARN_ONCE(
      get_logger(),
      "The field position_z is not available in the radar info message. Defaulting to %f.",
      default_position_z_);
  }

  if (!velocity_z_available_) {
    RCLCPP_WARN_ONCE(
      get_logger(),
      "The field velocity_z is not available in the radar info message. Defaulting to %f.",
      default_velocity_z_);
  }

  if (!size_x_available_) {
    RCLCPP_WARN_ONCE(
      get_logger(),
      "The field shape_x is not available in the radar info message. Defaulting to %f.",
      default_size_x_);
  }

  if (!size_y_available_) {
    RCLCPP_WARN_ONCE(
      get_logger(),
      "The field shape_y is not available in the radar info message. Defaulting to %f.",
      default_size_y_);
  }

  if (!size_z_available_) {
    RCLCPP_WARN_ONCE(
      get_logger(),
      "The field shape_z is not available in the radar info message. Defaulting to %f.",
      default_size_z_);
  }
}

}  // namespace autoware

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::RadarObjectsAdapter)
