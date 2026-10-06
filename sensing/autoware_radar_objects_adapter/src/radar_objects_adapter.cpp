// Copyright 2026 The Autoware Contributors
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

#include <autoware_utils_geometry/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <functional>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace autoware::radar_objects_adapter
{

// Maps for classification remapping
const std::map<std::string, RadarClassification::_label_type> RADAR_LABEL_TO_UINT_MAP = {
  {"UNKNOWN", RadarClassification::UNKNOWN}, {"CAR", RadarClassification::CAR},
  {"TRUCK", RadarClassification::TRUCK},     {"MOTORCYCLE", RadarClassification::MOTORCYCLE},
  {"BICYCLE", RadarClassification::BICYCLE}, {"PEDESTRIAN", RadarClassification::PEDESTRIAN},
  {"ANIMAL", RadarClassification::ANIMAL},   {"HAZARD", RadarClassification::HAZARD}};
const std::map<std::string, ObjectClassification::_label_type> OBJECT_LABEL_TO_UINT_MAP = {
  {"UNKNOWN", ObjectClassification::UNKNOWN}, {"CAR", ObjectClassification::CAR},
  {"TRUCK", ObjectClassification::TRUCK},     {"BUS", ObjectClassification::BUS},
  {"TRAILER", ObjectClassification::TRAILER}, {"MOTORCYCLE", ObjectClassification::MOTORCYCLE},
  {"BICYCLE", ObjectClassification::BICYCLE}, {"PEDESTRIAN", ObjectClassification::PEDESTRIAN},
  {"ANIMAL", ObjectClassification::ANIMAL}};

ClassificationRemapParseResult make_classification_remap_from_string_pair(
  const std::map<std::string, std::string> & classification_remap_str)
{
  ClassificationRemap classification_remap;
  std::map<std::string, std::string> unknown_perception_labels;

  for (const auto & [radar_label, perception_label] : classification_remap_str) {
    // Radar string → uint8
    auto radar_id = RADAR_LABEL_TO_UINT_MAP.at(radar_label);

    // Perception string → uint8
    auto perception_id = ObjectClassification::UNKNOWN;
    auto it = OBJECT_LABEL_TO_UINT_MAP.find(perception_label);
    if (it != OBJECT_LABEL_TO_UINT_MAP.end()) {
      perception_id = it->second;
    } else {
      unknown_perception_labels[radar_label] = perception_label;
    }

    classification_remap[radar_id] = perception_id;
  }

  return {classification_remap, unknown_perception_labels};
}

namespace
{

float mask_cov_value(double value)
{
  return static_cast<float>(
    value == autoware_sensing_msgs::msg::RadarObject::INVALID_COV_VALUE ? 0.0 : value);
}

std::string join(const std::vector<std::string> & words)
{
  std::string joined;
  for (const auto & word : words) {
    joined += (joined.empty() ? "" : ", ") + word;
  }
  return joined;
}

}  // namespace

void ConversionConfiguration::add_field(std::string name, float default_value)
{
  default_fields_.push_back({std::move(name), default_value});
}

const std::vector<std::pair<std::string, float>> & ConversionConfiguration::defaulted_fields() const
{
  return default_fields_;
}

InvalidRadarInfo::InvalidRadarInfo(const std::vector<std::string> & missing_required_fields)
: std::runtime_error(
    "Radar info message is not valid. Some required attributes are missing (" +
    join(missing_required_fields) + "). This radar may not be compatible with autoware"),
  missing_required_fields_(missing_required_fields)
{
}

MissingRadarInfo::MissingRadarInfo()
: std::runtime_error(
    "A Valid radar info message has not been received. Cannot convert radar objects.")
{
}

RadarObjectsAdapter::RadarObjectsAdapter(
  const RadarObjectsAdapterParams & params, const ClassificationRemap & classification_remap,
  const std::string & topic_name)
: params_(params), classification_remap_(classification_remap)
{
  required_attributes_ = {
    "existence_probability", "position_x",     "position_y", "velocity_x", "velocity_y",
    "acceleration_x",        "acceleration_y", "orientation"};

  std::size_t hash_code = std::hash<std::string>{}(topic_name);

  for (std::size_t i = 0; i < sizeof(std::size_t); ++i) {
    topic_hash_code_[i] = static_cast<std::uint8_t>((hash_code >> (i * 8)) & 0xFF);
  }
}

tl::expected<ConversionConfiguration, InvalidRadarInfo> RadarObjectsAdapter::update_radar_info(
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
    std::vector<std::string> missing_required_fields;
    for (const auto & attribute : required_attributes_) {
      if (field_info_map_.find(attribute) == field_info_map_.end()) {
        missing_required_fields.push_back(attribute);
      }
    }
    return tl::make_unexpected(InvalidRadarInfo(missing_required_fields));
  }

  position_z_available_ = field_info_map_.count("position_z") > 0;
  velocity_z_available_ = field_info_map_.count("velocity_z") > 0;
  acceleration_z_available_ = field_info_map_.count("acceleration_z") > 0;
  size_x_available_ = field_info_map_.count("size_x") > 0;
  size_y_available_ = field_info_map_.count("size_y") > 0;
  size_z_available_ = field_info_map_.count("size_z") > 0;

  orientation_std_available_ = field_info_map_.count("orientation_std") > 0;
  orientation_rate_std_available_ = field_info_map_.count("orientation_rate_std") > 0;

  // The fields filled from a parameter, in the order the node has always warned about them.
  // orientation_std and orientation_rate_std have no parameter: without them the yaw variances
  // stay zero, and that has never been warned about.
  ConversionConfiguration config;
  if (!position_z_available_) {
    config.add_field("position_z", params_.default_position_z);
  }
  if (!velocity_z_available_) {
    config.add_field("velocity_z", params_.default_velocity_z);
  }
  if (!acceleration_z_available_) {
    config.add_field("acceleration_z", params_.default_acceleration_z);
  }
  if (!size_x_available_) {
    config.add_field("size_x", params_.default_size_x);
  }
  if (!size_y_available_) {
    config.add_field("size_y", params_.default_size_y);
  }
  if (!size_z_available_) {
    config.add_field("size_z", params_.default_size_z);
  }
  return config;
}

tl::expected<ConversionOutcome, MissingRadarInfo> RadarObjectsAdapter::convert(
  const autoware_sensing_msgs::msg::RadarObjects & input_msg) const
{
  if (!valid_radar_info_) {
    return tl::make_unexpected(MissingRadarInfo());
  }

  // publish both detections and tracks
  auto detected_objects = this->to_detected_objects(input_msg);
  auto tracked_objects = this->to_tracked_objects(input_msg);

  return ConversionOutcome{detected_objects, tracked_objects};
}

void RadarObjectsAdapter::radar_cov_to_detection_pose_cov(
  const std::array<float, 6> & radar_pose_cov, const double orientation_std,
  std::array<double, 36> & pose_cov) const
{
  using DETECTION_COV_IDX = autoware_utils_geometry::xyzrpy_covariance_index::XYZRPY_COV_IDX;
  using RADAR_COV_IDX = autoware_utils_geometry::xyz_upper_covariance_index::XYZ_UPPER_COV_IDX;

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
  std::array<double, 36> & twist_cov) const
{
  using DETECTION_COV_IDX = autoware_utils_geometry::xyzrpy_covariance_index::XYZRPY_COV_IDX;
  using RADAR_COV_IDX = autoware_utils_geometry::xyz_upper_covariance_index::XYZ_UPPER_COV_IDX;

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
  using DETECTION_COV_IDX = autoware_utils_geometry::xyzrpy_covariance_index::XYZRPY_COV_IDX;
  using RADAR_COV_IDX = autoware_utils_geometry::xyz_upper_covariance_index::XYZ_UPPER_COV_IDX;

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

template <typename ObjectType>
void RadarObjectsAdapter::populate_common_fields(
  const autoware_sensing_msgs::msg::RadarObject & input_object, ObjectType & output_object,
  const float yaw) const
{
  auto default_size_x = params_.default_size_x;
  auto default_size_y = params_.default_size_y;
  auto default_size_z = params_.default_size_z;
  auto default_position_z = params_.default_position_z;
  auto default_velocity_z = params_.default_velocity_z;
  auto default_acceleration_z = params_.default_acceleration_z;

  output_object.existence_probability = input_object.existence_probability;

  auto & output_shape = output_object.shape;
  output_shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  output_shape.dimensions.x = size_x_available_ ? input_object.size.x : default_size_x;
  output_shape.dimensions.y = size_y_available_ ? input_object.size.y : default_size_y;
  output_shape.dimensions.z = size_z_available_ ? input_object.size.z : default_size_z;

  auto & output_pose = output_object.kinematics.pose_with_covariance.pose;
  output_pose.position.x = input_object.position.x;
  output_pose.position.y = input_object.position.y;
  output_pose.position.z = position_z_available_ ? input_object.position.z : default_position_z;
  output_pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(yaw);

  radar_cov_to_detection_pose_cov(
    input_object.position_covariance, input_object.orientation_std,
    output_object.kinematics.pose_with_covariance.covariance);

  auto & output_twist = output_object.kinematics.twist_with_covariance.twist;
  output_twist.linear.x =
    std::cos(yaw) * input_object.velocity.x + std::sin(yaw) * input_object.velocity.y;
  output_twist.linear.y =
    -std::sin(yaw) * input_object.velocity.x + std::cos(yaw) * input_object.velocity.y;
  output_twist.linear.z = velocity_z_available_ ? input_object.velocity.z : default_velocity_z;
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
      acceleration_z_available_ ? input_object.acceleration.z : default_acceleration_z;

    radar_cov_to_detection_acceleration_cov(
      input_object.acceleration_covariance, yaw,
      output_object.kinematics.acceleration_with_covariance.covariance);
  }
}

void RadarObjectsAdapter::populate_classifications(
  const std::vector<autoware_sensing_msgs::msg::RadarClassification> & input_classifications,
  std::vector<autoware_perception_msgs::msg::ObjectClassification> & output_classifications) const
{
  for (const auto & input_classification : input_classifications) {
    // class remap based on policy defined in parameter
    if (classification_remap_.count(input_classification.label)) {
      ObjectClassification output_classification;
      output_classification.label = classification_remap_.at(input_classification.label);
      output_classification.probability = input_classification.probability;
      output_classifications.push_back(output_classification);
    } else {
      // if no remap rule matched, set UNKNOWN
      ObjectClassification output_classification;
      output_classification.label = ObjectClassification::UNKNOWN;
      output_classification.probability = input_classification.probability;
      output_classifications.push_back(output_classification);
    }
  }
}

autoware_perception_msgs::msg::DetectedObjects RadarObjectsAdapter::to_detected_objects(
  const autoware_sensing_msgs::msg::RadarObjects & input_msg) const
{
  autoware_perception_msgs::msg::DetectedObjects output_msg;

  output_msg.header = input_msg.header;
  output_msg.objects.reserve(input_msg.objects.size());

  for (const auto & input_object : input_msg.objects) {
    autoware_perception_msgs::msg::DetectedObject output_object;

    // Populate common fields
    const auto & yaw = input_object.orientation;
    populate_common_fields(input_object, output_object, yaw);

    // Set flags for kinematics
    output_object.kinematics.has_position_covariance = true;
    output_object.kinematics.orientation_availability =
      autoware_perception_msgs::msg::DetectedObjectKinematics::AVAILABLE;
    output_object.kinematics.has_twist = true;
    output_object.kinematics.has_twist_covariance = true;

    // Set classification
    populate_classifications(input_object.classifications, output_object.classification);

    output_msg.objects.push_back(output_object);
  }

  return output_msg;
}

autoware_perception_msgs::msg::TrackedObjects RadarObjectsAdapter::to_tracked_objects(
  const autoware_sensing_msgs::msg::RadarObjects & input_msg) const
{
  autoware_perception_msgs::msg::TrackedObjects output_msg;

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

    // Populate common fields
    const auto & yaw = input_object.orientation;
    populate_common_fields(input_object, output_object, yaw);

    // Set flags for kinematics
    output_object.kinematics.orientation_availability =
      autoware_perception_msgs::msg::TrackedObjectKinematics::AVAILABLE;
    output_object.kinematics.is_stationary =
      input_object.movement_status !=
      autoware_sensing_msgs::msg::RadarObject::MOVEMENT_STATUS_DYNAMIC;

    // Populate classification
    populate_classifications(input_object.classifications, output_object.classification);

    output_msg.objects.push_back(output_object);
  }

  return output_msg;
}

}  // namespace autoware::radar_objects_adapter
