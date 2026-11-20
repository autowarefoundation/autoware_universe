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

#include <autoware/object_recognition_utils/conversion.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <cmath>
#include <functional>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace autoware::radar_objects_adapter
{

// Maps for classification remapping
const std::map<std::string, RadarClassification::_label_type> RADAR_LABEL_TO_UINT_MAP = {
  {"UNKNOWN", RadarClassification::UNKNOWN},
  {"CAR", RadarClassification::CAR},
  {"TRUCK", RadarClassification::TRUCK},
  {"BUS", RadarClassification::BUS},
  {"TRAILER", RadarClassification::TRAILER},
  {"MOTORCYCLE", RadarClassification::MOTORCYCLE},
  {"BICYCLE", RadarClassification::BICYCLE},
  {"PEDESTRIAN", RadarClassification::PEDESTRIAN},
  {"ANIMAL", RadarClassification::ANIMAL},
  {"HAZARD", RadarClassification::HAZARD},
  {"OVER_DRIVABLE", RadarClassification::OVER_DRIVABLE},
  {"UNDER_DRIVABLE", RadarClassification::UNDER_DRIVABLE}};
const std::map<std::string, ObjectClassification::_label_type> OBJECT_LABEL_TO_UINT_MAP = {
  {"UNKNOWN", ObjectClassification::UNKNOWN},
  {"CAR", ObjectClassification::CAR},
  {"TRUCK", ObjectClassification::TRUCK},
  {"BUS", ObjectClassification::BUS},
  {"TRAILER", ObjectClassification::TRAILER},
  {"MOTORCYCLE", ObjectClassification::MOTORCYCLE},
  {"BICYCLE", ObjectClassification::BICYCLE},
  {"PEDESTRIAN", ObjectClassification::PEDESTRIAN},
  {"ANIMAL", ObjectClassification::ANIMAL},
  {"HAZARD", ObjectClassification::HAZARD},
  {"OVER_DRIVABLE", ObjectClassification::OVER_DRIVABLE},
  {"UNDER_DRIVABLE", ObjectClassification::UNDER_DRIVABLE}};

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

ObjectUUIDGenerator::ObjectUUIDGenerator(const std::string & topic_name)
{
  const std::size_t hash_code = std::hash<std::string>{}(topic_name);
  for (std::size_t i = 0; i < sizeof(std::size_t); ++i) {
    topic_hash_code_[i] = static_cast<std::uint8_t>((hash_code >> (i * 8)) & 0xFF);
  }
}

unique_identifier_msgs::msg::UUID::_uuid_type ObjectUUIDGenerator::operator()(
  const std::uint32_t object_id) const
{
  unique_identifier_msgs::msg::UUID::_uuid_type uuid;

  uuid[0] = static_cast<uint8_t>((object_id >> 0) & 0xFF);
  uuid[1] = static_cast<uint8_t>((object_id >> 8) & 0xFF);
  uuid[2] = static_cast<uint8_t>((object_id >> 16) & 0xFF);
  uuid[3] = static_cast<uint8_t>((object_id >> 24) & 0xFF);

  for (std::size_t i = 4; i < uuid.size(); ++i) {
    if (i - 4 < topic_hash_code_.size()) {
      uuid[i] = topic_hash_code_[i - 4];
    } else {
      uuid[i] = 0;
    }
  }

  return uuid;
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

using DETECTION_COV_IDX = autoware_utils_geometry::xyzrpy_covariance_index::XYZRPY_COV_IDX;
using RADAR_COV_IDX = autoware_utils_geometry::xyz_upper_covariance_index::XYZ_UPPER_COV_IDX;

double squared(const double value)
{
  return value * value;
}

// A 6x6 covariance whose x/y block is copied from the radar's upper triangle as it is. Every
// other entry is zero.
std::array<double, 36> copied_xy_covariance(const std::array<float, 6> & radar_cov)
{
  std::array<double, 36> cov{};

  cov[DETECTION_COV_IDX::X_X] = mask_cov_value(radar_cov[RADAR_COV_IDX::X_X]);
  cov[DETECTION_COV_IDX::X_Y] = mask_cov_value(radar_cov[RADAR_COV_IDX::X_Y]);
  cov[DETECTION_COV_IDX::Y_X] = mask_cov_value(radar_cov[RADAR_COV_IDX::X_Y]);
  cov[DETECTION_COV_IDX::Y_Y] = mask_cov_value(radar_cov[RADAR_COV_IDX::Y_Y]);

  return cov;
}

// A 6x6 covariance whose x/y block is the radar's, rotated into the frame of an object facing
// `yaw`. Every other entry is zero. Serves the twist and the acceleration alike.
std::array<double, 36> rotated_xy_covariance(
  const std::array<float, 6> & radar_cov, const float yaw)
{
  std::array<double, 36> cov{};

  const float c = std::cos(yaw);
  const float s = std::sin(yaw);

  const float xx = mask_cov_value(radar_cov[RADAR_COV_IDX::X_X]);
  const float xy = mask_cov_value(radar_cov[RADAR_COV_IDX::X_Y]);
  const float yy = mask_cov_value(radar_cov[RADAR_COV_IDX::Y_Y]);

  cov[DETECTION_COV_IDX::X_X] = xx * c * c + yy * s * s + 2.f * xy * s * c;

  cov[DETECTION_COV_IDX::X_Y] = (yy - xx) * s * c + xy * (c * c - s * s);
  cov[DETECTION_COV_IDX::Y_X] = cov[DETECTION_COV_IDX::X_Y];

  cov[DETECTION_COV_IDX::Y_Y] = xx * s * s + yy * c * c - 2.f * xy * s * c;

  return cov;
}

}  // namespace

std::vector<std::pair<std::string, double>> ConversionConfiguration::defaulted_fields() const
{
  std::vector<std::pair<std::string, double>> fields;
  const auto add = [&fields](const char * name, const std::optional<double> & value) {
    if (value.has_value()) {
      fields.emplace_back(name, *value);
    }
  };
  add("position_z", default_position_z);
  add("velocity_z", default_velocity_z);
  add("acceleration_z", default_acceleration_z);
  add("size_x", default_size_x);
  add("size_y", default_size_y);
  add("size_z", default_size_z);
  return fields;
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

ClassificationRemapper::ClassificationRemapper(const ClassificationRemap & table) : table_(table)
{
}

std::vector<autoware_perception_msgs::msg::ObjectClassification> ClassificationRemapper::operator()(
  const std::vector<autoware_sensing_msgs::msg::RadarClassification> & classifications) const
{
  std::vector<ObjectClassification> output;
  output.reserve(classifications.size());
  for (const auto & classification : classifications) {
    ObjectClassification remapped;
    // class remap based on policy defined in parameter; if no remap rule matched, set UNKNOWN
    const auto it = table_.find(classification.label);
    remapped.label = it != table_.end() ? it->second : ObjectClassification::UNKNOWN;
    remapped.probability = classification.probability;
    output.push_back(remapped);
  }
  return output;
}

const std::vector<std::string> & TrackedObjectConverter::required_fields()
{
  static const std::vector<std::string> fields = {"existence_probability",
                                                  "position_x",
                                                  "position_y",
                                                  "velocity_x",
                                                  "velocity_y",
                                                  "acceleration_x",
                                                  "acceleration_y",
                                                  "orientation",
                                                  "orientation_rate",
                                                  "movement_status"};
  return fields;
}

tl::expected<ConversionConfiguration, InvalidRadarInfo> TrackedObjectConverter::configuration_for(
  const autoware_sensing_msgs::msg::RadarInfo & radar_info,
  const ConversionConfiguration & defaults)
{
  std::set<std::string> declared_fields;
  for (const auto & field_info : radar_info.object_fields_info) {
    declared_fields.insert(field_info.field_name.data);
  }

  std::vector<std::string> missing_required_fields;
  for (const auto & field : required_fields()) {
    if (declared_fields.count(field) == 0) {
      missing_required_fields.push_back(field);
    }
  }
  if (!missing_required_fields.empty()) {
    return tl::make_unexpected(InvalidRadarInfo(missing_required_fields));
  }

  // The fields the radar provides are copied from the object; the rest keep the parameter in
  // their place. orientation_std and orientation_rate_std have no parameter: without them the
  // yaw variances stay zero, and that has never been warned about.
  ConversionConfiguration config = defaults;
  const auto provided = [&declared_fields](const char * field) {
    return declared_fields.count(field) > 0;
  };
  if (provided("position_z")) {
    config.default_position_z.reset();
  }
  if (provided("velocity_z")) {
    config.default_velocity_z.reset();
  }
  if (provided("acceleration_z")) {
    config.default_acceleration_z.reset();
  }
  if (provided("size_x")) {
    config.default_size_x.reset();
  }
  if (provided("size_y")) {
    config.default_size_y.reset();
  }
  if (provided("size_z")) {
    config.default_size_z.reset();
  }
  config.orientation_std_provided = provided("orientation_std");
  config.orientation_rate_std_provided = provided("orientation_rate_std");

  return config;
}

TrackedObjectConverter::TrackedObjectConverter(
  const ConversionConfiguration & config, GenerateObjectUUID generate_uuid)
: config_(config), generate_uuid_(std::move(generate_uuid))
{
}

autoware_perception_msgs::msg::TrackedObject TrackedObjectConverter::operator()(
  const autoware_sensing_msgs::msg::RadarObject & input_object) const
{
  autoware_perception_msgs::msg::TrackedObject output_object;

  output_object.object_id.set__uuid(generate_uuid_(input_object.object_id));

  output_object.existence_probability = input_object.existence_probability;

  auto & output_shape = output_object.shape;
  output_shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  output_shape.dimensions.x = config_.default_size_x.value_or(input_object.size.x);
  output_shape.dimensions.y = config_.default_size_y.value_or(input_object.size.y);
  output_shape.dimensions.z = config_.default_size_z.value_or(input_object.size.z);

  const float yaw = input_object.orientation;

  auto & output_pose = output_object.kinematics.pose_with_covariance.pose;
  output_pose.position.x = input_object.position.x;
  output_pose.position.y = input_object.position.y;
  output_pose.position.z = config_.default_position_z.value_or(input_object.position.z);
  output_pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(yaw);

  auto & pose_cov = output_object.kinematics.pose_with_covariance.covariance;
  pose_cov = copied_xy_covariance(input_object.position_covariance);
  if (config_.orientation_std_provided) {
    pose_cov[DETECTION_COV_IDX::YAW_YAW] = squared(input_object.orientation_std);
  }

  auto & output_twist = output_object.kinematics.twist_with_covariance.twist;
  output_twist.linear.x =
    std::cos(yaw) * input_object.velocity.x + std::sin(yaw) * input_object.velocity.y;
  output_twist.linear.y =
    -std::sin(yaw) * input_object.velocity.x + std::cos(yaw) * input_object.velocity.y;
  output_twist.linear.z = config_.default_velocity_z.value_or(input_object.velocity.z);
  output_twist.angular.z = input_object.orientation_rate;

  auto & twist_cov = output_object.kinematics.twist_with_covariance.covariance;
  twist_cov = rotated_xy_covariance(input_object.velocity_covariance, yaw);
  if (config_.orientation_rate_std_provided) {
    twist_cov[DETECTION_COV_IDX::YAW_YAW] = squared(input_object.orientation_rate_std);
  }

  auto & output_acceleration = output_object.kinematics.acceleration_with_covariance.accel;
  output_acceleration.linear.x =
    std::cos(yaw) * input_object.acceleration.x + std::sin(yaw) * input_object.acceleration.y;
  output_acceleration.linear.y =
    -std::sin(yaw) * input_object.acceleration.x + std::cos(yaw) * input_object.acceleration.y;
  output_acceleration.linear.z =
    config_.default_acceleration_z.value_or(input_object.acceleration.z);

  output_object.kinematics.acceleration_with_covariance.covariance =
    rotated_xy_covariance(input_object.acceleration_covariance, yaw);

  // Set flags for kinematics
  output_object.kinematics.orientation_availability =
    autoware_perception_msgs::msg::TrackedObjectKinematics::AVAILABLE;
  output_object.kinematics.is_stationary =
    input_object.movement_status !=
    autoware_sensing_msgs::msg::RadarObject::MOVEMENT_STATUS_DYNAMIC;

  return output_object;
}

ConversionConfiguration RadarObjectsAdapterParams::to_config() const
{
  ConversionConfiguration config;
  config.default_position_z = default_position_z;
  config.default_velocity_z = default_velocity_z;
  config.default_acceleration_z = default_acceleration_z;
  config.default_size_x = default_size_x;
  config.default_size_y = default_size_y;
  config.default_size_z = default_size_z;
  return config;
}

RadarObjectsAdapter::RadarObjectsAdapter(
  const RadarObjectsAdapterParams & params, const ClassificationRemap & classification_remap,
  const std::string & topic_name)
: config_(params.to_config()),
  generate_uuid_(ObjectUUIDGenerator(topic_name)),
  classification_remapper_(classification_remap)
{
}

tl::expected<ConversionConfiguration, InvalidRadarInfo> RadarObjectsAdapter::update_radar_info(
  const autoware_sensing_msgs::msg::RadarInfo & radar_info_msg)
{
  // The answer is fixed by the first valid radar info: a later message is not looked at, and
  // comes back with the configuration already fixed.
  if (tracked_object_converter_.has_value()) {
    return tracked_object_converter_->configuration();
  }

  const auto config = TrackedObjectConverter::configuration_for(radar_info_msg, config_);
  if (config.has_value()) {
    tracked_object_converter_.emplace(*config, generate_uuid_);
  }
  return config;
}

tl::expected<ConversionOutcome, MissingRadarInfo> RadarObjectsAdapter::convert(
  const autoware_sensing_msgs::msg::RadarObjects & input_msg) const
{
  if (!tracked_object_converter_.has_value()) {
    return tl::make_unexpected(MissingRadarInfo());
  }

  autoware_perception_msgs::msg::TrackedObjects tracks;
  tracks.header = input_msg.header;
  tracks.objects.reserve(input_msg.objects.size());

  autoware_perception_msgs::msg::DetectedObjects detections;
  detections.header = input_msg.header;
  detections.objects.reserve(input_msg.objects.size());

  // A track is a detection plus its identity, its acceleration and whether it stands still, so
  // the track is built first and the detection is what is left of it.
  for (const auto & input_object : input_msg.objects) {
    autoware_perception_msgs::msg::TrackedObject track = (*tracked_object_converter_)(input_object);
    track.classification = classification_remapper_(input_object.classifications);

    detections.objects.push_back(autoware::object_recognition_utils::toDetectedObject(track));
    tracks.objects.push_back(std::move(track));
  }

  return ConversionOutcome{std::move(detections), std::move(tracks)};
}

}  // namespace autoware::radar_objects_adapter
