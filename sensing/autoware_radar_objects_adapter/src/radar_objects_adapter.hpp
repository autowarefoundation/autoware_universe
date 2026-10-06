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

#ifndef RADAR_OBJECTS_ADAPTER_HPP_
#define RADAR_OBJECTS_ADAPTER_HPP_

#include <tl/expected.hpp>

#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_sensing_msgs/msg/radar_classification.hpp>
#include <autoware_sensing_msgs/msg/radar_info.hpp>
#include <autoware_sensing_msgs/msg/radar_objects.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::radar_objects_adapter
{

using RadarClassification = autoware_sensing_msgs::msg::RadarClassification;
using ObjectClassification = autoware_perception_msgs::msg::ObjectClassification;

// The classification remap as label ids, built from the radar label -> perception label names of
// the parameters.
using ClassificationRemap =
  std::map<RadarClassification::_label_type, ObjectClassification::_label_type>;

// The remap built from the parameter names, with what could not be read as a name.
struct ClassificationRemapParseResult
{
  ClassificationRemap remap;
  // The entries whose value is not an ObjectClassification label name (radar label -> that
  // value). They map the radar label to UNKNOWN; the node warns about them, once per radar label.
  std::map<std::string, std::string> unknown_perception_labels;
};

// `classification_remap_str` must have RadarClassification label names as keys.
ClassificationRemapParseResult make_classification_remap_from_string_pair(
  const std::map<std::string, std::string> & classification_remap_str);

// Why a radar info cannot be used: the required fields it does not declare. what() names them.
class InvalidRadarInfo : public std::runtime_error
{
public:
  explicit InvalidRadarInfo(const std::vector<std::string> & missing_required_fields);

  // The required fields that no radar info has declared so far, in the order they are required.
  const std::vector<std::string> & missing_required_fields() const
  {
    return missing_required_fields_;
  }

private:
  std::vector<std::string> missing_required_fields_;
};

// What the adapter decided from a valid radar info about how it will convert: so far, which
// optional fields the radar does not provide and the parameter value used in place of each.
class ConversionConfiguration
{
public:
  void add_field(std::string name, float default_value);

  // The defaulted fields, in the order they were added.
  const std::vector<std::pair<std::string, float>> & defaulted_fields() const;

private:
  std::vector<std::pair<std::string, float>> default_fields_;
};

// What a radar objects message converts into: the messages to publish.
struct ConversionOutcome
{
  autoware_perception_msgs::msg::DetectedObjects detections;
  autoware_perception_msgs::msg::TrackedObjects tracks;
};

// Why radar objects were dropped: no radar info with every required field has arrived yet.
// what() says so.
class MissingRadarInfo : public std::runtime_error
{
public:
  MissingRadarInfo();
};

// The values used for the object fields a radar does not provide.
struct RadarObjectsAdapterParams
{
  float default_position_z{0.0f};
  float default_velocity_z{0.0f};
  float default_acceleration_z{0.0f};
  float default_size_x{0.0f};
  float default_size_y{0.0f};
  float default_size_z{0.0f};
};

// Converts radar objects into detected objects and into tracked objects. Learns which fields the
// radar provides from its radar info messages; until a valid one has arrived, nothing is converted.
class RadarObjectsAdapter
{
public:
  RadarObjectsAdapter(
    const RadarObjectsAdapterParams & params, const ClassificationRemap & classification_remap,
    const std::string & topic_name);

  // What the adapter decided from a radar info: the configuration it will convert with (the
  // optional fields it will fill from a parameter instead of the object), or why it cannot
  // convert at all. The node logs this; the adapter is where the field -> parameter pairing
  // lives. Nothing is decided from an invalid radar info.
  tl::expected<ConversionConfiguration, InvalidRadarInfo> update_radar_info(
    const autoware_sensing_msgs::msg::RadarInfo & radar_info_msg);

  // The messages to publish, or why the objects were dropped: no radar info with every required
  // field has arrived yet.
  tl::expected<ConversionOutcome, MissingRadarInfo> convert(
    const autoware_sensing_msgs::msg::RadarObjects & input_msg) const;

private:
  void radar_cov_to_detection_pose_cov(
    const std::array<float, 6> & radar_pose_cov, const double orientation_std,
    std::array<double, 36> & pose_cov) const;

  void radar_cov_to_detection_twist_cov(
    const std::array<float, 6> & radar_twist_cov, const float yaw, const float yaw_rate_std,
    std::array<double, 36> & twist_cov) const;

  static void radar_cov_to_detection_acceleration_cov(
    const std::array<float, 6> & radar_acceleration_cov, const float yaw,
    std::array<double, 36> & acceleration_cov);

  template <typename ObjectType>
  void populate_common_fields(
    const autoware_sensing_msgs::msg::RadarObject & input_object, ObjectType & output_object,
    const float yaw) const;

  void populate_classifications(
    const std::vector<autoware_sensing_msgs::msg::RadarClassification> & input_classifications,
    std::vector<autoware_perception_msgs::msg::ObjectClassification> & output_classifications)
    const;

  autoware_perception_msgs::msg::DetectedObjects to_detected_objects(
    const autoware_sensing_msgs::msg::RadarObjects & input_msg) const;
  autoware_perception_msgs::msg::TrackedObjects to_tracked_objects(
    const autoware_sensing_msgs::msg::RadarObjects & input_msg) const;

  RadarObjectsAdapterParams params_;

  std::unordered_map<std::string, autoware_sensing_msgs::msg::RadarFieldInfo> field_info_map_;

  bool valid_radar_info_{false};

  std::vector<std::string> required_attributes_;

  bool position_z_available_{false};
  bool velocity_z_available_{false};
  bool acceleration_z_available_{false};
  bool size_x_available_{false};
  bool size_y_available_{false};
  bool size_z_available_{false};

  bool orientation_std_available_{false};
  bool orientation_rate_std_available_{false};

  ClassificationRemap classification_remap_;

  std::array<std::uint8_t, sizeof(std::size_t)> topic_hash_code_;
};

}  // namespace autoware::radar_objects_adapter

#endif  // RADAR_OBJECTS_ADAPTER_HPP_
