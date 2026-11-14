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

// The conversion has three parts, composed by RadarObjectsAdapter: the classification remap
// (ClassificationRemapper), the conversion of one radar object into a tracked object
// (TrackedObjectConverter, parameterized by how the optional fields are filled and by how a
// tracked object's id is made), and the derivation of the detected object from the tracked one.

#include <tl/expected.hpp>

#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/tracked_object.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_sensing_msgs/msg/radar_classification.hpp>
#include <autoware_sensing_msgs/msg/radar_info.hpp>
#include <autoware_sensing_msgs/msg/radar_object.hpp>
#include <autoware_sensing_msgs/msg/radar_objects.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::radar_objects_adapter
{

using RadarClassification = autoware_sensing_msgs::msg::RadarClassification;
using ObjectClassification = autoware_perception_msgs::msg::ObjectClassification;

// ---------------------------------------------------------------------------------------------
// Classification remap
// ---------------------------------------------------------------------------------------------

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

// The remap every radar needs, whatever it reports: the labels the perception pipeline has no
// use for (HAZARD, OVER_DRIVABLE, UNDER_DRIVABLE) become UNKNOWN.
ClassificationRemap perception_friendly_classification_remap();

// Turns the classifications of a radar object into perception classifications: one entry per
// radar entry, in order, with the probability copied and the label looked up in one table. The
// table is the perception-friendly remap with the sensor-dependent one laid over it (the
// classification_remap parameters, which correct what the radar in use is known to report
// unreliably), so that the sensor's entry wins where both say something. A label neither
// mentions becomes UNKNOWN.
class ClassificationRemapper
{
public:
  ClassificationRemapper(
    const ClassificationRemap & perception_friendly, const ClassificationRemap & sensor_dependent);

  std::vector<autoware_perception_msgs::msg::ObjectClassification> operator()(
    const std::vector<autoware_sensing_msgs::msg::RadarClassification> & classifications) const;

  const ClassificationRemap & combined() const { return combined_; }

private:
  ClassificationRemap combined_;
};

// ---------------------------------------------------------------------------------------------
// Tracked object conversion
// ---------------------------------------------------------------------------------------------

// How a tracked object's id is made from the radar's 32-bit object id. A function, so that the
// conversion can be given another way of making ids.
using GenerateObjectUUID =
  std::function<unique_identifier_msgs::msg::UUID::_uuid_type(std::uint32_t object_id)>;

// The way the node makes them: the id, least significant byte first, then the hash of the input
// topic name, least significant byte first, then zeros. The hash tells the tracks of one radar
// from those of another when they are merged downstream.
class ObjectUUIDGenerator
{
public:
  explicit ObjectUUIDGenerator(const std::string & topic_name);

  unique_identifier_msgs::msg::UUID::_uuid_type operator()(std::uint32_t object_id) const;

private:
  std::array<std::uint8_t, sizeof(std::size_t)> topic_hash_code_;
};

// How the optional object fields are filled when a radar object is converted. Each of the six
// that have a default_* parameter is copied from the object when the radar provides it (nullopt)
// and otherwise set to the value held here, the parameter's, as the double the object fields
// are. The two standard deviations have no parameter: when the radar does not provide one, the
// variance it would set stays zero.
//
// The parameters alone make the configuration for a radar that provides no optional field; the
// adapter makes the one it converts with from them and a valid radar info, which declares what
// the radar provides.
struct ConversionConfiguration
{
  std::optional<double> default_position_z;
  std::optional<double> default_velocity_z;
  std::optional<double> default_acceleration_z;
  std::optional<double> default_size_x;
  std::optional<double> default_size_y;
  std::optional<double> default_size_z;

  bool orientation_std_provided{false};
  bool orientation_rate_std_provided{false};

  // The fields set to a parameter's value, with the value, in the order the node warns about
  // them.
  std::vector<std::pair<std::string, double>> defaulted_fields() const;
};

// Converts one radar object into a tracked object: everything the radar reports about it, with
// the optional fields filled as the configuration says, the velocity and the acceleration
// rotated into the object's frame, and the id made by the given function. The classification is
// not set here; the remapper supplies it.
class TrackedObjectConverter
{
public:
  TrackedObjectConverter(const ConversionConfiguration & config, GenerateObjectUUID generate_uuid);

  autoware_perception_msgs::msg::TrackedObject operator()(
    const autoware_sensing_msgs::msg::RadarObject & input_object) const;

  // What it converts with.
  const ConversionConfiguration & configuration() const { return config_; }

private:
  ConversionConfiguration config_;
  GenerateObjectUUID generate_uuid_;
};

// ---------------------------------------------------------------------------------------------
// The adapter
// ---------------------------------------------------------------------------------------------

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

// The values used for the object fields a radar does not provide: the six default_* parameters,
// as the node hands them to the adapter.
struct RadarObjectsAdapterParams
{
  double default_position_z{0.0};
  double default_velocity_z{0.0};
  double default_acceleration_z{0.0};
  double default_size_x{0.0};
  double default_size_y{0.0};
  double default_size_z{0.0};

  // The configuration for a radar that provides no optional field: each parameter in place of
  // its field. The adapter takes away the entries for the fields a radar info declares.
  [[nodiscard]] ConversionConfiguration to_config() const;
};

// Converts radar objects into detected objects and into tracked objects. Learns which fields the
// radar provides from its radar info messages: the first message that declares every required
// field fixes the answer, and later messages are ignored; until then, each message is judged on
// its own and nothing is converted.
class RadarObjectsAdapter
{
public:
  RadarObjectsAdapter(
    const RadarObjectsAdapterParams & params, const ClassificationRemap & classification_remap,
    const std::string & topic_name);

  // What the adapter decided from a radar info: the configuration it will convert with (the
  // parameters, less the fields the radar provides), or why it cannot convert at all. The node
  // logs this; the adapter is where the field -> parameter pairing lives. Nothing is decided
  // from an invalid radar info, and nothing new from a message after the first valid one: that
  // one comes back with the configuration already fixed.
  tl::expected<ConversionConfiguration, InvalidRadarInfo> update_radar_info(
    const autoware_sensing_msgs::msg::RadarInfo & radar_info_msg);

  // The messages to publish, or why the objects were dropped: no radar info with every required
  // field has arrived yet.
  tl::expected<ConversionOutcome, MissingRadarInfo> convert(
    const autoware_sensing_msgs::msg::RadarObjects & input_msg) const;

private:
  // The parameters as a configuration: the converter gets this less the fields the radar
  // provides.
  ConversionConfiguration config_;
  GenerateObjectUUID generate_uuid_;
  ClassificationRemapper classification_remapper_;

  std::vector<std::string> required_attributes_;

  // Set by the first radar info that declares every required field, and kept; converts nothing
  // before.
  std::optional<TrackedObjectConverter> tracked_object_converter_;
};

}  // namespace autoware::radar_objects_adapter

#endif  // RADAR_OBJECTS_ADAPTER_HPP_
