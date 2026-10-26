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

// Unit tests of RadarObjectsAdapter, the conversion the node delegates to, called directly.
//
// These are the characterization cases of the node that read the converted objects, ported one
// to one: the same radar info and radar objects go in, through update_radar_info() and convert()
// instead of the topics, and the same detected and tracked objects are checked. They run without
// a ROS context. The adapter is used as the node uses it - the constructor, the remap built from
// the parameter names, update_radar_info() and convert() - and nothing reads its internals, so
// the implementation can be restructured under these tests.

#include "radar_objects_adapter.hpp"

#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_sensing_msgs/msg/radar_info.hpp>
#include <autoware_sensing_msgs/msg/radar_objects.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{
using autoware::radar_objects_adapter::make_classification_remap_from_string_pair;
using autoware::radar_objects_adapter::RadarObjectsAdapter;
using autoware::radar_objects_adapter::RadarObjectsAdapterParams;
using autoware_perception_msgs::msg::DetectedObject;
using autoware_perception_msgs::msg::DetectedObjectKinematics;
using autoware_perception_msgs::msg::DetectedObjects;
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::Shape;
using autoware_perception_msgs::msg::TrackedObject;
using autoware_perception_msgs::msg::TrackedObjectKinematics;
using autoware_perception_msgs::msg::TrackedObjects;
using autoware_sensing_msgs::msg::RadarClassification;
using autoware_sensing_msgs::msg::RadarFieldInfo;
using autoware_sensing_msgs::msg::RadarInfo;
using autoware_sensing_msgs::msg::RadarObject;
using autoware_sensing_msgs::msg::RadarObjects;

// The node fixes its own name in the constructor and resolves its topics against it. The adapter
// is given the input topic name as the node resolves it, because its hash is part of every
// tracked object's id.
constexpr char node_name[] = "radar_objects_adapter";

std::string node_topic(const std::string & relative_name)
{
  return "/" + std::string(node_name) + "/" + relative_name;
}

// Values for the six parameters the node declares without a default. They are chosen so that a
// field filled from a parameter can be told apart from one copied out of a radar object: no radar
// object built below carries any of these values.
struct DefaultParameters
{
  double position_z = 0.25;
  double velocity_z = 0.5;
  double acceleration_z = 0.75;
  double size_x = 5.0;
  double size_y = 2.0;
  double size_z = 1.5;

  // The six parameters as the node hands them to the adapter.
  [[nodiscard]] RadarObjectsAdapterParams to_params() const
  {
    RadarObjectsAdapterParams params;
    params.default_position_z = position_z;
    params.default_velocity_z = velocity_z;
    params.default_acceleration_z = acceleration_z;
    params.default_size_x = size_x;
    params.default_size_y = size_y;
    params.default_size_z = size_z;
    return params;
  }
};

// The six optional fields that have a parameter, with the value DefaultParameters gives each, in
// the order update_radar_info() reports them. orientation_std and orientation_rate_std have no
// parameter and are never reported.
using DefaultedField = std::pair<std::string, double>;
const std::vector<DefaultedField> all_defaulted = {{"position_z", 0.25},     {"velocity_z", 0.5},
                                                   {"acceleration_z", 0.75}, {"size_x", 5.0},
                                                   {"size_y", 2.0},          {"size_z", 1.5}};

std::vector<DefaultedField> all_defaulted_except(const std::string & declared)
{
  std::vector<DefaultedField> fields;
  for (const auto & field : all_defaulted) {
    if (field.first != declared) {
      fields.push_back(field);
    }
  }
  return fields;
}

// A radar info message that declares exactly these object fields. Only the names matter to the
// adapter; the min/max/resolution of each field are left unset.
RadarInfo make_radar_info(const std::set<std::string> & field_names)
{
  RadarInfo info;
  info.header.frame_id = "base_link";
  for (const auto & name : field_names) {
    RadarFieldInfo field;
    field.field_name.data = name;
    info.object_fields_info.push_back(field);
  }
  return info;
}

std::set<std::string> without(std::set<std::string> fields, const std::string & removed)
{
  fields.erase(removed);
  return fields;
}

std::set<std::string> with(std::set<std::string> fields, const std::set<std::string> & added)
{
  fields.insert(added.begin(), added.end());
  return fields;
}

// The eight object fields the adapter insists on. Without any one of them it reports the radar as
// incompatible and converts nothing.
const std::set<std::string> required_fields = {
  "existence_probability", "position_x",     "position_y", "velocity_x", "velocity_y",
  "acceleration_x",        "acceleration_y", "orientation"};

// The same eight, in the order the adapter requires (and names) them.
const std::vector<std::string> required_fields_in_order = {
  "existence_probability", "position_x",     "position_y", "velocity_x", "velocity_y",
  "acceleration_x",        "acceleration_y", "orientation"};

// The optional object fields the adapter looks at: six that are filled from a parameter when
// undeclared, and the two standard deviations, whose variances then stay zero.
const std::set<std::string> optional_fields = {
  "position_z", "velocity_z", "acceleration_z",  "size_x",
  "size_y",     "size_z",     "orientation_std", "orientation_rate_std"};

// The object fields a Continental ARS548 declares through its nebula driver (read from
// continental_ars548_decoder_wrapper.cpp on 2026-09-17). velocity_z, acceleration_z and size_z are
// not among them, so on the vehicle those three come from the parameters.
const std::set<std::string> ars548_fields = with(
  required_fields,
  {"object_id", "age", "measurement_status", "movement_status", "position_z", "size_x", "size_y",
   "orientation_std", "orientation_rate", "orientation_rate_std"});

// Every field the adapter looks at, so that nothing falls back to a parameter.
const std::set<std::string> all_fields =
  with(ars548_fields, {"velocity_z", "acceleration_z", "size_z"});

RadarClassification make_classification(uint8_t label, float probability)
{
  RadarClassification classification;
  classification.label = label;
  classification.probability = probability;
  return classification;
}

// A radar object with every field set to a distinct, recognizable value. Tests that care about a
// particular field overwrite it.
RadarObject make_radar_object()
{
  RadarObject object;
  object.object_id = 0x04030201u;
  object.age = 12;
  object.measurement_status = RadarObject::MEASUREMENT_STATUS_MEASURED;
  object.movement_status = RadarObject::MOVEMENT_STATUS_DYNAMIC;
  object.position.x = 10.0;
  object.position.y = -2.0;
  object.position.z = 0.8;
  object.velocity.x = 3.0;
  object.velocity.y = 0.5;
  object.velocity.z = 0.1;
  object.acceleration.x = 0.2;
  object.acceleration.y = -0.1;
  object.acceleration.z = 0.05;
  object.size.x = 4.5;
  object.size.y = 1.8;
  object.size.z = 1.4;
  // Upper triangles in the order XX, XY, XZ, YY, YZ, ZZ.
  object.position_covariance = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
  object.velocity_covariance = {0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f};
  object.acceleration_covariance = {0.01f, 0.02f, 0.03f, 0.04f, 0.05f, 0.06f};
  object.size_covariance = {0.7f, 0.8f, 0.9f, 1.1f, 1.2f, 1.3f};
  object.orientation = 0.3f;
  object.orientation_std = 0.1f;
  object.orientation_rate = 0.05f;
  object.orientation_rate_std = 0.02f;
  object.existence_probability = 0.9f;
  object.classifications = {make_classification(RadarClassification::CAR, 0.8f)};
  return object;
}

builtin_interfaces::msg::Time make_stamp(int32_t sec, uint32_t nanosec)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = sec;
  stamp.nanosec = nanosec;
  return stamp;
}

// Stamps are deliberately constant: the adapter copies the input header into both outputs, so a
// recognizable stamp tells which input an output came from.
const builtin_interfaces::msg::Time first_stamp = make_stamp(1700000000, 100);
const builtin_interfaces::msg::Time second_stamp = make_stamp(1700000001, 200);

RadarObjects make_radar_objects(
  const std::vector<RadarObject> & objects,
  const builtin_interfaces::msg::Time & stamp = first_stamp)
{
  RadarObjects msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = "base_link";
  msg.objects = objects;
  return msg;
}

// A quarter turn puts the object's x axis along the radar's y axis, which makes every rotation the
// adapter applies come out as a swap of components - checkable by hand.
constexpr double quarter_turn = 1.5707963267948966;

RadarObject facing(RadarObject object, double yaw)
{
  object.orientation = static_cast<float>(yaw);
  return object;
}

// A vector in the radar frame, expressed in the frame of an object facing `yaw`: the rotation by
// -yaw, written out.
std::pair<double, double> into_object_frame(double x, double y, double yaw)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return {c * x + s * y, -s * x + c * y};
}

// The 2x2 covariance [[xx, xy], [xy, yy]] of a vector in the radar frame, expressed in the frame
// of an object facing `yaw`: R Sigma R^T with R the rotation by -yaw, written out.
struct Covariance2x2
{
  double xx;
  double xy;
  double yy;
};

Covariance2x2 into_object_frame(const Covariance2x2 & cov, double yaw)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  // R = [[c, s], [-s, c]]; R Sigma = [[c xx + s xy, c xy + s yy], [-s xx + c xy, -s xy + c yy]]
  const double r00 = c * cov.xx + s * cov.xy;
  const double r01 = c * cov.xy + s * cov.yy;
  const double r10 = -s * cov.xx + c * cov.xy;
  const double r11 = -s * cov.xy + c * cov.yy;
  // (R Sigma) R^T
  return {r00 * c + r01 * s, r00 * (-s) + r01 * c, r10 * (-s) + r11 * c};
}

// A yaw for the rotation cases that go beyond the quarter turn: one at which neither sine nor
// cosine vanishes, so that the rotation by -yaw written out is told from any other mixing of the
// components, and which is not small, negative and past a half turn, so that it is also told
// from a rotation that loses the sign or the quadrant of the angle. The radar reports the
// orientation in radians, within -pi to pi.
constexpr double general_yaw = -2.5;

// Indices into the 6x6 row-major covariance matrices (x, y, z, roll, pitch, yaw) of the perception
// messages. The radar covariances are 3x3 upper triangles: XX, XY, XZ, YY, YZ, ZZ.
constexpr size_t cov_x_x = 0;
constexpr size_t cov_x_y = 1;
constexpr size_t cov_x_z = 2;
constexpr size_t cov_y_x = 6;
constexpr size_t cov_y_y = 7;
constexpr size_t cov_z_z = 14;
constexpr size_t cov_yaw_yaw = 35;
constexpr double covariance_tolerance = 1e-6;

// Every entry of a 6x6 covariance other than the ones named is zero.
bool only_these_entries_set(
  const std::array<double, 36> & covariance, const std::vector<size_t> & set_indices)
{
  for (size_t i = 0; i < covariance.size(); ++i) {
    const bool is_set = std::find(set_indices.begin(), set_indices.end(), i) != set_indices.end();
    if (!is_set && covariance[i] != 0.0) {
      return false;
    }
  }
  return true;
}

// The classification_remap.* parameters as the node declares them, with their built-in defaults:
// every label kept as it is, except HAZARD, which becomes UNKNOWN.
using RemapNames = std::map<std::string, std::string>;
const RemapNames built_in_remap = {{"UNKNOWN", "UNKNOWN"}, {"CAR", "CAR"},
                                   {"TRUCK", "TRUCK"},     {"MOTORCYCLE", "MOTORCYCLE"},
                                   {"BICYCLE", "BICYCLE"}, {"PEDESTRIAN", "PEDESTRIAN"},
                                   {"ANIMAL", "ANIMAL"},   {"HAZARD", "UNKNOWN"}};

// The remap parameters with the given radar labels remapped, on top of the built-in defaults.
RemapNames remap_names(
  const std::vector<std::pair<std::string, std::string>> & radar_to_perception_labels)
{
  RemapNames names = built_in_remap;
  for (const auto & [radar_label, perception_label] : radar_to_perception_labels) {
    names[radar_label] = perception_label;
  }
  return names;
}

// The adapter as the node constructs it: from the six parameters, the remap built from the
// classification_remap.* parameter values, and the input topic name.
RadarObjectsAdapter make_adapter(
  const RadarObjectsAdapterParams & params = DefaultParameters{}.to_params(),
  const RemapNames & remap = built_in_remap)
{
  return RadarObjectsAdapter(
    params, make_classification_remap_from_string_pair(remap).remap, node_topic("input/objects"));
}

// What the adapter returned for one radar objects message.
struct Outputs
{
  DetectedObjects detections;
  TrackedObjects tracks;
};

// Constructs the adapter with `params` and `remap`, opens the gate with `info`, converts `objects`
// once and collects what came out on both outputs - or nothing, if the adapter did not convert.
std::optional<Outputs> run_adapter_and_collect_outputs(
  const RadarInfo & info, const std::vector<RadarObject> & objects,
  const RadarObjectsAdapterParams & params = DefaultParameters{}.to_params(),
  const RemapNames & remap = built_in_remap)
{
  RadarObjectsAdapter adapter = make_adapter(params, remap);
  if (!adapter.update_radar_info(info).has_value()) {
    return std::nullopt;
  }
  const auto result = adapter.convert(make_radar_objects(objects));
  if (!result.has_value()) {
    return std::nullopt;
  }
  return Outputs{result->detections, result->tracks};
}

}  // namespace

// ---------------------------------------------------------------------------------------------
// The radar info gate. Radar objects are converted only after a radar info message has declared
// all of the fields the adapter requires; until then convert() answers with an error and no
// messages.
// ---------------------------------------------------------------------------------------------

// On the vehicle the radar info arrives later than the first radar objects: the driver publishes
// it only every tenth objects message. Until it arrives the adapter does not know which fields the
// radar provides, and it converts nothing rather than guess. The objects dropped in the meantime
// are gone for good - nothing is kept and replayed once the radar info is in - and only the
// objects that arrive after it are converted.
TEST(RadarObjectsAdapter, Gate_ObjectsBeforeRadarInfo_DroppedNotReplayed)
{
  // Arrange
  const RadarObjects before_info = make_radar_objects({make_radar_object()}, first_stamp);
  const RadarInfo info = make_radar_info(ars548_fields);
  const RadarObjects after_info = make_radar_objects({make_radar_object()}, second_stamp);
  RadarObjectsAdapter adapter = make_adapter();

  // Act: radar objects before any radar info ...
  const auto dropped = adapter.convert(before_info);

  // Assert: ... produce nothing
  EXPECT_FALSE(dropped.has_value());

  // Act: the radar info arrives, then more radar objects
  ASSERT_TRUE(adapter.update_radar_info(info).has_value());
  const auto converted = adapter.convert(after_info);

  // Assert: only the message converted after the radar info came through
  ASSERT_TRUE(converted.has_value());
  EXPECT_EQ(converted->detections.header.stamp, second_stamp);
  EXPECT_EQ(converted->tracks.header.stamp, second_stamp);
}

// A radar info that lacks one of the required fields (here: orientation) marks the radar as
// incompatible, and radar objects keep being dropped.
//
// Which field is missing makes no difference to the gate, so only one is tried.
TEST(RadarObjectsAdapter, Gate_RadarInfoMissingRequiredField_ObjectsDropped)
{
  // Arrange: the only radar info the adapter has seen lacks a required field
  const RadarInfo incomplete_info = make_radar_info(without(required_fields, "orientation"));
  const RadarObjects objects = make_radar_objects({make_radar_object()});
  RadarObjectsAdapter adapter = make_adapter();
  ASSERT_FALSE(adapter.update_radar_info(incomplete_info).has_value());

  // Act
  const auto result = adapter.convert(objects);

  // Assert
  EXPECT_FALSE(result.has_value());
}

// Once a radar info declares all required fields, every radar objects message produces one
// detected objects message and one tracked objects message, both carrying the input header.
//
// The contents of the objects are pinned by the conversion tests, not here.
TEST(RadarObjectsAdapter, Gate_ValidRadarInfo_ObjectsConverted)
{
  // Arrange: the adapter has seen a radar info that declares every required field
  const RadarInfo info = make_radar_info(ars548_fields);
  const RadarObjects objects = make_radar_objects({make_radar_object()});
  RadarObjectsAdapter adapter = make_adapter();
  ASSERT_TRUE(adapter.update_radar_info(info).has_value());

  // Act
  const auto result = adapter.convert(objects);

  // Assert
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->detections.header.stamp, first_stamp);
  EXPECT_EQ(result->detections.header.frame_id, "base_link");
  EXPECT_EQ(result->detections.objects.size(), 1u);

  EXPECT_EQ(result->tracks.header.stamp, first_stamp);
  EXPECT_EQ(result->tracks.header.frame_id, "base_link");
  EXPECT_EQ(result->tracks.objects.size(), 1u);
}

// ---------------------------------------------------------------------------------------------
// What update_radar_info() reports about a radar info: which required fields it is missing, and
// which optional fields will be filled from a parameter, with what value. The node used to log
// this; nothing read it, so the characterization suite could not pin it.
// ---------------------------------------------------------------------------------------------

// A radar info that declares the required fields and nothing else is valid, and every optional
// field that has a parameter is reported as filled from it, with the parameter's value.
TEST(RadarObjectsAdapter, RadarInfo_AllRequiredFields_ValidAndAllDefaulted)
{
  // Arrange
  RadarObjectsAdapter adapter = make_adapter();

  // Act
  const auto result = adapter.update_radar_info(make_radar_info(required_fields));

  // Assert
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->defaulted_fields(), all_defaulted);
}

// Each of the eight required fields is required on its own: leaving out any one of them keeps
// the radar info invalid, and the result names exactly that field.
TEST(RadarObjectsAdapter, RadarInfo_OneRequiredFieldMissing_InvalidAndNamed)
{
  for (const auto & missing : required_fields) {
    SCOPED_TRACE(missing);
    // Arrange
    RadarObjectsAdapter adapter = make_adapter();

    // Act
    const auto result =
      adapter.update_radar_info(make_radar_info(without(required_fields, missing)));

    // Assert
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().missing_required_fields(), std::vector<std::string>{missing});
  }
}

// When several required fields are missing, all of them are named, in the order they are
// required, whatever order the radar info declared the others in; the error's text, which the
// node logs, lists them the same way.
TEST(RadarObjectsAdapter, RadarInfo_SeveralRequiredFieldsMissing_AllNamedInRequiredOrder)
{
  // Arrange
  RadarObjectsAdapter adapter = make_adapter();

  // Act
  const auto result =
    adapter.update_radar_info(make_radar_info({"velocity_y", "position_x", "orientation"}));

  // Assert
  ASSERT_FALSE(result.has_value());
  const std::vector<std::string> expected = {
    "existence_probability", "position_y", "velocity_x", "acceleration_x", "acceleration_y"};
  EXPECT_EQ(result.error().missing_required_fields(), expected);
  const std::string what = result.error().what();
  EXPECT_NE(
    what.find("existence_probability, position_y, velocity_x, acceleration_x, acceleration_y"),
    std::string::npos)
    << what;
}

// A radar info with no object fields at all names every required field.
TEST(RadarObjectsAdapter, RadarInfo_NoFields_AllRequiredFieldsNamed)
{
  // Arrange
  RadarObjectsAdapter adapter = make_adapter();

  // Act
  const auto result = adapter.update_radar_info(make_radar_info({}));

  // Assert
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().missing_required_fields(), required_fields_in_order);
}

// Each optional field, once declared, drops out of the defaulted list and only it does.
TEST(RadarObjectsAdapter, RadarInfo_OneOptionalField_OnlyItNotDefaulted)
{
  for (const auto & declared : optional_fields) {
    SCOPED_TRACE(declared);
    // Arrange
    RadarObjectsAdapter adapter = make_adapter();

    // Act
    const auto result =
      adapter.update_radar_info(make_radar_info(with(required_fields, {declared})));

    // Assert
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->defaulted_fields(), all_defaulted_except(declared));
  }
}

// With every optional field declared, nothing is defaulted.
TEST(RadarObjectsAdapter, RadarInfo_AllOptionalFields_NothingDefaulted)
{
  // Arrange
  RadarObjectsAdapter adapter = make_adapter();

  // Act
  const auto result =
    adapter.update_radar_info(make_radar_info(with(required_fields, optional_fields)));

  // Assert
  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(result->defaulted_fields().empty());
}

// Fields the adapter does not know (the ones the radar in use declares on top of the ones above,
// and arbitrary names) are ignored: they neither help the required check nor count as optional.
TEST(RadarObjectsAdapter, RadarInfo_UnknownFields_Ignored)
{
  // Arrange
  RadarObjectsAdapter adapter = make_adapter();

  // Act
  const auto result = adapter.update_radar_info(make_radar_info(with(
    without(required_fields, "orientation"),
    {"object_id", "age", "measurement_status", "movement_status", "orientation_rate",
     "not_a_field"})));

  // Assert
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().missing_required_fields(), std::vector<std::string>{"orientation"});
}

// Until a valid radar info arrives, each message is judged on its own: two messages that each
// declare part of the required fields do not add up, and nothing is converted in between.
TEST(RadarObjectsAdapter, RadarInfo_BeforeValid_EachMessageJudgedOnItsOwn)
{
  // Arrange
  const RadarObjects objects = make_radar_objects({make_radar_object()});
  RadarObjectsAdapter adapter = make_adapter();

  // Act: a radar info lacking one required field, then one declaring only that field
  const auto first =
    adapter.update_radar_info(make_radar_info(without(required_fields, "orientation")));
  const auto second = adapter.update_radar_info(make_radar_info({"orientation"}));

  // Assert: the second is judged on its own, and nothing is converted
  EXPECT_FALSE(first.has_value());
  ASSERT_FALSE(second.has_value());
  const std::vector<std::string> all_but_orientation(
    required_fields_in_order.begin(), required_fields_in_order.end() - 1);
  EXPECT_EQ(second.error().missing_required_fields(), all_but_orientation);
  EXPECT_FALSE(adapter.convert(objects).has_value());

  // Act: a radar info declaring every required field
  const auto third = adapter.update_radar_info(make_radar_info(required_fields));

  // Assert
  EXPECT_TRUE(third.has_value());
  EXPECT_TRUE(adapter.convert(objects).has_value());
}

// The first valid radar info fixes the answer. A later message is not looked at: it comes back
// with the first's configuration, never as an error, and changes nothing about the conversion,
// whether it declares more fields, fewer, or lacks a required one.
TEST(RadarObjectsAdapter, RadarInfo_AfterValid_LaterRadarInfoIgnored)
{
  // Arrange: the first valid radar info declares position_z and no other optional field
  const DefaultParameters defaults;
  const RadarObjects objects = make_radar_objects({make_radar_object()});
  RadarObjectsAdapter adapter = make_adapter(defaults.to_params());
  ASSERT_TRUE(
    adapter.update_radar_info(make_radar_info(with(required_fields, {"position_z"}))).has_value());

  for (const auto & later :
       {with(required_fields, {"size_x"}), without(required_fields, "orientation"),
        std::set<std::string>{}}) {
    // Act
    const auto result = adapter.update_radar_info(make_radar_info(later));

    // Assert: still the first radar info's answer, and still its conversion (position_z copied,
    // size_x from the parameter)
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->defaulted_fields(), all_defaulted_except("position_z"));
    const auto converted = adapter.convert(objects);
    ASSERT_TRUE(converted.has_value());
    const DetectedObject & detected = converted->detections.objects.at(0);
    EXPECT_DOUBLE_EQ(
      detected.kinematics.pose_with_covariance.pose.position.z, objects.objects.at(0).position.z);
    EXPECT_DOUBLE_EQ(detected.shape.dimensions.x, defaults.size_x);
  }
}

// ---------------------------------------------------------------------------------------------
// Conversion into detected objects. A radar object is converted into a detected object with the
// fields that are explicitly declared in a preceding radar info. Each test case evaluates the
// converted outcome against a radar object with some specific aspects. The tracked object made of
// the same input is covered further down.
//
// Every case follows the same shape: the radar info and the radar objects are prepared first,
// the adapter is run on them once, and the output is read.
// ---------------------------------------------------------------------------------------------

// A radar objects message with no objects still produces one detected objects message and one
// tracked objects message, each empty and carrying the input header. This keeps the outputs alive
// while the radar sees nothing.
TEST(RadarObjectsAdapter, Conversion_EmptyObjects_EmptyOutputs)
{
  // Arrange
  const RadarInfo info = make_radar_info(ars548_fields);

  // Act
  const auto outputs = run_adapter_and_collect_outputs(info, {});
  ASSERT_TRUE(outputs.has_value());

  // Assert
  EXPECT_EQ(outputs->detections.header.stamp, first_stamp);
  EXPECT_TRUE(outputs->detections.objects.empty());
  EXPECT_EQ(outputs->tracks.header.stamp, first_stamp);
  EXPECT_TRUE(outputs->tracks.objects.empty());
}

// When the radar info declares a field, the adapter copies it from the radar object as it is - no
// coordinate transform, no scaling. Every field the radar info can declare is declared here, so
// nothing falls back to a parameter.
//
// The velocity's x and y are not copied but rotated, and are pinned separately.
TEST(RadarObjectsAdapter, Detections_DeclaredFields_CopiedFromObject)
{
  // Arrange
  const RadarInfo info = make_radar_info(all_fields);
  const RadarObject radar = make_radar_object();

  // Act
  const auto outputs = run_adapter_and_collect_outputs(info, {radar});
  ASSERT_TRUE(outputs.has_value());

  // Assert: every declared field carries the radar object's value
  const DetectedObject & detected = outputs->detections.objects.at(0);

  EXPECT_FLOAT_EQ(detected.existence_probability, radar.existence_probability);

  const auto & position = detected.kinematics.pose_with_covariance.pose.position;
  EXPECT_DOUBLE_EQ(position.x, radar.position.x);
  EXPECT_DOUBLE_EQ(position.y, radar.position.y);
  EXPECT_DOUBLE_EQ(position.z, radar.position.z);

  const auto & twist = detected.kinematics.twist_with_covariance.twist;
  EXPECT_DOUBLE_EQ(twist.linear.z, radar.velocity.z);
  EXPECT_DOUBLE_EQ(twist.angular.z, static_cast<double>(radar.orientation_rate));

  EXPECT_EQ(detected.shape.type, Shape::BOUNDING_BOX);
  EXPECT_DOUBLE_EQ(detected.shape.dimensions.x, radar.size.x);
  EXPECT_DOUBLE_EQ(detected.shape.dimensions.y, radar.size.y);
  EXPECT_DOUBLE_EQ(detected.shape.dimensions.z, radar.size.z);
}

// When the radar info does not declare a field, the adapter ignores the value in the radar object
// and fills the field from the parameter. This is the situation on the vehicle for velocity_z,
// acceleration_z and size_z; here every optional field is left undeclared so that every fallback
// is seen at once. The yaw variances stay zero because the orientation standard deviations are
// undeclared as well.
TEST(RadarObjectsAdapter, Detections_UndeclaredFields_FilledFromParameters)
{
  // Arrange
  const DefaultParameters defaults;
  const RadarInfo info = make_radar_info(required_fields);
  const RadarObject radar = make_radar_object();

  // Act
  const auto outputs = run_adapter_and_collect_outputs(info, {radar}, defaults.to_params());
  ASSERT_TRUE(outputs.has_value());

  // Assert: every undeclared field carries the parameter's value
  const DetectedObject & detected = outputs->detections.objects.at(0);

  const auto & kinematics = detected.kinematics;
  EXPECT_DOUBLE_EQ(kinematics.pose_with_covariance.pose.position.z, defaults.position_z);
  EXPECT_DOUBLE_EQ(kinematics.twist_with_covariance.twist.linear.z, defaults.velocity_z);
  EXPECT_DOUBLE_EQ(kinematics.pose_with_covariance.covariance[cov_yaw_yaw], 0.0);
  EXPECT_DOUBLE_EQ(kinematics.twist_with_covariance.covariance[cov_yaw_yaw], 0.0);

  EXPECT_EQ(detected.shape.type, Shape::BOUNDING_BOX);
  EXPECT_DOUBLE_EQ(detected.shape.dimensions.x, defaults.size_x);
  EXPECT_DOUBLE_EQ(detected.shape.dimensions.y, defaults.size_y);
  EXPECT_DOUBLE_EQ(detected.shape.dimensions.z, defaults.size_z);
}

// The two cases above declare all optional fields or none. The choice is made per field: each
// optional value is copied from the object when its field alone is declared, and taken from the
// parameter when it alone is left out, on whichever output carries it.
TEST(RadarObjectsAdapter, Detections_OptionalFields_ChoiceMadePerField)
{
  // Arrange
  const DefaultParameters defaults;
  const RadarObject radar = make_radar_object();

  struct Case
  {
    std::string field;
    std::function<double(const DetectedObject &, const TrackedObject &)> read;
    double from_object;
    double from_parameter;
  };
  const std::vector<Case> cases = {
    {"position_z",
     [](const DetectedObject & d, const TrackedObject &) {
       return d.kinematics.pose_with_covariance.pose.position.z;
     },
     radar.position.z, defaults.position_z},
    {"velocity_z",
     [](const DetectedObject & d, const TrackedObject &) {
       return d.kinematics.twist_with_covariance.twist.linear.z;
     },
     radar.velocity.z, defaults.velocity_z},
    {"acceleration_z",
     [](const DetectedObject &, const TrackedObject & t) {
       return t.kinematics.acceleration_with_covariance.accel.linear.z;
     },
     radar.acceleration.z, defaults.acceleration_z},
    {"size_x", [](const DetectedObject & d, const TrackedObject &) { return d.shape.dimensions.x; },
     radar.size.x, defaults.size_x},
    {"size_y", [](const DetectedObject & d, const TrackedObject &) { return d.shape.dimensions.y; },
     radar.size.y, defaults.size_y},
    {"size_z", [](const DetectedObject & d, const TrackedObject &) { return d.shape.dimensions.z; },
     radar.size.z, defaults.size_z},
  };

  for (const auto & c : cases) {
    SCOPED_TRACE(c.field);
    // Act: the field alone declared, then the field alone left out
    const auto declared =
      run_adapter_and_collect_outputs(make_radar_info(with(required_fields, {c.field})), {radar});
    const auto undeclared =
      run_adapter_and_collect_outputs(make_radar_info(without(all_fields, c.field)), {radar});
    ASSERT_TRUE(declared.has_value());
    ASSERT_TRUE(undeclared.has_value());

    // Assert
    EXPECT_DOUBLE_EQ(
      c.read(declared->detections.objects.at(0), declared->tracks.objects.at(0)), c.from_object);
    EXPECT_DOUBLE_EQ(
      c.read(undeclared->detections.objects.at(0), undeclared->tracks.objects.at(0)),
      c.from_parameter);
  }
}

// The parameters above are values a float represents exactly, so they cannot tell a parameter
// that was rounded to float on the way from one that was not. These cannot be represented as
// float: each must reach the output as written, on whichever output carries it.
TEST(RadarObjectsAdapter, Detections_UndeclaredFields_ParameterNotRoundedToFloat)
{
  // Arrange
  DefaultParameters defaults;
  defaults.position_z = 0.1;
  defaults.velocity_z = 0.2;
  defaults.acceleration_z = 0.3;
  defaults.size_x = 4.1;
  defaults.size_y = 1.9;
  defaults.size_z = 1.7;
  const RadarInfo info = make_radar_info(required_fields);
  const RadarObject radar = make_radar_object();

  // Act
  const auto outputs = run_adapter_and_collect_outputs(info, {radar}, defaults.to_params());
  ASSERT_TRUE(outputs.has_value());

  // Assert: the parameter's value as written, not its nearest float
  const DetectedObject & detected = outputs->detections.objects.at(0);
  const TrackedObject & tracked = outputs->tracks.objects.at(0);
  EXPECT_DOUBLE_EQ(detected.kinematics.pose_with_covariance.pose.position.z, 0.1);
  EXPECT_DOUBLE_EQ(detected.kinematics.twist_with_covariance.twist.linear.z, 0.2);
  EXPECT_DOUBLE_EQ(tracked.kinematics.acceleration_with_covariance.accel.linear.z, 0.3);
  EXPECT_DOUBLE_EQ(detected.shape.dimensions.x, 4.1);
  EXPECT_DOUBLE_EQ(detected.shape.dimensions.y, 1.9);
  EXPECT_DOUBLE_EQ(detected.shape.dimensions.z, 1.7);
}

// The radar's yaw angle becomes the orientation quaternion: a rotation about z alone.
TEST(RadarObjectsAdapter, Detections_Orientation_QuaternionFromYaw)
{
  // Arrange
  const RadarInfo info = make_radar_info(ars548_fields);
  const RadarObject rotated_object = facing(make_radar_object(), 0.3);

  // Act
  const auto outputs = run_adapter_and_collect_outputs(info, {rotated_object});
  ASSERT_TRUE(outputs.has_value());

  // Assert: a yaw of 0.3 rad about z is (0, 0, sin(0.15), cos(0.15))
  const auto & orientation =
    outputs->detections.objects.at(0).kinematics.pose_with_covariance.pose.orientation;
  EXPECT_NEAR(orientation.x, 0.0, 1e-9);
  EXPECT_NEAR(orientation.y, 0.0, 1e-9);
  EXPECT_NEAR(orientation.z, std::sin(0.15), 1e-6);
  EXPECT_NEAR(orientation.w, std::cos(0.15), 1e-6);
}

namespace
{
// The detected object's kinematics flags say what the message carries, not where a value came
// from: the pose covariance, the twist and the twist covariance are declared present and the
// orientation fully known. Two cases check that this holds whatever the radar info declares.
void expect_all_kinematics_flags_set(const DetectedObjectKinematics & kinematics)
{
  EXPECT_TRUE(kinematics.has_position_covariance);
  EXPECT_EQ(kinematics.orientation_availability, DetectedObjectKinematics::AVAILABLE);
  EXPECT_TRUE(kinematics.has_twist);
  EXPECT_TRUE(kinematics.has_twist_covariance);
}
}  // namespace

// With only the required fields declared, both flagged covariances come from the parameters -
// and the flags are set all the same.
TEST(RadarObjectsAdapter, Detections_KinematicsFlags_SetWithOnlyRequiredFields)
{
  // Arrange
  const RadarInfo info = make_radar_info(required_fields);
  const RadarObject radar = make_radar_object();

  // Act
  const auto outputs = run_adapter_and_collect_outputs(info, {radar});
  ASSERT_TRUE(outputs.has_value());

  // Assert
  expect_all_kinematics_flags_set(outputs->detections.objects.at(0).kinematics);
}

// With every optional field declared, the same flags are set.
TEST(RadarObjectsAdapter, Detections_KinematicsFlags_SetWithAllFields)
{
  // Arrange
  const RadarInfo info = make_radar_info(all_fields);
  const RadarObject radar = make_radar_object();

  // Act
  const auto outputs = run_adapter_and_collect_outputs(info, {radar});
  ASSERT_TRUE(outputs.has_value());

  // Assert
  expect_all_kinematics_flags_set(outputs->detections.objects.at(0).kinematics);
}

// The radar reports velocity in its own frame; the detected object's twist is expressed in the
// object's frame, so the adapter rotates the velocity by the object's yaw. With a quarter turn, a
// velocity along the radar's x axis becomes a velocity along the object's negative y axis.
//
// Since ars548_fields does not have velocity_z, radar.velocity.z is ignored; the z component is
// not part of the rotation, and where it comes from is pinned by the declared/undeclared cases.
TEST(RadarObjectsAdapter, Detections_Twist_VelocityRotatedIntoObjectFrame)
{
  // Arrange
  const RadarInfo info = make_radar_info(ars548_fields);
  RadarObject radar = facing(make_radar_object(), quarter_turn);
  radar.velocity.x = 1.0;
  radar.velocity.y = 0.0;

  // Act
  const auto outputs = run_adapter_and_collect_outputs(info, {radar});
  ASSERT_TRUE(outputs.has_value());

  // Assert: (1, 0) in the radar's frame is (0, -1) in the object's frame
  const auto & linear =
    outputs->detections.objects.at(0).kinematics.twist_with_covariance.twist.linear;
  EXPECT_NEAR(linear.x, 0.0, 1e-6);
  EXPECT_NEAR(linear.y, -1.0, 1e-6);
}

// The x/y block of the position covariance is copied without rotation - it is expressed in the
// header frame, like the position itself - and the yaw variance is the square of the radar's
// orientation standard deviation. The z entries of the radar covariance are dropped, and every
// other entry of the 6x6 matrix stays zero.
TEST(RadarObjectsAdapter, Detections_PoseCovariance_CopiedWithoutRotationWithYawVariance)
{
  // Arrange
  const RadarInfo info = make_radar_info(ars548_fields);
  const RadarObject radar = facing(make_radar_object(), quarter_turn);

  // Act
  const auto outputs = run_adapter_and_collect_outputs(info, {radar});
  ASSERT_TRUE(outputs.has_value());

  // Assert
  const auto & covariance =
    outputs->detections.objects.at(0).kinematics.pose_with_covariance.covariance;
  // The x/y block, as the radar reported it (XX, XY, YY), not rotated by the quarter turn
  EXPECT_NEAR(covariance[cov_x_x], radar.position_covariance[0], covariance_tolerance);
  EXPECT_NEAR(covariance[cov_x_y], radar.position_covariance[1], covariance_tolerance);
  EXPECT_NEAR(covariance[cov_y_x], radar.position_covariance[1], covariance_tolerance);
  EXPECT_NEAR(covariance[cov_y_y], radar.position_covariance[3], covariance_tolerance);
  // The z entries of the radar covariance are not carried over
  EXPECT_DOUBLE_EQ(covariance[cov_x_z], 0.0);
  EXPECT_DOUBLE_EQ(covariance[cov_z_z], 0.0);
  // The yaw variance
  const double yaw_std = radar.orientation_std;
  EXPECT_NEAR(covariance[cov_yaw_yaw], yaw_std * yaw_std, covariance_tolerance);
  EXPECT_TRUE(
    only_these_entries_set(covariance, {cov_x_x, cov_x_y, cov_y_x, cov_y_y, cov_yaw_yaw}));
}

// The x/y block of the velocity covariance is rotated into the object's frame along with the
// velocity. With a quarter turn the x and y variances swap places and the covariance changes
// sign. The yaw rate variance is the square of the radar's orientation rate standard deviation.
TEST(RadarObjectsAdapter, Detections_TwistCovariance_RotatedByYaw)
{
  // Arrange
  const RadarInfo info = make_radar_info(ars548_fields);
  RadarObject radar = facing(make_radar_object(), quarter_turn);
  radar.velocity_covariance = {1.0f, 0.5f, 0.0f, 4.0f, 0.0f, 0.0f};

  // Act
  const auto outputs = run_adapter_and_collect_outputs(info, {radar});
  ASSERT_TRUE(outputs.has_value());

  // Assert
  const auto & covariance =
    outputs->detections.objects.at(0).kinematics.twist_with_covariance.covariance;
  // Variances swapped, covariance negated
  EXPECT_NEAR(covariance[cov_x_x], 4.0, covariance_tolerance);
  EXPECT_NEAR(covariance[cov_y_y], 1.0, covariance_tolerance);
  EXPECT_NEAR(covariance[cov_x_y], -0.5, covariance_tolerance);
  EXPECT_NEAR(covariance[cov_y_x], -0.5, covariance_tolerance);
  // The yaw rate variance
  const double yaw_rate_std = radar.orientation_rate_std;
  EXPECT_NEAR(covariance[cov_yaw_yaw], yaw_rate_std * yaw_rate_std, covariance_tolerance);
  EXPECT_TRUE(
    only_these_entries_set(covariance, {cov_x_x, cov_x_y, cov_y_x, cov_y_y, cov_yaw_yaw}));
}

// The quarter turn above shows the rotation as a swap; at a general yaw the velocity and its
// covariance must match the rotation by -yaw written out, which pins the direction and the
// formula rather than the special case.
TEST(RadarObjectsAdapter, Detections_Twist_RotatedAtGeneralYaw)
{
  // Arrange
  const RadarInfo info = make_radar_info(ars548_fields);
  RadarObject radar = facing(make_radar_object(), general_yaw);
  radar.velocity_covariance = {1.0f, 0.5f, 9.0f, 4.0f, 9.0f, 9.0f};

  // Act
  const auto outputs = run_adapter_and_collect_outputs(info, {radar});
  ASSERT_TRUE(outputs.has_value());

  // Assert: the vector and the x/y block of its covariance, rotated by -yaw; the z entries of
  // the radar covariance dropped
  const auto & twist = outputs->detections.objects.at(0).kinematics.twist_with_covariance;
  const auto [vx, vy] = into_object_frame(radar.velocity.x, radar.velocity.y, general_yaw);
  EXPECT_NEAR(twist.twist.linear.x, vx, 1e-6);
  EXPECT_NEAR(twist.twist.linear.y, vy, 1e-6);
  const Covariance2x2 expected = into_object_frame({1.0, 0.5, 4.0}, general_yaw);
  EXPECT_NEAR(twist.covariance[cov_x_x], expected.xx, covariance_tolerance);
  EXPECT_NEAR(twist.covariance[cov_x_y], expected.xy, covariance_tolerance);
  EXPECT_NEAR(twist.covariance[cov_y_x], expected.xy, covariance_tolerance);
  EXPECT_NEAR(twist.covariance[cov_y_y], expected.yy, covariance_tolerance);
  EXPECT_TRUE(
    only_these_entries_set(twist.covariance, {cov_x_x, cov_x_y, cov_y_x, cov_y_y, cov_yaw_yaw}));
}

// A radar marks a covariance entry it cannot provide with INVALID_COV_VALUE (-1). The adapter
// turns those into zero instead of passing a negative variance downstream. The yaw variances are
// not affected, because they come from the standard deviations, not from these arrays.
TEST(RadarObjectsAdapter, Detections_InvalidCovariance_MaskedToZero)
{
  // Arrange
  const RadarInfo info = make_radar_info(ars548_fields);
  RadarObject radar = make_radar_object();
  radar.position_covariance.fill(RadarObject::INVALID_COV_VALUE);
  radar.velocity_covariance.fill(RadarObject::INVALID_COV_VALUE);

  // Act
  const auto outputs = run_adapter_and_collect_outputs(info, {radar});
  ASSERT_TRUE(outputs.has_value());

  // Assert: the x/y blocks are zero, the yaw variances are not
  const auto & kinematics = outputs->detections.objects.at(0).kinematics;
  EXPECT_TRUE(only_these_entries_set(kinematics.pose_with_covariance.covariance, {cov_yaw_yaw}));
  EXPECT_GT(kinematics.pose_with_covariance.covariance[cov_yaw_yaw], 0.0);
  EXPECT_TRUE(only_these_entries_set(kinematics.twist_with_covariance.covariance, {cov_yaw_yaw}));
  EXPECT_GT(kinematics.twist_with_covariance.covariance[cov_yaw_yaw], 0.0);
}

// ---------------------------------------------------------------------------------------------
// Classification. Each radar classification (a label with a probability) becomes one perception
// classification; the label is looked up in the classification_remap table and the probability
// is copied. The tests read the detected object; the tracked object gets the same list, which is
// pinned with the tracked object tests.
//
// What the built-in defaults of the classification_remap.* parameters are is the node's, and
// stays with the node test; the adapter is given the table those parameters build.
// ---------------------------------------------------------------------------------------------

namespace
{
// A classification list as (label, probability) pairs, so that a test can state the whole
// expected list in one place.
using LabeledProbability = std::pair<uint8_t, float>;

std::vector<LabeledProbability> labeled_probabilities(
  const std::vector<ObjectClassification> & classifications)
{
  std::vector<LabeledProbability> result;
  for (const auto & classification : classifications) {
    result.emplace_back(classification.label, classification.probability);
  }
  return result;
}
}  // namespace

// The shipped parameter file remaps MOTORCYCLE and BICYCLE to CAR, because a radar tends to
// report a far car as a two-wheeler. The remap works entry by entry: an object that carries a
// CAR, a MOTORCYCLE and a BICYCLE probability comes out with three CAR entries, each with its own
// probability, while labels the remap does not mention (TRUCK, PEDESTRIAN) stay as they are.
//
// NOTE(characterization): the duplicate labels are pinned as they are. The README notes them as
// harmless for the current consumers; merging the probabilities is a candidate change for later.
TEST(RadarObjectsAdapter, Classification_ConfiguredRemap_TwoWheelersBecomeCar)
{
  // Arrange
  const RadarInfo info = make_radar_info(ars548_fields);
  const RemapNames remap = remap_names({{"MOTORCYCLE", "CAR"}, {"BICYCLE", "CAR"}});
  RadarObject radar = make_radar_object();
  radar.classifications = {
    make_classification(RadarClassification::CAR, 0.5f),
    make_classification(RadarClassification::TRUCK, 0.1f),
    make_classification(RadarClassification::MOTORCYCLE, 0.8f),
    make_classification(RadarClassification::BICYCLE, 0.3f),
    make_classification(RadarClassification::PEDESTRIAN, 0.02f)};

  // Act
  const auto outputs =
    run_adapter_and_collect_outputs(info, {radar}, DefaultParameters{}.to_params(), remap);
  ASSERT_TRUE(outputs.has_value());

  // Assert
  const std::vector<LabeledProbability> expected = {
    {ObjectClassification::CAR, 0.5f},
    {ObjectClassification::TRUCK, 0.1f},
    {ObjectClassification::CAR, 0.8f},
    {ObjectClassification::CAR, 0.3f},
    {ObjectClassification::PEDESTRIAN, 0.02f}};
  EXPECT_EQ(labeled_probabilities(outputs->detections.objects.at(0).classification), expected);
}

// The remap table has entries for eight of the twelve radar labels. A label without an entry -
// BUS, TRAILER, OVER_DRIVABLE and UNDER_DRIVABLE - becomes UNKNOWN, keeping its probability. The
// radar in use does not report these labels, so this path is not taken on the vehicle.
TEST(RadarObjectsAdapter, Classification_LabelWithoutRemapEntry_BecomesUnknown)
{
  // Arrange
  const RadarInfo info = make_radar_info(ars548_fields);
  RadarObject radar = make_radar_object();
  radar.classifications = {
    make_classification(RadarClassification::BUS, 0.7f),
    make_classification(RadarClassification::TRAILER, 0.2f),
    make_classification(RadarClassification::OVER_DRIVABLE, 0.1f),
    make_classification(RadarClassification::UNDER_DRIVABLE, 0.05f)};

  // Act
  const auto outputs = run_adapter_and_collect_outputs(info, {radar});
  ASSERT_TRUE(outputs.has_value());

  // Assert
  const std::vector<LabeledProbability> expected = {
    {ObjectClassification::UNKNOWN, 0.7f},
    {ObjectClassification::UNKNOWN, 0.2f},
    {ObjectClassification::UNKNOWN, 0.1f},
    {ObjectClassification::UNKNOWN, 0.05f}};
  EXPECT_EQ(labeled_probabilities(outputs->detections.objects.at(0).classification), expected);
}

// A classification_remap.* value that is not a perception label name is not an error: the table
// is built and maps that radar label to UNKNOWN.
//
// The warning the node logs for such a value is not pinned.
TEST(RadarObjectsAdapter, Classification_UnknownLabelInParameter_MapsToUnknown)
{
  // Arrange
  const RadarInfo info = make_radar_info(ars548_fields);
  const RemapNames remap = remap_names({{"CAR", "SPACESHIP"}});
  RadarObject radar = make_radar_object();
  radar.classifications = {make_classification(RadarClassification::CAR, 0.8f)};

  // Act
  const auto outputs =
    run_adapter_and_collect_outputs(info, {radar}, DefaultParameters{}.to_params(), remap);
  ASSERT_TRUE(outputs.has_value());

  // Assert
  const std::vector<LabeledProbability> expected = {{ObjectClassification::UNKNOWN, 0.8f}};
  EXPECT_EQ(labeled_probabilities(outputs->detections.objects.at(0).classification), expected);
}

// An object without classifications comes out without classifications, on both outputs.
TEST(RadarObjectsAdapter, Classification_NoClassifications_EmptyList)
{
  // Arrange
  const RadarInfo info = make_radar_info(ars548_fields);
  RadarObject radar = make_radar_object();
  radar.classifications.clear();

  // Act
  const auto outputs = run_adapter_and_collect_outputs(info, {radar});
  ASSERT_TRUE(outputs.has_value());

  // Assert
  EXPECT_TRUE(outputs->detections.objects.at(0).classification.empty());
  EXPECT_TRUE(outputs->tracks.objects.at(0).classification.empty());
}

// ---------------------------------------------------------------------------------------------
// Conversion into tracked objects. The same radar object goes in, and the tracked object made of
// it is read. It shares pose, twist, shape and classification with the detected object, and adds
// what a track has over a detection: an identity, an acceleration and a stationary flag.
// ---------------------------------------------------------------------------------------------

// The 16-byte UUID is assembled from the radar's 32-bit object id, least significant byte first,
// followed by 8 bytes of a hash of the fully qualified input topic name, again least significant
// byte first, followed by 4 zero bytes. The hash keeps the ids of two radars apart when their
// tracks are merged downstream, and it is the same for every object of one adapter.
//
// Both the adapter and this test take the bytes out of the integers with shifts, so the layout is
// pinned independently of the byte order of the machine. The hash is std::hash<std::string> of
// the topic name, so it is recomputed here rather than written down: its value depends on the
// standard library.
TEST(RadarObjectsAdapter, Tracks_Uuid_ObjectIdThenTopicHashThenZeros)
{
  constexpr size_t object_id_bytes = 4;
  constexpr size_t topic_hash_bytes = 8;

  // Arrange
  const RadarInfo info = make_radar_info(ars548_fields);
  RadarObject first = make_radar_object();
  first.object_id = 0x04030201u;
  RadarObject second = make_radar_object();
  second.object_id = 0x44332211u;

  // Act
  const auto outputs = run_adapter_and_collect_outputs(info, {first, second});
  ASSERT_TRUE(outputs.has_value());

  // Assert
  const auto & first_uuid = outputs->tracks.objects.at(0).object_id.uuid;
  const auto & second_uuid = outputs->tracks.objects.at(1).object_id.uuid;

  // Bytes 0-3: the radar's object id
  EXPECT_EQ(
    (std::vector<uint8_t>{first_uuid[0], first_uuid[1], first_uuid[2], first_uuid[3]}),
    (std::vector<uint8_t>{0x01, 0x02, 0x03, 0x04}));
  EXPECT_EQ(
    (std::vector<uint8_t>{second_uuid[0], second_uuid[1], second_uuid[2], second_uuid[3]}),
    (std::vector<uint8_t>{0x11, 0x22, 0x33, 0x44}));

  // Bytes 4-11: the hash of the input topic name
  const size_t topic_hash = std::hash<std::string>{}(node_topic("input/objects"));
  for (size_t i = 0; i < topic_hash_bytes; ++i) {
    SCOPED_TRACE(i);
    EXPECT_EQ(
      first_uuid[object_id_bytes + i], static_cast<uint8_t>((topic_hash >> (i * 8)) & 0xFF));
  }

  // Bytes 12-15: zero
  for (size_t i = object_id_bytes + topic_hash_bytes; i < first_uuid.size(); ++i) {
    SCOPED_TRACE(i);
    EXPECT_EQ(first_uuid[i], 0);
  }

  // Everything after the object id is shared by every object of this adapter
  EXPECT_TRUE(
    std::equal(
      first_uuid.begin() + object_id_bytes, first_uuid.end(),
      second_uuid.begin() + object_id_bytes));
}

// Two adapters on different input topics give the same object different ids: the hash is what
// tells one radar's tracks from another's.
TEST(RadarObjectsAdapter, Tracks_Uuid_DiffersBetweenTopics)
{
  // Arrange
  const RadarInfo info = make_radar_info(ars548_fields);
  const RadarObjects objects = make_radar_objects({make_radar_object()});
  RadarObjectsAdapter front = make_adapter();
  RadarObjectsAdapter rear(
    DefaultParameters{}.to_params(),
    make_classification_remap_from_string_pair(built_in_remap).remap,
    "/sensing/radar/rear/objects_raw");
  ASSERT_TRUE(front.update_radar_info(info).has_value());
  ASSERT_TRUE(rear.update_radar_info(info).has_value());

  // Act
  const auto from_front = front.convert(objects);
  const auto from_rear = rear.convert(objects);
  ASSERT_TRUE(from_front.has_value());
  ASSERT_TRUE(from_rear.has_value());

  // Assert
  EXPECT_NE(
    from_front->tracks.objects.at(0).object_id.uuid,
    from_rear->tracks.objects.at(0).object_id.uuid);
}

// Only an object the radar reports as DYNAMIC is moving; STATIC, but also INVALID and UNKNOWN,
// are reported as stationary.
TEST(RadarObjectsAdapter, Tracks_MovementStatus_OnlyDynamicIsMoving)
{
  // Arrange: one object per movement status, in this order
  const RadarInfo info = make_radar_info(ars548_fields);
  const auto with_status = [](uint8_t status) {
    RadarObject radar = make_radar_object();
    radar.movement_status = status;
    return radar;
  };
  const std::vector<RadarObject> radars = {
    with_status(RadarObject::MOVEMENT_STATUS_DYNAMIC),
    with_status(RadarObject::MOVEMENT_STATUS_STATIC),
    with_status(RadarObject::MOVEMENT_STATUS_INVALID),
    with_status(RadarObject::MOVEMENT_STATUS_UNKNOWN)};

  // Act
  const auto outputs = run_adapter_and_collect_outputs(info, radars);
  ASSERT_TRUE(outputs.has_value());

  // Assert
  const auto & tracks = outputs->tracks.objects;
  EXPECT_FALSE(tracks.at(0).kinematics.is_stationary);
  EXPECT_TRUE(tracks.at(1).kinematics.is_stationary);
  EXPECT_TRUE(tracks.at(2).kinematics.is_stationary);
  EXPECT_TRUE(tracks.at(3).kinematics.is_stationary);
}

// The acceleration gets the same treatment as the velocity: rotated into the object's frame
// together with its x/y covariance. The detected object has no acceleration, so this is only
// checked here. The z component is not rotated and is pinned by the two cases that follow.
TEST(RadarObjectsAdapter, Tracks_Acceleration_RotatedByYaw)
{
  // Arrange
  const RadarInfo info = make_radar_info(ars548_fields);
  RadarObject radar = facing(make_radar_object(), quarter_turn);
  radar.acceleration.x = 1.0;
  radar.acceleration.y = 0.0;
  radar.acceleration_covariance = {1.0f, 0.5f, 0.0f, 4.0f, 0.0f, 0.0f};

  // Act
  const auto outputs = run_adapter_and_collect_outputs(info, {radar});
  ASSERT_TRUE(outputs.has_value());

  // Assert
  const auto & acceleration = outputs->tracks.objects.at(0).kinematics.acceleration_with_covariance;
  EXPECT_NEAR(acceleration.accel.linear.x, 0.0, 1e-6);
  EXPECT_NEAR(acceleration.accel.linear.y, -1.0, 1e-6);

  // Variances swapped, covariance negated, like the twist covariance
  EXPECT_NEAR(acceleration.covariance[cov_x_x], 4.0, covariance_tolerance);
  EXPECT_NEAR(acceleration.covariance[cov_y_y], 1.0, covariance_tolerance);
  EXPECT_NEAR(acceleration.covariance[cov_x_y], -0.5, covariance_tolerance);
  EXPECT_NEAR(acceleration.covariance[cov_y_x], -0.5, covariance_tolerance);
  EXPECT_TRUE(
    only_these_entries_set(acceleration.covariance, {cov_x_x, cov_x_y, cov_y_x, cov_y_y}));
}

// At a general yaw the acceleration and its covariance match the rotation by -yaw written out,
// like the velocity.
TEST(RadarObjectsAdapter, Tracks_Acceleration_RotatedAtGeneralYaw)
{
  // Arrange
  const RadarInfo info = make_radar_info(ars548_fields);
  RadarObject radar = facing(make_radar_object(), general_yaw);
  radar.acceleration_covariance = {2.0f, -0.3f, 9.0f, 0.5f, 9.0f, 9.0f};

  // Act
  const auto outputs = run_adapter_and_collect_outputs(info, {radar});
  ASSERT_TRUE(outputs.has_value());

  // Assert
  const auto & acceleration = outputs->tracks.objects.at(0).kinematics.acceleration_with_covariance;
  const auto [ax, ay] = into_object_frame(radar.acceleration.x, radar.acceleration.y, general_yaw);
  EXPECT_NEAR(acceleration.accel.linear.x, ax, 1e-6);
  EXPECT_NEAR(acceleration.accel.linear.y, ay, 1e-6);
  const Covariance2x2 expected = into_object_frame({2.0, -0.3, 0.5}, general_yaw);
  EXPECT_NEAR(acceleration.covariance[cov_x_x], expected.xx, covariance_tolerance);
  EXPECT_NEAR(acceleration.covariance[cov_x_y], expected.xy, covariance_tolerance);
  EXPECT_NEAR(acceleration.covariance[cov_y_x], expected.xy, covariance_tolerance);
  EXPECT_NEAR(acceleration.covariance[cov_y_y], expected.yy, covariance_tolerance);
  EXPECT_TRUE(
    only_these_entries_set(acceleration.covariance, {cov_x_x, cov_x_y, cov_y_x, cov_y_y}));
}

// Like the other z components, acceleration.z is copied from the object when the radar info
// declares acceleration_z ...
TEST(RadarObjectsAdapter, Tracks_AccelerationZ_Declared_CopiedFromObject)
{
  // Arrange
  const RadarInfo info = make_radar_info(all_fields);
  const RadarObject radar = make_radar_object();

  // Act
  const auto outputs = run_adapter_and_collect_outputs(info, {radar});
  ASSERT_TRUE(outputs.has_value());

  // Assert
  const auto & linear =
    outputs->tracks.objects.at(0).kinematics.acceleration_with_covariance.accel.linear;
  EXPECT_DOUBLE_EQ(linear.z, radar.acceleration.z);
}

// ... and filled from default_acceleration_z when it does not. An ARS548 does not declare it, so
// this is the path taken on the vehicle.
TEST(RadarObjectsAdapter, Tracks_AccelerationZ_Undeclared_FilledFromParameter)
{
  // Arrange
  const DefaultParameters defaults;
  const RadarInfo info = make_radar_info(ars548_fields);
  const RadarObject radar = make_radar_object();

  // Act
  const auto outputs = run_adapter_and_collect_outputs(info, {radar}, defaults.to_params());
  ASSERT_TRUE(outputs.has_value());

  // Assert
  const auto & linear =
    outputs->tracks.objects.at(0).kinematics.acceleration_with_covariance.accel.linear;
  EXPECT_DOUBLE_EQ(linear.z, defaults.acceleration_z);
}

// Everything a tracked object shares with a detected object is filled in identically for the
// same input: existence probability, classification, pose and twist with their covariances,
// shape, and a fully known orientation. A track is a detection plus identity, acceleration and
// the stationary flag - nothing else differs.
TEST(RadarObjectsAdapter, Tracks_SharedFields_MatchDetection)
{
  // Arrange
  const RadarInfo info = make_radar_info(ars548_fields);
  RadarObject radar = make_radar_object();
  radar.classifications = {
    make_classification(RadarClassification::CAR, 0.8f),
    make_classification(RadarClassification::HAZARD, 0.05f)};

  // Act
  const auto outputs = run_adapter_and_collect_outputs(info, {radar});
  ASSERT_TRUE(outputs.has_value());

  // Assert
  const TrackedObject & track = outputs->tracks.objects.at(0);
  const DetectedObject & detected = outputs->detections.objects.at(0);
  EXPECT_EQ(track.existence_probability, detected.existence_probability);
  EXPECT_EQ(track.classification, detected.classification);
  EXPECT_EQ(track.kinematics.pose_with_covariance, detected.kinematics.pose_with_covariance);
  EXPECT_EQ(track.kinematics.twist_with_covariance, detected.kinematics.twist_with_covariance);
  EXPECT_EQ(track.shape, detected.shape);
  EXPECT_EQ(track.kinematics.orientation_availability, TrackedObjectKinematics::AVAILABLE);
}

// ---------------------------------------------------------------------------------------------
// The remap table built from the parameter names (make_classification_remap_from_string_pair),
// on its own: what the cases above feed the adapter, read before the adapter is given it.
// ---------------------------------------------------------------------------------------------

namespace
{
using autoware::radar_objects_adapter::ClassificationRemap;

// The eight radar labels the parameters can name, with their ids.
const std::map<std::string, RadarClassification::_label_type> radar_labels = {
  {"UNKNOWN", RadarClassification::UNKNOWN}, {"CAR", RadarClassification::CAR},
  {"TRUCK", RadarClassification::TRUCK},     {"MOTORCYCLE", RadarClassification::MOTORCYCLE},
  {"BICYCLE", RadarClassification::BICYCLE}, {"PEDESTRIAN", RadarClassification::PEDESTRIAN},
  {"ANIMAL", RadarClassification::ANIMAL},   {"HAZARD", RadarClassification::HAZARD}};

// The nine perception labels a parameter value can name, with their ids.
const std::map<std::string, ObjectClassification::_label_type> perception_labels = {
  {"UNKNOWN", ObjectClassification::UNKNOWN}, {"CAR", ObjectClassification::CAR},
  {"TRUCK", ObjectClassification::TRUCK},     {"BUS", ObjectClassification::BUS},
  {"TRAILER", ObjectClassification::TRAILER}, {"MOTORCYCLE", ObjectClassification::MOTORCYCLE},
  {"BICYCLE", ObjectClassification::BICYCLE}, {"PEDESTRIAN", ObjectClassification::PEDESTRIAN},
  {"ANIMAL", ObjectClassification::ANIMAL}};
}  // namespace

// No names in, no entries out.
TEST(RadarObjectsAdapter, RemapTable_Empty_NoEntries)
{
  // Act
  const auto [remap, unknown_perception_labels] = make_classification_remap_from_string_pair({});

  // Assert
  EXPECT_TRUE(remap.empty());
  EXPECT_TRUE(unknown_perception_labels.empty());
}

// Every radar label name can be a key and every perception label name can be a value; the
// table holds the ids of both. Radar labels the parameters do not name get no entry.
TEST(RadarObjectsAdapter, RemapTable_KnownNames_MappedToIds)
{
  for (const auto & [radar_name, radar_id] : radar_labels) {
    for (const auto & [perception_name, perception_id] : perception_labels) {
      SCOPED_TRACE(radar_name + " -> " + perception_name);
      // Act
      const auto [remap, unknown_perception_labels] =
        make_classification_remap_from_string_pair({{radar_name, perception_name}});

      // Assert
      EXPECT_EQ(remap, (ClassificationRemap{{radar_id, perception_id}}));
      EXPECT_TRUE(unknown_perception_labels.empty());
    }
  }
}

// A perception label name the table does not know is not an error: the radar label maps to
// UNKNOWN, and the pair is reported so that the node can warn about it. The node's warning is
// what Classification_UnknownLabelInParameter_MapsToUnknown could not pin.
TEST(RadarObjectsAdapter, RemapTable_UnknownPerceptionName_MapsToUnknownAndReported)
{
  // Act
  const auto [remap, unknown_perception_labels] = make_classification_remap_from_string_pair(
    {{"CAR", "SPACESHIP"}, {"TRUCK", "TRUCK"}, {"BICYCLE", "bicycle"}});

  // Assert
  const ClassificationRemap expected = {
    {RadarClassification::CAR, ObjectClassification::UNKNOWN},
    {RadarClassification::TRUCK, ObjectClassification::TRUCK},
    {RadarClassification::BICYCLE, ObjectClassification::UNKNOWN}};
  EXPECT_EQ(remap, expected);
  // Reported by radar label, so that each entry is reported once
  const RemapNames reported = {{"BICYCLE", "bicycle"}, {"CAR", "SPACESHIP"}};
  EXPECT_EQ(unknown_perception_labels, reported);
}

// The radar labels the parameters can name are the eight above. Perception labels that the
// radar side also defines (BUS, TRAILER, OVER_DRIVABLE, UNDER_DRIVABLE) are not among them, and
// naming one as a key is an error rather than a silent entry: the parameter set is fixed.
TEST(RadarObjectsAdapter, RemapTable_UnknownRadarName_Throws)
{
  for (const auto & name : {"BUS", "TRAILER", "OVER_DRIVABLE", "UNDER_DRIVABLE", "car", ""}) {
    SCOPED_TRACE(name);
    // Act and assert
    EXPECT_THROW(make_classification_remap_from_string_pair({{name, "CAR"}}), std::out_of_range);
  }
}
