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
//
// Characterization tests for RadarObjectsAdapter.
//
// These tests pin the behavior of the node as it is today, before its logic is separated from the
// node class. They drive the node over its real topics: a radar info message and radar objects are
// published, and the detected and tracked objects the node publishes are checked. They are
// deliberately not exhaustive - the goal is to catch a fatal regression during the refactoring
// (does not build, does not start, publishes nothing, the conversion no longer runs), not to
// specify every corner of the node's behavior. Once the logic is covered by unit tests, this file
// is replaced by a small integration test.

#include "radar_objects_adapter.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_sensing_msgs/msg/radar_info.hpp>
#include <autoware_sensing_msgs/msg/radar_objects.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace
{
using autoware::RadarObjectsAdapter;
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

// The node fixes its own name in the constructor and resolves its topics against it.
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

  // The six parameters, in the order the node declares them.
  static const std::vector<std::string> & names()
  {
    static const std::vector<std::string> parameter_names = {
      "default_position_z", "default_velocity_z", "default_acceleration_z",
      "default_size_x",     "default_size_y",     "default_size_z"};
    return parameter_names;
  }

  // Parameter overrides for the node under test, with all six parameters set.
  [[nodiscard]] rclcpp::NodeOptions to_options() const { return to_options_without(""); }

  // Parameter overrides with one parameter left unset.
  [[nodiscard]] rclcpp::NodeOptions to_options_without(const std::string & omitted) const
  {
    rclcpp::NodeOptions options;
    const auto add = [&](const std::string & name, double value) {
      if (name != omitted) {
        options.append_parameter_override(name, value);
      }
    };
    add("default_position_z", position_z);
    add("default_velocity_z", velocity_z);
    add("default_acceleration_z", acceleration_z);
    add("default_size_x", size_x);
    add("default_size_y", size_y);
    add("default_size_z", size_z);
    return options;
  }
};

// A radar info message that declares exactly these object fields. Only the names matter to the
// node; the min/max/resolution of each field are left unset.
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

// The eight object fields the node insists on. Without any one of them it reports the radar as
// incompatible and converts nothing.
const std::set<std::string> required_fields = {
  "existence_probability", "position_x",     "position_y", "velocity_x", "velocity_y",
  "acceleration_x",        "acceleration_y", "orientation"};

// The object fields a Continental ARS548 declares through its nebula driver (read from
// continental_ars548_decoder_wrapper.cpp on 2026-09-17). velocity_z, acceleration_z and size_z are
// not among them, so on the vehicle those three come from the parameters.
const std::set<std::string> ars548_fields = with(
  required_fields,
  {"object_id", "age", "measurement_status", "movement_status", "position_z", "size_x", "size_y",
   "orientation_std", "orientation_rate", "orientation_rate_std"});

// Every field the node looks at, so that nothing falls back to a parameter.
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

// Stamps are deliberately constant: the node copies the input header into both outputs, so a
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
// node applies come out as a swap of components - checkable by hand.
constexpr double quarter_turn = 1.5707963267948966;

RadarObject facing(RadarObject object, double yaw)
{
  object.orientation = static_cast<float>(yaw);
  return object;
}

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

}  // namespace

// Drives the node over its real topics from the test thread. There is no background spin: the
// executor holding both the node and its peer is pumped only from here, so the steps of a test
// reach the node in the order written.
class RadarObjectsAdapterCharacterization : public ::testing::Test
{
protected:
  // rclcpp::init() may only be called once per process, so it is done per suite rather than per
  // test. The node itself is recreated for every test, because its parameters are read once in
  // the constructor.
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void TearDown() override
  {
    executor_.reset();
    tracks_sub_.reset();
    detections_sub_.reset();
    radar_info_pub_.reset();
    objects_pub_.reset();
    peer_.reset();
    node_.reset();
  }

  // Time given to pub/sub discovery between the peer and the node.
  static constexpr auto discovery_budget = std::chrono::milliseconds(5000);
  // Time a published message gets to reach the node. The tests that assert that nothing comes
  // back always wait it out, so it also sets how long those tests take.
  static constexpr auto delivery_budget = std::chrono::milliseconds(500);
  // Time given to the node to publish its outputs after a radar objects message.
  static constexpr auto output_budget = std::chrono::milliseconds(3000);

  void start_node(const rclcpp::NodeOptions & options = DefaultParameters{}.to_options())
  {
    node_ = std::make_shared<RadarObjectsAdapter>(options);
    peer_ = std::make_shared<rclcpp::Node>("characterization_peer");

    // The node subscribes to both inputs with sensor data QoS.
    objects_pub_ =
      peer_->create_publisher<RadarObjects>(node_topic("input/objects"), rclcpp::SensorDataQoS());
    radar_info_pub_ =
      peer_->create_publisher<RadarInfo>(node_topic("input/radar_info"), rclcpp::SensorDataQoS());
    detections_sub_ = peer_->create_subscription<DetectedObjects>(
      node_topic("output/detections"), rclcpp::QoS{10},
      [this](DetectedObjects::ConstSharedPtr message) { detections_.push_back(message); });
    tracks_sub_ = peer_->create_subscription<TrackedObjects>(
      node_topic("output/tracks"), rclcpp::QoS{10},
      [this](TrackedObjects::ConstSharedPtr message) { tracks_.push_back(message); });

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    // The node derives from autoware::agnocast_wrapper::Node, not from rclcpp::Node, so it joins
    // the executor through its base interface.
    executor_->add_node(node_->get_node_base_interface());
    executor_->add_node(peer_);
  }

  void pump(std::chrono::milliseconds duration)
  {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_->spin_some(std::chrono::milliseconds(10));
    }
  }

  template <typename Predicate>
  bool pump_until(Predicate done, std::chrono::milliseconds budget)
  {
    const auto deadline = std::chrono::steady_clock::now() + budget;
    while (!done() && std::chrono::steady_clock::now() < deadline) {
      executor_->spin_some(std::chrono::milliseconds(10));
    }
    return done();
  }

  // Both inputs subscribed by the node and both outputs advertised to the peer.
  bool wait_for_discovery()
  {
    return pump_until(
      [this] {
        return objects_pub_->get_subscription_count() > 0 &&
               radar_info_pub_->get_subscription_count() > 0 &&
               detections_sub_->get_publisher_count() > 0 && tracks_sub_->get_publisher_count() > 0;
      },
      discovery_budget);
  }

  // Publishes a radar info message and gives it time to arrive. The node does not acknowledge
  // it; what it made of the message shows only in what it does with the next radar objects.
  void send_radar_info(const RadarInfo & info)
  {
    radar_info_pub_->publish(info);
    pump(delivery_budget);
  }

  // Publishes radar objects and waits until both outputs have grown by one message.
  bool send_objects_and_wait_for_outputs(const RadarObjects & objects)
  {
    const size_t detections_before = detections_.size();
    const size_t tracks_before = tracks_.size();
    objects_pub_->publish(objects);
    return pump_until(
      [&] { return detections_.size() > detections_before && tracks_.size() > tracks_before; },
      output_budget);
  }

  // Publishes radar objects that are expected to produce nothing, and waits the delivery budget
  // out so that a publication would have been seen.
  void send_objects_expecting_no_output(const RadarObjects & objects)
  {
    objects_pub_->publish(objects);
    pump(delivery_budget);
  }

  // What the node published for one radar objects message.
  struct Outputs
  {
    DetectedObjects detections;
    TrackedObjects tracks;
  };

  // Starts the node with `options`, opens the gate with `info`, publishes `objects` once and
  // collects what came out on both outputs - or nothing, if the node did not publish in time.
  std::optional<Outputs> run_node_and_collect_outputs(
    const RadarInfo & info, const std::vector<RadarObject> & objects,
    const rclcpp::NodeOptions & options = DefaultParameters{}.to_options())
  {
    start_node(options);
    if (!wait_for_discovery()) {
      return std::nullopt;
    }
    send_radar_info(info);
    if (!send_objects_and_wait_for_outputs(make_radar_objects(objects))) {
      return std::nullopt;
    }
    return Outputs{*detections_.back(), *tracks_.back()};
  }

  // Parameter overrides that remap the given radar labels, on top of the six required parameters.
  static rclcpp::NodeOptions remap_options(
    const std::vector<std::pair<std::string, std::string>> & radar_to_perception_labels)
  {
    rclcpp::NodeOptions options = DefaultParameters{}.to_options();
    for (const auto & [radar_label, perception_label] : radar_to_perception_labels) {
      options.append_parameter_override("classification_remap." + radar_label, perception_label);
    }
    return options;
  }

  std::shared_ptr<RadarObjectsAdapter> node_;
  std::shared_ptr<rclcpp::Node> peer_;
  rclcpp::Publisher<RadarObjects>::SharedPtr objects_pub_;
  rclcpp::Publisher<RadarInfo>::SharedPtr radar_info_pub_;
  rclcpp::Subscription<DetectedObjects>::SharedPtr detections_sub_;
  rclcpp::Subscription<TrackedObjects>::SharedPtr tracks_sub_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::vector<DetectedObjects::ConstSharedPtr> detections_;
  std::vector<TrackedObjects::ConstSharedPtr> tracks_;
};

// The six default_* parameters are declared without a default value, so leaving out any one of
// them alone keeps the node from starting. This is what makes every test below pass all six.
//
// The exception type is not pinned: it comes from rclcpp, not from this node.
TEST_F(RadarObjectsAdapterCharacterization, Construct_DefaultParameterMissing_Throws)
{
  for (const auto & name : DefaultParameters::names()) {
    SCOPED_TRACE(name);
    // Arrange
    const rclcpp::NodeOptions options = DefaultParameters{}.to_options_without(name);

    // Act and assert
    EXPECT_THROW(std::make_shared<RadarObjectsAdapter>(options), std::exception);
  }
}

// The classification_remap.* parameters, on the other hand, all have a default in the code, so
// the six default_* parameters alone are enough to construct the node. What those built-in
// defaults map to is pinned with the classification tests.
TEST_F(RadarObjectsAdapterCharacterization, Construct_RemapParametersOmitted_Constructs)
{
  // Arrange
  const rclcpp::NodeOptions options = DefaultParameters{}.to_options();

  // Act and assert
  EXPECT_NO_THROW(std::make_shared<RadarObjectsAdapter>(options));
}

// ---------------------------------------------------------------------------------------------
// The radar info gate. Radar objects are converted only after a radar info message has declared
// all of the fields the node requires; until then every radar objects message is dropped with a
// throttled warning.
// ---------------------------------------------------------------------------------------------

// On the vehicle the radar info arrives later than the first radar objects: the driver publishes
// it only every tenth objects message. Until it arrives the node does not know which fields the
// radar provides, and it converts nothing rather than guess. The objects dropped in the meantime
// are gone for good - nothing is queued and replayed once the radar info is in - and only the
// objects that arrive after it are converted.
//
// The warning logged while the gate is closed is not pinned.
TEST_F(RadarObjectsAdapterCharacterization, Gate_ObjectsBeforeRadarInfo_DroppedNotReplayed)
{
  // Arrange
  const RadarObjects before_info = make_radar_objects({make_radar_object()}, first_stamp);
  const RadarInfo info = make_radar_info(ars548_fields);
  const RadarObjects after_info = make_radar_objects({make_radar_object()}, second_stamp);
  start_node();
  ASSERT_TRUE(wait_for_discovery());

  // Act: radar objects before any radar info ...
  send_objects_expecting_no_output(before_info);

  // Assert: ... produce nothing
  EXPECT_TRUE(detections_.empty());
  EXPECT_TRUE(tracks_.empty());

  // Act: the radar info arrives, then more radar objects
  send_radar_info(info);
  ASSERT_TRUE(send_objects_and_wait_for_outputs(after_info));

  // Assert: only the message published after the radar info came through
  ASSERT_EQ(detections_.size(), 1u);
  EXPECT_EQ(detections_[0]->header.stamp, second_stamp);
  ASSERT_EQ(tracks_.size(), 1u);
  EXPECT_EQ(tracks_[0]->header.stamp, second_stamp);
}

// A radar info that lacks one of the required fields (here: orientation) marks the radar as
// incompatible, and radar objects keep being dropped.
//
// Which field is missing makes no difference to the node, so only one is tried.
//
// What this case tells apart is a gate that looks at the content of a radar info from one that
// opens on any radar info. Whether the node rejected the message or merely ignored it is not
// observable over the topics, and neither is which of the eight fields is required; both belong
// to the unit tests of the separated logic.
TEST_F(RadarObjectsAdapterCharacterization, Gate_RadarInfoMissingRequiredField_ObjectsDropped)
{
  // Arrange: the only radar info the node has seen lacks a required field
  const RadarInfo incomplete_info = make_radar_info(without(required_fields, "orientation"));
  const RadarObjects objects = make_radar_objects({make_radar_object()});
  start_node();
  ASSERT_TRUE(wait_for_discovery());
  send_radar_info(incomplete_info);

  // Act
  send_objects_expecting_no_output(objects);

  // Assert
  EXPECT_TRUE(detections_.empty());
  EXPECT_TRUE(tracks_.empty());
}

// Once a radar info declares all required fields, every radar objects message produces one
// detected objects message and one tracked objects message, both carrying the input header.
//
// The contents of the objects are pinned by the conversion tests, not here.
TEST_F(RadarObjectsAdapterCharacterization, Gate_ValidRadarInfo_ObjectsConverted)
{
  // Arrange: the node has seen a radar info that declares every required field
  const RadarInfo info = make_radar_info(ars548_fields);
  const RadarObjects objects = make_radar_objects({make_radar_object()});
  start_node();
  ASSERT_TRUE(wait_for_discovery());
  send_radar_info(info);

  // Act
  ASSERT_TRUE(send_objects_and_wait_for_outputs(objects));

  // Assert
  ASSERT_EQ(detections_.size(), 1u);
  EXPECT_EQ(detections_[0]->header.stamp, first_stamp);
  EXPECT_EQ(detections_[0]->header.frame_id, "base_link");
  EXPECT_EQ(detections_[0]->objects.size(), 1u);

  ASSERT_EQ(tracks_.size(), 1u);
  EXPECT_EQ(tracks_[0]->header.stamp, first_stamp);
  EXPECT_EQ(tracks_[0]->header.frame_id, "base_link");
  EXPECT_EQ(tracks_[0]->objects.size(), 1u);
}

// ---------------------------------------------------------------------------------------------
// Conversion into detected objects. A radar object is converted into a detected object with the
// fields that are explicitly declared in a preceding radar info. Each test case evaluates the
// converted outcome against a radar object with some specific aspects. The tracked object made of
// the same input is covered further down.
//
// Every case follows the same shape: the radar info and the radar objects are prepared first,
// the node is run on them once, and the output is read.
// ---------------------------------------------------------------------------------------------

// A radar objects message with no objects still produces one detected objects message and one
// tracked objects message, each empty and carrying the input header. This keeps the outputs alive
// while the radar sees nothing, and it is the minimal behavior an integration test can check
// without knowing anything about the conversion.
TEST_F(RadarObjectsAdapterCharacterization, Conversion_EmptyObjects_PublishesEmptyOutputs)
{
  // Arrange
  const RadarInfo info = make_radar_info(ars548_fields);

  // Act
  const auto outputs = run_node_and_collect_outputs(info, {});
  ASSERT_TRUE(outputs.has_value());

  // Assert
  EXPECT_EQ(outputs->detections.header.stamp, first_stamp);
  EXPECT_TRUE(outputs->detections.objects.empty());
  EXPECT_EQ(outputs->tracks.header.stamp, first_stamp);
  EXPECT_TRUE(outputs->tracks.objects.empty());
}

// When the radar info declares a field, the node copies it from the radar object as it is - no
// coordinate transform, no scaling. Every field the radar info can declare is declared here, so
// nothing falls back to a parameter.
//
// The velocity's x and y are not copied but rotated, and are pinned separately.
TEST_F(RadarObjectsAdapterCharacterization, Detections_DeclaredFields_CopiedFromObject)
{
  // Arrange
  const RadarInfo info = make_radar_info(all_fields);
  const RadarObject radar = make_radar_object();

  // Act
  const auto outputs = run_node_and_collect_outputs(info, {radar});
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

// When the radar info does not declare a field, the node ignores the value in the radar object
// and fills the field from the parameter. This is the situation on the vehicle for velocity_z,
// acceleration_z and size_z; here every optional field is left undeclared so that every fallback
// is seen at once. The yaw variances stay zero because the orientation standard deviations are
// undeclared as well.
TEST_F(RadarObjectsAdapterCharacterization, Detections_UndeclaredFields_FilledFromParameters)
{
  // Arrange
  const DefaultParameters defaults;
  const RadarInfo info = make_radar_info(required_fields);
  const RadarObject radar = make_radar_object();

  // Act
  const auto outputs = run_node_and_collect_outputs(info, {radar}, defaults.to_options());
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

// The radar's yaw angle becomes the orientation quaternion: a rotation about z alone.
TEST_F(RadarObjectsAdapterCharacterization, Detections_Orientation_QuaternionFromYaw)
{
  // Arrange
  const RadarInfo info = make_radar_info(ars548_fields);
  const RadarObject rotated_object = facing(make_radar_object(), 0.3);

  // Act
  const auto outputs = run_node_and_collect_outputs(info, {rotated_object});
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
TEST_F(RadarObjectsAdapterCharacterization, Detections_KinematicsFlags_SetWithOnlyRequiredFields)
{
  // Arrange
  const RadarInfo info = make_radar_info(required_fields);
  const RadarObject radar = make_radar_object();

  // Act
  const auto outputs = run_node_and_collect_outputs(info, {radar});
  ASSERT_TRUE(outputs.has_value());

  // Assert
  expect_all_kinematics_flags_set(outputs->detections.objects.at(0).kinematics);
}

// With every optional field declared, the same flags are set.
TEST_F(RadarObjectsAdapterCharacterization, Detections_KinematicsFlags_SetWithAllFields)
{
  // Arrange
  const RadarInfo info = make_radar_info(all_fields);
  const RadarObject radar = make_radar_object();

  // Act
  const auto outputs = run_node_and_collect_outputs(info, {radar});
  ASSERT_TRUE(outputs.has_value());

  // Assert
  expect_all_kinematics_flags_set(outputs->detections.objects.at(0).kinematics);
}

// The radar reports velocity in its own frame; the detected object's twist is expressed in the
// object's frame, so the node rotates the velocity by the object's yaw. With a quarter turn, a
// velocity along the radar's x axis becomes a velocity along the object's negative y axis.
//
// Since ars548_fields does not have velocity_z, radar.velocity.z is ignored; the z component is
// not part of the rotation, and where it comes from is pinned by the declared/undeclared cases.
TEST_F(RadarObjectsAdapterCharacterization, Detections_Twist_VelocityRotatedIntoObjectFrame)
{
  // Arrange
  const RadarInfo info = make_radar_info(ars548_fields);
  RadarObject radar = facing(make_radar_object(), quarter_turn);
  radar.velocity.x = 1.0;
  radar.velocity.y = 0.0;

  // Act
  const auto outputs = run_node_and_collect_outputs(info, {radar});
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
TEST_F(
  RadarObjectsAdapterCharacterization,
  Detections_PoseCovariance_CopiedWithoutRotationWithYawVariance)
{
  // Arrange
  const RadarInfo info = make_radar_info(ars548_fields);
  const RadarObject radar = facing(make_radar_object(), quarter_turn);

  // Act
  const auto outputs = run_node_and_collect_outputs(info, {radar});
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
TEST_F(RadarObjectsAdapterCharacterization, Detections_TwistCovariance_RotatedByYaw)
{
  // Arrange
  const RadarInfo info = make_radar_info(ars548_fields);
  RadarObject radar = facing(make_radar_object(), quarter_turn);
  radar.velocity_covariance = {1.0f, 0.5f, 0.0f, 4.0f, 0.0f, 0.0f};

  // Act
  const auto outputs = run_node_and_collect_outputs(info, {radar});
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

// A radar marks a covariance entry it cannot provide with INVALID_COV_VALUE (-1). The node turns
// those into zero instead of passing a negative variance downstream. The yaw variances are not
// affected, because they come from the standard deviations, not from these arrays.
TEST_F(RadarObjectsAdapterCharacterization, Detections_InvalidCovariance_MaskedToZero)
{
  // Arrange
  const RadarInfo info = make_radar_info(ars548_fields);
  RadarObject radar = make_radar_object();
  radar.position_covariance.fill(RadarObject::INVALID_COV_VALUE);
  radar.velocity_covariance.fill(RadarObject::INVALID_COV_VALUE);

  // Act
  const auto outputs = run_node_and_collect_outputs(info, {radar});
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

// Without any classification_remap.* parameter, the built-in table keeps every label as it is
// except HAZARD, which the perception pipeline does not use and which becomes UNKNOWN. The
// order of the list and the probabilities are preserved.
//
// This is the built-in default, not the shipped configuration: the parameter file in config/
// additionally remaps MOTORCYCLE and BICYCLE, which the next test covers.
TEST_F(RadarObjectsAdapterCharacterization, Classification_BuiltInRemap_LabelsKeptExceptHazard)
{
  // Arrange
  const RadarInfo info = make_radar_info(ars548_fields);
  RadarObject radar = make_radar_object();
  radar.classifications = {
    make_classification(RadarClassification::CAR, 0.8f),
    make_classification(RadarClassification::TRUCK, 0.1f),
    make_classification(RadarClassification::MOTORCYCLE, 0.3f),
    make_classification(RadarClassification::HAZARD, 0.05f),
    make_classification(RadarClassification::PEDESTRIAN, 0.02f)};

  // Act
  const auto outputs = run_node_and_collect_outputs(info, {radar});
  ASSERT_TRUE(outputs.has_value());

  // Assert
  const std::vector<LabeledProbability> expected = {
    {ObjectClassification::CAR, 0.8f},
    {ObjectClassification::TRUCK, 0.1f},
    {ObjectClassification::MOTORCYCLE, 0.3f},
    {ObjectClassification::UNKNOWN, 0.05f},
    {ObjectClassification::PEDESTRIAN, 0.02f}};
  EXPECT_EQ(labeled_probabilities(outputs->detections.objects.at(0).classification), expected);
}

// The shipped parameter file remaps MOTORCYCLE and BICYCLE to CAR, because a radar tends to
// report a far car as a two-wheeler. The remap works entry by entry: an object that carries a
// CAR, a MOTORCYCLE and a BICYCLE probability comes out with three CAR entries, each with its own
// probability, while labels the remap does not mention (TRUCK, PEDESTRIAN) stay as they are.
//
// NOTE(characterization): the duplicate labels are pinned as they are. The README notes them as
// harmless for the current consumers; merging the probabilities is a candidate change for later.
TEST_F(RadarObjectsAdapterCharacterization, Classification_ConfiguredRemap_TwoWheelersBecomeCar)
{
  // Arrange
  const RadarInfo info = make_radar_info(ars548_fields);
  const rclcpp::NodeOptions options = remap_options({{"MOTORCYCLE", "CAR"}, {"BICYCLE", "CAR"}});
  RadarObject radar = make_radar_object();
  radar.classifications = {
    make_classification(RadarClassification::CAR, 0.5f),
    make_classification(RadarClassification::TRUCK, 0.1f),
    make_classification(RadarClassification::MOTORCYCLE, 0.8f),
    make_classification(RadarClassification::BICYCLE, 0.3f),
    make_classification(RadarClassification::PEDESTRIAN, 0.02f)};

  // Act
  const auto outputs = run_node_and_collect_outputs(info, {radar}, options);
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
TEST_F(RadarObjectsAdapterCharacterization, Classification_LabelWithoutRemapEntry_BecomesUnknown)
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
  const auto outputs = run_node_and_collect_outputs(info, {radar});
  ASSERT_TRUE(outputs.has_value());

  // Assert
  const std::vector<LabeledProbability> expected = {
    {ObjectClassification::UNKNOWN, 0.7f},
    {ObjectClassification::UNKNOWN, 0.2f},
    {ObjectClassification::UNKNOWN, 0.1f},
    {ObjectClassification::UNKNOWN, 0.05f}};
  EXPECT_EQ(labeled_probabilities(outputs->detections.objects.at(0).classification), expected);
}

// A classification_remap.* value that is not a perception label name is not an error: the node
// starts and maps that radar label to UNKNOWN.
//
// The warning the node logs for such a value is not pinned.
TEST_F(RadarObjectsAdapterCharacterization, Classification_UnknownLabelInParameter_MapsToUnknown)
{
  // Arrange
  const RadarInfo info = make_radar_info(ars548_fields);
  const rclcpp::NodeOptions options = remap_options({{"CAR", "SPACESHIP"}});
  RadarObject radar = make_radar_object();
  radar.classifications = {make_classification(RadarClassification::CAR, 0.8f)};

  // Act
  const auto outputs = run_node_and_collect_outputs(info, {radar}, options);
  ASSERT_TRUE(outputs.has_value());

  // Assert
  const std::vector<LabeledProbability> expected = {{ObjectClassification::UNKNOWN, 0.8f}};
  EXPECT_EQ(labeled_probabilities(outputs->detections.objects.at(0).classification), expected);
}

// ---------------------------------------------------------------------------------------------
// Conversion into tracked objects. The same radar object goes in, and the tracked object made of
// it is read. It shares pose, twist, shape and classification with the detected object, and adds
// what a track has over a detection: an identity, an acceleration and a stationary flag.
// ---------------------------------------------------------------------------------------------

// The 16-byte UUID is assembled from the radar's 32-bit object id, least significant byte first,
// followed by 8 bytes of a hash of the fully qualified input topic name, again least significant
// byte first, followed by 4 zero bytes. The hash keeps the ids of two radars apart when their
// tracks are merged downstream, and it is the same for every object of one node.
//
// Both the node and this test take the bytes out of the integers with shifts, so the layout is
// pinned independently of the byte order of the machine. The hash is std::hash<std::string> of
// the topic name, so it is recomputed here rather than written down: its value depends on the
// standard library.
TEST_F(RadarObjectsAdapterCharacterization, Tracks_Uuid_ObjectIdThenTopicHashThenZeros)
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
  const auto outputs = run_node_and_collect_outputs(info, {first, second});
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

  // Everything after the object id is shared by every object of this node
  EXPECT_TRUE(
    std::equal(
      first_uuid.begin() + object_id_bytes, first_uuid.end(),
      second_uuid.begin() + object_id_bytes));
}

// Only an object the radar reports as DYNAMIC is moving; STATIC, but also INVALID and UNKNOWN,
// are reported as stationary.
TEST_F(RadarObjectsAdapterCharacterization, Tracks_MovementStatus_OnlyDynamicIsMoving)
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
  const auto outputs = run_node_and_collect_outputs(info, radars);
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
TEST_F(RadarObjectsAdapterCharacterization, Tracks_Acceleration_RotatedByYaw)
{
  // Arrange
  const RadarInfo info = make_radar_info(ars548_fields);
  RadarObject radar = facing(make_radar_object(), quarter_turn);
  radar.acceleration.x = 1.0;
  radar.acceleration.y = 0.0;
  radar.acceleration_covariance = {1.0f, 0.5f, 0.0f, 4.0f, 0.0f, 0.0f};

  // Act
  const auto outputs = run_node_and_collect_outputs(info, {radar});
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

// Like the other z components, acceleration.z is copied from the object when the radar info
// declares acceleration_z ...
TEST_F(RadarObjectsAdapterCharacterization, Tracks_AccelerationZ_Declared_CopiedFromObject)
{
  // Arrange
  const RadarInfo info = make_radar_info(all_fields);
  const RadarObject radar = make_radar_object();

  // Act
  const auto outputs = run_node_and_collect_outputs(info, {radar});
  ASSERT_TRUE(outputs.has_value());

  // Assert
  const auto & linear =
    outputs->tracks.objects.at(0).kinematics.acceleration_with_covariance.accel.linear;
  EXPECT_DOUBLE_EQ(linear.z, radar.acceleration.z);
}

// ... and filled from default_acceleration_z when it does not. An ARS548 does not declare it, so
// this is the path taken on the vehicle.
TEST_F(RadarObjectsAdapterCharacterization, Tracks_AccelerationZ_Undeclared_FilledFromParameter)
{
  // Arrange
  const DefaultParameters defaults;
  const RadarInfo info = make_radar_info(ars548_fields);
  const RadarObject radar = make_radar_object();

  // Act
  const auto outputs = run_node_and_collect_outputs(info, {radar}, defaults.to_options());
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
TEST_F(RadarObjectsAdapterCharacterization, Tracks_SharedFields_MatchDetection)
{
  // Arrange
  const RadarInfo info = make_radar_info(ars548_fields);
  RadarObject radar = make_radar_object();
  radar.classifications = {
    make_classification(RadarClassification::CAR, 0.8f),
    make_classification(RadarClassification::HAZARD, 0.05f)};

  // Act
  const auto outputs = run_node_and_collect_outputs(info, {radar});
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
