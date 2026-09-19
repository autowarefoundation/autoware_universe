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

#include <chrono>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace
{
using autoware::RadarObjectsAdapter;
using autoware_perception_msgs::msg::DetectedObjects;
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
    EXPECT_THROW(
      std::make_shared<RadarObjectsAdapter>(DefaultParameters{}.to_options_without(name)),
      std::exception);
  }
}

// The classification_remap.* parameters, on the other hand, all have a default in the code, so
// the six default_* parameters alone are enough to construct the node. What those built-in
// defaults map to is pinned with the classification tests.
TEST_F(RadarObjectsAdapterCharacterization, Construct_RemapParametersOmitted_Constructs)
{
  EXPECT_NO_THROW(std::make_shared<RadarObjectsAdapter>(DefaultParameters{}.to_options()));
}

// The node subscribes to its two inputs and advertises its two outputs right away, under the
// names and message types below. There is no lazy subscription: the inputs are subscribed whether
// or not anything listens to the outputs.
TEST_F(RadarObjectsAdapterCharacterization, Interface_Startup_SubscribesInputsAdvertisesOutputs)
{
  start_node();

  ASSERT_TRUE(wait_for_discovery());

  const auto objects_subs = peer_->get_subscriptions_info_by_topic(node_topic("input/objects"));
  ASSERT_EQ(objects_subs.size(), 1u);
  EXPECT_EQ(objects_subs[0].node_name(), node_name);
  EXPECT_EQ(objects_subs[0].topic_type(), "autoware_sensing_msgs/msg/RadarObjects");

  const auto info_subs = peer_->get_subscriptions_info_by_topic(node_topic("input/radar_info"));
  ASSERT_EQ(info_subs.size(), 1u);
  EXPECT_EQ(info_subs[0].node_name(), node_name);
  EXPECT_EQ(info_subs[0].topic_type(), "autoware_sensing_msgs/msg/RadarInfo");

  const auto detections_pubs = peer_->get_publishers_info_by_topic(node_topic("output/detections"));
  ASSERT_EQ(detections_pubs.size(), 1u);
  EXPECT_EQ(detections_pubs[0].node_name(), node_name);
  EXPECT_EQ(detections_pubs[0].topic_type(), "autoware_perception_msgs/msg/DetectedObjects");

  const auto tracks_pubs = peer_->get_publishers_info_by_topic(node_topic("output/tracks"));
  ASSERT_EQ(tracks_pubs.size(), 1u);
  EXPECT_EQ(tracks_pubs[0].node_name(), node_name);
  EXPECT_EQ(tracks_pubs[0].topic_type(), "autoware_perception_msgs/msg/TrackedObjects");
}

// The inputs are taken with sensor data QoS (best effort), while both outputs are published
// reliable and transient local, so a subscriber that joins late still receives the most recent
// messages. Downstream nodes that subscribe best effort or volatile stay compatible with that.
//
// The history depth of the outputs (10) is not pinned: the graph does not report it reliably.
TEST_F(RadarObjectsAdapterCharacterization, Interface_Qos_BestEffortInputsLatchedOutputs)
{
  start_node();
  ASSERT_TRUE(wait_for_discovery());

  const auto objects_qos =
    peer_->get_subscriptions_info_by_topic(node_topic("input/objects")).at(0).qos_profile();
  EXPECT_EQ(objects_qos.reliability(), rclcpp::ReliabilityPolicy::BestEffort);

  const auto info_qos =
    peer_->get_subscriptions_info_by_topic(node_topic("input/radar_info")).at(0).qos_profile();
  EXPECT_EQ(info_qos.reliability(), rclcpp::ReliabilityPolicy::BestEffort);

  const auto detections_qos =
    peer_->get_publishers_info_by_topic(node_topic("output/detections")).at(0).qos_profile();
  EXPECT_EQ(detections_qos.reliability(), rclcpp::ReliabilityPolicy::Reliable);
  EXPECT_EQ(detections_qos.durability(), rclcpp::DurabilityPolicy::TransientLocal);

  const auto tracks_qos =
    peer_->get_publishers_info_by_topic(node_topic("output/tracks")).at(0).qos_profile();
  EXPECT_EQ(tracks_qos.reliability(), rclcpp::ReliabilityPolicy::Reliable);
  EXPECT_EQ(tracks_qos.durability(), rclcpp::DurabilityPolicy::TransientLocal);
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
  start_node();
  ASSERT_TRUE(wait_for_discovery());

  send_objects_expecting_no_output(make_radar_objects({make_radar_object()}, first_stamp));
  EXPECT_TRUE(detections_.empty());
  EXPECT_TRUE(tracks_.empty());

  send_radar_info(make_radar_info(ars548_fields));
  ASSERT_TRUE(
    send_objects_and_wait_for_outputs(make_radar_objects({make_radar_object()}, second_stamp)));

  // Only the message published after the radar info came through
  ASSERT_EQ(detections_.size(), 1u);
  EXPECT_EQ(detections_[0]->header.stamp, second_stamp);
  ASSERT_EQ(tracks_.size(), 1u);
  EXPECT_EQ(tracks_[0]->header.stamp, second_stamp);
}

// A radar info that lacks one of the required fields (here: orientation) marks the radar as
// incompatible, and radar objects keep being dropped.
//
// Which field is missing makes no difference to the node, so only one is tried.
TEST_F(RadarObjectsAdapterCharacterization, Gate_RadarInfoMissingRequiredField_ObjectsDropped)
{
  start_node();
  ASSERT_TRUE(wait_for_discovery());
  send_radar_info(make_radar_info(without(required_fields, "orientation")));

  send_objects_expecting_no_output(make_radar_objects({make_radar_object()}));

  EXPECT_TRUE(detections_.empty());
  EXPECT_TRUE(tracks_.empty());
}

// Once a radar info declares all required fields, every radar objects message produces one
// detected objects message and one tracked objects message, both carrying the input header.
//
// The contents of the objects are pinned by the conversion tests, not here.
TEST_F(RadarObjectsAdapterCharacterization, Gate_ValidRadarInfo_ObjectsConverted)
{
  start_node();
  ASSERT_TRUE(wait_for_discovery());
  send_radar_info(make_radar_info(ars548_fields));

  ASSERT_TRUE(send_objects_and_wait_for_outputs(make_radar_objects({make_radar_object()})));

  ASSERT_EQ(detections_.size(), 1u);
  EXPECT_EQ(detections_[0]->header.stamp, first_stamp);
  EXPECT_EQ(detections_[0]->header.frame_id, "base_link");
  EXPECT_EQ(detections_[0]->objects.size(), 1u);

  ASSERT_EQ(tracks_.size(), 1u);
  EXPECT_EQ(tracks_[0]->header.stamp, first_stamp);
  EXPECT_EQ(tracks_[0]->header.frame_id, "base_link");
  EXPECT_EQ(tracks_[0]->objects.size(), 1u);
}
