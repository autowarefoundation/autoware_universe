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
#include <string>
#include <vector>

namespace
{
using autoware::RadarObjectsAdapter;
using autoware_perception_msgs::msg::DetectedObjects;
using autoware_perception_msgs::msg::TrackedObjects;
using autoware_sensing_msgs::msg::RadarInfo;
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
