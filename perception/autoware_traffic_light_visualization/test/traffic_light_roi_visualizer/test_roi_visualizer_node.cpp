// Copyright 2026 Tier IV, Inc.
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
// Integration tests for TrafficLightRoiVisualizerNode.
//
// The drawing is covered by test_roi_visualizer.cpp, which calls it directly. What is left for
// the node is everything a unit test cannot reach, because it is a property of the node and not
// of the drawing: that it starts, or refuses to; that it subscribes to its inputs only while
// something watches its output, and picks the fourth input from a parameter; that the
// synchronizer will not fire while one of its topics is silent; and that either publisher
// delivers.
//
// Four cases drive the node end to end, each synchronizer against each publisher, to show that a
// synchronized set of messages reaches the drawing and comes back out. All four assert the same
// minimum: the output is RGB8, and one pixel carries the signal color. That something was drawn
// is the point; what was drawn is the unit tests' job.
//
// Every case that publishes anything arranges the same way, so the per-case Arrange says nothing
// about it: start the node, subscribe to its output, and wait until it has subscribed to its
// inputs. The last step is not optional - the node unsubscribes while nothing watches its output,
// so anything published before it has subscribed is simply not received.

#include "traffic_light_roi_visualizer/roi_visualizer_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <tier4_perception_msgs/msg/traffic_light_array.hpp>
#include <tier4_perception_msgs/msg/traffic_light_roi_array.hpp>

#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace
{
using autoware::traffic_light::TrafficLightRoiVisualizerNode;
using sensor_msgs::msg::Image;
using tier4_perception_msgs::msg::TrafficLight;
using tier4_perception_msgs::msg::TrafficLightArray;
using tier4_perception_msgs::msg::TrafficLightElement;
using tier4_perception_msgs::msg::TrafficLightRoi;
using tier4_perception_msgs::msg::TrafficLightRoiArray;

// One pixel's three channel values. The order depends on the image encoding, so the names of
// the constants and parameters below say which order they are in.
using Pixel = std::array<uint8_t, 3>;

// The node resolves its topics against its own name, which is fixed in the constructor.
constexpr char node_namespace[] = "/traffic_light_roi_visualizer_node";

std::string node_topic(const std::string & relative_name)
{
  return std::string(node_namespace) + "/" + relative_name;
}

// The image is large enough for the label box that is drawn above a ROI (about 96 x 27 px for a
// single shape): draw_shape() returns early when that box would fall outside the image.
constexpr int image_width = 640;
constexpr int image_height = 480;
constexpr uint8_t background_level = 40;

constexpr int64_t signal_id = 42;

struct Box
{
  int x;
  int y;
  int width;
  int height;
};

// A fine ROI (~/input/rois) and a rough ROI (~/input/rough/rois) that encloses it. Each corner
// checked in the high accuracy test lies on exactly one of the two rectangles, which is what lets
// that test tell them apart. Both are far enough from the top edge for the label box to fit.
constexpr Box fine_box{200, 150, 40, 90};
constexpr Box rough_box{190, 140, 60, 110};

// Golden colors, recorded from this node on 2026-09-10 (ROS 2 Jazzy, Ubuntu 24.04, OpenCV 4.6).
// The published image is RGB8, so the components are red, green, blue in that order.
constexpr Pixel background_rgb{background_level, background_level, background_level};
constexpr Pixel green_signal_rgb{149, 254, 161};  // str_to_color("green")

// `pixel` fills every pixel of the image and is in the channel order implied by `encoding`, not
// necessarily RGB. The stamp is left unset: the fixture stamps a message just before publishing it.
Image make_uniform_image(const std::string & encoding, const Pixel & pixel)
{
  Image image;
  image.header.frame_id = "camera";
  image.width = image_width;
  image.height = image_height;
  image.encoding = encoding;
  image.step = image_width * 3;
  image.data.resize(static_cast<size_t>(image.step) * image_height);
  for (size_t i = 0; i < image.data.size(); i += 3) {
    image.data[i] = pixel[0];
    image.data[i + 1] = pixel[1];
    image.data[i + 2] = pixel[2];
  }
  return image;
}

TrafficLightRoiArray make_roi_array(int64_t traffic_light_id, const std::vector<Box> & boxes)
{
  TrafficLightRoiArray array;
  for (const auto & box : boxes) {
    TrafficLightRoi roi;
    roi.traffic_light_id = traffic_light_id;
    roi.roi.x_offset = box.x;
    roi.roi.y_offset = box.y;
    roi.roi.width = box.width;
    roi.roi.height = box.height;
    array.rois.push_back(roi);
  }
  return array;
}

TrafficLightArray make_signal_array(
  int64_t traffic_light_id, uint8_t color, uint8_t shape, float confidence = 0.87f)
{
  TrafficLightElement element;
  element.color = color;
  element.shape = shape;
  element.status = TrafficLightElement::SOLID_ON;
  element.confidence = confidence;

  TrafficLight signal;
  signal.traffic_light_id = traffic_light_id;
  signal.elements.push_back(element);

  TrafficLightArray array;
  array.signals.push_back(signal);
  return array;
}

// The inputs the tests send. They are built once and stamped when published, because
// message_filters::ApproximateTime pairs the four topics up by their header stamps.
const Image background_image = make_uniform_image("rgb8", background_rgb);
const TrafficLightRoiArray fine_rois = make_roi_array(signal_id, {fine_box});
const TrafficLightRoiArray rough_rois = make_roi_array(signal_id, {rough_box});
const TrafficLightArray green_signal =
  make_signal_array(signal_id, TrafficLightElement::GREEN, TrafficLightElement::CIRCLE);

Pixel pixel_at(const Image & image, int x, int y)
{
  const size_t offset = static_cast<size_t>(y) * image.step + static_cast<size_t>(x) * 3;
  return {image.data.at(offset), image.data.at(offset + 1), image.data.at(offset + 2)};
}

}  // namespace

class TrafficLightRoiVisualizerNodeTest : public ::testing::Test
{
protected:
  // rclcpp::init() may only be called once per process, so it is done per suite rather than per
  // test. The node itself is recreated for every test, because the parameters under test decide
  // which callback and which publisher the constructor installs.
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void TearDown() override
  {
    executor_.reset();
    output_sub_.reset();
    signal_pub_.reset();
    rough_roi_pub_.reset();
    roi_pub_.reset();
    image_pub_.reset();
    peer_.reset();
    node_.reset();
  }

  // Time given to the node's 100 ms connect_cb timer, plus topic discovery, to subscribe to the
  // inputs once the output has a subscriber.
  static constexpr auto connect_budget = std::chrono::milliseconds(5000);
  // Time a published message gets to reach the node. Only the tests that assert that nothing comes
  // back use it, and they always wait it out, so it also sets how long those three tests take.
  static constexpr auto delivery_budget = std::chrono::milliseconds(500);
  // Time given to the node to publish an output image after a full set of inputs.
  static constexpr auto output_budget = std::chrono::milliseconds(3000);

  void start_node(bool use_high_accuracy_detection, bool use_image_transport)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
      {{"use_high_accuracy_detection", use_high_accuracy_detection},
       {"use_image_transport", use_image_transport}});
    node_ = std::make_shared<TrafficLightRoiVisualizerNode>(options);
    peer_ = std::make_shared<rclcpp::Node>("characterization_peer");

    // The node subscribes to the image with sensor data QoS and to the rest with a depth of 1.
    image_pub_ = peer_->create_publisher<Image>(node_topic("input/image"), rclcpp::SensorDataQoS());
    roi_pub_ =
      peer_->create_publisher<TrafficLightRoiArray>(node_topic("input/rois"), rclcpp::QoS{1});
    rough_roi_pub_ =
      peer_->create_publisher<TrafficLightRoiArray>(node_topic("input/rough/rois"), rclcpp::QoS{1});
    signal_pub_ = peer_->create_publisher<TrafficLightArray>(
      node_topic("input/traffic_signals"), rclcpp::QoS{1});

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    executor_->add_node(peer_);
  }

  void subscribe_output()
  {
    output_sub_ = peer_->create_subscription<Image>(
      node_topic("output/image"), rclcpp::QoS{1},
      [this](Image::ConstSharedPtr message) { output_ = message; });
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

  // The node only subscribes to its inputs while its output has a subscriber, so every test that
  // feeds inputs has to wait for that to happen first.
  bool wait_until_node_subscribes_inputs()
  {
    return pump_until([this] { return image_pub_->get_subscription_count() > 0; }, connect_budget);
  }

  // Stamps a copy of a message. The prepared inputs carry no stamp of their own, so that every
  // test publishes messages that are current.
  template <typename Message>
  Message stamped(Message message, const rclcpp::Time & stamp) const
  {
    message.header.stamp = stamp;
    return message;
  }

  template <typename Message>
  Message stamped(const Message & message) const
  {
    return stamped(message, peer_->now());
  }

  // Publishes one synchronized set of inputs and waits for the output image. All messages get the
  // same stamp, because the node pairs them up with message_filters::ApproximateTime.
  bool send_inputs_and_wait_for_output(
    const Image & image, const TrafficLightRoiArray & rois, const TrafficLightArray & signals,
    const std::optional<TrafficLightRoiArray> & rough_roi_input = std::nullopt)
  {
    const auto now = peer_->now();
    image_pub_->publish(stamped(image, now));
    roi_pub_->publish(stamped(rois, now));
    if (rough_roi_input.has_value()) {
      rough_roi_pub_->publish(stamped(rough_roi_input.value(), now));
    }
    signal_pub_->publish(stamped(signals, now));
    return pump_until([this] { return output_ != nullptr; }, output_budget);
  }

  std::shared_ptr<TrafficLightRoiVisualizerNode> node_;
  std::shared_ptr<rclcpp::Node> peer_;
  rclcpp::Publisher<Image>::SharedPtr image_pub_;
  rclcpp::Publisher<TrafficLightRoiArray>::SharedPtr roi_pub_;
  rclcpp::Publisher<TrafficLightRoiArray>::SharedPtr rough_roi_pub_;
  rclcpp::Publisher<TrafficLightArray>::SharedPtr signal_pub_;
  rclcpp::Subscription<Image>::SharedPtr output_sub_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  Image::ConstSharedPtr output_;
};

// Each parameter is declared without a default, so leaving out either one on its own is already
// enough to keep the node from starting. This is what makes every test below pass both parameters
// explicitly. Leaving out both is not tested separately, because these two cover it.
//
// The exception type is not pinned: it comes from rclcpp, not from this node.
TEST_F(TrafficLightRoiVisualizerNodeTest, Construct_HighAccuracyParameterMissing_Throws)
{
  // Arrange: both parameters are declared without a default, so both are required. Only
  // use_image_transport is given here; use_high_accuracy_detection is left out on purpose.
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"use_image_transport", false}});

  // Act and Assert
  EXPECT_THROW(std::make_shared<TrafficLightRoiVisualizerNode>(options), std::exception);
}

TEST_F(TrafficLightRoiVisualizerNodeTest, Construct_ImageTransportParameterMissing_Throws)
{
  // Arrange: the mirror of the case above - use_image_transport is the one left out.
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"use_high_accuracy_detection", false}});

  // Act and Assert
  EXPECT_THROW(std::make_shared<TrafficLightRoiVisualizerNode>(options), std::exception);
}

// While nothing subscribes to the output image, the node keeps its input subscriptions closed, so
// no upstream node has to serialize images for it. connect_cb() re-checks this every 100 ms. High
// accuracy detection is on so that the gate is observed on all four inputs, rough ROIs included.
//
// The reverse transition (dropping the subscriptions again once the last output subscriber goes
// away) is not pinned; only the initial state is.
TEST_F(TrafficLightRoiVisualizerNodeTest, Interface_OutputUnsubscribed_InputsNotSubscribed)
{
  // Arrange
  start_node(/*use_high_accuracy_detection=*/true, /*use_image_transport=*/false);

  // Act: run the executor so that the node's 100 ms connect timer gets a chance to fire. Nothing
  // subscribes to the output, so this is the whole of the stimulus.
  pump(delivery_budget);

  // Assert: the node leaves all four inputs alone
  EXPECT_EQ(image_pub_->get_subscription_count(), 0u);
  EXPECT_EQ(roi_pub_->get_subscription_count(), 0u);
  EXPECT_EQ(rough_roi_pub_->get_subscription_count(), 0u);
  EXPECT_EQ(signal_pub_->get_subscription_count(), 0u);
}

// Once the output image has a subscriber, the node subscribes to the image, the fine ROIs and the
// traffic signals. Without high accuracy detection it leaves the rough ROIs alone.
TEST_F(TrafficLightRoiVisualizerNodeTest, Interface_OutputSubscribed_FineInputsSubscribed)
{
  // Arrange
  start_node(/*use_high_accuracy_detection=*/false, /*use_image_transport=*/false);

  // Act: subscribing to the output is the event the lazy subscription reacts to; the wait gives
  // the connect timer time to notice.
  subscribe_output();
  ASSERT_TRUE(wait_until_node_subscribes_inputs());

  // Assert: the three inputs the three-input synchronizer needs, and not the rough ROIs
  EXPECT_GT(roi_pub_->get_subscription_count(), 0u);
  EXPECT_GT(signal_pub_->get_subscription_count(), 0u);
  EXPECT_EQ(rough_roi_pub_->get_subscription_count(), 0u);
}

// With high accuracy detection the node additionally subscribes to the rough ROIs, which selects
// the four-input synchronizer and image_rough_roi_callback().
TEST_F(TrafficLightRoiVisualizerNodeTest, Interface_HighAccuracy_RoughRoiSubscribed)
{
  // Arrange
  start_node(/*use_high_accuracy_detection=*/true, /*use_image_transport=*/false);

  // Act: as above, subscribing to the output is what makes the node subscribe to its inputs
  subscribe_output();
  ASSERT_TRUE(wait_until_node_subscribes_inputs());

  // Assert: with the parameter on, the fourth input is subscribed too
  EXPECT_GT(rough_roi_pub_->get_subscription_count(), 0u);
}

// message_filters pairs the inputs up by their stamps and only then calls the callback, so an
// incomplete set produces nothing at all. Without high accuracy detection the set is image + fine
// ROIs + signals, and leaving out the fine ROIs alone is enough to keep the callback from running.
//
// This is a property of the synchronizer, not a branch in the node: nothing in the node's own code
// runs. An empty ROI array is a different thing - it completes the set, and the image comes back
// unchanged (Visualization_NoFineRois_ImageUnchanged). The synchronizer's tolerance for
// differing stamps is not pinned either; every other test publishes one stamp for the whole set.
TEST_F(TrafficLightRoiVisualizerNodeTest, Sync_FineRoisMissing_NoOutput)
{
  // Arrange
  start_node(/*use_high_accuracy_detection=*/false, /*use_image_transport=*/false);
  subscribe_output();
  ASSERT_TRUE(wait_until_node_subscribes_inputs());

  // Act: publish every input the three-input synchronizer takes except the fine ROIs, all with
  // the same stamp so that only the missing one keeps it from pairing them up.
  const auto now = peer_->now();
  image_pub_->publish(stamped(background_image, now));
  signal_pub_->publish(stamped(green_signal, now));
  pump(delivery_budget);

  // Assert: the callback never runs, so nothing is published
  EXPECT_EQ(output_, nullptr);
}

// With high accuracy detection the synchronizer waits for a fourth topic, so a set that would be
// complete for the other callback is not enough here: the rough ROIs are missing.
//
// The reverse case (the rough ROIs arriving while high accuracy detection is off) needs no test of
// its own, because the node does not even subscribe to them then - see
// Interface_OutputSubscribed_FineInputsSubscribed.
TEST_F(TrafficLightRoiVisualizerNodeTest, Sync_RoughRoisMissing_NoOutput)
{
  // Arrange
  start_node(/*use_high_accuracy_detection=*/true, /*use_image_transport=*/false);
  subscribe_output();
  ASSERT_TRUE(wait_until_node_subscribes_inputs());

  // Act: the same, one synchronizer up - everything except the rough ROIs, same stamp
  const auto now = peer_->now();
  image_pub_->publish(stamped(background_image, now));
  roi_pub_->publish(stamped(fine_rois, now));
  signal_pub_->publish(stamped(green_signal, now));
  pump(delivery_budget);

  // Assert: the callback never runs, so nothing is published
  EXPECT_EQ(output_, nullptr);
}

// The three-input synchronizer, end to end: an image, the fine ROIs and the signals go in, and a
// drawn image comes back out.
//
// One pixel of the frame around the fine ROI is enough to show that the callback reached the
// drawing and published what came back. RoiWithSignalGetsFrameAndLabelBox checks what is drawn
// there.
TEST_F(TrafficLightRoiVisualizerNodeTest, Pipeline_FineInputs_DrawnImagePublished)
{
  // Arrange
  start_node(/*use_high_accuracy_detection=*/false, /*use_image_transport=*/false);
  subscribe_output();
  ASSERT_TRUE(wait_until_node_subscribes_inputs());

  // Act
  ASSERT_TRUE(send_inputs_and_wait_for_output(background_image, fine_rois, green_signal));

  // Assert
  EXPECT_EQ(output_->encoding, "rgb8");

  const auto frame_corner = pixel_at(*output_, fine_box.x, fine_box.y);
  EXPECT_EQ(frame_corner, green_signal_rgb);
}

// The four-input synchronizer, end to end. use_high_accuracy_detection adds the rough ROIs as a
// fourth input and hands the set to the other callback.
//
// The pixel checked is the rough ROI's corner, which lies on no other rectangle. Only
// visualize_with_rough_rois() ever paints it, so it also says which of the two callbacks ran.
// RoughAndFineWithSignalDrawBothFramesAndOneLabel checks what that call draws.
TEST_F(TrafficLightRoiVisualizerNodeTest, Pipeline_RoughAndFineInputs_DrawnImagePublished)
{
  // Arrange
  start_node(/*use_high_accuracy_detection=*/true, /*use_image_transport=*/false);
  subscribe_output();
  ASSERT_TRUE(wait_until_node_subscribes_inputs());

  // Act
  ASSERT_TRUE(
    send_inputs_and_wait_for_output(background_image, fine_rois, green_signal, rough_rois));

  // Assert
  EXPECT_EQ(output_->encoding, "rgb8");

  const auto rough_frame_corner = pixel_at(*output_, rough_box.x, rough_box.y);
  EXPECT_EQ(rough_frame_corner, green_signal_rgb);
}

// ---------------------------------------------------------------------------------------------
// The publisher the parameters select. Which one is used is a property of the node, so it stays
// here even though nothing about the drawing is being checked.
// ---------------------------------------------------------------------------------------------

// use_image_transport selects an image_transport publisher instead of a plain rclcpp one. Both
// advertise ~/output/image, so a plain subscriber sees the same output either way. What this test
// pins is that the branch still starts the node and still publishes the drawn image there.
//
// It deliberately does NOT show which of the two publishers was used: that is not observable from
// outside in this build. image_transport only advertises extra topics for the transport plugins
// that are installed, and here there are none - checked on 2026-09-15, `ros2 pkg list` lists
// image_transport alone, and with the parameter on the node still advertises only the raw topic.
// Where compressed_image_transport is installed, ~/output/image/compressed would tell them apart.
TEST_F(TrafficLightRoiVisualizerNodeTest, Interface_ImageTransportEnabled_SameOutput)
{
  // Arrange
  start_node(/*use_high_accuracy_detection=*/false, /*use_image_transport=*/true);
  subscribe_output();
  ASSERT_TRUE(wait_until_node_subscribes_inputs());

  // Act
  ASSERT_TRUE(send_inputs_and_wait_for_output(background_image, fine_rois, green_signal));

  // Assert
  EXPECT_EQ(output_->encoding, "rgb8");

  const auto frame_corner = pixel_at(*output_, fine_box.x, fine_box.y);
  EXPECT_EQ(frame_corner, green_signal_rgb);
}

// The same publisher on the other synchronizer. Both callbacks publish through one helper, so
// this is not a second copy of the branch above; it is the fourth cell of the synchronizer by
// publisher pairing, and the rough ROI's corner is again what says which callback ran.
TEST_F(TrafficLightRoiVisualizerNodeTest, Interface_ImageTransportHighAccuracy_SameOutput)
{
  // Arrange
  start_node(/*use_high_accuracy_detection=*/true, /*use_image_transport=*/true);
  subscribe_output();
  ASSERT_TRUE(wait_until_node_subscribes_inputs());

  // Act
  ASSERT_TRUE(
    send_inputs_and_wait_for_output(background_image, fine_rois, green_signal, rough_rois));

  // Assert
  EXPECT_EQ(output_->encoding, "rgb8");

  const auto rough_frame_corner = pixel_at(*output_, rough_box.x, rough_box.y);
  EXPECT_EQ(rough_frame_corner, green_signal_rgb);
}
