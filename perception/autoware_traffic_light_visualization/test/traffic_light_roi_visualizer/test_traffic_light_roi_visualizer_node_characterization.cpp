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
// Characterization tests for TrafficLightRoiVisualizerNode.
//
// These tests pin the behavior of the node as it is today, before the core logic is separated
// from rclcpp::Node. They drive the node through its real topics: inputs are published, the
// output image is subscribed, and the drawing is checked by reading pixels out of the published
// image. They are deliberately not exhaustive - the goal is to catch a fatal regression during
// the refactoring (does not build, does not start, publishes nothing, drawing no longer runs),
// not to specify every corner of the node's behavior. Once the logic is covered by unit tests,
// this file is replaced by a small integration test.

#include "traffic_light_roi_visualizer/node.hpp"

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
constexpr int64_t other_signal_id = 43;

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
constexpr Pixel red_signal_rgb{254, 149, 149};    // str_to_color("red")
constexpr Pixel amber_signal_rgb{254, 250, 149};  // str_to_color("yellow")
constexpr Pixel green_signal_rgb{149, 254, 161};  // str_to_color("green")
// The frame color comes from the circle element of the label only. Without a circle the color
// stays at extract_shape_info()'s initial value, which is also what draw_roi_with_id() is handed
// for a ROI with no signal at all - the two are indistinguishable in the output.
constexpr Pixel no_circle_rgb{255, 255, 255};
constexpr Pixel unknown_circle_rgb{250, 250, 250};  // str_to_color() fallback, e.g. for "unknown"

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
const TrafficLightRoiArray no_rois = make_roi_array(signal_id, {});
// A fine ROI for a traffic light that the rough ROIs do not mention.
const TrafficLightRoiArray fine_rois_for_other_light = make_roi_array(other_signal_id, {fine_box});
const TrafficLightArray green_signal =
  make_signal_array(signal_id, TrafficLightElement::GREEN, TrafficLightElement::CIRCLE);
const TrafficLightArray unknown_signal =
  make_signal_array(signal_id, TrafficLightElement::UNKNOWN, TrafficLightElement::CIRCLE);
// A green arrow, i.e. a classified signal whose only element is not a circle.
const TrafficLightArray arrow_signal =
  make_signal_array(signal_id, TrafficLightElement::GREEN, TrafficLightElement::LEFT_ARROW);
// A green signal reported for a different traffic light than the one the ROI belongs to.
const TrafficLightArray signal_for_other_light =
  make_signal_array(other_signal_id, TrafficLightElement::GREEN, TrafficLightElement::CIRCLE);

TrafficLightArray merge(TrafficLightArray first, const TrafficLightArray & second)
{
  first.signals.insert(first.signals.end(), second.signals.begin(), second.signals.end());
  return first;
}

const TrafficLightArray signals_for_both_lights = merge(green_signal, signal_for_other_light);

Pixel pixel_at(const Image & image, int x, int y);

// Two points in the strip above a ROI, where a label box is drawn. Offsets measured on 2026-09-15.
constexpr Pixel label_icon_rgb{0, 0, 0};

// The fill of the box, right of where the id text reaches: background unless a box was drawn.
Pixel label_box_pixel(const Image & image, const Box & box)
{
  return pixel_at(image, box.x + 80, box.y - 13);
}

// The shape icon inside the box. Black gives it away: the frames and the id text use the signal
// color, so nothing else in the output is black.
Pixel label_icon_pixel(const Image & image, const Box & box)
{
  return pixel_at(image, box.x + 13, box.y - 13);
}

Pixel pixel_at(const Image & image, int x, int y)
{
  const size_t offset = static_cast<size_t>(y) * image.step + static_cast<size_t>(x) * 3;
  return {image.data.at(offset), image.data.at(offset + 1), image.data.at(offset + 2)};
}

size_t count_pixels_differing_from(const Image & image, const Pixel & reference)
{
  size_t count = 0;
  for (int y = 0; y < static_cast<int>(image.height); ++y) {
    for (int x = 0; x < static_cast<int>(image.width); ++x) {
      if (pixel_at(image, x, y) != reference) {
        ++count;
      }
    }
  }
  return count;
}

}  // namespace

class TrafficLightRoiVisualizerCharacterization : public ::testing::Test
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
TEST_F(TrafficLightRoiVisualizerCharacterization, Construct_HighAccuracyParameterMissing_Throws)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"use_image_transport", false}});

  EXPECT_THROW(std::make_shared<TrafficLightRoiVisualizerNode>(options), std::exception);
}

TEST_F(TrafficLightRoiVisualizerCharacterization, Construct_ImageTransportParameterMissing_Throws)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"use_high_accuracy_detection", false}});

  EXPECT_THROW(std::make_shared<TrafficLightRoiVisualizerNode>(options), std::exception);
}

// While nothing subscribes to the output image, the node keeps its input subscriptions closed, so
// no upstream node has to serialize images for it. connect_cb() re-checks this every 100 ms. High
// accuracy detection is on so that the gate is observed on all four inputs, rough ROIs included.
//
// The reverse transition (dropping the subscriptions again once the last output subscriber goes
// away) is not pinned; only the initial state is.
TEST_F(TrafficLightRoiVisualizerCharacterization, Interface_OutputUnsubscribed_InputsNotSubscribed)
{
  start_node(/*use_high_accuracy_detection=*/true, /*use_image_transport=*/false);

  pump(delivery_budget);

  EXPECT_EQ(image_pub_->get_subscription_count(), 0u);
  EXPECT_EQ(roi_pub_->get_subscription_count(), 0u);
  EXPECT_EQ(rough_roi_pub_->get_subscription_count(), 0u);
  EXPECT_EQ(signal_pub_->get_subscription_count(), 0u);
}

// Once the output image has a subscriber, the node subscribes to the image, the fine ROIs and the
// traffic signals. Without high accuracy detection it leaves the rough ROIs alone.
TEST_F(TrafficLightRoiVisualizerCharacterization, Interface_OutputSubscribed_FineInputsSubscribed)
{
  start_node(/*use_high_accuracy_detection=*/false, /*use_image_transport=*/false);
  subscribe_output();

  ASSERT_TRUE(wait_until_node_subscribes_inputs());

  EXPECT_GT(roi_pub_->get_subscription_count(), 0u);
  EXPECT_GT(signal_pub_->get_subscription_count(), 0u);
  EXPECT_EQ(rough_roi_pub_->get_subscription_count(), 0u);
}

// With high accuracy detection the node additionally subscribes to the rough ROIs, which selects
// the four-input synchronizer and image_rough_roi_callback().
TEST_F(TrafficLightRoiVisualizerCharacterization, Interface_HighAccuracy_RoughRoiSubscribed)
{
  start_node(/*use_high_accuracy_detection=*/true, /*use_image_transport=*/false);
  subscribe_output();

  ASSERT_TRUE(wait_until_node_subscribes_inputs());

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
TEST_F(TrafficLightRoiVisualizerCharacterization, Sync_FineRoisMissing_NoOutput)
{
  start_node(/*use_high_accuracy_detection=*/false, /*use_image_transport=*/false);
  subscribe_output();
  ASSERT_TRUE(wait_until_node_subscribes_inputs());

  const auto now = peer_->now();
  image_pub_->publish(stamped(background_image, now));
  signal_pub_->publish(stamped(green_signal, now));
  pump(delivery_budget);

  EXPECT_EQ(output_, nullptr);
}

// With high accuracy detection the synchronizer waits for a fourth topic, so a set that would be
// complete for the other callback is not enough here: the rough ROIs are missing.
//
// The reverse case (the rough ROIs arriving while high accuracy detection is off) needs no test of
// its own, because the node does not even subscribe to them then - see
// Interface_OutputSubscribed_FineInputsSubscribed.
TEST_F(TrafficLightRoiVisualizerCharacterization, Sync_RoughRoisMissing_NoOutput)
{
  start_node(/*use_high_accuracy_detection=*/true, /*use_image_transport=*/false);
  subscribe_output();
  ASSERT_TRUE(wait_until_node_subscribes_inputs());

  const auto now = peer_->now();
  image_pub_->publish(stamped(background_image, now));
  roi_pub_->publish(stamped(fine_rois, now));
  signal_pub_->publish(stamped(green_signal, now));
  pump(delivery_budget);

  EXPECT_EQ(output_, nullptr);
}

// ---------------------------------------------------------------------------------------------
// Drawing without high accuracy detection: the callback walks the fine ROIs, so a fine ROI is the
// subject, and the only question per ROI is whether a signal was classified for it. The tests run
// from an empty array, through a ROI without a signal, to a ROI with one.
// ---------------------------------------------------------------------------------------------
// With no ROIs to draw, the node still republishes the image, unchanged apart from the conversion
// to RGB8. This is the path that keeps the output alive while nothing is detected.
TEST_F(TrafficLightRoiVisualizerCharacterization, Visualization_NoFineRois_ImageUnchanged)
{
  start_node(/*use_high_accuracy_detection=*/false, /*use_image_transport=*/false);
  subscribe_output();
  ASSERT_TRUE(wait_until_node_subscribes_inputs());

  ASSERT_TRUE(send_inputs_and_wait_for_output(background_image, no_rois, green_signal));

  EXPECT_EQ(count_pixels_differing_from(*output_, background_rgb), 0u);
}

// A ROI without a matching signal goes through draw_roi_with_id(), which draws no
// label box at all - there is no shape and no confidence to show. The frame itself is still drawn.
//
// The frame color is not pinned here; it belongs to the colors section further down.
TEST_F(TrafficLightRoiVisualizerCharacterization, Visualization_FineNoSignal_NoLabelBox)
{
  start_node(/*use_high_accuracy_detection=*/false, /*use_image_transport=*/false);
  subscribe_output();
  ASSERT_TRUE(wait_until_node_subscribes_inputs());

  ASSERT_TRUE(send_inputs_and_wait_for_output(background_image, fine_rois, signal_for_other_light));

  // The frame is drawn
  const auto frame_corner = pixel_at(*output_, fine_box.x, fine_box.y);
  EXPECT_NE(frame_corner, background_rgb);

  // The label box is not drawn
  const auto above_the_roi = label_box_pixel(*output_, fine_box);
  EXPECT_EQ(above_the_roi, background_rgb);
}

// A ROI whose id matches a classified traffic signal is drawn in the color of that signal's
// circle, and the label box with the shape icon and the confidence is drawn above it. The output
// keeps the geometry and the RGB8 encoding of the input image.
//
// The exact shape of the label box is not pinned - only that it is drawn above the ROI in the
// signal color and contains black text - because it is a rendering detail.
TEST_F(TrafficLightRoiVisualizerCharacterization, Visualization_FineWithSignal_FrameAndLabelDrawn)
{
  start_node(/*use_high_accuracy_detection=*/false, /*use_image_transport=*/false);
  subscribe_output();
  ASSERT_TRUE(wait_until_node_subscribes_inputs());

  ASSERT_TRUE(send_inputs_and_wait_for_output(background_image, fine_rois, green_signal));

  EXPECT_EQ(output_->width, static_cast<uint32_t>(image_width));
  EXPECT_EQ(output_->height, static_cast<uint32_t>(image_height));
  EXPECT_EQ(output_->encoding, "rgb8");

  // The rectangle around the ROI should be painted.
  const auto frame_corner = pixel_at(*output_, fine_box.x, fine_box.y);
  const auto frame_bottom_left = pixel_at(*output_, fine_box.x, fine_box.y + fine_box.height);
  const auto frame_top_right = pixel_at(*output_, fine_box.x + fine_box.width, fine_box.y);
  EXPECT_EQ(frame_corner, green_signal_rgb);
  EXPECT_EQ(frame_bottom_left, green_signal_rgb);
  EXPECT_EQ(frame_top_right, green_signal_rgb);
  // The inside of the ROI is left untouched.
  const auto roi_interior =
    pixel_at(*output_, fine_box.x + fine_box.width / 2, fine_box.y + fine_box.height / 2);
  EXPECT_EQ(roi_interior, background_rgb);
  // The label box sits above the ROI, filled with the signal color and carrying the shape icon.
  const auto above_the_roi = label_box_pixel(*output_, fine_box);
  const auto label_icon = label_icon_pixel(*output_, fine_box);
  EXPECT_EQ(above_the_roi, green_signal_rgb);
  EXPECT_EQ(label_icon, label_icon_rgb);
}

// ---------------------------------------------------------------------------------------------
// Drawing with high accuracy detection: the callback walks the rough ROIs, so a rough ROI is the
// subject, and per ROI both a fine ROI and a signal may or may not be found for its id. The tests
// run in the same order as above, each signal case split by whether the fine ROI is there.
// ---------------------------------------------------------------------------------------------
// With no rough ROIs the loop body never runs, so the image is republished untouched even though
// signals are there. This is the high accuracy counterpart of
// Visualization_NoFineRois_ImageUnchanged.
TEST_F(TrafficLightRoiVisualizerCharacterization, Visualization_NoRoughRois_ImageUnchanged)
{
  start_node(/*use_high_accuracy_detection=*/true, /*use_image_transport=*/false);
  subscribe_output();
  ASSERT_TRUE(wait_until_node_subscribes_inputs());

  ASSERT_TRUE(send_inputs_and_wait_for_output(background_image, no_rois, green_signal, no_rois));

  EXPECT_EQ(count_pixels_differing_from(*output_, background_rgb), 0u);
}

// The rough ROI callback looks up a fine ROI and a signal by id for each rough ROI and draws one of
// four combinations. This is the one with a fine ROI but no signal: both boxes are drawn, both in
// white, and the id is drawn twice because both go through the cv::Scalar overload.
//
// This test and the next three cover the whole dispatch, which is the part that moves into the
// logic class when it is separated.
TEST_F(TrafficLightRoiVisualizerCharacterization, Visualization_RoughAndFineNoSignal_BothWhite)
{
  start_node(/*use_high_accuracy_detection=*/true, /*use_image_transport=*/false);
  subscribe_output();
  ASSERT_TRUE(wait_until_node_subscribes_inputs());

  ASSERT_TRUE(send_inputs_and_wait_for_output(
    background_image, fine_rois, signal_for_other_light, rough_rois));

  // Both frames are drawn, and both in white: extract_shape_info() of an empty label falls back to
  // it, so even the rough frame carries no signal color.
  const auto rough_frame_corner = pixel_at(*output_, rough_box.x, rough_box.y);
  const auto fine_frame_corner = pixel_at(*output_, fine_box.x, fine_box.y);
  EXPECT_EQ(rough_frame_corner, no_circle_rgb);
  EXPECT_EQ(fine_frame_corner, no_circle_rgb);
}

// With neither a fine ROI nor a signal, only the white rough box is left: the node still shows
// that the map expects a traffic light there.
TEST_F(TrafficLightRoiVisualizerCharacterization, Visualization_RoughOnlyNoSignal_RoughWhite)
{
  start_node(/*use_high_accuracy_detection=*/true, /*use_image_transport=*/false);
  subscribe_output();
  ASSERT_TRUE(wait_until_node_subscribes_inputs());

  ASSERT_TRUE(
    send_inputs_and_wait_for_output(background_image, no_rois, signal_for_other_light, rough_rois));

  // The rough frame is drawn, in white
  const auto rough_frame_corner = pixel_at(*output_, rough_box.x, rough_box.y);
  EXPECT_EQ(rough_frame_corner, no_circle_rgb);

  // No label box above it, because no shape is known
  const auto above_the_rough_roi = label_box_pixel(*output_, rough_box);
  EXPECT_EQ(above_the_rough_roi, background_rgb);
}

// With high accuracy detection both rectangles are drawn: the rough ROI from the map based
// detector and, inside it, the fine ROI from the fine detector.
//
// The two corners checked below lie on exactly one rectangle each, and the label box is checked at
// the fine ROI - the opposite of Visualization_RoughOnlyWithSignal_RoughLabeled, where the same
// box lands on the rough ROI instead.
TEST_F(TrafficLightRoiVisualizerCharacterization, Visualization_RoughAndFineWithSignal_BothDrawn)
{
  start_node(/*use_high_accuracy_detection=*/true, /*use_image_transport=*/false);
  subscribe_output();
  ASSERT_TRUE(wait_until_node_subscribes_inputs());

  ASSERT_TRUE(
    send_inputs_and_wait_for_output(background_image, fine_rois, green_signal, rough_rois));

  // Both frames are drawn. Each of these corners lies on one rectangle only.
  const auto rough_frame_corner = pixel_at(*output_, rough_box.x, rough_box.y);
  const auto fine_frame_bottom_left = pixel_at(*output_, fine_box.x, fine_box.y + fine_box.height);
  EXPECT_EQ(rough_frame_corner, green_signal_rgb);
  EXPECT_EQ(fine_frame_bottom_left, green_signal_rgb);

  // The label box goes above the fine ROI, not the rough one
  const auto icon_above_the_fine_roi = label_icon_pixel(*output_, fine_box);
  EXPECT_EQ(icon_above_the_fine_roi, label_icon_rgb);
}

// Without a fine ROI the signal is drawn on the rough box instead, label box included, so a
// classified traffic light is always shown somewhere.
TEST_F(TrafficLightRoiVisualizerCharacterization, Visualization_RoughOnlyWithSignal_RoughLabeled)
{
  start_node(/*use_high_accuracy_detection=*/true, /*use_image_transport=*/false);
  subscribe_output();
  ASSERT_TRUE(wait_until_node_subscribes_inputs());

  ASSERT_TRUE(send_inputs_and_wait_for_output(background_image, no_rois, green_signal, rough_rois));

  // The rough frame is drawn in the signal color
  const auto rough_frame_corner = pixel_at(*output_, rough_box.x, rough_box.y);
  EXPECT_EQ(rough_frame_corner, green_signal_rgb);

  // The label box goes above the rough ROI. The icon rather than the fill of the box, because the
  // id text is drawn over the fill in the same color.
  const auto icon_above_the_rough_roi = label_icon_pixel(*output_, rough_box);
  EXPECT_EQ(icon_above_the_rough_roi, label_icon_rgb);

  // Nothing is drawn where a fine ROI would have been
  const auto where_the_fine_roi_would_be = pixel_at(*output_, fine_box.x, fine_box.y);
  EXPECT_EQ(where_the_fine_roi_would_be, background_rgb);
}

// The loop runs over the rough ROIs only, so a traffic light that the rough ROIs do not mention is
// dropped without a trace - even when both its fine ROI and its signal are there. Without high
// accuracy detection the same fine ROI would be drawn. That makes every combination without a
// rough ROI ("fine only", "signal only", "fine + signal") behave the same, which is why only this
// one is pinned.
//
// NOTE(characterization): whether that asymmetry is intended is unclear. The callback assumes that
// the rough ROIs cover every fine ROI ("a rough roi will always have correspond roi", node.cpp).
TEST_F(TrafficLightRoiVisualizerCharacterization, Visualization_FineWithoutRoughRoi_NotDrawn)
{
  start_node(/*use_high_accuracy_detection=*/true, /*use_image_transport=*/false);
  subscribe_output();
  ASSERT_TRUE(wait_until_node_subscribes_inputs());

  // Signals for both traffic lights, but a rough ROI only for the first one.
  ASSERT_TRUE(send_inputs_and_wait_for_output(
    background_image, fine_rois_for_other_light, signals_for_both_lights, rough_rois));

  // The rough ROI that was mentioned is drawn
  const auto rough_frame_corner = pixel_at(*output_, rough_box.x, rough_box.y);
  EXPECT_EQ(rough_frame_corner, green_signal_rgb);

  // The fine ROI of the other traffic light is not, although its signal was there too
  const auto where_the_other_fine_roi_is = pixel_at(*output_, fine_box.x, fine_box.y);
  EXPECT_EQ(where_the_other_fine_roi_is, background_rgb);
}

// ---------------------------------------------------------------------------------------------
// Properties shared by both callbacks. The frame color is derived by get_classification_result()
// and extract_shape_info(), which both callbacks call, and measuring the rough and the fine frame
// side by side on 2026-09-15 gave the same color in every case below - so it is pinned once, on the
// fine path. Only the circle element of a label decides the color; the other elements are drawn as
// icons but do not change it.
// ---------------------------------------------------------------------------------------------

// No signal at all: the frame is plain white.
TEST_F(TrafficLightRoiVisualizerCharacterization, Visualization_NoSignal_FrameWhite)
{
  start_node(/*use_high_accuracy_detection=*/false, /*use_image_transport=*/false);
  subscribe_output();
  ASSERT_TRUE(wait_until_node_subscribes_inputs());

  ASSERT_TRUE(send_inputs_and_wait_for_output(background_image, fine_rois, signal_for_other_light));

  // The frame falls back to plain white
  const auto frame_corner = pixel_at(*output_, fine_box.x, fine_box.y);
  EXPECT_EQ(frame_corner, no_circle_rgb);
}

// A circle whose color is known: the frame takes that color. str_to_color() knows three of them,
// and all three are pinned because they are the mapping the README documents - one case each, so
// that a failure names the color that broke.
TEST_F(TrafficLightRoiVisualizerCharacterization, Visualization_RedCircleSignal_FrameRed)
{
  start_node(/*use_high_accuracy_detection=*/false, /*use_image_transport=*/false);
  subscribe_output();
  ASSERT_TRUE(wait_until_node_subscribes_inputs());

  ASSERT_TRUE(send_inputs_and_wait_for_output(
    background_image, fine_rois,
    make_signal_array(signal_id, TrafficLightElement::RED, TrafficLightElement::CIRCLE)));

  // The frame takes the color of the circle
  const auto frame_corner = pixel_at(*output_, fine_box.x, fine_box.y);
  EXPECT_EQ(frame_corner, red_signal_rgb);
}

TEST_F(TrafficLightRoiVisualizerCharacterization, Visualization_AmberCircleSignal_FrameAmber)
{
  start_node(/*use_high_accuracy_detection=*/false, /*use_image_transport=*/false);
  subscribe_output();
  ASSERT_TRUE(wait_until_node_subscribes_inputs());

  ASSERT_TRUE(send_inputs_and_wait_for_output(
    background_image, fine_rois,
    make_signal_array(signal_id, TrafficLightElement::AMBER, TrafficLightElement::CIRCLE)));

  const auto frame_corner = pixel_at(*output_, fine_box.x, fine_box.y);
  EXPECT_EQ(frame_corner, amber_signal_rgb);
}

TEST_F(TrafficLightRoiVisualizerCharacterization, Visualization_GreenCircleSignal_FrameGreen)
{
  start_node(/*use_high_accuracy_detection=*/false, /*use_image_transport=*/false);
  subscribe_output();
  ASSERT_TRUE(wait_until_node_subscribes_inputs());

  ASSERT_TRUE(send_inputs_and_wait_for_output(background_image, fine_rois, green_signal));

  const auto frame_corner = pixel_at(*output_, fine_box.x, fine_box.y);
  EXPECT_EQ(frame_corner, green_signal_rgb);
}

// A circle whose color is UNKNOWN: the frame takes str_to_color()'s fallback, an off-white that is
// five levels darker than the plain white above.
//
// NOTE(characterization): two whites that close together look unintended rather than designed.
TEST_F(TrafficLightRoiVisualizerCharacterization, Visualization_UnknownCircleSignal_FrameOffWhite)
{
  start_node(/*use_high_accuracy_detection=*/false, /*use_image_transport=*/false);
  subscribe_output();
  ASSERT_TRUE(wait_until_node_subscribes_inputs());

  ASSERT_TRUE(send_inputs_and_wait_for_output(background_image, fine_rois, unknown_signal));

  // The frame takes str_to_color()'s fallback, not the plain white above
  const auto frame_corner = pixel_at(*output_, fine_box.x, fine_box.y);
  EXPECT_EQ(frame_corner, unknown_circle_rgb);
}

// A signal with no circle element at all - a lit arrow, which is a normal state for a Japanese
// traffic light - leaves the frame at the initial color, the same plain white as no signal at all.
//
// NOTE(characterization): "a green arrow" and "nothing classified" therefore look identical in the
// output, which the README does not mention (it only says unknown shows as white).
TEST_F(TrafficLightRoiVisualizerCharacterization, Visualization_NonCircleSignal_FrameWhite)
{
  start_node(/*use_high_accuracy_detection=*/false, /*use_image_transport=*/false);
  subscribe_output();
  ASSERT_TRUE(wait_until_node_subscribes_inputs());

  ASSERT_TRUE(send_inputs_and_wait_for_output(background_image, fine_rois, arrow_signal));

  // The frame stays at the initial color, the same white as no signal at all
  const auto frame_corner = pixel_at(*output_, fine_box.x, fine_box.y);
  EXPECT_EQ(frame_corner, no_circle_rgb);
}

// The image is converted to RGB8 whatever its input encoding is, so a BGR8 image comes out with
// its red and blue channels swapped.
TEST_F(TrafficLightRoiVisualizerCharacterization, Visualization_Bgr8Image_RepublishedAsRgb8)
{
  start_node(/*use_high_accuracy_detection=*/false, /*use_image_transport=*/false);
  subscribe_output();
  ASSERT_TRUE(wait_until_node_subscribes_inputs());

  // Distinct channel values so that a missing conversion cannot pass unnoticed.
  constexpr Pixel bgr_channels{10, 20, 30};
  constexpr Pixel expected_rgb{30, 20, 10};

  ASSERT_TRUE(send_inputs_and_wait_for_output(
    make_uniform_image("bgr8", bgr_channels), no_rois, green_signal));

  EXPECT_EQ(output_->encoding, "rgb8");

  // Any pixel will do: the image is uniform and no ROI is drawn on it
  const auto any_pixel = pixel_at(*output_, 0, 0);
  EXPECT_EQ(any_pixel, expected_rgb);
}

// use_image_transport selects an image_transport publisher instead of a plain rclcpp one. Both
// advertise ~/output/image, so a plain subscriber sees the same output either way. What this test
// pins is that the branch still starts the node and still publishes the drawn image there.
//
// It deliberately does NOT show which of the two publishers was used: that is not observable from
// outside in this build. image_transport only advertises extra topics for the transport plugins
// that are installed, and here there are none - checked on 2026-09-15, `ros2 pkg list` lists
// image_transport alone, and with the parameter on the node still advertises only the raw topic.
// Where compressed_image_transport is installed, ~/output/image/compressed would tell them apart.
TEST_F(TrafficLightRoiVisualizerCharacterization, Interface_ImageTransportEnabled_SameOutput)
{
  start_node(/*use_high_accuracy_detection=*/false, /*use_image_transport=*/true);
  subscribe_output();
  ASSERT_TRUE(wait_until_node_subscribes_inputs());

  ASSERT_TRUE(send_inputs_and_wait_for_output(background_image, fine_rois, green_signal));

  EXPECT_EQ(output_->encoding, "rgb8");

  // One drawn pixel is enough: this case is about the publisher, not the drawing
  const auto frame_corner = pixel_at(*output_, fine_box.x, fine_box.y);
  EXPECT_EQ(frame_corner, green_signal_rgb);
}

// Each callback carries its own copy of that branch, so the one that walks the rough ROIs needs a
// case of its own - the test above only exercises the other one.
TEST_F(TrafficLightRoiVisualizerCharacterization, Interface_ImageTransportHighAccuracy_SameOutput)
{
  start_node(/*use_high_accuracy_detection=*/true, /*use_image_transport=*/true);
  subscribe_output();
  ASSERT_TRUE(wait_until_node_subscribes_inputs());

  ASSERT_TRUE(
    send_inputs_and_wait_for_output(background_image, fine_rois, green_signal, rough_rois));

  EXPECT_EQ(output_->encoding, "rgb8");

  const auto rough_frame_corner = pixel_at(*output_, rough_box.x, rough_box.y);
  EXPECT_EQ(rough_frame_corner, green_signal_rgb);
}
