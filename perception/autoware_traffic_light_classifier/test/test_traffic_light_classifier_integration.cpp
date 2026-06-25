// Copyright 2026 TIER IV, Inc.
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
// Integration tests for TrafficLightClassifierNodelet.
//
// These stand up the real node and drive it over its ROS interface: an Image and
// a TrafficLightRoiArray are published on the input topics, the node's own
// lazy-subscription (connectCb) and message_filters ExactTime sync wire them
// through imageRoiCallback, and the test observes the published traffic_signals
// and /diagnostics topics. This exercises the full pub/sub + sync + diagnostics
// path that a unit-level callback invocation would bypass.
//
// Scope is intentionally narrow -- a typical (image, rois) round trip and the
// empty-rois early-return. The per-ROI classification behavior (type filtering,
// exposure, UNKNOWN-handling, ordering) is owned by the ROS-free core test
// (test_traffic_light_classifier) and is not re-checked here.
//

#include "../src/traffic_light_classifier_node.hpp"

#include <rclcpp/rclcpp.hpp>

// cppcheck-suppress preprocessorErrorDirective
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <opencv2/core/core.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <tier4_perception_msgs/msg/traffic_light_array.hpp>
#include <tier4_perception_msgs/msg/traffic_light_roi_array.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

namespace
{
namespace tl = autoware::traffic_light;

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using sensor_msgs::msg::Image;
using tier4_perception_msgs::msg::TrafficLight;
using tier4_perception_msgs::msg::TrafficLightRoi;
using tier4_perception_msgs::msg::TrafficLightRoiArray;

using std::chrono_literals::operator""ms;
using std::chrono_literals::operator""s;
using std::placeholders::_1;

constexpr char node_name[] = "traffic_light_classifier_node";
constexpr char image_topic[] = "/traffic_light_classifier_node/input/image";
constexpr char rois_topic[] = "/traffic_light_classifier_node/input/rois";
constexpr char output_topic[] = "/traffic_light_classifier_node/output/traffic_signals";

constexpr uint8_t car_type = TrafficLight::CAR_TRAFFIC_LIGHT;

// Wide exposure thresholds so a synthetic image is never flagged; exposure
// handling itself is the core test's concern.
constexpr double no_over_threshold = 2.0;
constexpr double no_under_threshold = -2.0;

std_msgs::msg::Header make_header()
{
  std_msgs::msg::Header header;
  header.frame_id = "camera";
  header.stamp.sec = 123;
  header.stamp.nanosec = 456;
  return header;
}

Image make_dummy_image(int width = 16, int height = 16)
{
  constexpr uint8_t neutral_gray = 100;
  cv::Mat mat(height, width, CV_8UC3, cv::Scalar(neutral_gray, neutral_gray, neutral_gray));
  return *cv_bridge::CvImage(make_header(), "rgb8", mat).toImageMsg();
}

TrafficLightRoi make_valid_roi(int64_t id)
{
  TrafficLightRoi roi;
  roi.traffic_light_id = id;
  roi.traffic_light_type = car_type;
  roi.roi.x_offset = 0;
  roi.roi.y_offset = 0;
  roi.roi.width = 16;
  roi.roi.height = 16;
  return roi;
}

// Captured node output for a single round trip. The two inputs are published
// with identical stamps, so ExactTime sync matches them.
struct Captured
{
  bool got_signals = false;
  tier4_perception_msgs::msg::TrafficLightArray signals;
  bool got_diag = false;
  DiagnosticStatus diag;  // status authored by the node under test (hardware_id == node_name)
};

const diagnostic_msgs::msg::KeyValue * find_key_value(
  const DiagnosticStatus & status, const std::string & key)
{
  for (const auto & kv : status.values) {
    if (kv.key == key) {
      return &kv;
    }
  }
  return nullptr;
}

// Checks the node's published exposure diagnostic: severity level plus the two
// over/under exposure flags. Returns an AssertionResult (no gtest macros inside
// helpers) so the assertion stays in the test body.
::testing::AssertionResult has_exposure_diag(
  const Captured & cap, int8_t expected_level, bool expect_over, bool expect_under)
{
  if (!cap.got_diag) {
    return ::testing::AssertionFailure() << "no diagnostic was published by the node";
  }
  if (cap.diag.level != expected_level) {
    return ::testing::AssertionFailure() << "diag level = " << static_cast<int>(cap.diag.level)
                                         << ", expected " << static_cast<int>(expected_level);
  }
  const std::pair<const char *, bool> flags[] = {
    {"detect_traffic_light_over_exposure", expect_over},
    {"detect_traffic_light_under_exposure", expect_under}};
  for (const auto & [key, expected] : flags) {
    const auto * kv = find_key_value(cap.diag, key);
    if (kv == nullptr) {
      return ::testing::AssertionFailure() << "missing diagnostic key: " << key;
    }
    const std::string want = expected ? "True" : "False";
    if (kv->value != want) {
      return ::testing::AssertionFailure()
             << key << " = \"" << kv->value << "\", expected \"" << want << "\"";
    }
  }
  return ::testing::AssertionSuccess();
}

// Subscribes to the node's output + /diagnostics and records what the node under
// test publishes. Named member callbacks (bound below) keep the capture
// lambda-free.
class OutputCapture
{
public:
  void onSignals(tier4_perception_msgs::msg::TrafficLightArray::ConstSharedPtr msg)
  {
    cap.signals = *msg;
    cap.got_signals = true;
  }

  void onDiagnostics(DiagnosticArray::ConstSharedPtr msg)
  {
    for (const auto & status : msg->status) {
      if (status.hardware_id == node_name) {
        cap.diag = status;
        cap.got_diag = true;
      }
    }
  }

  Captured cap;
};

class IntegrationTest : public ::testing::Test
{
protected:
  std::shared_ptr<tl::TrafficLightClassifierNodelet> make_node_under_test()
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override(
      "classifier_type",
      static_cast<int>(tl::TrafficLightClassifierNodelet::ClassifierType::HSVFilter));
    options.append_parameter_override("traffic_light_type", static_cast<int>(car_type));
    options.append_parameter_override("approximate_sync", false);
    options.append_parameter_override("over_exposure_threshold", no_over_threshold);
    options.append_parameter_override("under_exposure_threshold", no_under_threshold);
    options.append_parameter_override("build_only", false);
    node_ = std::make_shared<tl::TrafficLightClassifierNodelet>(options);
    return node_;
  }

  // Publish (image, rois) on the input topics and capture the node's output. The
  // inputs are re-published in a wait loop so the test is robust against the
  // lazy connectCb subscription and pub/sub discovery latency; identical stamps
  // make every pair an ExactTime match. A missing signal publish is a setup
  // failure, surfaced by throwing (helpers stay free of gtest assertions).
  Captured run(const Image & image, const TrafficLightRoiArray & rois)
  {
    OutputCapture capture;
    auto tester = std::make_shared<rclcpp::Node>("integration_tester");
    auto signal_sub = tester->create_subscription<tier4_perception_msgs::msg::TrafficLightArray>(
      output_topic, rclcpp::QoS{1}, std::bind(&OutputCapture::onSignals, &capture, _1));
    auto diag_sub = tester->create_subscription<DiagnosticArray>(
      "/diagnostics", rclcpp::QoS{10}, std::bind(&OutputCapture::onDiagnostics, &capture, _1));
    auto image_pub = tester->create_publisher<Image>(image_topic, rclcpp::SensorDataQoS());
    auto rois_pub = tester->create_publisher<TrafficLightRoiArray>(rois_topic, rclcpp::QoS{1});

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node_);
    exec.add_node(tester);

    const auto deadline = std::chrono::steady_clock::now() + 10s;
    while (!capture.cap.got_signals && std::chrono::steady_clock::now() < deadline) {
      image_pub->publish(image);
      rois_pub->publish(rois);
      exec.spin_some();
      std::this_thread::sleep_for(10ms);
    }
    if (!capture.cap.got_signals) {
      throw std::runtime_error(
        "run(): node under test published no traffic_signals within the timeout");
    }
    // Drain a little more so a diagnostic published in the same callback is
    // received. The empty-rois path publishes none, so this simply times out.
    for (int i = 0; i < 50 && !capture.cap.got_diag; ++i) {
      image_pub->publish(image);
      rois_pub->publish(rois);
      exec.spin_some();
      std::this_thread::sleep_for(5ms);
    }
    return capture.cap;
  }

  std::shared_ptr<tl::TrafficLightClassifierNodelet> node_;
};

// --------------------------------------------------------------------------
// Typical round trip: an image with one valid ROI published on the inputs comes
// back as one signal on the output topic with the input header propagated, and
// the node emits an OK exposure diagnostic (both exposure flags False).
// --------------------------------------------------------------------------
TEST_F(IntegrationTest, TypicalImageRoiRoundTrip)
{
  // Arrange
  make_node_under_test();
  const auto image = make_dummy_image();
  TrafficLightRoiArray rois;
  rois.header = make_header();  // same stamp as the image -> ExactTime match
  rois.rois.push_back(make_valid_roi(/*id=*/1));

  // Act
  const auto cap = run(image, rois);

  // Assert: output signals
  EXPECT_EQ(cap.signals.header, image.header);
  ASSERT_EQ(cap.signals.signals.size(), 1u);
  EXPECT_EQ(cap.signals.signals[0].traffic_light_id, 1);
  EXPECT_EQ(cap.signals.signals[0].elements.size(), 1u);

  // Assert: diagnostics
  EXPECT_TRUE(has_exposure_diag(cap, DiagnosticStatus::OK, /*over=*/false, /*under=*/false));
}

// --------------------------------------------------------------------------
// Empty rois message: the node short-circuits and publishes an empty signal
// array carrying the input header. The early return is taken before diagnostics,
// so no diagnostic is published for this frame.
// --------------------------------------------------------------------------
TEST_F(IntegrationTest, EmptyRoisPublishesEmptySignalsWithHeader)
{
  // Arrange
  make_node_under_test();
  const auto image = make_dummy_image();
  TrafficLightRoiArray rois;
  rois.header = make_header();  // same stamp as the image -> ExactTime match
  // no rois

  // Act
  const auto cap = run(image, rois);

  // Assert
  EXPECT_EQ(cap.signals.signals.size(), 0u);
  EXPECT_EQ(cap.signals.header, image.header);
  EXPECT_FALSE(cap.got_diag);
}

}  // namespace

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
