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
// Characterization tests for TrafficLightClassifierNodelet.
//
// Purpose: pin the *current* observable behavior of the node's orchestration
// logic (TrafficLightClassifierNodelet::imageRoiCallback).
// Test strategy:
//   * Drive the node by calling the public imageRoiCallback() directly,
//     bypassing message_filters time-sync / the lazy-subscription timer so the
//     tests are deterministic.
//   * The exact lamp *color* chosen by ColorClassifier is its own concern and
//     is intentionally NOT pinned here; only the node-level orchestration
//     (filtering, UNKNOWN handling, ordering, id/type propagation, exposure
//     overwrite, diagnostics) is characterized.
//
// Not characterized (a refactor may change these without a failure here):
//   * message_filters time-sync, the connectCb lazy subscription, and other
//     construction-time concerns (build_only) -- bypassed by the direct call.
//   * Error / degenerate paths impractical to drive via the HSV backend:
//     classifier failure (early return), out-of-bounds ROI (OpenCV throws), and
//     cv_bridge conversion failure -- the catch does NOT return, so a bad image
//     hits a null cv_ptr deref (a latent bug; left unfixed, tracked separately).
//   * Diagnostic timestamp and authoring hardware_id/name.
//

#include "../src/traffic_light_classifier_node.hpp"

#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <tier4_perception_msgs/msg/traffic_light_array.hpp>
#include <tier4_perception_msgs/msg/traffic_light_roi_array.hpp>

#include <gtest/gtest.h>

#include <chrono>
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
using tier4_perception_msgs::msg::TrafficLightArray;
using tier4_perception_msgs::msg::TrafficLightElement;
using tier4_perception_msgs::msg::TrafficLightRoi;
using tier4_perception_msgs::msg::TrafficLightRoiArray;

using std::chrono_literals::operator""ms;
using std::chrono_literals::operator""s;

constexpr char node_name[] = "traffic_light_classifier_node";
constexpr char output_topic[] = "/traffic_light_classifier_node/output/traffic_signals";

// CAR == 0 is the default classification target used throughout these tests.
constexpr uint8_t car_type = TrafficLight::CAR_TRAFFIC_LIGHT;
constexpr uint8_t pedestrian_type = TrafficLight::PEDESTRIAN_TRAFFIC_LIGHT;

// Wide exposure thresholds so that compute_brightness (range ~[-1.0, 1.27])
// never flags a normal/synthetic image. Used by every non-exposure test.
constexpr double no_over_threshold = 2.0;
constexpr double no_under_threshold = -2.0;

// Named rgb8 fill colors (R, G, B) for synthesizing exposure / classification
// inputs, so call sites read by intent rather than raw scalars.
const cv::Scalar white(255, 255, 255);     // bright -> over-exposed
const cv::Scalar black(0, 0, 0);           // dark -> under-exposed
const cv::Scalar green_lamp(82, 200, 82);  // desaturated green -> classifies as GREEN once primed

std_msgs::msg::Header make_header()
{
  std_msgs::msg::Header header;
  header.frame_id = "camera";
  header.stamp.sec = 123;
  header.stamp.nanosec = 456;
  return header;
}

// A uniformly painted image. The exposure tests use `white` (over-exposed) and
// `black` (under-exposed); make_dummy_image builds on it for a neutral fill. The
// default 16x16 hosts a single full-frame 16x16 ROI; pass larger explicit
// dimensions when laying out multiple ROIs (the image must contain every ROI).
Image::SharedPtr make_solid_image(const cv::Scalar & color, int width = 16, int height = 16)
{
  cv::Mat mat(height, width, CV_8UC3, color);
  return cv_bridge::CvImage(make_header(), "rgb8", mat).toImageMsg();
}

// A placeholder image for tests that don't care about pixel content -- only the
// dimensions matter (ROIs index into it). The fixed neutral gray is chosen so
// it never trips the exposure thresholds; the color itself carries no meaning.
Image::SharedPtr make_dummy_image(int width = 16, int height = 16)
{
  constexpr uint8_t neutral_gray = 100;
  return make_solid_image(cv::Scalar(neutral_gray, neutral_gray, neutral_gray), width, height);
}

// A 32x16 image split into two 16x16 halves: left half painted `left`, right
// half painted `right`. Pairs with make_valid_roi(id) (left half) and
// make_valid_roi(id, /*x=*/16) (right half) to give each ROI a distinct color.
Image::SharedPtr make_left_right_image(const cv::Scalar & left, const cv::Scalar & right)
{
  cv::Mat mat(/*rows=*/16, /*cols=*/32, CV_8UC3, left);
  mat(cv::Rect(/*x=*/16, /*y=*/0, /*width=*/16, /*height=*/16)).setTo(right);
  return cv_bridge::CvImage(make_header(), "rgb8", mat).toImageMsg();
}

TrafficLightRoi make_roi(
  int64_t id, uint8_t type, uint32_t x, uint32_t y, uint32_t width, uint32_t height)
{
  TrafficLightRoi roi;
  roi.traffic_light_id = id;
  roi.traffic_light_type = type;
  roi.roi.x_offset = x;
  roi.roi.y_offset = y;
  roi.roi.width = width;
  roi.roi.height = height;
  return roi;
}

// A valid ROI: the target (car) type with non-zero size, so it gets classified.
// Geometry defaults to a single full-frame 16x16 region; pass x (and the rest)
// when laying out multiple ROIs side by side.
TrafficLightRoi make_valid_roi(
  int64_t id, uint32_t x = 0, uint32_t y = 0, uint32_t width = 16, uint32_t height = 16)
{
  return make_roi(id, car_type, x, y, width, height);
}

// A zero-sized ROI of the target type: treated as undetected and appended as
// UNKNOWN (it is never handed to the classifier).
TrafficLightRoi make_zero_sized_roi(int64_t id)
{
  return make_roi(id, car_type, 0, 0, 0, 0);
}

// Captured node output for a single imageRoiCallback() invocation.
struct Captured
{
  bool got_signals = false;
  TrafficLightArray signals;
  bool got_diag = false;
  DiagnosticStatus diag;  // status authored by this node (hardware_id == node_name)
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
// helpers) so the assertion stays in the test body, e.g.
//   EXPECT_TRUE(has_exposure_diag(cap, DiagnosticStatus::OK, false, false));
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

// A signal whose single element was reset to UNKNOWN: color and shape UNKNOWN
// with confidence 0 -- the canonical setSignalUnknown shape, produced both for
// undetected (zero-sized) ROIs and for exposure-overwritten slots. Returns an
// AssertionResult so the assertion stays in the test body.
::testing::AssertionResult is_unknown_signal(const TrafficLight & signal)
{
  if (signal.elements.size() != 1u) {
    return ::testing::AssertionFailure()
           << "element count = " << signal.elements.size() << ", expected 1";
  }
  const auto & element = signal.elements[0];
  if (element.color != TrafficLightElement::UNKNOWN) {
    return ::testing::AssertionFailure()
           << "color = " << static_cast<int>(element.color) << ", expected UNKNOWN";
  }
  if (element.shape != TrafficLightElement::UNKNOWN) {
    return ::testing::AssertionFailure()
           << "shape = " << static_cast<int>(element.shape) << ", expected UNKNOWN";
  }
  if (element.confidence != 0.0f) {
    return ::testing::AssertionFailure() << "confidence = " << element.confidence << ", expected 0";
  }
  return ::testing::AssertionSuccess();
}

class CharacterizationTest : public ::testing::Test
{
protected:
  // Build a fresh node. Defaults to the HSV (ColorClassifier) CPU backend;
  // classifier_type can be overridden (e.g. an unknown value to exercise the
  // "no classifier constructed" path).
  std::shared_ptr<tl::TrafficLightClassifierNodelet> make_node_under_test(
    uint8_t classify_type, double over_thr, double under_thr,
    int classifier_type =
      static_cast<int>(tl::TrafficLightClassifierNodelet::ClassifierType::HSVFilter))
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("classifier_type", classifier_type);
    options.append_parameter_override("traffic_light_type", static_cast<int>(classify_type));
    options.append_parameter_override("approximate_sync", false);
    options.append_parameter_override("over_exposure_threshold", over_thr);
    options.append_parameter_override("under_exposure_threshold", under_thr);
    options.append_parameter_override("build_only", false);
    node_ = std::make_shared<tl::TrafficLightClassifierNodelet>(options);
    return node_;
  }

  // Invoke the callback directly and capture what the node publishes. The
  // callback is re-invoked in a wait loop so the test is robust against
  // pub/sub discovery latency (the callback is pure w.r.t. its inputs).
  // Every caller expects a publish, so a capture failure is treated as a setup
  // failure and surfaced by throwing (helpers stay free of gtest assertions);
  // callers therefore need not re-check cap.got_signals.
  Captured process(
    const Image::ConstSharedPtr & image, const TrafficLightRoiArray::ConstSharedPtr & rois)
  {
    Captured cap;

    auto capture_node = std::make_shared<rclcpp::Node>("output_capture");
    auto signal_sub = capture_node->create_subscription<TrafficLightArray>(
      output_topic, rclcpp::QoS{1}, [&cap](TrafficLightArray::ConstSharedPtr msg) {
        cap.signals = *msg;
        cap.got_signals = true;
      });
    auto diag_sub = capture_node->create_subscription<DiagnosticArray>(
      "/diagnostics", rclcpp::QoS{10}, [&cap](DiagnosticArray::ConstSharedPtr msg) {
        for (const auto & status : msg->status) {
          if (status.hardware_id == node_name) {
            cap.diag = status;
            cap.got_diag = true;
          }
        }
      });

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(capture_node);

    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (!cap.got_signals && std::chrono::steady_clock::now() < deadline) {
      node_->imageRoiCallback(image, rois);
      exec.spin_some();
      std::this_thread::sleep_for(10ms);
    }
    // Drain a little more so diagnostics published in the same call are received.
    for (int i = 0; i < 20 && !cap.got_diag && std::chrono::steady_clock::now() < deadline; ++i) {
      node_->imageRoiCallback(image, rois);
      exec.spin_some();
      std::this_thread::sleep_for(5ms);
    }
    if (!cap.got_signals) {
      throw std::runtime_error(
        "process(): node under test published no traffic_signals within the timeout");
    }
    return cap;
  }

  // ColorClassifier leaves its HSV threshold members default-constructed
  // (cv::Scalar(0,0,0,0)) until a parameter is set at runtime: they are only
  // assigned in its on-set-parameter callback, which is registered AFTER the
  // initial declarations, so neither defaults nor overrides trigger it. Until
  // primed, every ROI is classified as UNKNOWN. Setting a parameter forces the
  // callback to populate all thresholds from the declared defaults so that
  // color classification becomes active.
  void prime_color_thresholds() { node_->set_parameter(rclcpp::Parameter("green_min_h", 50)); }

  // Run the callback briefly and assert nothing is published (used for the
  // early-return path). Returns true if a signal message was observed.
  bool observed_any_publish(
    const Image::ConstSharedPtr & image, const TrafficLightRoiArray::ConstSharedPtr & rois)
  {
    bool got = false;
    auto capture_node = std::make_shared<rclcpp::Node>("output_capture");
    auto sub = capture_node->create_subscription<TrafficLightArray>(
      output_topic, rclcpp::QoS{1}, [&got](TrafficLightArray::ConstSharedPtr) { got = true; });

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(capture_node);

    const auto deadline = std::chrono::steady_clock::now() + 500ms;
    while (!got && std::chrono::steady_clock::now() < deadline) {
      node_->imageRoiCallback(image, rois);
      exec.spin_some();
      std::this_thread::sleep_for(10ms);
    }
    return got;
  }

  std::shared_ptr<tl::TrafficLightClassifierNodelet> node_;
};

// --------------------------------------------------------------------------
// Empty ROI array -> empty signal array is published, input header is copied,
// and the early-return path skips diagnostics entirely.
// --------------------------------------------------------------------------
TEST_F(CharacterizationTest, EmptyRoisPublishEmptySignalsWithHeader)
{
  // Arrange
  make_node_under_test(car_type, no_over_threshold, no_under_threshold);
  auto image = make_dummy_image();
  auto rois = std::make_shared<TrafficLightRoiArray>();  // no rois

  // Act
  const auto cap = process(image, rois);

  // Assert
  EXPECT_EQ(cap.signals.signals.size(), 0u);
  // Output header is propagated from the input image (not the rois / not now()).
  EXPECT_EQ(cap.signals.header, image->header);
  // The exposure diagnostic reports per-ROI brightness; with no ROIs there is
  // nothing to assess, so the node publishes no diagnostic at all (not even OK).
  EXPECT_FALSE(cap.got_diag);
}

// --------------------------------------------------------------------------
// A ROI whose traffic_light_type differs from the node's target type is
// dropped: it is neither classified nor appended as UNKNOWN.
// --------------------------------------------------------------------------
TEST_F(CharacterizationTest, NonMatchingTypeRoiIsDropped)
{
  // Arrange
  make_node_under_test(car_type, no_over_threshold, no_under_threshold);
  auto image = make_dummy_image();
  auto rois = std::make_shared<TrafficLightRoiArray>();
  // Non-matching type (pedestrian) while the node targets car -- raw make_roi
  // since the mismatched type is the whole point of this case.
  rois->rois.push_back(make_roi(/*id=*/1, pedestrian_type, 0, 0, 16, 16));

  // Act
  const auto cap = process(image, rois);

  // Assert
  EXPECT_EQ(cap.signals.signals.size(), 0u);
}

// --------------------------------------------------------------------------
// A zero-sized ROI of the target type is appended to the output as UNKNOWN
// (id/type propagated, single UNKNOWN/UNKNOWN element, confidence 0).
// --------------------------------------------------------------------------
TEST_F(CharacterizationTest, ZeroSizedMatchingRoiAppendedAsUnknown)
{
  // Arrange
  make_node_under_test(car_type, no_over_threshold, no_under_threshold);
  auto image = make_dummy_image();
  auto rois = std::make_shared<TrafficLightRoiArray>();
  rois->rois.push_back(make_zero_sized_roi(/*id=*/1));

  // Act
  const auto cap = process(image, rois);

  // Assert
  ASSERT_EQ(cap.signals.signals.size(), 1u);
  const auto & signal = cap.signals.signals[0];
  EXPECT_EQ(signal.traffic_light_id, 1);
  EXPECT_EQ(signal.traffic_light_type, car_type);
  EXPECT_TRUE(is_unknown_signal(signal));
}

// --------------------------------------------------------------------------
// A valid, properly-exposed ROI of the target type is classified: exactly one
// element is emitted and id/type are propagated. The specific color is the
// ColorClassifier's concern and is intentionally not asserted here.
// --------------------------------------------------------------------------
TEST_F(CharacterizationTest, ValidRoiProducesExactlyOneElement)
{
  // Arrange
  make_node_under_test(car_type, no_over_threshold, no_under_threshold);
  auto image = make_dummy_image();
  auto rois = std::make_shared<TrafficLightRoiArray>();
  rois->rois.push_back(make_valid_roi(/*id=*/1));

  // Act
  const auto cap = process(image, rois);

  // Assert
  ASSERT_EQ(cap.signals.signals.size(), 1u);
  // Output header is propagated from the input image
  EXPECT_EQ(cap.signals.header, image->header);
  const auto & signal = cap.signals.signals[0];
  EXPECT_EQ(signal.traffic_light_id, 1);  // input id propagated to the slot
  EXPECT_EQ(signal.elements.size(), 1u);  // classifier produced exactly one element

  // No exposure issues -> OK, both exposure flags false.
  EXPECT_TRUE(has_exposure_diag(cap, DiagnosticStatus::OK, /*over=*/false, /*under=*/false));
}

// --------------------------------------------------------------------------
// Multiple valid ROIs (batch > 1) map to ordered output slots: signals are
// produced in input order, each carries its own id/type, and each yields
// exactly one element. (Lamp color is left UNKNOWN here -- thresholds are not
// primed -- so only structure/ids are pinned.)
// --------------------------------------------------------------------------
TEST_F(CharacterizationTest, MultipleValidRoisMapToOrderedSlots)
{
  // Arrange
  make_node_under_test(car_type, no_over_threshold, no_under_threshold);
  auto image = make_dummy_image(32, 16);
  auto rois = std::make_shared<TrafficLightRoiArray>();
  rois->rois.push_back(make_valid_roi(/*id=*/1));
  rois->rois.push_back(make_valid_roi(/*id=*/2, /*x=*/16));

  // Act
  const auto cap = process(image, rois);

  // Assert
  ASSERT_EQ(cap.signals.signals.size(), 2u);
  // input ids map to slots in order, each with exactly one element
  EXPECT_EQ(cap.signals.signals[0].traffic_light_id, 1);
  EXPECT_EQ(cap.signals.signals[0].elements.size(), 1u);
  EXPECT_EQ(cap.signals.signals[1].traffic_light_id, 2);
  EXPECT_EQ(cap.signals.signals[1].elements.size(), 1u);
}

// --------------------------------------------------------------------------
// Output order is NOT input order: classified (valid) signals fill the leading
// slots, then UNKNOWN signals from zero-sized ROIs are appended after them. The
// input is deliberately [zero-sized, valid] so the two reorder -- a naive
// input-order-preserving implementation would put the UNKNOWN first and fail.
// ids are preserved per slot.
// --------------------------------------------------------------------------
TEST_F(CharacterizationTest, ValidThenZeroSizedRoiOrdering)
{
  // Arrange
  make_node_under_test(car_type, no_over_threshold, no_under_threshold);
  auto image = make_dummy_image();
  auto rois = std::make_shared<TrafficLightRoiArray>();
  // Reversed relative to the expected output: zero-sized first, valid second.
  rois->rois.push_back(make_zero_sized_roi(/*id=*/1));
  rois->rois.push_back(make_valid_roi(/*id=*/2));

  // Act
  const auto cap = process(image, rois);

  // Assert
  ASSERT_EQ(cap.signals.signals.size(), 2u);

  // [0] classified valid ROI -- input-second, but classified ROIs lead the output
  EXPECT_EQ(cap.signals.signals[0].traffic_light_id, 2);
  EXPECT_EQ(cap.signals.signals[0].elements.size(), 1u);

  // [1] appended UNKNOWN for the zero-sized ROI -- input-first, but appended last
  EXPECT_EQ(cap.signals.signals[1].traffic_light_id, 1);
  EXPECT_TRUE(is_unknown_signal(cap.signals.signals[1]));
}

// --------------------------------------------------------------------------
// Over-exposed ROI: result is overwritten with UNKNOWN and a WARN diagnostic
// is published with detect_traffic_light_over_exposure == True. A solid `white`
// fill (brightness ~1.27) clears the 0.85 threshold; compute_brightness on real
// backlit images is covered separately in test_utils.
// --------------------------------------------------------------------------
TEST_F(CharacterizationTest, OverExposedRoiOverwrittenWithUnknownAndWarns)
{
  // Arrange
  make_node_under_test(car_type, /*over=*/0.85, /*under=*/no_under_threshold);
  auto image = make_solid_image(white);
  auto rois = std::make_shared<TrafficLightRoiArray>();
  rois->rois.push_back(make_valid_roi(/*id=*/1));  // full-frame ROI

  // Act
  const auto cap = process(image, rois);

  // Assert
  ASSERT_EQ(cap.signals.signals.size(), 1u);
  const auto & signal = cap.signals.signals[0];
  EXPECT_EQ(signal.traffic_light_id, 1);
  EXPECT_TRUE(is_unknown_signal(signal));

  EXPECT_TRUE(has_exposure_diag(cap, DiagnosticStatus::WARN, /*over=*/true, /*under=*/false));
}

// --------------------------------------------------------------------------
// Under-exposed ROI: same overwrite-to-UNKNOWN + WARN behavior, driven by
// detect_traffic_light_under_exposure. A solid `black` fill (brightness -1.0)
// falls below the -0.85 threshold; compute_brightness on real dimmed images is
// covered separately in test_utils.
// --------------------------------------------------------------------------
TEST_F(CharacterizationTest, UnderExposedRoiOverwrittenWithUnknownAndWarns)
{
  // Arrange
  make_node_under_test(car_type, /*over=*/no_over_threshold, /*under=*/-0.85);
  auto image = make_solid_image(black);
  auto rois = std::make_shared<TrafficLightRoiArray>();
  rois->rois.push_back(make_valid_roi(/*id=*/1));  // full-frame ROI

  // Act
  const auto cap = process(image, rois);

  // Assert
  ASSERT_EQ(cap.signals.signals.size(), 1u);
  const auto & signal = cap.signals.signals[0];
  EXPECT_EQ(signal.traffic_light_id, 1);
  EXPECT_TRUE(is_unknown_signal(signal));

  EXPECT_TRUE(has_exposure_diag(cap, DiagnosticStatus::WARN, /*over=*/false, /*under=*/true));
}

// --------------------------------------------------------------------------
// Exposure overwrite targets only the flagged slot. With [normal, over-exposed]
// ROIs, slot 0 keeps its classifier result (a real GREEN lamp) while slot 1 is
// forced to UNKNOWN. This pins exposure_out_of_range_indices targeting the
// correct signal index. Thresholds are primed so the normal slot is a distinct,
// non-UNKNOWN color and the "not overwritten" property is observable.
// --------------------------------------------------------------------------
TEST_F(CharacterizationTest, OnlyExposedSlotIsOverwritten)
{
  // Arrange
  make_node_under_test(car_type, /*over=*/0.85, /*under=*/no_under_threshold);
  prime_color_thresholds();
  // Left half green (normal, classifies as GREEN), right half white (over-exposed).
  auto image = make_left_right_image(green_lamp, white);
  auto rois = std::make_shared<TrafficLightRoiArray>();
  rois->rois.push_back(make_valid_roi(/*id=*/1));            // normal (left half)
  rois->rois.push_back(make_valid_roi(/*id=*/2, /*x=*/16));  // over-exposed (right half)

  // Act
  const auto cap = process(image, rois);

  // Assert
  ASSERT_EQ(cap.signals.signals.size(), 2u);
  // slot 0: classifier result preserved (not overwritten)
  EXPECT_EQ(cap.signals.signals[0].traffic_light_id, 1);
  ASSERT_EQ(cap.signals.signals[0].elements.size(), 1u);
  EXPECT_EQ(cap.signals.signals[0].elements[0].color, TrafficLightElement::GREEN);
  EXPECT_GT(cap.signals.signals[0].elements[0].confidence, 0.0f);
  // slot 1: overwritten with UNKNOWN by the exposure handling
  EXPECT_EQ(cap.signals.signals[1].traffic_light_id, 2);
  EXPECT_TRUE(is_unknown_signal(cap.signals.signals[1]));

  EXPECT_TRUE(has_exposure_diag(cap, DiagnosticStatus::WARN, /*over=*/true, /*under=*/false));
}

// --------------------------------------------------------------------------
// Over- and under-exposure detected in the same call: both flagged slots are
// overwritten with UNKNOWN and the diagnostic reports both conditions True.
// --------------------------------------------------------------------------
TEST_F(CharacterizationTest, OverAndUnderExposureInSameCall)
{
  // Arrange
  make_node_under_test(car_type, /*over=*/0.85, /*under=*/-0.85);
  // Left half white (over-exposed), right half black (under-exposed).
  auto image = make_left_right_image(white, black);
  auto rois = std::make_shared<TrafficLightRoiArray>();
  rois->rois.push_back(make_valid_roi(/*id=*/1));            // over-exposed (left half, white)
  rois->rois.push_back(make_valid_roi(/*id=*/2, /*x=*/16));  // under-exposed (right half, black)

  // Act
  const auto cap = process(image, rois);

  // Assert
  ASSERT_EQ(cap.signals.signals.size(), 2u);
  EXPECT_TRUE(is_unknown_signal(cap.signals.signals[0]));  // over-exposed (left half)
  EXPECT_TRUE(is_unknown_signal(cap.signals.signals[1]));  // under-exposed (right half)

  EXPECT_TRUE(has_exposure_diag(cap, DiagnosticStatus::WARN, /*over=*/true, /*under=*/true));
}

// --------------------------------------------------------------------------
// An unknown classifier_type leaves classifier_ptr_ unset; the callback returns
// early and publishes nothing at all (not even an empty message). This pins a
// defensive guard: a misconfigured classifier degrades to a silent no-op rather
// than a crash or an empty publish.
// --------------------------------------------------------------------------
TEST_F(CharacterizationTest, UnknownClassifierTypePublishesNothing)
{
  // Arrange
  make_node_under_test(car_type, no_over_threshold, no_under_threshold, /*classifier_type=*/99);
  auto image = make_dummy_image();
  auto rois = std::make_shared<TrafficLightRoiArray>();
  rois->rois.push_back(make_valid_roi(/*id=*/1));

  // Act
  const bool published = observed_any_publish(image, rois);

  // Assert
  EXPECT_FALSE(published);
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
