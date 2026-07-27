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
// Unit tests for the ROS-free classification core (TrafficLightClassifier).
//
// classify() is exercised directly over an in-memory cv::Mat and a
// TrafficLightRoiArray, with a hand-written FakeClassifier standing in for the
// real backend. This isolates the per-ROI orchestration (type filtering,
// cropping, exposure handling, UNKNOWN-append, ordering, the classifier-failure
// early return) from rclcpp, message_filters and the actual color/CNN models, so
// the assertions are on classify()'s return value rather than on what a node
// happens to publish.
//
// What is intentionally NOT covered here (it belongs to other layers):
//   * The concrete lamp color/shape a real backend assigns -- the fake stamps a
//     fixed color, so only the slots/ids/exposure structure is pinned.
//   * compute_brightness on real camera frames -- covered by test_utils.
//   * Node wiring (pub/sub, diagnostics, header stamping, sync) -- covered by
//     test_traffic_light_classifier_integration.
//

#include "../src/classifier/classifier_interface.hpp"
#include "../src/traffic_light_classifier.hpp"

#include <opencv2/core/core.hpp>

#include <std_msgs/msg/header.hpp>
#include <tier4_perception_msgs/msg/traffic_light_array.hpp>
#include <tier4_perception_msgs/msg/traffic_light_element.hpp>
#include <tier4_perception_msgs/msg/traffic_light_roi_array.hpp>

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <vector>

namespace
{
namespace tl = autoware::traffic_light;

using tier4_perception_msgs::msg::TrafficLight;
using tier4_perception_msgs::msg::TrafficLightElement;
using tier4_perception_msgs::msg::TrafficLightRoi;
using tier4_perception_msgs::msg::TrafficLightRoiArray;

// CAR == 0 is the classification target used throughout; PEDESTRIAN is the
// non-target type used to exercise the drop path.
constexpr uint8_t car_type = TrafficLight::CAR_TRAFFIC_LIGHT;
constexpr uint8_t pedestrian_type = TrafficLight::PEDESTRIAN_TRAFFIC_LIGHT;

// Wide thresholds so compute_brightness (range ~[-1.0, 1.27]) never flags a
// crop. Used by every test that is not about exposure.
constexpr double no_over_threshold = 2.0;
constexpr double no_under_threshold = -2.0;

// rgb8 fill colors (R, G, B). white/black drive the exposure paths; gray is a
// neutral fill that never trips a threshold; red/blue tag the two halves of a
// split image so the crop geometry handed to the backend is observable.
const cv::Scalar white(255, 255, 255);
const cv::Scalar black(0, 0, 0);
const cv::Scalar gray(100, 100, 100);
const cv::Scalar red(255, 0, 0);
const cv::Scalar blue(0, 0, 255);

// A backend stand-in for ClassifierInterface. It records what classify() hands
// it and, on success, stamps every pre-populated signal slot with one element
// of a fixed color -- mirroring the real backends' contract (one element per
// image, id/type left as classify() set them). The behavior knobs are public so
// each test can configure failure or a distinctive color before calling.
class FakeClassifier : public tl::ClassifierInterface
{
public:
  bool succeed = true;
  uint8_t element_color = TrafficLightElement::GREEN;
  float element_confidence = 0.9f;

  int call_count = 0;
  std::vector<cv::Mat> received_images;
  // Recorded by make_debug_image so a test can confirm the wrapper forwards roi_images here.
  mutable std::vector<cv::Mat> debug_images_received;

  bool getTrafficSignals(
    const std::vector<cv::Mat> & images,
    tier4_perception_msgs::msg::TrafficLightArray & traffic_signals) override
  {
    ++call_count;
    for (const auto & image : images) {
      received_images.push_back(image.clone());
    }
    if (!succeed) {
      return false;
    }
    for (auto & signal : traffic_signals.signals) {
      TrafficLightElement element;
      element.color = element_color;
      element.shape = TrafficLightElement::CIRCLE;
      element.confidence = element_confidence;
      signal.elements.push_back(element);
    }
    return true;
  }

  cv::Mat make_debug_image(const std::vector<cv::Mat> & images) const override
  {
    debug_images_received = images;
    // The row count encodes the batch size so callers can verify what was forwarded.
    return cv::Mat(static_cast<int>(images.size()), 1, CV_8UC3, cv::Scalar(0, 0, 0));
  }
};

cv::Mat make_solid_image(const cv::Scalar & color, int width = 16, int height = 16)
{
  return cv::Mat(height, width, CV_8UC3, color);
}

// A 32x16 image split into two 16x16 halves: left painted `left`, right painted
// `right`. Pairs with two side-by-side ROIs (x=0 and x=16) so each ROI crops a
// distinctly colored region.
cv::Mat make_left_right_image(const cv::Scalar & left, const cv::Scalar & right)
{
  cv::Mat mat(/*rows=*/16, /*cols=*/32, CV_8UC3, left);
  mat(cv::Rect(/*x=*/16, /*y=*/0, /*width=*/16, /*height=*/16)).setTo(right);
  return mat;
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
// Geometry defaults to a full-frame 16x16 region; pass x when laying out
// multiple ROIs side by side.
TrafficLightRoi make_valid_roi(
  int64_t id, uint32_t x = 0, uint32_t y = 0, uint32_t width = 16, uint32_t height = 16)
{
  return make_roi(id, car_type, x, y, width, height);
}

// A zero-sized ROI of the target type: treated as undetected and appended as
// UNKNOWN (never handed to the backend).
TrafficLightRoi make_zero_sized_roi(int64_t id)
{
  return make_roi(id, car_type, 0, 0, 0, 0);
}

// A signal whose single element was reset to UNKNOWN: color and shape UNKNOWN
// with confidence 0 -- the setSignalUnknown shape, produced both for undetected
// (zero-sized) ROIs and for exposure-overwritten slots. Returns an
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

class TrafficLightClassifierTest : public ::testing::Test
{
protected:
  // Build a core bound to a fresh FakeClassifier (reachable afterwards via fake_classifier_
  // for behavior knobs and recorded inputs).
  tl::TrafficLightClassifier make_classifier(
    double over_threshold, double under_threshold, uint8_t classify_type = car_type)
  {
    fake_classifier_ = std::make_shared<FakeClassifier>();
    return tl::TrafficLightClassifier(
      fake_classifier_, classify_type, over_threshold, under_threshold);
  }

  std::shared_ptr<FakeClassifier> fake_classifier_;
};

// --------------------------------------------------------------------------
// roi_images is the *classified subset* -- the valid, target-type ROIs in
// classification order -- NOT the input ROIs and NOT the post-processed output
// signals (which also carry appended-UNKNOWN slots). make_debug_image forwards
// exactly that subset to the backend. This 1:1 alignment between roi_images[i]
// and the backend's per-image view is what lets the node render debug crops
// correctly; if roi_images were ever set to the final signals, debug labels and
// crops would silently fall out of sync.
// --------------------------------------------------------------------------
TEST_F(TrafficLightClassifierTest, RoiImagesAreClassifiedSubsetForwardedToDebug)
{
  // Arrange: three ROIs of which only one is classified --
  //   id=1 valid car      -> classified (the sole entry in roi_images)
  //   id=2 zero-sized car  -> appended to signals as UNKNOWN, NOT in roi_images
  //   id=3 pedestrian type -> dropped entirely (in neither)
  auto classifier = make_classifier(no_over_threshold, no_under_threshold);
  const auto image = make_solid_image(gray);
  TrafficLightRoiArray rois;
  rois.rois.push_back(make_valid_roi(/*id=*/1));
  rois.rois.push_back(make_zero_sized_roi(/*id=*/2));
  rois.rois.push_back(make_roi(/*id=*/3, pedestrian_type, 0, 0, 16, 16));

  // Act
  const auto result = classifier.classify(image, rois);
  ASSERT_TRUE(result.has_value());
  classifier.make_debug_image(result->roi_images);

  // Assert: roi_images holds only the 1 classified ROI -- distinct from both the 3
  // inputs and the 2 output signals (classified + appended-UNKNOWN) -- and that same
  // subset is exactly what make_debug_image hands the backend.
  EXPECT_EQ(result->roi_images.size(), 1u);
  EXPECT_EQ(result->signals.signals.size(), 2u);
  EXPECT_EQ(fake_classifier_->debug_images_received.size(), 1u);
}

// --------------------------------------------------------------------------
// No ROIs -> empty result, no exposure flags, and the backend is never invoked
// (the images batch is empty, so classify() skips it).
// --------------------------------------------------------------------------
TEST_F(TrafficLightClassifierTest, EmptyRoisYieldEmptyResult)
{
  // Arrange
  auto classifier = make_classifier(no_over_threshold, no_under_threshold);
  const auto image = make_solid_image(gray);
  const TrafficLightRoiArray rois;  // no rois

  // Act
  const auto result = classifier.classify(image, rois);

  // Assert
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->signals.signals.size(), 0u);
  EXPECT_FALSE(result->detected_over_exposure);
  EXPECT_FALSE(result->detected_under_exposure);
  EXPECT_EQ(fake_classifier_->call_count, 0);
}

// --------------------------------------------------------------------------
// A ROI whose type differs from the target is dropped: not classified, not
// appended as UNKNOWN, and the backend sees nothing.
// --------------------------------------------------------------------------
TEST_F(TrafficLightClassifierTest, NonMatchingTypeRoiIsDropped)
{
  // Arrange
  auto classifier = make_classifier(no_over_threshold, no_under_threshold);
  const auto image = make_solid_image(gray);
  TrafficLightRoiArray rois;
  rois.rois.push_back(make_roi(/*id=*/1, pedestrian_type, 0, 0, 16, 16));

  // Act
  const auto result = classifier.classify(image, rois);

  // Assert
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->signals.signals.size(), 0u);
  EXPECT_EQ(fake_classifier_->call_count, 0);
}

// --------------------------------------------------------------------------
// A zero-sized ROI of the target type is appended as UNKNOWN (id/type
// propagated, single UNKNOWN element); it is never handed to the backend.
// --------------------------------------------------------------------------
TEST_F(TrafficLightClassifierTest, ZeroSizedMatchingRoiAppendedAsUnknown)
{
  // Arrange
  auto classifier = make_classifier(no_over_threshold, no_under_threshold);
  const auto image = make_solid_image(gray);
  TrafficLightRoiArray rois;
  rois.rois.push_back(make_zero_sized_roi(/*id=*/1));

  // Act
  const auto result = classifier.classify(image, rois);

  // Assert
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->signals.signals.size(), 1u);
  const auto & signal = result->signals.signals[0];
  EXPECT_EQ(signal.traffic_light_id, 1);
  EXPECT_EQ(signal.traffic_light_type, car_type);
  EXPECT_TRUE(is_unknown_signal(signal));
  EXPECT_EQ(fake_classifier_->call_count, 0);
}

// --------------------------------------------------------------------------
// A valid ROI is classified into exactly one element with id/type propagated.
// The result header is left default-constructed: stamping it is the node's job,
// and classify() must not touch it.
// --------------------------------------------------------------------------
TEST_F(TrafficLightClassifierTest, ValidRoiIsClassifiedAndHeaderLeftUnset)
{
  // Arrange
  auto classifier = make_classifier(no_over_threshold, no_under_threshold);
  const auto image = make_solid_image(gray);
  TrafficLightRoiArray rois;
  rois.rois.push_back(make_valid_roi(/*id=*/7));

  // Act
  const auto result = classifier.classify(image, rois);

  // Assert
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->signals.signals.size(), 1u);
  const auto & signal = result->signals.signals[0];
  EXPECT_EQ(signal.traffic_light_id, 7);
  EXPECT_EQ(signal.traffic_light_type, car_type);
  ASSERT_EQ(signal.elements.size(), 1u);
  EXPECT_EQ(signal.elements[0].color, TrafficLightElement::GREEN);  // backend result preserved
  EXPECT_EQ(fake_classifier_->call_count, 1);
  // Header intentionally untouched by the core.
  EXPECT_EQ(result->signals.header, std_msgs::msg::Header{});
}

// --------------------------------------------------------------------------
// Multiple valid ROIs map to ordered output slots: signals are produced in
// input order, each carrying its own id and exactly one element.
// --------------------------------------------------------------------------
TEST_F(TrafficLightClassifierTest, MultipleValidRoisMapToOrderedSlots)
{
  // Arrange
  auto classifier = make_classifier(no_over_threshold, no_under_threshold);
  const auto image = make_solid_image(gray, /*width=*/32);
  TrafficLightRoiArray rois;
  rois.rois.push_back(make_valid_roi(/*id=*/1));
  rois.rois.push_back(make_valid_roi(/*id=*/2, /*x=*/16));

  // Act
  const auto result = classifier.classify(image, rois);

  // Assert
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->signals.signals.size(), 2u);
  EXPECT_EQ(result->signals.signals[0].traffic_light_id, 1);
  EXPECT_EQ(result->signals.signals[0].elements.size(), 1u);
  EXPECT_EQ(result->signals.signals[1].traffic_light_id, 2);
  EXPECT_EQ(result->signals.signals[1].elements.size(), 1u);
}

// --------------------------------------------------------------------------
// Output order is not input order: classified (valid) signals lead, then
// UNKNOWNs from zero-sized ROIs are appended. The input is [zero-sized, valid]
// so the two reorder -- a naive input-order implementation would fail. ids are
// preserved per slot.
// --------------------------------------------------------------------------
TEST_F(TrafficLightClassifierTest, ValidThenZeroSizedRoiOrdering)
{
  // Arrange
  auto classifier = make_classifier(no_over_threshold, no_under_threshold);
  const auto image = make_solid_image(gray);
  TrafficLightRoiArray rois;
  rois.rois.push_back(make_zero_sized_roi(/*id=*/1));  // appended last as UNKNOWN
  rois.rois.push_back(make_valid_roi(/*id=*/2));       // classified, leads the output

  // Act
  const auto result = classifier.classify(image, rois);

  // Assert
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->signals.signals.size(), 2u);
  EXPECT_EQ(result->signals.signals[0].traffic_light_id, 2);
  EXPECT_EQ(result->signals.signals[0].elements.size(), 1u);
  EXPECT_EQ(result->signals.signals[1].traffic_light_id, 1);
  EXPECT_TRUE(is_unknown_signal(result->signals.signals[1]));
}

// --------------------------------------------------------------------------
// The image region handed to the backend matches each ROI's geometry: with a
// split image and two side-by-side ROIs, the backend receives two crops of the
// ROI size, in ROI order, each carrying its half's color. This pins the
// image(cv::Rect(x_offset, y_offset, width, height)) crop.
// --------------------------------------------------------------------------
TEST_F(TrafficLightClassifierTest, CropHandedToBackendMatchesRoiGeometry)
{
  // Arrange
  auto classifier = make_classifier(no_over_threshold, no_under_threshold);
  const auto image = make_left_right_image(red, blue);
  TrafficLightRoiArray rois;
  rois.rois.push_back(make_valid_roi(/*id=*/1));            // left half (red)
  rois.rois.push_back(make_valid_roi(/*id=*/2, /*x=*/16));  // right half (blue)

  // Act
  const auto result = classifier.classify(image, rois);

  // Assert
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(fake_classifier_->received_images.size(), 2u);
  EXPECT_EQ(fake_classifier_->received_images[0].size(), cv::Size(16, 16));
  EXPECT_EQ(fake_classifier_->received_images[1].size(), cv::Size(16, 16));
  // Pixel (channels R,G,B) confirms the crop landed on the correct half.
  EXPECT_EQ(fake_classifier_->received_images[0].at<cv::Vec3b>(0, 0), cv::Vec3b(255, 0, 0));  // red
  EXPECT_EQ(
    fake_classifier_->received_images[1].at<cv::Vec3b>(0, 0), cv::Vec3b(0, 0, 255));  // blue
}

// --------------------------------------------------------------------------
// An over-exposed ROI is overwritten with UNKNOWN and detected_over_exposure is
// raised. A solid `white` crop (brightness ~1.27) clears the 0.85 threshold.
// --------------------------------------------------------------------------
TEST_F(TrafficLightClassifierTest, OverExposedRoiOverwrittenWithUnknownAndFlagged)
{
  // Arrange
  auto classifier = make_classifier(/*over=*/0.85, no_under_threshold);
  const auto image = make_solid_image(white);
  TrafficLightRoiArray rois;
  rois.rois.push_back(make_valid_roi(/*id=*/1));

  // Act
  const auto result = classifier.classify(image, rois);

  // Assert
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->signals.signals.size(), 1u);
  EXPECT_EQ(result->signals.signals[0].traffic_light_id, 1);
  EXPECT_TRUE(is_unknown_signal(result->signals.signals[0]));
  EXPECT_TRUE(result->detected_over_exposure);
  EXPECT_FALSE(result->detected_under_exposure);
}

// --------------------------------------------------------------------------
// An under-exposed ROI gets the same overwrite-to-UNKNOWN treatment, flagged
// via detected_under_exposure. A solid `black` crop (brightness -1.0) falls
// below the -0.85 threshold.
// --------------------------------------------------------------------------
TEST_F(TrafficLightClassifierTest, UnderExposedRoiOverwrittenWithUnknownAndFlagged)
{
  // Arrange
  auto classifier = make_classifier(no_over_threshold, /*under=*/-0.85);
  const auto image = make_solid_image(black);
  TrafficLightRoiArray rois;
  rois.rois.push_back(make_valid_roi(/*id=*/1));

  // Act
  const auto result = classifier.classify(image, rois);

  // Assert
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->signals.signals.size(), 1u);
  EXPECT_EQ(result->signals.signals[0].traffic_light_id, 1);
  EXPECT_TRUE(is_unknown_signal(result->signals.signals[0]));
  EXPECT_FALSE(result->detected_over_exposure);
  EXPECT_TRUE(result->detected_under_exposure);
}

// --------------------------------------------------------------------------
// Exposure overwrite targets only the flagged slot. With [normal, over-exposed]
// ROIs, slot 0 keeps its backend result while slot 1 is forced to UNKNOWN --
// pinning that exposure_out_of_range_indices addresses the correct signal slot.
// --------------------------------------------------------------------------
TEST_F(TrafficLightClassifierTest, OnlyExposedSlotIsOverwritten)
{
  // Arrange
  auto classifier = make_classifier(/*over=*/0.85, no_under_threshold);
  const auto image = make_left_right_image(gray, white);  // left normal, right over-exposed
  TrafficLightRoiArray rois;
  rois.rois.push_back(make_valid_roi(/*id=*/1));            // normal (left half)
  rois.rois.push_back(make_valid_roi(/*id=*/2, /*x=*/16));  // over-exposed (right half)

  // Act
  const auto result = classifier.classify(image, rois);

  // Assert
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->signals.signals.size(), 2u);
  // slot 0: backend result preserved (not overwritten).
  EXPECT_EQ(result->signals.signals[0].traffic_light_id, 1);
  ASSERT_EQ(result->signals.signals[0].elements.size(), 1u);
  EXPECT_EQ(result->signals.signals[0].elements[0].color, TrafficLightElement::GREEN);
  EXPECT_GT(result->signals.signals[0].elements[0].confidence, 0.0f);
  // slot 1: overwritten with UNKNOWN by the exposure handling.
  EXPECT_EQ(result->signals.signals[1].traffic_light_id, 2);
  EXPECT_TRUE(is_unknown_signal(result->signals.signals[1]));

  EXPECT_TRUE(result->detected_over_exposure);
  EXPECT_FALSE(result->detected_under_exposure);
}

// --------------------------------------------------------------------------
// Over- and under-exposure in the same call: both flagged slots are overwritten
// with UNKNOWN and both flags are raised.
// --------------------------------------------------------------------------
TEST_F(TrafficLightClassifierTest, OverAndUnderExposureInSameCall)
{
  // Arrange
  auto classifier = make_classifier(/*over=*/0.85, /*under=*/-0.85);
  const auto image = make_left_right_image(white, black);  // left over, right under
  TrafficLightRoiArray rois;
  rois.rois.push_back(make_valid_roi(/*id=*/1));            // over-exposed (left half)
  rois.rois.push_back(make_valid_roi(/*id=*/2, /*x=*/16));  // under-exposed (right half)

  // Act
  const auto result = classifier.classify(image, rois);

  // Assert
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->signals.signals.size(), 2u);
  EXPECT_TRUE(is_unknown_signal(result->signals.signals[0]));
  EXPECT_TRUE(is_unknown_signal(result->signals.signals[1]));
  EXPECT_TRUE(result->detected_over_exposure);
  EXPECT_TRUE(result->detected_under_exposure);
}

// --------------------------------------------------------------------------
// When the backend reports failure, classify() returns nullopt so the caller can
// skip publishing. (Impractical to drive via the real HSV backend, hence pinned
// here with the fake.)
// --------------------------------------------------------------------------
TEST_F(TrafficLightClassifierTest, BackendFailureReturnsNullopt)
{
  // Arrange
  auto classifier = make_classifier(no_over_threshold, no_under_threshold);
  fake_classifier_->succeed = false;
  const auto image = make_solid_image(gray);
  TrafficLightRoiArray rois;
  rois.rois.push_back(make_valid_roi(/*id=*/1));

  // Act
  const auto result = classifier.classify(image, rois);

  // Assert
  EXPECT_FALSE(result.has_value());
  EXPECT_EQ(fake_classifier_->call_count, 1);
}

}  // namespace

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
