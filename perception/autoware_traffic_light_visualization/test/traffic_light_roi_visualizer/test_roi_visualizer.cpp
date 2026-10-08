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

// Unit tests for TrafficLightRoiVisualizer, the drawing separated out of the node. They call it
// directly, so there is no executor, no topic and no waiting: an image message and the detection
// messages go in, the drawn image comes back, and the tests read its pixels.

#include "traffic_light_roi_visualizer/roi_visualizer.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <tier4_perception_msgs/msg/traffic_light_array.hpp>
#include <tier4_perception_msgs/msg/traffic_light_roi_array.hpp>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>  // for ROS 2 Jazzy or newer
#else
#include <cv_bridge/cv_bridge.h>  // for ROS 2 Humble or older
#endif

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace
{
using autoware::traffic_light::TrafficLightRoiVisualizer;
using sensor_msgs::msg::Image;
using tier4_perception_msgs::msg::TrafficLightArray;
using tier4_perception_msgs::msg::TrafficLightElement;
using tier4_perception_msgs::msg::TrafficLightRoi;
using tier4_perception_msgs::msg::TrafficLightRoiArray;

using Pixel = std::array<uint8_t, 3>;

// Large enough for the label box drawn above a ROI (about 86 x 27 px for one shape at 87%):
// draw_shape() gives up silently when that box would fall outside the image.
constexpr int image_width = 640;
constexpr int image_height = 480;
constexpr Pixel background_rgb{40, 40, 40};

constexpr int64_t signal_id = 42;
constexpr int64_t other_signal_id = 43;

struct Box
{
  int x;
  int y;
  int width;
  int height;
};

// The rough box encloses the fine one, and the two corners the tests probe - the rough top left
// and the fine bottom left - each lie on one box only.
constexpr Box fine_box{200, 150, 40, 90};
constexpr Box rough_box{190, 140, 60, 110};

// Colors the visualizer draws with. Measured from the real drawing on 2026-09-10
// (ROS 2 Jazzy / Ubuntu 24.04 / OpenCV 4.6).
constexpr Pixel red_signal_rgb{254, 149, 149};    // str_to_color("red")
constexpr Pixel amber_signal_rgb{254, 250, 149};  // str_to_color("yellow")
constexpr Pixel green_signal_rgb{149, 254, 161};  // str_to_color("green")
// A label with no circle in it leaves extract_shape_info() at its initial value, which is also
// what a ROI with no signal at all is drawn with - the two are the same color in the output.
constexpr Pixel no_circle_rgb{255, 255, 255};
constexpr Pixel unknown_circle_rgb{250, 250, 250};  // str_to_color() fallback
constexpr Pixel label_icon_rgb{0, 0, 0};            // the shape icon inside the label box

std::string shape_image_dir()
{
  return ament_index_cpp::get_package_share_directory("autoware_traffic_light_visualization") +
         "/images/";
}

Image make_image(const std::string & encoding, const Pixel & color)
{
  Image image;
  image.header.frame_id = "camera";
  image.header.stamp.sec = 123;
  image.header.stamp.nanosec = 456;
  image.height = image_height;
  image.width = image_width;
  image.encoding = encoding;
  image.step = image_width * 3;
  image.data.resize(static_cast<size_t>(image_height) * image.step);
  for (size_t i = 0; i < image.data.size(); i += 3) {
    image.data[i] = color[0];
    image.data[i + 1] = color[1];
    image.data[i + 2] = color[2];
  }
  return image;
}

TrafficLightRoiArray make_rois(int64_t traffic_light_id, const std::vector<Box> & boxes)
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

TrafficLightArray make_signals(
  int64_t traffic_light_id, const std::vector<std::pair<uint8_t, uint8_t>> & color_and_shapes,
  float confidence = 0.87f)
{
  TrafficLightArray array;
  tier4_perception_msgs::msg::TrafficLight signal;
  signal.traffic_light_id = traffic_light_id;
  for (const auto & [color, shape] : color_and_shapes) {
    TrafficLightElement element;
    element.color = color;
    element.shape = shape;
    element.confidence = confidence;
    signal.elements.push_back(element);
  }
  array.signals.push_back(signal);
  return array;
}

TrafficLightArray make_signal(
  int64_t traffic_light_id, uint8_t color, uint8_t shape, float confidence = 0.87f)
{
  return make_signals(traffic_light_id, {{color, shape}}, confidence);
}

Pixel pixel_at(const Image & image, int x, int y)
{
  const size_t offset = static_cast<size_t>(y) * image.step + static_cast<size_t>(x) * 3;
  return {image.data.at(offset), image.data.at(offset + 1), image.data.at(offset + 2)};
}

// Two points in the strip above a ROI, where the label box is drawn. Offsets measured on
// 2026-09-15.
//
// The fill is probed near the top of the box rather than in the middle of it, and near its left
// edge rather than to the right. Both matter. The box is only as wide as the icon and the
// confidence it holds - about 54 px with neither - so a point chosen to clear a full one misses a
// narrow one, and a test asserting the box is absent would pass while it is drawn. And the id
// text, written on the same corner, reaches 22 rows above the ROI while the box reaches 27, so
// the top rows of the box are the only ones nothing else can write to.
Pixel label_box_pixel(const Image & image, const Box & box)
{
  return pixel_at(image, box.x + 5, box.y - 25);
}
Pixel label_icon_pixel(const Image & image, const Box & box)
{
  return pixel_at(image, box.x + 13, box.y - 13);
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

TrafficLightRoiVisualizer make_visualizer()
{
  return TrafficLightRoiVisualizer(shape_image_dir());
}

const Image background_image = make_image("rgb8", background_rgb);
const TrafficLightRoiArray fine_rois = make_rois(signal_id, {fine_box});
const TrafficLightRoiArray rough_rois = make_rois(signal_id, {rough_box});
const TrafficLightRoiArray no_rois = make_rois(signal_id, {});
const TrafficLightArray no_signals;
const TrafficLightArray green_signal =
  make_signal(signal_id, TrafficLightElement::GREEN, TrafficLightElement::CIRCLE);

}  // namespace

// ---------------------------------------------------------------------------------------------
// visualize(): the fine ROIs are the subject, one frame per ROI.
// ---------------------------------------------------------------------------------------------

TEST(TrafficLightRoiVisualizer, NoRoisLeavesImageUntouched)
{
  // Arrange
  const auto visualizer = make_visualizer();

  // Act
  const auto output = visualizer.visualize(background_image, no_rois, no_signals);
  ASSERT_NE(output, nullptr);

  // Assert
  EXPECT_EQ(count_pixels_differing_from(*output, background_rgb), 0u);
}

TEST(TrafficLightRoiVisualizer, OutputKeepsSizeEncodingAndHeaderOfInput)
{
  // Arrange
  const auto visualizer = make_visualizer();

  // Act
  const auto output = visualizer.visualize(background_image, fine_rois, green_signal);
  ASSERT_NE(output, nullptr);

  // Assert: the drawing changes the pixels and nothing else about the message
  EXPECT_EQ(output->width, background_image.width);
  EXPECT_EQ(output->height, background_image.height);
  EXPECT_EQ(output->encoding, "rgb8");
  EXPECT_EQ(output->header.frame_id, background_image.header.frame_id);
  EXPECT_EQ(output->header.stamp.sec, background_image.header.stamp.sec);
  EXPECT_EQ(output->header.stamp.nanosec, background_image.header.stamp.nanosec);
}

TEST(TrafficLightRoiVisualizer, RoiWithoutSignalGetsFrameButNoLabelBox)
{
  // Arrange
  const auto visualizer = make_visualizer();

  // Act
  const auto output = visualizer.visualize(background_image, fine_rois, no_signals);
  ASSERT_NE(output, nullptr);

  // Assert: the frame is drawn, in the color used when nothing is known about the signal
  const auto frame_corner = pixel_at(*output, fine_box.x, fine_box.y);
  EXPECT_EQ(frame_corner, no_circle_rgb);

  // The label box is not drawn
  const auto above_the_roi = label_box_pixel(*output, fine_box);
  EXPECT_EQ(above_the_roi, background_rgb);
}

TEST(TrafficLightRoiVisualizer, RoiWithSignalGetsFrameAndLabelBox)
{
  // Arrange
  const auto visualizer = make_visualizer();

  // Act
  const auto output = visualizer.visualize(background_image, fine_rois, green_signal);
  ASSERT_NE(output, nullptr);

  // Assert: the frame takes the color of the signal
  const auto frame_corner = pixel_at(*output, fine_box.x, fine_box.y);
  EXPECT_EQ(frame_corner, green_signal_rgb);

  // The label box is drawn above the ROI, with the shape icon in it
  const auto above_the_roi = label_box_pixel(*output, fine_box);
  const auto icon_above_the_roi = label_icon_pixel(*output, fine_box);
  EXPECT_EQ(above_the_roi, green_signal_rgb);
  EXPECT_EQ(icon_above_the_roi, label_icon_rgb);

  // The inside of the ROI is left alone: only the frame is drawn, not a fill
  const auto roi_interior =
    pixel_at(*output, fine_box.x + fine_box.width / 2, fine_box.y + fine_box.height / 2);
  EXPECT_EQ(roi_interior, background_rgb);
}

TEST(TrafficLightRoiVisualizer, SignalReportedForAnotherIdIsNotUsed)
{
  // Arrange
  const auto other =
    make_signal(other_signal_id, TrafficLightElement::GREEN, TrafficLightElement::CIRCLE);
  const auto visualizer = make_visualizer();

  // Act
  const auto output = visualizer.visualize(background_image, fine_rois, other);
  ASSERT_NE(output, nullptr);

  // Assert: the ROI is drawn as if there were no signal at all
  const auto frame_corner = pixel_at(*output, fine_box.x, fine_box.y);
  EXPECT_EQ(frame_corner, no_circle_rgb);
  const auto above_the_roi = label_box_pixel(*output, fine_box);
  EXPECT_EQ(above_the_roi, background_rgb);
}

TEST(TrafficLightRoiVisualizer, EveryRoiInArrayIsDrawn)
{
  // Arrange
  constexpr Box second_box{400, 150, 40, 90};
  const auto two = make_rois(signal_id, {fine_box, second_box});
  const auto visualizer = make_visualizer();

  // Act
  const auto output = visualizer.visualize(background_image, two, no_signals);
  ASSERT_NE(output, nullptr);

  // Assert: the top left corner of each ROI, which is where its frame starts
  const auto first_frame = pixel_at(*output, fine_box.x, fine_box.y);
  const auto second_frame = pixel_at(*output, second_box.x, second_box.y);
  EXPECT_EQ(first_frame, no_circle_rgb);
  EXPECT_EQ(second_frame, no_circle_rgb);
}

// ---------------------------------------------------------------------------------------------
// The frame color, which comes from the signal alone and is shared by both entry points.
// ---------------------------------------------------------------------------------------------

TEST(TrafficLightRoiVisualizer, RedCircleColorsFrameRed)
{
  // Arrange
  const auto signal = make_signal(signal_id, TrafficLightElement::RED, TrafficLightElement::CIRCLE);
  const auto visualizer = make_visualizer();

  // Act
  const auto output = visualizer.visualize(background_image, fine_rois, signal);
  ASSERT_NE(output, nullptr);

  // Assert
  const auto frame_corner = pixel_at(*output, fine_box.x, fine_box.y);
  EXPECT_EQ(frame_corner, red_signal_rgb);
}

TEST(TrafficLightRoiVisualizer, AmberCircleColorsFrameAmber)
{
  // Arrange
  const auto signal =
    make_signal(signal_id, TrafficLightElement::AMBER, TrafficLightElement::CIRCLE);
  const auto visualizer = make_visualizer();

  // Act
  const auto output = visualizer.visualize(background_image, fine_rois, signal);
  ASSERT_NE(output, nullptr);

  // Assert
  const auto frame_corner = pixel_at(*output, fine_box.x, fine_box.y);
  EXPECT_EQ(frame_corner, amber_signal_rgb);
}

TEST(TrafficLightRoiVisualizer, GreenCircleColorsFrameGreen)
{
  // Arrange
  const auto visualizer = make_visualizer();

  // Act
  const auto output = visualizer.visualize(background_image, fine_rois, green_signal);
  ASSERT_NE(output, nullptr);

  // Assert
  const auto frame_corner = pixel_at(*output, fine_box.x, fine_box.y);
  EXPECT_EQ(frame_corner, green_signal_rgb);
}

TEST(TrafficLightRoiVisualizer, UnknownCircleFallsBackToOffWhite)
{
  // Arrange
  const auto signal =
    make_signal(signal_id, TrafficLightElement::UNKNOWN, TrafficLightElement::CIRCLE);
  const auto visualizer = make_visualizer();

  // Act
  const auto output = visualizer.visualize(background_image, fine_rois, signal);
  ASSERT_NE(output, nullptr);

  // Assert: the fallback of str_to_color(), not the plain white of a ROI with no signal
  const auto frame_corner = pixel_at(*output, fine_box.x, fine_box.y);
  EXPECT_EQ(frame_corner, unknown_circle_rgb);
  EXPECT_NE(frame_corner, no_circle_rgb);
}

TEST(TrafficLightRoiVisualizer, WhiteCircleAlsoFallsBackToOffWhite)
{
  // Arrange: str_to_color() knows red, yellow and green only, so WHITE - a color the message
  // defines - lands in the same fallback as UNKNOWN.
  const auto signal =
    make_signal(signal_id, TrafficLightElement::WHITE, TrafficLightElement::CIRCLE);
  const auto visualizer = make_visualizer();

  // Act
  const auto output = visualizer.visualize(background_image, fine_rois, signal);
  ASSERT_NE(output, nullptr);

  // Assert
  const auto frame_corner = pixel_at(*output, fine_box.x, fine_box.y);
  EXPECT_EQ(frame_corner, unknown_circle_rgb);
}

TEST(TrafficLightRoiVisualizer, UndefinedColorCodeFallsBackToOffWhite)
{
  // Arrange: an element carrying a code outside the message definition. state_to_label() answers
  // with an empty string for it, so the label reads "-circle" and str_to_color("") takes the
  // fallback.
  constexpr uint8_t undefined_color = 200;
  const auto signal = make_signal(signal_id, undefined_color, TrafficLightElement::CIRCLE);
  const auto visualizer = make_visualizer();

  // Act
  const auto output = visualizer.visualize(background_image, fine_rois, signal);
  ASSERT_NE(output, nullptr);

  // Assert
  const auto frame_corner = pixel_at(*output, fine_box.x, fine_box.y);
  EXPECT_EQ(frame_corner, unknown_circle_rgb);
}

TEST(TrafficLightRoiVisualizer, SignalWithoutCircleLeavesFrameWhite)
{
  // Arrange: an arrow, i.e. a classified signal whose only element is not a circle. The color of
  // the element is dropped: only a circle decides the frame color.
  const auto signal =
    make_signal(signal_id, TrafficLightElement::GREEN, TrafficLightElement::LEFT_ARROW);
  const auto visualizer = make_visualizer();

  // Act
  const auto output = visualizer.visualize(background_image, fine_rois, signal);
  ASSERT_NE(output, nullptr);

  // Assert
  const auto frame_corner = pixel_at(*output, fine_box.x, fine_box.y);
  EXPECT_EQ(frame_corner, no_circle_rgb);
}

TEST(TrafficLightRoiVisualizer, InvalidRecognitionIsWhiteLikeArrow)
{
  // Arrange: unknown-unknown is how an invalid recognition reaches this node. Its shape is not a
  // circle either, so it gets the same white as the arrow above.
  const auto signal =
    make_signal(signal_id, TrafficLightElement::UNKNOWN, TrafficLightElement::UNKNOWN);
  const auto visualizer = make_visualizer();

  // Act
  const auto output = visualizer.visualize(background_image, fine_rois, signal);
  ASSERT_NE(output, nullptr);

  // Assert
  const auto frame_corner = pixel_at(*output, fine_box.x, fine_box.y);
  EXPECT_EQ(frame_corner, no_circle_rgb);
}

TEST(TrafficLightRoiVisualizer, CircleDecidesColorAmongSeveralElements)
{
  // Arrange: A green circle together with a red arrow, the way a signal reports several lamps at
  // once.
  const auto signal = make_signals(
    signal_id, {{TrafficLightElement::RED, TrafficLightElement::LEFT_ARROW},
                {TrafficLightElement::GREEN, TrafficLightElement::CIRCLE}});
  const auto visualizer = make_visualizer();

  // Act
  const auto output = visualizer.visualize(background_image, fine_rois, signal);
  ASSERT_NE(output, nullptr);

  // Assert
  const auto frame_corner = pixel_at(*output, fine_box.x, fine_box.y);
  EXPECT_EQ(frame_corner, green_signal_rgb);
}

// ---------------------------------------------------------------------------------------------
// The image itself: encodings, and the two places where drawing gives up quietly.
// ---------------------------------------------------------------------------------------------

TEST(TrafficLightRoiVisualizer, AnyConvertibleEncodingComesBackAsRgb8)
{
  // Arrange: bgr8 carries the channels the other way round, so a drawing that ignored the encoding
  // would put the signal color in the wrong channels.
  const Pixel background_bgr{background_rgb[2], background_rgb[1], background_rgb[0]};
  const auto input = make_image("bgr8", background_bgr);
  const auto visualizer = make_visualizer();

  // Act
  const auto output = visualizer.visualize(input, fine_rois, green_signal);
  ASSERT_NE(output, nullptr);

  // Assert: the output is rgb8, the frame is in the signal color rather than its mirror, and the
  // background came through the conversion as the rgb8 value the test wrote as bgr8
  EXPECT_EQ(output->encoding, "rgb8");
  const auto frame_corner = pixel_at(*output, fine_box.x, fine_box.y);
  EXPECT_EQ(frame_corner, green_signal_rgb);
  const auto untouched = pixel_at(*output, 0, 0);
  EXPECT_EQ(untouched, background_rgb);
}

TEST(TrafficLightRoiVisualizer, EncodingCvBridgeCannotConvertThrows)
{
  // Arrange
  const auto input = make_image("no_such_encoding", background_rgb);
  const auto visualizer = make_visualizer();

  // Act and Assert
  EXPECT_THROW(visualizer.visualize(input, fine_rois, green_signal), cv_bridge::Exception);
}

TEST(TrafficLightRoiVisualizer, RoiAtOriginGetsFrameButNoLabelBox)
{
  // Arrange: draw_shape() reads a ROI at (0,0) as undetected and returns before drawing anything.
  // The frame is drawn all the same, because that happens before the label box.
  constexpr Box at_origin{0, 0, 40, 90};
  const auto rois = make_rois(signal_id, {at_origin});
  const auto visualizer = make_visualizer();

  // Act
  const auto output = visualizer.visualize(background_image, rois, green_signal);
  ASSERT_NE(output, nullptr);

  // Assert: the frame is there
  const auto frame_corner = pixel_at(*output, at_origin.x, at_origin.y);
  EXPECT_EQ(frame_corner, green_signal_rgb);

  // Nothing is drawn below it where the box would have gone if the ROI sat lower
  const auto inside_the_roi = pixel_at(*output, at_origin.x + 20, at_origin.y + 45);
  EXPECT_EQ(inside_the_roi, background_rgb);
}

// The label box must fit inside the image, and draw_shape() tests all four of its edges in one
// if. Each term gets a case of its own below, including the one no ROI can reach today: the guard
// is the drawing's own contract, and the message types that feed it are free to change.
TEST(TrafficLightRoiVisualizer, RoiTooCloseToTopGetsNoLabelBox)
{
  // Arrange: the label box goes above the ROI, so a ROI within its height of the top edge leaves no
  // room. draw_shape() checks and returns rather than clipping.
  constexpr Box near_the_top{200, 10, 40, 90};
  const auto rois = make_rois(signal_id, {near_the_top});
  const auto visualizer = make_visualizer();

  // Act
  const auto output = visualizer.visualize(background_image, rois, green_signal);
  ASSERT_NE(output, nullptr);

  // Assert: the frame is drawn
  const auto frame_corner = pixel_at(*output, near_the_top.x, near_the_top.y);
  EXPECT_EQ(frame_corner, green_signal_rgb);

  // The strip above it stays background: no box, not even a clipped one
  const auto above_the_roi = pixel_at(*output, near_the_top.x + 30, near_the_top.y - 5);
  EXPECT_EQ(above_the_roi, background_rgb);
}

TEST(TrafficLightRoiVisualizer, RoiTooCloseToRightGetsNoLabelBox)
{
  // Arrange: the box runs to the right of the ROI, so a ROI within its width of the right edge
  // leaves no room. The box is about 86 px wide for one shape at 87%, so 600 is inside the margin.
  constexpr Box near_the_right{600, 150, 40, 90};
  const auto rois = make_rois(signal_id, {near_the_right});
  const auto visualizer = make_visualizer();

  // Act
  const auto output = visualizer.visualize(background_image, rois, green_signal);
  ASSERT_NE(output, nullptr);

  // Assert: the frame is drawn, clipped at the right edge
  const auto frame_corner = pixel_at(*output, near_the_right.x, near_the_right.y);
  EXPECT_EQ(frame_corner, green_signal_rgb);

  // The strip above it stays background: no box, not even a clipped one
  const auto above_the_roi = label_box_pixel(*output, near_the_right);
  EXPECT_EQ(above_the_roi, background_rgb);
}

TEST(TrafficLightRoiVisualizer, RoiPastBottomGetsNoLabelBox)
{
  // Arrange: a ROI below the bottom edge. Nothing of it is on the image, so this is the one case
  // of the four where the frame does not appear either.
  constexpr Box past_the_bottom{200, 500, 40, 90};
  const auto rois = make_rois(signal_id, {past_the_bottom});
  const auto visualizer = make_visualizer();

  // Act: the guard is what keeps this from throwing. Without it the box is cut from the image by
  // a rectangle that reaches past its last row, and OpenCV raises rather than clipping.
  const auto output = visualizer.visualize(background_image, rois, green_signal);
  ASSERT_NE(output, nullptr);

  // Assert
  EXPECT_EQ(count_pixels_differing_from(*output, background_rgb), 0u);
}

TEST(TrafficLightRoiVisualizer, RoiPastLeftGetsNoLabelBox)
{
  // Arrange: the box starts at the ROI's own left edge, so this term needs a negative x, and
  // x_offset is uint32 in the message: no ROI can state one. The term is tested all the same,
  // because the guard belongs to the drawing rather than to the type that happens to feed it
  // today. A negative offset does reach draw_shape(): it wraps into the uint32 field and wraps
  // back when cv::Point holds it as an int, so -16 arrives as -16.
  constexpr Box past_the_left{-16, 150, 40, 90};
  const auto rois = make_rois(signal_id, {past_the_left});
  const auto visualizer = make_visualizer();

  // Act
  const auto output = visualizer.visualize(background_image, rois, green_signal);
  ASSERT_NE(output, nullptr);

  // Assert: the frame is drawn, clipped at the left edge, so x = 10 is on its top side
  const auto frame_top_side = pixel_at(*output, 10, past_the_left.y);
  EXPECT_EQ(frame_top_side, green_signal_rgb);

  // The strip above it stays background: no box, not even a clipped one. The box would have
  // covered this point, reaching from the ROI's left edge to 70 px right of it.
  const auto above_the_roi = pixel_at(*output, 10, past_the_left.y - 25);
  EXPECT_EQ(above_the_roi, background_rgb);
}

// ---------------------------------------------------------------------------------------------
// visualize_with_rough_rois(): the rough ROIs are the subject. Per rough ROI a fine ROI and a
// signal may or may not be found for its id, and only the two cases with a fine ROI are expected
// in a working pipeline - the others need a synchronizer mismatch upstream.
// ---------------------------------------------------------------------------------------------

TEST(TrafficLightRoiVisualizer, NoRoughRoisLeavesImageUntouched)
{
  // Arrange
  const auto visualizer = make_visualizer();

  // Act
  const auto output =
    visualizer.visualize_with_rough_rois(background_image, fine_rois, no_rois, green_signal);
  ASSERT_NE(output, nullptr);

  // Assert
  EXPECT_EQ(count_pixels_differing_from(*output, background_rgb), 0u);
}

TEST(TrafficLightRoiVisualizer, RoughAndFineWithSignalDrawBothFramesAndOneLabel)
{
  // Arrange
  const auto visualizer = make_visualizer();

  // Act
  const auto output =
    visualizer.visualize_with_rough_rois(background_image, fine_rois, rough_rois, green_signal);
  ASSERT_NE(output, nullptr);

  // Assert: both frames are drawn, both in the signal color
  const auto rough_frame_corner = pixel_at(*output, rough_box.x, rough_box.y);
  const auto fine_frame_bottom_left = pixel_at(*output, fine_box.x, fine_box.y + fine_box.height);
  EXPECT_EQ(rough_frame_corner, green_signal_rgb);
  EXPECT_EQ(fine_frame_bottom_left, green_signal_rgb);

  // The label box goes above the fine ROI, not the rough one
  const auto icon_above_the_fine_roi = label_icon_pixel(*output, fine_box);
  EXPECT_EQ(icon_above_the_fine_roi, label_icon_rgb);
}

TEST(TrafficLightRoiVisualizer, RoughAndFineWithoutSignalDrawBothFramesInWhite)
{
  // Arrange
  const auto visualizer = make_visualizer();

  // Act
  const auto output =
    visualizer.visualize_with_rough_rois(background_image, fine_rois, rough_rois, no_signals);
  ASSERT_NE(output, nullptr);

  // Assert: both frames are white: an empty label falls back to the same color the fine frame is
  // handed
  const auto rough_frame_corner = pixel_at(*output, rough_box.x, rough_box.y);
  const auto fine_frame_bottom_left = pixel_at(*output, fine_box.x, fine_box.y + fine_box.height);
  EXPECT_EQ(rough_frame_corner, no_circle_rgb);
  EXPECT_EQ(fine_frame_bottom_left, no_circle_rgb);

  // Neither gets a label box
  const auto above_the_rough_roi = label_box_pixel(*output, rough_box);
  const auto above_the_fine_roi = label_box_pixel(*output, fine_box);
  EXPECT_EQ(above_the_rough_roi, background_rgb);
  EXPECT_EQ(above_the_fine_roi, background_rgb);
}

TEST(TrafficLightRoiVisualizer, RoughRoiWithoutFineOneCarriesLabelItself)
{
  // Arrange
  const auto visualizer = make_visualizer();

  // Act
  const auto output =
    visualizer.visualize_with_rough_rois(background_image, no_rois, rough_rois, green_signal);
  ASSERT_NE(output, nullptr);

  // Assert: the rough frame is drawn in the signal color and carries the label box
  const auto rough_frame_corner = pixel_at(*output, rough_box.x, rough_box.y);
  const auto icon_above_the_rough_roi = label_icon_pixel(*output, rough_box);
  EXPECT_EQ(rough_frame_corner, green_signal_rgb);
  EXPECT_EQ(icon_above_the_rough_roi, label_icon_rgb);

  // Nothing is drawn where a fine ROI would have been
  const auto where_the_fine_roi_would_be = pixel_at(*output, fine_box.x, fine_box.y);
  EXPECT_EQ(where_the_fine_roi_would_be, background_rgb);
}

TEST(TrafficLightRoiVisualizer, RoughRoiWithNeitherIsDrawnOnItsOwn)
{
  // Arrange
  const auto visualizer = make_visualizer();

  // Act
  const auto output =
    visualizer.visualize_with_rough_rois(background_image, no_rois, rough_rois, no_signals);
  ASSERT_NE(output, nullptr);

  // Assert: the rough frame is drawn, white, with no label box
  const auto rough_frame_corner = pixel_at(*output, rough_box.x, rough_box.y);
  const auto above_the_rough_roi = label_box_pixel(*output, rough_box);
  EXPECT_EQ(rough_frame_corner, no_circle_rgb);
  EXPECT_EQ(above_the_rough_roi, background_rgb);

  // Nothing is drawn where a fine ROI would have been
  const auto where_the_fine_roi_would_be = pixel_at(*output, fine_box.x, fine_box.y);
  EXPECT_EQ(where_the_fine_roi_would_be, background_rgb);
}

TEST(TrafficLightRoiVisualizer, FineRoiNoRoughRoiMentionsIsNotDrawn)
{
  // Arrange: the loop walks the rough ROIs and looks the fine ones up by id, so a fine ROI for a
  // traffic light the rough ROIs say nothing about is never reached - not even a frame.
  const auto orphan = make_rois(other_signal_id, {fine_box});
  const auto signal_for_it =
    make_signal(other_signal_id, TrafficLightElement::GREEN, TrafficLightElement::CIRCLE);
  const auto visualizer = make_visualizer();

  // Act
  const auto output =
    visualizer.visualize_with_rough_rois(background_image, orphan, rough_rois, signal_for_it);
  ASSERT_NE(output, nullptr);

  // Assert
  const auto where_the_orphan_is = pixel_at(*output, fine_box.x, fine_box.y);
  EXPECT_EQ(where_the_orphan_is, background_rgb);
}

TEST(TrafficLightRoiVisualizer, EveryRoughRoiInArrayIsDrawn)
{
  // Arrange
  constexpr Box second_rough{400, 140, 60, 110};
  const auto two = make_rois(signal_id, {rough_box, second_rough});
  const auto visualizer = make_visualizer();

  // Act
  const auto output =
    visualizer.visualize_with_rough_rois(background_image, no_rois, two, no_signals);
  ASSERT_NE(output, nullptr);

  // Assert: the top left corner of each rough ROI
  const auto first_frame = pixel_at(*output, rough_box.x, rough_box.y);
  const auto second_frame = pixel_at(*output, second_rough.x, second_rough.y);
  EXPECT_EQ(first_frame, no_circle_rgb);
  EXPECT_EQ(second_frame, no_circle_rgb);
}
