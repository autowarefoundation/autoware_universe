// Copyright 2025 Autoware Foundation
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

#include "lidar_marker_localizer.hpp"
#include "marker_detector.hpp"

#include <autoware/point_types/types.hpp>

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace autoware::lidar_marker_localizer
{
namespace
{
using PointT = autoware::point_types::PointXYZIRC;

constexpr std::uint8_t low_intensity = 10;
constexpr std::uint8_t high_intensity = 200;

// The marker is detected where the intensity changes low, high, high, low along the x-axis.
DetectionParam make_param()
{
  DetectionParam param{};
  param.road_surface_mode = false;
  param.resolution = 1.0;
  param.intensity_pattern = {-1, 1, 1, -1};
  param.match_intensity_difference_threshold = 10;
  param.positive_match_num_threshold = 2;
  param.negative_match_num_threshold = 2;
  param.vote_threshold_for_detect_marker = 1;
  param.marker_to_vehicle_offset_y = 1.0;
  param.marker_height_from_ground = 0.5;
  param.reference_ring_number = 255;  // use all rings
  param.lower_ring_id_init = 128;
  param.upper_ring_id_init = 0;
  return param;
}

// Rings 0, 1 and 2 at y = 2.0, 2.5 and 3.0, with one point per bin at x = 0, 1, ..., x_num - 1.
// The points at x = 4 and 5 have the high intensity if with_marker is true.
pcl::PointCloud<PointT> make_points(const int x_num, const bool with_marker)
{
  pcl::PointCloud<PointT> points;
  for (std::uint16_t ring = 0; ring < 3; ++ring) {
    for (int x = 0; x < x_num; ++x) {
      PointT point;
      point.x = static_cast<float>(x);
      point.y = 2.0F + 0.5F * static_cast<float>(ring);
      point.z = 0.0F;
      point.intensity = (with_marker && (x == 4 || x == 5)) ? high_intensity : low_intensity;
      point.channel = ring;
      points.push_back(point);
    }
  }
  return points;
}

std_msgs::msg::Header make_header()
{
  std_msgs::msg::Header header;
  header.frame_id = "base_link";
  return header;
}
}  // namespace

// The pattern matches only at the window starting at bin 3 (bins 3 to 6 are low, high, high,
// low) in each of the 3 rings, so vote[3] = 3 exceeds the threshold of 1. The marker x is
// (3 + 4 / 2) * resolution + min_x = 5.0.
TEST(DetectLandmarks, DetectMarkerOnVerticalSurface)
{
  const DetectionParam param = make_param();

  const DetectionResult result = detect_landmarks(make_points(10, true), make_header(), param);

  EXPECT_EQ(result.status, DetectionStatus::ok);
  ASSERT_EQ(result.landmarks.size(), 1U);
  const auto & position = result.landmarks.front().pose.position;
  EXPECT_DOUBLE_EQ(position.x, 5.0);
  // the smallest y of the points in the bin, which is ring 0
  EXPECT_DOUBLE_EQ(position.y, 2.0);
  EXPECT_DOUBLE_EQ(position.z, 0.5);
}

TEST(DetectLandmarks, DetectMarkerOnRoadSurface)
{
  DetectionParam param = make_param();
  param.road_surface_mode = true;

  const DetectionResult result = detect_landmarks(make_points(10, true), make_header(), param);

  EXPECT_EQ(result.status, DetectionStatus::ok);
  ASSERT_EQ(result.landmarks.size(), 1U);
  const auto & position = result.landmarks.front().pose.position;
  EXPECT_DOUBLE_EQ(position.x, 5.0);
  // fixed to marker_to_vehicle_offset_y instead of the y of the points
  EXPECT_DOUBLE_EQ(position.y, 1.0);
  EXPECT_DOUBLE_EQ(position.z, 0.5);
}

TEST(DetectLandmarks, NoMarkerOnUniformIntensity)
{
  const DetectionResult result =
    detect_landmarks(make_points(10, false), make_header(), make_param());

  EXPECT_EQ(result.status, DetectionStatus::ok);
  EXPECT_TRUE(result.landmarks.empty());
}

TEST(DetectLandmarks, BinNumTooSmall)
{
  // 3 bins are fewer than the 4 elements of the intensity pattern
  const DetectionResult result =
    detect_landmarks(make_points(3, false), make_header(), make_param());

  EXPECT_EQ(result.status, DetectionStatus::bin_num_too_small);
  EXPECT_TRUE(result.landmarks.empty());
}

TEST(DetectLandmarks, NoPoints)
{
  const DetectionResult result =
    detect_landmarks(pcl::PointCloud<PointT>{}, make_header(), make_param());

  EXPECT_EQ(result.status, DetectionStatus::no_points);
  EXPECT_TRUE(result.landmarks.empty());
}

// In the road surface mode the grid of 3 rings * 1.0 m lies on the road surface, with its rows
// from y = offset + 1.5 to y = offset - 1.5 for a positive offset (rotated by roll = pi) and from
// y = offset - 1.5 to y = offset + 1.5 for a negative offset.
TEST(DetectLandmarks, RoadSurfaceGridIsCenteredOnOffset)
{
  DetectionParam param = make_param();
  param.road_surface_mode = true;

  param.marker_to_vehicle_offset_y = 1.0;
  const auto left = detect_landmarks(make_points(10, true), make_header(), param);
  const auto & left_origin = left.center_intensity_grid.info.origin;
  EXPECT_EQ(left.center_intensity_grid.info.height, 3U);
  EXPECT_DOUBLE_EQ(left_origin.position.y, 2.5);
  EXPECT_DOUBLE_EQ(left_origin.position.z, 0.5);
  EXPECT_NEAR(left_origin.orientation.x, 1.0, 1e-9);
  EXPECT_NEAR(left_origin.orientation.w, 0.0, 1e-9);

  param.marker_to_vehicle_offset_y = -1.0;
  const auto right = detect_landmarks(make_points(10, true), make_header(), param);
  const auto & right_origin = right.center_intensity_grid.info.origin;
  EXPECT_DOUBLE_EQ(right_origin.position.y, -2.5);
  EXPECT_DOUBLE_EQ(right_origin.position.z, 0.5);
  EXPECT_NEAR(right_origin.orientation.x, 0.0, 1e-9);
  EXPECT_NEAR(right_origin.orientation.w, 1.0, 1e-9);
}

// The node declares all the parameters, including road_surface_mode.
TEST(LidarMarkerLocalizer, Initialization)
{
  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides(
    {{"enable_read_all_target_ids", false},
     {"target_ids", std::vector<std::string>{}},
     {"queue_size_for_output_pose", 1},
     {"marker_name", std::string("")},
     {"road_surface_mode", false},
     {"resolution", 1.0},
     {"intensity_pattern", std::vector<int64_t>{}},
     {"match_intensity_difference_threshold", 1},
     {"positive_match_num_threshold", 1},
     {"negative_match_num_threshold", 1},
     {"vote_threshold_for_detect_marker", 1},
     {"marker_to_vehicle_offset_y", 0.0},
     {"marker_height_from_ground", 0.0},
     {"reference_ring_number", 0},
     {"marker_width", 0.8},
     {"self_pose_timeout_sec", 1.0},
     {"self_pose_distance_tolerance_m", 1.0},
     {"limit_distance_from_self_pose_to_nearest_marker", 1.0},
     {"limit_distance_from_self_pose_to_nearest_marker_y", 1.0},
     {"limit_distance_from_self_pose_to_marker", 1.0},
     {"base_covariance", std::vector<double>(36, 0.0)},
     {"lower_ring_id_init", 0},
     {"upper_ring_id_init", 0},
     {"enable_save_log", false},
     {"save_file_directory_path", std::string("")},
     {"save_file_name", std::string("")},
     {"save_frame_id", std::string("")},
     {"radius_for_extracting_marker_pointcloud", 1.0},
     {"queue_size_for_debug_pub_msg", 1}});

  std::shared_ptr<LidarMarkerLocalizer> localizer;
  ASSERT_NO_THROW(localizer = std::make_shared<LidarMarkerLocalizer>(node_options));
  EXPECT_FALSE(localizer->get_parameter("road_surface_mode").as_bool());
}

}  // namespace autoware::lidar_marker_localizer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
