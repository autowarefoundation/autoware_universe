// Copyright 2023 Autoware Foundation
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

#include <autoware/point_types/types.hpp>

#include <gtest/gtest.h>

namespace autoware::lidar_marker_localizer
{

class TestableLidarMarkerLocalizer : public LidarMarkerLocalizer
{
public:
  using LidarMarkerLocalizer::LidarMarkerLocalizer;

  template <typename PointT>
  auto callDetectLandmarks(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & points_msg_ptr)
  {
    return this->detect_landmarks<PointT>(points_msg_ptr);
  }

  Param & public_mutable_param() { return mutable_param(); }
  const Param & public_param() const { return param(); }
};

class LidarMarkerLocalizerTest : public ::testing::Test
{
protected:
  rclcpp::NodeOptions node_options;
  std::shared_ptr<TestableLidarMarkerLocalizer> localizer;

  void SetUp() override
  {
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
       {"self_pose_timeout_sec", 1.0},
       {"self_pose_distance_tolerance_m", 1.0},
       {"limit_distance_from_self_pose_to_nearest_marker", 1.0},
       {"limit_distance_from_self_pose_to_nearest_marker_y", 1.0},
       {"limit_distance_from_self_pose_to_marker", 1.0},
       {"base_covariance", std::vector<double>(36, 0.0)},
       {"marker_width", 1.0},
       {"lower_ring_id_init", 0},
       {"upper_ring_id_init", 0},
       {"enable_save_log", false},
       {"save_file_directory_path", std::string("")},
       {"save_file_name", std::string("")},
       {"save_frame_id", std::string("")},
       {"radius_for_extracting_marker_pointcloud", 1.0},
       {"queue_size_for_debug_pub_msg", 1}});
    localizer = std::make_shared<TestableLidarMarkerLocalizer>(node_options);
  }

};  // class LidarMarkerLocalizerTest : public ::testing::Test

TEST_F(LidarMarkerLocalizerTest, Initialization)
{
  ASSERT_NE(localizer, nullptr);
}

TEST_F(LidarMarkerLocalizerTest, DetectLandmarksRoadSurfaceModeSwitch)
{
  // ダミーPointCloud2生成
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "base_link";
  cloud.header.stamp.sec = 0;
  cloud.header.stamp.nanosec = 0;
  // 必要なフィールドをセット
  cloud.height = 1;
  cloud.width = 2;
  cloud.is_dense = true;
  cloud.is_bigendian = false;
  cloud.point_step =
    sizeof(float) * 4 + sizeof(uint16_t) + sizeof(float);  // x, y, z, intensity, channel, dummy
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.fields.resize(5);
  cloud.fields[0].name = "x";
  cloud.fields[0].offset = 0;
  cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[0].count = 1;
  cloud.fields[1].name = "y";
  cloud.fields[1].offset = 4;
  cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[1].count = 1;
  cloud.fields[2].name = "z";
  cloud.fields[2].offset = 8;
  cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[2].count = 1;
  cloud.fields[3].name = "intensity";
  cloud.fields[3].offset = 12;
  cloud.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[3].count = 1;
  cloud.fields[4].name = "channel";
  cloud.fields[4].offset = 16;
  cloud.fields[4].datatype = sensor_msgs::msg::PointField::UINT16;
  cloud.fields[4].count = 1;
  cloud.data.resize(cloud.row_step * cloud.height);

  // パラメータセット
  auto & param = localizer->public_mutable_param();
  param.lower_ring_id_init = 0;
  param.upper_ring_id_init = 0;
  param.resolution = 1.0;
  param.intensity_pattern = {1, 1};
  param.positive_match_num_threshold = 1;
  param.negative_match_num_threshold = 1;
  param.match_intensity_difference_threshold = 1;
  param.vote_threshold_for_detect_marker = 0;
  param.marker_to_vehicle_offset_y = 123.0f;
  param.marker_height_from_ground = 2.0f;
  param.reference_ring_number = std::numeric_limits<uint8_t>::max();
  param.marker_width = 1.0f;

  // road_surface_mode OFF
  param.road_surface_mode = false;
  auto result_off = localizer->callDetectLandmarks<autoware::point_types::PointXYZIRC>(
    std::make_shared<sensor_msgs::msg::PointCloud2>(cloud));
  // OFF時はLandmarkのyがreference_ring_y（ダミーなのでmax値）
  if (!result_off.empty()) {
    EXPECT_EQ(result_off[0].pose.position.y, std::numeric_limits<float>::max());
  }

  // road_surface_mode ON
  param.road_surface_mode = true;
  auto result_on = localizer->callDetectLandmarks<autoware::point_types::PointXYZIRC>(
    std::make_shared<sensor_msgs::msg::PointCloud2>(cloud));
  // ON時はLandmarkのyがmarker_to_vehicle_offset_y
  if (!result_on.empty()) {
    EXPECT_EQ(result_on[0].pose.position.y, param.marker_to_vehicle_offset_y);
  }
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
