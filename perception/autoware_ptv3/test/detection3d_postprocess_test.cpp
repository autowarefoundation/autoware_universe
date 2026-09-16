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

#include "autoware/ptv3/postprocess/detection3d_postprocess.hpp"

#include "ptv3_test_fixture.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

namespace autoware::ptv3
{
namespace test
{

PTv3Config makePostprocessConfig()
{
  PTv3ConfigParams params;
  params.use_seg3d_head = false;
  params.use_det3d_head = true;
  params.cloud_capacity = 8;
  params.voxels_num = {1, 4, 8};
  params.point_cloud_range = {0.0F, 0.0F, 0.0F, 16.0F, 16.0F, 4.0F};
  params.voxel_size = {1.0F, 1.0F, 1.0F};
  params.pooling_strides = {2, 2, 2, 2};
  params.enc_channels = {8, 16, 32, 64, 128};
  params.bbox_voxel_size = {8.0F, 8.0F, 4.0F};
  params.distance_bin_upper_limits = {100.0F};
  params.detection_score_thresholds = {0.4F, 0.4F};
  params.yaw_norm_thresholds = {0.1F, 0.1F};
  params.num_proposals = 3;
  params.post_center_range = {-1.0F, -1.0F, -1.0F, 20.0F, 20.0F, 20.0F};
  return makeConfig(params);
}

class Detection3DPostprocessTest : public PTv3CudaTest
{
};

TEST_F(Detection3DPostprocessTest, DecodesFiltersAndSortsBoxes)
{
  const auto config = makePostprocessConfig();
  Detection3DPostprocess postprocess(config, stream_);

  const std::vector<float> query_heatmap_score{1.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F};
  const std::vector<std::int64_t> query_labels{0, 1, -1};
  const std::vector<float> heatmap{0.0F, 0.0F, 0.0F, 0.0F, 1.3862944F, 0.0F};
  const std::vector<float> center{1.0F, 0.5F, 0.0F, 1.0F, 0.5F, 0.0F};
  const std::vector<float> height{2.0F, 3.0F, 0.0F};
  const std::vector<float> dim{std::log(2.0F), std::log(3.0F), 0.0F,
                               std::log(1.0F), std::log(2.0F), 0.0F,
                               std::log(2.0F), std::log(1.0F), 0.0F};
  const std::vector<float> rot{0.0F, 1.0F, 0.0F, 1.0F, 0.0F, 1.0F};
  const std::vector<float> vel{0.1F, 0.2F, 0.0F, 1.1F, 1.2F, 0.0F};

  auto query_heatmap_score_d = makeDeviceBuffer<float>(query_heatmap_score.size());
  auto query_labels_d = makeDeviceBuffer<std::int64_t>(query_labels.size());
  auto heatmap_d = makeDeviceBuffer<float>(heatmap.size());
  auto center_d = makeDeviceBuffer<float>(center.size());
  auto height_d = makeDeviceBuffer<float>(height.size());
  auto dim_d = makeDeviceBuffer<float>(dim.size());
  auto rot_d = makeDeviceBuffer<float>(rot.size());
  auto vel_d = makeDeviceBuffer<float>(vel.size());
  copyToDevice(query_heatmap_score_d.get(), query_heatmap_score);
  copyToDevice(query_labels_d.get(), query_labels);
  copyToDevice(heatmap_d.get(), heatmap);
  copyToDevice(center_d.get(), center);
  copyToDevice(height_d.get(), height);
  copyToDevice(dim_d.get(), dim);
  copyToDevice(rot_d.get(), rot);
  copyToDevice(vel_d.get(), vel);

  EXPECT_EQ(
    postprocess.process(
      query_heatmap_score_d.get(), query_labels_d.get(), heatmap_d.get(), center_d.get(),
      height_d.get(), dim_d.get(), rot_d.get(), vel_d.get(), stream_),
    cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(stream_), cudaSuccess);

  ASSERT_EQ(postprocess.numBoxes(), 2U);
  const auto boxes = copyToHost(postprocess.deviceBoxes(), postprocess.numBoxes());
  EXPECT_EQ(boxes[0].label, 1);
  EXPECT_NEAR(boxes[0].score, 0.8F, 1e-5F);
  EXPECT_FLOAT_EQ(boxes[0].x, 4.0F);
  EXPECT_FLOAT_EQ(boxes[0].y, 4.0F);
  EXPECT_FLOAT_EQ(boxes[0].vel_x, 0.2F);
  EXPECT_FLOAT_EQ(boxes[0].vel_y, 1.2F);
  EXPECT_EQ(boxes[1].label, 0);
  EXPECT_NEAR(boxes[1].score, 0.5F, 1e-5F);
  EXPECT_FLOAT_EQ(boxes[1].x, 8.0F);
  EXPECT_FLOAT_EQ(boxes[1].y, 8.0F);
}

}  // namespace test
}  // namespace autoware::ptv3
