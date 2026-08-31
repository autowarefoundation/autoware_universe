// Copyright 2026 NEWSLab, National Taiwan University
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

#include "autoware/cuda_pointcloud_preprocessor/cuda_crop_box_filter/cuda_crop_box_filter.hpp"

#include <cuda_blackboard/cuda_pointcloud2.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <cuda_runtime.h>
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

namespace
{

using autoware::cuda_pointcloud_preprocessor::CudaCropBoxFilter;

/// A layout with payload past xyz, so a test can prove the filter moves whole
/// points rather than reconstructing them from the three fields it reads.
/// The trailing pad is deliberate: real driver layouts have alignment holes, and
/// they must survive too, because whatever wrote them may put meaning there.
struct TestPoint
{
  float x;
  float y;
  float z;
  float intensity;
  std::uint16_t ring;
  std::uint16_t pad;
};
static_assert(sizeof(TestPoint) == 20, "the test relies on a hole-free 20-byte layout");

sensor_msgs::msg::PointField makeField(
  const std::string & name, std::uint32_t offset, std::uint8_t datatype)
{
  sensor_msgs::msg::PointField field;
  field.name = name;
  field.offset = offset;
  field.datatype = datatype;
  field.count = 1;
  return field;
}

/// Build a device-resident cloud from host points.
///
/// `xyz_datatype` is a parameter only so a test can hand the filter a layout it
/// is supposed to refuse.
cuda_blackboard::CudaPointCloud2 makeCloud(
  const std::vector<TestPoint> & points,
  std::uint8_t xyz_datatype = sensor_msgs::msg::PointField::FLOAT32)
{
  cuda_blackboard::CudaPointCloud2 cloud;
  cloud.header.frame_id = "base_link";
  cloud.fields = {
    makeField("x", 0, xyz_datatype), makeField("y", 4, xyz_datatype),
    makeField("z", 8, xyz_datatype),
    makeField("intensity", 12, sensor_msgs::msg::PointField::FLOAT32),
    makeField("ring", 16, sensor_msgs::msg::PointField::UINT16)};
  cloud.point_step = sizeof(TestPoint);
  cloud.height = 1;
  cloud.width = static_cast<std::uint32_t>(points.size());
  cloud.row_step = cloud.width * cloud.point_step;
  cloud.is_bigendian = false;
  cloud.is_dense = false;

  if (points.empty()) {
    return cloud;
  }

  const std::size_t bytes = points.size() * sizeof(TestPoint);
  cloud.data = cuda_blackboard::make_unique<std::uint8_t[]>(bytes);
  EXPECT_EQ(
    cudaMemcpy(cloud.data.get(), points.data(), bytes, cudaMemcpyHostToDevice), cudaSuccess);
  return cloud;
}

/// Copy a result back as raw bytes, which is what the byte-for-byte assertions
/// need; the typed view is a reinterpretation of the same buffer.
std::vector<std::uint8_t> readBack(const cuda_blackboard::CudaPointCloud2 & cloud)
{
  const std::size_t bytes = static_cast<std::size_t>(cloud.width) *
                            static_cast<std::size_t>(cloud.height) * cloud.point_step;
  std::vector<std::uint8_t> host(bytes);
  if (bytes == 0) {
    return host;
  }
  EXPECT_EQ(cudaMemcpy(host.data(), cloud.data.get(), bytes, cudaMemcpyDeviceToHost), cudaSuccess);
  return host;
}

std::vector<TestPoint> readBackPoints(const cuda_blackboard::CudaPointCloud2 & cloud)
{
  const auto bytes = readBack(cloud);
  std::vector<TestPoint> points(bytes.size() / sizeof(TestPoint));
  if (!points.empty()) {
    std::memcpy(points.data(), bytes.data(), bytes.size());
  }
  return points;
}

/// A box the tests share: [-10, 10] on x and y, [-5, 5] on z.
CudaCropBoxFilter::BoxParams box(bool negative)
{
  CudaCropBoxFilter::BoxParams params{};
  params.min_x = -10.0f;
  params.max_x = 10.0f;
  params.min_y = -10.0f;
  params.max_y = 10.0f;
  params.min_z = -5.0f;
  params.max_z = 5.0f;
  params.negative = negative;
  return params;
}

TestPoint point(float x, float y, float z, float intensity = 0.0f, std::uint16_t ring = 0)
{
  return TestPoint{x, y, z, intensity, ring, 0};
}

bool haveCudaDevice()
{
  int count = 0;
  return cudaGetDeviceCount(&count) == cudaSuccess && count > 0;
}

class CudaCropBoxFilterTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!haveCudaDevice()) {
      GTEST_SKIP() << "no CUDA device available; the crop box has no CPU path to fall back to";
    }
  }
};

TEST_F(CudaCropBoxFilterTest, KeepsInsideDropsOutside)
{
  CudaCropBoxFilter filter(box(false));

  const std::vector<TestPoint> input = {
    point(0.0f, 0.0f, 0.0f),     // origin, inside
    point(9.9f, -9.9f, 4.9f),    // just inside every bound
    point(10.1f, 0.0f, 0.0f),    // past max_x
    point(-10.1f, 0.0f, 0.0f),   // past min_x
    point(0.0f, 10.1f, 0.0f),    // past max_y
    point(0.0f, -10.1f, 0.0f),   // past min_y
    point(0.0f, 0.0f, 5.1f),     // past max_z
    point(0.0f, 0.0f, -5.1f),    // past min_z
    point(-1.0f, 2.0f, -3.0f)};  // inside

  const auto cloud = makeCloud(input);
  const auto output = filter.filter(cloud);
  ASSERT_NE(output, nullptr);

  const auto kept = readBackPoints(*output);
  ASSERT_EQ(output->width, 3u);
  ASSERT_EQ(kept.size(), 3u);
  EXPECT_FLOAT_EQ(kept[0].x, 0.0f);
  EXPECT_FLOAT_EQ(kept[1].x, 9.9f);
  EXPECT_FLOAT_EQ(kept[2].x, -1.0f);

  // The output is a flat cloud regardless of the input's organisation, and the
  // cropped result has no non-finite points left in it.
  EXPECT_EQ(output->height, 1u);
  EXPECT_TRUE(output->is_dense);
  EXPECT_EQ(output->row_step, output->width * output->point_step);
}

TEST_F(CudaCropBoxFilterTest, BoundsAreInclusiveOnAllSixFaces)
{
  CudaCropBoxFilter filter(box(false));

  // One point per face, exactly on the plane and interior on the other two axes.
  // The CPU component uses >= / <=, so a point on a face belongs to the box; an
  // off-by-one to strict comparison here would quietly shave the map edges.
  const std::vector<TestPoint> input = {point(-10.0f, 0.0f, 0.0f), point(10.0f, 0.0f, 0.0f),
                                        point(0.0f, -10.0f, 0.0f), point(0.0f, 10.0f, 0.0f),
                                        point(0.0f, 0.0f, -5.0f),  point(0.0f, 0.0f, 5.0f)};

  const auto cloud = makeCloud(input);
  const auto output = filter.filter(cloud);
  ASSERT_NE(output, nullptr);
  EXPECT_EQ(output->width, 6u);

  // The corner where three faces meet is the same rule applied three times.
  const std::vector<TestPoint> corner = {point(10.0f, 10.0f, 5.0f), point(-10.0f, -10.0f, -5.0f)};
  const auto corner_cloud = makeCloud(corner);
  const auto corner_output = filter.filter(corner_cloud);
  ASSERT_NE(corner_output, nullptr);
  EXPECT_EQ(corner_output->width, 2u);
}

TEST_F(CudaCropBoxFilterTest, NegativeInvertsTheSelection)
{
  const std::vector<TestPoint> input = {
    point(0.0f, 0.0f, 0.0f), point(20.0f, 0.0f, 0.0f), point(0.0f, -30.0f, 0.0f),
    point(1.0f, 1.0f, 1.0f)};

  CudaCropBoxFilter positive(box(false));
  const auto inside_cloud = makeCloud(input);
  const auto inside = positive.filter(inside_cloud);
  ASSERT_NE(inside, nullptr);
  EXPECT_EQ(inside->width, 2u);

  CudaCropBoxFilter negative(box(true));
  const auto outside_cloud = makeCloud(input);
  const auto outside = negative.filter(outside_cloud);
  ASSERT_NE(outside, nullptr);
  ASSERT_EQ(outside->width, 2u);

  const auto kept = readBackPoints(*outside);
  EXPECT_FLOAT_EQ(kept[0].x, 20.0f);
  EXPECT_FLOAT_EQ(kept[1].y, -30.0f);

  // A face point is inside, so negative must exclude it -- the inclusive bound
  // and the inversion have to agree, or a point lands in both halves or neither.
  const std::vector<TestPoint> face = {point(10.0f, 0.0f, 0.0f)};
  const auto face_cloud = makeCloud(face);
  const auto face_output = negative.filter(face_cloud);
  ASSERT_NE(face_output, nullptr);
  EXPECT_EQ(face_output->width, 0u);
}

TEST_F(CudaCropBoxFilterTest, NonFinitePointsAreDroppedInBothPolarities)
{
  const float nan_value = std::numeric_limits<float>::quiet_NaN();
  const float inf_value = std::numeric_limits<float>::infinity();

  const std::vector<TestPoint> input = {point(nan_value, 0.0f, 0.0f), point(0.0f, nan_value, 0.0f),
                                        point(0.0f, 0.0f, nan_value), point(inf_value, 0.0f, 0.0f),
                                        point(1.0f, 1.0f, 1.0f),      point(50.0f, 50.0f, 50.0f)};

  CudaCropBoxFilter positive(box(false));
  const auto positive_cloud = makeCloud(input);
  const auto inside = positive.filter(positive_cloud);
  ASSERT_NE(inside, nullptr);
  ASSERT_EQ(inside->width, 1u);
  EXPECT_FLOAT_EQ(readBackPoints(*inside)[0].x, 1.0f);

  // The one that matters: every comparison against NaN is false, so a filter
  // that computed keep as !inside would hand NDT four NaN points here.
  CudaCropBoxFilter negative(box(true));
  const auto negative_cloud = makeCloud(input);
  const auto outside = negative.filter(negative_cloud);
  ASSERT_NE(outside, nullptr);
  ASSERT_EQ(outside->width, 1u);

  const auto kept = readBackPoints(*outside);
  EXPECT_FLOAT_EQ(kept[0].x, 50.0f);
  EXPECT_TRUE(std::isfinite(kept[0].x));
  EXPECT_TRUE(std::isfinite(kept[0].y));
  EXPECT_TRUE(std::isfinite(kept[0].z));
}

TEST_F(CudaCropBoxFilterTest, WholePointBlobSurvivesByteForByte)
{
  CudaCropBoxFilter filter(box(false));

  std::vector<TestPoint> input;
  for (int i = 0; i < 64; ++i) {
    // Alternate inside and outside so the kept points are not contiguous in the
    // input and the scatter actually has to move them.
    const float radius = (i % 2 == 0) ? 1.0f : 40.0f;
    TestPoint p = point(
      radius * static_cast<float>(i % 5 - 2), radius, 0.5f * static_cast<float>(i % 7 - 3),
      static_cast<float>(i) * 1.25f, static_cast<std::uint16_t>(i * 37));
    p.pad = static_cast<std::uint16_t>(0xA5A5u ^ i);  // a real layout's hole carries bits too
    input.push_back(p);
  }

  std::vector<std::uint8_t> input_bytes(input.size() * sizeof(TestPoint));
  std::memcpy(input_bytes.data(), input.data(), input_bytes.size());

  const auto cloud = makeCloud(input);
  const auto output = filter.filter(cloud);
  ASSERT_NE(output, nullptr);
  ASSERT_GT(output->width, 0u);
  EXPECT_EQ(output->point_step, cloud.point_step);
  EXPECT_EQ(output->fields.size(), cloud.fields.size());

  // Recompute the expected selection on the host, then compare the raw
  // point_step blobs. Comparing fields would only prove the fields this filter
  // already reads survived.
  std::vector<std::size_t> expected;
  for (std::size_t i = 0; i < input.size(); ++i) {
    const TestPoint & p = input[i];
    if (
      p.x >= -10.0f && p.x <= 10.0f && p.y >= -10.0f && p.y <= 10.0f && p.z >= -5.0f &&
      p.z <= 5.0f) {
      expected.push_back(i);
    }
  }
  ASSERT_EQ(output->width, expected.size());

  const auto output_bytes = readBack(*output);
  const std::size_t step = cloud.point_step;
  for (std::size_t k = 0; k < expected.size(); ++k) {
    const std::uint8_t * src = input_bytes.data() + expected[k] * step;
    const std::uint8_t * dst = output_bytes.data() + k * step;
    EXPECT_EQ(std::memcmp(src, dst, step), 0)
      << "point " << k << " (input index " << expected[k] << ") was not copied verbatim";
  }
}

TEST_F(CudaCropBoxFilterTest, EmptyInputYieldsEmptyOutput)
{
  CudaCropBoxFilter filter(box(false));

  const auto cloud = makeCloud({});
  const auto output = filter.filter(cloud);
  ASSERT_NE(output, nullptr);
  EXPECT_EQ(output->width, 0u);
  EXPECT_EQ(output->height, 1u);
  EXPECT_EQ(output->row_step, 0u);
  EXPECT_EQ(output->fields.size(), cloud.fields.size());

  // A cloud whose points are all rejected takes the same path as an empty one.
  const auto all_outside = makeCloud({point(100.0f, 100.0f, 100.0f)});
  const auto empty_result = filter.filter(all_outside);
  ASSERT_NE(empty_result, nullptr);
  EXPECT_EQ(empty_result->width, 0u);
  EXPECT_EQ(empty_result->row_step, 0u);
}

TEST_F(CudaCropBoxFilterTest, RejectsALayoutWithoutFloat32Xyz)
{
  CudaCropBoxFilter filter(box(false));

  // Same field names and offsets, declared FLOAT64. Reading four bytes at those
  // offsets would produce plausible-looking garbage, so the filter must refuse
  // rather than crop on it.
  const auto cloud = makeCloud({point(0.0f, 0.0f, 0.0f)}, sensor_msgs::msg::PointField::FLOAT64);
  EXPECT_EQ(filter.filter(cloud), nullptr);

  // A cloud missing z entirely is refused for the same reason.
  auto missing_z = makeCloud({point(0.0f, 0.0f, 0.0f)});
  missing_z.fields.erase(missing_z.fields.begin() + 2);
  EXPECT_EQ(filter.filter(missing_z), nullptr);
}

}  // namespace
