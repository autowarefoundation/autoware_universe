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

#include "autoware/cuda_pointcloud_preprocessor/cuda_downsample_filter/cuda_random_downsample_filter.hpp"

#include <sensor_msgs/msg/point_field.hpp>

#include <cuda_runtime.h>
#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <set>
#include <string>
#include <vector>

namespace
{

using autoware::cuda_pointcloud_preprocessor::CudaRandomDownsampleFilter;

// x, y, z, intensity as float32 then ring as uint16, padded to a 4-byte
// multiple the way a real Velodyne layout is. The payload past xyz is the point
// of the test: this filter must never interpret it, only move it whole, so a
// torn or reordered copy shows up as a blob that matches no input point.
constexpr std::size_t kOffX = 0;
constexpr std::size_t kOffY = 4;
constexpr std::size_t kOffZ = 8;
constexpr std::size_t kOffIntensity = 12;
constexpr std::size_t kOffRing = 16;
constexpr std::size_t kPointStep = 20;

/// True when this machine has a usable GPU. The suite is written for the AGX
/// Orin; on a CI box without a device every test skips rather than fails,
/// because "no GPU here" is not evidence about the filter.
bool haveCudaDevice()
{
  int count = 0;
  return cudaGetDeviceCount(&count) == cudaSuccess && count > 0;
}

void addField(
  cuda_blackboard::CudaPointCloud2 & cloud, const std::string & name, std::uint32_t offset,
  std::uint8_t datatype)
{
  sensor_msgs::msg::PointField f;
  f.name = name;
  f.offset = offset;
  f.datatype = datatype;
  f.count = 1;
  cloud.fields.push_back(f);
}

/// Host-side bytes for `n` points, each carrying a payload unique to its index.
std::vector<std::uint8_t> makeHostPoints(std::size_t n)
{
  // Zeroed so the padding bytes are deterministic; otherwise a byte-for-byte
  // comparison would be testing the allocator.
  std::vector<std::uint8_t> bytes(n * kPointStep, 0);
  for (std::size_t i = 0; i < n; ++i) {
    std::uint8_t * p = bytes.data() + i * kPointStep;
    const float x = static_cast<float>(i);
    const float y = static_cast<float>(i) + 0.5f;
    const float z = -static_cast<float>(i);
    const float intensity = static_cast<float>(i) * 3.0f + 1.0f;
    const std::uint16_t ring = static_cast<std::uint16_t>(i % 32);
    std::memcpy(p + kOffX, &x, sizeof(float));
    std::memcpy(p + kOffY, &y, sizeof(float));
    std::memcpy(p + kOffZ, &z, sizeof(float));
    std::memcpy(p + kOffIntensity, &intensity, sizeof(float));
    std::memcpy(p + kOffRing, &ring, sizeof(std::uint16_t));
  }
  return bytes;
}

cuda_blackboard::CudaPointCloud2 makeCloud(const std::vector<std::uint8_t> & host_bytes)
{
  cuda_blackboard::CudaPointCloud2 cloud;
  cloud.header.frame_id = "base_link";
  addField(cloud, "x", kOffX, sensor_msgs::msg::PointField::FLOAT32);
  addField(cloud, "y", kOffY, sensor_msgs::msg::PointField::FLOAT32);
  addField(cloud, "z", kOffZ, sensor_msgs::msg::PointField::FLOAT32);
  addField(cloud, "intensity", kOffIntensity, sensor_msgs::msg::PointField::FLOAT32);
  addField(cloud, "ring", kOffRing, sensor_msgs::msg::PointField::UINT16);
  cloud.point_step = kPointStep;
  cloud.height = 1;
  cloud.width = static_cast<std::uint32_t>(host_bytes.size() / kPointStep);
  cloud.row_step = static_cast<std::uint32_t>(host_bytes.size());
  cloud.is_bigendian = false;
  cloud.is_dense = true;

  if (!host_bytes.empty()) {
    cloud.data = cuda_blackboard::make_unique<std::uint8_t[]>(host_bytes.size());
    EXPECT_EQ(
      cudaSuccess,
      cudaMemcpy(cloud.data.get(), host_bytes.data(), host_bytes.size(), cudaMemcpyHostToDevice));
  }
  return cloud;
}

std::vector<std::uint8_t> readBack(const cuda_blackboard::CudaPointCloud2 & cloud)
{
  const std::size_t n = static_cast<std::size_t>(cloud.width) * cloud.height * cloud.point_step;
  std::vector<std::uint8_t> bytes(n, 0);
  if (n > 0) {
    EXPECT_EQ(cudaSuccess, cudaMemcpy(bytes.data(), cloud.data.get(), n, cudaMemcpyDeviceToHost));
  }
  return bytes;
}

/// The set of whole-point blobs, as opaque strings — the comparison the filter's
/// contract is actually about.
std::set<std::string> blobs(const std::vector<std::uint8_t> & bytes)
{
  std::set<std::string> out;
  for (std::size_t i = 0; i + kPointStep <= bytes.size(); i += kPointStep) {
    out.emplace(reinterpret_cast<const char *>(bytes.data() + i), kPointStep);
  }
  return out;
}

}  // namespace

TEST(CudaRandomDownsampleFilter, FewerPointsThanBudgetPassThroughUnchanged)
{
  if (!haveCudaDevice()) {
    GTEST_SKIP() << "no CUDA device";
  }
  const auto host = makeHostPoints(100);
  const auto input = makeCloud(host);

  CudaRandomDownsampleFilter filter(5000);
  const auto output = filter.filter(input);

  ASSERT_NE(nullptr, output);
  EXPECT_EQ(100u, output->width);
  EXPECT_EQ(1u, output->height);
  EXPECT_EQ(kPointStep, output->point_step);
  EXPECT_EQ(100u * kPointStep, output->row_step);
  EXPECT_EQ(input.fields.size(), output->fields.size());
  // Not merely the same set: under the budget nothing is chosen, so the order
  // must survive too.
  EXPECT_EQ(host, readBack(*output));
}

TEST(CudaRandomDownsampleFilter, BudgetIsExactWhenInputIsLarger)
{
  if (!haveCudaDevice()) {
    GTEST_SKIP() << "no CUDA device";
  }
  const auto host = makeHostPoints(10000);
  const auto input = makeCloud(host);

  CudaRandomDownsampleFilter filter(5000);

  // Repeated, because the seed advances per call: an off-by-one in the sort
  // prefix would only show on some seeds.
  for (int call = 0; call < 3; ++call) {
    const auto output = filter.filter(input);
    ASSERT_NE(nullptr, output);
    EXPECT_EQ(5000u, output->width);
    EXPECT_EQ(1u, output->height);
    EXPECT_EQ(5000u * kPointStep, output->row_step);
  }
}

TEST(CudaRandomDownsampleFilter, EveryOutputPointIsAWholeInputPoint)
{
  if (!haveCudaDevice()) {
    GTEST_SKIP() << "no CUDA device";
  }
  const auto host = makeHostPoints(4096);
  const auto input = makeCloud(host);

  CudaRandomDownsampleFilter filter(1000);
  const auto output = filter.filter(input);

  ASSERT_NE(nullptr, output);
  ASSERT_EQ(1000u, output->width);

  const auto input_blobs = blobs(host);
  const auto out_bytes = readBack(*output);
  const auto output_blobs = blobs(out_bytes);

  // Distinct, so a duplicated selection cannot hide behind the count.
  EXPECT_EQ(1000u, output_blobs.size());
  for (const auto & blob : output_blobs) {
    EXPECT_EQ(1u, input_blobs.count(blob))
      << "an output point does not byte-match any input point: the payload past "
         "xyz was torn or reassembled";
  }
}

TEST(CudaRandomDownsampleFilter, EmptyInputYieldsEmptyCloud)
{
  if (!haveCudaDevice()) {
    GTEST_SKIP() << "no CUDA device";
  }
  const auto input = makeCloud({});

  CudaRandomDownsampleFilter filter(5000);
  const auto output = filter.filter(input);

  ASSERT_NE(nullptr, output);
  EXPECT_EQ(0u, output->width);
  EXPECT_EQ(0u, output->row_step);
  EXPECT_EQ(1u, output->height);
  EXPECT_EQ(nullptr, output->data.get());
}

TEST(CudaRandomDownsampleFilter, SelectionIsRandomAndAdvancesPerCall)
{
  if (!haveCudaDevice()) {
    GTEST_SKIP() << "no CUDA device";
  }
  const auto host = makeHostPoints(4096);
  const auto input = makeCloud(host);

  CudaRandomDownsampleFilter filter(1000);
  const auto first = blobs(readBack(*filter.filter(input)));
  const auto second = blobs(readBack(*filter.filter(input)));

  // Guards the two ways this can look right and be wrong: taking the first
  // sample_num points (a subset in input order, which the ascending re-sort
  // would disguise), and reusing one seed for every frame (which would bias
  // NDT towards the same points scan after scan). Two independent draws of
  // 1000 from 4096 agreeing exactly has probability far below any flake budget.
  EXPECT_NE(first, second);

  const auto prefix = blobs(std::vector<std::uint8_t>(
    host.begin(), host.begin() + static_cast<std::ptrdiff_t>(1000 * kPointStep)));
  EXPECT_NE(prefix, first);
}
