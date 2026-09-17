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

// Tests for GetIndicesPairsPlugin::enqueue.
//
// enqueue() does not synchronize the stream it is given. These tests check that everything it
// produces is nevertheless fully stream-ordered: a consumer enqueued on the same stream right
// after enqueue() returns must observe the final outputs, back-to-back calls that share one
// workspace must not interfere, the host-side count written via cudaMemcpyAsync must survive the
// caller's stack frame going away, and the whole call must be capturable into a CUDA graph (which
// forbids cudaStreamSynchronize on the capturing stream).

#include "autoware/tensorrt_plugins/get_indices_pairs_plugin.hpp"

#include "test_utils.hpp"

#include <autoware/cuda_utils/cuda_gtest_utils.hpp>

#include <NvInferRuntimePlugin.h>
#include <cuda_runtime_api.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <random>
#include <set>
#include <tuple>
#include <utility>
#include <vector>

namespace
{

using autoware::tensorrt_plugins::test::copy_to_device;
using autoware::tensorrt_plugins::test::copy_to_host;
using autoware::tensorrt_plugins::test::CudaStreamGuard;
using autoware::tensorrt_plugins::test::DeviceBuffer;

constexpr std::int32_t kCoordDims = 4;  // batch index + 3 spatial coordinates
constexpr std::array<std::int32_t, 3> kKernelSize{3, 3, 3};
constexpr std::array<std::int32_t, 3> kSpatialShape{12, 10, 8};
constexpr std::int32_t kKernelVolume = kKernelSize[0] * kKernelSize[1] * kKernelSize[2];
constexpr std::int32_t kSentinel = -1;

nvinfer1::Dims to_dims(const std::vector<std::int32_t> & values)
{
  nvinfer1::Dims dims{};
  dims.nbDims = static_cast<std::int32_t>(values.size());
  for (std::size_t i = 0; i < values.size(); ++i) {
    dims.d[i] = values[i];
  }
  return dims;
}

nvinfer1::plugin::GetIndicesPairsParameters make_subm_params()
{
  nvinfer1::plugin::GetIndicesPairsParameters params{};
  params.batch_size = 1;
  params.algo = 0;
  params.dilation = {1, 1, 1};
  params.ksize = {kKernelSize[0], kKernelSize[1], kKernelSize[2]};
  params.out_padding = {0, 0, 0};
  params.padding = {1, 1, 1};
  params.spatial_shape = {kSpatialShape[0], kSpatialShape[1], kSpatialShape[2]};
  params.stride = {1, 1, 1};
  params.subm = 1;  // cSpell:ignore subm
  params.transpose = 0;

  params.dilation_dims = to_dims(params.dilation);
  params.ksize_dims = to_dims(params.ksize);
  params.out_padding_dims = to_dims(params.out_padding);
  params.padding_dims = to_dims(params.padding);
  params.spatial_shape_dims = to_dims(params.spatial_shape);
  params.stride_dims = to_dims(params.stride);
  return params;
}

// Distinct voxel coordinates (batch 0) laid out as [num_points, 4].
std::vector<std::int32_t> make_random_indices(
  const std::int32_t num_points, const std::uint32_t seed)
{
  std::mt19937 rng(seed);
  std::set<std::tuple<std::int32_t, std::int32_t, std::int32_t>> used;
  std::vector<std::int32_t> indices;
  indices.reserve(static_cast<std::size_t>(num_points) * kCoordDims);

  while (static_cast<std::int32_t>(used.size()) < num_points) {
    std::array<std::int32_t, 3> coord{};
    for (std::size_t d = 0; d < 3; ++d) {
      coord[d] = static_cast<std::int32_t>(rng() % static_cast<std::uint32_t>(kSpatialShape[d]));
    }
    if (!used.emplace(coord[0], coord[1], coord[2]).second) {
      continue;
    }
    indices.push_back(0);
    indices.insert(indices.end(), coord.begin(), coord.end());
  }
  return indices;
}

// (input index, output index) pairs per kernel offset, as an order-independent set.
using PairSets = std::vector<std::set<std::pair<std::int32_t, std::int32_t>>>;

// Reference for a submanifold convolution: for every active voxel i and every kernel offset k the
// neighbor at position(i) - offset(k) is looked up (spconv's convention: the kernel offset is
// measured from the output voxel to the input voxel); if it is active (index j), (i, j) is a pair
// for offset k. The center offset pairs every voxel with itself.
PairSets make_subm_reference(const std::vector<std::int32_t> & indices)
{
  const std::int32_t num_points = static_cast<std::int32_t>(indices.size() / kCoordDims);
  std::map<std::tuple<std::int32_t, std::int32_t, std::int32_t>, std::int32_t> lookup;
  for (std::int32_t i = 0; i < num_points; ++i) {
    const std::int32_t * c = &indices[static_cast<std::size_t>(i) * kCoordDims];
    lookup.emplace(std::make_tuple(c[1], c[2], c[3]), i);
  }

  PairSets reference(kKernelVolume);
  for (std::int32_t i = 0; i < num_points; ++i) {
    const std::int32_t * c = &indices[static_cast<std::size_t>(i) * kCoordDims];
    for (std::int32_t k = 0; k < kKernelVolume; ++k) {
      const std::int32_t k0 = k / (kKernelSize[1] * kKernelSize[2]);
      const std::int32_t k1 = (k / kKernelSize[2]) % kKernelSize[1];
      const std::int32_t k2 = k % kKernelSize[2];
      const auto neighbor = std::make_tuple(
        c[1] - (k0 - kKernelSize[0] / 2), c[2] - (k1 - kKernelSize[1] / 2),
        c[3] - (k2 - kKernelSize[2] / 2));
      const auto it = lookup.find(neighbor);
      if (it != lookup.end()) {
        reference[k].emplace(i, it->second);
      }
    }
  }
  return reference;
}

struct HostOutputs
{
  std::vector<std::int32_t> out_indices;  // [num_points, 4]
  std::vector<std::int32_t> pairs;        // [2, kernel_volume, num_points]
  std::vector<std::int32_t> pair_counts;  // [kernel_volume]
  std::int32_t num_act_out{kSentinel};
};

// Device-side buffers for the 4 plugin outputs, sized for `capacity` points.
struct DeviceOutputs
{
  explicit DeviceOutputs(const std::int32_t capacity)
  : capacity_(capacity),
    out_indices(static_cast<std::size_t>(capacity) * kCoordDims),
    pairs(static_cast<std::size_t>(2) * kKernelVolume * capacity),
    pair_counts(kKernelVolume),
    num_act_out(1)
  {
  }

  void fill_sentinel(cudaStream_t stream) const
  {
    ASSERT_EQ(
      cudaMemsetAsync(
        out_indices.get(), 0xFF, sizeof(std::int32_t) * capacity_ * kCoordDims, stream),
      cudaSuccess);
    ASSERT_EQ(
      cudaMemsetAsync(
        pairs.get(), 0xFF, sizeof(std::int32_t) * 2 * kKernelVolume * capacity_, stream),
      cudaSuccess);
    ASSERT_EQ(
      cudaMemsetAsync(pair_counts.get(), 0xFF, sizeof(std::int32_t) * kKernelVolume, stream),
      cudaSuccess);
    ASSERT_EQ(cudaMemsetAsync(num_act_out.get(), 0xFF, sizeof(std::int32_t), stream), cudaSuccess);
  }

  // Stream-ordered device-to-device copy of every output into `dst` (no host sync).
  void snapshot_into(
    const DeviceOutputs & dst, const std::int32_t num_points, cudaStream_t stream) const
  {
    ASSERT_EQ(
      cudaMemcpyAsync(
        dst.out_indices.get(), out_indices.get(), sizeof(std::int32_t) * num_points * kCoordDims,
        cudaMemcpyDeviceToDevice, stream),
      cudaSuccess);
    ASSERT_EQ(
      cudaMemcpyAsync(
        dst.pairs.get(), pairs.get(), sizeof(std::int32_t) * 2 * kKernelVolume * num_points,
        cudaMemcpyDeviceToDevice, stream),
      cudaSuccess);
    ASSERT_EQ(
      cudaMemcpyAsync(
        dst.pair_counts.get(), pair_counts.get(), sizeof(std::int32_t) * kKernelVolume,
        cudaMemcpyDeviceToDevice, stream),
      cudaSuccess);
    ASSERT_EQ(
      cudaMemcpyAsync(
        dst.num_act_out.get(), num_act_out.get(), sizeof(std::int32_t), cudaMemcpyDeviceToDevice,
        stream),
      cudaSuccess);
  }

  // Blocking read-back (only used once the stream has been synchronized).
  HostOutputs to_host(const std::int32_t num_points) const
  {
    HostOutputs host;
    host.out_indices =
      copy_to_host(out_indices.get(), static_cast<std::size_t>(num_points) * kCoordDims);
    host.pairs =
      copy_to_host(pairs.get(), static_cast<std::size_t>(2) * kKernelVolume * num_points);
    host.pair_counts = copy_to_host(pair_counts.get(), kKernelVolume);
    host.num_act_out = copy_to_host(num_act_out.get(), 1).front();
    return host;
  }

  std::array<void *, 4> pointers() const
  {
    return {out_indices.get(), pairs.get(), pair_counts.get(), num_act_out.get()};
  }

  std::int32_t capacity_;
  DeviceBuffer<std::int32_t> out_indices;
  DeviceBuffer<std::int32_t> pairs;
  DeviceBuffer<std::int32_t> pair_counts;
  DeviceBuffer<std::int32_t> num_act_out;
};

class GetIndicesPairsPluginTest : public autoware::cuda_utils::CudaTest
{
protected:
  void SetUp() override
  {
    autoware::cuda_utils::CudaTest::SetUp();
    if (IsSkipped()) {
      return;
    }
    plugin_ = std::make_unique<nvinfer1::plugin::GetIndicesPairsPlugin>("test", make_subm_params());

    // getWorkspaceSize() ignores the descriptors, but pass well-formed ones anyway.
    nvinfer1::DynamicPluginTensorDesc in{};
    in.desc.dims = to_dims({1, kCoordDims});
    in.desc.type = nvinfer1::DataType::kINT32;
    in.desc.format = nvinfer1::TensorFormat::kLINEAR;
    std::array<nvinfer1::DynamicPluginTensorDesc, 4> out{};
    workspace_size_ = plugin_->getWorkspaceSize(&in, 1, out.data(), out.size());
    ASSERT_GT(workspace_size_, 0U);
    workspace_ = std::make_unique<DeviceBuffer<std::uint8_t>>(workspace_size_);
  }

  // Issues one plugin call on `stream`. Returns without synchronizing.
  void enqueue(
    const std::int32_t * device_indices, const std::int32_t num_points,
    const DeviceOutputs & outputs, cudaStream_t stream)
  {
    nvinfer1::PluginTensorDesc input_desc{};
    input_desc.dims = to_dims({num_points, kCoordDims});
    input_desc.type = nvinfer1::DataType::kINT32;
    input_desc.format = nvinfer1::TensorFormat::kLINEAR;
    std::array<nvinfer1::PluginTensorDesc, 4> output_desc{};

    const void * inputs[1] = {device_indices};
    const auto output_pointers = outputs.pointers();

    const std::int32_t status = plugin_->enqueue(
      &input_desc, output_desc.data(), inputs, output_pointers.data(), workspace_->get(), stream);
    ASSERT_EQ(status, static_cast<std::int32_t>(cudaSuccess));
  }

  static void expect_matches_reference(
    const HostOutputs & host, const std::vector<std::int32_t> & indices,
    const bool check_num_act_out = true)
  {
    const std::int32_t num_points = static_cast<std::int32_t>(indices.size() / kCoordDims);
    const PairSets reference = make_subm_reference(indices);

    if (check_num_act_out) {
      EXPECT_EQ(host.num_act_out, num_points);
    }
    EXPECT_EQ(host.out_indices, indices);

    // spconv exploits the symmetry of submanifold convolutions: pairs are generated for the
    // offsets before the center only, each also written mirrored (i and j swapped) into the slice
    // of the opposite offset. The per-offset counts are therefore filled for the first half only;
    // the center offset (every voxel paired with itself) is implicit and its count stays zero.
    constexpr std::int32_t center = kKernelVolume / 2;
    for (std::int32_t k = 0; k < kKernelVolume; ++k) {
      const std::int32_t count = host.pair_counts[k];
      ASSERT_GE(count, 0) << "kernel offset " << k << " count still holds the sentinel";
      ASSERT_LE(count, num_points) << "kernel offset " << k;
      if (k == center) {
        EXPECT_EQ(count, 0) << "kernel offset " << k;
        continue;
      }
      if (k > center) {
        EXPECT_EQ(count, 0) << "kernel offset " << k;
      } else {
        EXPECT_EQ(static_cast<std::size_t>(count), reference[k].size()) << "kernel offset " << k;
      }
      const std::int32_t valid = k < center ? count : host.pair_counts[kKernelVolume - 1 - k];

      std::set<std::pair<std::int32_t, std::int32_t>> actual;
      for (std::int32_t n = 0; n < valid; ++n) {
        const std::size_t in_offset = static_cast<std::size_t>(k) * num_points + n;
        const std::size_t out_offset =
          static_cast<std::size_t>(kKernelVolume) * num_points + in_offset;
        actual.emplace(host.pairs[in_offset], host.pairs[out_offset]);
      }
      EXPECT_EQ(actual, reference[k]) << "kernel offset " << k;
    }
  }

  std::unique_ptr<nvinfer1::plugin::GetIndicesPairsPlugin> plugin_;
  std::size_t workspace_size_{0U};
  std::unique_ptr<DeviceBuffer<std::uint8_t>> workspace_;
};

// Deliberately clobbers a large stack region after enqueue() has returned, so that a plugin that
// relied on its own stack frame surviving until the copy executes would write garbage.
__attribute__((noinline)) std::int32_t trash_stack()
{
  volatile std::int32_t garbage[4096];
  for (std::size_t i = 0; i < 4096; ++i) {
    garbage[i] = static_cast<std::int32_t>(0x7EADBEEF ^ i);
  }
  return garbage[4095];
}

TEST_F(GetIndicesPairsPluginTest, OutputsMatchCpuReference)
{
  constexpr std::int32_t num_points = 300;
  const std::vector<std::int32_t> indices = make_random_indices(num_points, 1U);

  CudaStreamGuard stream;
  DeviceBuffer<std::int32_t> device_indices(indices.size());
  copy_to_device(device_indices.get(), indices);
  DeviceOutputs outputs(num_points);
  outputs.fill_sentinel(stream.get());

  enqueue(device_indices.get(), num_points, outputs, stream.get());
  ASSERT_EQ(cudaStreamSynchronize(stream.get()), cudaSuccess);

  expect_matches_reference(outputs.to_host(num_points), indices);
}

// A consumer enqueued on the same stream immediately after enqueue() returns, with no host
// synchronization in between, must observe the completed outputs (not the sentinel prefill).
TEST_F(GetIndicesPairsPluginTest, StreamOrderedConsumerSeesFinalOutputs)
{
  constexpr std::int32_t num_points = 300;
  const std::vector<std::int32_t> indices = make_random_indices(num_points, 2U);

  CudaStreamGuard stream;
  DeviceBuffer<std::int32_t> device_indices(indices.size());
  copy_to_device(device_indices.get(), indices);
  DeviceOutputs outputs(num_points);
  DeviceOutputs snapshot(num_points);
  outputs.fill_sentinel(stream.get());
  snapshot.fill_sentinel(stream.get());

  enqueue(device_indices.get(), num_points, outputs, stream.get());
  outputs.snapshot_into(snapshot, num_points, stream.get());
  ASSERT_EQ(cudaStreamSynchronize(stream.get()), cudaSuccess);

  expect_matches_reference(snapshot.to_host(num_points), indices);
}

// The host-side element count is uploaded with cudaMemcpyAsync from a local variable of
// enqueue(). Its value must not depend on that stack frame staying intact.
TEST_F(GetIndicesPairsPluginTest, NumActOutSurvivesCallerStackReuse)
{
  constexpr std::int32_t num_points = 123;
  const std::vector<std::int32_t> indices = make_random_indices(num_points, 3U);

  CudaStreamGuard stream;
  DeviceBuffer<std::int32_t> device_indices(indices.size());
  copy_to_device(device_indices.get(), indices);
  DeviceOutputs outputs(num_points);
  outputs.fill_sentinel(stream.get());

  enqueue(device_indices.get(), num_points, outputs, stream.get());
  trash_stack();
  ASSERT_EQ(cudaStreamSynchronize(stream.get()), cudaSuccess);

  EXPECT_EQ(copy_to_host(outputs.num_act_out.get(), 1).front(), num_points);
}

// Several calls with different inputs share one workspace and are issued back-to-back on one
// stream without any host synchronization. Each call's outputs are snapshotted (stream-ordered)
// right after it and must match that call's own input.
TEST_F(GetIndicesPairsPluginTest, BackToBackCallsDoNotInterfere)
{
  constexpr std::array<std::int32_t, 3> point_counts{250, 40, 400};
  constexpr std::int32_t rounds = 3;
  const std::int32_t max_points = *std::max_element(point_counts.begin(), point_counts.end());

  CudaStreamGuard stream;
  std::vector<std::vector<std::int32_t>> host_indices;
  std::vector<std::unique_ptr<DeviceBuffer<std::int32_t>>> device_indices;
  for (std::size_t i = 0; i < point_counts.size(); ++i) {
    host_indices.push_back(
      make_random_indices(point_counts[i], 10U + static_cast<std::uint32_t>(i)));
    device_indices.push_back(std::make_unique<DeviceBuffer<std::int32_t>>(host_indices[i].size()));
    copy_to_device(device_indices[i]->get(), host_indices[i]);
  }

  DeviceOutputs outputs(max_points);
  std::vector<std::unique_ptr<DeviceOutputs>> snapshots;
  for (std::int32_t round = 0; round < rounds; ++round) {
    for (std::size_t i = 0; i < point_counts.size(); ++i) {
      outputs.fill_sentinel(stream.get());
      enqueue(device_indices[i]->get(), point_counts[i], outputs, stream.get());
      snapshots.push_back(std::make_unique<DeviceOutputs>(point_counts[i]));
      outputs.snapshot_into(*snapshots.back(), point_counts[i], stream.get());
    }
  }
  ASSERT_EQ(cudaStreamSynchronize(stream.get()), cudaSuccess);

  for (std::size_t s = 0; s < snapshots.size(); ++s) {
    const std::size_t i = s % point_counts.size();
    expect_matches_reference(snapshots[s]->to_host(point_counts[i]), host_indices[i]);
  }
}

// cudaStreamSynchronize on a capturing stream is illegal, so a plugin that still synchronized
// internally could not be captured into a CUDA graph. Capture one call, replay the graph, and
// check the device-computed outputs.
//
// The element count is deliberately not checked here: it is uploaded with cudaMemcpyAsync from
// pageable host memory (a local of enqueue()). Outside of capture the value is staged at call
// time and therefore correct (see NumActOutSurvivesCallerStackReuse); inside a capture the node
// re-reads the host address at replay time, by which point the local is gone. TensorRT does not
// capture layers with data-dependent output shapes into graphs, so this is not reachable through
// TensorRT; making the count graph-safe would need a persistent (pinned) host buffer.
TEST_F(GetIndicesPairsPluginTest, IsCudaGraphCapturable)
{
  constexpr std::int32_t num_points = 300;
  const std::vector<std::int32_t> indices = make_random_indices(num_points, 4U);

  CudaStreamGuard stream;
  DeviceBuffer<std::int32_t> device_indices(indices.size());
  copy_to_device(device_indices.get(), indices);
  DeviceOutputs outputs(num_points);
  outputs.fill_sentinel(stream.get());
  ASSERT_EQ(cudaStreamSynchronize(stream.get()), cudaSuccess);

  cudaGraph_t graph{nullptr};
  ASSERT_EQ(cudaStreamBeginCapture(stream.get(), cudaStreamCaptureModeThreadLocal), cudaSuccess);
  enqueue(device_indices.get(), num_points, outputs, stream.get());
  ASSERT_EQ(cudaStreamEndCapture(stream.get(), &graph), cudaSuccess);
  ASSERT_NE(graph, nullptr);

  cudaGraphExec_t graph_exec{nullptr};
  ASSERT_EQ(cudaGraphInstantiate(&graph_exec, graph, 0), cudaSuccess);
  ASSERT_EQ(cudaGraphLaunch(graph_exec, stream.get()), cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(stream.get()), cudaSuccess);

  expect_matches_reference(outputs.to_host(num_points), indices, false);

  EXPECT_EQ(cudaGraphExecDestroy(graph_exec), cudaSuccess);
  EXPECT_EQ(cudaGraphDestroy(graph), cudaSuccess);
}

}  // namespace
