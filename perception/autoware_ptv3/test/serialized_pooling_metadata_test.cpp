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

#include "autoware/ptv3/preprocess/preprocess_kernel.hpp"
#include "autoware/ptv3/ptv3_config.hpp"
#include "ptv3_test_fixture.hpp"

#include <autoware/cuda_utils/cuda_unique_ptr.hpp>

#include <cuda_runtime_api.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <map>
#include <numeric>
#include <string>
#include <vector>

namespace autoware::ptv3
{
namespace test
{

struct DeviceStage
{
  DeviceStage(const std::size_t capacity, const std::size_t num_orders)
  : indices(autoware::cuda_utils::make_unique<std::int64_t[]>(capacity)),
    indptr(autoware::cuda_utils::make_unique<std::int64_t[]>(capacity + 1)),
    head_indices(autoware::cuda_utils::make_unique<std::int64_t[]>(capacity)),
    cluster(autoware::cuda_utils::make_unique<std::int64_t[]>(capacity)),
    grid_coord(autoware::cuda_utils::make_unique<std::int32_t[]>(capacity * 3)),
    serialized_code(autoware::cuda_utils::make_unique<std::int64_t[]>(capacity * num_orders)),
    serialized_order(autoware::cuda_utils::make_unique<std::int64_t[]>(capacity * num_orders)),
    serialized_inverse(autoware::cuda_utils::make_unique<std::int64_t[]>(capacity * num_orders))
  {
  }

  CudaUniquePtr<std::int64_t[]> indices;
  CudaUniquePtr<std::int64_t[]> indptr;
  CudaUniquePtr<std::int64_t[]> head_indices;
  CudaUniquePtr<std::int64_t[]> cluster;
  CudaUniquePtr<std::int32_t[]> grid_coord;
  CudaUniquePtr<std::int64_t[]> serialized_code;
  CudaUniquePtr<std::int64_t[]> serialized_order;
  CudaUniquePtr<std::int64_t[]> serialized_inverse;
};

struct CpuStage
{
  std::vector<std::int64_t> indices;
  std::vector<std::int64_t> indptr;
  std::vector<std::int64_t> head_indices;
  std::vector<std::int64_t> cluster;
  std::vector<std::int32_t> grid_coord;
  std::vector<std::int64_t> serialized_code;
  std::vector<std::int64_t> serialized_order;
  std::vector<std::int64_t> serialized_inverse;
};

std::int32_t pooling_depth(const std::int64_t stride)
{
  std::int32_t depth = 0;
  for (auto value = stride; value > 1; value >>= 1) {
    ++depth;
  }
  return depth;
}

std::vector<std::int64_t> make_serialized_code(
  const std::vector<std::int32_t> & grid_coord, const std::int32_t depth)
{
  const auto count = grid_coord.size() / 3;
  std::vector<std::int64_t> code(2 * count);
  for (std::size_t index = 0; index < count; ++index) {
    const auto x = grid_coord[index * 3 + 0];
    const auto y = grid_coord[index * 3 + 1];
    const auto z = grid_coord[index * 3 + 2];
    code[index] = serialize_coord(x, y, z, depth, false);
    code[count + index] = serialize_coord(x, y, z, depth, true);
  }
  return code;
}

// generateSerializedPoolingMetadata requires its input sorted by order-0 serialized code (as
// generateFeatures emits it), so hand-built levels must be sorted the same way.
std::vector<std::int32_t> sort_grid_coord_by_order0(
  const std::vector<std::int32_t> & grid_coord, const std::int32_t depth)
{
  const auto count = grid_coord.size() / 3;
  std::vector<std::size_t> order(count);
  std::iota(order.begin(), order.end(), 0);
  const auto code_of = [&grid_coord, depth](const std::size_t index) {
    return serialize_coord(
      grid_coord[index * 3 + 0], grid_coord[index * 3 + 1], grid_coord[index * 3 + 2], depth,
      false);
  };
  std::stable_sort(order.begin(), order.end(), [&code_of](const auto lhs, const auto rhs) {
    return code_of(lhs) < code_of(rhs);
  });

  std::vector<std::int32_t> sorted(grid_coord.size());
  for (std::size_t rank = 0; rank < count; ++rank) {
    for (std::size_t coord = 0; coord < 3; ++coord) {
      sorted[rank * 3 + coord] = grid_coord[order[rank] * 3 + coord];
    }
  }
  return sorted;
}

std::vector<std::int64_t> stable_argsort(const std::vector<std::int64_t> & values)
{
  std::vector<std::int64_t> order(values.size());
  std::iota(order.begin(), order.end(), 0);
  std::stable_sort(order.begin(), order.end(), [&values](const auto lhs, const auto rhs) {
    return values[static_cast<std::size_t>(lhs)] < values[static_cast<std::size_t>(rhs)];
  });
  return order;
}

CpuStage make_stage_reference(
  const std::vector<std::int32_t> & grid_coord_in,
  const std::vector<std::int64_t> & serialized_code_in, const std::size_t num_orders,
  const std::int64_t stride)
{
  const auto input_count = grid_coord_in.size() / 3;
  const auto depth = pooling_depth(stride);
  std::vector<std::int64_t> pooled_keys(input_count);
  for (std::size_t index = 0; index < input_count; ++index) {
    pooled_keys[index] = serialized_code_in[index] >> (depth * 3);
  }

  std::vector<std::int64_t> unique_keys = pooled_keys;
  std::sort(unique_keys.begin(), unique_keys.end());
  unique_keys.erase(std::unique(unique_keys.begin(), unique_keys.end()), unique_keys.end());

  std::map<std::int64_t, std::int64_t> key_to_cluster;
  for (std::size_t index = 0; index < unique_keys.size(); ++index) {
    key_to_cluster.emplace(unique_keys[index], static_cast<std::int64_t>(index));
  }

  CpuStage stage;
  stage.cluster.resize(input_count);
  stage.indptr.assign(unique_keys.size() + 1, 0);
  for (std::size_t index = 0; index < input_count; ++index) {
    const auto cluster = key_to_cluster.at(pooled_keys[index]);
    stage.cluster[index] = cluster;
    ++stage.indptr[static_cast<std::size_t>(cluster + 1)];
  }
  for (std::size_t index = 1; index < stage.indptr.size(); ++index) {
    stage.indptr[index] += stage.indptr[index - 1];
  }

  stage.indices = stable_argsort(stage.cluster);
  stage.head_indices.resize(unique_keys.size());
  for (std::size_t segment = 0; segment < unique_keys.size(); ++segment) {
    stage.head_indices[segment] = stage.indices[static_cast<std::size_t>(stage.indptr[segment])];
  }

  stage.grid_coord.resize(unique_keys.size() * 3);
  stage.serialized_code.resize(num_orders * unique_keys.size());
  for (std::size_t segment = 0; segment < unique_keys.size(); ++segment) {
    const auto source = static_cast<std::size_t>(stage.head_indices[segment]);
    for (std::size_t coord = 0; coord < 3; ++coord) {
      stage.grid_coord[segment * 3 + coord] = grid_coord_in[source * 3 + coord] >> depth;
    }
    for (std::size_t order = 0; order < num_orders; ++order) {
      stage.serialized_code[order * unique_keys.size() + segment] =
        serialized_code_in[order * input_count + source] >> (depth * 3);
    }
  }

  stage.serialized_order.resize(num_orders * unique_keys.size());
  stage.serialized_inverse.resize(num_orders * unique_keys.size());
  for (std::size_t order = 0; order < num_orders; ++order) {
    std::vector<std::int64_t> order_codes(unique_keys.size());
    for (std::size_t index = 0; index < unique_keys.size(); ++index) {
      order_codes[index] = stage.serialized_code[order * unique_keys.size() + index];
    }
    const auto sorted_order = stable_argsort(order_codes);
    for (std::size_t rank = 0; rank < sorted_order.size(); ++rank) {
      const auto input_index = sorted_order[rank];
      stage.serialized_order[order * unique_keys.size() + rank] = input_index;
      stage.serialized_inverse[order * unique_keys.size() + static_cast<std::size_t>(input_index)] =
        static_cast<std::int64_t>(rank);
    }
  }
  return stage;
}

PTv3Config make_test_config()
{
  PTv3ConfigParams params;
  params.cloud_capacity = 64;
  params.voxels_num = {1, 16, 32};
  params.point_cloud_range = {0.0F, 0.0F, 0.0F, 64.0F, 64.0F, 64.0F};
  params.segmentation_class_names = {"class"};
  params.segmentation_class_mapping = {{"class", "NOISE"}};
  params.palette = {0, 0, 0};
  params.filter_output_format = "XYZI";
  params.source_reconstruction = "none";
  return makeConfig(params);
}

PTv3Config make_detection_test_config()
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
  return makeConfig(params);
}

template <typename T>
void expect_equal(
  const std::vector<T> & actual, const std::vector<T> & expected, const std::string & name)
{
  EXPECT_EQ(actual, expected) << name;
}

void expect_all_in_range(
  const std::vector<std::int64_t> & values, const std::int64_t upper_bound,
  const std::string & name)
{
  for (std::size_t i = 0; i < values.size(); ++i) {
    EXPECT_GE(values[i], 0) << name << " at index " << i;
    EXPECT_LT(values[i], upper_bound) << name << " at index " << i;
  }
}

void expect_permutation(const std::vector<std::int64_t> & values, const std::string & name)
{
  std::vector<std::int64_t> sorted = values;
  std::sort(sorted.begin(), sorted.end());
  std::vector<std::int64_t> expected(values.size());
  std::iota(expected.begin(), expected.end(), 0);
  EXPECT_EQ(sorted, expected) << name;
}

// A fixture whose serialization orders rank a level identically cannot detect an implementation
// that returns the same row for every order, so require the orders to disagree.
void expect_orders_diverge(
  const std::vector<std::int64_t> & order, const std::size_t count, const std::size_t num_orders,
  const std::string & name)
{
  ASSERT_GE(num_orders, 2u) << name;
  const auto row = [&order, count](const std::size_t index) {
    const auto begin = static_cast<std::ptrdiff_t>(index * count);
    return std::vector<std::int64_t>(
      order.begin() + begin, order.begin() + begin + static_cast<std::ptrdiff_t>(count));
  };
  const auto first = row(0);
  for (std::size_t index = 1; index < num_orders; ++index) {
    EXPECT_NE(first, row(index))
      << name << ": serialization orders 0 and " << index << " rank this level identically, so the "
      << "fixture cannot detect a wrong per-order derivation. Pick coordinates that vary in both x "
      << "and y.";
  }
}

class SerializedPoolingMetadataTest : public PTv3CudaTest
{
};

TEST_F(SerializedPoolingMetadataTest, DetectionGridCoord3StaysInsideBevGrid)
{
  const auto config = make_detection_test_config();
  constexpr std::size_t kNumOrders = 2;
  const auto grid_coord =
    sort_grid_coord_by_order0({0, 0, 0, 15, 15, 0}, config.serialization_depth_);
  const auto serialized_code = make_serialized_code(grid_coord, config.serialization_depth_);
  const auto num_voxels = static_cast<std::int64_t>(grid_coord.size() / 3);

  PreprocessCuda preprocess(config, stream_);
  auto grid_coord_d = makeDeviceBuffer<std::int32_t>(grid_coord.size());
  auto serialized_code_d = makeDeviceBuffer<std::int64_t>(serialized_code.size());
  auto stage_counts_d = makeDeviceBuffer<std::int64_t>(config.pooling_strides_.size() + 1);
  std::vector<DeviceStage> device_stages;
  std::vector<SerializedPoolingDeviceStageView> stage_views;
  for (std::size_t stage = 0; stage < config.pooling_strides_.size(); ++stage) {
    device_stages.emplace_back(config.max_num_voxels_, kNumOrders);
  }
  for (auto & stage : device_stages) {
    stage_views.push_back(
      SerializedPoolingDeviceStageView{
        stage.indices.get(), stage.indptr.get(), stage.head_indices.get(), stage.cluster.get(),
        stage.grid_coord.get(), stage.serialized_code.get(), stage.serialized_order.get(),
        stage.serialized_inverse.get()});
  }

  copyToDevice(grid_coord_d.get(), grid_coord);
  copyToDevice(serialized_code_d.get(), serialized_code);
  preprocess.generateSerializedPoolingMetadata(
    grid_coord_d.get(), serialized_code_d.get(), num_voxels, stage_views, stage_counts_d.get());
  ASSERT_EQ(cudaStreamSynchronize(stream_), cudaSuccess);

  const auto stage_counts = copyToHost(stage_counts_d.get(), config.pooling_strides_.size() + 1);
  // TODO(mojomex): generalize to other detection feature depth settings.
  const std::size_t point_grid_coord_3_stage = 2;
  const auto point_grid_coord_3_count =
    static_cast<std::size_t>(stage_counts[point_grid_coord_3_stage + 1]);
  const auto point_grid_coord_3 = copyToHost(
    device_stages[point_grid_coord_3_stage].grid_coord.get(), point_grid_coord_3_count * 3);
  for (std::size_t i = 0; i < point_grid_coord_3_count; ++i) {
    EXPECT_GE(point_grid_coord_3[i * 3 + 0], 0);
    EXPECT_GE(point_grid_coord_3[i * 3 + 1], 0);
    EXPECT_LT(point_grid_coord_3[i * 3 + 0], static_cast<std::int32_t>(config.det_grid_x_size_));
    EXPECT_LT(point_grid_coord_3[i * 3 + 1], static_cast<std::int32_t>(config.det_grid_y_size_));
  }
}

// Hand-crafted voxel layouts against the CPU reference: each case pins one boundary of the
// scan-based derivation, and the levels are small enough to verify by hand (stride-2 pooling
// merges voxels whose grid coordinates match after one right-shift per stage).
TEST_F(SerializedPoolingMetadataTest, MatchesCpuReferenceForEdgeCaseClouds)
{
  const auto config = make_test_config();
  constexpr std::size_t kNumOrders = 2;
  const auto stage_count = config.pooling_strides_.size();

  struct EdgeCase
  {
    std::string name;
    // Unique (x, y, z) triplets in any order (the pipeline never emits duplicate voxels); sorted
    // by order-0 code before upload.
    std::vector<std::int32_t> grid_coord;
    // Hand-verified voxel count per level: input, then one entry per pooling stage.
    std::vector<std::int64_t> expected_stage_counts;
    // Whether every level of two or more voxels must satisfy expect_orders_diverge.
    bool orders_diverge;
  };

  // A 4x4x2 box fills the level to exactly max_num_voxels (asserted below), leaving no padded
  // entries, and pools into four runs of eight voxels each.
  ASSERT_EQ(config.max_num_voxels_, 32);
  std::vector<std::int32_t> full_capacity_box;
  for (std::int32_t x = 0; x < 4; ++x) {
    for (std::int32_t y = 0; y < 4; ++y) {
      for (std::int32_t z = 0; z < 2; ++z) {
        full_capacity_box.insert(full_capacity_box.end(), {x, y, z});
      }
    }
  }

  const std::vector<EdgeCase> cases = {
    // The input-level sorts and every run are skipped; all levels stay empty.
    {"empty input", {}, {0, 0, 0}, false},
    // One run of length 1 per level; all metadata is the identity mapping.
    {"single voxel", {5, 3, 1}, {1, 1, 1}, false},
    // A full 2x2x2 block: the whole level is a single run and collapses to one voxel.
    {"one full parent",
     {0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1},
     {8, 1, 1},
     false},
    // Voxels spaced four cells apart never share a parent: every run has length 1 and no stage
    // merges anything.
    {"no merging", {0, 0, 0, 4, 0, 0, 8, 0, 0, 12, 0, 0}, {4, 4, 4}, false},
    // Parent runs of length 3, 1 and 2 in level order: multi-voxel runs at both ends of the
    // level with a single-voxel run in between, then everything merges into one voxel.
    {"mixed run lengths", {0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 2, 0, 2, 0, 0, 3, 1, 1}, {6, 3, 1}, true},
    {"full capacity", full_capacity_box, {32, 4, 1}, true},
  };

  PreprocessCuda preprocess(config, stream_);
  auto grid_coord_d = makeDeviceBuffer<std::int32_t>(config.max_num_voxels_ * 3);
  auto serialized_code_d = makeDeviceBuffer<std::int64_t>(config.max_num_voxels_ * kNumOrders);
  auto stage_counts_d = makeDeviceBuffer<std::int64_t>(stage_count + 1);
  std::vector<DeviceStage> device_stages;
  std::vector<SerializedPoolingDeviceStageView> stage_views;
  for (std::size_t stage = 0; stage < stage_count; ++stage) {
    device_stages.emplace_back(config.max_num_voxels_, kNumOrders);
  }
  for (auto & stage : device_stages) {
    stage_views.push_back(
      SerializedPoolingDeviceStageView{
        stage.indices.get(), stage.indptr.get(), stage.head_indices.get(), stage.cluster.get(),
        stage.grid_coord.get(), stage.serialized_code.get(), stage.serialized_order.get(),
        stage.serialized_inverse.get()});
  }

  for (const auto & edge_case : cases) {
    const auto grid_coord =
      sort_grid_coord_by_order0(edge_case.grid_coord, config.serialization_depth_);
    const auto serialized_code = make_serialized_code(grid_coord, config.serialization_depth_);
    const auto num_voxels = static_cast<std::int64_t>(grid_coord.size() / 3);

    copyToDevice(grid_coord_d.get(), grid_coord);
    copyToDevice(serialized_code_d.get(), serialized_code);
    preprocess.generateSerializedPoolingMetadata(
      grid_coord_d.get(), serialized_code_d.get(), num_voxels, stage_views, stage_counts_d.get());
    ASSERT_EQ(cudaStreamSynchronize(stream_), cudaSuccess);

    std::vector<CpuStage> references;
    references.push_back(
      make_stage_reference(grid_coord, serialized_code, kNumOrders, config.pooling_strides_[0]));
    for (std::size_t stage = 1; stage < stage_count; ++stage) {
      references.push_back(make_stage_reference(
        references[stage - 1].grid_coord, references[stage - 1].serialized_code, kNumOrders,
        config.pooling_strides_[stage]));
    }

    const auto stage_counts = copyToHost(stage_counts_d.get(), stage_count + 1);
    ASSERT_EQ(stage_counts, edge_case.expected_stage_counts) << edge_case.name;

    for (std::size_t stage_index = 0; stage_index < references.size(); ++stage_index) {
      const auto & expected = references[stage_index];
      const auto & actual = device_stages[stage_index];
      const auto in_count = static_cast<std::size_t>(stage_counts[stage_index]);
      const auto out_count = static_cast<std::size_t>(stage_counts[stage_index + 1]);
      const auto prefix = edge_case.name + " stage " + std::to_string(stage_index) + " ";

      ASSERT_EQ(out_count, expected.head_indices.size()) << prefix + "out_count";
      if (edge_case.orders_diverge && out_count >= 2) {
        expect_orders_diverge(
          expected.serialized_order, out_count, kNumOrders, prefix + "reference serialized_order");
      }
      expect_equal(
        copyToHost(actual.indices.get(), in_count), expected.indices, prefix + "indices");
      expect_equal(
        copyToHost(actual.indptr.get(), out_count + 1), expected.indptr, prefix + "indptr");
      expect_equal(
        copyToHost(actual.head_indices.get(), out_count), expected.head_indices,
        prefix + "head_indices");
      expect_equal(
        copyToHost(actual.cluster.get(), in_count), expected.cluster, prefix + "cluster");
      expect_equal(
        copyToHost(actual.grid_coord.get(), out_count * 3), expected.grid_coord,
        prefix + "grid_coord");
      expect_equal(
        copyToHost(actual.serialized_code.get(), out_count * kNumOrders), expected.serialized_code,
        prefix + "serialized_code");
      expect_equal(
        copyToHost(actual.serialized_order.get(), out_count * kNumOrders),
        expected.serialized_order, prefix + "serialized_order");
      expect_equal(
        copyToHost(actual.serialized_inverse.get(), out_count * kNumOrders),
        expected.serialized_inverse, prefix + "serialized_inverse");
    }
  }
}

TEST_F(SerializedPoolingMetadataTest, MatchesCpuReferenceForOnnxFacingInputs)
{
  const auto config = make_test_config();
  constexpr std::size_t kNumOrders = 2;
  // Chosen so that "z" and "z-trans" rank the voxels differently at every level (enforced by
  // expect_orders_diverge below) and pooling merges voxels at every stage (10 -> 6 -> 4).
  const auto grid_coord = sort_grid_coord_by_order0(
    {3, 0, 2, 3, 1, 3, 0, 5, 2, 4, 2, 0, 5, 2, 1, 5, 3, 0, 4, 3, 3, 5, 4, 1, 4, 4, 2, 5, 4, 2},
    config.serialization_depth_);
  const auto serialized_code = make_serialized_code(grid_coord, config.serialization_depth_);
  const auto num_voxels = static_cast<std::int64_t>(grid_coord.size() / 3);

  PreprocessCuda preprocess(config, stream_);
  auto grid_coord_d = makeDeviceBuffer<std::int32_t>(grid_coord.size());
  auto serialized_code_d = makeDeviceBuffer<std::int64_t>(serialized_code.size());
  auto stage_counts_d = makeDeviceBuffer<std::int64_t>(config.pooling_strides_.size() + 1);
  std::vector<DeviceStage> device_stages;
  std::vector<SerializedPoolingDeviceStageView> stage_views;
  for (std::size_t stage = 0; stage < config.pooling_strides_.size(); ++stage) {
    device_stages.emplace_back(config.max_num_voxels_, kNumOrders);
  }
  for (auto & stage : device_stages) {
    stage_views.push_back(
      SerializedPoolingDeviceStageView{
        stage.indices.get(), stage.indptr.get(), stage.head_indices.get(), stage.cluster.get(),
        stage.grid_coord.get(), stage.serialized_code.get(), stage.serialized_order.get(),
        stage.serialized_inverse.get()});
  }

  copyToDevice(grid_coord_d.get(), grid_coord);
  copyToDevice(serialized_code_d.get(), serialized_code);

  preprocess.generateSerializedPoolingMetadata(
    grid_coord_d.get(), serialized_code_d.get(), num_voxels, stage_views, stage_counts_d.get());
  ASSERT_EQ(cudaStreamSynchronize(stream_), cudaSuccess);

  std::vector<CpuStage> references;
  references.push_back(
    make_stage_reference(grid_coord, serialized_code, kNumOrders, config.pooling_strides_[0]));
  references.push_back(make_stage_reference(
    references[0].grid_coord, references[0].serialized_code, kNumOrders,
    config.pooling_strides_[1]));

  const auto stage_counts = copyToHost(stage_counts_d.get(), config.pooling_strides_.size() + 1);
  ASSERT_EQ(stage_counts[0], num_voxels);
  ASSERT_EQ(stage_counts[1], static_cast<std::int64_t>(references[0].head_indices.size()));
  ASSERT_EQ(stage_counts[2], static_cast<std::int64_t>(references[1].head_indices.size()));

  for (std::size_t stage_index = 0; stage_index < references.size(); ++stage_index) {
    const auto & expected = references[stage_index];
    const auto & actual = device_stages[stage_index];
    const auto in_count = static_cast<std::size_t>(stage_counts[stage_index]);
    const auto out_count = static_cast<std::size_t>(stage_counts[stage_index + 1]);
    const auto prefix = "stage " + std::to_string(stage_index) + " ";

    const auto indices = copyToHost(actual.indices.get(), in_count);
    const auto indptr = copyToHost(actual.indptr.get(), out_count + 1);
    const auto head_indices = copyToHost(actual.head_indices.get(), out_count);
    const auto cluster = copyToHost(actual.cluster.get(), in_count);
    expect_all_in_range(indices, static_cast<std::int64_t>(in_count), prefix + "indices");
    expect_all_in_range(head_indices, static_cast<std::int64_t>(in_count), prefix + "head_indices");
    expect_all_in_range(cluster, static_cast<std::int64_t>(out_count), prefix + "cluster");
    EXPECT_TRUE(std::is_sorted(indptr.begin(), indptr.end())) << prefix + "indptr";
    expect_equal(indices, expected.indices, prefix + "indices");
    expect_equal(indptr, expected.indptr, prefix + "indptr");
    expect_equal(head_indices, expected.head_indices, prefix + "head_indices");
    expect_equal(cluster, expected.cluster, prefix + "cluster");
    expect_equal(
      copyToHost(actual.grid_coord.get(), out_count * 3), expected.grid_coord,
      prefix + "grid_coord");
    expect_equal(
      copyToHost(actual.serialized_code.get(), out_count * kNumOrders), expected.serialized_code,
      prefix + "serialized_code");
    const auto serialized_order = copyToHost(actual.serialized_order.get(), out_count * kNumOrders);
    const auto serialized_inverse =
      copyToHost(actual.serialized_inverse.get(), out_count * kNumOrders);
    expect_orders_diverge(
      expected.serialized_order, out_count, kNumOrders, prefix + "reference serialized_order");
    expect_equal(serialized_order, expected.serialized_order, prefix + "serialized_order");
    expect_equal(serialized_inverse, expected.serialized_inverse, prefix + "serialized_inverse");
    for (std::size_t order = 0; order < kNumOrders; ++order) {
      const auto begin = static_cast<std::ptrdiff_t>(order * out_count);
      const auto end = static_cast<std::ptrdiff_t>((order + 1) * out_count);
      expect_permutation(
        std::vector<std::int64_t>(serialized_order.begin() + begin, serialized_order.begin() + end),
        prefix + "serialized_order " + std::to_string(order));
      expect_permutation(
        std::vector<std::int64_t>(
          serialized_inverse.begin() + begin, serialized_inverse.begin() + end),
        prefix + "serialized_inverse " + std::to_string(order));
    }
  }
}

}  // namespace test
}  // namespace autoware::ptv3
