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

#include "autoware/bevfusion/preprocess/sparse_rulebook_precompute.hpp"

#include <autoware/cuda_utils/cuda_check_error.hpp>

// spconv (same headers the GetIndicePairsImplicitGemm plugin uses). cSpell:ignore spconvlib
#include <spconvlib/spconv/csrc/sparse/all/SpconvOps.h>
#include <spconvlib/spconv/csrc/sparse/alloc/StaticAllocator.h>

#include <functional>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace autoware::bevfusion
{

namespace
{
using SpconvOps = spconvlib::spconv::csrc::sparse::all::SpconvOps;
using StaticAllocator = spconvlib::spconv::csrc::sparse::alloc::StaticAllocator;

// kMaskImplicitGemm (matches the exported GetIndicePairsImplicitGemm ``algo`` attribute = 1).
constexpr int kAlgo = 1;
// mask_count: 1 for kMaskImplicitGemm (split-mask would be 2). pair_mask / mask_argsort are [1, N].
constexpr int kMaskCount = 1;
// do_sort: pair-mask argsort. Validated equivalent with sort on; output numerics are independent of
// the argsort ordering (it only schedules the masked gemm). Kept on to match the Python reference.
constexpr bool kDoSort = true;
constexpr std::size_t kThrustTempBytes = 8U * 1024U * 1024U;

// coors_d[num_in, cols] -> out[num_in, 4] = [batch=0, x, y, z].
// flip=true : input spatial cols are [z, y, x] (legacy graph-input contract) -> reversed to
// [x,y,z]. cols == 4 : assume input already [batch, x, y, z] and copy verbatim.
__global__ void buildBatchedCoordsKernel(
  const std::int32_t * __restrict__ in, std::int32_t * __restrict__ out, int num_in, int cols,
  bool flip)
{
  const int r = blockIdx.x * blockDim.x + threadIdx.x;
  if (r >= num_in) {
    return;
  }
  const std::int32_t * src = in + static_cast<std::int64_t>(r) * cols;
  std::int32_t * dst = out + static_cast<std::int64_t>(r) * 4;
  if (cols == 4) {
    dst[0] = src[0];
    dst[1] = src[1];
    dst[2] = src[2];
    dst[3] = src[3];
    return;
  }
  // cols == 3
  dst[0] = 0;  // batch
  if (flip) {
    dst[1] = src[2];  // x
    dst[2] = src[1];  // y
    dst[3] = src[0];  // z
  } else {
    dst[1] = src[0];
    dst[2] = src[1];
    dst[3] = src[2];
  }
}

bool useInt64HashK(
  const std::vector<int> & spatial, const std::vector<int> & ksize, const std::vector<int> & stride,
  const std::vector<int> & padding, const std::vector<int> & dilation)
{
  auto out_dims = SpconvOps::get_conv_output_size(spatial, ksize, stride, padding, dilation);
  std::int64_t vol = std::accumulate(
    out_dims.begin(), out_dims.end(), static_cast<std::int64_t>(1),
    std::multiplies<std::int64_t>());
  return vol >= static_cast<std::int64_t>(std::numeric_limits<int>::max());
}
}  // namespace

std::vector<SparseDownsampleStage> default_bevfusion_downsample_stages()
{
  // Matches the ONNX node attributes of the 4 stride>1 GetIndicePairsImplicitGemm nodes
  // (BEVFusion-L, sparse_shape [1440,1440,41]). Spatial cascade 1440->720->360->180.
  std::vector<SparseDownsampleStage> stages = {
    {"/pts_middle_encoder/encoder_layer1/encoder_layer1.2/encoder_layer1.2.0/"
     "GetIndicePairsImplicitGemm",
     {3, 3, 3},
     {2, 2, 2},
     {1, 1, 1},
     {1, 1, 1},
     {1440, 1440, 41},
     0},
    {"/pts_middle_encoder/encoder_layer2/encoder_layer2.2/encoder_layer2.2.0/"
     "GetIndicePairsImplicitGemm",
     {3, 3, 3},
     {2, 2, 2},
     {1, 1, 1},
     {1, 1, 1},
     {720, 720, 21},
     0},
    {"/pts_middle_encoder/encoder_layer3/encoder_layer3.2/encoder_layer3.2.0/"
     "GetIndicePairsImplicitGemm",
     {3, 3, 3},
     {2, 2, 2},
     {1, 1, 0},
     {1, 1, 1},
     {360, 360, 11},
     0},
    {"/pts_middle_encoder/conv_out/conv_out.0/GetIndicePairsImplicitGemm",
     {1, 1, 3},
     {1, 1, 2},
     {0, 0, 0},
     {1, 1, 1},
     {180, 180, 5},
     0},
  };
  for (auto & s : stages) {
    s.kernel_volume = std::accumulate(s.ksize.begin(), s.ksize.end(), 1, std::multiplies<int>());
  }
  return stages;
}

SparseRulebookPrecompute::SparseRulebookPrecompute(
  int out_indices_num_limit, std::vector<SparseDownsampleStage> stages, cudaStream_t stream)
: out_indices_num_limit_(out_indices_num_limit), stages_(std::move(stages)), stream_(stream)
{
  for (auto & s : stages_) {
    if (s.kernel_volume == 0) {
      s.kernel_volume = std::accumulate(s.ksize.begin(), s.ksize.end(), 1, std::multiplies<int>());
    }
  }
  stage_counts_.assign(stages_.size(), 0);
  allocateStageBuffers();
}

void SparseRulebookPrecompute::allocateStageBuffers()
{
  const int N = out_indices_num_limit_;

  int max_kv = 0;
  bool any_int64_hash = false;
  for (const auto & s : stages_) {
    max_kv = std::max(max_kv, s.kernel_volume);
    any_int64_hash =
      any_int64_hash || useInt64HashK(s.spatial_shape, s.ksize, s.stride, s.padding, s.dilation);

    out_indices_d_.emplace_back(
      autoware::cuda_utils::make_unique<std::int32_t[]>(static_cast<std::size_t>(N) * 4));
    pair_fwd_d_.emplace_back(
      autoware::cuda_utils::make_unique<std::int32_t[]>(
        static_cast<std::size_t>(s.kernel_volume) * N));
    pair_mask_d_.emplace_back(
      autoware::cuda_utils::make_unique<std::int32_t[]>(static_cast<std::size_t>(kMaskCount) * N));
    mask_argsort_d_.emplace_back(
      autoware::cuda_utils::make_unique<std::int32_t[]>(static_cast<std::size_t>(kMaskCount) * N));
  }

  coords_xyzb_d_ =
    autoware::cuda_utils::make_unique<std::int32_t[]>(static_cast<std::size_t>(N) * 4);

  // Workspace layout mirrors get_indices_pairs_implicit_gemm_plugin.cpp (non-subm):
  //   [ spconv index-gen workspace ]
  //   [ indices_kernel_num : KV int32 ]
  //   [ pair_bwd           : KV * N int32 ]
  //   [ pair_mask_bwd      : mask_count * N int32 ]
  //   [ mask_argsort_bwd   : mask_count * N int32 ]
  //   [ thrust_tmp         : kThrustTempBytes ]
  // Sized for the largest stage (max kernel_volume) and reused across stages.
  const bool use_direct_table = true;  // non-subm
  std::size_t spconv_ws = static_cast<std::size_t>(SpconvOps::get_indice_gen_workspace_size(
    max_kv, N, N, N, /*is_subm=*/false, any_int64_hash, use_direct_table));
  std::size_t sz = spconv_ws;
  sz += static_cast<std::size_t>(max_kv) * sizeof(std::int32_t);          // indices_kernel_num
  sz += static_cast<std::size_t>(max_kv) * N * sizeof(std::int32_t);      // pair_bwd
  sz += static_cast<std::size_t>(kMaskCount) * N * sizeof(std::int32_t);  // pair_mask_bwd
  sz += static_cast<std::size_t>(kMaskCount) * N * sizeof(std::int32_t);  // mask_argsort_bwd
  sz += kThrustTempBytes;

  spconv_workspace_size_ = sz;
  spconv_workspace_d_ = autoware::cuda_utils::make_unique<std::uint8_t[]>(sz);
}

int SparseRulebookPrecompute::computeStage(int i, const std::int32_t * coords_in_d, int num_in)
{
  const SparseDownsampleStage & s = stages_[i];
  const int N = out_indices_num_limit_;
  const int kv = s.kernel_volume;

  std::vector<int> ksize = s.ksize, stride = s.stride, padding = s.padding, dilation = s.dilation;
  std::vector<int> input_dims = s.spatial_shape;
  const bool use_direct_table = true;
  const bool use_int64_hash_k = useInt64HashK(s.spatial_shape, ksize, stride, padding, dilation);

  // Carve the workspace exactly like the plugin.
  std::uint8_t * ws = spconv_workspace_d_.get();
  std::size_t spconv_ws_size = static_cast<std::size_t>(SpconvOps::get_indice_gen_workspace_size(
    kv, N, N, N, /*is_subm=*/false, use_int64_hash_k, use_direct_table));

  auto ws_tensors = SpconvOps::get_indice_gen_tensors_from_workspace(
    ws, kv, N, N, /*max_act_out_theory=*/N, /*is_subm=*/false, use_int64_hash_k, use_direct_table);

  std::uint8_t * indice_num_ptr = ws + spconv_ws_size;
  tv::Tensor indices_kernel_num = tv::from_blob(indice_num_ptr, {kv}, tv::int32, 0);
  CHECK_CUDA_ERROR(cudaMemsetAsync(
    indice_num_ptr, 0, static_cast<std::size_t>(kv) * sizeof(std::int32_t), stream_));

  std::uint8_t * extra = indice_num_ptr + static_cast<std::size_t>(kv) * sizeof(std::int32_t);
  tv::Tensor pair_bwd = tv::from_blob(extra, {kv, N}, tv::int32, 0);
  extra += static_cast<std::size_t>(kv) * N * sizeof(std::int32_t);
  tv::Tensor pair_mask_bwd = tv::from_blob(extra, {kMaskCount, N}, tv::int32, 0);
  extra += static_cast<std::size_t>(kMaskCount) * N * sizeof(std::int32_t);
  tv::Tensor mask_argsort_bwd = tv::from_blob(extra, {kMaskCount, N}, tv::int32, 0);
  extra += static_cast<std::size_t>(kMaskCount) * N * sizeof(std::int32_t);
  tv::Tensor thrust_tmp =
    tv::from_blob(extra, {static_cast<std::int64_t>(kThrustTempBytes)}, tv::uint8, 0);

  // Outputs go to our stable per-stage buffers (= the engine input tensors).
  tv::Tensor out_indices = tv::from_blob(out_indices_d_[i].get(), {N, 4}, tv::int32, 0);
  tv::Tensor pair_fwd = tv::from_blob(pair_fwd_d_[i].get(), {kv, N}, tv::int32, 0);
  tv::Tensor pair_mask_fwd = tv::from_blob(pair_mask_d_[i].get(), {kMaskCount, N}, tv::int32, 0);
  tv::Tensor mask_argsort_fwd =
    tv::from_blob(mask_argsort_d_[i].get(), {kMaskCount, N}, tv::int32, 0);

  tv::Tensor input_indices =
    tv::from_blob(const_cast<std::int32_t *>(coords_in_d), {num_in, 4}, tv::int32, 0);

  ws_tensors.emplace(SPCONV_ALLOC_PAIR_FWD, pair_fwd);
  ws_tensors.emplace(SPCONV_ALLOC_PAIR_BWD, pair_bwd);
  ws_tensors.emplace(SPCONV_ALLOC_PAIR_MASK, pair_mask_fwd);
  ws_tensors.emplace(SPCONV_ALLOC_PAIR_MASK_BWD, pair_mask_bwd);
  ws_tensors.emplace(SPCONV_ALLOC_MASK_ARG_SORT, mask_argsort_fwd);
  ws_tensors.emplace(SPCONV_ALLOC_MASK_ARG_SORT_BWD, mask_argsort_bwd);
  ws_tensors.emplace(SPCONV_ALLOC_OUT_INDICES, out_indices);
  ws_tensors.emplace(SPCONV_ALLOC_INDICE_NUM_PER_LOC, indices_kernel_num);  // cSpell:ignore INDICE

  StaticAllocator alloc(ws_tensors);
  alloc.thrust_tmp_tensor_ = thrust_tmp;

  // cSpell:ignore indice
  auto pair_res = SpconvOps::get_indice_pairs_implicit_gemm(
    alloc, input_indices, /*batch_size=*/1, input_dims, kAlgo, ksize, stride, padding, dilation,
    {0, 0, 0}, /*subm=*/false, /*transpose=*/false, /*is_train=*/false,
    reinterpret_cast<std::uintptr_t>(stream_), N, tv::CUDAKernelTimer(false), use_direct_table,
    kDoSort);

  return std::get<1>(pair_res);  // num_act_out (host int; spconv performs the single D2H here)
}

void SparseRulebookPrecompute::compute(
  const std::int32_t * coors_d, int num_in, int coors_cols, bool flip_zyx_to_xyz)
{
  if (num_in <= 0) {
    throw std::runtime_error("SparseRulebookPrecompute: num_in must be > 0");
  }

  // Build [batch, x, y, z] int32 coords for the first down-sample stage.
  const int block = 256;
  const int grid = (num_in + block - 1) / block;
  buildBatchedCoordsKernel<<<grid, block, 0, stream_>>>(
    coors_d, coords_xyzb_d_.get(), num_in, coors_cols, flip_zyx_to_xyz);
  CHECK_CUDA_ERROR(cudaGetLastError());

  // Cascade: each down-sample's out_indices feed the next (submanifold layers in between do not
  // change coordinates, so they are not part of this cascade — they stay in the TRT graph).
  const std::int32_t * cur_coords = coords_xyzb_d_.get();
  int cur_num = num_in;
  for (int i = 0; i < static_cast<int>(stages_.size()); ++i) {
    int n_out = computeStage(i, cur_coords, cur_num);
    stage_counts_[i] = n_out;
    cur_coords = out_indices_d_[i].get();  // [n_out, 4]
    cur_num = n_out;
  }
}

}  // namespace autoware::bevfusion
