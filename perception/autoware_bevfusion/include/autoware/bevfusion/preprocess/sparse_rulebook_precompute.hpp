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

#ifndef AUTOWARE__BEVFUSION__PREPROCESS__SPARSE_RULEBOOK_PRECOMPUTE_HPP_
#define AUTOWARE__BEVFUSION__PREPROCESS__SPARSE_RULEBOOK_PRECOMPUTE_HPP_

// Runtime precompute of the BEVFusion sparse-encoder down-sample rulebooks for the
// trainStation/DDS-stripped engine (see AWML deploy flag ``spconv_remove_trainstation`` and
// BEVFusion_spconv_DDS_optimization.md, Slice 2/2b).
//
// The optimized sparse ONNX has its 4 down-sample ``GetIndicePairsImplicitGemm`` nodes removed and
// their outputs exposed as graph inputs. Those rulebooks depend only on voxel geometry, so they are
// computed here from the voxel coordinates and bound to the engine before ``enqueueV3`` — removing
// the in-graph ``DeviceToShapeHostCopy`` syncs that produced the ``[trainStationN]`` segments.
//
// This mirrors, per down-sample stage, the non-subm path of the GetIndicePairsImplicitGemm plugin
// (autoware_tensorrt_plugins/src/get_indices_pairs_implicit_gemm_plugin.cpp::enqueue) and the
// validated Python reference (AWML deployment/.../pipelines/sparse_rulebook_precompute.py).

#include <autoware/cuda_utils/cuda_unique_ptr.hpp>

#include <cuda_runtime.h>

#include <cstdint>
#include <string>
#include <vector>

namespace autoware::bevfusion
{

using autoware::cuda_utils::CudaUniquePtr;

// One stride>1 (down-sampling) sparse convolution whose rulebook is precomputed.
struct SparseDownsampleStage
{
  // ONNX graph-input base name; the 4 bound tensors are ``<onnx_base>_output_{0,1,2,3}``
  // (out_indices, pair_fwd, pair_mask, mask_argsort).
  std::string onnx_base;
  std::vector<int> ksize;          // e.g. {3,3,3} ; conv_out {1,1,3}
  std::vector<int> stride;         // e.g. {2,2,2} ; conv_out {1,1,2}
  std::vector<int> padding;        // per-stage
  std::vector<int> dilation;       // {1,1,1}
  std::vector<int> spatial_shape;  // input spatial shape of this stage (x,y,z); 1440->720->360->180
  int kernel_volume{0};            // prod(ksize); filled by ctor if 0
};

// Default stage descriptors for the BEVFusion-L sparse encoder (sparse_shape [1440,1440,41]).
// Matches the down-sample layers in projects/BEVFusion/bevfusion/sparse_encoder.py and the ONNX
// node attributes used by the export-side transform.
std::vector<SparseDownsampleStage> default_bevfusion_downsample_stages();

class SparseRulebookPrecompute
{
public:
  // out_indices_num_limit must equal the plugin's upper bound (256000) used for the TRT profile
  // max.
  SparseRulebookPrecompute(
    int out_indices_num_limit, std::vector<SparseDownsampleStage> stages, cudaStream_t stream);

  // Compute all stage rulebooks from input voxel coordinates.
  //   coors_d      : device buffer of voxel coords, row-major [num_in, coors_cols].
  //   num_in       : number of input voxels.
  //   coors_cols   : 3 (graph-input [z,y,x], no batch) or 4 ([batch, x, y, z]).
  //   flip_zyx_to_xyz : if true, treat coors as [.. z,y,x] and reverse the spatial columns to
  //                     [x,y,z] before prepending batch (the legacy Autoware graph-input contract).
  // After this returns, stageCount(i) and the device pointers below are valid for binding.
  void compute(const std::int32_t * coors_d, int num_in, int coors_cols, bool flip_zyx_to_xyz);

  int numStages() const { return static_cast<int>(stages_.size()); }
  const SparseDownsampleStage & stage(int i) const { return stages_[i]; }
  int stageCount(int i) const { return stage_counts_[i]; }  // active out-voxels of stage i

  // Device pointers to the (stable, upper-bound-sized) rulebook buffers, valid after compute().
  std::int32_t * outIndices(int i) const { return out_indices_d_[i].get(); }    // [count, 4]
  std::int32_t * pairFwd(int i) const { return pair_fwd_d_[i].get(); }          // [KV, count]
  std::int32_t * pairMask(int i) const { return pair_mask_d_[i].get(); }        // [count, 1]
  std::int32_t * maskArgsort(int i) const { return mask_argsort_d_[i].get(); }  // [count]

private:
  void allocateStageBuffers();
  // Run one down-sample stage's rulebook generation (mirrors the plugin non-subm enqueue).
  // Returns num_act_out for the stage; out_indices for the next stage's input.
  int computeStage(int i, const std::int32_t * coords_in_d, int num_in);

  int out_indices_num_limit_;
  std::vector<SparseDownsampleStage> stages_;
  cudaStream_t stream_;

  std::vector<int> stage_counts_;

  // Per-stage output (engine-input) buffers, sized to out_indices_num_limit_.
  std::vector<CudaUniquePtr<std::int32_t[]>> out_indices_d_;   // [N,4]
  std::vector<CudaUniquePtr<std::int32_t[]>> pair_fwd_d_;      // [KV,N]
  std::vector<CudaUniquePtr<std::int32_t[]>> pair_mask_d_;     // [N,1] (mask_count==1)
  std::vector<CudaUniquePtr<std::int32_t[]>> mask_argsort_d_;  // [N]

  // Scratch: [batch,x,y,z] int32 coords fed to spconv (max num voxels).
  CudaUniquePtr<std::int32_t[]> coords_xyzb_d_{nullptr};
  // spconv index-generation workspace + indices_kernel_num + thrust temp (per the plugin layout).
  CudaUniquePtr<std::uint8_t[]> spconv_workspace_d_{nullptr};
  std::size_t spconv_workspace_size_{0};
};

}  // namespace autoware::bevfusion

#endif  // AUTOWARE__BEVFUSION__PREPROCESS__SPARSE_RULEBOOK_PRECOMPUTE_HPP_
