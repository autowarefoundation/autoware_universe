// Copyright 2025 (c) OpenMMLab. All rights reserved.
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
// Modified from
// https://github.com/open-mmlab/mmdetection3d/blob/main/projects/BEVFusion/bevfusion/ops/bev_pool/src/bev_pool_cuda.cu
// https://github.com/mit-han-lab/bevfusion/blob/main/mmdet3d/ops/bev_pool/src/bev_pool_cuda.cu

#ifndef AUTOWARE__BEV_OPS__BEV_POOL_CUDA_HPP_
#define AUTOWARE__BEV_OPS__BEV_POOL_CUDA_HPP_

#include <cuda_runtime_api.h>

void bev_pool(
  int d, int h, int w, int c, int n_intervals, const float * x, const int * geom_feats,
  const int * interval_starts, const int * interval_lengths, float * out, cudaStream_t stream);

#endif  // AUTOWARE__BEV_OPS__BEV_POOL_CUDA_HPP_
