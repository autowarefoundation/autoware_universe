// Copyright 2025 TIER IV, Inc.
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

/*
 * SPDX-FileCopyrightText: Copyright (c) 1993-2025 NVIDIA CORPORATION & AFFILIATES. All rights
 *reserved. SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 **************************************************************************
 * Modified from Deformable DETR
 * Copyright (c) 2020 SenseTime. All Rights Reserved.
 * Licensed under the Apache License, Version 2.0 [see LICENSE for details]
 * https://github.com/fundamentalvision/Deformable-DETR/blob/main/LICENSE
 **************************************************************************
 * Modified from DCN (https://github.com/msracver/Deformable-ConvNets)
 * Copyright (c) 2018 Microsoft
 **************************************************************************
 */

// Copyright (c) OpenMMLab. All rights reserved
#ifndef AUTOWARE__MULTI_SCALE_DEFORM_ATTN_OPS__MS_DEFORM_ATTN_KERNEL_HPP_
#define AUTOWARE__MULTI_SCALE_DEFORM_ATTN_OPS__MS_DEFORM_ATTN_KERNEL_HPP_
#include <cublas_v2.h>
#include <cuda_runtime.h>

template <typename scalar_t>
int32_t ms_deform_attn_cuda_forward(
  const scalar_t * value, const int32_t * spatialShapes, const int32_t * levelStartIndex,
  const scalar_t * samplingLoc, const scalar_t * attnWeight, scalar_t * output, int32_t batch,
  int32_t mSpatialSize, int32_t mNumHeads, int32_t mChannels, int32_t mNumLevels, int32_t mNumQuery,
  int32_t mNumPoint, cudaStream_t stream);

#endif  // AUTOWARE__MULTI_SCALE_DEFORM_ATTN_OPS__MS_DEFORM_ATTN_KERNEL_HPP_
