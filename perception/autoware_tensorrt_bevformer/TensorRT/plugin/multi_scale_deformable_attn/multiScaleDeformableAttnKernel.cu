// Copyright 2025 The Autoware Contributors
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
//
/*
 * This file includes portions of code directly from the BEVFormer TensorRT implementation
 * by Derry Lin, available at:
 *   https://github.com/DerryHub/BEVFormer_tensorrt
 *
 * The included code is used under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * The Autoware Contributors have reused this code as-is in 2025, with no modifications.
 * Original creation by Derry Lin on 2022/10/22.
 */

#include "cuda_helper.h"
#include "helper.h"
#include "multiScaleDeformableAttnKernel.h"

#include <cuda/std/limits>

#include <cuda_fp16.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

template <typename T>
__forceinline__ __device__ T hmax(const T & a, const T & b)
{
  return max(a, b);
}

#if __CUDA_ARCH__ >= 800
template <>
__forceinline__ __device__ __half hmax(const __half & a, const __half & b)
{
  return __hmax(a, b);
}
template <>
__forceinline__ __device__ __half2 hmax(const __half2 & a, const __half2 & b)
{
  return __hmax2(a, b);
}
#else
template <>
__forceinline__ __device__ __half hmax(const __half & a, const __half & b)
{
  return __hgt(a, b) ? a : b;
}
template <>
__forceinline__ __device__ __half2 hmax(const __half2 & a, const __half2 & b)
{
  return __hfma2(__hgt2(a, b), a, __hmul2(__hle2(a, b), b));
}
#endif

template <typename T>
__forceinline__ __device__ T sign_05(T x)
{
  if (x > 0) {
    return 0.5f;
  }
  return -0.5f;
}

template <typename T>
__forceinline__ __device__ int8_t T2int8(T a)
{
  a = a > 127 ? 127 : a;
  a = a < -128 ? -128 : a;
  return int8_t(a + sign_05<T>(a));
}

template <>
__forceinline__ __device__ int8_t T2int8(__half a)
{
  short temp = __half2short_rn(a);
  temp = temp > static_cast<short>(127) ? static_cast<short>(127) : temp;
  temp = temp < static_cast<short>(-128) ? static_cast<short>(-128) : temp;
  return static_cast<int8_t>(temp);
}

template <typename T>
__forceinline__ __device__ uint8_t T2uint8(T a)
{
  a = a > 255 ? 255 : a;
  a = a < 0 ? 0 : a;
  return uint8_t(a + 0.5);
}

template <>
__forceinline__ __device__ uint8_t T2uint8(__half a)
{
  unsigned short temp = __half2ushort_rn(a);
  temp = temp > static_cast<short>(255) ? static_cast<short>(255) : temp;
  return static_cast<uint8_t>(temp);
}

__forceinline__ __device__ int8_t half2int8(const __half & hval, const float & scale)
{
  __half ret = __hdiv(hval, __float2half(scale));
  return T2int8<__half>(ret);
}

__forceinline__ __device__ uint8_t half2uint8(const __half & hval, const float & scale)
{
  __half ret = __hdiv(hval, __float2half(scale));
  return T2uint8<__half>(ret);
}

__forceinline__ __device__ void qmulf(const int32_4 & a, int8_4 & c, const float & b)
{
  c.x = T2int8<float>(a.x * b);
  c.y = T2int8<float>(a.y * b);
  c.z = T2int8<float>(a.z * b);
  c.w = T2int8<float>(a.w * b);
}

__forceinline__ __device__ void qmulh(const int32_4 & a, int8_4 & c, const __half & b)
{
  c.x = T2int8<__half>(__int2half_rn(a.x) * b);
  c.y = T2int8<__half>(__int2half_rn(a.y) * b);
  c.z = T2int8<__half>(__int2half_rn(a.z) * b);
  c.w = T2int8<__half>(__int2half_rn(a.w) * b);
}

__forceinline__ __device__ void dp4a(const int32_t * a, const int32_t * b, int32_t & c)
{
#if __CUDA_ARCH__ >= 610
  asm("dp4a.s32.s32 %0, %1, %2, %3;" : "+r"(c) : "r"(*a), "r"(*b), "r"(c));
#else
  auto ap = (int8_4 *)a, bp = (int8_4 *)b;

  c += ap->x * bp->x;
  c += ap->y * bp->y;
  c += ap->z * bp->z;
  c += ap->w * bp->w;
#endif
}

__forceinline__ __device__ void dp4a(const int32_t * a, const uint32_t * b, int32_t & c)
{
#if __CUDA_ARCH__ >= 610
  asm("dp4a.s32.u32 %0, %1, %2, %3;" : "+r"(c) : "r"(*a), "r"(*b), "r"(c));
#else
  auto ap = (int8_4 *)a;
  auto bp = (uint8_4 *)b;

  c += ap->x * bp->x;
  c += ap->y * bp->y;
  c += ap->z * bp->z;
  c += ap->w * bp->w;
#endif
}

template <typename scalar_t>
__device__ scalar_t ms_deform_attn_im2col_bilinear(
  const scalar_t *& bottom_data, const int & height, const int & width, const int & nheads,
  const int & channels, const scalar_t & h, const scalar_t & w)
{
  const int h_low = floorf(h);
  const int w_low = floorf(w);
  const int h_high = h_low + 1;
  const int w_high = w_low + 1;

  const scalar_t lh = h - h_low;
  const scalar_t lw = w - w_low;
  const scalar_t hh = 1 - lh, hw = 1 - lw;

  const int h_low_ptr_offset = h_low * width;
  const int h_high_ptr_offset = h_low_ptr_offset + width;
  const int w_low_ptr_offset = w_low;
  const int w_high_ptr_offset = w_low_ptr_offset + 1;
  const int step = channels * nheads;

  scalar_t v1 = 0;
  if (h_low >= 0 && w_low >= 0) {
    const int ptr1 = (h_low_ptr_offset + w_low_ptr_offset) * step;
    v1 = bottom_data[ptr1];
  }
  scalar_t v2 = 0;
  if (h_low >= 0 && w_high <= width - 1) {
    const int ptr2 = (h_low_ptr_offset + w_high_ptr_offset) * step;
    v2 = bottom_data[ptr2];
  }
  scalar_t v3 = 0;
  if (h_high <= height - 1 && w_low >= 0) {
    const int ptr3 = (h_high_ptr_offset + w_low_ptr_offset) * step;
    v3 = bottom_data[ptr3];
  }
  scalar_t v4 = 0;
  if (h_high <= height - 1 && w_high <= width - 1) {
    const int ptr4 = (h_high_ptr_offset + w_high_ptr_offset) * step;
    v4 = bottom_data[ptr4];
  }

  const scalar_t w1 = hh * hw, w2 = hh * lw, w3 = lh * hw, w4 = lh * lw;

  const scalar_t val = (w1 * v1 + w2 * v2 + w3 * v3 + w4 * v4);
  return val;
}

template <>
__device__ __half ms_deform_attn_im2col_bilinear(
  const __half *& bottom_data, const int & height, const int & width, const int & nheads,
  const int & channels, const __half & h, const __half & w)
{
  const __half h_low = hfloor(h);
  const __half w_low = hfloor(w);
  const __half h_high = __hadd(h_low, __float2half(1.f));
  const __half w_high = __hadd(w_low, __float2half(1.f));

  const __half lh = __hsub(h, h_low);
  const __half lw = __hsub(w, w_low);
  const __half hh = __hsub(__float2half(1.f), lh), hw = __hsub(__float2half(1.f), lw);

  const int h_low_ptr_offset = __half2int_rn(h_low) * width;
  const int h_high_ptr_offset = h_low_ptr_offset + width;
  const int w_low_ptr_offset = __half2int_rn(w_low);
  const int w_high_ptr_offset = w_low_ptr_offset + 1;
  const int step = channels * nheads;

  __half v1 = 0;
  if (__hge(h_low, __float2half(0.f)) && __hge(w_low, __float2half(0.f))) {
    const int ptr1 = (h_low_ptr_offset + w_low_ptr_offset) * step;
    v1 = bottom_data[ptr1];
  }
  __half v2 = 0;
  if (
    __hge(h_low, __float2half(0.f)) && __hle(w_high, __float2half(static_cast<float>(width - 1)))) {
    const int ptr2 = (h_low_ptr_offset + w_high_ptr_offset) * step;
    v2 = bottom_data[ptr2];
  }
  __half v3 = 0;
  if (
    __hle(h_high, __float2half(static_cast<float>(height - 1))) &&
    __hge(w_low, __float2half(0.f))) {
    const int ptr3 = (h_high_ptr_offset + w_low_ptr_offset) * step;
    v3 = bottom_data[ptr3];
  }
  __half v4 = 0;
  if (
    __hle(h_high, __float2half(static_cast<float>(height - 1))) &&
    __hle(w_high, __float2half(static_cast<float>(width - 1)))) {
    const int ptr4 = (h_high_ptr_offset + w_high_ptr_offset) * step;
    v4 = bottom_data[ptr4];
  }
  const __half w1 = __hmul(hh, hw), w2 = __hmul(hh, lw), w3 = __hmul(lh, hw), w4 = __hmul(lh, lw);
  return __hadd(__hadd(__hmul(w1, v1), __hmul(w2, v2)), __hadd(__hmul(w3, v3), __hmul(w4, v4)));
}

__device__ __half2 ms_deform_attn_im2col_bilinear_h2(
  const __half2 *& bottom_data, const int & height, const int & width, const int & nheads,
  const int & channels, const __half2 & wh)
{
  const __half2 wh_low = h2floor(wh);
  const __half2 wh_high = __hadd2(wh_low, __float2half2_rn(1.f));

  const __half2 lwh = __hsub2(wh, wh_low);
  const __half2 hwh = __hsub2(__float2half2_rn(1.f), lwh);

  const int h_low_ptr_offset = __high2float(wh_low) * width;
  const int h_high_ptr_offset = h_low_ptr_offset + width;
  const int w_low_ptr_offset = __low2float(wh_low);
  const int w_high_ptr_offset = w_low_ptr_offset + 1;
  const int step = channels * nheads;

  __half2 v1 = __float2half2_rn(0.f);
  if (
    __hge(__high2half(wh_low), __float2half(0.f)) && __hge(__low2half(wh_low), __float2half(0.f))) {
    const int ptr1 = (h_low_ptr_offset + w_low_ptr_offset) * step;
    v1 = bottom_data[ptr1];
  }
  __half2 v2 = __float2half2_rn(0.f);
  if (
    __hge(__high2half(wh_low), __float2half(0.f)) &&
    __hle(__low2half(wh_high), __float2half(static_cast<float>(width - 1)))) {
    const int ptr2 = (h_low_ptr_offset + w_high_ptr_offset) * step;
    v2 = bottom_data[ptr2];
  }
  __half2 v3 = __float2half2_rn(0.f);
  if (
    __hle(__high2half(wh_high), __float2half(static_cast<float>(height - 1))) &&
    __hge(__low2half(wh_low), __float2half(0.f))) {
    const int ptr3 = (h_high_ptr_offset + w_low_ptr_offset) * step;
    v3 = bottom_data[ptr3];
  }
  __half2 v4 = __float2half2_rn(0.f);
  if (
    __hle(__high2half(wh_high), __float2half(static_cast<float>(height - 1))) &&
    __hle(__low2half(wh_high), __float2half(static_cast<float>(width - 1)))) {
    const int ptr4 = (h_high_ptr_offset + w_high_ptr_offset) * step;
    v4 = bottom_data[ptr4];
  }
  const __half2 w1 = __half2half2(__hmul(__high2half(hwh), __low2half(hwh))),
                w2 = __half2half2(__hmul(__high2half(hwh), __low2half(lwh))),
                w3 = __half2half2(__hmul(__high2half(lwh), __low2half(hwh))),
                w4 = __half2half2(__hmul(__high2half(lwh), __low2half(lwh)));
  return __hadd2(
    __hadd2(__hmul2(w1, v1), __hmul2(w2, v2)), __hadd2(__hmul2(w3, v3), __hmul2(w4, v4)));
}

template <typename scalar_t>
__device__ int8_4 ms_deform_attn_im2col_bilinear_int8(
  const int8_4 *& bottom_data, const int & height, const int & width, const int & nheads,
  const int & channels, const scalar_t & h, const scalar_t & w)
{
  const int h_low = floorf(h);
  const int w_low = floorf(w);
  const int h_high = h_low + 1;
  const int w_high = w_low + 1;

  const scalar_t lh = h - h_low;
  const scalar_t lw = w - w_low;
  const scalar_t hh = 1 - lh, hw = 1 - lw;

  const int h_low_ptr_offset = h_low * width;
  const int h_high_ptr_offset = h_low_ptr_offset + width;
  const int w_low_ptr_offset = w_low;
  const int w_high_ptr_offset = w_low_ptr_offset + 1;
  const int step = channels * nheads;

  const float scale_area = 1 / 127.f;
  int8_4 weight = {
    T2int8<float>((hh * hw) / scale_area),
    T2int8<float>((hh * lw) / scale_area),
    T2int8<float>((lh * hw) / scale_area),
    T2int8<float>((lh * lw) / scale_area),
  };
  int8_4 inps[4] = {0, 0, 0, 0}, output;
  int32_t output_temp;

  if (h_low >= 0 && w_low >= 0) {
    const int ptr1 = (h_low_ptr_offset + w_low_ptr_offset) * step;
    const int8_4 & inp = bottom_data[ptr1];
    inps[0].x = inp.x;
    inps[1].x = inp.y;
    inps[2].x = inp.z;
    inps[3].x = inp.w;
  }

  if (h_low >= 0 && w_high <= width - 1) {
    const int ptr2 = (h_low_ptr_offset + w_high_ptr_offset) * step;
    const int8_4 & inp = bottom_data[ptr2];
    inps[0].y = inp.x;
    inps[1].y = inp.y;
    inps[2].y = inp.z;
    inps[3].y = inp.w;
  }

  if (h_high <= height - 1 && w_low >= 0) {
    const int ptr3 = (h_high_ptr_offset + w_low_ptr_offset) * step;
    const int8_4 & inp = bottom_data[ptr3];
    inps[0].z = inp.x;
    inps[1].z = inp.y;
    inps[2].z = inp.z;
    inps[3].z = inp.w;
  }

  if (h_high <= height - 1 && w_high <= width - 1) {
    const int ptr4 = (h_high_ptr_offset + w_high_ptr_offset) * step;
    const int8_4 & inp = bottom_data[ptr4];
    inps[0].w = inp.x;
    inps[1].w = inp.y;
    inps[2].w = inp.z;
    inps[3].w = inp.w;
  }

  output_temp = 0;
  dp4a((const int32_t *)inps, (const int32_t *)&weight, output_temp);
  output.x = T2int8<float>(output_temp * scale_area);

  output_temp = 0;
  dp4a((const int32_t *)(inps + 1), (const int32_t *)&weight, output_temp);
  output.y = T2int8<float>(output_temp * scale_area);

  output_temp = 0;
  dp4a((const int32_t *)(inps + 2), (const int32_t *)&weight, output_temp);
  output.z = T2int8<float>(output_temp * scale_area);

  output_temp = 0;
  dp4a((const int32_t *)(inps + 3), (const int32_t *)&weight, output_temp);
  output.w = T2int8<float>(output_temp * scale_area);

  return output;
}

__device__ void ms_deform_attn_im2col_bilinear_int8_h2(
  const int8_4 *& bottom_data, const int & height, const int & width, const int & nheads,
  const int & channels, const __half2 & wh, int8_t * output)
{
  const __half2 wh_low = h2floor(wh);

  const __half2 lwh = __hsub2(wh, wh_low);
  const __half2 hwh = __hsub2(__float2half2_rn(1.f), lwh);

  const int step = channels * nheads;
  const int h_low_ptr_offset = __half2int_rn(wh_low.y) * width * step;
  const int h_high_ptr_offset = h_low_ptr_offset + width * step;
  const int w_low_ptr_offset = __half2int_rn(wh_low.x) * step;
  const int w_high_ptr_offset = w_low_ptr_offset + step;

  const __half scale_area = 255.f;
  uint8_4 weight = {
    __half2ushort_rn(__hmul(__hmul(__high2half(hwh), __low2half(hwh)), scale_area)),
    __half2ushort_rn(__hmul(__hmul(__high2half(hwh), __low2half(lwh)), scale_area)),
    __half2ushort_rn(__hmul(__hmul(__high2half(lwh), __low2half(hwh)), scale_area)),
    __half2ushort_rn(__hmul(__hmul(__high2half(lwh), __low2half(lwh)), scale_area)),
  };
  auto weight_ui32 = reinterpret_cast<uint32_t *>(&weight);
  int8_4 inps[4] = {0};
  int32_t output_temp;
  auto inps_i32 = reinterpret_cast<int32_t *>(inps);
  auto inp = reinterpret_cast<int8_4 *>(&output_temp);

  if (__hge(wh_low.y, __float2half(0.f)) && __hge(wh_low.x, __float2half(0.f))) {
    *inp = bottom_data[h_low_ptr_offset + w_low_ptr_offset];
    inps[0].x = inp->x;
    inps[1].x = inp->y;
    inps[2].x = inp->z;
    inps[3].x = inp->w;
  }

  if (__hge(wh_low.y, __float2half(0.f)) && __hle(wh_low.x, __int2half_rn(width - 2))) {
    *inp = bottom_data[h_low_ptr_offset + w_high_ptr_offset];
    inps[0].y = inp->x;
    inps[1].y = inp->y;
    inps[2].y = inp->z;
    inps[3].y = inp->w;
  }

  if (__hle(wh_low.y, __int2half_rn(height - 2)) && __hge(wh_low.x, __float2half(0.f))) {
    *inp = bottom_data[h_high_ptr_offset + w_low_ptr_offset];
    inps[0].z = inp->x;
    inps[1].z = inp->y;
    inps[2].z = inp->z;
    inps[3].z = inp->w;
  }

  if (__hle(wh_low.y, __int2half_rn(height - 2)) && __hle(wh_low.x, __int2half_rn(width - 2))) {
    *inp = bottom_data[h_high_ptr_offset + w_high_ptr_offset];
    inps[0].w = inp->x;
    inps[1].w = inp->y;
    inps[2].w = inp->z;
    inps[3].w = inp->w;
  }

  output_temp = 0;
  dp4a(inps_i32, weight_ui32, output_temp);
  output[0] = T2int8<__half>(__hdiv(__int2half_rn(output_temp), scale_area));

  output_temp = 0;
  dp4a(++inps_i32, weight_ui32, output_temp);
  output[4] = T2int8<__half>(__hdiv(__int2half_rn(output_temp), scale_area));

  output_temp = 0;
  dp4a(++inps_i32, weight_ui32, output_temp);
  output[8] = T2int8<__half>(__hdiv(__int2half_rn(output_temp), scale_area));

  output_temp = 0;
  dp4a(++inps_i32, weight_ui32, output_temp);
  output[12] = T2int8<__half>(__hdiv(__int2half_rn(output_temp), scale_area));
}

__device__ void ms_deform_attn_im2col_bilinear_int8_h2_(
  const int8_4 *& bottom_data, const int & height, const int & width, const int & nheads,
  const int & channels, const __half2 & w, const __half2 & h, const __half2 & condition,
  int8_t * output)
{
  const __half2 w_low = h2floor(w);
  const __half2 h_low = h2floor(h);

  const __half2 lw = __hsub2(w, w_low);
  const __half2 hw = __hsub2(__float2half2_rn(1.f), lw);
  const __half2 lh = __hsub2(h, h_low);
  const __half2 hh = __hsub2(__float2half2_rn(1.f), lh);

  const __half2 scale_area = __float2half2_rn(255.f);
  const __half2 w1 = __hmul2(__hmul2(hh, hw), scale_area);
  const __half2 w2 = __hmul2(__hmul2(hh, lw), scale_area);
  const __half2 w3 = __hmul2(__hmul2(lh, hw), scale_area);
  const __half2 w4 = __hmul2(__hmul2(lh, lw), scale_area);

  const int step = channels * nheads;
  int32_t output_temp;
  if (condition.x) {
    const int h_low_ptr_offset = __half2int_rn(h_low.x) * width * step;
    const int h_high_ptr_offset = h_low_ptr_offset + width * step;
    const int w_low_ptr_offset = __half2int_rn(w_low.x) * step;
    const int w_high_ptr_offset = w_low_ptr_offset + step;

    uint8_4 weight = {
      __half2ushort_rn(w1.x),
      __half2ushort_rn(w2.x),
      __half2ushort_rn(w3.x),
      __half2ushort_rn(w4.x),
    };

    auto weight_ui32 = reinterpret_cast<uint32_t *>(&weight);
    int8_4 inps[4] = {0};
    auto inps_i32 = reinterpret_cast<int32_t *>(inps);
    auto inp = reinterpret_cast<int8_4 *>(&output_temp);

    if (__hge(h_low.x, __float2half(0.f)) && __hge(w_low.x, __float2half(0.f))) {
      *inp = bottom_data[h_low_ptr_offset + w_low_ptr_offset];
      inps[0].x = inp->x;
      inps[1].x = inp->y;
      inps[2].x = inp->z;
      inps[3].x = inp->w;
    }
    if (__hge(h_low.x, __float2half(0.f)) && __hle(w_low.x, __int2half_rn(width - 2))) {
      *inp = bottom_data[h_low_ptr_offset + w_high_ptr_offset];
      inps[0].y = inp->x;
      inps[1].y = inp->y;
      inps[2].y = inp->z;
      inps[3].y = inp->w;
    }
    if (__hle(h_low.x, __int2half_rn(height - 2)) && __hge(w_low.x, __float2half(0.f))) {
      *inp = bottom_data[h_high_ptr_offset + w_low_ptr_offset];
      inps[0].z = inp->x;
      inps[1].z = inp->y;
      inps[2].z = inp->z;
      inps[3].z = inp->w;
    }
    if (__hle(h_low.x, __int2half_rn(height - 2)) && __hle(w_low.x, __int2half_rn(width - 2))) {
      *inp = bottom_data[h_high_ptr_offset + w_high_ptr_offset];
      inps[0].w = inp->x;
      inps[1].w = inp->y;
      inps[2].w = inp->z;
      inps[3].w = inp->w;
    }

    output_temp = 0;
    dp4a(inps_i32, weight_ui32, output_temp);
    output[0] = T2int8<__half>(__hdiv(__int2half_rn(output_temp), scale_area.x));

    output_temp = 0;
    dp4a(++inps_i32, weight_ui32, output_temp);
    output[4] = T2int8<__half>(__hdiv(__int2half_rn(output_temp), scale_area.x));

    output_temp = 0;
    dp4a(++inps_i32, weight_ui32, output_temp);
    output[8] = T2int8<__half>(__hdiv(__int2half_rn(output_temp), scale_area.x));

    output_temp = 0;
    dp4a(++inps_i32, weight_ui32, output_temp);
    output[12] = T2int8<__half>(__hdiv(__int2half_rn(output_temp), scale_area.x));
  }
  if (condition.y) {
    const int h_low_ptr_offset = __half2int_rn(h_low.y) * width * step;
    const int h_high_ptr_offset = h_low_ptr_offset + width * step;
    const int w_low_ptr_offset = __half2int_rn(w_low.y) * step;
    const int w_high_ptr_offset = w_low_ptr_offset + step;

    uint8_4 weight = {
      __half2ushort_rn(w1.y),
      __half2ushort_rn(w2.y),
      __half2ushort_rn(w3.y),
      __half2ushort_rn(w4.y),
    };

    auto weight_ui32 = reinterpret_cast<uint32_t *>(&weight);
    int8_4 inps[4] = {0};
    auto inps_i32 = reinterpret_cast<int32_t *>(inps);
    auto inp = reinterpret_cast<int8_4 *>(&output_temp);

    if (__hge(h_low.y, __float2half(0.f)) && __hge(w_low.y, __float2half(0.f))) {
      *inp = bottom_data[h_low_ptr_offset + w_low_ptr_offset];
      inps[0].x = inp->x;
      inps[1].x = inp->y;
      inps[2].x = inp->z;
      inps[3].x = inp->w;
    }
    if (__hge(h_low.y, __float2half(0.f)) && __hle(w_low.y, __int2half_rn(width - 2))) {
      *inp = bottom_data[h_low_ptr_offset + w_high_ptr_offset];
      inps[0].y = inp->x;
      inps[1].y = inp->y;
      inps[2].y = inp->z;
      inps[3].y = inp->w;
    }
    if (__hle(h_low.y, __int2half_rn(height - 2)) && __hge(w_low.y, __float2half(0.f))) {
      *inp = bottom_data[h_high_ptr_offset + w_low_ptr_offset];
      inps[0].z = inp->x;
      inps[1].z = inp->y;
      inps[2].z = inp->z;
      inps[3].z = inp->w;
    }
    if (__hle(h_low.y, __int2half_rn(height - 2)) && __hle(w_low.y, __int2half_rn(width - 2))) {
      *inp = bottom_data[h_high_ptr_offset + w_high_ptr_offset];
      inps[0].w = inp->x;
      inps[1].w = inp->y;
      inps[2].w = inp->z;
      inps[3].w = inp->w;
    }

    output_temp = 0;
    dp4a(inps_i32, weight_ui32, output_temp);
    output[1] = T2int8<__half>(__hdiv(__int2half_rn(output_temp), scale_area.y));

    output_temp = 0;
    dp4a(++inps_i32, weight_ui32, output_temp);
    output[5] = T2int8<__half>(__hdiv(__int2half_rn(output_temp), scale_area.y));

    output_temp = 0;
    dp4a(++inps_i32, weight_ui32, output_temp);
    output[9] = T2int8<__half>(__hdiv(__int2half_rn(output_temp), scale_area.y));

    output_temp = 0;
    dp4a(++inps_i32, weight_ui32, output_temp);
    output[13] = T2int8<__half>(__hdiv(__int2half_rn(output_temp), scale_area.y));
  }
}

template <typename scalar_t>
__global__ void ms_deformable_im2col_gpu_kernel(
  const int n, const scalar_t * data_value, const int32_t * data_spatial_shapes,
  const scalar_t * data_reference_points, const scalar_t * data_sampling_offsets,
  const scalar_t * data_attn_weight, const int spatial_size, const int num_heads,
  const int channels, const int num_levels, const int num_query, const int num_point,
  const int points_per_group, scalar_t * data_col)
{
  CUDA_1D_KERNEL_LOOP(index, n)
  {
    const int temp = index;
    const int channel_index = index % channels;
    int idx = index / channels;
    const int head_index = idx % num_heads;
    idx /= num_heads;
    const int query_index = idx % num_query;
    const int batch_index = idx / num_query;

    const scalar_t * data_value_ptr =
      data_value + (batch_index * spatial_size * num_heads + head_index) * channels + channel_index;
    int data_weight_ptr =
      ((batch_index * num_query + query_index) * num_heads + head_index) * num_levels * num_point;
    int data_offset_w_ptr = data_weight_ptr << 1;
    int data_points_ptr = (batch_index * num_query + query_index) * points_per_group * 2;
    scalar_t * data_output_ptr = data_col + temp;
    scalar_t output = 0;

    const int all_points = num_levels * num_point;
    scalar_t max_weight = -cuda::std::numeric_limits<scalar_t>::infinity();
    scalar_t sum_weight = 0.f;
#pragma unroll
    for (int w_index = 0; w_index < all_points; ++w_index) {
      const scalar_t weight = data_attn_weight[data_weight_ptr + w_index];
      max_weight = max(max_weight, weight);
    }

    for (int level_index = 0; level_index < num_levels; ++level_index) {
      const int spatial_h_ptr = level_index << 1;
      const int spatial_h = data_spatial_shapes[spatial_h_ptr];
      const int spatial_w = data_spatial_shapes[spatial_h_ptr + 1];

      for (int point_index = 0; point_index < num_point; ++point_index) {
        const int point_index_per_group = point_index % points_per_group;
        const scalar_t reference_point_x =
          data_reference_points[data_points_ptr + point_index_per_group * 2];
        const scalar_t reference_point_y =
          data_reference_points[data_points_ptr + point_index_per_group * 2 + 1];
        const scalar_t loc_w =
          reference_point_x * spatial_w + data_sampling_offsets[data_offset_w_ptr];
        const scalar_t loc_h =
          reference_point_y * spatial_h + data_sampling_offsets[data_offset_w_ptr + 1];

        const scalar_t weight = exp(data_attn_weight[data_weight_ptr] - max_weight);
        sum_weight += weight;

        const scalar_t h_im = loc_h - 0.5f;
        const scalar_t w_im = loc_w - 0.5f;

        if (h_im > -1 && w_im > -1 && h_im < spatial_h && w_im < spatial_w) {
          scalar_t bilinear = ms_deform_attn_im2col_bilinear(
            data_value_ptr, spatial_h, spatial_w, num_heads, channels, h_im, w_im);
          output += bilinear * weight;
        }

        data_weight_ptr += 1;
        data_offset_w_ptr += 2;
      }
      data_value_ptr += spatial_h * spatial_w * channels * num_heads;
    }
    *data_output_ptr = output / sum_weight;
  }
}

template <>
__global__ void ms_deformable_im2col_gpu_kernel(
  const int n, const __half * data_value, const int32_t * data_spatial_shapes,
  const __half * data_reference_points, const __half * data_sampling_offsets,
  const __half * data_attn_weight, const int spatial_size, const int num_heads, const int channels,
  const int num_levels, const int num_query, const int num_point, const int points_per_group,
  __half * data_col)
{
  CUDA_1D_KERNEL_LOOP(index, n)
  {
    const int temp = index;
    const int channel_index = index % channels;
    index /= channels;
    const int head_index = index % num_heads;
    index /= num_heads;
    const int query_index = index % num_query;
    const int batch_index = index / num_query;

    const __half * data_value_ptr =
      data_value + (batch_index * spatial_size * num_heads + head_index) * channels + channel_index;
    int data_weight_ptr =
      ((batch_index * num_query + query_index) * num_heads + head_index) * num_levels * num_point;
    int data_offset_w_ptr = data_weight_ptr << 1;
    int data_points_ptr = (batch_index * num_query + query_index) * points_per_group * 2;
    __half * data_output_ptr = data_col + temp;
    __half output = 0.f;

    const int all_points = num_levels * num_point;
    __half max_weight = -cuda::std::numeric_limits<__half>::infinity();
    __half sum_weight = 0.f;
#pragma unroll
    for (int w_index = 0; w_index < all_points; ++w_index) {
      const __half weight = data_attn_weight[data_weight_ptr + w_index];
      max_weight = hmax(max_weight, weight);
    }

    for (int level_index = 0; level_index < num_levels; ++level_index) {
      const int spatial_h_ptr = level_index << 1;
      const int spatial_h = data_spatial_shapes[spatial_h_ptr];
      const int spatial_w = data_spatial_shapes[spatial_h_ptr + 1];

      for (int point_index = 0; point_index < num_point; ++point_index) {
        const int point_index_per_group = point_index % points_per_group;
        const __half reference_point_x =
          data_reference_points[data_points_ptr + point_index_per_group * 2];
        const __half reference_point_y =
          data_reference_points[data_points_ptr + point_index_per_group * 2 + 1];
        const __half loc_w = __hfma(
          reference_point_x, __int2half_rn(spatial_w), data_sampling_offsets[data_offset_w_ptr]);
        const __half loc_h = __hfma(
          reference_point_y, __int2half_rn(spatial_h),
          data_sampling_offsets[data_offset_w_ptr + 1]);

        const __half weight = hexp(data_attn_weight[data_weight_ptr] - max_weight);
        sum_weight += weight;

        const __half h_im = __hsub(loc_h, __float2half(0.5f));
        const __half w_im = __hsub(loc_w, __float2half(0.5f));

        if (
          __hgt(h_im, __float2half(-1.f)) && __hgt(w_im, __float2half(-1.f)) &&
          __hlt(h_im, __int2half_rn(spatial_h)) && __hlt(w_im, __int2half_rn(spatial_w))) {
          output += ms_deform_attn_im2col_bilinear(
                      data_value_ptr, spatial_h, spatial_w, num_heads, channels, h_im, w_im) *
                    weight;
        }

        data_weight_ptr += 1;
        data_offset_w_ptr += 2;
      }
      data_value_ptr += spatial_h * spatial_w * channels * num_heads;
    }
    *data_output_ptr = __hdiv(output, sum_weight);
  }
}

__global__ void ms_deformable_im2col_gpu_kernel_h2(
  const int n, const __half2 * data_value, const int32_t * data_spatial_shapes,
  const __half2 * data_reference_points, const __half2 * data_sampling_offsets,
  const __half * data_attn_weight, const int spatial_size, const int num_heads, const int channels,
  const int num_levels, const int num_query, const int num_point, const int points_per_group,
  __half2 * data_col)
{
  CUDA_1D_KERNEL_LOOP(index, n)
  {
    const int temp = index;
    const int channel_index = index % channels;
    index /= channels;
    const int head_index = index % num_heads;
    index /= num_heads;
    const int query_index = index % num_query;
    const int batch_index = index / num_query;

    const __half2 * data_value_ptr =
      data_value + (batch_index * spatial_size * num_heads + head_index) * channels + channel_index;
    int data_weight_ptr =
      ((batch_index * num_query + query_index) * num_heads + head_index) * num_levels * num_point;
    int data_points_ptr = (batch_index * num_query + query_index) * points_per_group;
    __half2 * data_output_ptr = data_col + temp;
    __half2 output = __float2half2_rn(0.f);

    const int all_points = num_levels * num_point;
    __half max_weight = -cuda::std::numeric_limits<__half>::infinity();
    __half sum_weight = 0.f;
#pragma unroll
    for (int w_index = 0; w_index < all_points; ++w_index) {
      const __half weight = data_attn_weight[data_weight_ptr + w_index];
      max_weight = hmax(max_weight, weight);
    }

    for (int level_index = 0; level_index < num_levels; ++level_index) {
      const int spatial_h_ptr = level_index << 1;
      const int spatial_h = data_spatial_shapes[spatial_h_ptr];
      const int spatial_w = data_spatial_shapes[spatial_h_ptr + 1];

      for (int point_index = 0; point_index < num_point; ++point_index) {
        const int point_index_per_group = point_index % points_per_group;
        const __half2 reference_point_xy =
          data_reference_points[data_points_ptr + point_index_per_group];
        const __half2 offset =
          __hsub2(data_sampling_offsets[data_weight_ptr], __float2half2_rn(0.5f));
        const __half2 wh_im = __hfma2(
          reference_point_xy,
          __floats2half2_rn(static_cast<float>(spatial_w), static_cast<float>(spatial_h)), offset);

        const __half weight = hexp(data_attn_weight[data_weight_ptr] - max_weight);
        sum_weight += weight;

        const __half2 condition = __hmul2(
          __hgt2(wh_im, __float2half2_rn(-1.f)),
          __hlt2(
            wh_im,
            __floats2half2_rn(static_cast<float>(spatial_w), static_cast<float>(spatial_h))));

        if (__low2float(condition) * __high2float(condition)) {
          output = __hfma2(
            ms_deform_attn_im2col_bilinear_h2(
              data_value_ptr, spatial_h, spatial_w, num_heads, channels, wh_im),
            __half2half2(weight), output);
        }
        data_weight_ptr += 1;
      }
      data_value_ptr += spatial_h * spatial_w * channels * num_heads;
    }
    *data_output_ptr = __h2div(output, __half2half2(sum_weight));
  }
}

template <typename scalar_t>
__global__ void ms_deformable_im2col_gpu_kernel_int8(
  const int n, const int8_4 * data_value, float scale_value, const int32_t * data_spatial_shapes,
  const scalar_t * data_reference_points, const int8_4 * data_sampling_offsets, float scale_offset,
  const int8_4 * data_attn_weight, float scale_weight, const int spatial_size, const int num_heads,
  const int channels, const int num_levels, const int num_query, const int num_point,
  const int points_per_group, int8_4 * data_col, float scale_out)
{
  CUDA_1D_KERNEL_LOOP(index, n)
  {
    const int temp = index;
    const int channel_index = index % channels;
    index /= channels;
    const int head_index = index % num_heads;
    index /= num_heads;
    const int query_index = index % num_query;
    const int batch_index = index / num_query;

    const int8_4 * data_value_ptr =
      data_value + (batch_index * spatial_size * num_heads + head_index) * channels + channel_index;
    int data_weight_ptr =
      ((batch_index * num_query + query_index) * num_heads + head_index) * num_levels * num_point;
    int data_offset_w_ptr = data_weight_ptr << 1;
    int data_points_ptr = (batch_index * num_query + query_index) * points_per_group * 2;
    int8_4 * data_output_ptr = data_col + temp;
    int32_4 output = 0;
    const float scale_softmax = 127.f;
    const float scale_o = scale_value * __frcp_rn(scale_out);
    auto data_sampling_offsets_i8 = reinterpret_cast<const int8_t *>(data_sampling_offsets);
    auto data_attn_weight_i8 = reinterpret_cast<const int8_t *>(data_attn_weight);

    const int all_points = num_levels * num_point;
    float max_weight = -cuda::std::numeric_limits<float>::infinity();
    float sum_weight = 0.f;
#pragma unroll
    for (int w_index = 0; w_index < all_points; ++w_index) {
      const float weight =
        static_cast<float>(data_attn_weight_i8[data_weight_ptr + w_index]) * scale_weight;
      max_weight = max(max_weight, weight);
    }

    for (int level_index = 0; level_index < num_levels; ++level_index) {
      const int spatial_h_ptr = level_index << 1;
      const int spatial_h = data_spatial_shapes[spatial_h_ptr];
      const int spatial_w = data_spatial_shapes[spatial_h_ptr + 1];

      for (int point_index = 0; point_index < num_point / 4; point_index++) {
        int8_t values[16] = {0};
        int8_t weights_arr[4];
        auto weights = reinterpret_cast<int32_t *>(weights_arr);

#pragma unroll
        for (int i = 0; i < 4; i++, data_weight_ptr++, data_offset_w_ptr += 2) {
          const int point_index_per_group = (point_index * 4 + i) % points_per_group;
          const float reference_point_x =
            data_reference_points[data_points_ptr + point_index_per_group * 2];
          const float reference_point_y =
            data_reference_points[data_points_ptr + point_index_per_group * 2 + 1];
          const float loc_w = reference_point_x * spatial_w +
                              data_sampling_offsets_i8[data_offset_w_ptr] * scale_offset;
          const float loc_h = reference_point_y * spatial_h +
                              data_sampling_offsets_i8[data_offset_w_ptr + 1] * scale_offset;

          const float h_im = loc_h - 0.5f;
          const float w_im = loc_w - 0.5f;

          weights_arr[i] = T2int8(
            exp(data_attn_weight_i8[data_weight_ptr] * scale_weight - max_weight) * scale_softmax);
          sum_weight += weights_arr[i];

          if (!(h_im > -1 && w_im > -1 && h_im < spatial_h && w_im < spatial_w)) continue;
          const int8_4 & val = ms_deform_attn_im2col_bilinear_int8(
            data_value_ptr, spatial_h, spatial_w, num_heads, channels, h_im, w_im);

          values[i] = val.x;
          values[i + 4] = val.y;
          values[i + 8] = val.z;
          values[i + 12] = val.w;
        }

        dp4a((const int32_t *)values, weights, output.x);
        dp4a((const int32_t *)(values + 4), weights, output.y);
        dp4a((const int32_t *)(values + 8), weights, output.z);
        dp4a((const int32_t *)(values + 12), weights, output.w);
      }
      data_value_ptr += spatial_h * spatial_w * channels * num_heads;
    }
    int8_4 data_output;
    qmulf(output, data_output, scale_o * __frcp_rn(sum_weight));
    *data_output_ptr = data_output;
  }
}

template <>
__global__ void ms_deformable_im2col_gpu_kernel_int8(
  const int n, const int8_4 * data_value, float scale_value, const int32_t * data_spatial_shapes,
  const __half2 * data_reference_points, const int8_4 * data_sampling_offsets, float scale_offset,
  const int8_4 * data_attn_weight, float scale_weight, const int spatial_size, const int num_heads,
  const int channels, const int num_levels, const int num_query, const int num_point,
  const int points_per_group, int8_4 * data_col, float scale_out)
{
  CUDA_1D_KERNEL_LOOP(index, n)
  {
    const int temp = index;
    const int channel_index = index % channels;
    index /= channels;
    const int head_index = index % num_heads;
    index /= num_heads;
    const int query_index = index % num_query;
    const int batch_index = index / num_query;

    const int8_4 * data_value_ptr =
      data_value + (batch_index * spatial_size * num_heads + head_index) * channels + channel_index;
    int data_weight_ptr =
      ((batch_index * num_query + query_index) * num_heads + head_index) * num_levels * num_point >>
      2;
    int data_points_ptr = (batch_index * num_query + query_index) * points_per_group;
    int8_4 * data_output_ptr = data_col + temp;
    int32_4 output = 0;
    const __half scale_o = __float2half(scale_value * __frcp_rn(scale_out));
    const __half2 scale_softmax = __float2half2_rn(255.f);
    const __half2 scale_weight_h2 = __float2half2_rn(scale_weight);
    const __half2 scale_offset_h2 = __float2half2_rn(scale_offset);

    const int all_points = num_levels * num_point;
    __half2 max_weight = __half2half2(-cuda::std::numeric_limits<__half>::infinity());
    __half2 sum_weight = __float2half2_rn(0.f);
#pragma unroll
    for (int w_index = 0; w_index < all_points >> 2; ++w_index) {
      const int8_4 weights = data_attn_weight[data_weight_ptr + w_index];

      __half2 weight = __hmul2(
        __halves2half2(__short2half_rn(weights.x), __short2half_rn(weights.y)), scale_weight_h2);
      max_weight = hmax(max_weight, weight);

      weight = __hmul2(
        __halves2half2(__short2half_rn(weights.z), __short2half_rn(weights.w)), scale_weight_h2);
      max_weight = hmax(max_weight, weight);
    }
    max_weight = hmax(max_weight, __lowhigh2highlow(max_weight));

    for (int level_index = 0; level_index < num_levels; ++level_index) {
      const int spatial_h_ptr = level_index << 1;
      const int spatial_h = data_spatial_shapes[spatial_h_ptr];
      const int spatial_w = data_spatial_shapes[spatial_h_ptr + 1];
      const int32_t value_step = spatial_h * spatial_w * channels * num_heads;
      const __half2 spatial_wh = __halves2half2(__int2half_rn(spatial_w), __int2half_rn(spatial_h));

      for (int point_index = 0; point_index < (num_point >> 2); point_index++) {
        int8_t values[16] = {0};
        auto values_i32 = reinterpret_cast<int32_t *>(values);
        uint8_2 weights_arr[4];
        auto weights = reinterpret_cast<uint32_t *>(weights_arr);
        auto weights_i84 = reinterpret_cast<int8_4 *>(weights_arr);
        auto weights_h2 = reinterpret_cast<__half2 *>(weights_arr);
        *weights_i84 = data_attn_weight[data_weight_ptr];
        weights_h2[1] = __hmul2(
          h2exp(__hfma2(
            __halves2half2(__short2half_rn(weights_i84->z), __short2half_rn(weights_i84->w)),
            scale_weight_h2, __hneg2(max_weight))),
          scale_softmax);
        weights_h2[0] = __hmul2(
          h2exp(__hfma2(
            __halves2half2(__short2half_rn(weights_i84->x), __short2half_rn(weights_i84->y)),
            scale_weight_h2, __hneg2(max_weight))),
          scale_softmax);

#pragma unroll
        for (int i = 0; i < 2; i++) {
          const int8_4 offsets = data_sampling_offsets[(data_weight_ptr << 1) + i];
          const int point_index_per_group_0 = ((point_index << 2) + (i << 1)) % points_per_group;
          const int point_index_per_group_1 =
            ((point_index << 2) + (i << 1) + 1) % points_per_group;
          const __half2 reference_point_xy_0 =
            data_reference_points[data_points_ptr + point_index_per_group_0];
          const __half2 reference_point_xy_1 =
            data_reference_points[data_points_ptr + point_index_per_group_1];

          const __half2 offset_xy_0 = __hfma2(
            __halves2half2(__short2half_rn(offsets.x), __short2half_rn(offsets.y)), scale_offset_h2,
            __float2half2_rn(-0.5f));
          const __half2 offset_xy_1 = __hfma2(
            __halves2half2(__short2half_rn(offsets.z), __short2half_rn(offsets.w)), scale_offset_h2,
            __float2half2_rn(-0.5f));

          const __half2 wh_im_0 = __hfma2(reference_point_xy_0, spatial_wh, offset_xy_0);
          const __half2 wh_im_1 = __hfma2(reference_point_xy_1, spatial_wh, offset_xy_1);

          const __half2 condition_0 =
            __hmul2(__hgt2(wh_im_0, __float2half2_rn(-1.f)), __hlt2(wh_im_0, spatial_wh));
          const __half2 condition_1 =
            __hmul2(__hgt2(wh_im_1, __float2half2_rn(-1.f)), __hlt2(wh_im_1, spatial_wh));

          const __half2 condition =
            __halves2half2(condition_0.x * condition_0.y, condition_1.x * condition_1.y);

          if (condition.x) {
            ms_deform_attn_im2col_bilinear_int8_h2(
              data_value_ptr, spatial_h, spatial_w, num_heads, channels, wh_im_0, values + 2 * i);
          }
          if (condition.y) {
            ms_deform_attn_im2col_bilinear_int8_h2(
              data_value_ptr, spatial_h, spatial_w, num_heads, channels, wh_im_1,
              values + 2 * i + 1);
          }
          sum_weight = __hadd2(weights_h2[i], sum_weight);
          weights_h2[i] = __hmul2(weights_h2[i], condition);
          weights_arr[i].x = __half2ushort_rn(weights_h2[i].x);
          weights_arr[i].y = __half2ushort_rn(weights_h2[i].y);
        }
        data_weight_ptr++;

        dp4a(values_i32, weights, output.x);
        dp4a(++values_i32, weights, output.y);
        dp4a(++values_i32, weights, output.z);
        dp4a(++values_i32, weights, output.w);
      }
      data_value_ptr += value_step;
    }
    int8_4 data_output;
    qmulh(output, data_output, scale_o * hrcp(sum_weight.x + sum_weight.y));
    *data_output_ptr = data_output;
  }
}

template <typename scalar_t>
void ms_deformable_im2col_cuda(
  const scalar_t * data_value, const int32_t * data_spatial_shapes,
  const scalar_t * data_reference_points, const scalar_t * data_sampling_offsets,
  const scalar_t * data_attn_weight, const int batch_size, const int spatial_size,
  const int num_heads, const int channels, const int num_levels, const int num_query,
  const int num_point, const int points_per_group, scalar_t * data_col, cudaStream_t stream)
{
  const int num_kernels = batch_size * num_query * num_heads * channels;
  ms_deformable_im2col_gpu_kernel<scalar_t>
    <<<GET_BLOCKS(num_kernels), THREADS_PER_BLOCK, 0, stream>>>(
      num_kernels, data_value, data_spatial_shapes, data_reference_points, data_sampling_offsets,
      data_attn_weight, spatial_size, num_heads, channels, num_levels, num_query, num_point,
      points_per_group, data_col);
  cudaError_t err = cudaGetLastError();
  if (err != cudaSuccess) {
    printf("error in ms_deformable_im2col_cuda: %s\n", cudaGetErrorString(err));
  }
}

template <>
void ms_deformable_im2col_cuda(
  const __half * data_value, const int32_t * data_spatial_shapes,
  const __half * data_reference_points, const __half * data_sampling_offsets,
  const __half * data_attn_weight, const int batch_size, const int spatial_size,
  const int num_heads, const int channels, const int num_levels, const int num_query,
  const int num_point, const int points_per_group, __half * data_col, cudaStream_t stream)
{
  const int num_kernels = batch_size * num_query * num_heads * channels;
  ms_deformable_im2col_gpu_kernel<__half>
    <<<GET_BLOCKS(num_kernels), THREADS_PER_BLOCK, 0, stream>>>(
      num_kernels, data_value, data_spatial_shapes, data_reference_points, data_sampling_offsets,
      data_attn_weight, spatial_size, num_heads, channels, num_levels, num_query, num_point,
      points_per_group, data_col);
  cudaError_t err = cudaGetLastError();
  if (err != cudaSuccess) {
    printf("error in ms_deformable_im2col_cuda: %s\n", cudaGetErrorString(err));
  }
}

void ms_deformable_im2col_cuda_h2(
  const __half2 * data_value, const int32_t * data_spatial_shapes,
  const __half2 * data_reference_points, const __half2 * data_sampling_offsets,
  const __half * data_attn_weight, const int batch_size, const int spatial_size,
  const int num_heads, int channels, const int num_levels, const int num_query, const int num_point,
  const int points_per_group, __half2 * data_col, cudaStream_t stream)
{
  channels >>= 1;
  const int num_kernels = batch_size * num_query * num_heads * channels;
  ms_deformable_im2col_gpu_kernel_h2<<<GET_BLOCKS(num_kernels), THREADS_PER_BLOCK, 0, stream>>>(
    num_kernels, data_value, data_spatial_shapes, data_reference_points, data_sampling_offsets,
    data_attn_weight, spatial_size, num_heads, channels, num_levels, num_query, num_point,
    points_per_group, data_col);
  cudaError_t err = cudaGetLastError();
  if (err != cudaSuccess) {
    printf("error in ms_deformable_im2col_cuda: %s\n", cudaGetErrorString(err));
  }
}

template <typename scalar_t>
void ms_deformable_im2col_cuda_int8(
  const int8_4 * data_value, float scale_value, const int32_t * data_spatial_shapes,
  const scalar_t * data_reference_points, const int8_4 * data_sampling_offsets, float scale_offset,
  const int8_4 * data_attn_weight, float scale_weight, const int batch_size, const int spatial_size,
  const int num_heads, int channels, const int num_levels, const int num_query, const int num_point,
  const int points_per_group, int8_4 * data_col, float scale_out, cudaStream_t stream)
{
  channels >>= 2;
  const int num_kernels = batch_size * num_query * num_heads * channels;
  ms_deformable_im2col_gpu_kernel_int8<scalar_t>
    <<<GET_BLOCKS(num_kernels), THREADS_PER_BLOCK, 0, stream>>>(
      num_kernels, data_value, scale_value, data_spatial_shapes, data_reference_points,
      data_sampling_offsets, scale_offset, data_attn_weight, scale_weight, spatial_size, num_heads,
      channels, num_levels, num_query, num_point, points_per_group, data_col, scale_out);
  cudaError_t err = cudaGetLastError();
  if (err != cudaSuccess) {
    printf("error in ms_deformable_im2col_cuda: %s\n", cudaGetErrorString(err));
  }
}

template <>
void ms_deformable_im2col_cuda_int8(
  const int8_4 * data_value, float scale_value, const int32_t * data_spatial_shapes,
  const __half2 * data_reference_points, const int8_4 * data_sampling_offsets, float scale_offset,
  const int8_4 * data_attn_weight, float scale_weight, const int batch_size, const int spatial_size,
  const int num_heads, int channels, const int num_levels, const int num_query, const int num_point,
  const int points_per_group, int8_4 * data_col, float scale_out, cudaStream_t stream)
{
  channels >>= 2;
  const int num_kernels = batch_size * num_query * num_heads * channels;
  ms_deformable_im2col_gpu_kernel_int8<<<GET_BLOCKS(num_kernels), THREADS_PER_BLOCK, 0, stream>>>(
    num_kernels, data_value, scale_value, data_spatial_shapes, data_reference_points,
    data_sampling_offsets, scale_offset, data_attn_weight, scale_weight, spatial_size, num_heads,
    channels, num_levels, num_query, num_point, points_per_group, data_col, scale_out);
  cudaError_t err = cudaGetLastError();
  if (err != cudaSuccess) {
    printf("error in ms_deformable_im2col_cuda: %s\n", cudaGetErrorString(err));
  }
}

template void ms_deformable_im2col_cuda<float>(
  const float * data_value, const int32_t * data_spatial_shapes,
  const float * data_reference_points, const float * data_sampling_offsets,
  const float * data_attn_weight, const int batch_size, const int spatial_size, const int num_heads,
  const int channels, const int num_levels, const int num_query, const int num_point,
  const int points_per_group, float * data_col, cudaStream_t stream);

template void ms_deformable_im2col_cuda<__half>(
  const __half * data_value, const int32_t * data_spatial_shapes,
  const __half * data_reference_points, const __half * data_sampling_offsets,
  const __half * data_attn_weight, const int batch_size, const int spatial_size,
  const int num_heads, const int channels, const int num_levels, const int num_query,
  const int num_point, const int points_per_group, __half * data_col, cudaStream_t stream);

template void ms_deformable_im2col_cuda_int8<float>(
  const int8_4 * data_value, float scale_value, const int32_t * data_spatial_shapes,
  const float * data_reference_points, const int8_4 * data_sampling_offsets, float scale_offset,
  const int8_4 * data_attn_weight, float scale_weight, const int batch_size, const int spatial_size,
  const int num_heads, int channels, const int num_levels, const int num_query, const int num_point,
  const int points_per_group, int8_4 * data_col, float scale_out, cudaStream_t stream);

template void ms_deformable_im2col_cuda_int8<__half2>(
  const int8_4 * data_value, float scale_value, const int32_t * data_spatial_shapes,
  const __half2 * data_reference_points, const int8_4 * data_sampling_offsets, float scale_offset,
  const int8_4 * data_attn_weight, float scale_weight, const int batch_size, const int spatial_size,
  const int num_heads, int channels, const int num_levels, const int num_query, const int num_point,
  const int points_per_group, int8_4 * data_col, float scale_out, cudaStream_t stream);
