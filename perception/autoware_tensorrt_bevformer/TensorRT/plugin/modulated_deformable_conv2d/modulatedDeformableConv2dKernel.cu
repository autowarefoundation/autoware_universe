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
#include "modulatedDeformableConv2dKernel.h"

#include <cuda_fp16.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

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

__forceinline__ __device__ void qmulf(const int8_4 & a, int8_4 & c, const float & b)
{
  c.x = T2int8<float>(a.x * b);
  c.y = T2int8<float>(a.y * b);
  c.z = T2int8<float>(a.z * b);
  c.w = T2int8<float>(a.w * b);
}

__forceinline__ __device__ void dp4a(const int32_t * a, const int32_t * b, int32_t & c)
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
__device__ scalar_t dmcn_im2col_bilinear(
  const scalar_t * input, const int data_width, const int height, const int width, scalar_t h,
  scalar_t w)
{
  int h_low = floorf(h);
  int w_low = floorf(w);
  int h_high = h_low + 1;
  int w_high = w_low + 1;

  scalar_t lh = h - h_low;
  scalar_t lw = w - w_low;
  scalar_t hh = 1 - lh, hw = 1 - lw;

  scalar_t v1 = 0;
  if (h_low >= 0 && w_low >= 0) v1 = input[h_low * data_width + w_low];
  scalar_t v2 = 0;
  if (h_low >= 0 && w_high <= width - 1) v2 = input[h_low * data_width + w_high];
  scalar_t v3 = 0;
  if (h_high <= height - 1 && w_low >= 0) v3 = input[h_high * data_width + w_low];
  scalar_t v4 = 0;
  if (h_high <= height - 1 && w_high <= width - 1) v4 = input[h_high * data_width + w_high];

  scalar_t w1 = hh * hw, w2 = hh * lw, w3 = lh * hw, w4 = lh * lw;

  scalar_t val = (w1 * v1 + w2 * v2 + w3 * v3 + w4 * v4);
  return val;
}

template <>
__device__ __half dmcn_im2col_bilinear(
  const __half * input, const int data_width, const int height, const int width, __half h, __half w)
{
  __half h_low = hfloor(h);
  __half w_low = hfloor(w);
  __half h_high = __hadd(h_low, __float2half(1.f));
  __half w_high = __hadd(w_low, __float2half(1.f));

  __half lh = __hsub(h, h_low);
  __half lw = __hsub(w, w_low);
  __half hh = __hsub(__float2half(1.f), lh), hw = __hsub(__float2half(1.f), lw);

  __half v1 = __float2half(0.f);
  if (__hge(h_low, __float2half(0.f)) && __hge(w_low, __float2half(0.f)))
    v1 = input[static_cast<int>(h_low) * data_width + static_cast<int>(w_low)];
  __half v2 = __float2half(0.f);
  if (__hge(h_low, __float2half(0.f)) && __hle(w_high, __float2half(static_cast<float>(width - 1))))
    v2 = input[static_cast<int>(h_low) * data_width + static_cast<int>(w_high)];
  __half v3 = __float2half(0.f);
  if (
    __hle(h_high, __float2half(static_cast<float>(height - 1))) && __hge(w_low, __float2half(0.f)))
    v3 = input[static_cast<int>(h_high) * data_width + static_cast<int>(w_low)];
  __half v4 = __float2half(0.f);
  if (
    __hle(h_high, __float2half(static_cast<float>(height - 1))) &&
    __hle(w_high, __float2half(static_cast<float>(width - 1))))
    v4 = input[static_cast<int>(h_high) * data_width + static_cast<int>(w_high)];

  __half w1 = __hmul(hh, hw), w2 = __hmul(hh, lw), w3 = __hmul(lh, hw), w4 = __hmul(lh, lw);

  __half val =
    __hadd(__hadd(__hmul(w1, v1), __hmul(w2, v2)), __hadd(__hmul(w3, v3), __hmul(w4, v4)));
  return val;
}

__device__ __half2 dmcn_im2col_bilinear_h2(
  const __half2 * input, const int data_width, const int height, const int width, __half2 hw)
{
  __half2 hw_low = h2floor(hw);
  __half2 hw_high = __hadd2(hw_low, __float2half2_rn(1.f));
  __half2 lhw = __hsub2(hw, hw_low);
  __half2 hhw = __hsub2(__float2half2_rn(1.f), lhw);
  __half2 w1 = __half2half2(__hmul(__low2half(hhw), __high2half(hhw))),
          w2 = __half2half2(__hmul(__low2half(hhw), __high2half(lhw))),
          w3 = __half2half2(__hmul(__low2half(lhw), __high2half(hhw))),
          w4 = __half2half2(__hmul(__low2half(lhw), __high2half(lhw)));

  int h_low = static_cast<int>(__low2float(hw_low)), w_low = static_cast<int>(__high2float(hw_low)),
      h_high = static_cast<int>(__low2float(hw_high)),
      w_high = static_cast<int>(__high2float(hw_high));
  __half2 val = __float2half2_rn(0.f);

  if (h_low >= 0 && w_low >= 0) {
    val = __hfma2(input[h_low * data_width + w_low], w1, val);
  }
  if (h_low >= 0 && w_high < width) {
    val = __hfma2(input[h_low * data_width + w_high], w2, val);
  }
  if (h_high < height && w_low >= 0) {
    val = __hfma2(input[h_high * data_width + w_low], w3, val);
  }
  if (h_high < height && w_high < width) {
    val = __hfma2(input[h_high * data_width + w_high], w4, val);
  }
  return val;
}

__forceinline__ __device__ void dmcn_im2col_bilinear_int8(
  const int8_4 * input, const float & scale_i, const int data_width, const int height,
  const int width, __half2 hw, int8_4 & output)
{
  __half2 hw_low = h2floor(hw);
  __half2 hw_high = __hadd2(hw_low, __float2half2_rn(1.f));
  __half2 lhw = __hsub2(hw, hw_low);
  __half2 hhw = __hsub2(__float2half2_rn(1.f), lhw);

  const float scale_area = 1 / 255.f;
  uint8_4 weight = {
    half2uint8(__hmul(__low2half(hhw), __high2half(hhw)), scale_area),
    half2uint8(__hmul(__low2half(hhw), __high2half(lhw)), scale_area),
    half2uint8(__hmul(__low2half(lhw), __high2half(hhw)), scale_area),
    half2uint8(__hmul(__low2half(lhw), __high2half(lhw)), scale_area)};

  int h_low = static_cast<int>(__low2float(hw_low)), w_low = static_cast<int>(__high2float(hw_low)),
      h_high = static_cast<int>(__low2float(hw_high)),
      w_high = static_cast<int>(__high2float(hw_high));
  int8_4 inps[4] = {0, 0, 0, 0};
  int32_t output_temp;

  if (h_low >= 0 && w_low >= 0) {
    const int8_4 & inp = input[h_low * data_width + w_low];
    inps[0].x = inp.x;
    inps[1].x = inp.y;
    inps[2].x = inp.z;
    inps[3].x = inp.w;
  }
  if (h_low >= 0 && w_high < width) {
    const int8_4 & inp = input[h_low * data_width + w_high];
    inps[0].y = inp.x;
    inps[1].y = inp.y;
    inps[2].y = inp.z;
    inps[3].y = inp.w;
  }
  if (h_high < height && w_low >= 0) {
    const int8_4 & inp = input[h_high * data_width + w_low];
    inps[0].z = inp.x;
    inps[1].z = inp.y;
    inps[2].z = inp.z;
    inps[3].z = inp.w;
  }
  if (h_high < height && w_high < width) {
    const int8_4 & inp = input[h_high * data_width + w_high];
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
}

template <typename scalar_t>
__global__ void modulated_deformable_im2col_gpu_kernel(
  const int n, const scalar_t * data_im, const scalar_t * data_offset, const scalar_t * data_mask,
  const int height, const int width, const int kernel_h, const int kernel_w, const int pad_h,
  const int pad_w, const int stride_h, const int stride_w, const int dilation_h,
  const int dilation_w, const int channel_per_deformable_group, const int batch_size,
  const int num_channels, const int deformable_group, const int height_col, const int width_col,
  scalar_t * data_col)
{
  CUDA_1D_KERNEL_LOOP(index, n)
  {
    // index of output matrix
    const int w_col = index % width_col;
    const int h_col = (index / width_col) % height_col;
    const int b_col = (index / width_col / height_col) % batch_size;
    const int c_im = (index / width_col / height_col) / batch_size;
    const int c_col = c_im * kernel_h * kernel_w;

    // compute deformable group index
    const int deformable_group_index = c_im / channel_per_deformable_group;

    const int h_in = h_col * stride_h - pad_h;
    const int w_in = w_col * stride_w - pad_w;

    scalar_t * data_col_ptr =
      data_col + ((c_col * batch_size + b_col) * height_col + h_col) * width_col + w_col;
    const scalar_t * data_im_ptr = data_im + (b_col * num_channels + c_im) * height * width;
    const scalar_t * data_offset_ptr =
      data_offset + (b_col * deformable_group + deformable_group_index) * 2 * kernel_h * kernel_w *
                      height_col * width_col;

    const scalar_t * data_mask_ptr =
      data_mask + (b_col * deformable_group + deformable_group_index) * kernel_h * kernel_w *
                    height_col * width_col;

    for (int i = 0; i < kernel_h; ++i) {
      for (int j = 0; j < kernel_w; ++j) {
        const int data_offset_h_ptr =
          ((2 * (i * kernel_w + j)) * height_col + h_col) * width_col + w_col;
        const int data_offset_w_ptr =
          ((2 * (i * kernel_w + j) + 1) * height_col + h_col) * width_col + w_col;
        const int data_mask_hw_ptr = ((i * kernel_w + j) * height_col + h_col) * width_col + w_col;
        const scalar_t offset_h = data_offset_ptr[data_offset_h_ptr];
        const scalar_t offset_w = data_offset_ptr[data_offset_w_ptr];
        const scalar_t mask = data_mask_ptr[data_mask_hw_ptr];

        auto val = static_cast<scalar_t>(0);
        const scalar_t h_im = h_in + i * dilation_h + offset_h;
        const scalar_t w_im = w_in + j * dilation_w + offset_w;

        if (h_im > -1 && w_im > -1 && h_im < height && w_im < width)
          val = dmcn_im2col_bilinear(data_im_ptr, width, height, width, h_im, w_im);

        *data_col_ptr = val * mask;
        data_col_ptr += batch_size * height_col * width_col;
      }
    }
  }
}

template <>
__global__ void modulated_deformable_im2col_gpu_kernel(
  const int n, const __half * data_im, const __half * data_offset, const __half * data_mask,
  const int height, const int width, const int kernel_h, const int kernel_w, const int pad_h,
  const int pad_w, const int stride_h, const int stride_w, const int dilation_h,
  const int dilation_w, const int channel_per_deformable_group, const int batch_size,
  const int num_channels, const int deformable_group, const int height_col, const int width_col,
  __half * data_col)
{
  CUDA_1D_KERNEL_LOOP(index, n)
  {
    // index of output matrix
    const int w_col = index % width_col;
    const int h_col = (index / width_col) % height_col;
    const int b_col = (index / width_col / height_col) % batch_size;
    const int c_im = (index / width_col / height_col) / batch_size;
    const int c_col = c_im * kernel_h * kernel_w;

    // compute deformable group index
    const int deformable_group_index = c_im / channel_per_deformable_group;

    const int h_in = h_col * stride_h - pad_h;
    const int w_in = w_col * stride_w - pad_w;

    __half * data_col_ptr =
      data_col + ((c_col * batch_size + b_col) * height_col + h_col) * width_col + w_col;
    const __half * data_im_ptr = data_im + (b_col * num_channels + c_im) * height * width;
    const __half * data_offset_ptr =
      data_offset + (b_col * deformable_group + deformable_group_index) * 2 * kernel_h * kernel_w *
                      height_col * width_col;

    const __half * data_mask_ptr = data_mask + (b_col * deformable_group + deformable_group_index) *
                                                 kernel_h * kernel_w * height_col * width_col;

    for (int i = 0; i < kernel_h; ++i) {
      for (int j = 0; j < kernel_w; ++j) {
        const int data_offset_h_ptr =
          ((2 * (i * kernel_w + j)) * height_col + h_col) * width_col + w_col;
        const int data_offset_w_ptr =
          ((2 * (i * kernel_w + j) + 1) * height_col + h_col) * width_col + w_col;
        const int data_mask_hw_ptr = ((i * kernel_w + j) * height_col + h_col) * width_col + w_col;
        const __half offset_h = data_offset_ptr[data_offset_h_ptr];
        const __half offset_w = data_offset_ptr[data_offset_w_ptr];
        const __half mask = data_mask_ptr[data_mask_hw_ptr];
        __half val = __float2half(0.f);
        const __half h_im = __hadd(
          offset_h,
          __hfma(
            __float2half(static_cast<float>(i)), __float2half(static_cast<float>(dilation_h)),
            __float2half(static_cast<float>(h_in))));
        const __half w_im = __hadd(
          offset_w,
          __hfma(
            __float2half(static_cast<float>(j)), __float2half(static_cast<float>(dilation_w)),
            __float2half(static_cast<float>(w_in))));
        if (
          __hgt(h_im, __float2half(-1.f)) && __hgt(w_im, __float2half(-1.f)) &&
          __hlt(h_im, __float2half(static_cast<float>(height))) &&
          __hlt(w_im, __float2half(static_cast<float>(width))))
          val = dmcn_im2col_bilinear(data_im_ptr, width, height, width, h_im, w_im);
        *data_col_ptr = __hmul(val, mask);
        data_col_ptr += batch_size * height_col * width_col;
      }
    }
  }
}

__global__ void modulated_deformable_im2col_gpu_kernel_h2(
  const int n, const __half2 * data_im, const __half2 * data_offset, const __half * data_mask,
  const int height, const int width, const int kernel_h, const int kernel_w, const int pad_h,
  const int pad_w, const int stride_h, const int stride_w, const int dilation_h,
  const int dilation_w, const int channel_per_deformable_group, const int batch_size,
  const int num_channels, const int deformable_group, const int height_col, const int width_col,
  __half2 * data_col)
{
  CUDA_1D_KERNEL_LOOP(index, n)
  {
    // index of output matrix
    const int w_col = index % width_col;
    const int h_col = (index / width_col) % height_col;
    const int b_col = (index / width_col / height_col) % batch_size;
    const int c_im = (index / width_col / height_col) / batch_size;
    const int c_col = c_im * kernel_h * kernel_w;

    // compute deformable group index
    const int deformable_group_index = c_im / channel_per_deformable_group;

    const int h_in = h_col * stride_h - pad_h;
    const int w_in = w_col * stride_w - pad_w;

    auto * data_col_ptr = (__half *)data_col +
                          ((c_col * 2 * batch_size + b_col * 2) * height_col + h_col) * width_col +
                          w_col;
    const __half2 * data_im_ptr =
      data_im + (b_col * ((num_channels + 1) / 2) + c_im) * height * width;
    const __half2 * data_offset_ptr =
      data_offset + (b_col * deformable_group + deformable_group_index) * kernel_h * kernel_w *
                      height_col * width_col;

    const __half * data_mask_ptr = data_mask + (b_col * deformable_group + deformable_group_index) *
                                                 kernel_h * kernel_w * height_col * width_col;

    __half2 condition;
    for (int i = 0; i < kernel_h; ++i) {
      for (int j = 0; j < kernel_w; ++j) {
        const int data_offset_hw_ptr =
          ((i * kernel_w + j) * height_col + h_col) * width_col + w_col;
        const int data_mask_hw_ptr = ((i * kernel_w + j) * height_col + h_col) * width_col + w_col;
        const __half2 offset_hw = data_offset_ptr[data_offset_hw_ptr];
        const __half2 mask = __half2half2(data_mask_ptr[data_mask_hw_ptr]);
        __half2 val = __float2half2_rn(0.f);
        const __half2 hw_im = __hadd2(
          offset_hw,
          __hfma2(
            __floats2half2_rn(static_cast<float>(i), static_cast<float>(j)),
            __floats2half2_rn(static_cast<float>(dilation_h), static_cast<float>(dilation_w)),
            __floats2half2_rn(static_cast<float>(h_in), static_cast<float>(w_in))));
        condition = __hmul2(
          __hgt2(hw_im, __float2half2_rn(-1.f)),
          __hlt2(hw_im, __floats2half2_rn(static_cast<float>(height), static_cast<float>(width))));
        if (__low2float(condition) * __high2float(condition)) {
          val = dmcn_im2col_bilinear_h2(data_im_ptr, width, height, width, hw_im);
        }
        val = __hmul2(val, mask);
        *data_col_ptr = __low2half(val);
        *(data_col_ptr + batch_size * height_col * width_col) = __high2half(val);
        data_col_ptr += batch_size * height_col * width_col * 2;
      }
    }
  }
}

__global__ void modulated_deformable_im2col_gpu_kernel_int8(
  const int n, const int8_4 * data_im, const float scale_i, const int8_t * data_offset,
  const float scale_off, const int8_t * data_mask, const float scale_mask, const int height,
  const int width, const int kernel_h, const int kernel_w, const int pad_h, const int pad_w,
  const int stride_h, const int stride_w, const int dilation_h, const int dilation_w,
  const int channel_per_deformable_group, const int batch_size, const int num_channels,
  const int deformable_group, const int height_col, const int width_col, const int hw4,
  int8_4 * data_col)
{
  CUDA_1D_KERNEL_LOOP(index, n)
  {
    // index of output matrix
    const int w_col = index % width_col;
    const int h_col = (index / width_col) % height_col;
    const int b_col = (index / width_col / height_col) % batch_size;
    const int c_im = (index / width_col / height_col) / batch_size;
    const int c_col = c_im * kernel_h * kernel_w;

    // compute deformable group index
    const int deformable_group_index = c_im / channel_per_deformable_group;

    const int h_in = h_col * stride_h - pad_h;
    const int w_in = w_col * stride_w - pad_w;

    auto * data_col_ptr =
      (int8_t *)data_col + (c_col * 4 * batch_size + b_col * 4) * hw4 + h_col * width_col + w_col;
    const int8_4 * data_im_ptr =
      data_im + (b_col * ((num_channels + 3) / 4) + c_im) * height * width;
    const int8_t * data_offset_ptr =
      data_offset + (b_col * deformable_group + deformable_group_index) * kernel_h * kernel_w *
                      height_col * width_col * 2;

    const int8_t * data_mask_ptr = data_mask + (b_col * deformable_group + deformable_group_index) *
                                                 kernel_h * kernel_w * height_col * width_col;
    const int32_t output_step = batch_size * hw4;

    __half2 condition;
    for (int i = 0; i < kernel_h; ++i) {
      for (int j = 0; j < kernel_w; ++j) {
        const int data_offset_h_ptr =
          ((2 * (i * kernel_w + j)) * height_col + h_col) * width_col + w_col;
        const int data_offset_w_ptr =
          ((2 * (i * kernel_w + j) + 1) * height_col + h_col) * width_col + w_col;
        const int data_mask_hw_ptr = ((i * kernel_w + j) * height_col + h_col) * width_col + w_col;

        const __half2 offset_hw = __floats2half2_rn(
          data_offset_ptr[data_offset_h_ptr] * scale_off,
          data_offset_ptr[data_offset_w_ptr] * scale_off);

        const float mask = data_mask_ptr[data_mask_hw_ptr] * scale_mask;

        const __half2 hw_im = __hadd2(
          offset_hw,
          __hfma2(
            __floats2half2_rn(static_cast<float>(i), static_cast<float>(j)),
            __floats2half2_rn(static_cast<float>(dilation_h), static_cast<float>(dilation_w)),
            __floats2half2_rn(static_cast<float>(h_in), static_cast<float>(w_in))));
        condition = __hmul2(
          __hgt2(hw_im, __float2half2_rn(-1.f)),
          __hlt2(hw_im, __floats2half2_rn(static_cast<float>(height), static_cast<float>(width))));
        int8_4 val = 0;
        if (__low2float(condition) * __high2float(condition)) {
          dmcn_im2col_bilinear_int8(data_im_ptr, scale_i, width, height, width, hw_im, val);
        }
        qmulf(val, val, mask);

        *data_col_ptr = val.x;
        data_col_ptr += output_step;
        *data_col_ptr = val.y;
        data_col_ptr += output_step;
        *data_col_ptr = val.z;
        data_col_ptr += output_step;
        *data_col_ptr = val.w;
        data_col_ptr += output_step;
      }
    }
  }
}

template <typename scalar_t>
__global__ void output_add_bias_kernel(
  scalar_t * output, const scalar_t * bias, size_t step_batch, size_t step_channel, size_t n)
{
  CUDA_1D_KERNEL_LOOP(index, n)
  {
    // Calculate bias index
    size_t bias_index = (index % step_batch) / step_channel;
    // Add bias to output
    output[index] += bias[bias_index];
  }
}

template <>
__global__ void output_add_bias_kernel(
  __half * output, const __half * bias, size_t step_batch, size_t step_channel, size_t n)
{
  CUDA_1D_KERNEL_LOOP(index, n)
  {
    output[index] = __hadd(bias[(index % step_batch) / step_channel], output[index]);
  }
}

template <typename T>
__global__ void output_add_bias_kernel_int8(
  const int32_t * int32_iw, float scale_iw, const T * bias, int8_t * output, float scale_o,
  size_t n, int hw_out, int hw4)
{
  CUDA_1D_KERNEL_LOOP(index, n)
  {
    const int bias_index = index / hw_out;
    const int temp32_index = bias_index * hw4 + index % hw_out;

    output[index] = T2int8<float>((int32_iw[temp32_index] * scale_iw + bias[bias_index]) / scale_o);
  }
}

template <>
__global__ void output_add_bias_kernel_int8(
  const int32_t * int32_iw, float scale_iw, const __half * bias, int8_t * output, float scale_o,
  size_t n, int hw_out, int hw4)
{
  CUDA_1D_KERNEL_LOOP(index, n)
  {
    const int bias_index = index / hw_out;
    const int temp32_index = bias_index * hw4 + index % hw_out;

    output[index] =
      T2int8<float>((int32_iw[temp32_index] * scale_iw + __half2float(bias[bias_index])) / scale_o);
  }
}

__global__ void output_wo_bias_kernel_int8(
  const int32_t * int32_iw, float scale_iw, int8_t * output, float scale_o, size_t n, int hw_out,
  int hw4)
{
  CUDA_1D_KERNEL_LOOP(index, n)
  {
    const int temp32_index = index / hw_out * hw4 + index % hw_out;
    output[index] = T2int8<float>((int32_iw[temp32_index] * scale_iw) / scale_o);
  }
}

template <typename scalar_t>
void trt_modulated_deformable_im2col(
  const scalar_t * data_im_, const scalar_t * data_offset_, const scalar_t * data_mask_,
  const int batch_size, const int channels, const int height_im, const int width_im,
  const int height_col, const int width_col, const int kernel_h, const int kenerl_w,
  const int pad_h, const int pad_w, const int stride_h, const int stride_w, const int dilation_h,
  const int dilation_w, const int deformable_group, scalar_t * data_col_, cudaStream_t stream)
{
  // num_axes should be smaller than block size
  const int channel_per_deformable_group = channels / deformable_group;
  const int num_kernels = channels * batch_size * height_col * width_col;

  modulated_deformable_im2col_gpu_kernel<scalar_t>
    <<<GET_BLOCKS(num_kernels), THREADS_PER_BLOCK, 0, stream>>>(
      num_kernels, data_im_, data_offset_, data_mask_, height_im, width_im, kernel_h, kenerl_w,
      pad_h, pad_w, stride_h, stride_w, dilation_h, dilation_w, channel_per_deformable_group,
      batch_size, channels, deformable_group, height_col, width_col, data_col_);

  cudaCheckError();
}

void trt_modulated_deformable_im2col_h2(
  const __half2 * data_im_, const __half2 * data_offset_, const __half * data_mask_,
  const int batch_size, const int channels, const int height_im, const int width_im,
  const int height_col, const int width_col, const int kernel_h, const int kenerl_w,
  const int pad_h, const int pad_w, const int stride_h, const int stride_w, const int dilation_h,
  const int dilation_w, const int deformable_group, __half2 * data_col_, cudaStream_t stream)
{
  // num_axes should be smaller than block size
  const int channel_per_deformable_group = (channels + 1) / 2 / deformable_group;
  const int num_kernels = (channels + 1) / 2 * batch_size * height_col * width_col;

  modulated_deformable_im2col_gpu_kernel_h2<<<
    GET_BLOCKS(num_kernels), THREADS_PER_BLOCK, 0, stream>>>(
    num_kernels, data_im_, data_offset_, data_mask_, height_im, width_im, kernel_h, kenerl_w, pad_h,
    pad_w, stride_h, stride_w, dilation_h, dilation_w, channel_per_deformable_group, batch_size,
    channels, deformable_group, height_col, width_col, data_col_);

  cudaCheckError();
}

void trt_modulated_deformable_im2col_int8(
  const int8_4 * data_im_, const float & scale_i, const int8_t * data_offset_,
  const float & scale_off, const int8_t * data_mask_, const float & scale_mask,
  const int batch_size, const int channels, const int height_im, const int width_im,
  const int height_col, const int width_col, const int kernel_h, const int kenerl_w,
  const int pad_h, const int pad_w, const int stride_h, const int stride_w, const int dilation_h,
  const int dilation_w, const int deformable_group, int8_4 * data_col_, cudaStream_t stream)
{
  // num_axes should be smaller than block size
  const int channel_per_deformable_group = (channels + 3) / 4 / deformable_group;
  const int num_kernels = (channels + 3) / 4 * batch_size * height_col * width_col;
  const int hw4 = (height_col * width_col + 3) / 4 * 4;

  modulated_deformable_im2col_gpu_kernel_int8<<<
    GET_BLOCKS(num_kernels), THREADS_PER_BLOCK, 0, stream>>>(
    num_kernels, data_im_, scale_i, data_offset_, scale_off, data_mask_, scale_mask, height_im,
    width_im, kernel_h, kenerl_w, pad_h, pad_w, stride_h, stride_w, dilation_h, dilation_w,
    channel_per_deformable_group, batch_size, channels, deformable_group, height_col, width_col,
    hw4, data_col_);

  cudaCheckError();
}

template <typename scalar_t>
static void output_add_bias(
  scalar_t * output, const scalar_t * bias, size_t batch, size_t channel, size_t height,
  size_t width, cudaStream_t stream)
{
  size_t step_channel = height * width;
  size_t step_batch = step_channel * channel;
  size_t n = step_batch * batch;

  int blocks = GET_BLOCKS(n);

  output_add_bias_kernel<scalar_t>
    <<<blocks, THREADS_PER_BLOCK, 0, stream>>>(output, bias, step_batch, step_channel, n);
}

template <typename scalar_t>
void ModulatedDeformConvForwardCUDAKernel(
  const scalar_t * input, const scalar_t * weight, const scalar_t * bias, const scalar_t * offset,
  const scalar_t * mask, scalar_t * output, void * workspace, int batch, int channels, int height,
  int width, int channels_out, int kernel_w, int kernel_h, int stride_w, int stride_h, int pad_w,
  int pad_h, int dilation_w, int dilation_h, int group, int deformable_group, int im2col_step,
  cublasHandle_t cublas_handle, cudaStream_t stream)
{
  im2col_step = std::min(int(batch), im2col_step);
  ASSERT(batch % im2col_step == 0)

  const int height_out = (height + 2 * pad_h - (dilation_h * (kernel_h - 1) + 1)) / stride_h + 1;
  const int width_out = (width + 2 * pad_w - (dilation_w * (kernel_w - 1) + 1)) / stride_w + 1;

  auto columns = (scalar_t *)workspace;

  const size_t input_step = channels * height * width;
  const size_t offset_step = deformable_group * kernel_h * kernel_w * 2 * height_out * width_out;
  const size_t mask_step = deformable_group * kernel_h * kernel_w * height_out * width_out;
  const size_t out_step = channels_out * height_out * width_out;
  const size_t out_group_step = out_step / group;
  const size_t col_g_step = channels * kernel_w * kernel_h / group * height_out * width_out;
  const size_t weight_g_step = channels_out / group * channels / group * kernel_h * kernel_w;

  const int m = channels_out / group;
  const int n = height_out * width_out;
  const int k = channels / group * kernel_h * kernel_w;
  scalar_t alpha = 1.;
  scalar_t beta = 0.;

  for (int b = 0; b < batch; b++) {
    const scalar_t * input_start = input + b * input_step;
    const scalar_t * offset_start = offset + b * offset_step;
    const scalar_t * mask_start = mask + b * mask_step;

    trt_modulated_deformable_im2col<scalar_t>(
      input_start, offset_start, mask_start, 1, channels, height, width, height_out, width_out,
      kernel_h, kernel_w, pad_h, pad_w, stride_h, stride_w, dilation_h, dilation_w,
      deformable_group, columns, stream);

    for (int g = 0; g < group; g++) {
      const scalar_t * weight_start = weight + g * weight_g_step;
      scalar_t * col_start = columns + g * col_g_step;
      scalar_t * out_buffer_start = output + b * out_step + g * out_group_step;
      cublasGemmWrap<scalar_t>(
        cublas_handle, CUBLAS_OP_N, CUBLAS_OP_N, n, m, k, &alpha, col_start, n, weight_start, k,
        &beta, out_buffer_start, n);
      cudaCheckError();
    }
  }
}

template <>
void ModulatedDeformConvForwardCUDAKernel(
  const __half * input, const __half * weight, const __half * bias, const __half * offset,
  const __half * mask, __half * output, void * workspace, int batch, int channels, int height,
  int width, int channels_out, int kernel_w, int kernel_h, int stride_w, int stride_h, int pad_w,
  int pad_h, int dilation_w, int dilation_h, int group, int deformable_group, int im2col_step,
  cublasHandle_t cublas_handle, cudaStream_t stream)
{
  bool with_bias = (bias != nullptr);

  im2col_step = std::min(int(batch), im2col_step);
  ASSERT(batch % im2col_step == 0)

  const int height_out = (height + 2 * pad_h - (dilation_h * (kernel_h - 1) + 1)) / stride_h + 1;
  const int width_out = (width + 2 * pad_w - (dilation_w * (kernel_w - 1) + 1)) / stride_w + 1;

  auto columns = (__half *)workspace;

  const size_t input_step = channels * height * width;
  const size_t offset_step = deformable_group * kernel_h * kernel_w * 2 * height_out * width_out;
  const size_t mask_step = deformable_group * kernel_h * kernel_w * height_out * width_out;
  const size_t out_step = channels_out * height_out * width_out;
  const size_t out_group_step = out_step / group;
  const size_t col_g_step = channels * kernel_w * kernel_h / group * height_out * width_out;
  const size_t weight_g_step = channels_out / group * channels / group * kernel_h * kernel_w;

  const int m = channels_out / group;
  const int n = height_out * width_out;
  const int k = channels / group * kernel_h * kernel_w;
  __half alpha = __float2half(1.f);
  __half beta = __float2half(0.f);

  for (int b = 0; b < batch; b++) {
    const __half * input_start = input + b * input_step;
    const __half * offset_start = offset + b * offset_step;
    const __half * mask_start = mask + b * mask_step;
    trt_modulated_deformable_im2col<__half>(
      input_start, offset_start, mask_start, 1, channels, height, width, height_out, width_out,
      kernel_h, kernel_w, pad_h, pad_w, stride_h, stride_w, dilation_h, dilation_w,
      deformable_group, columns, stream);

    for (int g = 0; g < group; g++) {
      const __half * weight_start = weight + g * weight_g_step;
      __half * col_start = columns + g * col_g_step;
      __half * out_buffer_start = output + b * out_step + g * out_group_step;

      cublasGemmWrap<__half>(
        cublas_handle, CUBLAS_OP_N, CUBLAS_OP_N, n, m, k, &alpha, col_start, n, weight_start, k,
        &beta, out_buffer_start, n);
      cudaCheckError();
    }
  }

  if (with_bias) {
    output_add_bias<__half>(output, bias, batch, channels_out, height_out, width_out, stream);
  }
}

template <>
void ModulatedDeformConvForwardCUDAKernel(
  const __half2 * input, const __half2 * weight, const __half2 * bias, const __half2 * offset,
  const __half2 * mask, __half2 * output, void * workspace, int batch, int channels, int height,
  int width, int channels_out, int kernel_w, int kernel_h, int stride_w, int stride_h, int pad_w,
  int pad_h, int dilation_w, int dilation_h, int group, int deformable_group, int im2col_step,
  cublasHandle_t cublas_handle, cudaStream_t stream)
{
  bool with_bias = (bias != nullptr);

  im2col_step = std::min(int(batch), im2col_step);
  ASSERT(batch % im2col_step == 0)

  const int height_out = (height + 2 * pad_h - (dilation_h * (kernel_h - 1) + 1)) / stride_h + 1;
  const int width_out = (width + 2 * pad_w - (dilation_w * (kernel_w - 1) + 1)) / stride_w + 1;

  auto columns = (__half2 *)workspace;

  const size_t input_step = (channels + 1) / 2 * height * width;
  const size_t offset_step = deformable_group * kernel_h * kernel_w * height_out * width_out;
  const size_t mask_step = deformable_group * kernel_h * kernel_w * height_out * width_out;
  const size_t out_step = channels_out * height_out * width_out;
  const size_t out_group_step = out_step / group;
  const size_t col_g_step =
    (channels + 1) / 2 * kernel_w * kernel_h / group * height_out * width_out;
  const size_t weight_g_step =
    channels_out / group * ((channels + 1) / 2) / group * kernel_h * kernel_w;

  const int m = channels_out / group;
  const int n = height_out * width_out;
  const int k = (channels + 1) / 2 / group * kernel_h * kernel_w * 2;
  __half alpha = __float2half(1.f);
  __half beta = __float2half(0.f);

  for (int b = 0; b < batch; b++) {
    const __half2 * input_start = input + b * input_step;
    const __half2 * offset_start = offset + b * offset_step;
    const __half * mask_start = (__half *)mask + b * mask_step;
    trt_modulated_deformable_im2col_h2(
      input_start, offset_start, mask_start, 1, channels, height, width, height_out, width_out,
      kernel_h, kernel_w, pad_h, pad_w, stride_h, stride_w, dilation_h, dilation_w,
      deformable_group, columns, stream);

    for (int g = 0; g < group; g++) {
      const __half2 * weight_start = weight + g * weight_g_step;
      __half2 * col_start = columns + g * col_g_step;
      __half * out_buffer_start = (__half *)output + b * out_step + g * out_group_step;

      cublasGemmWrap<__half>(
        cublas_handle, CUBLAS_OP_N, CUBLAS_OP_N, n, m, k, &alpha, (__half *)col_start, n,
        (__half *)weight_start, k, &beta, out_buffer_start, n);
      cudaCheckError();
    }
  }

  if (with_bias) {
    output_add_bias<__half>(
      (__half *)output, (__half *)bias, batch, channels_out, height_out, width_out, stream);
  }
}

template <typename T>
void ModulatedDeformConvForwardCUDAKernel_int8(
  const int8_4 * input, const float & scale_i, const int8_4 * weight, const float & scale_w,
  const T * bias, const int8_t * offset, const float & scale_off, const int8_t * mask,
  const float & scale_mask, int8_t * output, const float & scale_o, void * workspace, int batch,
  int channels, int height, int width, int channels_out, int kernel_w, int kernel_h, int stride_w,
  int stride_h, int pad_w, int pad_h, int dilation_w, int dilation_h, int group,
  int deformable_group, int im2col_step, cublasHandle_t cublas_handle, cudaStream_t stream)
{
  bool with_bias = (bias != nullptr);

  im2col_step = std::min(int(batch), im2col_step);
  ASSERT(batch % im2col_step == 0)

  const int height_out = (height + 2 * pad_h - (dilation_h * (kernel_h - 1) + 1)) / stride_h + 1;
  const int width_out = (width + 2 * pad_w - (dilation_w * (kernel_w - 1) + 1)) / stride_w + 1;
  const int hw4 = (height_out * width_out + 3) / 4 * 4;
  const int hw_out = height_out * width_out;

  auto columns = (int8_4 *)workspace;
  auto int32_temp = (int32_t *)(columns + (channels + 3) / 4 * kernel_w * kernel_h * hw4);

  const size_t input_step = (channels + 3) / 4 * height * width;
  const size_t offset_step = deformable_group * kernel_h * kernel_w * height_out * width_out * 2;
  const size_t mask_step = deformable_group * kernel_h * kernel_w * height_out * width_out;
  const size_t out_step = channels_out * height_out * width_out;
  const size_t out_group_step = out_step / group;
  const size_t col_g_step = (channels + 3) / 4 * kernel_w * kernel_h / group * hw4;
  const size_t weight_g_step =
    channels_out / group * ((channels + 3) / 4) / group * kernel_h * kernel_w;

  const int m = channels_out / group;
  const int n = hw4;
  const int k = (channels + 3) / 4 / group * kernel_h * kernel_w * 4;
  int32_t alpha = 1, beta = 0;
  const float scale_iw = scale_i * scale_w;
  const int output_kernel_count = channels_out / group * height_out * width_out;

  for (int b = 0; b < batch; b++) {
    const int8_4 * input_start = input + b * input_step;
    const int8_t * offset_start = offset + b * offset_step;
    const int8_t * mask_start = mask + b * mask_step;
    trt_modulated_deformable_im2col_int8(
      input_start, scale_i, offset_start, scale_off, mask_start, scale_mask, 1, channels, height,
      width, height_out, width_out, kernel_h, kernel_w, pad_h, pad_w, stride_h, stride_w,
      dilation_h, dilation_w, deformable_group, columns, stream);

    for (int g = 0; g < group; g++) {
      const int8_4 * weight_start = weight + g * weight_g_step;
      int8_4 * col_start = columns + g * col_g_step;
      int8_t * out_buffer_start = output + b * out_step + g * out_group_step;

      cublasGemmWrap_int8(
        cublas_handle, CUBLAS_OP_N, CUBLAS_OP_N, n, m, k, &alpha, (int8_t *)col_start, n,
        (int8_t *)weight_start, k, &beta, int32_temp, n);

      if (with_bias) {
        const T * bias_start = bias + g * m;
        output_add_bias_kernel_int8<T>
          <<<GET_BLOCKS(output_kernel_count), THREADS_PER_BLOCK, 0, stream>>>(
            int32_temp, scale_iw, bias_start, out_buffer_start, scale_o, output_kernel_count,
            hw_out, hw4);
      } else {
        output_wo_bias_kernel_int8<<<
          GET_BLOCKS(output_kernel_count), THREADS_PER_BLOCK, 0, stream>>>(
          int32_temp, scale_iw, out_buffer_start, scale_o, output_kernel_count, hw_out, hw4);
      }

      cudaCheckError();
    }
  }
}

template void ModulatedDeformConvForwardCUDAKernel<float>(
  const float * input, const float * weight, const float * bias, const float * offset,
  const float * mask, float * output, void * workspace, int batch, int channels, int height,
  int width, int channels_out, int kernel_w, int kernel_h, int stride_w, int stride_h, int pad_w,
  int pad_h, int dilation_w, int dilation_h, int group, int deformable_group, int im2col_step,
  cublasHandle_t cublas_handle, cudaStream_t stream);

template void ModulatedDeformConvForwardCUDAKernel<__half>(
  const __half * input, const __half * weight, const __half * bias, const __half * offset,
  const __half * mask, __half * output, void * workspace, int batch, int channels, int height,
  int width, int channels_out, int kernel_w, int kernel_h, int stride_w, int stride_h, int pad_w,
  int pad_h, int dilation_w, int dilation_h, int group, int deformable_group, int im2col_step,
  cublasHandle_t cublas_handle, cudaStream_t stream);

template void ModulatedDeformConvForwardCUDAKernel<__half2>(
  const __half2 * input, const __half2 * weight, const __half2 * bias, const __half2 * offset,
  const __half2 * mask, __half2 * output, void * workspace, int batch, int channels, int height,
  int width, int channels_out, int kernel_w, int kernel_h, int stride_w, int stride_h, int pad_w,
  int pad_h, int dilation_w, int dilation_h, int group, int deformable_group, int im2col_step,
  cublasHandle_t cublas_handle, cudaStream_t stream);

template void ModulatedDeformConvForwardCUDAKernel_int8<float>(
  const int8_4 * input, const float & scale_i, const int8_4 * weight, const float & scale_w,
  const float * bias, const int8_t * offset, const float & scale_off, const int8_t * mask,
  const float & scale_mask, int8_t * output, const float & scale_o, void * workspace, int batch,
  int channels, int height, int width, int channels_out, int kernel_w, int kernel_h, int stride_w,
  int stride_h, int pad_w, int pad_h, int dilation_w, int dilation_h, int group,
  int deformable_group, int im2col_step, cublasHandle_t cublas_handle, cudaStream_t stream);

template void ModulatedDeformConvForwardCUDAKernel_int8<__half>(
  const int8_4 * input, const float & scale_i, const int8_4 * weight, const float & scale_w,
  const __half * bias, const int8_t * offset, const float & scale_off, const int8_t * mask,
  const float & scale_mask, int8_t * output, const float & scale_o, void * workspace, int batch,
  int channels, int height, int width, int channels_out, int kernel_w, int kernel_h, int stride_w,
  int stride_h, int pad_w, int pad_h, int dilation_w, int dilation_h, int group,
  int deformable_group, int im2col_step, cublasHandle_t cublas_handle, cudaStream_t stream);
