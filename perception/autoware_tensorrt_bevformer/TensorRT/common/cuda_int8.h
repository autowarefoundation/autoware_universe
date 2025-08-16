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

// cspell:ignore BEVFORMER

#ifndef PERCEPTION__AUTOWARE_TENSORRT_BEVFORMER__TENSORRT__COMMON__CUDA_INT8_H_
#define PERCEPTION__AUTOWARE_TENSORRT_BEVFORMER__TENSORRT__COMMON__CUDA_INT8_H_

#include "cuda_helper.h"

#include <cuda_fp16.h>

struct int8_4
{
  int8_t x = 0, y = 0, z = 0, w = 0;
  __device__ int8_4() {}
  __device__ int8_4(const int8_t & val) : x(val), y(val), z(val), w(val) {}
  __device__ int8_4(const int8_t & x, const int8_t & y, const int8_t & z, const int8_t & w)
  : x(x), y(y), z(z), w(w)
  {
  }
};

struct uint8_4
{
  uint8_t x = 0, y = 0, z = 0, w = 0;
  __device__ uint8_4() {}
  __device__ uint8_4(const uint8_t & val) : x(val), y(val), z(val), w(val) {}
  __device__ uint8_4(const uint8_t & x, const uint8_t & y, const uint8_t & z, const uint8_t & w)
  : x(x), y(y), z(z), w(w)
  {
  }
};

struct int8_2
{
  int8_t x = 0, y = 0;
  __device__ int8_2() {}
  __device__ int8_2(const int8_t & val) : x(val), y(val) {}
  __device__ int8_2(const int8_t & x, const int8_t & y) : x(x), y(y) {}
};

struct uint8_2
{
  uint8_t x = 0, y = 0;
  __device__ uint8_2() {}
  __device__ uint8_2(const uint8_t & val) : x(val), y(val) {}
  __device__ uint8_2(const uint8_t & x, const uint8_t & y) : x(x), y(y) {}
};

struct int32_4
{
  int32_t x = 0, y = 0, z = 0, w = 0;
  __device__ int32_4() {}
  __device__ int32_4(const int32_t & val) : x(val), y(val), z(val), w(val) {}
};

struct float_4
{
  float x = 0, y = 0, z = 0, w = 0;
  __device__ float_4() {}
  __device__ float_4(const float & val) : x(val), y(val), z(val), w(val) {}
};

typedef struct int8_4 int8_4;
typedef struct int32_4 int32_4;

#endif  // PERCEPTION__AUTOWARE_TENSORRT_BEVFORMER__TENSORRT__COMMON__CUDA_INT8_H_
