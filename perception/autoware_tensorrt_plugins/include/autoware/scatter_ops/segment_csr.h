// Copyright (c) 2020 Matthias Fey <matthias.fey@tu-dortmund.de>
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef AUTOWARE__SCATTER_OPS__SEGMENT_CSR_H_
#define AUTOWARE__SCATTER_OPS__SEGMENT_CSR_H_

#include "autoware/scatter_ops/reduction.h"

#include <cuda_runtime.h>

#include <cstdint>
#include <vector>

template <typename scalar_t, ReductionType REDUCE>
int32_t segment_csr_launch(
  const scalar_t * src_in, const std::vector<int32_t> & src_size_in, const int64_t * indptr_in,
  const std::vector<int32_t> & indptr_size_in, const scalar_t * base_values_in,
  scalar_t * reduced_values_out, int64_t * arg_indices_out, cudaStream_t stream_in);

template <typename scalar_t, ReductionType REDUCE>
int32_t segment_csr_launch(
  const scalar_t * src_in, const std::vector<int32_t> & src_size_in, const int64_t * indptr_in,
  const std::vector<int32_t> & indptr_size_in, scalar_t * reduced_values_out,
  int64_t * arg_indices_out, cudaStream_t stream_in);

#endif  // AUTOWARE__SCATTER_OPS__SEGMENT_CSR_H_
