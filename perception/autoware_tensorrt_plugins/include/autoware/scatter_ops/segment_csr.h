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

/// Reduces a row-major `src` matrix along contiguous CSR segments on `stream_in`.
///
/// Each output segment `s` is the `REDUCE` of the rows
/// `src_in[indptr_in[s] : indptr_in[s + 1]]`, applied per column.
///
/// Input contract:
/// - `src_in` points to `num_rows_in * num_cols_in` `scalar_t` values, row-major.
/// - `num_rows_in`, `num_cols_in`, and `indptr_size_in` must be non-negative.
/// - `indptr_in` points to `indptr_size_in` int64 CSR row pointers in non-decreasing order
///   with `0 <= indptr_in[i] <= num_rows_in`. The segment count is
///   `num_segments = max(indptr_size_in - 1, 0)`; when `indptr_size_in == 0`, `indptr_in` is not
///   read and no output segments are produced.
/// - `reduced_values_out` points to `num_segments * num_cols_in` `scalar_t` values.
/// - `arg_indices_out` is only used for `REDUCE == MIN` or `REDUCE == MAX`. When non-null for those
///   reductions it points to `num_segments * num_cols_in` int64 values. Pass `nullptr` for SUM,
///   MEAN, MUL, and DIV, or when argmin/argmax indices are not needed.
/// - All pointers are device pointers.
///
/// Output contract:
/// - Returns `0` if all work was enqueued successfully, or `-1` if any size argument is negative.
/// - `reduced_values_out[s * num_cols_in + c]` contains the reduction of column `c` of
///   `src_in[indptr_in[s] : indptr_in[s + 1]]`. Empty segments produce the reducer's empty value:
///   `0` for SUM, MEAN, MIN, and MAX, and `1` for MUL and DIV.
/// - When written, `arg_indices_out[s * num_cols_in + c]` contains the source row index that
///   produced the min/max for that output cell. Cells corresponding to empty segments are left at
///   the sentinel value `num_rows_in` (an out-of-range index).
template <typename scalar_t, ReductionType REDUCE>
int32_t segment_csr_launch(
  const scalar_t * src_in, int32_t num_rows_in, int32_t num_cols_in, const int64_t * indptr_in,
  int32_t indptr_size_in, scalar_t * reduced_values_out, int64_t * arg_indices_out,
  cudaStream_t stream_in);

#endif  // AUTOWARE__SCATTER_OPS__SEGMENT_CSR_H_
