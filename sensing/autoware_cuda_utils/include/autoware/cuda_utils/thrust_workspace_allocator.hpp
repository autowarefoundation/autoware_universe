// Copyright 2026 Tier IV, Inc.
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

#ifndef AUTOWARE__CUDA_UTILS__THRUST_WORKSPACE_ALLOCATOR_HPP_
#define AUTOWARE__CUDA_UTILS__THRUST_WORKSPACE_ALLOCATOR_HPP_

#include "autoware/cuda_utils/cuda_check_error.hpp"

#include <cuda_runtime_api.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>

namespace autoware::cuda_utils
{

/// \brief Pre-allocated device-memory workspace that backs Thrust temporary
/// allocations, so Thrust algorithms incur no per-call device allocation.
///
/// Thrust algorithms (sort, reduce/count, scan, copy_if, ...) need scratch
/// memory. With Thrust's default device allocator they obtain it via plain
/// \c cudaMalloc / \c cudaFree, each of which is a *process-wide* device
/// synchronization. In a process that hosts several GPU workloads (e.g. one ROS
/// component container running multiple CUDA nodes) those barriers serialize
/// otherwise-independent streams.
///
/// \c ThrustWorkspace hands out slices of a single device buffer with a bump
/// pointer and resets once every outstanding allocation has been returned —
/// which is always the case between Thrust algorithm invocations. In steady
/// state it therefore performs no device allocation, letting Thrust run free of
/// the process-wide barriers. Use it together with the node's own stream:
///
/// \code
///   ThrustWorkspace ws{1 << 20};  // construct once (e.g. a node member)
///   const auto policy = thrust::cuda::par(ws.allocator()).on(stream);
///   thrust::sort(policy, first, last);
///   const auto n = thrust::count_if(policy, first, last, pred);
/// \endcode
///
/// The buffer grows (once) if an algorithm ever needs more than the current
/// capacity, converging to a fixed footprint after the largest workload has
/// been seen; pass a sufficient \p initial_capacity_bytes to avoid even that.
///
/// \note Not thread-safe. Use one workspace per stream/thread (the GPU work it
/// backs runs serially on a single stream anyway).
class ThrustWorkspace
{
public:
  explicit ThrustWorkspace(std::size_t initial_capacity_bytes = 0)
  {
    if (initial_capacity_bytes > 0) {
      grow(initial_capacity_bytes);
    }
  }

  ~ThrustWorkspace()
  {
    if (buffer_ != nullptr) {
      static_cast<void>(::cudaFree(buffer_));  // best-effort; never throw from dtor
    }
  }

  ThrustWorkspace(const ThrustWorkspace &) = delete;
  ThrustWorkspace & operator=(const ThrustWorkspace &) = delete;

  /// \brief Lightweight, copyable handle that satisfies Thrust's allocator
  /// requirements (value_type == char). Pass to \c thrust::cuda::par(...).
  class Allocator
  {
  public:
    using value_type = char;

    explicit Allocator(ThrustWorkspace * workspace) : workspace_(workspace) {}

    char * allocate(std::ptrdiff_t num_bytes)
    {
      return workspace_->allocate(static_cast<std::size_t>(num_bytes));
    }

    void deallocate(char * ptr, std::size_t /*num_bytes*/) { workspace_->deallocate(ptr); }

  private:
    ThrustWorkspace * workspace_;
  };

  [[nodiscard]] Allocator allocator() { return Allocator{this}; }

  [[nodiscard]] std::size_t capacity_bytes() const { return capacity_; }
  [[nodiscard]] std::size_t peak_bytes() const { return peak_; }
  [[nodiscard]] std::size_t fallback_count() const { return fallback_count_; }

private:
  static constexpr std::size_t kAlignment = 256;

  static std::size_t align_up(std::size_t value, std::size_t alignment)
  {
    return (value + alignment - 1) & ~(alignment - 1);
  }

  char * allocate(std::size_t num_bytes)
  {
    const std::size_t offset = align_up(offset_, kAlignment);
    const std::size_t end = offset + num_bytes;

    if (end <= capacity_) {  // fast path: carve a slice out of the buffer
      char * ptr = buffer_ + offset;
      offset_ = end;
      peak_ = std::max(peak_, offset_);
      ++live_;
      return ptr;
    }

    if (live_ == 0) {  // nothing outstanding: safe to (re)allocate a larger buffer
      grow(std::max(end, capacity_ * 2));
      char * ptr = buffer_;  // offset_ was reset to 0 by grow()
      offset_ = num_bytes;
      peak_ = std::max(peak_, offset_);
      ++live_;
      return ptr;
    }

    // Mid-algorithm overflow (rare): satisfy this one block directly and grow on
    // the next reset so steady state stops hitting this path.
    pending_capacity_ = std::max(pending_capacity_, end);
    char * ptr = nullptr;
    CHECK_CUDA_ERROR(::cudaMalloc(reinterpret_cast<void **>(&ptr), num_bytes));
    ++live_;
    ++fallback_count_;
    return ptr;
  }

  void deallocate(char * ptr)
  {
    --live_;
    const bool from_buffer = (ptr >= buffer_) && (ptr < buffer_ + capacity_);
    if (!from_buffer) {
      static_cast<void>(::cudaFree(ptr));  // a fallback block
    }
    if (live_ == 0) {
      offset_ = 0;
      if (pending_capacity_ > capacity_) {
        grow(pending_capacity_);
        pending_capacity_ = 0;
      }
    }
  }

  void grow(std::size_t bytes)
  {
    if (buffer_ != nullptr) {
      static_cast<void>(::cudaFree(buffer_));
    }
    CHECK_CUDA_ERROR(::cudaMalloc(reinterpret_cast<void **>(&buffer_), bytes));
    capacity_ = bytes;
    offset_ = 0;
  }

  char * buffer_{nullptr};
  std::size_t capacity_{0};
  std::size_t offset_{0};
  std::size_t peak_{0};
  std::size_t pending_capacity_{0};
  std::int64_t live_{0};
  std::size_t fallback_count_{0};
};

}  // namespace autoware::cuda_utils

#endif  // AUTOWARE__CUDA_UTILS__THRUST_WORKSPACE_ALLOCATOR_HPP_
