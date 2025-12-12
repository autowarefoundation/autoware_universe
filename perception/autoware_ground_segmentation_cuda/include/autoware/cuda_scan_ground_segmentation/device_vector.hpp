// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE__CUDA_SCAN_GROUND_SEGMENTATION__DEVICE_VECTOR_HPP_
#define AUTOWARE__CUDA_SCAN_GROUND_SEGMENTATION__DEVICE_VECTOR_HPP_

#include "cuda_mempool_wrapper.hpp"
#include "cuda_stream_wrapper.hpp"

#include <autoware/cuda_utils/cuda_check_error.hpp>

#include <cuda_runtime_api.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware
{

template <typename T>
class device_vector
{
public:
  device_vector(
    std::shared_ptr<CudaStream> stream = std::make_shared<CudaStream>(true),
    std::shared_ptr<CudaMempool> mem_pool = nullptr);

  device_vector(
    const device_vector & other,
    std::shared_ptr<CudaStream> stream = std::make_shared<CudaStream>(true),
    std::shared_ptr<CudaMempool> mem_pool = nullptr,
    bool copy_all = false  // If true, use the stream and mem_pool of the other
  );

  device_vector(device_vector && other);

  device_vector(
    size_t ele_num, std::shared_ptr<CudaStream> stream = std::make_shared<CudaStream>(true),
    std::shared_ptr<CudaMempool> mem_pool = nullptr);

  device_vector(
    const std::vector<T> & other,
    std::shared_ptr<CudaStream> stream = std::make_shared<CudaStream>(true),
    std::shared_ptr<CudaMempool> mem_pool = nullptr);

  device_vector & operator=(const device_vector & other);
  device_vector & operator=(device_vector && other);
  device_vector & operator=(const std::vector<T> & other);

  void to_vector(std::vector<T> & output) const
  {
    output.resize(ele_num_);
    copyDtoH(output.data(), data_, ele_num_);
  }

  // Slow, do not use unless necessary
  T operator[](int idx) const
  {
    if (idx < 0 || idx >= ele_num_) {
      throw std::invalid_argument("Error: out-of-bound access at index " + std::to_string(idx));
    }
    T val;

    copyDtoH(&val, data_ + idx, 1);

    CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_->get()));

    return val;
  }

  T * data() { return data_; }
  const T * data() const { return data_; }
  size_t size() const { return ele_num_; }

  void resize(size_t new_size);
  void reserve(size_t new_size);
  bool empty() const { return (ele_num_ == 0); }
  void clear();

  std::shared_ptr<CudaStream> get_stream() { return stream_; }
  std::shared_ptr<CudaMempool> get_mempool() { return mempool_; }

  ~device_vector();

private:
  // Return false if allocation fail
  void allocate(size_t ele_num);
  void release();
  void copyDtoD(T * dst, const T * src, size_t ele_num);
  void copyHtoD(T * dst, const T * src, size_t ele_num);
  void copyDtoH(T * dst, const T * src, size_t ele_num) const;

  T * data_;
  size_t ele_num_;                        // The assumed size
  size_t actual_ele_num_;                 // The actual size of the vector
  std::shared_ptr<CudaStream> stream_;    // If null, use the default stream
  std::shared_ptr<CudaMempool> mempool_;  // If null, use the default non-pool allocation
};

template <typename T>
device_vector<T>::device_vector(
  std::shared_ptr<CudaStream> stream, std::shared_ptr<CudaMempool> mem_pool)
: stream_(stream), mempool_(mem_pool)
{
  data_ = nullptr;
  ele_num_ = actual_ele_num_ = 0;
}

template <typename T>
device_vector<T>::device_vector(
  const device_vector & other, std::shared_ptr<CudaStream> stream,
  std::shared_ptr<CudaMempool> mem_pool, bool copy_all)
: stream_(stream), mempool_(mem_pool), data_(nullptr)
{
  ele_num_ = actual_ele_num_ = 0;

  if (copy_all) {
    stream_ = other.stream_;
    mempool_ = other.mempool_;
  }

  if (other.empty()) {
    return;
  }

  // Allocate memory
  resize(other.size());

  // Copy
  copyDtoD(data_, other.data_, other.size());
}

template <typename T>
device_vector<T>::device_vector(device_vector && other)
{
  data_ = other.data_;
  ele_num_ = other.ele_num_;
  actual_ele_num_ = other.actual_ele_num_;
  stream_ = other.stream_;
  mempool_ = other.mempool_;

  other.data_ = nullptr;
  other.ele_num_ = other.actual_ele_num_ = 0;
  other.stream_.reset();
  other.mempool_.reset();
}

template <typename T>
device_vector<T>::device_vector(
  size_t ele_num, std::shared_ptr<CudaStream> stream, std::shared_ptr<CudaMempool> mem_pool)
: stream_(stream), mempool_(mem_pool), data_(nullptr)
{
  ele_num_ = actual_ele_num_ = 0;

  if (ele_num > 0) {
    resize(ele_num);
  }
}

template <typename T>
device_vector<T>::device_vector(
  const std::vector<T> & other, std::shared_ptr<CudaStream> stream,
  std::shared_ptr<CudaMempool> mem_pool)
: stream_(stream), mempool_(mem_pool), data_(nullptr)
{
  ele_num_ = actual_ele_num_ = 0;

  if (other.empty()) {
    return;
  }

  allocate(other.size());
  copyHtoD(data_, other.data(), other.size());
  ele_num_ = other.size();
}

template <typename T>
device_vector<T> & device_vector<T>::operator=(const device_vector & other)
{
  size_t other_size = other.size();

  resize(other_size);

  if (other_size == 0) {
    copyDtoD(data_, other.data_, other_size);
  }

  return *this;
}

template <typename T>
device_vector<T> & device_vector<T>::operator=(device_vector && other)
{
  release();

  data_ = other.data_;
  ele_num_ = other.ele_num_;
  actual_ele_num_ = other.actual_ele_num_;
  stream_ = other.stream_;
  mempool_ = other.mempool_;

  // TODO: is stream_ = other.stream_ necessary?
  other.data_ = nullptr;
  other.ele_num_ = other.actual_ele_num_ = 0;
  other.mempool_.reset();
  other.stream_.reset();

  return *this;
}

template <typename T>
device_vector<T> & device_vector<T>::operator=(const std::vector<T> & other)
{
  release();

  if (other.empty()) {
    return *this;
  }

  copyHtoD(data_, other.data(), sizeof(T) * other.size());

  return *this;
}

template <typename T>
void device_vector<T>::resize(size_t new_ele_num)
{
  // If no significant change, just change the number of elements
  if (new_ele_num >= actual_ele_num_ * 0.8 && new_ele_num <= actual_ele_num_) {
    ele_num_ = new_ele_num;
    return;
  }

  // Otherwise, reallocate the data
  release();
  allocate(new_ele_num);
  ele_num_ = new_ele_num;
}

template <typename T>
void device_vector<T>::reserve(size_t new_ele_num)
{
  if (new_ele_num <= actual_ele_num_ * 0.8) {
    return;
  }

  allocate(new_ele_num);
}

template <typename T>
void device_vector<T>::clear()
{
  // Release the memory only
  release();
}

template <typename T>
device_vector<T>::~device_vector()
{
  // Release the memory
  release();
  // Release the stream and memory pool
  stream_.reset();
  mempool_.reset();
}

template <typename T>
void device_vector<T>::allocate(size_t ele_num)
{
  if (ele_num > 0) {
    if (mempool_) {
      CHECK_CUDA_ERROR(
        cudaMallocFromPoolAsync(&data_, sizeof(T) * ele_num, mempool_->get(), stream_->get()));
      CHECK_CUDA_ERROR(cudaMemsetAsync(data_, 0, sizeof(T) * ele_num, stream_->get()));
    } else {
      CHECK_CUDA_ERROR(cudaMallocAsync(&data_, sizeof(T) * ele_num, stream_->get()));
      CHECK_CUDA_ERROR(cudaMemsetAsync(data_, 0, sizeof(T) * ele_num, stream_->get()));
    }
  }

  actual_ele_num_ = ele_num;
  // Currently no element yet
  ele_num_ = 0;
}

template <typename T>
void device_vector<T>::release()
{
  if (data_) {
    CHECK_CUDA_ERROR(cudaFreeAsync(data_, stream_->get()));
  }

  data_ = nullptr;
  ele_num_ = 0;
  actual_ele_num_ = 0;
}

template <typename T>
void device_vector<T>::copyDtoD(T * dst, const T * src, size_t ele_num)
{
  CHECK_CUDA_ERROR(
    cudaMemcpyAsync(dst, src, sizeof(T) * ele_num, cudaMemcpyDeviceToDevice, stream_->get()));
}

template <typename T>
void device_vector<T>::copyHtoD(T * dst, const T * src, size_t ele_num)
{
  CHECK_CUDA_ERROR(
    cudaMemcpyAsync(dst, src, sizeof(T) * ele_num, cudaMemcpyHostToDevice, stream_->get()));
}

template <typename T>
void device_vector<T>::copyDtoH(T * dst, const T * src, size_t ele_num) const
{
  CHECK_CUDA_ERROR(
    cudaMemcpyAsync(dst, src, sizeof(T) * ele_num, cudaMemcpyDeviceToHost, stream_->get()));
}

}  // namespace autoware

#endif  // AUTOWARE__CUDA_SCAN_GROUND_SEGMENTATION__DEVICE_VECTOR_HPP_
