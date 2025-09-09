#ifndef AUTOWARE__CUDA_UTILITIES_HPP_
#define AUTOWARE__CUDA_UTILITIES_HPP_

#include <cuda_runtime.h>

#include <cub/cub.cuh>

#include "cuda_common.hpp"
#include "device_vector.hpp"
#include "cuda_stream_wrapper.hpp"
#include "cuda_mempool_wrapper.hpp"

namespace autoware::cuda
{

template <int block_size, typename... Args>
inline cudaError_t launchAsync(int thread_num, int shared_size, cudaStream_t& stream, void (*f)(Args...), Args... args)
{
    int block_x = (thread_num > block_size) ? block_size : thread_num;

    if (block_x <= 0) {
        return cudaErrorLaunchFailure;
    }

    int grid_x = (thread_num + block_x - 1) / block_x;

    f<<<grid_x, block_x, shared_size, stream>>>(args...);

    return cudaGetLastError();
}

template <typename T>
cudaError_t ExclusiveScan(
    T * input, 
    T * output,
    int ele_num, 
    std::shared_ptr<CudaStream> stream,
    std::shared_ptr<CudaMempool> mempool
)
{
    if (ele_num == 0) {
        return cudaSuccess;
    }

    if (ele_num < 0 || !stream) {
        return cudaErrorInvalidValue;
    }


    device_vector<int> d_temp_storage(stream, mempool);
    size_t temp_storage_bytes = 0;

    cub::DeviceScan::ExclusiveSum(
        (void*)(d_temp_storage.data()),
        temp_storage_bytes,
        input, output, ele_num,
        stream->get()
    );

    int ele_num = (temp_storage_bytes + sizeof(int) - 1) / sizeof(int);
    d_temp_storage.resize(ele_num);

    cub::DeviceScan::ExclusiveSum(
        (void*)(d_temp_storage.data()),
        temp_storage_bytes,
        input, output, ele_num,
        stream->get()
    );

    return cudaGetLastError();
} 

template <typename T>
cudaError_t ExclusiveScan(
    device_vector<T> & input,
    device_vector<T> & output
)
{
    if (input.empty()) {
        return cudaSuccess;
    }

    output.resize(input.size());

    return ExclusiveScan(
        input.data(), 
        output.data(), 
        (int)(input.size()), 
        input.get_stream(),
        input.get_mempool()
    );
}

template <typename T>
__global__ void fillVector(T* vec, int ele_num, T init_val)
{
  int index = threadIdx.x + blockIdx.x * blockDim.x;
  int stride = blockDim.x * gridDim.x;

  for (int i = index; i < ele_num; i += stride)
  {
    vec[i] = init_val;
  }
}

template <typename T>
cudaError_t fill(T * input, int ele_num, T val, 
                    std::shared_ptr<CudaStream> stream)
{
    if (ele_num == 0) {
        return cudaSuccess;
    }

    if (ele_num < 0 || !stream) {
        return cudaErrorInvalidValue;
    }

    return launchAsync<BLOCK_SIZE_X>(
        ele_num, 0, stream->get(), 
        fillVector, 
        input, ele_num, val
    );
}

template <typename T>
cudaError_t fill(device_vector<T> & input, T val)
{
    return fill(input.data(), (int)(input.size()), val, input.get_stream());
}


}

#endif