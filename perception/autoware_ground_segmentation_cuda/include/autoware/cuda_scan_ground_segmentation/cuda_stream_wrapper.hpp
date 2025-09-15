#ifndef AUTOWARE__CUDA_SCAN_GROUND_SEGMENTATION__CUDA_STREAM_WRAPPER_HPP_
#define AUTOWARE__CUDA_SCAN_GROUND_SEGMENTATION__CUDA_STREAM_WRAPPER_HPP_

#include <cuda_runtime_api.h>
#include <memory>

#include <autoware/cuda_utils/cuda_check_error.hpp>

namespace autoware 
{

// A wrapper on the cudaStream_t, with a destructor to 
class CudaStream
{
public:
    inline CudaStream(bool is_null = false) {
        // If is_null is true, we use the default stream
        // Otherwise, we create a new stream
        if (!is_null) {
            CHECK_CUDA_ERROR(cudaStreamCreate(&stream_));
        }
    }

    CudaStream(const CudaStream&) = delete;
    CudaStream(CudaStream&& other) : stream_(std::move(other.stream_)) {
        other.stream_ = nullptr;
    }

    CudaStream& operator=(const CudaStream&) = delete;
    inline CudaStream& operator=(CudaStream&& other) {
        if (this != &other) {
            release();
            stream_ = std::move(other.stream_);
            other.stream_ = nullptr;
        }

        return *this;
    }

    inline cudaStream_t& get() {
        return stream_;
    }

    ~CudaStream() {
        release();
    }

private:
    inline void release() {
        if (stream_) {
           CHECK_CUDA_ERROR(cudaStreamDestroy(stream_));
        }
    }

    cudaStream_t stream_{nullptr};
};

}

#endif