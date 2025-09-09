#ifndef AUTOWARE__CUDA_SCAN_GROUND_SEGMENTATION__CUDA_PTR_HPP
#define AUTOWARE__CUDA_SCAN_GROUND_SEGMENTATION__CUDA_PTR_HPP

#include <cuda_runtime_api.h>
#include <memory>

#include <autoware/cuda_utils/cuda_check_error.hpp>

namespace autoware 
{

// A wrapper on the cudaStream_t, with a destructor to 
class CudaStream
{
public:
    CudaStream(bool is_null = false) {
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
    CudaStream& operator=(CudaStream&& other);

    cudaStream_t get() {
        return stream_;
    }

    ~CudaStream();

private:
    void release();

    cudaStream_t stream_{nullptr};
};

CudaStream::CudaStream()
{
    CHECK_CUDA_ERROR(cudaStreamCreate(&stream_));
}

CudaStream& CudaStream::operator=(CudaStream&& other)
{
    if (this != &other) {
        release();
        stream_ = std::move(other.stream_);
        other.stream_ = nullptr;
    }

    return *this;
}

void CudaStream::release()
{
    if (stream_) {
        CHECK_CUDA_ERROR(cudaStreamDestroy(stream_));
    }
}

CudaStream::~CudaStream()
{
    release();
}

}

#endif