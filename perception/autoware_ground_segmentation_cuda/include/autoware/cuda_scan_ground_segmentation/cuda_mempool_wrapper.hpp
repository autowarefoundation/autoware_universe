#ifndef AUTOWARE_CUDA_MEMPOOL_HPP_
#define AUTOWARE_CUDA_MEMPOOL_HPP_

#include <cuda_runtime.h>
#include <autoware/cuda_utils/cuda_check_error.hpp>

namespace autoware
{

class CudaMempool
{
public:
    explicit CudaMempool(int device_id = 0, int64_t pool_release_threshold = 0);
    CudaMempool(const CudaMempool&) = delete;
    CudaMempool(CudaMempool&& other) : pool_(std::move(other.pool_)) 
    {
        other.pool_ = nullptr;
    }

    CudaMempool& operator=(const CudaMempool&) = delete;
    CudaMempool& operator=(CudaMempool&& other);

    cudaMempool_t get() { return pool_; }

    ~CudaMempool();

private:
    void release();

    cudaMempool_t pool_;
};

CudaMempool::CudaMempool(int device_id, int64_t pool_release_threshold)
{
    cudaMemPoolProps props, pool_props;

    // Check if the device is valid
    CHECK_CUDA_ERROR(cudaGetDeviceProperties(&props, device_id));
    pool_props.allocType = cudaMemAllocationTypePinned;
    pool_props.location.id = device_id;
    pool_props.location.type = cudaMemLocationTypeDevice;

    CHECK_CUDA_ERROR(cudaMemPoolCreate(&pool_, &pool_props));    

    if (pool_release_threshold >= 0) {
        CHECK_CUDA_ERROR(cudaMemPoolSetAttribute(
            pool_, cudaMemPoolAttrReleaseThreshold, &pool_release_threshold
        ));
    }
}

CudaMempool& CudaMempool::operator=(CudaMempool&& other)
{
    if (this != &other) {
        release();
        pool_ = std::move(other.pool_);
        other.pool_ = nullptr;
    }

    return *this;
}

CudaMempool::~CudaMempool()
{
    release();
}

void CudaMempoo::release()
{
    if (pool_) {
        CHECK_CUDA_ERROR(cudaMemPoolDestroy(pool_));
    }

    pool_ = nullptr;
}

}

#endif