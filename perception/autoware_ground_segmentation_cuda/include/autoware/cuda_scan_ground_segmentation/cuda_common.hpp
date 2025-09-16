#ifndef AUTOWARE__CUDA_SCAN_GROUND_SEGMENTATION__CUDA_COMMON_HPP_
#define AUTOWARE__CUDA_SCAN_GROUND_SEGMENTATION__CUDA_COMMON_HPP_

#include <autoware/cuda_utils/cuda_check_error.hpp>

#include <cuda_runtime.h>

#ifndef CUDAH
#define CUDAH __forceinline__ __host__ __device__
#endif

#ifndef BLOCK_SIZE_X
#define BLOCK_SIZE_X (256)
#endif

#ifndef WARP_SIZE
#define WARP_SIZE (32)
#endif

#ifndef FULL_MASK
#define FULL_MASK (0xFFFFFFFF)
#endif

#endif  // AUTOWARE__CUDA_SCAN_GROUND_SEGMENTATION__CUDA_COMMON_HPP_
