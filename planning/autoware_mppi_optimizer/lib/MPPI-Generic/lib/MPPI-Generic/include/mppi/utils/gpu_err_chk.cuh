#ifndef CUDA_TESTING_GPU_ERR_CHK_CUH
#define CUDA_TESTING_GPU_ERR_CHK_CUH

#include <cuda_runtime.h>
#include <cufft.h>
#include <curand.h>
#include <device_launch_parameters.h>  // For block idx and thread idx, etc
#include <iostream>

#ifndef DEPRECATED
#if __cplusplus >= 201402L
#define DEPRECATED [[deprecated]]
#elif defined(__GNUC__) || defined(__clang__) || defined(__CUDACC__)
#define DEPRECATED __attribute__((deprecated))
#elif defined(_MSC_VER)
#define DEPRECATED __declspec(deprecated)
#else
#pragma message("WARNING: You need to implement DEPRECATED for this compiler")
#define DEPRECATED
#endif
#endif
// #ifndef __DEPRECATED__
// #if defined(_WIN32)
// # define __DEPRECATED__(msg) __declspec(deprecated(msg))
// #elif (defined(__GNUC__) && (__GNUC__ < 4 || (__GNUC__ == 4 && __GNUC_MINOR__ < 5 && !defined(__clang__))))
// # define __DEPRECATED__(msg) __attribute__((deprecated))
// #else
// # define __DEPRECATED__(msg) __attribute__((deprecated(msg)))
// #endif
// #endif

inline void gpuAssert(cudaError_t code, const char* file, int line, bool abort = true)
{
  if (code != cudaSuccess)
  {
    fprintf(stderr, "GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
    if (abort)
      exit(code);
  }
}

inline void __cudaCheckError(const char* file, const int line)
{
  cudaError err = cudaGetLastError();
  if (cudaSuccess != err)
  {
    fprintf(stderr, "cudaCheckError() failed at %s:%i : %s\n", file, line, cudaGetErrorString(err));
    exit(-1);
  }

  // More careful checking. However, this will affect performance.
  // Comment away if needed.
  err = cudaDeviceSynchronize();
  if (cudaSuccess != err)
  {
    fprintf(stderr, "cudaCheckError() with sync failed at %s:%i : %s\n", file, line, cudaGetErrorString(err));
    exit(-1);
  }
}

inline const char* cufftGetErrorString(cufftResult& code)
{
  // Codes from https://docs.nvidia.com/cuda/cufft/index.html#cufftresult
  switch (code)
  {
    case CUFFT_SUCCESS:
      return "Success";
    case CUFFT_INVALID_PLAN:
      return "cuFFT was passed an invalid plan handle";
    case CUFFT_ALLOC_FAILED:
      return "cuFFT failed to allocate GPU or CPU memory";
    case CUFFT_INVALID_VALUE:
      return "User specified an invalid pointer or parameter";
    case CUFFT_INTERNAL_ERROR:
      return "Driver or internal cuFFT library error";
    case CUFFT_EXEC_FAILED:
      return "Failed to execute an FFT on the GPU";
    case CUFFT_SETUP_FAILED:
      return "The cuFFT library failed to initialize";
    case CUFFT_INVALID_SIZE:
      return "User specified an invalid transform size";
    case CUFFT_UNALIGNED_DATA:
      return "No longer used but unaligned data";
    case CUFFT_INVALID_DEVICE:
      return "Execution of a plan was on different GPU than plan creation";
    case CUFFT_NO_WORKSPACE:
      return "No workspace has been provided prior to plan execution";
    case CUFFT_NOT_IMPLEMENTED:
      return "Function does not implement functionality for parameters given.";
#if CUDART_VERSION < 13000
    case CUFFT_INCOMPLETE_PARAMETER_LIST:
      return "Missing parameters in call";
    case CUFFT_PARSE_ERROR:
      return "Internal plan database error";
    case CUFFT_LICENSE_ERROR:
      return "License Error. Used in previous versions";
#else
    case CUFFT_MISSING_DEPENDENCY:
      return "cuFFT is unable to find a dependency";
    case CUFFT_NVRTC_FAILURE:
      return "An NVRTC failure was encountered during a cuFFT operation";
    case CUFFT_NVJITLINK_FAILURE:
      return "An nvJitLink failure was encountered during a cuFFT operation";
    case CUFFT_NVSHMEM_FAILURE:
      return "An NVSHMEM failure was encountered during a cuFFT operation";
#endif
    case CUFFT_NOT_SUPPORTED:
      return "Operation is not supported for parameters given.";
    default:
      return "cuFFT ERROR";
  }
}

inline const char* curandGetErrorString(curandStatus_t code)
{
  // Codes from https://docs.nvidia.com/cuda/curand/group__HOST.html#group__HOST_1gb94a31d5c165858c96b6c18b70644437
  switch (code)
  {
    case CURAND_STATUS_SUCCESS:
      return "No errors.";
    case CURAND_STATUS_VERSION_MISMATCH:
      return "Header file and linked library version do not match.";
    case CURAND_STATUS_NOT_INITIALIZED:
      return "Generator not initialized.";
    case CURAND_STATUS_ALLOCATION_FAILED:
      return "Memory allocation failed.";
    case CURAND_STATUS_TYPE_ERROR:
      return "Generator is wrong type.";
    case CURAND_STATUS_OUT_OF_RANGE:
      return "Argument out of range.";
    case CURAND_STATUS_LENGTH_NOT_MULTIPLE:
      return "Length requested is not a multple of dimension.";
    case CURAND_STATUS_DOUBLE_PRECISION_REQUIRED:
      return "GPU does not have double precision required by MRG32k3a.";
    case CURAND_STATUS_LAUNCH_FAILURE:
      return "Kernel launch failure.";
    case CURAND_STATUS_PREEXISTING_FAILURE:
      return "Preexisting failure on library entry.";
    case CURAND_STATUS_INITIALIZATION_FAILED:
      return "Initialization of CUDA failed.";
    case CURAND_STATUS_ARCH_MISMATCH:
      return "Architecture mismatch, GPU does not support requested feature.";
    case CURAND_STATUS_INTERNAL_ERROR:
      return "Internal library error.";
    default:
      return "Cureand Error";
  }
}

inline void cufftAssert(cufftResult code, const char* file, int line, bool abort = true)
{
  if (code != CUFFT_SUCCESS)
  {
    fprintf(stderr, "CUFFTassert: %s %s %d\n", cufftGetErrorString(code), file, line);
    if (abort)
    {
      exit(code);
    }
  }
}

inline void curandAssert(curandStatus_t code, const char* file, int line, bool abort = true)
{
  if (code != CURAND_STATUS_SUCCESS)
  {
    fprintf(stderr, "Curandassert: %s %s %d\n", curandGetErrorString(code), file, line);
    if (abort)
    {
      exit(code);
    }
  }
}

#define CudaCheckError() __cudaCheckError(__FILE__, __LINE__)
#define HANDLE_ERROR(ans)                                                                                              \
  {                                                                                                                    \
    gpuAssert((ans), __FILE__, __LINE__);                                                                              \
  }

#define HANDLE_CUFFT_ERROR(ans)                                                                                        \
  {                                                                                                                    \
    cufftAssert((ans), __FILE__, __LINE__);                                                                            \
  }

#define HANDLE_CURAND_ERROR(ans)                                                                                       \
  {                                                                                                                    \
    curandAssert((ans), __FILE__, __LINE__);                                                                           \
  }

#endif  //! CUDA_TESTING_GPU_ERR_CHK_CUH
