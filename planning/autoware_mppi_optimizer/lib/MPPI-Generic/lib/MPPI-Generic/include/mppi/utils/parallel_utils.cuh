//
// Created by Bogdan on 8/20/23.
//
#pragma once
#include <mppi/utils/cuda_math_utils.cuh>

namespace mppi
{
namespace p1  // parallelize to 1 index and step
{
enum class Parallel1Dir : int
{
  THREAD_X = 0,
  THREAD_Y,
  THREAD_Z,
  THREAD_XY,
  THREAD_YX,
  THREAD_XZ,
  THREAD_ZX,
  THREAD_YZ,
  THREAD_ZY,
  THREAD_XYZ,
  GLOBAL_X,
  GLOBAL_Y,
  GLOBAL_Z,
  NONE,
};

template <Parallel1Dir P_DIR>
inline __host__ __device__ void getParallel1DIndex(int& p_index, int& p_step);

template <>
inline __host__ __device__ void getParallel1DIndex<Parallel1Dir::THREAD_X>(int& p_index, int& p_step)
{
#ifdef __CUDA_ARCH__
  p_index = threadIdx.x;
  p_step = blockDim.x;
#else
  p_index = 0;
  p_step = 1;
#endif
}

template <>
inline __host__ __device__ void getParallel1DIndex<Parallel1Dir::THREAD_Y>(int& p_index, int& p_step)
{
#ifdef __CUDA_ARCH__
  p_index = threadIdx.y;
  p_step = blockDim.y;
#else
  p_index = 0;
  p_step = 1;
#endif
}

template <>
inline __host__ __device__ void getParallel1DIndex<Parallel1Dir::THREAD_Z>(int& p_index, int& p_step)
{
#ifdef __CUDA_ARCH__
  p_index = threadIdx.z;
  p_step = blockDim.z;
#else
  p_index = 0;
  p_step = 1;
#endif
}

template <>
inline __host__ __device__ void getParallel1DIndex<Parallel1Dir::THREAD_XY>(int& p_index, int& p_step)
{
#ifdef __CUDA_ARCH__
  p_index = threadIdx.x + blockDim.x * threadIdx.y;
  p_step = blockDim.x * blockDim.y;
#else
  p_index = 0;
  p_step = 1;
#endif
}

template <>
inline __host__ __device__ void getParallel1DIndex<Parallel1Dir::THREAD_XZ>(int& p_index, int& p_step)
{
#ifdef __CUDA_ARCH__
  p_index = threadIdx.x + blockDim.x * threadIdx.z;
  p_step = blockDim.x * blockDim.z;
#else
  p_index = 0;
  p_step = 1;
#endif
}

template <>
inline __host__ __device__ void getParallel1DIndex<Parallel1Dir::THREAD_YX>(int& p_index, int& p_step)
{
#ifdef __CUDA_ARCH__
  p_index = threadIdx.y + blockDim.y * threadIdx.x;
  p_step = blockDim.y * blockDim.x;
#else
  p_index = 0;
  p_step = 1;
#endif
}

template <>
inline __host__ __device__ void getParallel1DIndex<Parallel1Dir::THREAD_YZ>(int& p_index, int& p_step)
{
#ifdef __CUDA_ARCH__
  p_index = threadIdx.y + blockDim.y * threadIdx.z;
  p_step = blockDim.y * blockDim.z;
#else
  p_index = 0;
  p_step = 1;
#endif
}

template <>
inline __host__ __device__ void getParallel1DIndex<Parallel1Dir::THREAD_ZX>(int& p_index, int& p_step)
{
#ifdef __CUDA_ARCH__
  p_index = threadIdx.z + blockDim.z * threadIdx.x;
  p_step = blockDim.z * blockDim.x;
#else
  p_index = 0;
  p_step = 1;
#endif
}

template <>
inline __host__ __device__ void getParallel1DIndex<Parallel1Dir::THREAD_ZY>(int& p_index, int& p_step)
{
#ifdef __CUDA_ARCH__
  p_index = threadIdx.z + blockDim.z * threadIdx.y;
  p_step = blockDim.z * blockDim.y;
#else
  p_index = 0;
  p_step = 1;
#endif
}

template <>
inline __host__ __device__ void getParallel1DIndex<Parallel1Dir::THREAD_XYZ>(int& p_index, int& p_step)
{
#ifdef __CUDA_ARCH__
  p_index = threadIdx.x + blockDim.x * (threadIdx.y + blockDim.y * threadIdx.z);
  p_step = blockDim.x * blockDim.y * blockDim.z;
#else
  p_index = 0;
  p_step = 1;
#endif
}

template <>
inline __host__ __device__ void getParallel1DIndex<Parallel1Dir::GLOBAL_X>(int& p_index, int& p_step)
{
#ifdef __CUDA_ARCH__
  p_index = threadIdx.x + blockDim.x * blockIdx.x;
  p_step = gridDim.x * blockDim.x;
#else
  p_index = 0;
  p_step = 1;
#endif
}

template <>
inline __host__ __device__ void getParallel1DIndex<Parallel1Dir::GLOBAL_Y>(int& p_index, int& p_step)
{
#ifdef __CUDA_ARCH__
  p_index = threadIdx.y + blockDim.y * blockIdx.y;
  p_step = gridDim.y * blockDim.y;
#else
  p_index = 0;
  p_step = 1;
#endif
}

template <>
inline __host__ __device__ void getParallel1DIndex<Parallel1Dir::GLOBAL_Z>(int& p_index, int& p_step)
{
#ifdef __CUDA_ARCH__
  p_index = threadIdx.z + blockDim.z * blockIdx.z;
  p_step = gridDim.z * blockDim.z;
#else
  p_index = 0;
  p_step = 1;
#endif
}

template <>
inline __host__ __device__ void getParallel1DIndex<Parallel1Dir::NONE>(int& p_index, int& p_step)
{
  p_index = 0;
  p_step = 1;
}

template <Parallel1Dir P_DIR = Parallel1Dir::THREAD_Y, class T = float>
inline __device__ void loadArrayParallel(T* __restrict__ a1, const int off1, const T* __restrict__ a2, const int off2,
                                         const int N)
{
  int p_index, p_step;
  getParallel1DIndex<P_DIR>(p_index, p_step);
  if (N % 4 == 0 && sizeof(type4<T>) <= 16 && off1 % 4 == 0 && off2 % 4 == 0)
  {
    for (int i = p_index; i < N / 4; i += p_step)
    {
      reinterpret_cast<type4<T>*>(&a1[off1])[i] = reinterpret_cast<const type4<T>*>(&a2[off2])[i];
    }
  }
  else if (N % 2 == 0 && sizeof(type2<T>) <= 16 && off1 % 2 == 0 && off2 % 2 == 0)
  {
    for (int i = p_index; i < N / 2; i += p_step)
    {
      reinterpret_cast<type2<T>*>(&a1[off1])[i] = reinterpret_cast<const type2<T>*>(&a2[off2])[i];
    }
  }
  else
  {
    for (int i = p_index; i < N; i += p_step)
    {
      a1[off1 + i] = a2[off2 + i];
    }
  }
}

template <int N, Parallel1Dir P_DIR = Parallel1Dir::THREAD_Y, class T = float>
inline __device__ void loadArrayParallel(T* __restrict__ a1, const int off1, const T* __restrict__ a2, const int off2)
{
  loadArrayParallel<P_DIR, T>(a1, off1, a2, off2, N);
}
}  // namespace p1

namespace p2  // parallelize using 2 indices and steps
{
enum class Parallel2Dir : int
{
  THREAD_XY = 0,
  THREAD_XZ,
  THREAD_YZ,
  THREAD_YX,
  THREAD_ZX,
  THREAD_ZY,
  NONE
};

template <Parallel2Dir P_DIR>
inline __host__ __device__ void getParallel2DIndex(int& p1_index, int& p2_index, int& p1_step, int& p2_step)
{
#ifndef __CUDA_ARCH__
  p1_index = 0;
  p2_index = 0;
  p1_step = 1;
  p2_step = 1;
#endif
}

template <>
inline __device__ void getParallel2DIndex<Parallel2Dir::THREAD_XY>(int& p1_index, int& p2_index, int& p1_step,
                                                                   int& p2_step)
{
#ifdef __CUDA_ARCH__
  p1_index = threadIdx.x;
  p1_step = blockDim.x;
  p2_index = threadIdx.y;
  p2_step = blockDim.y;
#else
  p1_index = 0;
  p2_index = 0;
  p1_step = 1;
  p2_step = 1;
#endif
}

template <>
inline __device__ void getParallel2DIndex<Parallel2Dir::THREAD_YZ>(int& p1_index, int& p2_index, int& p1_step,
                                                                   int& p2_step)
{
#ifdef __CUDA_ARCH__
  p1_index = threadIdx.y;
  p1_step = blockDim.y;
  p2_index = threadIdx.z;
  p2_step = blockDim.z;
#else
  p1_index = 0;
  p2_index = 0;
  p1_step = 1;
  p2_step = 1;
#endif
}

template <>
inline __device__ void getParallel2DIndex<Parallel2Dir::THREAD_XZ>(int& p1_index, int& p2_index, int& p1_step,
                                                                   int& p2_step)
{
#ifdef __CUDA_ARCH__
  p1_index = threadIdx.x;
  p1_step = blockDim.x;
  p2_index = threadIdx.z;
  p2_step = blockDim.z;
#else
  p1_index = 0;
  p2_index = 0;
  p1_step = 1;
  p2_step = 1;
#endif
}

template <>
inline __device__ void getParallel2DIndex<Parallel2Dir::THREAD_YX>(int& p1_index, int& p2_index, int& p1_step,
                                                                   int& p2_step)
{
#ifdef __CUDA_ARCH__
  p1_index = threadIdx.y;
  p1_step = blockDim.y;
  p2_index = threadIdx.x;
  p2_step = blockDim.x;
#else
  p1_index = 0;
  p2_index = 0;
  p1_step = 1;
  p2_step = 1;
#endif
}

template <>
inline __device__ void getParallel2DIndex<Parallel2Dir::THREAD_ZY>(int& p1_index, int& p2_index, int& p1_step,
                                                                   int& p2_step)
{
#ifdef __CUDA_ARCH__
  p1_index = threadIdx.z;
  p1_step = blockDim.z;
  p2_index = threadIdx.y;
  p2_step = blockDim.y;
#else
  p1_index = 0;
  p2_index = 0;
  p1_step = 1;
  p2_step = 1;
#endif
}

template <>
inline __device__ void getParallel2DIndex<Parallel2Dir::THREAD_ZX>(int& p1_index, int& p2_index, int& p1_step,
                                                                   int& p2_step)
{
#ifdef __CUDA_ARCH__
  p1_index = threadIdx.z;
  p1_step = blockDim.z;
  p2_index = threadIdx.x;
  p2_step = blockDim.x;
#else
  p1_index = 0;
  p2_index = 0;
  p1_step = 1;
  p2_step = 1;
#endif
}

template <>
inline __device__ void getParallel2DIndex<Parallel2Dir::NONE>(int& p1_index, int& p2_index, int& p1_step, int& p2_step)
{
  p1_index = 0;
  p1_step = 1;
  p2_index = 0;
  p2_step = 1;
}
}  // namespace p2
}  // namespace mppi
