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

#ifndef AUTOWARE__SCATTER_OPS_ATOMICS_CUH
#define AUTOWARE__SCATTER_OPS_ATOMICS_CUH

#include <cuda_bf16.h>
#include <cuda_fp16.h>

#include <cstdint>  // cSpell:ignore cstdint

// cSpell:ignore newval bfloat ifdef
#define ATOMIC(NAME)                                                                               \
  template <typename scalar, size_t size>                                                          \
  struct Atomic##NAME##IntegerImpl;                                                                \
                                                                                                   \
  template <typename scalar>                                                                       \
  struct Atomic##NAME##IntegerImpl<scalar, 1>                                                      \
  {                                                                                                \
    inline __device__ void operator()(scalar * address, scalar val)                                \
    {                                                                                              \
      uint32_t * address_as_ui = (uint32_t *)(address - ((size_t)address & 3));                    \
      uint32_t old = *address_as_ui;                                                               \
      uint32_t shift = ((size_t)address & 3) * 8;                                                  \
      uint32_t sum;                                                                                \
      uint32_t assumed;                                                                            \
                                                                                                   \
      do {                                                                                         \
        assumed = old;                                                                             \
        sum = OP(val, scalar((old >> shift) & 0xff));                                              \
        old = (old & ~(0x000000ff << shift)) | (sum << shift);                                     \
        old = atomicCAS(address_as_ui, assumed, old);                                              \
      } while (assumed != old);                                                                    \
    }                                                                                              \
  };                                                                                               \
                                                                                                   \
  template <typename scalar>                                                                       \
  struct Atomic##NAME##IntegerImpl<scalar, 2>                                                      \
  {                                                                                                \
    inline __device__ void operator()(scalar * address, scalar val)                                \
    {                                                                                              \
      uint32_t * address_as_ui = (uint32_t *)((char *)address - ((size_t)address & 2));            \
      uint32_t old = *address_as_ui;                                                               \
      uint32_t sum;                                                                                \
      uint32_t newval;                                                                             \
      uint32_t assumed;                                                                            \
                                                                                                   \
      do {                                                                                         \
        assumed = old;                                                                             \
        sum = OP(val, (size_t)address & 2 ? scalar(old >> 16) : scalar(old & 0xffff));             \
        newval = (size_t)address & 2 ? (old & 0xffff) | (sum << 16) : (old & 0xffff0000) | sum;    \
        old = atomicCAS(address_as_ui, assumed, newval);                                           \
      } while (assumed != old);                                                                    \
    }                                                                                              \
  };                                                                                               \
                                                                                                   \
  template <typename scalar>                                                                       \
  struct Atomic##NAME##IntegerImpl<scalar, 4>                                                      \
  {                                                                                                \
    inline __device__ void operator()(scalar * address, scalar val)                                \
    {                                                                                              \
      uint32_t * address_as_ui = (uint32_t *)address;                                              \
      uint32_t old = *address_as_ui;                                                               \
      uint32_t assumed;                                                                            \
                                                                                                   \
      do {                                                                                         \
        assumed = old;                                                                             \
        old = atomicCAS(address_as_ui, assumed, OP(val, (scalar)old));                             \
      } while (assumed != old);                                                                    \
    }                                                                                              \
  };                                                                                               \
                                                                                                   \
  template <typename scalar>                                                                       \
  struct Atomic##NAME##IntegerImpl<scalar, 8>                                                      \
  {                                                                                                \
    inline __device__ void operator()(scalar * address, scalar val)                                \
    {                                                                                              \
      unsigned long long * address_as_ull = (unsigned long long *)address;                         \
      unsigned long long old = *address_as_ull;                                                    \
      unsigned long long assumed;                                                                  \
                                                                                                   \
      do {                                                                                         \
        assumed = old;                                                                             \
        old = atomicCAS(address_as_ull, assumed, OP(val, (scalar)old));                            \
      } while (assumed != old);                                                                    \
    }                                                                                              \
  };                                                                                               \
                                                                                                   \
  template <typename scalar, size_t size>                                                          \
  struct Atomic##NAME##DecimalImpl;                                                                \
                                                                                                   \
  template <>                                                                                      \
  struct Atomic##NAME##DecimalImpl<__half, 2>                                                      \
  {                                                                                                \
    inline __device__ void operator()(__half * address, __half val)                                \
    {                                                                                              \
      unsigned short * address_as_us = (unsigned short *)address;                                  \
      unsigned short old = *address_as_us;                                                         \
      unsigned short assumed;                                                                      \
      do {                                                                                         \
        assumed = old;                                                                             \
        old = atomicCAS(address_as_us, assumed, __half_as_ushort(OP(val, __ushort_as_half(old)))); \
      } while (assumed != old);                                                                    \
    }                                                                                              \
  };                                                                                               \
                                                                                                   \
  template <>                                                                                      \
  struct Atomic##NAME##DecimalImpl<__nv_bfloat16, 2>                                               \
  {                                                                                                \
    inline __device__ void operator()(__nv_bfloat16 * address, __nv_bfloat16 val)                  \
    {                                                                                              \
      unsigned short * address_as_us = (unsigned short *)address;                                  \
      unsigned short old = *address_as_us;                                                         \
      unsigned short assumed;                                                                      \
      do {                                                                                         \
        assumed = old;                                                                             \
        old = atomicCAS(                                                                           \
          address_as_us, assumed, __bfloat16_as_ushort(OP(val, __ushort_as_bfloat16(old))));       \
      } while (assumed != old);                                                                    \
    }                                                                                              \
  };                                                                                               \
                                                                                                   \
  template <typename scalar>                                                                       \
  struct Atomic##NAME##DecimalImpl<scalar, 4>                                                      \
  {                                                                                                \
    inline __device__ void operator()(scalar * address, scalar val)                                \
    {                                                                                              \
      unsigned int * address_as_i = (unsigned int *)address;                                       \
      unsigned int old = *address_as_i;                                                            \
      unsigned int assumed;                                                                        \
                                                                                                   \
      do {                                                                                         \
        assumed = old;                                                                             \
        old = atomicCAS(address_as_i, assumed, __float_as_int(OP(val, __int_as_float(assumed))));  \
      } while (assumed != old);                                                                    \
    }                                                                                              \
  };                                                                                               \
                                                                                                   \
  template <typename scalar>                                                                       \
  struct Atomic##NAME##DecimalImpl<scalar, 8>                                                      \
  {                                                                                                \
    inline __device__ void operator()(scalar * address, scalar val)                                \
    {                                                                                              \
      unsigned long long int * address_as_ull = (unsigned long long int *)address;                 \
      unsigned long long int old = *address_as_ull;                                                \
      unsigned long long int assumed;                                                              \
                                                                                                   \
      do {                                                                                         \
        assumed = old;                                                                             \
        old = atomicCAS(                                                                           \
          address_as_ull, assumed, __double_as_longlong(OP(val, __longlong_as_double(assumed))));  \
      } while (assumed != old);                                                                    \
    }                                                                                              \
  };

#define OP(X, Y) ((Y) + (X))
ATOMIC(Add)
#undef OP
static inline __device__ void atomAdd(uint8_t * address, uint8_t val)
{
  AtomicAddIntegerImpl<uint8_t, sizeof(uint8_t)>()(address, val);
}
static inline __device__ void atomAdd(int8_t * address, int8_t val)
{
  AtomicAddIntegerImpl<int8_t, sizeof(int8_t)>()(address, val);
}
static inline __device__ void atomAdd(int16_t * address, int16_t val)
{
  AtomicAddIntegerImpl<int16_t, sizeof(int16_t)>()(address, val);
}
static inline __device__ void atomAdd(int32_t * address, int32_t val)
{
  atomicAdd(address, val);
}
static inline __device__ void atomAdd(int64_t * address, int64_t val)
{
  AtomicAddIntegerImpl<int64_t, sizeof(int64_t)>()(address, val);
}
#if defined(__CUDA_ARCH__) && (__CUDA_ARCH__ < 700 || CUDA_VERSION < 10000)
static inline __device__ void atomAdd(__half * address, __half val)
{
  AtomicAddDecimalImpl<__half, sizeof(__half)>()(address, val);
}
#else
static inline __device__ void atomAdd(__half * address, __half val)
{
  atomicAdd(address, val);
}
#endif
static inline __device__ void atomAdd(float * address, float val)
{
  atomicAdd(address, val);
}
#if defined(__CUDA_ARCH__) && (__CUDA_ARCH__ < 600 || CUDA_VERSION < 8000)
static inline __device__ void atomAdd(double * address, double val)
{
  AtomicAddDecimalImpl<double, sizeof(double)>()(address, val);
}
#else
static inline __device__ void atomAdd(double * address, double val)
{
  atomicAdd(address, val);
}
#endif

#ifdef __CUDA_ARCH__
static inline __device__ void atomAdd(__nv_bfloat16 * address, __nv_bfloat16 val)
{
  AtomicAddDecimalImpl<__nv_bfloat16, 2>()(address, val);
}
#endif

#define OP(X, Y) ((Y) * (X))
ATOMIC(Mul)
#undef OP
static inline __device__ void atomMul(uint8_t * address, uint8_t val)
{
  AtomicMulIntegerImpl<uint8_t, sizeof(uint8_t)>()(address, val);
}
static inline __device__ void atomMul(int8_t * address, int8_t val)
{
  AtomicMulIntegerImpl<int8_t, sizeof(int8_t)>()(address, val);
}
static inline __device__ void atomMul(int16_t * address, int16_t val)
{
  AtomicMulIntegerImpl<int16_t, sizeof(int16_t)>()(address, val);
}
static inline __device__ void atomMul(int32_t * address, int32_t val)
{
  AtomicMulIntegerImpl<int32_t, sizeof(int32_t)>()(address, val);
}
static inline __device__ void atomMul(int64_t * address, int64_t val)
{
  AtomicMulIntegerImpl<int64_t, sizeof(int64_t)>()(address, val);
}
static inline __device__ void atomMul(float * address, float val)
{
  AtomicMulDecimalImpl<float, sizeof(float)>()(address, val);
}
static inline __device__ void atomMul(__half * address, __half val)
{
  AtomicMulDecimalImpl<__half, sizeof(__half)>()(address, val);
}
static inline __device__ void atomMul(double * address, double val)
{
  AtomicMulDecimalImpl<double, sizeof(double)>()(address, val);
}

#ifdef __CUDA_ARCH__
static inline __device__ void atomMul(__nv_bfloat16 * address, __nv_bfloat16 val)
{
  AtomicMulDecimalImpl<__nv_bfloat16, 2>()(address, val);
}
#endif

#define OP(X, Y) ((Y) / (X))
ATOMIC(Div)
#undef OP
static inline __device__ void atomDiv(uint8_t * address, uint8_t val)
{
  AtomicDivIntegerImpl<uint8_t, sizeof(uint8_t)>()(address, val);
}
static inline __device__ void atomDiv(int8_t * address, int8_t val)
{
  AtomicDivIntegerImpl<int8_t, sizeof(int8_t)>()(address, val);
}
static inline __device__ void atomDiv(int16_t * address, int16_t val)
{
  AtomicDivIntegerImpl<int16_t, sizeof(int16_t)>()(address, val);
}
static inline __device__ void atomDiv(int32_t * address, int32_t val)
{
  AtomicDivIntegerImpl<int32_t, sizeof(int32_t)>()(address, val);
}
static inline __device__ void atomDiv(int64_t * address, int64_t val)
{
  AtomicDivIntegerImpl<int64_t, sizeof(int64_t)>()(address, val);
}
static inline __device__ void atomDiv(__half * address, __half val)
{
  AtomicDivDecimalImpl<__half, sizeof(__half)>()(address, val);
}
static inline __device__ void atomDiv(float * address, float val)
{
  AtomicDivDecimalImpl<float, sizeof(float)>()(address, val);
}
static inline __device__ void atomDiv(double * address, double val)
{
  AtomicDivDecimalImpl<double, sizeof(double)>()(address, val);
}

#ifdef __CUDA_ARCH__
static inline __device__ void atomDiv(__nv_bfloat16 * address, __nv_bfloat16 val)
{
  AtomicDivDecimalImpl<__nv_bfloat16, 2>()(address, val);
}
#endif

#define OP(X, Y) ((X) >= (Y) ? (X) : (Y))
ATOMIC(Max)
#undef OP
static inline __device__ void atomMax(uint8_t * address, uint8_t val)
{
  AtomicMaxIntegerImpl<uint8_t, sizeof(uint8_t)>()(address, val);
}
static inline __device__ void atomMax(int8_t * address, int8_t val)
{
  AtomicMaxIntegerImpl<int8_t, sizeof(int8_t)>()(address, val);
}
static inline __device__ void atomMax(int16_t * address, int16_t val)
{
  AtomicMaxIntegerImpl<int16_t, sizeof(int16_t)>()(address, val);
}
static inline __device__ void atomMax(int32_t * address, int32_t val)
{
  atomicMax(address, val);
}
static inline __device__ void atomMax(int64_t * address, int64_t val)
{
  AtomicMaxIntegerImpl<int64_t, sizeof(int64_t)>()(address, val);
}
static inline __device__ void atomMax(__half * address, __half val)
{
  AtomicMaxDecimalImpl<__half, sizeof(__half)>()(address, val);
}
static inline __device__ void atomMax(float * address, float val)
{
  AtomicMaxDecimalImpl<float, sizeof(float)>()(address, val);
}
static inline __device__ void atomMax(double * address, double val)
{
  AtomicMaxDecimalImpl<double, sizeof(double)>()(address, val);
}

#ifdef __CUDA_ARCH__
static inline __device__ void atomMax(__nv_bfloat16 * address, __nv_bfloat16 val)
{
  AtomicMaxDecimalImpl<__nv_bfloat16, 2>()(address, val);
}
#endif

#define OP(X, Y) ((X) <= (Y) ? (X) : (Y))
ATOMIC(Min)
#undef OP
static inline __device__ void atomMin(uint8_t * address, uint8_t val)
{
  AtomicMinIntegerImpl<uint8_t, sizeof(uint8_t)>()(address, val);
}
static inline __device__ void atomMin(int8_t * address, int8_t val)
{
  AtomicMinIntegerImpl<int8_t, sizeof(int8_t)>()(address, val);
}
static inline __device__ void atomMin(int16_t * address, int16_t val)
{
  AtomicMinIntegerImpl<int16_t, sizeof(int16_t)>()(address, val);
}
static inline __device__ void atomMin(int32_t * address, int32_t val)
{
  atomicMin(address, val);
}
static inline __device__ void atomMin(int64_t * address, int64_t val)
{
  AtomicMinIntegerImpl<int64_t, sizeof(int64_t)>()(address, val);
}
static inline __device__ void atomMin(__half * address, __half val)
{
  AtomicMinDecimalImpl<__half, sizeof(__half)>()(address, val);
}
static inline __device__ void atomMin(float * address, float val)
{
  AtomicMinDecimalImpl<float, sizeof(float)>()(address, val);
}
static inline __device__ void atomMin(double * address, double val)
{
  AtomicMinDecimalImpl<double, sizeof(double)>()(address, val);
}

#ifdef __CUDA_ARCH__
static inline __device__ void atomMin(__nv_bfloat16 * address, __nv_bfloat16 val)
{
  AtomicMinDecimalImpl<__nv_bfloat16, 2>()(address, val);
}
#endif

#endif  // AUTOWARE__SCATTER_OPS_ATOMICS_CUH
