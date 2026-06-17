//
// Created by jason on 4/25/22.
//

#ifndef MPPIGENERIC_CUDA_MATH_UTILS_CUH
#define MPPIGENERIC_CUDA_MATH_UTILS_CUH

#include <cassert>

#ifndef __UNROLL
#define __xstr__(s) __str__(s)
#define __str__(s) #s
#ifdef __CUDACC__
#define __UNROLL(a) _Pragma("unroll")
#elif defined(__GNUC__)  // GCC is the compiler and uses different unroll syntax
#define __UNROLL(a) _Pragma(__xstr__(GCC unroll a))
#endif
#endif

// Matching float4 syntax
template <class T = float>
struct __align__(4 * sizeof(T)) type4
{
  T x;
  T y;
  T z;
  T w;
  // Allow writing to struct using array index
  __host__ __device__ T& operator[](int i)
  {
    assert(i >= 0);
    assert(i < 4);
    return (i > 1) ? ((i == 2) ? z : w) : ((i == 0) ? x : y);
  }
  // Allow reading from struct using array index
  __host__ __device__ const T& operator[](int i) const
  {
    assert(i >= 0);
    assert(i < 4);
    return (i > 1) ? ((i == 2) ? z : w) : ((i == 0) ? x : y);
  }
};

template <class T = float>
struct __align__(2 * sizeof(T)) type2
{
  T x;
  T y;

  // Allow writing to struct using array index
  __host__ __device__ T& operator[](int i)
  {
    assert(i >= 0);
    assert(i < 2);
    return (i == 0) ? x : y;
  }
  // Allow reading from struct using array index
  __host__ __device__ const T& operator[](int i) const
  {
    assert(i >= 0);
    assert(i < 2);
    return (i == 0) ? x : y;
  }
};

// Scalar-Vector Multiplication
__host__ __device__ inline float2 operator*(const float2& a, const float& b)
{
  return make_float2(a.x * b, a.y * b);
}
__host__ __device__ inline float3 operator*(const float3& a, const float& b)
{
  return make_float3(a.x * b, a.y * b, a.z * b);
}
__host__ __device__ inline float4 operator*(const float4& a, const float& b)
{
  return make_float4(a.x * b, a.y * b, a.z * b, a.w * b);
}

__host__ __device__ inline float2 operator*(const float& b, const float2& a)
{
  return make_float2(a.x * b, a.y * b);
}
__host__ __device__ inline float3 operator*(const float& b, const float3& a)
{
  return make_float3(a.x * b, a.y * b, a.z * b);
}
__host__ __device__ inline float4 operator*(const float& b, const float4& a)
{
  return make_float4(a.x * b, a.y * b, a.z * b, a.w * b);
}

// Scalar-Vector Addition
__host__ __device__ inline float2 operator+(const float2& a, const float& b)
{
  return make_float2(a.x + b, a.y + b);
}
__host__ __device__ inline float3 operator+(const float3& a, const float& b)
{
  return make_float3(a.x + b, a.y + b, a.z + b);
}
__host__ __device__ inline float4 operator+(const float4& a, const float& b)
{
  return make_float4(a.x + b, a.y + b, a.z + b, a.w + b);
}

// Scalar-Vector Subtraction
__host__ __device__ inline float2 operator-(const float2& a, const float& b)
{
  return make_float2(a.x - b, a.y - b);
}
__host__ __device__ inline float3 operator-(const float3& a, const float& b)
{
  return make_float3(a.x - b, a.y - b, a.z - b);
}
__host__ __device__ inline float4 operator-(const float4& a, const float& b)
{
  return make_float4(a.x - b, a.y - b, a.z - b, a.w - b);
}

// Scalar-Vector Dvision
__host__ __device__ inline float2 operator/(const float2& a, const float& b)
{
  return make_float2(a.x / b, a.y / b);
}
__host__ __device__ inline float3 operator/(const float3& a, const float& b)
{
  return make_float3(a.x / b, a.y / b, a.z / b);
}
__host__ __device__ inline float4 operator/(const float4& a, const float& b)
{
  return make_float4(a.x / b, a.y / b, a.z / b, a.w / b);
}

// Vector-Vector Addition
__host__ __device__ inline float2 operator+(const float2& a, const float2& b)
{
  return make_float2(a.x + b.x, a.y + b.y);
}
__host__ __device__ inline float3 operator+(const float3& a, const float3& b)
{
  return make_float3(a.x + b.x, a.y + b.y, a.z + b.z);
}
__host__ __device__ inline float4 operator+(const float4& a, const float4& b)
{
  return make_float4(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w);
}

// Vector-Vector Subtraction
__host__ __device__ inline float2 operator-(const float2& a, const float2& b)
{
  return make_float2(a.x - b.x, a.y - b.y);
}

__host__ __device__ inline float3 operator-(const float3& a, const float3& b)
{
  return make_float3(a.x - b.x, a.y - b.y, a.z - b.z);
}

__host__ __device__ inline float4 operator-(const float4& a, const float4& b)
{
  return make_float4(a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w);
}

// Vector-Vector Multiplication
__host__ __device__ inline float2 operator*(const float2& a, const float2& b)
{
  return make_float2(a.x * b.x, a.y * b.y);
}

__host__ __device__ inline float3 operator*(const float3& a, const float3& b)
{
  return make_float3(a.x * b.x, a.y * b.y, a.z * b.z);
}

__host__ __device__ inline float4 operator*(const float4& a, const float4& b)
{
  return make_float4(a.x * b.x, a.y * b.y, a.z * b.z, a.w * b.w);
}

// Vector-Vector Division
__host__ __device__ inline float2 operator/(const float2& a, const float2& b)
{
  return make_float2(a.x / b.x, a.y / b.y);
}

__host__ __device__ inline float3 operator/(const float3& a, const float3& b)
{
  return make_float3(a.x / b.x, a.y / b.y, a.z / b.z);
}

__host__ __device__ inline float4 operator/(const float4& a, const float4& b)
{
  return make_float4(a.x / b.x, a.y / b.y, a.z / b.z, a.w / b.w);
}

// Scalar-Vector multiply and set
__host__ __device__ inline float2& operator*=(float2& a, const float& b)
{
  a.x *= b;
  a.y *= b;
  return a;
}
__host__ __device__ inline float3& operator*=(float3& a, const float& b)
{
  a.x *= b;
  a.y *= b;
  a.z *= b;
  return a;
}
__host__ __device__ inline float4& operator*=(float4& a, const float& b)
{
  a.x *= b;
  a.y *= b;
  a.z *= b;
  a.w *= b;
  return a;
}

// Vector-Vector add and set
__host__ __device__ inline float2& operator+=(float2& a, const float2& b)
{
  a.x += b.x;
  a.y += b.y;
  return a;
}

__host__ __device__ inline float3& operator+=(float3& a, const float3& b)
{
  a.x += b.x;
  a.y += b.y;
  a.z += b.z;
  return a;
}
__host__ __device__ inline float4& operator+=(float4& a, const float4& b)
{
  a.x += b.x;
  a.y += b.y;
  a.z += b.z;
  a.w += b.w;
  return a;
}

__host__ __device__ inline float dot(const float2& a, const float2& b)
{
  return a.x * b.x + a.y * b.y;
}

__host__ __device__ inline float cross(const float2& a, const float2& b)
{
  return a.x * b.y - a.y * b.x;
}

__host__ __device__ inline float norm(const float2& a)
{
  return sqrtf(dot(a, a));
}

__host__ __device__ inline float dot(const float3& a, const float3& b)
{
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

__host__ __device__ inline float norm(const float3& a)
{
  return sqrtf(dot(a, a));
}

__host__ __device__ inline float dot(const float4& a, const float4& b)
{
  return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

__host__ __device__ inline float norm(const float4& a)
{
  return sqrtf(dot(a, a));
}

__host__ __device__ inline float2 operator-(const float2& a)
{
  return make_float2(-a.x, -a.y);
}

__host__ __device__ inline float3 operator-(const float3& a)
{
  return make_float3(-a.x, -a.y, -a.z);
}

__host__ __device__ inline float4 operator-(const float4& a)
{
  return make_float4(-a.x, -a.y, -a.z, -a.w);
}

__host__ __device__ inline bool operator==(const float2& lhs, const float2& rhs)
{
  return (lhs.x == rhs.x) && (lhs.y == rhs.y);
}

__host__ __device__ inline bool operator==(const float3& lhs, const float3& rhs)
{
  return (lhs.x == rhs.x) && (lhs.y == rhs.y) && (lhs.z == rhs.z);
}

__host__ __device__ inline bool operator==(const float4& lhs, const float4& rhs)
{
  return (lhs.x == rhs.x) && (lhs.y == rhs.y) && (lhs.z == rhs.z) && (lhs.w == rhs.w);
}

// Create different data types given the same input. Needed for handling border values in cudaTextures with Border
// adress mode
template <class DATA_T>
inline __host__ __device__ DATA_T createPartialCudaTuple(const float& r, const float& g, const float& b,
                                                         const float& a);

template <>
inline __host__ __device__ float createPartialCudaTuple<float>(const float& r, const float& g, const float& b,
                                                               const float& a)
{
  return r;
}

template <>
inline __host__ __device__ float2 createPartialCudaTuple<float2>(const float& r, const float& g, const float& b,
                                                                 const float& a)
{
  return make_float2(r, g);
}

template <>
inline __host__ __device__ float3 createPartialCudaTuple<float3>(const float& r, const float& g, const float& b,
                                                                 const float& a)
{
  return make_float3(r, g, b);
}

template <>
inline __host__ __device__ float4 createPartialCudaTuple<float4>(const float& r, const float& g, const float& b,
                                                                 const float& a)
{
  return make_float4(r, g, b, a);
}

template <>
inline __host__ __device__ int createPartialCudaTuple<int>(const float& r, const float& g, const float& b,
                                                           const float& a)
{
  return static_cast<int>(r);
}

template <>
inline __host__ __device__ int2 createPartialCudaTuple<int2>(const float& r, const float& g, const float& b,
                                                             const float& a)
{
  return make_int2(static_cast<int>(r), static_cast<int>(g));
}

template <>
inline __host__ __device__ int3 createPartialCudaTuple<int3>(const float& r, const float& g, const float& b,
                                                             const float& a)
{
  return make_int3(static_cast<int>(r), static_cast<int>(g), static_cast<int>(b));
}

template <>
inline __host__ __device__ int4 createPartialCudaTuple<int4>(const float& r, const float& g, const float& b,
                                                             const float& a)
{
  return make_int4(static_cast<int>(r), static_cast<int>(g), static_cast<int>(b), static_cast<int>(a));
}
#endif  // MPPIGENERIC_CUDA_MATH_UTILS_CUH
