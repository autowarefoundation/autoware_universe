#ifndef ACTIVATION_FUNCTIONS_CUH_
#define ACTIVATION_FUNCTIONS_CUH_

#include "math_utils.h"
namespace mppi
{
namespace nn
{
/**
 * There is hardware support for the tanh in some GPUs, so the hand written version is slower
 * Checked on a 3080 and 1050ti
 * @param input
 * @return
 */
inline __host__ __device__ float tanh(float input)
{
  // There is hardware support for tanh starting in Turing (above 750+) that makes this approximation slower
// #if defined(__CUDA_ARCH__) && (__CUDA_ARCH__ < 750)
//   const float num = __expf(2.0f * input);
//   // return __fdiv_rz(num - 1.0f, num + 1.0f);
//   return (num - 1.0f) / (num + 1.0f);
// #else
//   return tanhf(input);
// #endif
  return tanhf(input);
}

inline __host__ __device__ float tanh_accurate(float input)
{
  return tanhf(input);
}

inline __host__ __device__ float tanh_deriv(float input)
{
  return 1.0f - SQ(mppi::nn::tanh(input));
}

inline __host__ __device__ float relu(float input)
{
  return fmaxf(0.0f, input);
}

/**
 * Uses hardware support for tanh function, should be faster
 * Tested on 3080 and 1050ti
 * @param input
 * @return
 */
inline __host__ __device__ float sigmoid(float input)
{
#ifdef __CUDA_ARCH__
  // these three are roughly equivalent on 3080
  // return __fmul_rz(1.0f + tanhf(__fmul_rz(input, 0.5f)), 0.5f);
  // return __fmaf_rz(tanhf(__fmul_rz(input, 0.5f)), 0.5f, 0.5f);
  return (1.0f + mppi::nn::tanh(input / 2.0f)) / 2.0f;
#else
  return (1.0f / (1.0f + expf(-input)));
#endif
}

inline __host__ __device__ float sigmoid_accurate(float input)
{
  return (1.0f / (1.0f + expf(-input)));
}

/**
 *
 * @param input
 * @return
 */
__host__ __device__ static inline float tanh_vel_scale(float state, float vel, const float constants[2])
{
  return state * constants[1] * mppi::nn::tanh(vel * constants[0]);
}

/**
 *
 * @param input
 * @return
 */
__host__ __device__ static inline float tanh_scale(float state, const float constants[2])
{
  return constants[1] * mppi::nn::tanh(state * constants[0]);
}

/**
 *
 * @param x
 * @return
 */
__host__ __device__ static inline float tanhshrink(float x)
{
  return x - mppi::nn::tanh(x);
}

/**
 *
 * @param x
 * @return
 */
__host__ __device__ static inline float tanhshrink_scale(float x, float scale)
{
  return scale * tanhshrink(x / scale);
}

}  // namespace nn
}  // namespace mppi

#endif  // ACTIVATION_FUNCTIONS_CUH_
