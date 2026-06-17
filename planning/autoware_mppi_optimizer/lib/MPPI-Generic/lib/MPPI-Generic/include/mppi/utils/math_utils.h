/*
 * Created on Mon Jun 01 2020 by Bogdan
 *
 */
#ifndef MATH_UTILS_H_
#define MATH_UTILS_H_

// Needed for sampling without replacement
#include <chrono>
#include <type_traits>
#include <unordered_set>
#include <random>
#include <vector>
#include <algorithm>

#include <cuda_runtime.h>
#include <Eigen/Dense>

#include <mppi/utils/angle_utils.cuh>
#include <mppi/utils/matrix_mult_utils.cuh>
#include <mppi/utils/risk_utils.cuh>

#ifndef SQ
#define SQ(a) ((a) * (a))
#endif  // SQ

// For aligning parameters within structs such as a float array to 16 bytes
// Ex: float name[size] MPPI_ALIGN(16) = {0.0f};
#if defined(__CUDACC__)  // NVCC
#define MPPI_ALIGN(n) __align__(n)
#elif defined(__GNUC__)  // GCC
#define MPPI_ALIGN(n) __attribute__((aligned(n)))
#elif defined(_MSC_VER)  // MSVC
#define MPPI_ALIGN(n) __declspec(align(n))
#else
#error "Please provide a definition for MPPI_ALIGN macro for your host compiler!"
#endif

namespace mppi
{
namespace math
{
namespace matrix = ::mppi::matrix_multiplication;

const float GRAVITY = 9.81f;
// Based off of https://gormanalysis.com/blog/random-numbers-in-cpp
inline std::vector<int> sample_without_replacement(const int k, const int N,
                                                   std::default_random_engine g = std::default_random_engine())
{
  if (k > N)
  {
    throw std::logic_error("Can't sample more than n times without replacement");
  }
  // Create an unordered set to store the samples
  std::unordered_set<int> samples;

  // For loop runs k times
  for (int r = N - k; r < N; r++)
  {
    if (r == 0)
    {
      samples.insert(N - 1);
      continue;
    }
    int v = std::uniform_int_distribution<>(1, r)(g);  // sample between 1 and r
    if (!samples.insert(v - 1).second)
    {  // if v exists in the set
      samples.insert(r - 1);
    }
  }
  // Copy set into a vector
  std::vector<int> final_sequence(samples.begin(), samples.end());
  // Shuffle the vector to get the final sequence of sampling
  std::shuffle(final_sequence.begin(), final_sequence.end(), g);
  return final_sequence;
}

inline __host__ __device__ float expr(const float r, const float x)
{
  float mid_term = 1.0f + (r - 1.0f) * x;
  return (mid_term > 0) * powf(mid_term, 1.0f / (r - 1.0f));
}

/**
 * Linear interpolation
 * Given two coordinates (x_min, y_min) and (x_max, y_max)
 * And the x location of a third (x), return the y location
 * along the line between the two points
 */
inline __host__ __device__ float linInterp(const float x, const float x_min, const float x_max, const float y_min,
                                           const float y_max)
{
  return (x - x_min) / (x_max - x_min) * (y_max - y_min) + y_min;
}

/**
 * Return the sign of a variable (1 for positive, -1 for negative, 0 for 0)
 **/
template <class T = float>
inline __host__ __device__ int sign(const T& a)
{
  return (a > 0) - (a < 0);
}

inline __host__ __device__ int int_ceil(const int& a, const int& b)
{
  return a == 0 ? a : (a - 1) / b + 1;
}

inline constexpr __host__ __device__ int int_ceil_const(const int& a, const int& b)
{
  return a == 0 ? a : (a - 1) / b + 1;
}

/**
 * @brief gives back the next multiple of b larger than or equal to a.
 * For example, if a = 3, b = 2, this method returns 4
 *
 * @param a - int to be larger than
 * @param b - int to be multiple of
 * @return int - the next multiple of b larger than a
 */
inline constexpr __host__ __device__ int int_multiple_const(const int& a, const int& b)
{
  return a == 0 ? a : ((a - 1) / b + 1) * b;
}

// Returns the int version of ceil(a/4)
inline __host__ __device__ int nearest_quotient_4(const int& a)
{
  return int_ceil(a, 4);
}

// Returns the next multiple of 4 larger than or equal to a, Useful for calculating aligned memory sizes
inline __host__ __device__ int nearest_multiple_4(const int& a)
{
  return int_ceil(a, 4) * 4;
}

/**
 * Calculates the normalized distance from the centerline
 * @param r - current radius
 * @param r_in - the inside radius of a track
 * @param r_out - the outside radius of a track
 * @return norm_dist - a normalized distance away from the centerline
 * norm_dist = 0 -> on the centerline
 * norm_dist = 1 -> on one of the track boundaries inner, or outer
 */
inline __host__ __device__ float normDistFromCenter(const float r, const float r_in, const float r_out)
{
  float r_center = (r_in + r_out) / 2.0f;
  float r_width = (r_out - r_in);
  float dist_from_center = fabsf(r - r_center);
  float norm_dist = dist_from_center / (r_width * 0.5f);
  return norm_dist;
}

/**
 * Multiply two quaternions together which gives you their rotations added together
 * q_3 = q_1 x q_2
 * Inputs:
 *  q_1 - first quaternion
 *  q_2 - second quaternion
 *  q_3 - output quaternion
 */
inline __host__ __device__ void QuatMultiply(const float q_1[4], const float q_2[4], float q_3[4],
                                             bool normalize = true)
{
  q_3[0] = q_1[0] * q_2[0] - q_1[1] * q_2[1] - q_1[2] * q_2[2] - q_1[3] * q_2[3];
  q_3[1] = q_1[1] * q_2[0] + q_1[0] * q_2[1] - q_1[3] * q_2[2] + q_1[2] * q_2[3];
  q_3[2] = q_1[2] * q_2[0] + q_1[3] * q_2[1] + q_1[0] * q_2[2] - q_1[1] * q_2[3];
  q_3[3] = q_1[3] * q_2[0] - q_1[2] * q_2[1] + q_1[1] * q_2[2] + q_1[0] * q_2[3];
  if (normalize)
  {
#ifdef __CUDA_ARCH__
    float inv_norm = rsqrtf(SQ(q_3[0]) + SQ(q_3[1]) + SQ(q_3[2]) + SQ(q_3[3]));
#else
    float inv_norm = 1.0f / sqrtf(SQ(q_3[0]) + SQ(q_3[1]) + SQ(q_3[2]) + SQ(q_3[3]));
#endif
    __UNROLL(4)
    for (int i = 0; i < 4; i++)
    {
      q_3[i] *= inv_norm;
    }
  }
}

inline __host__ __device__ void QuatInv(const float q[4], float q_inv[4])
{
#ifdef __CUDA_ARCH__
  float inv_norm = rsqrtf(SQ(q[0]) + SQ(q[1]) + SQ(q[2]) + SQ(q[3]));
#else
  float inv_norm = 1.0f / sqrtf(SQ(q[0]) + SQ(q[1]) + SQ(q[2]) + SQ(q[3]));
#endif
  q_inv[0] = q[0] * inv_norm;
  q_inv[1] = -q[1] * inv_norm;
  q_inv[2] = -q[2] * inv_norm;
  q_inv[3] = -q[3] * inv_norm;
}

/**
 * Calculate the rotation required to get from q_1 to q_2
 * In Euler angles, this would be direct subraction but
 * in quaternions, it doesn't quite work that way
 */
inline __device__ void QuatSubtract(float q_1[4], float q_2[4], float q_3[4])
{
  float q_1_inv[4];
  QuatInv(q_1, q_1_inv);
  QuatMultiply(q_2, q_1_inv, q_3);
}

/*
 * The Euler rotation sequence is 3-2-1 (roll, pitch, yaw) from Body to World
 */
inline __device__ void Euler2QuatNWU(const double& r, const double& p, const double& y, double q[4])
{
  double phi_2 = r / 2.0;
  double theta_2 = p / 2.0;
  double psi_2 = y / 2.0;
  double cos_phi_2 = cos(phi_2);
  double sin_phi_2 = sin(phi_2);
  double cos_theta_2 = cos(theta_2);
  double sin_theta_2 = sin(theta_2);
  double cos_psi_2 = cos(psi_2);
  double sin_psi_2 = sin(psi_2);

  q[0] = cos_phi_2 * cos_theta_2 * cos_psi_2 + sin_phi_2 * sin_theta_2 * sin_psi_2;
  q[1] = -cos_phi_2 * sin_theta_2 * sin_psi_2 + cos_theta_2 * cos_psi_2 * sin_phi_2;
  q[2] = cos_phi_2 * cos_psi_2 * sin_theta_2 + sin_phi_2 * cos_theta_2 * sin_psi_2;
  q[3] = cos_phi_2 * cos_theta_2 * sin_psi_2 - sin_phi_2 * cos_psi_2 * sin_theta_2;
}

/*
 * The Euler rotation sequence is 3-2-1 (roll, pitch, yaw) from Body to World
 */
inline __host__ __device__ void Euler2QuatNWU(const float& r, const float& p, const float& y, float q[4])
{
  float sin_phi_2, cos_phi_2, sin_theta_2, cos_theta_2, sin_psi_2, cos_psi_2;
#ifdef __CUDA_ARCH__
  float r_norm = angle_utils::normalizeAngle(r * 0.5f);
  float p_norm = angle_utils::normalizeAngle(p * 0.5f);
  float y_norm = angle_utils::normalizeAngle(y * 0.5f);
  __sincosf(r_norm, &sin_phi_2, &cos_phi_2);
  __sincosf(p_norm, &sin_theta_2, &cos_theta_2);
  __sincosf(y_norm, &sin_psi_2, &cos_psi_2);
#else
  sincosf(r * 0.5f, &sin_phi_2, &cos_phi_2);
  sincosf(p * 0.5f, &sin_theta_2, &cos_theta_2);
  sincosf(y * 0.5f, &sin_psi_2, &cos_psi_2);
#endif

  q[0] = cos_phi_2 * cos_theta_2 * cos_psi_2 + sin_phi_2 * sin_theta_2 * sin_psi_2;
  q[1] = -cos_phi_2 * sin_theta_2 * sin_psi_2 + cos_theta_2 * cos_psi_2 * sin_phi_2;
  q[2] = cos_phi_2 * cos_psi_2 * sin_theta_2 + sin_phi_2 * cos_theta_2 * sin_psi_2;
  q[3] = cos_phi_2 * cos_theta_2 * sin_psi_2 - sin_phi_2 * cos_psi_2 * sin_theta_2;
}

// (RPY rotation sequence)
/*
 * Returns an euler sequence 3-2-1 (roll pitch yaw) that when applied takes you from body to world
 */
inline __host__ __device__ void Quat2EulerNWU(const float q[4], float& r, float& p, float& y)
{
  r = atan2f(2.0f * q[3] * q[2] + 2.0f * q[0] * q[1], q[0] * q[0] + q[3] * q[3] - q[2] * q[2] - q[1] * q[1]);
  float temp = -2.0f * q[0] * q[2] + 2.0f * q[1] * q[3];
  // Clamp value between -1 and 1 to prevent NaNs
  p = -asinf(fmaxf(fminf(1.0f, temp), -1.0f));
  y = atan2f(2.0f * q[2] * q[1] + 2.0f * q[3] * q[0], q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
}

inline __host__ __device__ void Quat2DCM(const float q[4], float M[3][3])
{
  M[0][0] = SQ(q[0]) + SQ(q[1]) - SQ(q[2]) - SQ(q[3]);
  M[0][1] = 2 * (q[1] * q[2] - q[0] * q[3]);
  M[0][2] = 2 * (q[1] * q[3] + q[0] * q[2]);
  M[1][0] = 2 * (q[1] * q[2] + q[0] * q[3]);
  M[1][1] = SQ(q[0]) - SQ(q[1]) + SQ(q[2]) - SQ(q[3]);
  M[1][2] = 2 * (q[2] * q[3] - q[0] * q[1]);
  M[2][0] = 2 * (q[1] * q[3] - q[0] * q[2]);
  M[2][1] = 2 * (q[2] * q[3] + q[0] * q[1]);
  M[2][2] = SQ(q[0]) - SQ(q[1]) - SQ(q[2]) + SQ(q[3]);
}

inline __host__ __device__ void QuatSubtract(const Eigen::Quaternionf& q_1, const Eigen::Quaternionf& q_2,
                                             Eigen::Quaternionf& q_3)
{
  q_3 = q_2 * q_1.inverse();
}

inline __host__ __device__ void QuatMultiply(const Eigen::Quaternionf& q_1, const Eigen::Quaternionf& q_2,
                                             Eigen::Quaternionf& q_3, bool normalize = true)
{
  q_3 = q_1 * q_2;
  if (normalize)
  {
    q_3.normalize();
  }
}

inline __host__ __device__ void QuatInv(const Eigen::Quaternionf& q, Eigen::Quaternionf& q_f)
{
  q_f = q.inverse();
}

/*
 * rotates a point by the given quaternion
 */
inline __host__ __device__ void RotatePointByQuat(const float q[4], const float3& point, float3& output)
{
  // converts the point into a quaternion format
  float pq[4] = { 0.0f, point.x, point.y, point.z };
  float q_inv[4];
  float temp[4];
  QuatInv(q, q_inv);
  QuatMultiply(q, pq, temp, false);
  QuatMultiply(temp, q_inv, pq, false);
  // converts the quaternion back into a point
  output = make_float3(pq[1], pq[2], pq[3]);
}

/*
 * rotates a point by the given quaternion
 */
inline __host__ __device__ void RotatePointByQuat(const Eigen::Quaternionf& q, const float3& point, float3& output)
{
  // converts the point into a quaternion format
  Eigen::Quaternionf q_inv, temp, pq;
  pq.w() = 0.0f;
  pq.x() = point.x;
  pq.y() = point.y;
  pq.z() = point.z;
  QuatInv(q, q_inv);
  QuatMultiply(q, pq, temp, false);
  QuatMultiply(temp, q_inv, pq, false);
  // converts the quaternion back into a point
  output = make_float3(pq.x(), pq.y(), pq.z());
}
/*
 * rotates a point by the given quaternion
 */
inline __host__ __device__ void RotatePointByQuat(const Eigen::Quaternionf& q, const Eigen::Ref<Eigen::Vector3f>& point,
                                                  Eigen::Ref<Eigen::Vector3f> output)
{
  output = q * point;
}

/*
 * rotates a point by the given quaternion
 */
inline __host__ __device__ void RotatePointByQuat(const float q[4], float3& point)
{
  RotatePointByQuat(q, point, point);
}

/*
 * rotates a point by the given quaternion
 */
inline __host__ __device__ void RotatePointByQuat(const Eigen::Quaternionf& q, float3& point)
{
  RotatePointByQuat(q, point, point);
}

/*
 * rotates a point by the given quaternion
 */
inline __host__ __device__ void RotatePointByQuat(const Eigen::Quaternionf& q, Eigen::Ref<Eigen::Vector3f> point)
{
  RotatePointByQuat(q, point, point);
}

/*
 * rotates a point by the given DCM Matrix. Transpose is used as M stored in row-major
 */
inline __host__ __device__ void RotatePointByDCM(const float M[3][3], const float3& point, float3& output,
                                                 matrix::MAT_OP operation = matrix::MAT_OP::NONE)
{
  if (operation == matrix::MAT_OP::NONE)
  {
    matrix::gemm1<3, 3, 1, p1::Parallel1Dir::NONE>((float*)M, (const float*)&point, (float*)&output, 1.0f, 0.0f,
                                                   matrix::MAT_OP::TRANSPOSE);
  }
  else if (operation == matrix::MAT_OP::TRANSPOSE)
  {
    matrix::gemm1<3, 3, 1, p1::Parallel1Dir::NONE>((float*)M, (const float*)&point, (float*)&output, 1.0f, 0.0f,
                                                   matrix::MAT_OP::NONE);
  }
}

/*
 * rotates a point by the given DCM Matrix
 */
inline __host__ __device__ void RotatePointByDCM(const Eigen::Ref<Eigen::Matrix3f>& M, const float3& point,
                                                 float3& output, matrix::MAT_OP operation = matrix::MAT_OP::NONE)
{
  Eigen::Map<Eigen::Vector3f> point_eigen((float*)&point);
  Eigen::Map<Eigen::Vector3f> output_eigen((float*)&output);
  if (operation == matrix::MAT_OP::NONE)
  {
    output_eigen = M * point_eigen;
  }
  else if (operation == matrix::MAT_OP::TRANSPOSE)
  {
    output_eigen = M.transpose() * point_eigen;
  }
}

/*
 * rotates a point by the given DCM Matrix
 */
inline __host__ __device__ void RotatePointByDCM(const Eigen::Ref<Eigen::Matrix3f>& M,
                                                 const Eigen::Ref<Eigen::Vector3f>& point,
                                                 Eigen::Ref<Eigen::Vector3f> output,
                                                 matrix::MAT_OP operation = matrix::MAT_OP::NONE)
{
  if (operation == matrix::MAT_OP::NONE)
  {
    output = M * point;
  }
  else if (operation == matrix::MAT_OP::TRANSPOSE)
  {
    output = M.transpose() * point;
  }
}

/*
 * The Euler rotation sequence is 3-2-1 (roll, pitch, yaw) from Body to World
 */
inline __host__ __device__ void Euler2QuatNWU(const float& r, const float& p, const float& y, Eigen::Quaternionf& q)
{
  // double psi = clamp_radians(euler.roll);
  // double theta = clamp_radians(euler.pitch);
  // double phi = clamp_radians(euler.yaw);
  float sin_phi_2, cos_phi_2, sin_theta_2, cos_theta_2, sin_psi_2, cos_psi_2;
#ifdef __CUDA_ARCH__
  float r_norm = angle_utils::normalizeAngle(r * 0.5f);
  float p_norm = angle_utils::normalizeAngle(p * 0.5f);
  float y_norm = angle_utils::normalizeAngle(y * 0.5f);
  __sincosf(r_norm, &sin_phi_2, &cos_phi_2);
  __sincosf(p_norm, &sin_theta_2, &cos_theta_2);
  __sincosf(y_norm, &sin_psi_2, &cos_psi_2);
#else
  sincosf(r * 0.5f, &sin_phi_2, &cos_phi_2);
  sincosf(p * 0.5f, &sin_theta_2, &cos_theta_2);
  sincosf(y * 0.5f, &sin_psi_2, &cos_psi_2);
#endif

  q.w() = cos_phi_2 * cos_theta_2 * cos_psi_2 + sin_phi_2 * sin_theta_2 * sin_psi_2;
  q.x() = -cos_phi_2 * sin_theta_2 * sin_psi_2 + cos_theta_2 * cos_psi_2 * sin_phi_2;
  q.y() = cos_phi_2 * cos_psi_2 * sin_theta_2 + sin_phi_2 * cos_theta_2 * sin_psi_2;
  q.z() = cos_phi_2 * cos_theta_2 * sin_psi_2 - sin_phi_2 * cos_psi_2 * sin_theta_2;
}

/*
 * The Euler rotation sequence is 3-2-1 (roll, pitch, yaw) from Body to World
 */
inline __host__ __device__ void Euler2DCM_NWU(const float& r, const float& p, const float& y, float M[3][3])
{
  float sin_phi, cos_phi, sin_theta, cos_theta, sin_psi, cos_psi;
#ifdef __CUDA_ARCH__
  float r_norm = angle_utils::normalizeAngle(r);
  float p_norm = angle_utils::normalizeAngle(p);
  float y_norm = angle_utils::normalizeAngle(y);
  __sincosf(r_norm, &sin_phi, &cos_phi);
  __sincosf(p_norm, &sin_theta, &cos_theta);
  __sincosf(y_norm, &sin_psi, &cos_psi);
#else
  sincosf(r, &sin_phi, &cos_phi);
  sincosf(p, &sin_theta, &cos_theta);
  sincosf(y, &sin_psi, &cos_psi);
#endif

  M[0][0] = cos_theta * cos_psi;
  M[0][1] = sin_phi * sin_theta * cos_psi - cos_phi * sin_psi;
  M[0][2] = cos_phi * sin_theta * cos_psi + sin_phi * sin_psi;
  M[1][0] = cos_theta * sin_psi;
  M[1][1] = sin_phi * sin_theta * sin_psi + cos_phi * cos_psi;
  M[1][2] = cos_phi * sin_theta * sin_psi - sin_phi * cos_psi;
  M[2][0] = -sin_theta;
  M[2][1] = sin_phi * cos_theta;
  M[2][2] = cos_phi * cos_theta;
}

/*
 * The Euler rotation sequence is 3-2-1 (roll, pitch, yaw) from Body to World
 */
inline __host__ __device__ void Euler2DCM_NWU(const float& r, const float& p, const float& y,
                                              Eigen::Ref<Eigen::Matrix3f> M)
{
  float sin_phi, cos_phi, sin_theta, cos_theta, sin_psi, cos_psi;
#ifdef __CUDA_ARCH__
  float r_norm = angle_utils::normalizeAngle(r);
  float p_norm = angle_utils::normalizeAngle(p);
  float y_norm = angle_utils::normalizeAngle(y);
  __sincosf(r_norm, &sin_phi, &cos_phi);
  __sincosf(p_norm, &sin_theta, &cos_theta);
  __sincosf(y_norm, &sin_psi, &cos_psi);
#else
  sincosf(r, &sin_phi, &cos_phi);
  sincosf(p, &sin_theta, &cos_theta);
  sincosf(y, &sin_psi, &cos_psi);
#endif

  M(0, 0) = cos_theta * cos_psi;
  M(0, 1) = sin_phi * sin_theta * cos_psi - cos_phi * sin_psi;
  M(0, 2) = cos_phi * sin_theta * cos_psi + sin_phi * sin_psi;
  M(1, 0) = cos_theta * sin_psi;
  M(1, 1) = sin_phi * sin_theta * sin_psi + cos_phi * cos_psi;
  M(1, 2) = cos_phi * sin_theta * sin_psi - sin_phi * cos_psi;
  M(2, 0) = -sin_theta;
  M(2, 1) = sin_phi * cos_theta;
  M(2, 2) = cos_phi * cos_theta;
}

// (RPY rotation sequence)
/*
 * Returns an euler sequence 3-2-1 (roll pitch yaw) that when applied takes you from body to world
 */
inline void __host__ __device__ Quat2EulerNWU(const Eigen::Quaternionf& q, float& r, float& p, float& y)
{
  r = atan2f(2.0f * q.z() * q.y() + 2.0f * q.w() * q.x(),
             q.w() * q.w() + q.z() * q.z() - q.y() * q.y() - q.x() * q.x());
  float temp = -2.0f * q.w() * q.y() + 2.0f * q.x() * q.z();
  p = -asinf(fmaxf(-1.0f, fminf(temp, 1.0f)));
  y = atan2f(2.0f * q.y() * q.x() + 2.0f * q.z() * q.w(),
             q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z());
}

inline void Quat2DCM(const Eigen::Quaternionf& q, Eigen::Ref<Eigen::Matrix3f> DCM)
{
  DCM = q.toRotationMatrix();
}

inline __device__ void omega2edot(const float p, const float q, const float r, const float e[4], float ed[4])
{
  ed[0] = 0.5f * (-p * e[1] - q * e[2] - r * e[3]);
  ed[1] = 0.5f * (p * e[0] - q * e[3] + r * e[2]);
  ed[2] = 0.5f * (p * e[3] + q * e[0] - r * e[1]);
  ed[3] = 0.5f * (-p * e[2] + q * e[1] + r * e[0]);
}

// Can't use Eigen::Ref on Quaternions
inline void omega2edot(const float p, const float q, const float r, const Eigen::Quaternionf& e, Eigen::Quaternionf& ed)
{
  ed.w() = 0.5f * (-p * e.x() - q * e.y() - r * e.z());
  ed.x() = 0.5f * (p * e.w() - q * e.z() + r * e.y());
  ed.y() = 0.5f * (p * e.z() + q * e.w() - r * e.x());
  ed.z() = 0.5f * (-p * e.y() + q * e.x() + r * e.w());
}

inline __host__ __device__ void bodyOffsetToWorldPoseQuat(const float3& offset, const float3& body_pose,
                                                          const float q[4], float3& output)
{
  // rotate body vector into world frame
  float3 rotated_offset = make_float3(offset.x, offset.y, offset.z);
  RotatePointByQuat(q, rotated_offset);
  // add offset to body pose
  output.x = body_pose.x + rotated_offset.x;
  output.y = body_pose.y + rotated_offset.y;
  output.z = body_pose.z + rotated_offset.z;
}

inline __host__ __device__ void bodyOffsetToWorldPoseQuat(const float3& offset, const float3& body_pose,
                                                          const Eigen::Quaternionf& q, float3& output)
{
  // rotate body vector into world frame
  float3 rotated_offset = make_float3(offset.x, offset.y, offset.z);
  RotatePointByQuat(q, rotated_offset);
  // add offset to body pose
  output.x = body_pose.x + rotated_offset.x;
  output.y = body_pose.y + rotated_offset.y;
  output.z = body_pose.z + rotated_offset.z;
}

inline __host__ __device__ void bodyOffsetToWorldPoseQuat(const Eigen::Ref<Eigen::Vector3f>& offset,
                                                          const Eigen::Ref<Eigen::Vector3f>& body_pose,
                                                          const Eigen::Quaternionf& q,
                                                          Eigen::Ref<Eigen::Vector3f> output)
{
  RotatePointByQuat(q, offset, output);
  // add offset to body pose
  output += body_pose;
}

inline __host__ __device__ void bodyOffsetToWorldPoseEuler(const float3& offset, const float3& body_pose,
                                                           const float3& rotation, float3& output)
{
  // convert RPY to Rotation Matrix
  float M[3][3];
  math::Euler2DCM_NWU(rotation.x, rotation.y, rotation.z, M);
  RotatePointByDCM(M, offset, output);

  // add offset to body pose
  output.x += body_pose.x;
  output.y += body_pose.y;
  output.z += body_pose.z;
}

inline __host__ __device__ void bodyOffsetToWorldPoseEuler(const float3& offset, const float3& body_pose,
                                                           const Eigen::Ref<Eigen::Vector3f>& rotation, float3& output)
{
  // convert RPY to Rotation Matrix
  float M[3][3];
  math::Euler2DCM_NWU(rotation.x(), rotation.y(), rotation.z(), M);
  RotatePointByDCM(M, offset, output);

  // add offset to body pose
  output.x += body_pose.x;
  output.y += body_pose.y;
  output.z += body_pose.z;
}

inline __host__ __device__ void bodyOffsetToWorldPoseEuler(const Eigen::Ref<Eigen::Vector3f>& offset,
                                                           const Eigen::Ref<Eigen::Vector3f>& body_pose,
                                                           const Eigen::Ref<Eigen::Vector3f>& rotation,
                                                           Eigen::Ref<Eigen::Vector3f> output)
{
  // convert RPY to Rotation Matrix
  Eigen::Matrix3f M;
  math::Euler2DCM_NWU(rotation.x(), rotation.y(), rotation.z(), M);
  RotatePointByDCM(M, offset, output);
  // add offset to body pose
  output += body_pose;
}

inline __host__ __device__ void bodyOffsetToWorldPoseDCM(const float3& offset, const float3& body_pose,
                                                         const float rotation[3][3], float3& output)
{
  RotatePointByDCM(rotation, offset, output);

  // add offset to body pose
  output.x += body_pose.x;
  output.y += body_pose.y;
  output.z += body_pose.z;
}

inline __host__ __device__ void bodyOffsetToWorldPoseDCM(const float3& offset, const float3& body_pose,
                                                         const Eigen::Ref<Eigen::Matrix3f>& rotation, float3& output)
{
  RotatePointByDCM(rotation, offset, output);

  // add offset to body pose
  output.x += body_pose.x;
  output.y += body_pose.y;
  output.z += body_pose.z;
}

inline __host__ __device__ void bodyOffsetToWorldPoseDCM(const Eigen::Ref<Eigen::Vector3f>& offset,
                                                         const Eigen::Ref<Eigen::Vector3f>& body_pose,
                                                         const Eigen::Ref<Eigen::Matrix3f>& rotation,
                                                         Eigen::Ref<Eigen::Vector3f> output)
{
  RotatePointByDCM(rotation, offset, output);

  // add offset to body pose
  output += body_pose;
}

inline __device__ __host__ Eigen::Matrix3f skewSymmetricMatrix(Eigen::Vector3f& v)
{
  Eigen::Matrix3f m;
  m << 0.0f, -v[2], v[1], v[2], 0.0f, -v[0], -v[1], v[0], 0.0f;
  return m;
}

inline __host__ double timeDiffms(const std::chrono::steady_clock::time_point& end,
                                  const std::chrono::steady_clock::time_point& start)
{
  return (end - start).count() / 1e6;
}

inline __host__ __device__ double normalCDF(double x)
{
  return 0.5 * erfc(-x * M_SQRT1_2);
}

inline __host__ std::vector<double> calculateCk(size_t steps)
{
  // calculate params only when more steps are required
  static std::vector<double> c_vec;
  if (c_vec.size() < steps)
  {
    c_vec.resize(steps, 0);
    c_vec[0] = 1.0;
    for (size_t k = 1; k <= steps; k++)
    {
      double c_k = 0;
      for (size_t m = 0; m < k; m++)
      {
        c_k += c_vec[m] * c_vec[k - 1 - m] / ((m + 1.0) * (2.0 * m + 1.0));
      }
      c_vec[k] = c_k;
    }
  }
  return c_vec;
}

/**
 * Implementation based on
 * https://en.wikipedia.org/wiki/Error_function#Inverse_functions and Horner's
 * method
 */
inline __host__ double inverseErrorFunc(double x, int num_precision = 5)
{
  std::vector<double> c_k = calculateCk(num_precision);
  double output = 0;
  for (int i = num_precision; i > 0; i--)
  {
    output = (c_k[i] / (2.0 * i + 1.0) + output) * x * x * M_PI / 4.0;
  }
  output = (output + c_k[0]) * x / M_2_SQRTPI;
  return output;
}

inline __host__ double inverseErrorFuncSlow(double x, int num_precision = 5)
{
  std::vector<double> c_k = calculateCk(num_precision);
  double slow_output = 0;
  for (int i = 0; i <= num_precision; i++)
  {
    slow_output += c_k[i] / (2.0 * i + 1.0) * std::pow(x / M_2_SQRTPI, 2 * i + 1);
  }
  return slow_output;
}

/**
 * https://en.wikipedia.org/wiki/Normal_distribution#Quantile_function
 */
inline __host__ double inverseNormalCDF(double x, int num_precision = 10)
{
  return M_SQRT2 * inverseErrorFunc(2.0 * x - 1.0, num_precision);
}

inline __host__ double inverseNormalCDFSlow(double x, int num_precision = 10)
{
  return M_SQRT2 * inverseErrorFuncSlow(2.0 * x - 1.0, num_precision);
}

inline __device__ __host__ float clamp(float value, float min, float max)
{
  return fminf(fmaxf(value, min), max);
}

inline __device__ __host__ float sign(float value)
{
  return value >= 0 ? 1 : -1;
}

}  // namespace math

}  // namespace mppi

#endif  // MATH_UTILS_H_
