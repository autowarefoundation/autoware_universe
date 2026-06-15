/**
 * Shared host/device geometry for analytic path-tracking costs (polyline distance, OBB overlap).
 */
#pragma once

#ifndef MPPI_COST_FUNCTIONS_PATH_TRACKING_GEOMETRY_CUH_
#define MPPI_COST_FUNCTIONS_PATH_TRACKING_GEOMETRY_CUH_

#include <algorithm>
#include <cmath>

namespace mppi
{
namespace cost
{
namespace detail
{
#ifdef __CUDA_ARCH__
__device__ inline float clampUnitInterval(const float t)
{
  return fmaxf(0.0F, fminf(1.0F, t));
}

__device__ inline float vectorLength(const float dx, const float dy)
{
  return sqrtf(dx * dx + dy * dy);
}
#else
inline float clampUnitInterval(const float t)
{
  return std::max(0.0F, std::min(1.0F, t));
}

inline float vectorLength(const float dx, const float dy)
{
  return std::sqrt(dx * dx + dy * dy);
}
#endif

__host__ __device__ inline float distancePointToSegment(
  const float px, const float py, const float x0, const float y0, const float x1, const float y1)
{
  const float dx = x1 - x0;
  const float dy = y1 - y0;
  const float len_sq = dx * dx + dy * dy;
  if (len_sq < 1.0E-8F) {
    return vectorLength(px - x0, py - y0);
  }

  const float t = clampUnitInterval(((px - x0) * dx + (py - y0) * dy) / len_sq);
  return vectorLength(px - (x0 + t * dx), py - (y0 + t * dy));
}

/** Signed lateral offset from segment; positive = left of forward tangent. */
__host__ __device__ inline float signedLateralOffsetPointToSegment(
  const float px, const float py, const float x0, const float y0, const float x1, const float y1)
{
  const float dx = x1 - x0;
  const float dy = y1 - y0;
  const float len_sq = dx * dx + dy * dy;
  if (len_sq < 1.0E-8F) {
    return px - x0;
  }

  const float t = clampUnitInterval(((px - x0) * dx + (py - y0) * dy) / len_sq);
  const float cx = x0 + t * dx;
  const float cy = y0 + t * dy;
  const float len = vectorLength(dx, dy);
  return ((px - cx) * (-dy) + (py - cy) * dx) / len;
}

__host__ __device__ inline float dot2(
  const float ax, const float ay, const float bx, const float by)
{
  return ax * bx + ay * by;
}

/** Separating-axis test for two oriented boxes (body x = forward, y = left). */
__host__ __device__ inline bool orientedBoxesOverlap(
  const float acx, const float acy, const float a_cos, const float a_sin, const float a_hl,
  const float a_hw, const float bcx, const float bcy, const float b_cos, const float b_sin,
  const float b_hl, const float b_hw)
{
  const float tx = bcx - acx;
  const float ty = bcy - acy;

  const float a0x = a_cos;
  const float a0y = a_sin;
  const float a1x = -a_sin;
  const float a1y = a_cos;
  const float b0x = b_cos;
  const float b0y = b_sin;
  const float b1x = -b_sin;
  const float b1y = b_cos;

#ifdef __CUDA_ARCH__
#define MPPI_COST_ABS(x) fabsf(x)
#else
#define MPPI_COST_ABS(x) std::fabs(x)
#endif

  float t = MPPI_COST_ABS(dot2(tx, ty, a0x, a0y));
  float r = a_hl + MPPI_COST_ABS(dot2(b0x, b0y, a0x, a0y)) * b_hl +
            MPPI_COST_ABS(dot2(b1x, b1y, a0x, a0y)) * b_hw;
  if (t > r) {
    return false;
  }

  t = MPPI_COST_ABS(dot2(tx, ty, a1x, a1y));
  r = a_hw + MPPI_COST_ABS(dot2(b0x, b0y, a1x, a1y)) * b_hl +
      MPPI_COST_ABS(dot2(b1x, b1y, a1x, a1y)) * b_hw;
  if (t > r) {
    return false;
  }

  t = MPPI_COST_ABS(dot2(tx, ty, b0x, b0y));
  r = b_hl + MPPI_COST_ABS(dot2(a0x, a0y, b0x, b0y)) * a_hl +
      MPPI_COST_ABS(dot2(a1x, a1y, b0x, b0y)) * a_hw;
  if (t > r) {
    return false;
  }

  t = MPPI_COST_ABS(dot2(tx, ty, b1x, b1y));
  r = b_hw + MPPI_COST_ABS(dot2(a0x, a0y, b1x, b1y)) * a_hl +
      MPPI_COST_ABS(dot2(a1x, a1y, b1x, b1y)) * a_hw;
  if (t > r) {
    return false;
  }

#undef MPPI_COST_ABS
  return true;
}
}  // namespace detail
}  // namespace cost
}  // namespace mppi

#endif  // MPPI_COST_FUNCTIONS_PATH_TRACKING_GEOMETRY_CUH_
