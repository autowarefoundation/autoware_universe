/**
 * Project a vehicle pose (x, y) onto a Path2D.
 * Analytic circle/line use closed-form projection; polylines use segment search + Newton refinement.
 */
#pragma once

#include <mppi/path/path2d.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace mppi
{
namespace path
{

struct PathProjection
{
  float arc_length_s = 0.0F;
  float distance = 0.0F;
  /** Signed lateral offset: positive = left of path tangent (inside for CCW circle). */
  float signed_lateral_error = 0.0F;
  Pose2D footpoint{};
};

namespace detail
{

inline float distSquaredPointToSegment(const float px, const float py, const float ax, const float ay, const float bx,
                                       const float by, float& t_clamped)
{
  const float abx = bx - ax;
  const float aby = by - ay;
  const float apx = px - ax;
  const float apy = py - ay;
  const float ab2 = abx * abx + aby * aby;
  float t = 0.0F;
  if (ab2 > 1.0E-12F)
  {
    t = (apx * abx + apy * aby) / ab2;
  }
  t = std::max(0.0F, std::min(1.0F, t));
  t_clamped = t;
  const float cx = ax + t * abx;
  const float cy = ay + t * aby;
  const float dx = px - cx;
  const float dy = py - cy;
  return dx * dx + dy * dy;
}

inline float arcLengthOnSegment(const Path2D& path, const int seg, const float t)
{
  const auto& a = path.anchors();
  const float s0 = a[static_cast<size_t>(seg)].s;
  const float s1 = a[static_cast<size_t>(seg + 1)].s;
  return s0 + t * (s1 - s0);
}

inline float newtonStep(const Path2D& path, const float px, const float py, const float s)
{
  const Pose2D p = path.poseAt(s);
  float tx = 0.0F;
  float ty = 0.0F;
  path.tangentAt(s, tx, ty);
  constexpr float ds = 0.05F;
  float tx_p = 0.0F;
  float ty_p = 0.0F;
  path.tangentAt(s + ds, tx_p, ty_p);
  const float ddx = (tx_p - tx) / ds;
  const float ddy = (ty_p - ty) / ds;

  const float ex = p.x - px;
  const float ey = p.y - py;
  const float fp = ex * tx + ey * ty;
  const float fpp = tx * tx + ty * ty + ex * ddx + ey * ddy;
  if (std::fabs(fpp) < 1.0E-8F)
  {
    return s;
  }
  return s - fp / fpp;
}

inline float wrapArcLengthDelta(const float a, const float b, const float period)
{
  float d = a - b;
  d = d - std::round(d / period) * period;
  return d;
}

inline PathProjection projectOntoCircle(const Path2D& path, const float px, const float py,
                                        const float s_hint = std::numeric_limits<float>::quiet_NaN())
{
  PathProjection result;
  const CircleGeometry& c = path.circleGeometry();
  const float dx = px - c.center_x;
  const float dy = py - c.center_y;
  const float rho = std::sqrt(dx * dx + dy * dy);
  const float theta = std::atan2(dy, dx);

  const float ang = detail::positiveAngleFromTheta0(theta, c.theta0);
  float s = c.radius * ang;

  if (!std::isnan(s_hint))
  {
    const float period = path.length();
    float best_s = s;
    float best_cost = std::fabs(wrapArcLengthDelta(s, s_hint, period));
    for (int k = -2; k <= 2; ++k)
    {
      const float candidate = s + static_cast<float>(k) * period;
      const float cost = std::fabs(wrapArcLengthDelta(candidate, s_hint, period));
      if (cost < best_cost)
      {
        best_cost = cost;
        best_s = candidate;
      }
    }
    s = path.wrapArcLength(best_s);
  }
  else
  {
    s = path.wrapArcLength(s);
  }

  result.arc_length_s = s;
  result.footpoint = path.poseAt(s);
  float tx = 0.0F;
  float ty = 0.0F;
  path.tangentAt(s, tx, ty);
  const float ex = px - result.footpoint.x;
  const float ey = py - result.footpoint.y;
  result.distance = std::sqrt(ex * ex + ey * ey);
  result.signed_lateral_error = c.radius - rho;
  return result;
}

inline PathProjection projectOntoLine(const Path2D& path, const float px, const float py)
{
  PathProjection result;
  const Pose2D p0 = path.poseAt(0.0F);
  const Pose2D p1 = path.poseAt(path.length());
  const float segx = p1.x - p0.x;
  const float segy = p1.y - p0.y;
  const float seg_len2 = segx * segx + segy * segy;
  float t = 0.0F;
  if (seg_len2 > 1.0E-12F)
  {
    t = ((px - p0.x) * segx + (py - p0.y) * segy) / seg_len2;
  }
  t = std::max(0.0F, std::min(1.0F, t));
  result.arc_length_s = t * path.length();
  result.footpoint.x = p0.x + t * segx;
  result.footpoint.y = p0.y + t * segy;
  result.footpoint.yaw = p0.yaw;
  float tx = 0.0F;
  float ty = 0.0F;
  path.tangentAt(result.arc_length_s, tx, ty);
  const float ex = px - result.footpoint.x;
  const float ey = py - result.footpoint.y;
  result.distance = std::sqrt(ex * ex + ey * ey);
  result.signed_lateral_error = -ex * ty + ey * tx;
  return result;
}

inline PathProjection projectOntoPolyline(const Path2D& path, const float px, const float py,
                                          const float s_hint = std::numeric_limits<float>::quiet_NaN(),
                                          const int newton_iters = 6)
{
  PathProjection result;
  const auto& anchors = path.anchors();
  const int n_seg = static_cast<int>(anchors.size()) - 1;

  float best_d2 = std::numeric_limits<float>::max();
  float best_s = 0.0F;

  auto consider_segment = [&](const int seg) {
    float t = 0.0F;
    const float ax = anchors[static_cast<size_t>(seg)].x;
    const float ay = anchors[static_cast<size_t>(seg)].y;
    const float bx = anchors[static_cast<size_t>(seg + 1)].x;
    const float by = anchors[static_cast<size_t>(seg + 1)].y;
    const float d2 = distSquaredPointToSegment(px, py, ax, ay, bx, by, t);
    if (d2 < best_d2)
    {
      best_d2 = d2;
      best_s = arcLengthOnSegment(path, seg, t);
    }
  };

  if (!std::isnan(s_hint))
  {
    const float window = std::max(5.0F, 0.15F * path.length());
    const float s_lo = path.wrapArcLength(s_hint - window);
    const float s_hi = path.wrapArcLength(s_hint + window);
    for (int seg = 0; seg < n_seg; ++seg)
    {
      const float s0 = anchors[static_cast<size_t>(seg)].s;
      const float s1 = anchors[static_cast<size_t>(seg + 1)].s;
      if (s1 >= s_lo && s0 <= s_hi)
      {
        consider_segment(seg);
      }
    }
  }

  if (best_d2 >= std::numeric_limits<float>::max() * 0.5F)
  {
    for (int seg = 0; seg < n_seg; ++seg)
    {
      consider_segment(seg);
    }
  }

  float s = best_s;
  const float s_min = 0.0F;
  const float s_max = path.length();
  for (int k = 0; k < newton_iters; ++k)
  {
    s = newtonStep(path, px, py, s);
    if (path.closed())
    {
      s = path.wrapArcLength(s);
    }
    else
    {
      s = std::max(s_min, std::min(s, s_max));
    }
  }

  result.arc_length_s = s;
  result.footpoint = path.poseAt(s);
  float tx = 0.0F;
  float ty = 0.0F;
  path.tangentAt(s, tx, ty);
  const float dx = px - result.footpoint.x;
  const float dy = py - result.footpoint.y;
  result.distance = std::sqrt(dx * dx + dy * dy);
  result.signed_lateral_error = -dx * ty + dy * tx;
  return result;
}

}  // namespace detail

inline PathProjection projectPoseOntoPath(const Path2D& path, const float px, const float py,
                                          const float s_hint = std::numeric_limits<float>::quiet_NaN(),
                                          const int newton_iters = 6)
{
  PathProjection result;
  if (path.empty())
  {
    return result;
  }

  if (path.isAnalyticCircle())
  {
    return detail::projectOntoCircle(path, px, py, s_hint);
  }
  if (path.isAnalyticLine())
  {
    return detail::projectOntoLine(path, px, py);
  }
  return detail::projectOntoPolyline(path, px, py, s_hint, newton_iters);
}

}  // namespace path
}  // namespace mppi
