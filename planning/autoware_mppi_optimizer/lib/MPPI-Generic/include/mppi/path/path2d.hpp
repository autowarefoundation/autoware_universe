/**
 * Planar reference paths for Dubins / bicycle path tracking.
 *
 * Supports exact analytic primitives (circle, straight line) and polyline paths
 * (stadium, Catmull–Rom). Analytic paths use true arc-length geometry for pose,
 * tangent, curvature, and projection; polyline anchors are optional and only used
 * for visualization export.
 */
#pragma once

#include <mppi/utils/angle_utils.cuh>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

namespace mppi
{
namespace path
{

struct Pose2D
{
  float x = 0.0F;
  float y = 0.0F;
  float yaw = 0.0F;
};

/** Anchor on the path with cumulative arc length s [m] from the start. */
struct PathAnchor : Pose2D
{
  float s = 0.0F;
};

enum class PathGeometryKind
{
  Polyline,
  Circle,
  Line
};

struct CircleGeometry
{
  float center_x = 0.0F;
  float center_y = 0.0F;
  float radius = 1.0F;
  /** Polar angle at s = 0; CCW motion, tangent yaw = theta + pi/2. */
  float theta0 = 0.0F;
};

struct LineGeometry
{
  float x0 = 0.0F;
  float y0 = 0.0F;
  float x1 = 0.0F;
  float y1 = 0.0F;
  float yaw = 0.0F;
  float length = 0.0F;
};

class Path2D
{
public:
  PathGeometryKind geometryKind() const
  {
    return geometry_;
  }

  bool isAnalyticCircle() const
  {
    return geometry_ == PathGeometryKind::Circle;
  }

  bool isAnalyticLine() const
  {
    return geometry_ == PathGeometryKind::Line;
  }

  const CircleGeometry& circleGeometry() const
  {
    return circle_;
  }

  bool closed() const
  {
    return closed_;
  }

  float length() const
  {
    return length_;
  }

  /** Polyline samples for plotting only (analytic paths are densely sampled from exact geometry). */
  const std::vector<PathAnchor>& anchors() const
  {
    return anchors_;
  }

  bool empty() const
  {
    if (geometry_ == PathGeometryKind::Circle)
    {
      return circle_.radius < 1.0E-6F;
    }
    if (geometry_ == PathGeometryKind::Line)
    {
      return line_.length < 1.0E-6F;
    }
    return anchors_.size() < 2U;
  }

  float wrapArcLength(float s) const;

  Pose2D poseAt(float s) const;

  void tangentAt(float s, float& tx, float& ty) const;

  float curvatureAt(float s) const;

  /** θ(s) for analytic circle; undefined for other geometries. */
  float circleThetaAt(float s) const;

  static Path2D straightLine(float x0, float y0, float x1, float y1, int plot_samples = 64);

  /**
   * Exact circle: CCW, yaw tangent = θ + π/2, s = 0 at θ = theta0, length = 2πR.
   * @param plot_samples  Polyline density for CSV/plot export only (not used in tracking).
   */
  static Path2D circle(float center_x, float center_y, float radius, float theta0 = 0.0F,
                       int plot_samples = 256);

  static Path2D stadium(float straight_length, float radius, int samples_per_arc = 48);

  static Path2D catmullRom(const std::vector<std::pair<float, float>>& control_xy, bool closed,
                           int samples_per_span = 16);

private:
  PathGeometryKind geometry_ = PathGeometryKind::Polyline;
  bool closed_ = false;
  float length_ = 0.0F;
  CircleGeometry circle_{};
  LineGeometry line_{};
  std::vector<PathAnchor> anchors_;

  void finalizeFromPoses(std::vector<Pose2D> poses, bool closed);
  void buildPlotAnchorsFromGeometry(int num_samples);

  int segmentIndexForS(float s_wrapped, float& local_u) const;

  Pose2D poseOnCircle(float s) const;
  Pose2D poseOnLine(float s) const;
  void tangentOnCircle(float s, float& tx, float& ty) const;
  void tangentOnLine(float s, float& tx, float& ty) const;
};

namespace detail
{
inline float positiveAngleFromTheta0(const float theta, const float theta0)
{
  float d = angle_utils::normalizeAngle(theta - theta0);
  if (d < 0.0F)
  {
    d += 2.0F * static_cast<float>(M_PI);
  }
  return d;
}
}  // namespace detail

inline float Path2D::circleThetaAt(const float s) const
{
  return circle_.theta0 + s / circle_.radius;
}

inline Pose2D Path2D::poseOnCircle(const float s) const
{
  const float sw = wrapArcLength(s);
  const float theta = circleThetaAt(sw);
  Pose2D out;
  out.x = circle_.center_x + circle_.radius * std::cos(theta);
  out.y = circle_.center_y + circle_.radius * std::sin(theta);
  out.yaw = angle_utils::normalizeAngle(theta + static_cast<float>(M_PI) * 0.5F);
  return out;
}

inline Pose2D Path2D::poseOnLine(const float s) const
{
  const float sw = wrapArcLength(s);
  const float u = sw / std::max(line_.length, 1.0E-9F);
  Pose2D out;
  out.x = line_.x0 + u * (line_.x1 - line_.x0);
  out.y = line_.y0 + u * (line_.y1 - line_.y0);
  out.yaw = line_.yaw;
  return out;
}

inline void Path2D::tangentOnCircle(const float s, float& tx, float& ty) const
{
  const float theta = circleThetaAt(wrapArcLength(s));
  tx = -std::sin(theta);
  ty = std::cos(theta);
}

inline void Path2D::tangentOnLine(const float, float& tx, float& ty) const
{
  tx = std::cos(line_.yaw);
  ty = std::sin(line_.yaw);
}

inline float Path2D::wrapArcLength(float s) const
{
  if (geometry_ == PathGeometryKind::Circle)
  {
    if (circle_.radius < 1.0E-6F)
    {
      return 0.0F;
    }
    const float two_pi_r = length_;
    s = s - std::floor(s / two_pi_r) * two_pi_r;
    if (s < 0.0F)
    {
      s += two_pi_r;
    }
    return s;
  }
  if (geometry_ == PathGeometryKind::Line)
  {
    return std::max(0.0F, std::min(s, length_));
  }
  if (anchors_.size() < 2U)
  {
    return 0.0F;
  }
  if (closed_)
  {
    if (length_ < 1.0E-6F)
    {
      return 0.0F;
    }
    s = s - std::floor(s / length_) * length_;
    if (s < 0.0F)
    {
      s += length_;
    }
    return s;
  }
  return std::max(0.0F, std::min(s, length_));
}

inline int Path2D::segmentIndexForS(float s_wrapped, float& local_u) const
{
  const int n = static_cast<int>(anchors_.size());
  if (n < 2)
  {
    local_u = 0.0F;
    return 0;
  }
  if (s_wrapped <= anchors_.front().s)
  {
    local_u = 0.0F;
    return 0;
  }
  for (int i = 0; i < n - 1; ++i)
  {
    const float s1 = anchors_[i + 1].s;
    if (s_wrapped <= s1)
    {
      const float s0 = anchors_[i].s;
      const float ds = std::max(s1 - s0, 1.0E-9F);
      local_u = (s_wrapped - s0) / ds;
      return i;
    }
  }
  local_u = 1.0F;
  return n - 2;
}

inline Pose2D Path2D::poseAt(float s) const
{
  if (geometry_ == PathGeometryKind::Circle)
  {
    return poseOnCircle(s);
  }
  if (geometry_ == PathGeometryKind::Line)
  {
    return poseOnLine(s);
  }
  if (anchors_.size() < 2U)
  {
    return {};
  }
  const float sw = wrapArcLength(s);
  float u = 0.0F;
  const int seg = segmentIndexForS(sw, u);
  const PathAnchor& a0 = anchors_[seg];
  const PathAnchor& a1 = anchors_[seg + 1];
  Pose2D out;
  out.x = a0.x + u * (a1.x - a0.x);
  out.y = a0.y + u * (a1.y - a0.y);
  out.yaw = angle_utils::interpolateEulerAngleLinear(a0.yaw, a1.yaw, u);
  return out;
}

inline void Path2D::tangentAt(float s, float& tx, float& ty) const
{
  if (geometry_ == PathGeometryKind::Circle)
  {
    tangentOnCircle(s, tx, ty);
    return;
  }
  if (geometry_ == PathGeometryKind::Line)
  {
    tangentOnLine(s, tx, ty);
    return;
  }
  constexpr float ds = 0.05F;
  const float sw = wrapArcLength(s);
  const Pose2D p_m = poseAt(sw - ds);
  const Pose2D p_p = poseAt(sw + ds);
  const float dx = p_p.x - p_m.x;
  const float dy = p_p.y - p_m.y;
  const float n = std::sqrt(dx * dx + dy * dy);
  if (n < 1.0E-9F)
  {
    const float c = std::cos(poseAt(sw).yaw);
    const float sn = std::sin(poseAt(sw).yaw);
    tx = c;
    ty = sn;
    return;
  }
  tx = dx / n;
  ty = dy / n;
}

inline float Path2D::curvatureAt(float s) const
{
  if (geometry_ == PathGeometryKind::Circle)
  {
    (void)s;
    return 1.0F / circle_.radius;
  }
  if (geometry_ == PathGeometryKind::Line)
  {
    (void)s;
    return 0.0F;
  }
  constexpr float ds = 0.1F;
  const float sw = wrapArcLength(s);
  const float yaw_m = poseAt(sw - ds).yaw;
  const float yaw_p = poseAt(sw + ds).yaw;
  return angle_utils::shortestAngularDistance(yaw_m, yaw_p) / (2.0F * ds);
}

inline void Path2D::buildPlotAnchorsFromGeometry(const int num_samples)
{
  const int n = std::max(2, num_samples);
  anchors_.clear();
  anchors_.reserve(static_cast<size_t>(n));
  for (int i = 0; i < n; ++i)
  {
    const float s = length_ * static_cast<float>(i) / static_cast<float>(n - 1);
    const Pose2D p = poseAt(s);
    PathAnchor a;
    a.x = p.x;
    a.y = p.y;
    a.yaw = p.yaw;
    a.s = s;
    anchors_.push_back(a);
  }
  if (geometry_ == PathGeometryKind::Circle && n >= 2)
  {
    anchors_.back() = anchors_.front();
    anchors_.back().s = length_;
  }
}

inline void Path2D::finalizeFromPoses(std::vector<Pose2D> poses, const bool closed)
{
  geometry_ = PathGeometryKind::Polyline;
  if (poses.size() < 2U)
  {
    throw std::invalid_argument("Path2D: need at least two poses");
  }
  closed_ = closed;
  anchors_.clear();
  anchors_.reserve(poses.size());
  float s_acc = 0.0F;
  for (size_t i = 0; i < poses.size(); ++i)
  {
    PathAnchor a;
    a.x = poses[i].x;
    a.y = poses[i].y;
    a.yaw = poses[i].yaw;
    a.s = s_acc;
    anchors_.push_back(a);
    if (i + 1U < poses.size())
    {
      const float dx = poses[i + 1].x - poses[i].x;
      const float dy = poses[i + 1].y - poses[i].y;
      s_acc += std::sqrt(dx * dx + dy * dy);
    }
  }
  length_ = s_acc;
  if (closed_ && poses.size() >= 2U)
  {
    const float dx = poses.front().x - poses.back().x;
    const float dy = poses.front().y - poses.back().y;
    length_ = anchors_.back().s + std::sqrt(dx * dx + dy * dy);
    PathAnchor close_pt = anchors_.front();
    close_pt.s = length_;
    anchors_.push_back(close_pt);
  }
}

inline Path2D Path2D::straightLine(const float x0, const float y0, const float x1, const float y1,
                                     const int plot_samples)
{
  Path2D path;
  path.geometry_ = PathGeometryKind::Line;
  path.closed_ = false;
  path.line_.x0 = x0;
  path.line_.y0 = y0;
  path.line_.x1 = x1;
  path.line_.y1 = y1;
  const float dx = x1 - x0;
  const float dy = y1 - y0;
  path.line_.length = std::sqrt(dx * dx + dy * dy);
  path.line_.yaw = std::atan2(dy, dx);
  path.length_ = path.line_.length;
  path.buildPlotAnchorsFromGeometry(plot_samples);
  return path;
}

inline Path2D Path2D::circle(const float center_x, const float center_y, const float radius, const float theta0,
                               const int plot_samples)
{
  if (radius < 1.0E-6F)
  {
    throw std::invalid_argument("Path2D::circle: radius must be positive");
  }
  Path2D path;
  path.geometry_ = PathGeometryKind::Circle;
  path.closed_ = true;
  path.circle_.center_x = center_x;
  path.circle_.center_y = center_y;
  path.circle_.radius = radius;
  path.circle_.theta0 = theta0;
  path.length_ = 2.0F * static_cast<float>(M_PI) * radius;
  path.buildPlotAnchorsFromGeometry(plot_samples);
  return path;
}

inline Path2D Path2D::stadium(const float straight_length, const float radius, const int samples_per_arc)
{
  const int na = std::max(4, samples_per_arc);
  const float hs = 0.5F * straight_length;
  std::vector<Pose2D> poses;
  poses.reserve(static_cast<size_t>(2 * na + 2));

  auto append = [&poses](const float x, const float y, const float yaw) {
    Pose2D pose;
    pose.x = x;
    pose.y = y;
    pose.yaw = yaw;
    poses.push_back(pose);
  };

  append(-hs, -radius, 0.0F);
  append(hs, -radius, 0.0F);
  for (int i = 1; i <= na; ++i)
  {
    const float a = -static_cast<float>(M_PI) * 0.5F +
                    static_cast<float>(M_PI) * static_cast<float>(i) / static_cast<float>(na);
    append(hs + radius * std::cos(a), radius * std::sin(a), a + static_cast<float>(M_PI) * 0.5F);
  }
  append(hs, radius, static_cast<float>(M_PI));
  append(-hs, radius, static_cast<float>(M_PI));
  for (int i = 1; i <= na; ++i)
  {
    const float a = static_cast<float>(M_PI) * 0.5F +
                    static_cast<float>(M_PI) * static_cast<float>(i) / static_cast<float>(na);
    append(-hs + radius * std::cos(a), radius * std::sin(a), a + static_cast<float>(M_PI) * 0.5F);
  }

  Path2D path;
  path.finalizeFromPoses(std::move(poses), true);
  return path;
}

namespace detail
{
inline void catmullRomPoint(const float p0x, const float p0y, const float p1x, const float p1y, const float p2x,
                           const float p2y, const float p3x, const float p3y, const float u, float& x, float& y)
{
  const float u2 = u * u;
  const float u3 = u2 * u;
  x = 0.5F * ((2.0F * p1x) + (-p0x + p2x) * u + (2.0F * p0x - 5.0F * p1x + 4.0F * p2x - p3x) * u2 +
              (-p0x + 3.0F * p1x - 3.0F * p2x + p3x) * u3);
  y = 0.5F * ((2.0F * p1y) + (-p0y + p2y) * u + (2.0F * p0y - 5.0F * p1y + 4.0F * p2y - p3y) * u2 +
              (-p0y + 3.0F * p1y - 3.0F * p2y + p3y) * u3);
}
}  // namespace detail

inline Path2D Path2D::catmullRom(const std::vector<std::pair<float, float>>& control_xy, const bool closed,
                                 const int samples_per_span)
{
  const int m = static_cast<int>(control_xy.size());
  if (m < 2)
  {
    throw std::invalid_argument("Path2D::catmullRom: need >= 2 control points");
  }
  const int ns = std::max(2, samples_per_span);
  std::vector<Pose2D> poses;
  const int n_span = closed ? m : m - 1;
  for (int span = 0; span < n_span; ++span)
  {
    auto idx = [m, closed](int i) -> int {
      if (closed)
      {
        return (i % m + m) % m;
      }
      return std::max(0, std::min(i, m - 1));
    };
    const int i0 = idx(span - 1);
    const int i1 = idx(span);
    const int i2 = idx(span + 1);
    const int i3 = idx(span + 2);
    const int u_start = (span == 0) ? 0 : 1;
    for (int k = u_start; k <= ns; ++k)
    {
      const float u = static_cast<float>(k) / static_cast<float>(ns);
      float x = 0.0F;
      float y = 0.0F;
      detail::catmullRomPoint(control_xy[static_cast<size_t>(i0)].first, control_xy[static_cast<size_t>(i0)].second,
                              control_xy[static_cast<size_t>(i1)].first, control_xy[static_cast<size_t>(i1)].second,
                              control_xy[static_cast<size_t>(i2)].first, control_xy[static_cast<size_t>(i2)].second,
                              control_xy[static_cast<size_t>(i3)].first, control_xy[static_cast<size_t>(i3)].second, u,
                              x, y);
      float yaw = 0.0F;
      if (k < ns || span < n_span - 1)
      {
        float xn = 0.0F;
        float yn = 0.0F;
        const float un = std::min(1.0F, u + 1.0F / static_cast<float>(ns));
        detail::catmullRomPoint(
            control_xy[static_cast<size_t>(i0)].first, control_xy[static_cast<size_t>(i0)].second,
            control_xy[static_cast<size_t>(i1)].first, control_xy[static_cast<size_t>(i1)].second,
            control_xy[static_cast<size_t>(i2)].first, control_xy[static_cast<size_t>(i2)].second,
            control_xy[static_cast<size_t>(i3)].first, control_xy[static_cast<size_t>(i3)].second, un, xn, yn);
        yaw = std::atan2(yn - y, xn - x);
      }
      else if (!poses.empty())
      {
        yaw = poses.back().yaw;
      }
      Pose2D pose;
      pose.x = x;
      pose.y = y;
      pose.yaw = yaw;
      poses.push_back(pose);
    }
  }
  Path2D path;
  path.finalizeFromPoses(std::move(poses), closed);
  return path;
}

}  // namespace path
}  // namespace mppi
