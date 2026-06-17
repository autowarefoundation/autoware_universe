/**
 * Time-stamped reference along a Path2D for MPPI tracking (samples every dt).
 *
 * Reference construction:
 * 1. Start at the closest point on the path centerline (orthogonal projection of x, y).
 * 2. March forward in arc length: s_{k+1} = s_k + v_ref(s_k) * dt, where v_ref is the path
 *    reference speed (target speed, curvature-limited, end-of-path slowed).
 *
 * Vehicle speed is not used to seed or advance the reference.
 */
#pragma once

#include <mppi/path/path2d.hpp>
#include <mppi/path/path_projection.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

namespace mppi
{
namespace path
{

struct PathReferenceSample
{
  float t = 0.0F;
  float arc_length_s = 0.0F;
  float x = 0.0F;
  float y = 0.0F;
  float yaw = 0.0F;
  float v = 0.0F;
};

class PathReferenceGenerator
{
public:
  PathReferenceGenerator() = default;

  explicit PathReferenceGenerator(const float dt) : dt_(dt)
  {
  }

  void setDt(const float dt)
  {
    dt_ = dt;
  }

  float dt() const
  {
    return dt_;
  }

  void setSpeedCap(const float v_max)
  {
    v_max_ = v_max;
    if (target_speed_ > v_max_)
    {
      target_speed_ = v_max_;
    }
  }

  void setTargetSpeed(const float v_target)
  {
    target_speed_ = std::min(v_target, v_max_);
  }

  float targetSpeed() const
  {
    return target_speed_;
  }

  void setLateralAccelCap(const float a_lat_max)
  {
    a_lat_max_ = a_lat_max;
  }

  /** Reference speed at arc length s (target speed, curvature-limited, end-of-path slowed). */
  float referenceSpeedAt(const Path2D& path, const float s) const
  {
    const float kappa = std::fabs(path.curvatureAt(s));
    const float kappa_eps = 1.0E-4F;
    float v_ref = target_speed_;
    if (kappa >= kappa_eps)
    {
      v_ref = std::min(target_speed_, std::sqrt(a_lat_max_ / kappa));
    }
    if (!path.closed())
    {
      const float dist_to_end = path.length() - s;
      if (dist_to_end < 5.0F)
      {
        v_ref = std::min(v_ref, target_speed_ * std::max(0.0F, dist_to_end / 5.0F));
      }
      if (dist_to_end <= 0.0F)
      {
        v_ref = 0.0F;
      }
    }
    return std::min(v_ref, v_max_);
  }

  float speedAt(const Path2D& path, const float s) const
  {
    return referenceSpeedAt(path, s);
  }

  /**
   * Reference from arc length s0: sample at t = 0, dt, ... marching with v_ref(s) * dt.
   */
  std::vector<PathReferenceSample> generate(const Path2D& path, const float start_s, const int count) const
  {
    std::vector<PathReferenceSample> out;
    if (count <= 0 || path.empty())
    {
      return out;
    }
    out.resize(static_cast<size_t>(count));
    float s = path.wrapArcLength(start_s);
    for (int k = 0; k < count; ++k)
    {
      PathReferenceSample& r = out[static_cast<size_t>(k)];
      r.t = static_cast<float>(k) * dt_;
      r.arc_length_s = s;
      const Pose2D p = path.poseAt(s);
      r.x = p.x;
      r.y = p.y;
      r.yaw = p.yaw;
      r.v = referenceSpeedAt(path, s);
      if (k + 1 < count)
      {
        const float s_next = s + r.v * dt_;
        if (path.closed())
        {
          s = path.wrapArcLength(s_next);
        }
        else
        {
          s = std::min(s_next, path.length());
        }
      }
    }
    return out;
  }

  /**
   * Project (x, y) to the closest centerline point, then march forward at reference speed.
   * @param s_hint  Optional arc-length hint for projection search (e.g. last step's s).
   */
  std::vector<PathReferenceSample> generate(const Path2D& path, const int count, const float x, const float y,
                                            const float s_hint = 0.0F) const
  {
    if (count <= 0 || path.empty())
    {
      return {};
    }
    const PathProjection proj = projectPoseOntoPath(path, x, y, path.wrapArcLength(s_hint));
    return generate(path, proj.arc_length_s, count);
  }

  /** @deprecated Use generate(path, count, x, y, s_hint); yaw and v are ignored. */
  std::vector<PathReferenceSample> generate(const Path2D& path, const float start_s, const int count, const float x,
                                          const float y, const float yaw, const float v) const
  {
    (void)yaw;
    (void)v;
    return generate(path, count, x, y, start_s);
  }

private:
  float dt_ = 0.1F;
  float v_max_ = 3.0F;
  float target_speed_ = 3.0F;
  float a_lat_max_ = 8.0F;
};

}  // namespace path
}  // namespace mppi
