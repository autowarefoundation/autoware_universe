// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE__POINTCLOUD_PREPROCESSOR__DISTORTION_CORRECTOR__MOTION_TABLE_HPP_
#define AUTOWARE__POINTCLOUD_PREPROCESSOR__DISTORTION_CORRECTOR__MOTION_TABLE_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <vector>

/// Ego motion over one pointcloud, quantized in time.
///
/// Bin `k` covers the times `[k, k + 1) * bin_width_ns` after the pointcloud's header stamp and
/// holds the pose of the base frame at the start of the bin, expressed in the base frame at the
/// header stamp, together with the rates that carry that pose to the start of the next bin. A
/// point finds its bin from its `time_stamp` field alone, as `time_stamp >> bin_shift`, so the
/// correction of a point depends on neither its position in the cloud nor on any other point.
///
/// The velocity at a time `t` is that of the first motion sample stamped at or after `t` (the last
/// sample if there is none), and counts as missing if that sample is more than
/// `max_sample_distance_ns` away from `t`.
namespace autoware::pointcloud_preprocessor::motion_table
{

inline constexpr std::uint32_t bin_shift = 20;
inline constexpr std::uint32_t bin_width_ns = 1U << bin_shift;
inline constexpr std::uint32_t bin_remainder_mask = bin_width_ns - 1U;
inline constexpr float bin_width_sec = 1e-9f * static_cast<float>(bin_width_ns);
/// `time_stamp` is 32 bits wide, so no point lies beyond this many bins.
inline constexpr std::int64_t max_bin_count = std::int64_t{1} << (32U - bin_shift);
inline constexpr std::int64_t max_sample_distance_ns = 100'000'000;

inline constexpr std::uint32_t flag_twist_missing = 1U << 0U;
inline constexpr std::uint32_t flag_imu_missing = 1U << 1U;
/// The point counts towards the timestamp mismatch diagnostics.
inline constexpr std::uint32_t flag_mismatch = 1U << 2U;

/// Motion samples are in the base frame and stamped relative to the pointcloud's header stamp.
struct TwistSample
{
  std::int64_t stamp_ns;
  Eigen::Vector3f linear;
  Eigen::Vector3f angular;
};

struct AngularVelocitySample
{
  std::int64_t stamp_ns;
  Eigen::Vector3f angular;
};

// Bins are read by CUDA kernels, so they hold plain floats.

struct Bin2D
{
  float cos_theta;
  float sin_theta;
  float x;
  float y;
  float rate_x;      // [m/s], in the frame at the header stamp
  float rate_y;      // [m/s], in the frame at the header stamp
  float rate_theta;  // [rad/s]
  std::uint32_t flags;
};

struct Bin3D
{
  float rotation[9];          // row-major
  float translation[3];       // [m]
  float translation_rate[3];  // [m/s], in the frame at the header stamp
  float rotation_rate[3];     // [rad/s], in the frame at the start of the bin
  std::uint32_t flags;
};

using RowMajorMatrix3f = Eigen::Matrix<float, 3, 3, Eigen::RowMajor>;

/// The bin of a point and the time from the start of that bin to the point, in seconds.
template <typename BinT>
inline const BinT & lookup(
  const BinT * bins, std::uint32_t last_bin_index, std::uint32_t time_stamp_ns, float & remainder)
{
  remainder = 1e-9f * static_cast<float>(time_stamp_ns & bin_remainder_mask);
  const std::uint32_t bin_index = time_stamp_ns >> bin_shift;
  return bins[bin_index < last_bin_index ? bin_index : last_bin_index];
}

/// Moves a point in the base frame at its own time into the base frame at the header stamp.
inline void undistort(const Bin2D & bin, float remainder, float & x, float & y)
{
  // The rotation within a bin is far below a degree, the series are exact to float precision
  const float delta = bin.rate_theta * remainder;
  const float cos_delta = 1.f - 0.5f * delta * delta;
  const float sin_delta = delta * (1.f - delta * delta / 6.f);
  const float cos_theta = bin.cos_theta * cos_delta - bin.sin_theta * sin_delta;
  const float sin_theta = bin.sin_theta * cos_delta + bin.cos_theta * sin_delta;

  const float distorted_x = x;
  const float distorted_y = y;
  x = cos_theta * distorted_x - sin_theta * distorted_y + bin.x + bin.rate_x * remainder;
  y = sin_theta * distorted_x + cos_theta * distorted_y + bin.y + bin.rate_y * remainder;
}

inline void undistort(const Bin3D & bin, float remainder, float & x, float & y, float & z)
{
  const Eigen::Map<const RowMajorMatrix3f> rotation(bin.rotation);
  const Eigen::Map<const Eigen::Vector3f> translation(bin.translation);
  const Eigen::Map<const Eigen::Vector3f> translation_rate(bin.translation_rate);
  const Eigen::Map<const Eigen::Vector3f> rotation_rate(bin.rotation_rate);

  // Rotation by the small rotation vector d, to second order: p + d x p + d x (d x p) / 2
  const Eigen::Vector3f p(x, y, z);
  const Eigen::Vector3f d = rotation_rate * remainder;
  const Eigen::Vector3f d_cross_p = d.cross(p);
  const Eigen::Vector3f undistorted = rotation * (p + d_cross_p + 0.5f * d.cross(d_cross_p)) +
                                      translation + translation_rate * remainder;

  x = undistorted.x();
  y = undistorted.y();
  z = undistorted.z();
}

// The integrators advance by the midpoint rule: a step translates along the orientation halfway
// through it. Steps are at most a bin long, which makes the error negligible.

/// Integrates planar motion (linear x, angular z).
class Integrator2D
{
public:
  using Bin = Bin2D;

  void integrate(const Eigen::Vector3f & linear, const Eigen::Vector3f & angular, double dt)
  {
    const double half_turn = 0.5 * angular.z() * dt;
    theta_ += half_turn;
    x_ += linear.x() * dt * std::cos(theta_);
    y_ += linear.x() * dt * std::sin(theta_);
    theta_ += half_turn;
  }

  /// The bin that starts at this pose and reaches the pose `next` a bin width later.
  [[nodiscard]] Bin make_bin(const Integrator2D & next, std::uint32_t flags) const
  {
    Bin bin{};
    bin.cos_theta = static_cast<float>(std::cos(theta_));
    bin.sin_theta = static_cast<float>(std::sin(theta_));
    bin.x = static_cast<float>(x_);
    bin.y = static_cast<float>(y_);
    bin.rate_x = static_cast<float>((next.x_ - x_) / bin_width_sec);
    bin.rate_y = static_cast<float>((next.y_ - y_) / bin_width_sec);
    bin.rate_theta = static_cast<float>((next.theta_ - theta_) / bin_width_sec);
    bin.flags = flags;
    return bin;
  }

private:
  double x_{0.0};
  double y_{0.0};
  double theta_{0.0};
};

/// Integrates rigid motion in space.
class Integrator3D
{
public:
  using Bin = Bin3D;

  void integrate(const Eigen::Vector3f & linear, const Eigen::Vector3f & angular, double dt)
  {
    const Eigen::Vector3d half_turn = 0.5 * dt * angular.cast<double>();
    const Eigen::Matrix3d half_rotation =
      Eigen::AngleAxisd(half_turn.norm(), half_turn.stableNormalized()).toRotationMatrix();

    rotation_ = rotation_ * half_rotation;
    translation_ += rotation_ * linear.cast<double>() * dt;
    rotation_ = rotation_ * half_rotation;
  }

  /// The bin that starts at this pose and reaches the pose `next` a bin width later.
  [[nodiscard]] Bin make_bin(const Integrator3D & next, std::uint32_t flags) const
  {
    const Eigen::AngleAxisd turn(rotation_.transpose() * next.rotation_);
    const Eigen::Vector3d rotation_rate = turn.axis() * (turn.angle() / bin_width_sec);
    const Eigen::Vector3d translation_rate = (next.translation_ - translation_) / bin_width_sec;

    Bin bin{};
    Eigen::Map<RowMajorMatrix3f>(bin.rotation) = rotation_.cast<float>();
    Eigen::Map<Eigen::Vector3f>(bin.translation) = translation_.cast<float>();
    Eigen::Map<Eigen::Vector3f>(bin.translation_rate) = translation_rate.cast<float>();
    Eigen::Map<Eigen::Vector3f>(bin.rotation_rate) = rotation_rate.cast<float>();
    bin.flags = flags;
    return bin;
  }

private:
  Eigen::Matrix3d rotation_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d translation_{Eigen::Vector3d::Zero()};
};

namespace detail
{
struct Velocity
{
  Eigen::Vector3f linear{Eigen::Vector3f::Zero()};
  Eigen::Vector3f angular{Eigen::Vector3f::Zero()};
  std::uint32_t flags{0};
};

/// The first sample stamped at or after `time_ns`, or the last sample if there is none. Null if
/// that sample is more than `max_sample_distance_ns` away from `time_ns`, or if there is no sample.
template <typename SampleT>
const SampleT * find_sample(const std::vector<SampleT> & samples, std::int64_t time_ns)
{
  if (samples.empty()) {
    return nullptr;
  }

  const auto it = std::lower_bound(
    samples.begin(), samples.end(), time_ns,
    [](const SampleT & sample, std::int64_t stamp_ns) { return sample.stamp_ns < stamp_ns; });
  const SampleT & sample = it != samples.end() ? *it : samples.back();
  return std::abs(time_ns - sample.stamp_ns) <= max_sample_distance_ns ? &sample : nullptr;
}

/// The stamp of the first sample after `time_ns`, or the largest stamp there is if there is none.
template <typename SampleT>
std::int64_t next_stamp_after(const std::vector<SampleT> & samples, std::int64_t time_ns)
{
  const auto it = std::upper_bound(
    samples.begin(), samples.end(), time_ns,
    [](std::int64_t stamp_ns, const SampleT & sample) { return stamp_ns < sample.stamp_ns; });
  return it != samples.end() ? it->stamp_ns : std::numeric_limits<std::int64_t>::max();
}

/// The velocity at `time_ns`, zero where the samples are missing. `angular_velocities` take
/// precedence over the twists' angular velocity, and are expected if `use_angular_velocities`.
inline Velocity velocity_at(
  const std::vector<TwistSample> & twists,
  const std::vector<AngularVelocitySample> & angular_velocities, bool use_angular_velocities,
  std::int64_t time_ns)
{
  Velocity velocity;

  if (const TwistSample * twist = find_sample(twists, time_ns)) {
    velocity.linear = twist->linear;
    velocity.angular = twist->angular;
  } else {
    velocity.flags |= flag_twist_missing | flag_mismatch;
  }

  if (const AngularVelocitySample * sample = find_sample(angular_velocities, time_ns)) {
    velocity.angular = sample->angular;
  } else if (use_angular_velocities) {
    velocity.flags |= flag_mismatch;
    if (!angular_velocities.empty()) {
      velocity.flags |= flag_imu_missing;
    }
  }

  return velocity;
}

/// The time from which on every sample counts as missing.
template <typename SampleT>
std::int64_t end_of_validity(const std::vector<SampleT> & samples)
{
  return samples.empty() ? 0 : samples.back().stamp_ns + max_sample_distance_ns;
}
}  // namespace detail

/// Builds the motion table of one pointcloud into `bins`, which always ends up non-empty.
///
/// `twists` and `angular_velocities` must be sorted by stamp. The latter are ignored unless
/// `use_angular_velocities` is set. The table ends where the motion samples run out; its last bin
/// holds the final pose with zero rates, for all later points.
template <typename IntegratorT>
void build(
  const std::vector<TwistSample> & twists,
  const std::vector<AngularVelocitySample> & all_angular_velocities, bool use_angular_velocities,
  std::vector<typename IntegratorT::Bin> & bins)
{
  static const std::vector<AngularVelocitySample> none;
  const auto & angular_velocities = use_angular_velocities ? all_angular_velocities : none;

  const std::int64_t end_ns = std::max(
    {std::int64_t{0}, detail::end_of_validity(twists),
     detail::end_of_validity(angular_velocities)});
  const std::int64_t bin_count =
    std::min((end_ns + bin_width_ns - 1) / bin_width_ns, max_bin_count - 1);

  bins.clear();
  bins.reserve(bin_count + 1);

  IntegratorT pose;
  for (std::int64_t bin_index = 0; bin_index < bin_count; ++bin_index) {
    const std::int64_t bin_end_ns = (bin_index + 1) * bin_width_ns;
    const IntegratorT bin_start_pose = pose;
    std::uint32_t flags = 0;

    // The velocity is constant between two sample stamps
    for (std::int64_t time_ns = bin_index * bin_width_ns; time_ns < bin_end_ns;) {
      const std::int64_t step_end_ns = std::min(
        {bin_end_ns, detail::next_stamp_after(twists, time_ns),
         detail::next_stamp_after(angular_velocities, time_ns)});
      const detail::Velocity velocity =
        detail::velocity_at(twists, angular_velocities, use_angular_velocities, step_end_ns);

      pose.integrate(
        velocity.linear, velocity.angular, 1e-9 * static_cast<double>(step_end_ns - time_ns));
      flags |= velocity.flags;
      time_ns = step_end_ns;
    }

    bins.push_back(bin_start_pose.make_bin(pose, flags));
  }

  const std::uint32_t end_flags =
    detail::velocity_at(twists, angular_velocities, use_angular_velocities, end_ns + 1).flags;
  bins.push_back(pose.make_bin(pose, end_flags));
}

}  // namespace autoware::pointcloud_preprocessor::motion_table

#endif  // AUTOWARE__POINTCLOUD_PREPROCESSOR__DISTORTION_CORRECTOR__MOTION_TABLE_HPP_
