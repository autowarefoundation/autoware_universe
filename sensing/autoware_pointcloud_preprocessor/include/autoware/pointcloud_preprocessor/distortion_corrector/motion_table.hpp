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
inline constexpr std::size_t max_bin_count = (std::size_t{1} << (32U - bin_shift));
inline constexpr std::int64_t max_sample_distance_ns = 100'000'000;

inline constexpr std::uint32_t flag_twist_missing = 1U << 0U;
inline constexpr std::uint32_t flag_imu_missing = 1U << 1U;
/// The point counts towards the timestamp mismatch diagnostics.
inline constexpr std::uint32_t flag_mismatch = 1U << 2U;

/// A motion sample, stamped relative to the pointcloud's header stamp.
struct Sample
{
  std::int64_t stamp_ns;
  float value[3];
};

/// Linear velocity samples carry no more than the twist message does, so they are paired with the
/// angular velocity of the same message.
struct TwistSample
{
  std::int64_t stamp_ns;
  float linear[3];
  float angular[3];
};

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

/// Integrates planar motion (linear x, angular z).
class Integrator2D
{
public:
  using Bin = Bin2D;

  void integrate(const float * linear, const float * angular, double dt)
  {
    const double v = linear[0];
    const double w = angular[2];
    const double mid_theta = theta_ + 0.5 * w * dt;
    x_ += v * dt * std::cos(mid_theta);
    y_ += v * dt * std::sin(mid_theta);
    theta_ += w * dt;
  }

  [[nodiscard]] Bin make_bin(const Integrator2D & next, double bin_dt, std::uint32_t flags) const
  {
    Bin bin{};
    bin.cos_theta = static_cast<float>(std::cos(theta_));
    bin.sin_theta = static_cast<float>(std::sin(theta_));
    bin.x = static_cast<float>(x_);
    bin.y = static_cast<float>(y_);
    bin.rate_x = static_cast<float>((next.x_ - x_) / bin_dt);
    bin.rate_y = static_cast<float>((next.y_ - y_) / bin_dt);
    bin.rate_theta = static_cast<float>((next.theta_ - theta_) / bin_dt);
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

  void integrate(const float * linear, const float * angular, double dt)
  {
    const Eigen::Vector3d v(linear[0], linear[1], linear[2]);
    const Eigen::Vector3d rotation_vector =
      Eigen::Vector3d(angular[0], angular[1], angular[2]) * dt;
    const double angle = rotation_vector.norm();

    // exp() of the twist: the translation is the left Jacobian of SO(3) applied to v * dt
    Eigen::Matrix3d delta_rotation = Eigen::Matrix3d::Identity();
    Eigen::Vector3d delta_translation = v * dt;
    if (angle > 1e-9) {
      const Eigen::Vector3d axis = rotation_vector / angle;
      delta_rotation = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
      const Eigen::Vector3d cross = axis.cross(delta_translation);
      delta_translation += ((1.0 - std::cos(angle)) / angle) * cross +
                           (1.0 - std::sin(angle) / angle) * axis.cross(cross);
    }

    translation_ += rotation_ * delta_translation;
    rotation_ = rotation_ * delta_rotation;
  }

  [[nodiscard]] Bin make_bin(const Integrator3D & next, double bin_dt, std::uint32_t flags) const
  {
    const Eigen::AngleAxisd delta(rotation_.transpose() * next.rotation_);
    const Eigen::Vector3d rotation_rate = delta.axis() * (delta.angle() / bin_dt);
    const Eigen::Vector3d translation_rate = (next.translation_ - translation_) / bin_dt;

    Bin bin{};
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        bin.rotation[3 * row + col] = static_cast<float>(rotation_(row, col));
      }
      bin.translation[row] = static_cast<float>(translation_[row]);
      bin.translation_rate[row] = static_cast<float>(translation_rate[row]);
      bin.rotation_rate[row] = static_cast<float>(rotation_rate[row]);
    }
    bin.flags = flags;
    return bin;
  }

private:
  Eigen::Matrix3d rotation_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d translation_{Eigen::Vector3d::Zero()};
};

namespace detail
{
/// Walks a stamp-sorted sample list forwards in time.
template <typename SampleT>
class Cursor
{
public:
  explicit Cursor(const std::vector<SampleT> & samples) : samples_(samples) {}

  /// The stamp of the first sample after `time_ns`, or `fallback_ns` if there is none or it is
  /// later than that. `time_ns` must not decrease between calls.
  [[nodiscard]] std::int64_t next_stamp_after(std::int64_t time_ns, std::int64_t fallback_ns)
  {
    while (index_ < samples_.size() && samples_[index_].stamp_ns <= time_ns) {
      ++index_;
    }
    return index_ < samples_.size() ? std::min(fallback_ns, samples_[index_].stamp_ns)
                                    : fallback_ns;
  }

  /// The sample in effect up to and including `time_ns`, which must lie between the argument and
  /// the result of the last `next_stamp_after()` call. Null if there are no samples.
  [[nodiscard]] const SampleT * sample_at(std::int64_t time_ns, bool & is_missing) const
  {
    if (samples_.empty()) {
      is_missing = true;
      return nullptr;
    }
    const SampleT & sample = index_ < samples_.size() ? samples_[index_] : samples_.back();
    is_missing = std::llabs(time_ns - sample.stamp_ns) > max_sample_distance_ns;
    return &sample;
  }

private:
  const std::vector<SampleT> & samples_;
  std::size_t index_{0};
};
}  // namespace detail

/// Builds the motion table of one pointcloud into `bins`, which always ends up non-empty.
///
/// `twists` and `angular_velocities` must be sorted by stamp. Angular velocities take precedence
/// over the twists' when `use_angular_velocities` is set and one is available. The table ends
/// where the motion samples run out; its last bin holds the final pose with zero rates, for all
/// later points.
template <typename IntegratorT>
void build(
  const std::vector<TwistSample> & twists, const std::vector<Sample> & angular_velocities,
  bool use_angular_velocities, std::vector<typename IntegratorT::Bin> & bins)
{
  static const std::vector<Sample> no_samples;
  const std::vector<Sample> & imu_samples =
    use_angular_velocities ? angular_velocities : no_samples;

  std::int64_t last_stamp_ns = -max_sample_distance_ns;
  if (!twists.empty()) last_stamp_ns = std::max(last_stamp_ns, twists.back().stamp_ns);
  if (!imu_samples.empty()) last_stamp_ns = std::max(last_stamp_ns, imu_samples.back().stamp_ns);

  const std::int64_t end_ns = last_stamp_ns + max_sample_distance_ns;
  const auto bin_count = static_cast<std::size_t>(std::clamp<std::int64_t>(
    (end_ns + bin_width_ns - 1) / bin_width_ns, 0, static_cast<std::int64_t>(max_bin_count) - 1));

  bins.clear();
  bins.reserve(bin_count + 1);

  detail::Cursor<TwistSample> twist_cursor(twists);
  detail::Cursor<Sample> imu_cursor(imu_samples);
  const float zero[3] = {0.f, 0.f, 0.f};

  IntegratorT pose;
  std::uint32_t flags = 0;
  for (std::size_t bin_index = 0; bin_index < bin_count; ++bin_index) {
    const auto bin_start_ns = static_cast<std::int64_t>(bin_index) * bin_width_ns;
    const auto bin_end_ns = bin_start_ns + bin_width_ns;
    const IntegratorT bin_start_pose = pose;

    flags = 0;
    for (std::int64_t from_ns = bin_start_ns; from_ns < bin_end_ns;) {
      std::int64_t to_ns = twist_cursor.next_stamp_after(from_ns, bin_end_ns);
      to_ns = imu_cursor.next_stamp_after(from_ns, to_ns);

      bool is_twist_missing = false;
      bool is_imu_missing = false;
      const TwistSample * twist = twist_cursor.sample_at(to_ns, is_twist_missing);
      const Sample * imu = imu_cursor.sample_at(to_ns, is_imu_missing);

      const float * linear = is_twist_missing ? zero : twist->linear;
      const float * angular = is_twist_missing ? zero : twist->angular;
      if (!is_imu_missing) {
        angular = imu->value;
      }

      if (is_twist_missing) flags |= flag_twist_missing;
      if (imu != nullptr && is_imu_missing) flags |= flag_imu_missing;
      if (is_twist_missing || (use_angular_velocities && is_imu_missing)) flags |= flag_mismatch;

      pose.integrate(linear, angular, 1e-9 * static_cast<double>(to_ns - from_ns));
      from_ns = to_ns;
    }

    bins.push_back(bin_start_pose.make_bin(pose, 1e-9 * static_cast<double>(bin_width_ns), flags));
  }

  std::uint32_t end_flags = flag_twist_missing | flag_mismatch;
  if (!imu_samples.empty()) end_flags |= flag_imu_missing;
  bins.push_back(pose.make_bin(pose, 1.0, end_flags));
}

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
  const float delta_squared = delta * delta;
  const float cos_delta = 1.f - 0.5f * delta_squared;
  const float sin_delta = delta * (1.f - delta_squared * (1.f / 6.f));
  const float cos_theta = bin.cos_theta * cos_delta - bin.sin_theta * sin_delta;
  const float sin_theta = bin.sin_theta * cos_delta + bin.cos_theta * sin_delta;

  const float distorted_x = x;
  const float distorted_y = y;
  x = cos_theta * distorted_x - sin_theta * distorted_y + bin.x + bin.rate_x * remainder;
  y = sin_theta * distorted_x + cos_theta * distorted_y + bin.y + bin.rate_y * remainder;
}

inline void undistort(const Bin3D & bin, float remainder, float & x, float & y, float & z)
{
  // Second-order rotation by the small rotation vector d: p + d x p + (d x (d x p)) / 2
  const float dx = bin.rotation_rate[0] * remainder;
  const float dy = bin.rotation_rate[1] * remainder;
  const float dz = bin.rotation_rate[2] * remainder;

  const float cx = dy * z - dz * y;
  const float cy = dz * x - dx * z;
  const float cz = dx * y - dy * x;

  const float px = x + cx + 0.5f * (dy * cz - dz * cy);
  const float py = y + cy + 0.5f * (dz * cx - dx * cz);
  const float pz = z + cz + 0.5f * (dx * cy - dy * cx);

  const float * r = bin.rotation;
  x = r[0] * px + r[1] * py + r[2] * pz + bin.translation[0] + bin.translation_rate[0] * remainder;
  y = r[3] * px + r[4] * py + r[5] * pz + bin.translation[1] + bin.translation_rate[1] * remainder;
  z = r[6] * px + r[7] * py + r[8] * pz + bin.translation[2] + bin.translation_rate[2] * remainder;
}

}  // namespace autoware::pointcloud_preprocessor::motion_table

#endif  // AUTOWARE__POINTCLOUD_PREPROCESSOR__DISTORTION_CORRECTOR__MOTION_TABLE_HPP_
