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

#ifndef AUTOWARE__CALIBRATION_STATUS_CLASSIFIER__FILTER_HPP_
#define AUTOWARE__CALIBRATION_STATUS_CLASSIFIER__FILTER_HPP_

#include <cmath>
#include <optional>
#include <string>
#include <vector>

namespace autoware::calibration_status_classifier
{

/// @brief 3D position of a detected object in ego frame
struct ObjectInfo
{
  double x;
  double y;
  double z;
};

/// @brief 3D linear velocity components (m/s)
struct LinearVelocityInfo
{
  double x;
  double y;
  double z;
};

/// @brief 3D angular velocity components (rad/s)
struct AngularVelocityInfo
{
  double x;
  double y;
  double z;
};

/// @brief Snapshot of a filter's state at a given time
struct FilterResult
{
  std::string name;
  std::string unit;
  bool is_passed;
  bool is_threshold_exceeded;
  double current_value;
  double threshold;
  double timeout_sec;
  double state_age;
};

/// @brief Abstract base class for prerequisite filters
class Filter
{
public:
  virtual ~Filter() = default;
  Filter(const Filter &) = delete;
  Filter & operator=(const Filter &) = delete;
  Filter(Filter &&) = delete;
  Filter & operator=(Filter &&) = delete;

  [[nodiscard]] virtual bool is_passed(double stamp) const = 0;
  [[nodiscard]] virtual const std::string & name() const = 0;
  [[nodiscard]] virtual const std::string & unit() const = 0;
  [[nodiscard]] virtual FilterResult get_result(double stamp) const = 0;

protected:
  Filter() = default;
};

/**
 * @brief Threshold-based filter with one-way timeout debounce
 *
 * When the monitored value exceeds the threshold, the filter immediately
 * blocks (is_passed() returns false). When the value drops below the threshold,
 * the filter remains blocked until it has been continuously below the threshold
 * for at least `timeout_sec` seconds.
 *
 * @tparam ValueT The type of the monitored value (e.g., double, size_t)
 */
template <typename ValueT>
class ThresholdFilter : public Filter
{
public:
  ThresholdFilter(ValueT threshold, double timeout_sec)
  : threshold_(threshold), timeout_sec_(timeout_sec), current_value_(ValueT{})
  {
  }

  ~ThresholdFilter() override = default;
  ThresholdFilter(const ThresholdFilter &) = delete;
  ThresholdFilter & operator=(const ThresholdFilter &) = delete;
  ThresholdFilter(ThresholdFilter &&) = delete;
  ThresholdFilter & operator=(ThresholdFilter &&) = delete;

  void update(const ValueT value, const double stamp)
  {
    current_value_ = value;
    last_update_time_ = stamp;
    if (recovery_start_time_ > stamp) {  // handle time going backwards (e.g., due to clock reset)
      recovery_start_time_.reset();
    }
    if (exceeds_threshold(value)) {
      threshold_exceeded_ = true;
      recovery_start_time_.reset();
    } else if (threshold_exceeded_) {
      threshold_exceeded_ = false;
      recovery_start_time_ = stamp;
    }
  }

  [[nodiscard]] bool is_passed(double stamp) const override
  {
    if (exceeds_threshold(current_value_)) return false;
    if (!recovery_start_time_.has_value()) return true;
    return (stamp - recovery_start_time_.value()) >= timeout_sec_;
  }

  [[nodiscard]] FilterResult get_result(double stamp) const override
  {
    const double state_age =
      last_update_time_.has_value() ? (stamp - last_update_time_.value()) : 0.0;
    return {
      name(),
      unit(),
      is_passed(stamp),
      exceeds_threshold(current_value_),
      static_cast<double>(current_value_),
      static_cast<double>(threshold_),
      timeout_sec_,
      state_age};
  }

  [[nodiscard]] virtual bool exceeds_threshold(ValueT value) const { return value > threshold_; }

  [[nodiscard]] ValueT threshold() const { return threshold_; }
  [[nodiscard]] double timeout_sec() const { return timeout_sec_; }

protected:
  ValueT threshold_;
  double timeout_sec_;
  ValueT current_value_;
  bool threshold_exceeded_{false};
  std::optional<double> last_update_time_;
  std::optional<double> recovery_start_time_;
};

/**
 * @brief Filter that blocks when 3D linear velocity magnitude exceeds a threshold
 *
 * Computes sqrt(x^2 + y^2 + z^2) from the velocity components.
 */
class LinearVelocityFilter : public ThresholdFilter<double>
{
public:
  LinearVelocityFilter(double threshold, double timeout_sec);
  [[nodiscard]] const std::string & name() const override;
  [[nodiscard]] const std::string & unit() const override;

  void update(const LinearVelocityInfo & info, const double stamp);

private:
  const std::string name_;
  const std::string unit_;
};

/**
 * @brief Filter that blocks when 3D angular velocity magnitude exceeds a threshold
 *
 * Computes sqrt(x^2 + y^2 + z^2) from the angular velocity components.
 */
class AngularVelocityFilter : public ThresholdFilter<double>
{
public:
  AngularVelocityFilter(double threshold, double timeout_sec);
  [[nodiscard]] const std::string & name() const override;
  [[nodiscard]] const std::string & unit() const override;

  void update(const AngularVelocityInfo & info, const double stamp);

private:
  const std::string name_;
  const std::string unit_;
};

/// @brief Filter that blocks when the number of detected objects exceeds a threshold
class ObjectsFilter : public ThresholdFilter<size_t>
{
public:
  ObjectsFilter(size_t threshold, double timeout_sec);
  [[nodiscard]] const std::string & name() const override;
  [[nodiscard]] const std::string & unit() const override;

  void update(const std::vector<ObjectInfo> & objects, const double stamp);
  [[nodiscard]] const std::vector<ObjectInfo> & objects() const;

private:
  const std::string name_;
  const std::string unit_;
  std::vector<ObjectInfo> objects_;
};

}  // namespace autoware::calibration_status_classifier

#endif  // AUTOWARE__CALIBRATION_STATUS_CLASSIFIER__FILTER_HPP_
