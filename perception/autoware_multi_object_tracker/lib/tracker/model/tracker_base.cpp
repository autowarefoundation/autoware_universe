// Copyright 2020 Tier IV, Inc.
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
//
//

#include "autoware/multi_object_tracker/tracker/model/tracker_base.hpp"

#include <autoware_utils/geometry/geometry.hpp>

#include <algorithm>
#include <limits>
#include <random>
#include <vector>

namespace
{
float updateProbability(
  const float & prior, const float & true_positive, const float & false_positive,
  const bool clamp = true)
{
  float probability =
    (prior * true_positive) / (prior * true_positive + (1 - prior) * false_positive);

  if (clamp) {
    // Normalize the probability to [0.1, 0.999]
    constexpr float max_updated_probability = 0.999;
    constexpr float min_updated_probability = 0.100;
    probability = std::clamp(probability, min_updated_probability, max_updated_probability);
  }

  return probability;
}
float decayProbability(const float & prior, const float & delta_time)
{
  constexpr float minimum_probability = 0.001;
  const float decay_rate = log(0.5f) / 0.3f;  // half-life (50% decay) of 0.3s
  return std::max(prior * std::exp(decay_rate * delta_time), minimum_probability);
}
}  // namespace

namespace autoware::multi_object_tracker
{

Tracker::Tracker(const rclcpp::Time & time, const types::DynamicObject & detected_object)
: no_measurement_count_(0),
  total_no_measurement_count_(0),
  total_measurement_count_(1),
  last_update_with_measurement_time_(time),
  object_(detected_object)
{
  // Generate random number
  std::mt19937 gen(std::random_device{}());
  std::independent_bits_engine<std::mt19937, 8, uint8_t> bit_eng(gen);
  unique_identifier_msgs::msg::UUID uuid_msg;
  std::generate(uuid_msg.uuid.begin(), uuid_msg.uuid.end(), bit_eng);
  object_.uuid = uuid_msg;

  // Initialize existence probabilities
  existence_probabilities_.resize(types::max_channel_size, 0.001);
  total_existence_probability_ = 0.001;
}

void Tracker::initializeExistenceProbabilities(
  const uint & channel_index, const float & existence_probability)
{
  // The initial existence probability is normalized to [0.1, 0.999]
  // to avoid the existence probability being too low or too high
  // and to avoid the existence probability being too close to 0 or 1
  constexpr float max_probability = 0.999;
  constexpr float min_probability = 0.100;
  const float clamped_existence_probability =
    std::clamp(existence_probability, min_probability, max_probability);

  // existence probability on each channel
  existence_probabilities_[channel_index] = clamped_existence_probability;

  // total existence probability
  total_existence_probability_ = clamped_existence_probability;
}

void Tracker::updateTotalExistenceProbability(const float & existence_probability)
{
  total_existence_probability_ =
    updateProbability(total_existence_probability_, existence_probability, 0.2);
}

void Tracker::mergeExistenceProbabilities(std::vector<float> existence_probabilities)
{
  // existence probability on each channel
  for (size_t i = 0; i < existence_probabilities.size(); ++i) {
    // take larger value
    existence_probabilities_[i] = std::max(existence_probabilities_[i], existence_probabilities[i]);
  }
}

bool Tracker::updateWithMeasurement(
  const types::DynamicObject & object, const rclcpp::Time & measurement_time,
  const types::InputChannel & channel_info)
{
  // Update existence probability
  {
    no_measurement_count_ = 0;
    ++total_measurement_count_;

    // existence probability on each channel
    const float delta_time =
      std::abs((measurement_time - last_update_with_measurement_time_).seconds());
    constexpr float probability_true_detection = 0.9;
    constexpr float probability_false_detection = 0.2;

    // update measured channel probability without decay
    const uint & channel_index = channel_info.index;
    existence_probabilities_[channel_index] = updateProbability(
      existence_probabilities_[channel_index], probability_true_detection,
      probability_false_detection);

    // decay other channel probabilities
    for (size_t i = 0; i < existence_probabilities_.size(); ++i) {
      if (i != channel_index) {
        existence_probabilities_[i] = decayProbability(existence_probabilities_[i], delta_time);
      }
    }

    // update total existence probability
    const double existence_probability = channel_info.trust_existence_probability
                                           ? object.existence_probability
                                           : types::default_existence_probability;
    total_existence_probability_ = updateProbability(
      total_existence_probability_, existence_probability * probability_true_detection,
      probability_false_detection);
  }

  last_update_with_measurement_time_ = measurement_time;

  // Update classification
  if (
    channel_info.trust_classification &&
    autoware::object_recognition_utils::getHighestProbLabel(object.classification) !=
      autoware_perception_msgs::msg::ObjectClassification::UNKNOWN) {
    updateClassification(object.classification);
  }

  // Update object
  measure(object, measurement_time, channel_info);

  // Update object status
  getTrackedObject(measurement_time, object_);

  return true;
}

bool Tracker::updateWithoutMeasurement(const rclcpp::Time & timestamp)
{
  // Update existence probability
  ++no_measurement_count_;
  ++total_no_measurement_count_;
  {
    // decay existence probability
    float const delta_time = (timestamp - last_update_with_measurement_time_).seconds();
    for (float & existence_probability : existence_probabilities_) {
      existence_probability = decayProbability(existence_probability, delta_time);
    }
    total_existence_probability_ = decayProbability(total_existence_probability_, delta_time);
  }

  // Update object status
  getTrackedObject(timestamp, object_);

  return true;
}

void Tracker::updateClassification(
  const std::vector<autoware_perception_msgs::msg::ObjectClassification> & classification)
{
  // classification algorithm:
  // 0. Normalize the input classification
  // 1-1. Update the matched classification probability with a gain (ratio of 0.05)
  // 1-2. If the label is not found, add it to the classification list
  // 2. Remove the class with probability < remove_threshold (0.001)
  // 3. Normalize tracking classification

  // Parameters
  // if the remove_threshold is too high (compare to the gain), the classification will be removed
  // immediately
  const float gain = 0.05;
  constexpr float remove_threshold = 0.001;

  // Normalization function
  auto normalizeProbabilities =
    [](std::vector<autoware_perception_msgs::msg::ObjectClassification> & classification) {
      float sum = 0.0;
      for (const auto & a_class : classification) {
        sum += a_class.probability;
      }
      for (auto & a_class : classification) {
        a_class.probability /= sum;
      }
    };

  // Normalize the input
  auto classification_input = classification;
  normalizeProbabilities(classification_input);

  auto & classification_ = object_.classification;

  // Update the matched classification probability with a gain
  for (const auto & new_class : classification_input) {
    bool found = false;
    for (auto & old_class : classification_) {
      if (new_class.label == old_class.label) {
        old_class.probability += new_class.probability * gain;
        found = true;
        break;
      }
    }
    // If the label is not found, add it to the classification list
    if (!found) {
      auto adding_class = new_class;
      adding_class.probability *= gain;
      classification_.push_back(adding_class);
    }
  }

  // If the probability is less than the threshold, remove the class
  classification_.erase(
    std::remove_if(
      classification_.begin(), classification_.end(),
      [remove_threshold](const auto & a_class) { return a_class.probability < remove_threshold; }),
    classification_.end());

  // Normalize tracking classification
  normalizeProbabilities(classification_);
}

void Tracker::limitObjectExtension(const object_model::ObjectModel object_model)
{
  auto & object_extension = object_.shape.dimensions;
  // set maximum and minimum size
  object_extension.x = std::clamp(
    object_extension.x, object_model.size_limit.length_min, object_model.size_limit.length_max);
  object_extension.y = std::clamp(
    object_extension.y, object_model.size_limit.width_min, object_model.size_limit.width_max);
  object_extension.z = std::clamp(
    object_extension.z, object_model.size_limit.height_min, object_model.size_limit.height_max);
}

void Tracker::getPositionCovarianceEigenSq(
  const rclcpp::Time & time, double & major_axis_sq, double & minor_axis_sq) const
{
  // estimate the covariance of the position at the given time
  types::DynamicObject object = object_;
  if (object.time.seconds() + 1e-6 < time.seconds()) {  // 1usec is allowed error
    getTrackedObject(time, object);
  }
  using autoware_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;
  auto & pose_cov = object.pose_covariance;

  // principal component of the position covariance matrix
  Eigen::Matrix2d covariance;
  covariance << pose_cov[XYZRPY_COV_IDX::X_X], pose_cov[XYZRPY_COV_IDX::X_Y],
    pose_cov[XYZRPY_COV_IDX::Y_X], pose_cov[XYZRPY_COV_IDX::Y_Y];
  // check if the covariance is valid
  if (covariance(0, 0) <= 0.0 || covariance(1, 1) <= 0.0) {
    RCLCPP_WARN(
      rclcpp::get_logger("Tracker"), "Covariance is not valid. X_X: %f, Y_Y: %f", covariance(0, 0),
      covariance(1, 1));
    major_axis_sq = 0.0;
    minor_axis_sq = 0.0;
    return;
  }
  // Direct eigenvalue calculation for 2x2 symmetric matrix
  const double a = covariance(0, 0);
  const double b = covariance(0, 1);
  const double c = covariance(1, 1);
  const double trace = a + c;
  const double det = a * c - b * b;
  const double sqrt_term = std::sqrt(trace * trace / 4.0 - det);

  major_axis_sq = trace / 2.0 + sqrt_term;
  minor_axis_sq = trace / 2.0 - sqrt_term;
}

double Tracker::getBEVArea() const
{
  const auto & dims = object_.shape.dimensions;
  return dims.x * dims.y;
}

double Tracker::getDistanceSqToEgo(const std::optional<geometry_msgs::msg::Pose> & ego_pose) const
{
  constexpr double INVALID_DISTANCE_SQ = -1.0;
  if (!ego_pose) {
    return INVALID_DISTANCE_SQ;
  }
  const auto & p = object_.pose.position;
  const auto & e = ego_pose->position;
  const double dx = p.x - e.x;
  const double dy = p.y - e.y;
  return dx * dx + dy * dy;
}

double Tracker::computeAdaptiveThreshold(
  double base_threshold, double fallback_threshold, const AdaptiveThresholdCache & cache,
  const std::optional<geometry_msgs::msg::Pose> & ego_pose) const
{
  const double distance_sq = getDistanceSqToEgo(ego_pose);
  if (distance_sq < 0.0) return fallback_threshold;

  const double bev_area = getBEVArea();

  const double bev_area_influence = cache.getBEVAreaInfluence(bev_area);
  const double distance_influence = cache.getDistanceInfluence(distance_sq);

  return base_threshold + bev_area_influence + distance_influence;
}

bool Tracker::isConfident(
  const rclcpp::Time & time, const AdaptiveThresholdCache & cache,
  const std::optional<geometry_msgs::msg::Pose> & ego_pose) const
{
  // check the number of measurements. if the measurement is too small, definitely not confident
  const int count = getTotalMeasurementCount();
  if (count < 2) {
    return false;
  }

  double major_axis_sq = 0.0;
  double minor_axis_sq = 0.0;
  getPositionCovarianceEigenSq(time, major_axis_sq, minor_axis_sq);

  // if the covariance is very small, the tracker is confident
  constexpr double STRONG_COV_THRESHOLD = 0.28;
  if (major_axis_sq < STRONG_COV_THRESHOLD) {
    return true;
  }

  // if the existence probability is high and the covariance is small enough with respect to its
  // distance to ego and its bev area, the tracker is confident
  // base threshold is 1.6, fallback threshold is 2.6;
  const double adaptive_threshold = computeAdaptiveThreshold(1.6, 2.6, cache, ego_pose);

  if (getTotalExistenceProbability() > 0.50 && major_axis_sq < adaptive_threshold) {
    return true;
  }

  return false;
}

bool Tracker::isExpired(
  const rclcpp::Time & now, const AdaptiveThresholdCache & cache,
  const std::optional<geometry_msgs::msg::Pose> & ego_pose) const
{
  // check the number of no measurements
  const double elapsed_time = getElapsedTimeFromLastUpdate(now);

  // if the last measurement is too old, the tracker is expired
  constexpr double EXPIRED_TIME_THRESHOLD = 1.0;  // [sec]
  if (elapsed_time > EXPIRED_TIME_THRESHOLD) {
    return true;
  }

  // if the tracker is not confident, the tracker is expired
  constexpr double EXPIRED_PROBABILITY_THRESHOLD = 0.015;
  const float existence_probability = getTotalExistenceProbability();
  if (existence_probability < EXPIRED_PROBABILITY_THRESHOLD) {
    return true;
  }

  // if the tracker is a bit old and the existence probability is low, check the covariance size
  constexpr double TIME_TO_CHECK_COV = 0.18;  // [sec]
  constexpr double EXISTENCE_PROBABILITY_TO_CHECK_COV = 0.3;
  if (
    elapsed_time > TIME_TO_CHECK_COV &&
    existence_probability < EXISTENCE_PROBABILITY_TO_CHECK_COV) {
    // if the tracker covariance is too large, the tracker is expired
    double major_axis_sq = 0.0;
    double minor_axis_sq = 0.0;
    getPositionCovarianceEigenSq(now, major_axis_sq, minor_axis_sq);
    // major_cov: base_threshold is 2.8, fallback threshold is 3.8;
    // minor_cov: base_threshold is 2.7, fallback threshold is 3.7;
    const double major_cov_threshold = computeAdaptiveThreshold(2.8, 3.8, cache, ego_pose);
    const double minor_cov_threshold = computeAdaptiveThreshold(2.7, 3.7, cache, ego_pose);
    if (major_axis_sq > major_cov_threshold || minor_axis_sq > minor_cov_threshold) {
      return true;
    }
  }

  return false;
}

float Tracker::getKnownObjectProbability() const
{
  // find unknown probability
  float unknown_probability = 0.0;
  for (const auto & a_class : object_.classification) {
    if (a_class.label == autoware_perception_msgs::msg::ObjectClassification::UNKNOWN) {
      unknown_probability = a_class.probability;
      break;
    }
  }
  // known object probability is reverse of unknown probability
  return 1.0 - unknown_probability;
}

double Tracker::getPositionCovarianceDeterminant() const
{
  using autoware_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;
  auto & pose_cov = object_.pose_covariance;

  // The covariance size is defined as the square of the dominant eigenvalue
  // of the 2x2 covariance matrix:
  // | X_X  X_Y |
  // | Y_X  Y_Y |
  const double determinant = pose_cov[XYZRPY_COV_IDX::X_X] * pose_cov[XYZRPY_COV_IDX::Y_Y] -
                             pose_cov[XYZRPY_COV_IDX::X_Y] * pose_cov[XYZRPY_COV_IDX::Y_X];
  // covariance matrix is positive semi-definite
  if (determinant <= 0.0) {
    RCLCPP_WARN(
      rclcpp::get_logger("Tracker"), "Covariance is not positive semi-definite. X_X: %f, Y_Y: %f",
      pose_cov[XYZRPY_COV_IDX::X_X], pose_cov[XYZRPY_COV_IDX::Y_Y]);
    // return a large value to indicate the covariance is not valid
    return std::numeric_limits<double>::max();
  }
  return determinant;
}

}  // namespace autoware::multi_object_tracker
