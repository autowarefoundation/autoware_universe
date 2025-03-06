// Copyright 2025 TIER IV, Inc.
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

#ifndef AUTOWARE__SIMPL__ARCHETYPE__PREDICTION_HPP_
#define AUTOWARE__SIMPL__ARCHETYPE__PREDICTION_HPP_

#include "autoware/simpl/archetype/agent.hpp"
#include "autoware/simpl/archetype/exception.hpp"

#include <cstddef>
#include <utility>
#include <vector>

namespace autoware::simpl::archetype
{
/**
 * @brief Data container of predicted state at the specific time step.
 */
class PredictedState
{
public:
  PredictedState(double x, double y, double vx, double vy) : x(x), y(y), vx(vx), vy(vy) {}

  /**
   * @brief Return the number of predicted state attributes, which is `4`.
   */
  static size_t num_attribute() noexcept { return 4; }

  double x;   //!< Location x.
  double y;   //!< Location y.
  double vx;  //!< X-direction velocity.
  double vy;  //!< Y-direction velocity.
};

/**
 * @brief Data container of the prediction.
 */
class Prediction
{
public:
  using value_type = std::pair<double, std::vector<PredictedState>>;
  using size_type = std::vector<value_type>::size_type;
  using reference = std::vector<value_type>::reference;
  using const_reference = std::vector<value_type>::const_reference;
  using iterator = std::vector<value_type>::iterator;
  using const_iterator = std::vector<value_type>::const_iterator;

  /**
   * @brief Construct a new Prediction object.
   *
   * @param num_mode Number of predicted modes.
   * @param num_future Number of predicted future timestamps.
   * @param scores Predicted scores for each mode.
   * @param trajectories Predicted trajectories for each mode.
   */
  Prediction(
    size_t num_mode, size_t num_future, const std::vector<double> & scores,
    const std::vector<double> & trajectories)
  : num_mode(num_mode), num_future(num_future)
  {
    const size_t num_attribute = PredictedState::num_attribute();

    if (scores.size() != num_mode || trajectories.size() != num_mode * num_future * num_attribute) {
      throw SimplException(SimplError_t::InvalidValue, "Invalid prediction size.");
    }

    for (size_t m = 0; m < num_mode; ++m) {
      const auto & score = scores.at(m);
      std::vector<PredictedState> waypoints;
      for (size_t t = 0; t < num_future; ++t) {
        const size_t idx = (m * num_future + t) * num_attribute;
        auto x = trajectories[idx];
        auto y = trajectories[idx + 1];
        auto vx = trajectories[idx + 2];
        auto vy = trajectories[idx + 3];
        waypoints.emplace_back(x, y, vx, vy);
      }
      data_.emplace_back(score, waypoints);
    }
  }

  /**
   * @brief Return the read/write reference to the data at the specified mode index.
   *
   * @param m Mode index.
   */
  reference at(size_type m) noexcept { return data_.at(m); }

  /**
   * @brief Return the read only reference to the data at the specified mode index.
   *
   * @param m Mode index.
   */
  const_reference at(size_type m) const noexcept { return data_.at(m); }

  /**
   * @brief Return the read/write iterator that points the first state.
   */
  iterator begin() noexcept { return data_.begin(); }

  /**
   * @brief Return the read only iterator that points the first state.
   */
  const_iterator begin() const noexcept { return data_.begin(); }

  /**
   * @brief Return the read/write iterator that points the last state.
   */
  iterator end() noexcept { return data_.end(); }

  /**
   * @brief Return the read/write iterator that points the last state.
   */
  const_iterator end() const noexcept { return data_.end(); }

  size_t num_mode;    //!< Number of modes.
  size_t num_future;  //!< Number of future timestamps.

private:
  std::vector<value_type> data_;  //!< Container of prediction data.
};
}  // namespace autoware::simpl::archetype
#endif  // AUTOWARE__SIMPL__ARCHETYPE__PREDICTION_HPP_
