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

#ifndef AUTOWARE__TRAJECTORY_RANKER__INTERFACE__DATA_INTERFACE_HPP_
#define AUTOWARE__TRAJECTORY_RANKER__INTERFACE__DATA_INTERFACE_HPP_

#include "autoware/trajectory_ranker/data_structs.hpp"

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace autoware::trajectory_ranker
{

class DataInterface
{
public:
  explicit DataInterface(const std::shared_ptr<CoreData> & core_data, const size_t metrics_num)
  : core_data_{core_data},
    metrics_(metrics_num, std::vector<float>(core_data->points->size(), 0.0f)),
    scores_(metrics_num, 0.0f)
  {
  }

  void set_previous_points(const std::shared_ptr<TrajectoryPoints> & previous_points)
  {
    previous_points_ = previous_points;
  }

  void setup(const std::shared_ptr<TrajectoryPoints> & previous_points)
  {
    set_previous_points(previous_points);
  }

  void compress(const std::vector<std::vector<float>> & weight)
  {
    if (metrics_.empty() || weight.empty()) return;
    for (size_t i = 0; i < metrics_.size() && i < weight.size(); i++) {
      const auto & w = weight.at(i);
      const auto & metric = metrics_.at(i);
      scores_.at(i) = std::inner_product(w.begin(), w.end(), metric.begin(), 0.0f);
    }
  }

  void normalize(const float min, const float max, const size_t index, const bool flip = false)
  {
    if (std::abs(max - min) < std::numeric_limits<float>::epsilon()) {
      scores_.at(index) = 1.0f;
      return;
    }
    scores_.at(index) =
      flip ? (max - scores_.at(index)) / (max - min) : (scores_.at(index) - min) / (max - min);
  }

  void weighting(const std::vector<float> & weight)
  {
    if (weight.empty() || scores_.empty()) {
      total_ = 0.0f;
      return;
    }
    const size_t size = std::min(weight.size(), scores_.size());
    total_ = std::inner_product(weight.begin(), weight.begin() + size, scores_.begin(), 0.0f);
  }

  bool feasible() const
  {
    if (core_data_->points->empty()) return false;
    constexpr float epsilon = -1e-06f;

    const auto condition = [&epsilon](const auto & p) {
      return p.longitudinal_velocity_mps >= epsilon;
    };
    return std::all_of(core_data_->points->begin(), core_data_->points->end(), condition);
  }

  void set_metric(const size_t idx, const std::vector<float> & metric)
  {
    metrics_.at(idx) = metric;
  }

  float total() const { return total_; }

  float score(const size_t index) const { return scores_.at(index); }

  std::shared_ptr<TrajectoryPoints> points() const { return core_data_->points; }

  std::shared_ptr<TrajectoryPoints> previous() const { return core_data_->previous_points; }

  std::shared_ptr<TrajectoryPoints> original() const { return core_data_->original; }

  std::shared_ptr<PredictedObjects> objects() const { return core_data_->objects; }

  std::shared_ptr<SteeringReport> steering() const { return core_data_->steering; }

  std::shared_ptr<lanelet::ConstLanelets> preferred_lanes() const
  {
    return core_data_->preferred_lanes;
  }

  Header header() const { return core_data_->header; }

  UUID uuid() const { return core_data_->generator_id; }

  std::string tag() const { return core_data_->tag; }

private:
  std::shared_ptr<CoreData> core_data_;

  std::shared_ptr<TrajectoryPoints> previous_points_;

  std::vector<std::vector<float>> metrics_;

  std::vector<float> scores_;

  float total_;
};

}  // namespace autoware::trajectory_ranker

#endif  // AUTOWARE__TRAJECTORY_RANKER__INTERFACE__DATA_INTERFACE_HPP_
