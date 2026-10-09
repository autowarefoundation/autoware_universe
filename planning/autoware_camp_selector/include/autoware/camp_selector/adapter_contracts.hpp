// Copyright 2026 Xinchen Lin. Licensed under the Apache License, Version 2.0.
#ifndef AUTOWARE__CAMP_SELECTOR__ADAPTER_CONTRACTS_HPP_
#define AUTOWARE__CAMP_SELECTOR__ADAPTER_CONTRACTS_HPP_

#include <autoware/camp_selector/tensor_dimensions.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::camp_selector
{
struct RawTensorContext
{
  std::vector<float> lanes, route_lanes, route_speed_limits;
  bool route_has_traffic_light{false};
};

// Called on create_input_data's raw tensors, before infer. All batches share the map.
inline RawTensorContext capture_raw_tensor_context(
  const std::unordered_map<std::string, std::vector<float>> & inputs)
{
  const auto copy_first = [&inputs](const std::string & key, std::size_t count) {
    const auto & values = inputs.at(key);
    if (values.size() < count) throw std::invalid_argument("short raw tensor: " + key);
    return std::vector<float>(values.begin(), values.begin() + count);
  };
  RawTensorContext result;
  result.lanes = copy_first("lanes", NUM_SEGMENTS_IN_LANE * POINTS_PER_SEGMENT * SEGMENT_POINT_DIM);
  result.route_lanes =
    copy_first("route_lanes", NUM_SEGMENTS_IN_ROUTE * POINTS_PER_SEGMENT * SEGMENT_POINT_DIM);
  result.route_speed_limits = copy_first("route_lanes_speed_limit", NUM_SEGMENTS_IN_ROUTE);
  for (std::size_t point = 0; point < NUM_SEGMENTS_IN_ROUTE * POINTS_PER_SEGMENT; ++point) {
    for (std::size_t feature = 8; feature <= 11; ++feature) {
      if (result.route_lanes.at(point * SEGMENT_POINT_DIM + feature) > 0.5F)
        result.route_has_traffic_light = true;
    }
  }
  return result;
}

inline void validate_fixed_dp_contract(
  std::size_t model_k, int batch_size, const std::vector<double> & temperatures, bool snap_enabled,
  bool shift_x, const std::string & model_type)
{
  if (
    model_k != 8 || batch_size != 8 || temperatures.size() != 8 || temperatures.front() != 0.0 ||
    !std::all_of(temperatures.begin() + 1, temperatures.end(), [](double x) { return x == 1.0; }))
    throw std::invalid_argument("frozen CAMP pool requires K=8 and temperatures [0,1,1,1,1,1,1,1]");
  if (snap_enabled)
    throw std::invalid_argument("upstream snap follows row0; no public selected-plan setter");
  if (shift_x) throw std::invalid_argument("shift_x changes CAMP raw-plan/vehicle-frame alignment");
  if (model_type != "multi_step")
    throw std::invalid_argument("frozen CAMP adapter requires multi_step DP");
}

// One pending pool: asynchronous/stale/duplicate feedback cannot advance continuity.
// Plan can be CampWorldPlan, while these contracts remain testable without ROS/Eigen.
template <class Plan>
class PublishedPlanLedger
{
public:
  struct Previous
  {
    double origin_seconds;
    Plan states;
  };
  std::uint64_t stage(double origin_seconds, std::vector<Plan> plans)
  {
    if (!std::isfinite(origin_seconds) || plans.empty())
      throw std::invalid_argument("pending plan needs finite time and candidates");
    if (previous_ && origin_seconds < previous_->origin_seconds)
      throw std::invalid_argument("backward clock requires episode reset");
    if (next_id_ == UINT64_MAX) throw std::overflow_error("pool identity exhausted");
    pending_ = Pending{++next_id_, origin_seconds, std::move(plans)};
    return next_id_;
  }
  const Plan * candidate(std::uint64_t id, std::size_t row) const
  {
    if (!pending_ || pending_->id != id || row >= pending_->plans.size()) return nullptr;
    return &pending_->plans.at(row);
  }
  bool confirm_published(std::uint64_t id, std::size_t row)
  {
    const auto * plan = candidate(id, row);
    if (!plan) return false;
    previous_ = Previous{pending_->origin_seconds, *plan};
    pending_.reset();
    return true;
  }
  const std::optional<Previous> & previous() const { return previous_; }
  void discard_pending() { pending_.reset(); }
  void reset_episode()
  {
    pending_.reset();
    previous_.reset();
  }  // Never reuse ids.
private:
  struct Pending
  {
    std::uint64_t id;
    double origin_seconds;
    std::vector<Plan> plans;
  };
  std::uint64_t next_id_{0};
  std::optional<Pending> pending_;
  std::optional<Previous> previous_;
};
}  // namespace autoware::camp_selector
#endif
