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

#pragma once

#include "autoware/image_projection_based_fusion/fusion_collector.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cstddef>
#include <list>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>

namespace autoware::image_projection_based_fusion
{

template <class Msg3D, class Msg2D, class ExportObj>
class FusionNode;

struct MatchingContextBase
{
  virtual ~MatchingContextBase() = default;
};

struct Msg3dMatchingContext : public MatchingContextBase
{
  double msg3d_timestamp;

  explicit Msg3dMatchingContext(double msg3d_timestamp = 0.0) : msg3d_timestamp(msg3d_timestamp) {}
};

struct RoisMatchingContext : public MatchingContextBase
{
  double rois_timestamp;
  std::size_t rois_id;

  explicit RoisMatchingContext(double rois_timestamp = 0.0, std::size_t rois_id = 0)
  : rois_timestamp(rois_timestamp), rois_id(rois_id)
  {
  }
};

template <class Msg3D, class Msg2D, class ExportObj>
class FusionMatchingStrategy
{
public:
  virtual ~FusionMatchingStrategy() = default;

  [[nodiscard]] virtual std::optional<std::shared_ptr<FusionCollector<Msg3D, Msg2D, ExportObj>>>
  match_rois_to_collector(
    const std::list<std::shared_ptr<FusionCollector<Msg3D, Msg2D, ExportObj>>> & fusion_collectors,
    const std::shared_ptr<RoisMatchingContext> & matching_context) const = 0;

  [[nodiscard]] virtual std::optional<std::shared_ptr<FusionCollector<Msg3D, Msg2D, ExportObj>>>
  match_msg3d_to_collector(
    const std::list<std::shared_ptr<FusionCollector<Msg3D, Msg2D, ExportObj>>> & fusion_collectors,
    const std::shared_ptr<Msg3dMatchingContext> & matching_context) = 0;
  virtual void set_collector_info(
    std::shared_ptr<FusionCollector<Msg3D, Msg2D, ExportObj>> & collector,
    const std::shared_ptr<MatchingContextBase> & matching_context) = 0;
};

template <class Msg3D, class Msg2D, class ExportObj>
class NaiveMatchingStrategy : public FusionMatchingStrategy<Msg3D, Msg2D, ExportObj>
{
public:
  explicit NaiveMatchingStrategy(
    std::shared_ptr<FusionNode<Msg3D, Msg2D, ExportObj>> && ros2_parent_node,
    const std::unordered_map<std::size_t, double> & id_to_offset_map);

  [[nodiscard]] std::optional<std::shared_ptr<FusionCollector<Msg3D, Msg2D, ExportObj>>>
  match_rois_to_collector(
    const std::list<std::shared_ptr<FusionCollector<Msg3D, Msg2D, ExportObj>>> & fusion_collectors,
    const std::shared_ptr<RoisMatchingContext> & matching_context) const override;

  [[nodiscard]] std::optional<std::shared_ptr<FusionCollector<Msg3D, Msg2D, ExportObj>>>
  match_msg3d_to_collector(
    const std::list<std::shared_ptr<FusionCollector<Msg3D, Msg2D, ExportObj>>> & fusion_collectors,
    const std::shared_ptr<Msg3dMatchingContext> & matching_context) override;

  void set_collector_info(
    std::shared_ptr<FusionCollector<Msg3D, Msg2D, ExportObj>> & collector,
    const std::shared_ptr<MatchingContextBase> & matching_context) override;

private:
  std::shared_ptr<FusionNode<Msg3D, Msg2D, ExportObj>> ros2_parent_node_;
  std::unordered_map<std::size_t, double> id_to_offset_map_;
  double threshold_;
};

template <class Msg3D, class Msg2D, class ExportObj>
class AdvancedMatchingStrategy : public FusionMatchingStrategy<Msg3D, Msg2D, ExportObj>
{
public:
  explicit AdvancedMatchingStrategy(
    std::shared_ptr<FusionNode<Msg3D, Msg2D, ExportObj>> && ros2_parent_node,
    const std::unordered_map<std::size_t, double> & id_to_offset_map);

  [[nodiscard]] std::optional<std::shared_ptr<FusionCollector<Msg3D, Msg2D, ExportObj>>>
  match_rois_to_collector(
    const std::list<std::shared_ptr<FusionCollector<Msg3D, Msg2D, ExportObj>>> & fusion_collectors,
    const std::shared_ptr<RoisMatchingContext> & matching_context) const override;

  [[nodiscard]] std::optional<std::shared_ptr<FusionCollector<Msg3D, Msg2D, ExportObj>>>
  match_msg3d_to_collector(
    const std::list<std::shared_ptr<FusionCollector<Msg3D, Msg2D, ExportObj>>> & fusion_collectors,
    const std::shared_ptr<Msg3dMatchingContext> & matching_context) override;
  void set_collector_info(
    std::shared_ptr<FusionCollector<Msg3D, Msg2D, ExportObj>> & collector,
    const std::shared_ptr<MatchingContextBase> & matching_context) override;

  double get_concatenated_offset(
    const double & msg3d_timestamp,
    const std::optional<std::unordered_map<std::string, std::string>> & concatenated_status);

  double extract_fractional(double timestamp);
  void update_fractional_timestamp_set(double new_timestamp);
  double compute_offset(double input_timestamp);

private:
  std::shared_ptr<FusionNode<Msg3D, Msg2D, ExportObj>> ros2_parent_node_;
  std::unordered_map<std::size_t, double> id_to_offset_map_;
  std::unordered_map<std::size_t, double> id_to_noise_window_map_;
  double msg3d_noise_window_;
  std::set<double> fractional_timestamp_set_;  // Use set to store unique fractional timestamps
  int success_status_counter_{0};
  static constexpr int success_threshold{100};
  bool database_created_{false};
};
}  // namespace autoware::image_projection_based_fusion
