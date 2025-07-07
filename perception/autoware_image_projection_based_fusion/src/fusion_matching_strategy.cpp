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

#include "autoware/image_projection_based_fusion/fusion_matching_strategy.hpp"

#include "autoware/image_projection_based_fusion/fusion_collector.hpp"
#include "autoware/image_projection_based_fusion/fusion_node.hpp"
#include "autoware/image_projection_based_fusion/fusion_types.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <cstddef>
#include <limits>
#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::image_projection_based_fusion
{

template <class Msg3D, class Msg2D, class ExportObj>
NaiveMatchingStrategy<Msg3D, Msg2D, ExportObj>::NaiveMatchingStrategy(
  std::shared_ptr<FusionNode<Msg3D, Msg2D, ExportObj>> && ros2_parent_node,
  const std::unordered_map<std::size_t, double> & id_to_offset_map)
: ros2_parent_node_(std::move(ros2_parent_node)), id_to_offset_map_(id_to_offset_map)
{
  if (!ros2_parent_node_) {
    throw std::runtime_error("ros2_parent_node is nullptr in NaiveMatchingStrategy constructor.");
  }

  threshold_ = ros2_parent_node_->template declare_parameter<double>("matching_strategy.threshold");

  RCLCPP_INFO(ros2_parent_node_->get_logger(), "Utilize naive matching strategy for fusion nodes.");
}

template <class Msg3D, class Msg2D, class ExportObj>
std::optional<std::shared_ptr<FusionCollector<Msg3D, Msg2D, ExportObj>>>
NaiveMatchingStrategy<Msg3D, Msg2D, ExportObj>::match_rois_to_collector(
  const std::list<std::shared_ptr<FusionCollector<Msg3D, Msg2D, ExportObj>>> & fusion_collectors,
  const std::shared_ptr<RoisMatchingContext> & matching_context) const
{
  double smallest_time_difference = std::numeric_limits<double>::max();
  std::shared_ptr<FusionCollector<Msg3D, Msg2D, ExportObj>> closest_collector = nullptr;

  for (const auto & fusion_collector : fusion_collectors) {
    if (!fusion_collector->rois_exists(matching_context->rois_id)) {
      if (
        auto naive_info =
          std::dynamic_pointer_cast<NaiveCollectorInfo>(fusion_collector->get_info())) {
        auto offset_it = id_to_offset_map_.find(matching_context->rois_id);
        if (offset_it == id_to_offset_map_.end()) {
          RCLCPP_ERROR(
            ros2_parent_node_->get_logger(), "Missing offset for rois_id: %zu",
            matching_context->rois_id);
          continue;
        }

        double time_difference =
          std::abs(matching_context->rois_timestamp - naive_info->timestamp - offset_it->second);
        if (time_difference < smallest_time_difference && time_difference < naive_info->threshold) {
          smallest_time_difference = time_difference;
          closest_collector = fusion_collector;
        }
      }
    }
  }

  return closest_collector;  // Implicitly returns std::nullopt if closest_collector is nullptr
}

template <class Msg3D, class Msg2D, class ExportObj>
std::optional<std::shared_ptr<FusionCollector<Msg3D, Msg2D, ExportObj>>>
NaiveMatchingStrategy<Msg3D, Msg2D, ExportObj>::match_msg3d_to_collector(
  const std::list<std::shared_ptr<FusionCollector<Msg3D, Msg2D, ExportObj>>> & fusion_collectors,
  const std::shared_ptr<Msg3dMatchingContext> & matching_context)
{
  double smallest_time_difference = std::numeric_limits<double>::max();
  std::shared_ptr<FusionCollector<Msg3D, Msg2D, ExportObj>> closest_collector = nullptr;

  for (const auto & fusion_collector : fusion_collectors) {
    if (!fusion_collector->msg3d_exists()) {
      if (
        auto naive_info =
          std::dynamic_pointer_cast<NaiveCollectorInfo>(fusion_collector->get_info())) {
        double time_difference =
          std::abs(matching_context->msg3d_timestamp - naive_info->timestamp);

        if (time_difference < smallest_time_difference && time_difference < naive_info->threshold) {
          smallest_time_difference = time_difference;
          closest_collector = fusion_collector;
        }
      }
    }
  }

  return closest_collector;  // Implicitly returns std::nullopt if closest_collector is nullptr
}

template <class Msg3D, class Msg2D, class ExportObj>
void NaiveMatchingStrategy<Msg3D, Msg2D, ExportObj>::set_collector_info(
  std::shared_ptr<FusionCollector<Msg3D, Msg2D, ExportObj>> & collector,
  const std::shared_ptr<MatchingContextBase> & matching_context)
{
  if (!matching_context) {
    RCLCPP_ERROR(ros2_parent_node_->get_logger(), "matching_context is nullptr!");
    return;
  }

  if (
    auto msg3d_matching_context =
      std::dynamic_pointer_cast<Msg3dMatchingContext>(matching_context)) {
    auto info =
      std::make_shared<NaiveCollectorInfo>(msg3d_matching_context->msg3d_timestamp, threshold_);
    collector->set_info(info);

  } else if (
    auto rois_matching_context = std::dynamic_pointer_cast<RoisMatchingContext>(matching_context)) {
    auto offset_it = id_to_offset_map_.find(rois_matching_context->rois_id);
    if (offset_it == id_to_offset_map_.end()) {
      RCLCPP_ERROR(
        ros2_parent_node_->get_logger(), "Missing offset for rois_id: %zu",
        rois_matching_context->rois_id);
      return;
    }

    auto info = std::make_shared<NaiveCollectorInfo>(
      rois_matching_context->rois_timestamp - offset_it->second, threshold_);
    collector->set_info(info);
  }
}

template <class Msg3D, class Msg2D, class ExportObj>
AdvancedMatchingStrategy<Msg3D, Msg2D, ExportObj>::AdvancedMatchingStrategy(
  std::shared_ptr<FusionNode<Msg3D, Msg2D, ExportObj>> && ros2_parent_node,
  const std::unordered_map<std::size_t, double> & id_to_offset_map)
: ros2_parent_node_(std::move(ros2_parent_node)), id_to_offset_map_(id_to_offset_map)
{
  if (!ros2_parent_node_) {
    throw std::runtime_error(
      "ros2_parent_node is nullptr in AdvancedMatchingStrategy constructor.");
  }

  msg3d_noise_window_ =
    ros2_parent_node_->template declare_parameter<double>("matching_strategy.msg3d_noise_window");
  auto rois_timestamp_noise_window =
    ros2_parent_node_->template declare_parameter<std::vector<double>>(
      "matching_strategy.rois_timestamp_noise_window");

  auto rois_number = id_to_offset_map_.size();
  if (rois_timestamp_noise_window.size() < rois_number) {
    throw std::runtime_error(
      "Mismatch: rois_number (" + std::to_string(rois_number) +
      ") does not match rois_timestamp_noise_window size (" +
      std::to_string(rois_timestamp_noise_window.size()) + ").");
  }

  for (size_t i = 0; i < rois_number; i++) {
    id_to_noise_window_map_[i] = rois_timestamp_noise_window[i];
  }

  RCLCPP_INFO(
    ros2_parent_node_->get_logger(), "Utilize advanced matching strategy for fusion nodes.");
}

template <class Msg3D, class Msg2D, class ExportObj>
std::optional<std::shared_ptr<FusionCollector<Msg3D, Msg2D, ExportObj>>>
AdvancedMatchingStrategy<Msg3D, Msg2D, ExportObj>::match_rois_to_collector(
  const std::list<std::shared_ptr<FusionCollector<Msg3D, Msg2D, ExportObj>>> & fusion_collectors,
  const std::shared_ptr<RoisMatchingContext> & matching_context) const
{
  auto offset_it = id_to_offset_map_.find(matching_context->rois_id);
  auto noise_it = id_to_noise_window_map_.find(matching_context->rois_id);

  if (offset_it == id_to_offset_map_.end() || noise_it == id_to_noise_window_map_.end()) {
    RCLCPP_ERROR(
      ros2_parent_node_->get_logger(), "Missing offset or noise window for rois_id: %zu",
      matching_context->rois_id);
    return std::nullopt;
  }

  double adjusted_timestamp = matching_context->rois_timestamp - offset_it->second;
  double noise_window = noise_it->second;

  for (const auto & fusion_collector : fusion_collectors) {
    if (
      auto advanced_info =
        std::dynamic_pointer_cast<AdvancedCollectorInfo>(fusion_collector->get_info())) {
      double reference_timestamp_min = advanced_info->timestamp - advanced_info->noise_window;
      double reference_timestamp_max = advanced_info->timestamp + advanced_info->noise_window;

      if (
        adjusted_timestamp < reference_timestamp_max + noise_window &&
        adjusted_timestamp > reference_timestamp_min - noise_window) {
        return fusion_collector;
      }
    }
  }
  return std::nullopt;
}

template <class Msg3D, class Msg2D, class ExportObj>
std::optional<std::shared_ptr<FusionCollector<Msg3D, Msg2D, ExportObj>>>
AdvancedMatchingStrategy<Msg3D, Msg2D, ExportObj>::match_msg3d_to_collector(
  const std::list<std::shared_ptr<FusionCollector<Msg3D, Msg2D, ExportObj>>> & fusion_collectors,
  const std::shared_ptr<Msg3dMatchingContext> & matching_context)
{
  auto concatenated_status =
    ros2_parent_node_->find_concatenation_status(matching_context->msg3d_timestamp);

  double offset = get_concatenated_offset(matching_context->msg3d_timestamp, concatenated_status);
  double adjusted_timestamp = matching_context->msg3d_timestamp - offset;

  for (const auto & fusion_collector : fusion_collectors) {
    if (
      auto advanced_info =
        std::dynamic_pointer_cast<AdvancedCollectorInfo>(fusion_collector->get_info())) {
      double reference_timestamp_min = advanced_info->timestamp - advanced_info->noise_window;
      double reference_timestamp_max = advanced_info->timestamp + advanced_info->noise_window;

      if (
        adjusted_timestamp < reference_timestamp_max + msg3d_noise_window_ &&
        adjusted_timestamp > reference_timestamp_min - msg3d_noise_window_) {
        return fusion_collector;
      }
    }
  }
  return std::nullopt;
}

template <class Msg3D, class Msg2D, class ExportObj>
void AdvancedMatchingStrategy<Msg3D, Msg2D, ExportObj>::set_collector_info(
  std::shared_ptr<FusionCollector<Msg3D, Msg2D, ExportObj>> & collector,
  const std::shared_ptr<MatchingContextBase> & matching_context)
{
  if (!matching_context) {
    RCLCPP_ERROR(ros2_parent_node_->get_logger(), "matching_context is nullptr!");
    return;
  }

  if (
    auto msg3d_matching_context =
      std::dynamic_pointer_cast<Msg3dMatchingContext>(matching_context)) {
    auto concatenated_status =
      ros2_parent_node_->find_concatenation_status(msg3d_matching_context->msg3d_timestamp);
    double offset =
      get_concatenated_offset(msg3d_matching_context->msg3d_timestamp, concatenated_status);

    auto info = std::make_shared<AdvancedCollectorInfo>(
      msg3d_matching_context->msg3d_timestamp - offset, msg3d_noise_window_);
    collector->set_info(info);
  } else if (
    auto rois_matching_context = std::dynamic_pointer_cast<RoisMatchingContext>(matching_context)) {
    auto offset_it = id_to_offset_map_.find(rois_matching_context->rois_id);
    auto noise_it = id_to_noise_window_map_.find(rois_matching_context->rois_id);

    if (offset_it == id_to_offset_map_.end() || noise_it == id_to_noise_window_map_.end()) {
      RCLCPP_ERROR(
        ros2_parent_node_->get_logger(), "Missing offset or noise window for rois_id: %zu",
        rois_matching_context->rois_id);
      return;
    }

    auto info = std::make_shared<AdvancedCollectorInfo>(
      rois_matching_context->rois_timestamp - offset_it->second, noise_it->second);
    collector->set_info(info);
  }
}

template <class Msg3D, class Msg2D, class ExportObj>
double AdvancedMatchingStrategy<Msg3D, Msg2D, ExportObj>::get_concatenated_offset(
  const double & msg3d_timestamp,
  const std::optional<std::unordered_map<std::string, std::string>> & concatenated_status)
{
  double offset = 0.0;

  if (concatenated_status.has_value()) {
    bool concatenation_success = false;
    const auto & status_map = concatenated_status.value();

    // Find required keys in the map
    auto concat_success_it = status_map.find("Pointcloud concatenation succeeded");

    if (concat_success_it != status_map.end()) {
      concatenation_success = (concat_success_it->second == "True");
      if (concatenation_success && database_created_) {
        return offset;  // 0.0
      }
    }

    auto ref_min_it = status_map.find("Minimum reference timestamp");
    auto ref_max_it = status_map.find("Maximum reference timestamp");
    if (ref_min_it != status_map.end() && ref_max_it != status_map.end()) {
      try {
        double reference_min = std::stod(ref_min_it->second);
        double reference_max = std::stod(ref_max_it->second);

        if (!concatenation_success && msg3d_timestamp > reference_max) {
          offset = msg3d_timestamp - (reference_min + (reference_max - reference_min) / 2);
        } else if (!database_created_ && concatenation_success) {
          auto concat_cloud_it = status_map.find("Concatenated pointcloud timestamp");
          if (concat_cloud_it != status_map.end()) {
            double concatenated_cloud_timestamp = std::stod(concat_cloud_it->second);
            update_fractional_timestamp_set(concatenated_cloud_timestamp);
            success_status_counter_++;
            offset = 0.0;

            if (success_status_counter_ > success_threshold) {
              database_created_ = true;
            }
          }
        }
      } catch (const std::exception & e) {
        RCLCPP_ERROR(ros2_parent_node_->get_logger(), "Failed to parse timestamp: %s", e.what());
      }
    }
  } else {
    if (database_created_) {
      offset = compute_offset(msg3d_timestamp);
      RCLCPP_DEBUG(ros2_parent_node_->get_logger(), "Using database, computed offset: %f", offset);
    } else {
      offset = 0.0;  // Database not created yet, expect the concatenation is successful
    }
  }

  return offset;
}

template <class Msg3D, class Msg2D, class ExportObj>
double AdvancedMatchingStrategy<Msg3D, Msg2D, ExportObj>::extract_fractional(double timestamp)
{
  return fmod(timestamp, 1.0);
}

template <class Msg3D, class Msg2D, class ExportObj>
void AdvancedMatchingStrategy<Msg3D, Msg2D, ExportObj>::update_fractional_timestamp_set(
  double timestamp)
{
  double fractional_part = extract_fractional(timestamp);

  // Check if the new timestamp belongs to an existing element within noise tolerance
  for (auto it = fractional_timestamp_set_.begin(); it != fractional_timestamp_set_.end(); ++it) {
    if (std::abs(fractional_part - *it) < msg3d_noise_window_ * 2) {
      // If it belongs to an existing group, average the timestamp
      double updated_value = (*it + fractional_part) / 2;
      fractional_timestamp_set_.erase(it);
      fractional_timestamp_set_.insert(updated_value);
      return;
    }
  }

  fractional_timestamp_set_.insert(fractional_part);
}

template <class Msg3D, class Msg2D, class ExportObj>
double AdvancedMatchingStrategy<Msg3D, Msg2D, ExportObj>::compute_offset(double input_timestamp)
{
  if (fractional_timestamp_set_.empty()) {
    return 0.0;
  }

  double fractional_part = extract_fractional(input_timestamp);
  double expected_timestamp = -1.0;

  // Check if input timestamp is within an existing timestamp ± noise_tolerance
  for (const auto & timestamp : fractional_timestamp_set_) {
    if (
      fractional_part >= timestamp - msg3d_noise_window_ &&
      fractional_part < timestamp + msg3d_noise_window_) {
      return 0.0;  // If within range, offset is zero
    }
  }

  // Find the closest timestamp ≤ fractional_part
  auto it = fractional_timestamp_set_.lower_bound(fractional_part);
  if (it == fractional_timestamp_set_.end()) {
    --it;
    expected_timestamp = floor(input_timestamp) + *it;
  } else if (it == fractional_timestamp_set_.begin()) {
    // **If `new_timestamp` is smaller than all stored timestamps, use the largest timestamp**
    expected_timestamp = floor(input_timestamp) - 1 + *fractional_timestamp_set_.rbegin();
  } else {
    --it;
    expected_timestamp = floor(input_timestamp) + *it;
  }
  return input_timestamp - expected_timestamp;
}

// pointpainting fusion
template class NaiveMatchingStrategy<PointCloudMsgType, RoiMsgType, DetectedObjects>;

// roi cluster fusion
template class NaiveMatchingStrategy<ClusterMsgType, RoiMsgType, ClusterMsgType>;

// roi detected-object fusion
template class NaiveMatchingStrategy<DetectedObjects, RoiMsgType, DetectedObjects>;

// roi pointcloud fusion
template class NaiveMatchingStrategy<PointCloudMsgType, RoiMsgType, ClusterMsgType>;

// segment pointcloud fusion
template class NaiveMatchingStrategy<PointCloudMsgType, Image, PointCloudMsgType>;

// pointpainting fusion
template class AdvancedMatchingStrategy<PointCloudMsgType, RoiMsgType, DetectedObjects>;

// roi cluster fusion
template class AdvancedMatchingStrategy<ClusterMsgType, RoiMsgType, ClusterMsgType>;

// roi detected-object fusion
template class AdvancedMatchingStrategy<DetectedObjects, RoiMsgType, DetectedObjects>;

// roi pointcloud fusion
template class AdvancedMatchingStrategy<PointCloudMsgType, RoiMsgType, ClusterMsgType>;

// segment pointcloud fusion
template class AdvancedMatchingStrategy<PointCloudMsgType, Image, PointCloudMsgType>;

}  // namespace autoware::image_projection_based_fusion
