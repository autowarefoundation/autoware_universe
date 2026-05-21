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

#include "multi_camera_fusion.hpp"

#include <autoware_lanelet2_extension/utility/query.hpp>
#include <rclcpp/time.hpp>

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <utility>
#include <vector>

namespace autoware::traffic_light
{

namespace
{

double probability_to_log_odds(double prob)
{
  /**
   * @brief Converts a probability value to log-odds.
   *
   * Log-odds is the logarithm of the odds ratio, i.e., log(p / (1-p)).
   * This function is essential for Bayesian updating in log-space, as it allows
   * evidence to be additively combined.
   *
   * The function handles edge cases where the probability `p` is very close to
   * 0 or 1. As `p` -> 1, log-odds -> +inf. As `p` -> 0, log-odds -> -inf.
   * To prevent floating-point divergence (infinity), the input probability is
   * "clamped" to a safe range slightly away from the boundaries. The bounds
   * [1e-9, 1.0 - 1e-9] are chosen as a small epsilon to ensure numerical
   * stability while having a negligible impact on non-extreme probability values.
   *
   * @param prob The input probability, expected to be in the range [0.0, 1.0].
   * @return The corresponding log-odds value.
   */
  prob = std::clamp(prob, 1e-9, 1.0 - 1e-9);
  return std::log(prob / (1.0 - prob));
}

/**
 * @brief get the state key that has best log-odds.
 */
inline StateKey get_best_state_key(const std::map<StateKey, double> & accumulated_log_odds)
{
  auto best_element = std::max_element(
    accumulated_log_odds.begin(), accumulated_log_odds.end(), compare_state_key_log_odds);

  StateKey best_state_key = best_element->first;

  return best_state_key;
}

std::map<lanelet::Id, std::vector<lanelet::Id>> build_traffic_light_id_to_regulatory_ele_id(
  const lanelet::LaneletMapPtr & lanelet_map_ptr)
{
  std::map<lanelet::Id, std::vector<lanelet::Id>> traffic_light_id_to_regulatory_ele_id;
  if (!lanelet_map_ptr) {
    return traffic_light_id_to_regulatory_ele_id;
  }
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr);
  std::vector<lanelet::AutowareTrafficLightConstPtr> all_lanelet_traffic_lights =
    lanelet::utils::query::autowareTrafficLights(all_lanelets);
  for (auto tl_itr = all_lanelet_traffic_lights.begin(); tl_itr != all_lanelet_traffic_lights.end();
       ++tl_itr) {
    lanelet::AutowareTrafficLightConstPtr tl = *tl_itr;

    auto lights = tl->trafficLights();
    for (const auto & light : lights) {
      traffic_light_id_to_regulatory_ele_id[light.id()].emplace_back(tl->id());
    }
  }
  return traffic_light_id_to_regulatory_ele_id;
}

}  // namespace

MultiCameraFusion::MultiCameraFusion(const MultiCameraFusionConfig & config)
: config_(config),
  traffic_light_id_to_regulatory_ele_id_(
    build_traffic_light_id_to_regulatory_ele_id(config.lanelet_map_ptr))
{
  if (config_.use_signal_consistency_check) {
    signal_validator_ = std::make_unique<SignalValidator>();
  }
}

MultiCameraFusionResult MultiCameraFusion::fuse(
  const CamInfoType & cam_info, const RoiArrayType & rois, const SignalArrayType & signals)
{
  /*
  Insert the received record array to the table.
  Attention should be payed that this record array might not have the newest timestamp
  */
  record_arr_set_.insert(utils::FusionRecordArr{cam_info.header, cam_info, rois, signals});

  MultiCameraFusionResult result;
  std::map<IdType, utils::FusionRecord> fused_record_map, grouped_record_map;
  multi_camera_fusion(fused_record_map);
  group_fusion(fused_record_map, grouped_record_map, result.unmapped_traffic_light_ids);

  NewSignalArrayType msg_out;
  convert_output_msg(grouped_record_map, msg_out);
  msg_out.stamp = cam_info.header.stamp;
  result.traffic_light_groups = msg_out;

  result.conflicted_regulatory_element_status = conflicted_regulatory_element_status_;
  return result;
}

void MultiCameraFusion::convert_output_msg(
  const std::map<IdType, utils::FusionRecord> & grouped_record_map, NewSignalArrayType & msg_out)
{
  msg_out.traffic_light_groups.clear();
  for (const auto & p : grouped_record_map) {
    IdType reg_ele_id = p.first;
    const SignalType & signal = p.second.signal;
    NewSignalType signal_out;
    signal_out.traffic_light_group_id = reg_ele_id;
    for (const auto & ele : signal.elements) {
      signal_out.elements.push_back(utils::convert_t4_to_autoware(ele));
    }
    msg_out.traffic_light_groups.push_back(signal_out);
  }
}

void MultiCameraFusion::multi_camera_fusion(
  std::map<IdType, utils::FusionRecord> & fused_record_map)
{
  fused_record_map.clear();
  const rclcpp::Time & newest_stamp(record_arr_set_.rbegin()->header.stamp);
  for (auto it = record_arr_set_.begin(); it != record_arr_set_.end();) {
    /*
    remove all old record arrays whose timestamp difference with newest record is larger than
    threshold
    */
    if (
      (newest_stamp - rclcpp::Time(it->header.stamp)) >
      rclcpp::Duration::from_seconds(config_.message_lifespan)) {
      it = record_arr_set_.erase(it);
    } else {
      /*
      generate fused record result with the saved records
      */
      const utils::FusionRecordArr & record_arr = *it;
      for (size_t i = 0; i < record_arr.rois.rois.size(); i++) {
        const RoiType & roi = record_arr.rois.rois[i];
        auto signal_it = std::find_if(
          record_arr.signals.signals.begin(), record_arr.signals.signals.end(),
          [roi](const SignalType & s1) { return roi.traffic_light_id == s1.traffic_light_id; });
        /*
        failed to find corresponding signal. skip it
        */
        if (signal_it == record_arr.signals.signals.end()) {
          continue;
        }
        utils::FusionRecord record{record_arr.header, record_arr.cam_info, roi, *signal_it};
        /*
        if this traffic light is not detected yet or can be updated by higher priority record,
        update it
        */
        if (
          fused_record_map.find(roi.traffic_light_id) == fused_record_map.end() ||
          utils::compare_record(record, fused_record_map[roi.traffic_light_id]) >= 0) {
          fused_record_map[roi.traffic_light_id] = record;
        }
      }
      it++;
    }
  }
}

void MultiCameraFusion::group_fusion(
  const std::map<IdType, utils::FusionRecord> & fused_record_map,
  std::map<IdType, utils::FusionRecord> & grouped_record_map,
  std::vector<IdType> & unmapped_traffic_light_ids)
{
  grouped_record_map.clear();

  // Stage 1: Accumulate evidence from all fused records
  const std::map<IdType, GroupFusionInfo> group_fusion_info_map =
    accumulate_group_evidence(fused_record_map, unmapped_traffic_light_ids);

  // Stage 2: Determine the best state for each group from the accumulated evidence
  determine_best_group_state(group_fusion_info_map, grouped_record_map);
}

GroupFusionInfoMap MultiCameraFusion::accumulate_group_evidence(
  const std::map<IdType, utils::FusionRecord> & fused_record_map,
  std::vector<IdType> & unmapped_traffic_light_ids)
{
  GroupFusionInfoMap group_fusion_info_map;
  for (const auto & p : fused_record_map) {
    process_fused_record(group_fusion_info_map, p.second, unmapped_traffic_light_ids);
  }
  return group_fusion_info_map;
}

/**
 * @brief Processes a single fused record and updates the group_fusion_info_map.
 * (This function contains the logic from the outer loop)
 */
void MultiCameraFusion::process_fused_record(
  GroupFusionInfoMap & group_fusion_info_map, const utils::FusionRecord & record,
  std::vector<IdType> & unmapped_traffic_light_ids)
{
  const IdType roi_id = record.roi.traffic_light_id;

  // Guard Clause 1: Check if traffic light ID is in the map
  const auto it = traffic_light_id_to_regulatory_ele_id_.find(roi_id);
  if (it == traffic_light_id_to_regulatory_ele_id_.end()) {
    unmapped_traffic_light_ids.emplace_back(roi_id);
    return;
  }

  // Guard Clause 2: Check for elements
  if (record.signal.elements.empty()) {
    return;
  }

  const auto & reg_ele_id_vec = it->second;  // Use the iterator

  // Loop over all regulatory IDs associated with this traffic light
  for (const auto & reg_ele_id : reg_ele_id_vec) {
    // Delegate the innermost logic to another helper
    update_group_info_for_element(group_fusion_info_map, reg_ele_id, record);
  }
}

/**
 * @brief Updates the map for a single (element, regulatory_id) combination.
 */
void MultiCameraFusion::update_group_info_for_element(
  GroupFusionInfoMap & group_fusion_info_map, const IdType & reg_ele_id,
  const utils::FusionRecord & record) const
{
  StateKey state_key;
  for (const auto & element : record.signal.elements) {
    state_key.emplace_back(std::make_pair(element.color, element.shape));
  }
  const double confidence = utils::get_min_confidence(record.signal);
  auto & group_info = group_fusion_info_map[reg_ele_id];

  // Update Log-Odds
  update_log_odds(group_info.accumulated_log_odds, state_key, confidence);

  // Update Best Record
  update_best_record(group_info.best_record_for_state, state_key, confidence, record);
}

/**
 * @brief Handles the log-odds accumulation logic.
 */
void MultiCameraFusion::update_log_odds(
  std::map<StateKey, double> & log_odds_map, const StateKey & state_key, double confidence) const
{
  // try_emplace ensures we only add the 0.0 prior (from a 0.5 probability) once.
  log_odds_map.try_emplace(state_key, 0.0);

  const double evidence_log_odds = probability_to_log_odds(confidence);

  // Accumulate evidence
  log_odds_map[state_key] += evidence_log_odds - config_.prior_log_odds;
}

/**
 * @brief Handles the logic for tracking the best record for a given state.
 */
void MultiCameraFusion::update_best_record(
  std::map<StateKey, utils::FusionRecord> & best_record_map, const StateKey & state_key,
  double confidence, const utils::FusionRecord & record)
{
  const auto it = best_record_map.find(state_key);

  if (it == best_record_map.end()) {
    best_record_map[state_key] = record;
    return;
  }

  auto & existing_record = it->second;

  if (existing_record.signal.elements.empty()) {
    return;
  }

  if (confidence > utils::get_min_confidence(existing_record.signal)) {
    best_record_map[state_key] = record;
  }
}

void MultiCameraFusion::determine_best_group_state(
  const std::map<IdType, GroupFusionInfo> & group_fusion_info_map,
  std::map<IdType, utils::FusionRecord> & grouped_record_map)
{
  conflicted_regulatory_element_status_.clear();

  for (const auto & pair : group_fusion_info_map) {
    const IdType reg_ele_id = pair.first;
    const auto & group_info = pair.second;

    if (group_info.accumulated_log_odds.empty()) {
      continue;
    }

    if (!config_.use_signal_consistency_check || group_info.accumulated_log_odds.size() == 1) {
      // use the most probable one (the highest logarithmic odds) as the base
      const StateKey best_state_key = get_best_state_key(group_info.accumulated_log_odds);
      grouped_record_map[reg_ele_id] = group_info.best_record_for_state.at(best_state_key);

      continue;
    }

    // only records with multiple state keys reach here
    // these indicate conflicts, except when unknown states are present

    auto log_odds_it = group_info.accumulated_log_odds.begin();
    StateKey running_state = (*log_odds_it).first;

    ConflictStatus conflict_result{ConflictType::PARTIAL_CONFLICT, running_state};

    // check if conflicts exist among the signals within the same regulatory element id
    for (++log_odds_it; log_odds_it != group_info.accumulated_log_odds.end(); ++log_odds_it) {
      const StateKey & competitor_state = (*log_odds_it).first;
      conflict_result = signal_validator_->check_conflict(running_state, competitor_state);
      running_state = conflict_result.common_state_key;

      if (conflict_result.conflict_type == ConflictType::CONFLICT) {
        // critical conflict will be overwritten with fail-safe record
        // we immediately exit the loop
        break;
      } else {  // partial conflict
        if (config_.publish_partial_matched_signal) {
          continue;
        } else {
          break;
        }
      }
    }

    if (
      conflict_result.conflict_type == ConflictType::CONFLICT ||
      !config_.publish_partial_matched_signal) {
      // use a fail-safe record as a fallback for this regulatory element.

      // use the most probable one (the highest logarithmic odds) as the base
      const StateKey best_state_key = get_best_state_key(group_info.accumulated_log_odds);

      // set the best record that signal is overwritten with fail-safe record
      grouped_record_map[reg_ele_id] =
        utils::generate_failsafe_record(group_info.best_record_for_state.at(best_state_key));
    } else {
      // partially conflicted and allowed to publish the matched signals

      // rebuild the record based on the matched signals
      // copy the base data from the best original record, but replace the elements
      const StateKey best_state_key = get_best_state_key(group_info.accumulated_log_odds);
      utils::FusionRecord merged_record = group_info.best_record_for_state.at(best_state_key);

      merged_record.signal.elements.clear();

      const double min_confidence =
        utils::get_min_confidence(group_info.best_record_for_state.at(best_state_key).signal);
      for (const auto & elem : running_state) {
        tier4_perception_msgs::msg::TrafficLightElement new_elem;
        new_elem.color = elem.first;
        new_elem.shape = elem.second;
        // keep the min confidence of the base record
        new_elem.confidence = min_confidence;

        merged_record.signal.elements.push_back(new_elem);
      }

      grouped_record_map[reg_ele_id] = merged_record;
    }

    // suppress diagnostics for comparisons with unknown
    if (conflict_result.conflict_type != ConflictType::NO_CONFLICT) {
      // record it for diagnostics
      conflicted_regulatory_element_status_.push_back({reg_ele_id, conflict_result.conflict_type});
    }
  }
}

}  // namespace autoware::traffic_light
