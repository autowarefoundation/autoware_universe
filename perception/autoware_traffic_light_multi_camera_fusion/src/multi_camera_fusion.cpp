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
#include <set>
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

bool is_state_key_unknown(const StateKey & state_key)
{
  return state_key.size() == 1 &&
         state_key[0].first == tier4_perception_msgs::msg::TrafficLightElement::UNKNOWN;
}

bool compare_state_key_log_odds(
  const std::pair<StateKey, double> & key1, const std::pair<StateKey, double> & key2)
{
  // Ordering rule:
  // 1. Unknown StateKey is always lower priority
  // 2. Otherwise, smaller log-odds comes first
  const bool key1_is_unknown = is_state_key_unknown(key1.first);
  const bool key2_is_unknown = is_state_key_unknown(key2.first);
  if (key1_is_unknown && !key2_is_unknown) {
    return true;
  }
  if (!key1_is_unknown && key2_is_unknown) {
    return false;
  }
  return key1.second < key2.second;
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

void convert_output_msg(
  const std::map<MultiCameraFusion::IdType, utils::FusionRecord> & grouped_record_map,
  autoware_perception_msgs::msg::TrafficLightGroupArray & msg_out)
{
  msg_out.traffic_light_groups.clear();
  for (const auto & [regulatory_element_id, record] : grouped_record_map) {
    autoware_perception_msgs::msg::TrafficLightGroup signal_out;
    signal_out.traffic_light_group_id = regulatory_element_id;
    for (const auto & element : record.signal.elements) {
      signal_out.elements.push_back(utils::convert_t4_to_autoware(element));
    }
    msg_out.traffic_light_groups.push_back(signal_out);
  }
}

/**
 * @brief Handles the logic for tracking the best record for a given state.
 */
void update_best_record(
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

/**
 * @brief Collect the traffic light ids that were observed but are not registered in the map.
 */
std::vector<MultiCameraFusion::IdType> find_unmapped_traffic_light_ids(
  const std::map<MultiCameraFusion::IdType, utils::FusionRecord> & fused_record_map,
  const std::map<lanelet::Id, std::vector<lanelet::Id>> & traffic_light_id_to_regulatory_ele_id)
{
  std::vector<MultiCameraFusion::IdType> unmapped_traffic_light_ids;
  for (const auto & [traffic_light_id, record] : fused_record_map) {
    if (
      traffic_light_id_to_regulatory_ele_id.find(traffic_light_id) ==
      traffic_light_id_to_regulatory_ele_id.end()) {
      unmapped_traffic_light_ids.emplace_back(traffic_light_id);
    }
  }
  return unmapped_traffic_light_ids;
}

}  // namespace

std::map<MultiCameraFusion::IdType, utils::FusionRecord> multi_camera_fusion(
  std::multiset<utils::FusionRecordArr> & record_arr_set, double message_lifespan);

void update_log_odds(
  std::map<StateKey, double> & log_odds_map, const StateKey & state_key, double confidence,
  double prior_log_odds);

void update_group_info_for_element(
  GroupFusionInfoMap & group_fusion_info_map, const MultiCameraFusion::IdType & reg_ele_id,
  const utils::FusionRecord & record, double prior_log_odds);

MultiCameraFusion::MultiCameraFusion(const MultiCameraFusionConfig & config)
: config_(config),
  traffic_light_id_to_regulatory_ele_id_(
    build_traffic_light_id_to_regulatory_ele_id(config.lanelet_map_ptr))
{
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
  std::map<IdType, utils::FusionRecord> fused_record_map =
    multi_camera_fusion(record_arr_set_, config_.message_lifespan);
  result.unmapped_traffic_light_ids =
    find_unmapped_traffic_light_ids(fused_record_map, traffic_light_id_to_regulatory_ele_id_);
  GroupFusionResult group_result = group_fusion(fused_record_map);
  result.conflicted_regulatory_element_status = group_result.conflicts;

  NewSignalArrayType msg_out;
  convert_output_msg(group_result.grouped_record_map, msg_out);
  msg_out.stamp = cam_info.header.stamp;
  result.traffic_light_groups = msg_out;

  return result;
}

std::map<MultiCameraFusion::IdType, utils::FusionRecord> multi_camera_fusion(
  std::multiset<utils::FusionRecordArr> & record_arr_set, double message_lifespan)
{
  std::map<MultiCameraFusion::IdType, utils::FusionRecord> fused_record_map;
  const rclcpp::Time & newest_stamp(record_arr_set.rbegin()->header.stamp);
  for (auto it = record_arr_set.begin(); it != record_arr_set.end();) {
    /*
    remove all old record arrays whose timestamp difference with newest record is larger than
    threshold
    */
    if (
      (newest_stamp - rclcpp::Time(it->header.stamp)) >
      rclcpp::Duration::from_seconds(message_lifespan)) {
      it = record_arr_set.erase(it);
    } else {
      /*
      generate fused record result with the saved records
      */
      const utils::FusionRecordArr & record_arr = *it;
      for (size_t i = 0; i < record_arr.rois.rois.size(); i++) {
        const MultiCameraFusion::RoiType & roi = record_arr.rois.rois[i];
        auto signal_it = std::find_if(
          record_arr.signals.signals.begin(), record_arr.signals.signals.end(),
          [roi](const MultiCameraFusion::SignalType & s1) {
            return roi.traffic_light_id == s1.traffic_light_id;
          });
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
          utils::has_higher_or_equal_priority(record, fused_record_map[roi.traffic_light_id])) {
          fused_record_map[roi.traffic_light_id] = record;
        }
      }
      it++;
    }
  }
  return fused_record_map;
}

GroupFusionResult MultiCameraFusion::group_fusion(
  const std::map<IdType, utils::FusionRecord> & fused_record_map)
{
  // Stage 1: Accumulate evidence from all fused records
  const std::map<IdType, GroupFusionInfo> group_fusion_info_map =
    accumulate_group_evidence(fused_record_map);

  // Stage 2: Determine the best state for each group from the accumulated evidence
  GroupFusionResult result;
  result.conflicts = determine_best_group_state(group_fusion_info_map, result.grouped_record_map);
  return result;
}

GroupFusionInfoMap MultiCameraFusion::accumulate_group_evidence(
  const std::map<IdType, utils::FusionRecord> & fused_record_map)
{
  GroupFusionInfoMap group_fusion_info_map;
  for (const auto & [traffic_light_id, record] : fused_record_map) {
    process_fused_record(group_fusion_info_map, record);
  }
  return group_fusion_info_map;
}

/**
 * @brief Processes a single fused record and updates the group_fusion_info_map.
 */
void MultiCameraFusion::process_fused_record(
  GroupFusionInfoMap & group_fusion_info_map, const utils::FusionRecord & record)
{
  const IdType roi_id = record.roi.traffic_light_id;

  // Guard Clause 1: Check if traffic light ID is in the map
  const auto it = traffic_light_id_to_regulatory_ele_id_.find(roi_id);
  if (it == traffic_light_id_to_regulatory_ele_id_.end()) {
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
    update_group_info_for_element(
      group_fusion_info_map, reg_ele_id, record, config_.prior_log_odds);
  }
}

/**
 * @brief Updates the map for a single (element, regulatory_id) combination.
 */
void update_group_info_for_element(
  GroupFusionInfoMap & group_fusion_info_map, const MultiCameraFusion::IdType & reg_ele_id,
  const utils::FusionRecord & record, double prior_log_odds)
{
  StateKey state_key;
  for (const auto & element : record.signal.elements) {
    state_key.emplace_back(std::make_pair(element.color, element.shape));
  }
  const double confidence = utils::get_min_confidence(record.signal);
  auto & group_info = group_fusion_info_map[reg_ele_id];

  // Update Log-Odds
  update_log_odds(group_info.accumulated_log_odds, state_key, confidence, prior_log_odds);

  // Update Best Record
  update_best_record(group_info.best_record_for_state, state_key, confidence, record);
}

/**
 * @brief Handles the log-odds accumulation logic.
 */
void update_log_odds(
  std::map<StateKey, double> & log_odds_map, const StateKey & state_key, double confidence,
  double prior_log_odds)
{
  // try_emplace ensures we only add the 0.0 prior (from a 0.5 probability) once.
  log_odds_map.try_emplace(state_key, 0.0);

  const double evidence_log_odds = probability_to_log_odds(confidence);

  // Accumulate evidence
  log_odds_map[state_key] += evidence_log_odds - prior_log_odds;
}

std::vector<ConflictInfo> MultiCameraFusion::determine_best_group_state(
  const std::map<IdType, GroupFusionInfo> & group_fusion_info_map,
  std::map<IdType, utils::FusionRecord> & grouped_record_map) const
{
  std::vector<ConflictInfo> conflicted_regulatory_element_status;

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
      conflict_result = signal_validator::check_conflict(running_state, competitor_state);
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
      conflicted_regulatory_element_status.push_back({reg_ele_id, conflict_result.conflict_type});
    }
  }

  return conflicted_regulatory_element_status;
}

}  // namespace autoware::traffic_light
