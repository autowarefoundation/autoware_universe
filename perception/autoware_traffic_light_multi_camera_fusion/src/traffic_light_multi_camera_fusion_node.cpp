// Copyright 2023 TIER IV, Inc.
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

#include "traffic_light_multi_camera_fusion_node.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>

#include <algorithm>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace
{

double probabilityToLogOdds(double prob)
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

}  // namespace

namespace autoware::traffic_light
{

MultiCameraFusion::MultiCameraFusion(const rclcpp::NodeOptions & node_options)
: Node("traffic_light_multi_camera_fusion", node_options)
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;

  std::vector<std::string> camera_namespaces =
    this->declare_parameter<std::vector<std::string>>("camera_namespaces");
  is_approximate_sync_ = this->declare_parameter<bool>("approximate_sync");
  message_lifespan_ = this->declare_parameter<double>("message_lifespan");
  prior_log_odds_ = this->declare_parameter<double>("prior_log_odds");
  for (const std::string & camera_ns : camera_namespaces) {
    std::string signal_topic = camera_ns + "/classification/traffic_signals";
    std::string roi_topic = camera_ns + "/detection/rois";
    std::string cam_info_topic = camera_ns + "/camera_info";
    roi_subs_.emplace_back(
      new mf::Subscriber<RoiArrayType>(this, roi_topic, rclcpp::QoS{1}.get_rmw_qos_profile()));
    signal_subs_.emplace_back(new mf::Subscriber<SignalArrayType>(
      this, signal_topic, rclcpp::QoS{1}.get_rmw_qos_profile()));
    cam_info_subs_.emplace_back(new mf::Subscriber<CamInfoType>(
      this, cam_info_topic, rclcpp::SensorDataQoS().get_rmw_qos_profile()));
    if (is_approximate_sync_ == false) {
      exact_sync_subs_.emplace_back(new ExactSync(
        ExactSyncPolicy(10), *(cam_info_subs_.back()), *(roi_subs_.back()),
        *(signal_subs_.back())));
      exact_sync_subs_.back()->registerCallback(
        std::bind(&MultiCameraFusion::trafficSignalRoiCallback, this, _1, _2, _3));
    } else {
      approximate_sync_subs_.emplace_back(new ApproximateSync(
        ApproximateSyncPolicy(10), *(cam_info_subs_.back()), *(roi_subs_.back()),
        *(signal_subs_.back())));
      approximate_sync_subs_.back()->registerCallback(
        std::bind(&MultiCameraFusion::trafficSignalRoiCallback, this, _1, _2, _3));
    }
  }

  map_sub_ = create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
    "~/input/vector_map", rclcpp::QoS{1}.transient_local(),
    [this](const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg) {
      this->mapCallback(msg);
    });
  signal_pub_ = create_publisher<NewSignalArrayType>("~/output/traffic_signals", rclcpp::QoS{1});
}

void MultiCameraFusion::trafficSignalRoiCallback(
  const CamInfoType::ConstSharedPtr cam_info_msg, const RoiArrayType::ConstSharedPtr roi_msg,
  const SignalArrayType::ConstSharedPtr signal_msg)
{
  rclcpp::Time stamp(roi_msg->header.stamp);
  /*
  Insert the received record array to the table.
  Attention should be payed that this record array might not have the newest timestamp
  */
  record_arr_set_.insert(
    utils::FusionRecordArr{cam_info_msg->header, *cam_info_msg, *roi_msg, *signal_msg});

  std::map<IdType, utils::FusionRecord> fused_record_map, grouped_record_map;
  multiCameraFusion(fused_record_map);
  groupFusion(fused_record_map, grouped_record_map);

  NewSignalArrayType msg_out;
  convertOutputMsg(grouped_record_map, msg_out);
  msg_out.stamp = cam_info_msg->header.stamp;
  signal_pub_->publish(msg_out);
}

void MultiCameraFusion::mapCallback(
  const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr input_msg)
{
  lanelet::LaneletMapPtr lanelet_map_ptr = autoware::experimental::lanelet2_utils::remove_const(
    autoware::experimental::lanelet2_utils::from_autoware_map_msgs(*input_msg));
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr);
  std::vector<lanelet::AutowareTrafficLightConstPtr> all_lanelet_traffic_lights =
    lanelet::utils::query::autowareTrafficLights(all_lanelets);
  for (auto tl_itr = all_lanelet_traffic_lights.begin(); tl_itr != all_lanelet_traffic_lights.end();
       ++tl_itr) {
    lanelet::AutowareTrafficLightConstPtr tl = *tl_itr;

    auto lights = tl->trafficLights();
    for (const auto & light : lights) {
      traffic_light_id_to_regulatory_ele_id_[light.id()].emplace_back(tl->id());
    }
  }
}

void MultiCameraFusion::convertOutputMsg(
  const std::map<IdType, utils::FusionRecord> & grouped_record_map, NewSignalArrayType & msg_out)
{
  msg_out.traffic_light_groups.clear();
  for (const auto & p : grouped_record_map) {
    IdType reg_ele_id = p.first;
    const SignalType & signal = p.second.signal;
    NewSignalType signal_out;
    signal_out.traffic_light_group_id = reg_ele_id;
    for (const auto & ele : signal.elements) {
      signal_out.elements.push_back(utils::convertT4toAutoware(ele));
    }
    msg_out.traffic_light_groups.push_back(signal_out);
  }
}

void MultiCameraFusion::multiCameraFusion(std::map<IdType, utils::FusionRecord> & fused_record_map)
{
  fused_record_map.clear();
  /*
  this should not happen. Just in case
  */
  if (record_arr_set_.empty()) {
    RCLCPP_ERROR(get_logger(), "record_arr_set_ is empty! This should not happen");
    return;
  }
  const rclcpp::Time & newest_stamp(record_arr_set_.rbegin()->header.stamp);
  for (auto it = record_arr_set_.begin(); it != record_arr_set_.end();) {
    /*
    remove all old record arrays whose timestamp difference with newest record is larger than
    threshold
    */
    if (
      (newest_stamp - rclcpp::Time(it->header.stamp)) >
      rclcpp::Duration::from_seconds(message_lifespan_)) {
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
          utils::compareRecord(record, fused_record_map[roi.traffic_light_id]) >= 0) {
          fused_record_map[roi.traffic_light_id] = record;
        }
      }
      it++;
    }
  }
}

void MultiCameraFusion::groupFusion(
  const std::map<IdType, utils::FusionRecord> & fused_record_map,
  std::map<IdType, utils::FusionRecord> & grouped_record_map)
{
  grouped_record_map.clear();

  // Stage 1: Accumulate evidence from all fused records
  const std::map<IdType, GroupFusionInfo> group_fusion_info_map =
    accumulateGroupEvidence(fused_record_map);

  // Stage 2: Determine the best state for each group from the accumulated evidence
  determineBestGroupState(group_fusion_info_map, grouped_record_map);
}

GroupFusionInfoMap MultiCameraFusion::accumulateGroupEvidence(
  const std::map<IdType, utils::FusionRecord> & fused_record_map)
{
  GroupFusionInfoMap group_fusion_info_map;
  for (const auto & p : fused_record_map) {
    processFusedRecord(group_fusion_info_map, p.second);
  }
  return group_fusion_info_map;
}

/**
 * @brief Processes a single fused record and updates the group_fusion_info_map.
 * (This function contains the logic from the outer loop)
 */
void MultiCameraFusion::processFusedRecord(
  GroupFusionInfoMap & group_fusion_info_map, const utils::FusionRecord & record)
{
  const IdType roi_id = record.roi.traffic_light_id;

  // Guard Clause 1: Check if traffic light ID is in the map
  const auto it = traffic_light_id_to_regulatory_ele_id_.find(roi_id);
  if (it == traffic_light_id_to_regulatory_ele_id_.end()) {
    RCLCPP_WARN_STREAM(
      get_logger(), "Found Traffic Light Id = " << roi_id << " which is not defined in Map");
    return;
  }

  // Guard Clause 2: Check for elements
  if (record.signal.elements.empty()) {
    return;
  }

  const auto & reg_ele_id_vec = it->second;  // Use the iterator

  // Loop over all elements in this record
  for (const auto & element : record.signal.elements) {
    // Loop over all regulatory IDs associated with this traffic light
    for (const auto & reg_ele_id : reg_ele_id_vec) {
      // Delegate the innermost logic to another helper
      updateGroupInfoForElement(group_fusion_info_map, reg_ele_id, element, record);
    }
  }
}

/**
 * @brief Updates the map for a single (element, regulatory_id) combination.
 */
void MultiCameraFusion::updateGroupInfoForElement(
  GroupFusionInfoMap & group_fusion_info_map, const IdType & reg_ele_id,
  const tier4_perception_msgs::msg::TrafficLightElement & element,
  const utils::FusionRecord & record)
{
  const StateKey state_key = {element.color, element.shape};
  const double confidence = element.confidence;
  auto & group_info = group_fusion_info_map[reg_ele_id];

  // Update Log-Odds
  updateLogOdds(group_info.accumulated_log_odds, state_key, confidence);

  // Update Best Record
  updateBestRecord(group_info.best_record_for_state, state_key, confidence, record);
}

/**
 * @brief Handles the log-odds accumulation logic.
 */
void MultiCameraFusion::updateLogOdds(
  std::map<StateKey, double> & log_odds_map, const StateKey & state_key, double confidence)
{
  // try_emplace ensures we only add the 0.0 prior (from a 0.5 probability) once.
  log_odds_map.try_emplace(state_key, 0.0);

  const double evidence_log_odds = probabilityToLogOdds(confidence);

  // Accumulate evidence
  log_odds_map[state_key] += evidence_log_odds - prior_log_odds_;
}

/**
 * @brief Handles the logic for tracking the best record for a given state.
 */
void MultiCameraFusion::updateBestRecord(
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

  if (confidence > existing_record.signal.elements[0].confidence) {
    best_record_map[state_key] = record;
  }
}

void MultiCameraFusion::determineBestGroupState(
  const std::map<IdType, GroupFusionInfo> & group_fusion_info_map,
  std::map<IdType, utils::FusionRecord> & grouped_record_map)
{
  for (const auto & pair : group_fusion_info_map) {
    const IdType reg_ele_id = pair.first;
    const auto & group_info = pair.second;

    if (group_info.accumulated_log_odds.empty()) {
      continue;
    }

    // The color with the highest logarithmic odds is the most probable one.
    auto best_element = std::max_element(
      group_info.accumulated_log_odds.begin(), group_info.accumulated_log_odds.end(),
      [](const auto & a, const auto & b) { return a.second < b.second; });

    const StateKey best_state_key = best_element->first;
    grouped_record_map[reg_ele_id] = group_info.best_record_for_state.at(best_state_key);
  }
}

}  // namespace autoware::traffic_light

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::traffic_light::MultiCameraFusion)
