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

#ifndef MULTI_CAMERA_FUSION_HPP_
#define MULTI_CAMERA_FUSION_HPP_

#include "signal_validator.hpp"
#include "traffic_light_multi_camera_fusion_process.hpp"
#include "types.hpp"

#include <autoware_perception_msgs/msg/traffic_light_group.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tier4_perception_msgs/msg/traffic_light.hpp>
#include <tier4_perception_msgs/msg/traffic_light_array.hpp>
#include <tier4_perception_msgs/msg/traffic_light_roi.hpp>
#include <tier4_perception_msgs/msg/traffic_light_roi_array.hpp>

#include <lanelet2_core/Forward.h>

#include <map>
#include <set>
#include <vector>

namespace autoware::traffic_light
{

struct GroupFusionInfo
{
  std::map<StateKey, double> accumulated_log_odds;
  std::map<StateKey, utils::FusionRecord> best_record_for_state;
};

struct ConflictInfo
{
  tier4_perception_msgs::msg::TrafficLightRoi::_traffic_light_id_type id;
  ConflictType conflict_type;
};

using GroupFusionInfoMap =
  std::map<tier4_perception_msgs::msg::TrafficLightRoi::_traffic_light_id_type, GroupFusionInfo>;

struct GroupFusionResult
{
  std::map<tier4_perception_msgs::msg::TrafficLightRoi::_traffic_light_id_type, utils::FusionRecord>
    grouped_record_map;
  std::vector<ConflictInfo> conflicts;
};

struct MultiCameraFusionConfig
{
  /*
  For every input message input_m, if the timestamp difference between input_m and the latest
  message is smaller than message_lifespan, then input_m would be used for the fusion. Otherwise,
  it would be discarded.
  */
  double message_lifespan{1.0};
  /**
   * @brief The prior log-odds for a traffic light state.
   */
  double prior_log_odds{0.0};
  bool use_signal_consistency_check{false};
  bool publish_partial_matched_signal{false};
  lanelet::LaneletMapPtr lanelet_map_ptr{nullptr};
};

struct MultiCameraFusionResult
{
  std::vector<ConflictInfo> conflicted_regulatory_element_status;
  // Traffic light IDs that were observed by a camera but are not registered in the loaded map.
  // The Node logs a warning for each entry.
  std::vector<tier4_perception_msgs::msg::TrafficLightRoi::_traffic_light_id_type>
    unmapped_traffic_light_ids;
};

class MultiCameraFusion
{
public:
  using CamInfoType = sensor_msgs::msg::CameraInfo;
  using RoiType = tier4_perception_msgs::msg::TrafficLightRoi;
  using SignalType = tier4_perception_msgs::msg::TrafficLight;
  using SignalArrayType = tier4_perception_msgs::msg::TrafficLightArray;
  using RoiArrayType = tier4_perception_msgs::msg::TrafficLightRoiArray;
  using IdType = tier4_perception_msgs::msg::TrafficLightRoi::_traffic_light_id_type;
  using NewSignalType = autoware_perception_msgs::msg::TrafficLightGroup;
  using NewSignalArrayType = autoware_perception_msgs::msg::TrafficLightGroupArray;

  explicit MultiCameraFusion(const MultiCameraFusionConfig & config);
  MultiCameraFusion() = default;

  MultiCameraFusionResult fuse(
    const CamInfoType & cam_info, const RoiArrayType & rois, const SignalArrayType & signals,
    NewSignalArrayType & output_groups);

private:
  GroupFusionResult group_fusion(const std::map<IdType, utils::FusionRecord> & fused_record_map);

  /**
   * @brief Accumulates log-odds evidence for each traffic light group from individual fused
   * records.
   */
  GroupFusionInfoMap accumulate_group_evidence(
    const std::map<IdType, utils::FusionRecord> & fused_record_map);

  /**
   * @brief Processes a single fused record and updates the group_fusion_info_map.
   */
  void process_fused_record(
    GroupFusionInfoMap & group_fusion_info_map, const utils::FusionRecord & record);

  /**
   * @brief Determines the best state for each group based on accumulated evidence.
   * @return The conflicts detected during this call. Empty when no conflict is found.
   */
  std::vector<ConflictInfo> determine_best_group_state(
    const std::map<IdType, GroupFusionInfo> & group_fusion_info_map,
    std::map<IdType, utils::FusionRecord> & grouped_record_map) const;

  MultiCameraFusionConfig config_{};
  /*
  Mapping from traffic light instance id to regulatory element id (group id)
  */
  std::map<lanelet::Id, std::vector<lanelet::Id>> traffic_light_id_to_regulatory_ele_id_;
  /*
  Store record arrays in increasing timestamp order.
  Use multiset in case multiple cameras publish images at the exact same time.
  */
  std::multiset<utils::FusionRecordArr> record_arr_set_;
};

}  // namespace autoware::traffic_light

#endif  // MULTI_CAMERA_FUSION_HPP_
