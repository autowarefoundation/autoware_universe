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

#ifndef TRAFFIC_LIGHT_MULTI_CAMERA_FUSION_NODE_HPP_
#define TRAFFIC_LIGHT_MULTI_CAMERA_FUSION_NODE_HPP_

#include "traffic_light_multi_camera_fusion_process.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tier4_perception_msgs/msg/traffic_light.hpp>
#include <tier4_perception_msgs/msg/traffic_light_array.hpp>
#include <tier4_perception_msgs/msg/traffic_light_roi.hpp>
#include <tier4_perception_msgs/msg/traffic_light_roi_array.hpp>

#include <lanelet2_core/Forward.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

#include <list>
#include <map>
#include <memory>
#include <set>
#include <utility>
#include <vector>

namespace autoware::traffic_light
{
namespace mf = message_filters;

using StateKey = std::pair<uint8_t, uint8_t>;

struct GroupFusionInfo
{
  std::map<StateKey, double> accumulated_log_odds;
  std::map<StateKey, utils::FusionRecord> best_record_for_state;
};

using GroupFusionInfoMap = std::map<tier4_perception_msgs::msg::TrafficLightRoi::_traffic_light_id_type, GroupFusionInfo>;

class MultiCameraFusion : public rclcpp::Node
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

  using RecordArrayType = std::pair<RoiArrayType, SignalArrayType>;

  explicit MultiCameraFusion(const rclcpp::NodeOptions & node_options);

private:
  void trafficSignalRoiCallback(
    const CamInfoType::ConstSharedPtr cam_info_msg, const RoiArrayType::ConstSharedPtr roi_msg,
    const SignalArrayType::ConstSharedPtr signal_msg);

  void mapCallback(const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr input_msg);

  void multiCameraFusion(std::map<IdType, utils::FusionRecord> & fused_record_map);

  void convertOutputMsg(
    const std::map<IdType, utils::FusionRecord> & grouped_record_map, NewSignalArrayType & msg_out);

  void groupFusion(
    const std::map<IdType, utils::FusionRecord> & fused_record_map,
    std::map<IdType, utils::FusionRecord> & grouped_record_map);

  /**
   * @brief Accumulates log-odds evidence for each traffic light group from individual fused records.
   */
  GroupFusionInfoMap accumulateGroupEvidence(
    const std::map<IdType, utils::FusionRecord> & fused_record_map);
  
  /**
   * @brief Processes a single fused record and updates the group_fusion_info_map.
   */
  void processFusedRecord(
    GroupFusionInfoMap & group_fusion_info_map, const utils::FusionRecord & record);

  /**
   * @brief Updates the map for a single (element, regulatory_id) combination.
   */
  void updateGroupInfoForElement(
    GroupFusionInfoMap & group_fusion_info_map, const IdType & reg_ele_id,
    const tier4_perception_msgs::msg::TrafficLightElement & element,
    const utils::FusionRecord & record);

  /**
   * @brief Handles the log-odds accumulation logic.
   */
  void updateLogOdds(
    std::map<StateKey, double> & log_odds_map, const StateKey & state_key, double confidence);

  /**
   * @brief Handles the logic for tracking the best record for a given state.
   */
  void updateBestRecord(
    std::map<StateKey, utils::FusionRecord> & best_record_map, const StateKey & state_key,
    double confidence, const utils::FusionRecord & record);
  /**
   * @brief Determines the best state for each group based on accumulated evidence.
   */
  void determineBestGroupState(
    const std::map<IdType, GroupFusionInfo> & group_fusion_info_map,
    std::map<IdType, utils::FusionRecord> & grouped_record_map);

  using ExactSyncPolicy = mf::sync_policies::ExactTime<CamInfoType, RoiArrayType, SignalArrayType>;
  using ExactSync = mf::Synchronizer<ExactSyncPolicy>;
  using ApproximateSyncPolicy =
    mf::sync_policies::ApproximateTime<CamInfoType, RoiArrayType, SignalArrayType>;
  using ApproximateSync = mf::Synchronizer<ApproximateSyncPolicy>;

  std::vector<std::unique_ptr<mf::Subscriber<SignalArrayType>>> signal_subs_;
  std::vector<std::unique_ptr<mf::Subscriber<RoiArrayType>>> roi_subs_;
  std::vector<std::unique_ptr<mf::Subscriber<CamInfoType>>> cam_info_subs_;
  std::vector<std::unique_ptr<ExactSync>> exact_sync_subs_;
  std::vector<std::unique_ptr<ApproximateSync>> approximate_sync_subs_;
  rclcpp::Subscription<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr map_sub_;

  rclcpp::Publisher<NewSignalArrayType>::SharedPtr signal_pub_;
  /*
  Mapping from traffic light instance id to regulatory element id (group id)
  */
  std::map<lanelet::Id, std::vector<lanelet::Id>> traffic_light_id_to_regulatory_ele_id_;
  /*
  Store record arrays in increasing timestamp order.
  Use multiset in case multiple cameras publish images at the exact same time.
  */
  std::multiset<utils::FusionRecordArr> record_arr_set_;
  bool is_approximate_sync_;
  /*
  For every input message input_m, if the timestamp difference between input_m and the latest
  message is smaller than message_lifespan_, then input_m would be used for the fusion. Otherwise,
  it would be discarded.
  */
  double message_lifespan_;
  /**
   * @brief The prior log-odds for a traffic light state.
   */
  double prior_log_odds_;
};
}  // namespace autoware::traffic_light
#endif  // TRAFFIC_LIGHT_MULTI_CAMERA_FUSION_NODE_HPP_