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

#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::traffic_light
{

MultiCameraFusionNode::MultiCameraFusionNode(const rclcpp::NodeOptions & node_options)
: Node("traffic_light_multi_camera_fusion", node_options)
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;

  std::vector<std::string> camera_namespaces =
    this->declare_parameter<std::vector<std::string>>("camera_namespaces");
  const bool is_approximate_sync = this->declare_parameter<bool>("approximate_sync");

  fusion_config_.message_lifespan = this->declare_parameter<double>("message_lifespan");
  fusion_config_.prior_log_odds = this->declare_parameter<double>("prior_log_odds");

  fusion_config_.use_signal_consistency_check =
    this->declare_parameter<bool>("signal_consistency_check.enable");
  fusion_config_.publish_partial_matched_signal =
    this->declare_parameter<bool>("signal_consistency_check.publish_partial_matched_signal");

  fusion_ = MultiCameraFusion(fusion_config_);

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
    if (is_approximate_sync == false) {
      exact_sync_subs_.emplace_back(new ExactSync(
        ExactSyncPolicy(10), *(cam_info_subs_.back()), *(roi_subs_.back()),
        *(signal_subs_.back())));
      exact_sync_subs_.back()->registerCallback(
        std::bind(&MultiCameraFusionNode::traffic_signal_roi_callback, this, _1, _2, _3));
    } else {
      approximate_sync_subs_.emplace_back(new ApproximateSync(
        ApproximateSyncPolicy(10), *(cam_info_subs_.back()), *(roi_subs_.back()),
        *(signal_subs_.back())));
      approximate_sync_subs_.back()->registerCallback(
        std::bind(&MultiCameraFusionNode::traffic_signal_roi_callback, this, _1, _2, _3));
    }
  }

  map_sub_ = create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
    "~/input/vector_map", rclcpp::QoS{1}.transient_local(),
    [this](const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg) {
      this->map_callback(msg);
    });
  signal_pub_ =
    AUTOWARE_CREATE_PUBLISHER2(NewSignalArrayType, "~/output/traffic_signals", rclcpp::QoS{1});

  diagnostics_interface_ptr_ =
    std::make_unique<autoware_utils::DiagnosticsInterface>(this, "traffic light conflict status");
}

void MultiCameraFusionNode::traffic_signal_roi_callback(
  const CamInfoType::ConstSharedPtr cam_info_msg, const RoiArrayType::ConstSharedPtr roi_msg,
  const SignalArrayType::ConstSharedPtr signal_msg)
{
  rclcpp::Time stamp(roi_msg->header.stamp);

  auto msg_out = ALLOCATE_OUTPUT_MESSAGE_UNIQUE(signal_pub_);
  const MultiCameraFusionResult result =
    fusion_.fuse(*cam_info_msg, *roi_msg, *signal_msg, *msg_out);
  for (const auto & unmapped_id : result.unmapped_traffic_light_ids) {
    RCLCPP_WARN_STREAM(
      get_logger(), "Found Traffic Light Id = " << unmapped_id << " which is not defined in Map");
  }
  signal_pub_->publish(std::move(msg_out));

  if (result.conflicted_regulatory_element_status.size() > 0) {
    publish_diagnostics(result.conflicted_regulatory_element_status, stamp);
  }
}

void MultiCameraFusionNode::map_callback(
  const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr input_msg)
{
  fusion_config_.lanelet_map_ptr = autoware::experimental::lanelet2_utils::remove_const(
    autoware::experimental::lanelet2_utils::from_autoware_map_msgs(*input_msg));
  fusion_ = MultiCameraFusion(fusion_config_);
}

void MultiCameraFusionNode::publish_diagnostics(
  const std::vector<ConflictInfo> & conflicted_regulatory_element_status, rclcpp::Time stamp)
{
  diagnostics_interface_ptr_->clear();

  // publish only conflicted RE status
  for (const auto & conflicted_re : conflicted_regulatory_element_status) {
    diagnostics_interface_ptr_->add_key_value(
      std::to_string(static_cast<int64_t>(conflicted_re.id)),
      static_cast<int>(conflicted_re.conflict_type));
  }

  diagnostics_interface_ptr_->update_level_and_message(
    diagnostic_msgs::msg::DiagnosticStatus::WARN,
    "Detected traffic light signal conflict in the fusion process.");

  diagnostics_interface_ptr_->publish(stamp);
}

}  // namespace autoware::traffic_light

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::traffic_light::MultiCameraFusionNode)
