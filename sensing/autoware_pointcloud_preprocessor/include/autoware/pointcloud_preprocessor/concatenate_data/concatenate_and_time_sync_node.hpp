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

#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

// ROS includes
#include "cloud_collector.hpp"
#include "collector_matching_strategy.hpp"
#include "combine_cloud_handler.hpp"
#include "traits.hpp"

#include <autoware_utils/ros/debug_publisher.hpp>
#include <autoware_utils/ros/diagnostics_interface.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>

#include <autoware_internal_debug_msgs/msg/int32_stamped.hpp>
#include <autoware_internal_debug_msgs/msg/string_stamped.hpp>
#include <autoware_sensing_msgs/msg/concatenated_point_cloud_info.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <message_filters/pass_through.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

namespace autoware::pointcloud_preprocessor
{

template <typename MsgTraits>
class PointCloudConcatenateDataSynchronizerComponentTemplated : public rclcpp::Node
{
public:
  using PointCloudMessage = typename MsgTraits::PointCloudMessage;
  using PublisherType = typename MsgTraits::PublisherType;
  using SubscriberType = typename MsgTraits::SubscriberType;

  explicit PointCloudConcatenateDataSynchronizerComponentTemplated(
    const rclcpp::NodeOptions & node_options);
  ~PointCloudConcatenateDataSynchronizerComponentTemplated() override = default;

  void publish_clouds(
    ConcatenatedCloudResult<MsgTraits> && concatenated_cloud_result,
    std::shared_ptr<CollectorInfoBase> collector_info);

  void manage_collector_list();
  void delete_collector(CloudCollector<MsgTraits> & cloud_collector);

  std::list<std::shared_ptr<CloudCollector<MsgTraits>>> get_cloud_collectors();

  void add_cloud_collector(const std::shared_ptr<CloudCollector<MsgTraits>> & collector);

private:
  struct Parameters
  {
    bool use_naive_approach;
    bool debug_mode;
    double rosbag_length;
    int maximum_queue_size;
    double timeout_sec;
    bool is_motion_compensated;
    bool publish_synchronized_pointcloud;
    bool keep_input_frame_in_synchronized_pointcloud;
    bool publish_previous_but_late_pointcloud;
    std::string synchronized_pointcloud_postfix;
    std::string input_twist_topic_type;
    std::vector<std::string> input_topics;
    std::string output_frame;
    std::string matching_strategy;
  } params_;

  double current_concatenate_cloud_timestamp_{0.0};
  double latest_concatenate_cloud_timestamp_{0.0};

  struct DiagnosticInfo
  {
    bool drop_previous_but_late_pointcloud{false};
    bool publish_pointcloud{false};
    bool is_concatenated_cloud_empty{false};
    std::shared_ptr<CollectorInfoBase> collector_info;
    std::unordered_map<std::string, double> topic_to_original_stamp_map;
    std::unordered_map<std::string, double> topic_to_pipeline_latency_map;
    double processing_time{0.0};
    double pipeline_latency{0.0};
  };

  std::shared_ptr<CombineCloudHandler<MsgTraits>> combine_cloud_handler_;
  std::list<std::shared_ptr<CloudCollector<MsgTraits>>> cloud_collectors_;
  std::unique_ptr<CollectorMatchingStrategy<MsgTraits>> collector_matching_strategy_;

  bool init_collector_list_{false};
  static constexpr const int num_of_collectors{3};

  // default postfix name for synchronized pointcloud
  static constexpr const char * default_sync_topic_postfix = "_synchronized";

  // subscribers
  std::vector<std::shared_ptr<SubscriberType>> pointcloud_subs_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // publishers
  std::shared_ptr<PublisherType> concatenated_cloud_publisher_;
  rclcpp::Publisher<autoware_sensing_msgs::msg::ConcatenatedPointCloudInfo>::SharedPtr
    concatenation_info_publisher_;
  std::unordered_map<std::string, std::shared_ptr<PublisherType>>
    topic_to_transformed_cloud_publisher_map_;
  std::unique_ptr<autoware_utils::DebugPublisher> debug_publisher_;

  std::unique_ptr<autoware_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_;

  std::unique_ptr<autoware_utils::DiagnosticsInterface> diagnostics_interface_;
  void publish_debug_message(
    const double processing_time, const double cyclic_time,
    const std::unordered_map<std::string, double> & topic_to_pipeline_latency_map);
  void check_concat_status(const DiagnosticInfo & diagnostic_info);

  void initialize_pub_sub();

  void cloud_callback(
    const typename PointCloudMessage::ConstSharedPtr & input_ptr, const std::string & topic_name);

  void twist_callback(const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr input);
  void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr input);

  std::string replace_sync_topic_name_postfix(
    const std::string & original_topic_name, const std::string & postfix);
  void initialize_collector_list();
  typename std::list<std::shared_ptr<CloudCollector<MsgTraits>>>::iterator
  find_and_reset_oldest_collector();
};

class PointCloudConcatenateDataSynchronizerComponent
: public PointCloudConcatenateDataSynchronizerComponentTemplated<PointCloud2Traits>
{
public:
  explicit PointCloudConcatenateDataSynchronizerComponent(const rclcpp::NodeOptions & node_options)
  : PointCloudConcatenateDataSynchronizerComponentTemplated<PointCloud2Traits>(node_options)
  {
  }
  ~PointCloudConcatenateDataSynchronizerComponent() override = default;
};

}  // namespace autoware::pointcloud_preprocessor

#include "autoware/pointcloud_preprocessor/concatenate_data/concatenate_and_time_sync_node.ipp"
