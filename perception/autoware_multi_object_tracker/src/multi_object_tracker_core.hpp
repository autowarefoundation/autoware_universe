#ifndef MULTI_OBJECT_TRACKER_CORE_HPP_
#define MULTI_OBJECT_TRACKER_CORE_HPP_

#include "autoware/multi_object_tracker/object_model/types.hpp"
#include "autoware/multi_object_tracker/association/association.hpp"
#include "autoware/multi_object_tracker/odometry.hpp"
#include "processor/input_manager.hpp"
#include "processor/processor.hpp"
#include "debugger/debugger.hpp"
#include <autoware_utils_debug/time_keeper.hpp>
#include <rclcpp/rclcpp.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <memory>
#include <vector>
#include <string>
#include <optional>

namespace autoware::multi_object_tracker
{

struct MultiObjectTrackerParameters
{
  double publish_rate;
  std::string world_frame_id;
  std::string ego_frame_id;
  bool enable_delay_compensation;
  bool enable_odometry_uncertainty;
  bool publish_processing_time_detail;
  bool publish_merged_objects;
  
  std::vector<types::InputChannel> input_channels_config;
  TrackerProcessorConfig processor_config;
  AssociatorConfig associator_config;
};

struct MultiObjectTrackerInternalState
{
  std::unique_ptr<TrackerProcessor> processor;
  std::unique_ptr<InputManager> input_manager;
  std::shared_ptr<Odometry> odometry;
  
  rclcpp::Time last_published_time;
  rclcpp::Time last_updated_time;
  
  MultiObjectTrackerInternalState();
};

namespace core
{

struct PublishData
{
  autoware_perception_msgs::msg::TrackedObjects tracked_objects;
  std::optional<autoware_perception_msgs::msg::DetectedObjects> merged_objects;
  std::optional<autoware_perception_msgs::msg::TrackedObjects> tentative_objects;
};

void process_objects(
  const types::DynamicObjectList & objects,
  const rclcpp::Time & current_time,
  const MultiObjectTrackerParameters & params,
  MultiObjectTrackerInternalState & state,
  TrackerDebugger & debugger,
  const rclcpp::Logger & logger,
  const std::shared_ptr<autoware_utils_debug::TimeKeeper> & time_keeper);

bool should_publish(
  const rclcpp::Time & current_time,
  const MultiObjectTrackerParameters & params,
  MultiObjectTrackerInternalState & state);

PublishData get_output(
  const rclcpp::Time & publish_time,
  const rclcpp::Time & current_time,
  const std::optional<geometry_msgs::msg::Transform> & tf_base_to_world,
  const MultiObjectTrackerParameters & params,
  MultiObjectTrackerInternalState & state,
  TrackerDebugger & debugger,
  const rclcpp::Logger & logger,
  const std::shared_ptr<autoware_utils_debug::TimeKeeper> & time_keeper);

void prune_objects(
  const rclcpp::Time & time,
  MultiObjectTrackerInternalState & state);

}  // namespace core

}  // namespace autoware::multi_object_tracker

#endif  // MULTI_OBJECT_TRACKER_CORE_HPP_
