#include "multi_object_tracker_core.hpp"

#include <autoware_utils_debug/time_keeper.hpp>

#include <unordered_map>
#include <memory>

namespace autoware::multi_object_tracker
{

using autoware_utils_debug::ScopedTimeTrack;

MultiObjectTrackerInternalState::MultiObjectTrackerInternalState()
: last_published_time(0, 0, RCL_ROS_TIME), last_updated_time(0, 0, RCL_ROS_TIME)
{
}

namespace core
{

void process_objects(
  const types::DynamicObjectList & objects,
  const rclcpp::Time & current_time,
  [[maybe_unused]] const MultiObjectTrackerParameters & params,
  MultiObjectTrackerInternalState & state,
  TrackerDebugger & debugger,
  const rclcpp::Logger & logger,
  const std::shared_ptr<autoware_utils_debug::TimeKeeper> & time_keeper)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper);

  // Get the time of the measurement
  const rclcpp::Time measurement_time =
    rclcpp::Time(objects.header.stamp, current_time.get_clock_type());

  std::optional<geometry_msgs::msg::Pose> ego_pose;
  if (const auto odometry_info = state.odometry->getOdometryFromTf(measurement_time)) {
    ego_pose = odometry_info->pose.pose;
  }

  if (!ego_pose) {
    RCLCPP_WARN(
      logger, "No odometry information available at the measurement time: %.9f",
      measurement_time.seconds());
  }

  /* predict trackers to the measurement time */
  state.processor->predict(measurement_time, ego_pose);

  /* object association */
  std::unordered_map<int, int> direct_assignment, reverse_assignment;
  state.processor->associate(objects, direct_assignment, reverse_assignment);

  // Collect debug information - tracker list, existence probabilities, association results
  debugger.collectObjectInfo(
    measurement_time, state.processor->getListTracker(), objects, direct_assignment,
    reverse_assignment);

  /* tracker update */
  state.processor->update(objects, direct_assignment);

  /* tracker pruning */
  state.processor->prune(measurement_time);

  /* spawn new tracker */
  state.processor->spawn(objects, reverse_assignment);
  
  state.last_updated_time = current_time;
}

bool should_publish(
  const rclcpp::Time & current_time,
  const MultiObjectTrackerParameters & params,
  MultiObjectTrackerInternalState & state)
{
  if (state.last_updated_time.nanoseconds() == 0) {
    state.last_updated_time = current_time;
  }
  
  // ensure minimum interval: room for the next process(prediction)
  static constexpr double minimum_publish_interval_ratio = 0.85;
  static constexpr double maximum_publish_interval_ratio = 1.05;
  
  const double publisher_period = 1.0 / params.publish_rate;
  const double minimum_publish_interval = publisher_period * minimum_publish_interval_ratio;
  const auto elapsed_time = (current_time - state.last_published_time).seconds();
  
  if (elapsed_time < minimum_publish_interval) {
    return false;
  }

  // if there was update after publishing, publish new messages
  bool should_publish = state.last_published_time < state.last_updated_time;

  // if there was no update, publish if the elapsed time is longer than the maximum publish latency
  // in this case, it will perform extrapolate/remove old objects
  const double maximum_publish_interval = publisher_period * maximum_publish_interval_ratio;
  should_publish = should_publish || elapsed_time > maximum_publish_interval;

  return should_publish;
}

void prune_objects(
  const rclcpp::Time & time,
  MultiObjectTrackerInternalState & state)
{
    state.processor->prune(time);
}

PublishData get_output(
  const rclcpp::Time & publish_time,
  const rclcpp::Time & current_time,
  const std::optional<geometry_msgs::msg::Transform> & tf_base_to_world,
  const MultiObjectTrackerParameters & params,
  MultiObjectTrackerInternalState & state,
  TrackerDebugger & debugger,
  const rclcpp::Logger & logger,
  const std::shared_ptr<autoware_utils_debug::TimeKeeper> & time_keeper)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper);

  debugger.startPublishTime(current_time);

  PublishData output;

  // Create output msg
  output.tracked_objects.header.frame_id = params.world_frame_id;
  const rclcpp::Time object_time = params.enable_delay_compensation ? current_time : publish_time;
  state.processor->getTrackedObjects(object_time, output.tracked_objects);

  if (params.publish_merged_objects) {
    if (tf_base_to_world) {
      autoware_perception_msgs::msg::DetectedObjects merged_output_msg;
      state.processor->getMergedObjects(object_time, *tf_base_to_world, merged_output_msg);
      merged_output_msg.header.frame_id = params.ego_frame_id;
      output.merged_objects = merged_output_msg;
    } else {
      RCLCPP_WARN(
        logger, "No odometry information available at the publishing time: %.9f",
        publish_time.seconds());
    }
  }

  // Debug messages handling
  {
    std::unique_ptr<ScopedTimeTrack> st_debug_ptr;
    if (time_keeper)
      st_debug_ptr = std::make_unique<ScopedTimeTrack>("debug_publish", *time_keeper);
      
    debugger.endPublishTime(current_time, publish_time);

    // Update the diagnostic values
    const double min_extrapolation_time = (publish_time - state.last_updated_time).seconds();
    debugger.updateDiagnosticValues(min_extrapolation_time, output.tracked_objects.objects.size());

    if (debugger.shouldPublishTentativeObjects()) {
      autoware_perception_msgs::msg::TrackedObjects tentative_output_msg;
      tentative_output_msg.header.frame_id = params.world_frame_id;
      state.processor->getTentativeObjects(object_time, tentative_output_msg);
      output.tentative_objects = tentative_output_msg;
    }
  }
  
  state.last_published_time = current_time;
  
  return output;
}

}  // namespace core
}  // namespace autoware::multi_object_tracker
