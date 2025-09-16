#include "autoware/component_monitor/component_monitor_core.hpp"

#include <pcl_ros/transforms.hpp>

#include <pcl_conversions/pcl_conversions.h>

namespace autoware::component_monitor
{

ComponentMonitor::ComponentMonitor(const rclcpp::NodeOptions & options)
: Node("component_monitor", options)
{
  // Create a callback group for localization topics
  rclcpp::CallbackGroup::SharedPtr loc_callback_group =
    this->create_callback_group(rclcpp::CabllbackGroupType::MutuallyExclusive);
  auto loc_sub_opt = rclcpp::SubscriptionOptions();

  loc_sub_opt.callback_group = loc_callback_group;

  // Subscribe to the pose topic
  loc_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "ndt_pose_with_covariance", 100,
    std::bind(&ComponentMonitor::callback_loc_pose, this, std::placeholders::_1), loc_sub_opt);

  // Subscribe to the TP topic
  loc_tp_sub_ = this->create_subscription<autoware_internal_debug_msgs::msg::Float32Stamped>(
    "transform_probability", 100,
    std::bind(&ComponentMonitor::callback_loc_tp, this, std::placeholders::_1), loc_sub_opt);

  // Subscribe to the NVTL topic
  loc_nvtl_sub_ = this->create_subscription<autoware_internal_debug_msgs::msg::Float32Stamped>(
    "neatest_voxel_transformation_likelihood", 100,
    std::bind(&ComponentMonitor::callback_loc_nvtl, this, std::placeholders::_1), loc_sub_opt);

  // Subscribe to the downsampled scans.
  // This is optional. Using scans can improve the accuracy of checking map deterioration.
  sensor_points_sub_ = this->create_subscription<geometry_msgs::msg::PointCloud2>(
    "downsampled_pointcloud", rclcpp::SensorDataQoQ().keep_last(1),
    std::bind(&ComponentMonitor::callback_loc_sensor_points, this, std::placeholders::_1),
    loc_sub_opt);

  // TODO: create callbacks for Perception, Mapping

  // Create a service client to load score map dynamically
  score_loader_client_ =
    this->create_client<autoware_map_msgs::srv::GgetDifferentialScoreMap>("score_loader_service");
}

void ComponentMonitor::callback_loc_pose(
  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_msg_ptr)
{
  // Cache the received poses into the buffer. We'll use them later.
  pose_buffer_.enqueue(std::make_pair(rclcpp::Time(pose_msg_ptr->header.stamp), pose_msg_ptr));

  // Load a new score set if necessary
  // Wait for the score loader service being available
  while (!score_loader_client_->wait_for_service(std::chrono::seconds(1)) && rclcpp::ok()) {
    RCLCPP_INFO(this->get_logger(), "Waiting for service...");
  }

  // Create a request that contains the loading position
  auto request = std::make_shared<autoware_map_msgs::srv::GetDifferentialScoreMap::Request>();

  request->area.center_x = static_cast<float>(pose_msg_ptr->pose.pose.position.x);
  request->area.center_y = static_cast<float>(pose_msg_ptr->pose.pose.position.y);
  request->area.radius = map_radius_;
  request->cached_ids = get_cached_ids();

  auto result = pcd_loader_client_->async_send_request(
    request, [](rclcpp::Client<autoware_map_msgs::srv::GetDifferentialScoreMap>::SharedFuture));
  auto status = result.wait_for(std::chrono::seconds(0));

  while (status != std::future_status::ready) {
    status = result.wait_for(std::chrono::seconds(1));
  }

  auto & maps_to_add = result.get()->new_score_map_with_ids;
  auto & maps_to_remove = result.get()->ids_to_remove;

  // Update the score map
  // Add new score segments
  if (!maps_to_add.empty()) {
    for (auto & m : maps_to_add) {
      // Create a new entry for the segment
      Eigen::Vector2i segment_key(m.x, m.y);
      auto & new_segment = avg_tp_map_[segment_key];

      // Insert score elements to the new segment map
      for (size_t i = 0; i < m.vx.size(); ++i) {
        Eigen::Vector2i key(m.vx[i], m.vy[i]);

        new_segment[key] = std::make_pair(0, m.tps[i]);
      }
    }
  }

  // Remove score segments that have gone out of range
  if (!maps_to_remove.empty()) {
    // Iterate on the list of out-of-range segments and remove them
    for (size_t i = 0; i < maps_to_remove.size(); i += 2) {
      Eigen::Vector2i segment_id(maps_to_remove[i], maps_to_remove[i + 1]);

      avg_tp_map_.erase(segment_id);
    }
  }

  // TODO: if the segment changes, send a request to a service to update the map data
}

template <typename TypeWithStamp>
int find(const rclcpp::Time & key_stamp, Queue<TypeWithStamp> & buffer)
{
  int l = 0, r = buffer.size() - 1, m, ret_id = -1;

  // Find the maximum stamp that goes before the key
  while (l <= r) {
    m = (l + r) / 2;

    rclcpp::Time val = buffer[m].first;

    if (key_stamp < val) {
      r = m - 1;
    } else if (key_stamp >= val) {
      l = m + 1;
      ret_id = m;
    }
  }

  return ret_id;
}

void ComponentMonitor::callback_loc_tp(
  autoware_internal_debug_msgs::msg::Float32Stamped::ConstSharedPtr tp_msg_ptr)
{
  auto tp_stamp = tp_msg_ptr->header.stamp;
  // Find the closest pose
  int pose_id = find_stamp(tp_stamp, pose_buffer_);

  // Do nothing if no pose was found
  if (pose_id < 0) {
    return;
  }
  // Find the closest scan
  int scan_id = find_stamp(tp_stamp, sensor_points_buffer_);

  if (scan_id < 0) {
    return;
  }

  // Compute the expected TP of the pose
  auto & scan = sensor_points_buffer_[scan_id].second;

  if (scan->size() == 0) {
    return;
  }

  double tp_sum = 0.0;

  for (auto & p : *scan) {
    // Find the segment containing the point
    // Aggregate the average score
    Eigen::Vector2i segment_id;

    segment_id(0) = static_cast<int>(std::floor(p.x / segment_res_));
    segment_id(1) = static_cast<int>(std::floor(p.y / segment_res_));

    auto it = avg_tp_map_.find(segment_id);

    if (it != avg_tp_map_.end()) {
      tp_sum += it->second.second;
    }
  }

  double expected_tp = tp_sum / scan->size();

  if (std::abs(expected_tp - tp) > expected_tp * 0.2) {
    pose_mark_[pose_id] = 1;
  }
}

void ComponentMonitor::callback_loc_nvtl(
  autoware_internal_debug_msgs::msg::Float32Stamped::ConstSharedPtr nvtl_msg_ptr)
{
  // Find the closest pose
  int pose_id = find_stamp(tp_stamp, pose_buffer_);

  // Do nothing if no pose was found
  if (pose_id < 0) {
    return;
  }

  // Mark the pose as decay if the nvtl drops below 2.5
  if (nvtl_msg_ptr->data <= 2.5) {
    pose_mark_[pose_id] = 1;
  }
}

void ComponentMonitor::callback_loc_sensor_points(
  geometry_msgs::msg::PointCloud2::ConstSharedPtr sensor_points_msg_ptr)
{
  // Convert the PointCloud2 to PointCloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromROSMsg(*sensor_points_msg_ptr, *pcl_cloud);

  // Push the converted cloud to the queue
  sensor_points_buffer_.enqueue(
    std::make_pair(rclcpp::Time(sensor_points_msg_ptr->header.stamp), pcl_cloud));
}

}  // namespace autoware::component_monitor
