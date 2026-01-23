// Copyright 2021 TIER IV, Inc.
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

#include "autoware/lidar_centerpoint/node.hpp"

#include "autoware/lidar_centerpoint/centerpoint_config.hpp"
#include "autoware/lidar_centerpoint/preprocess/pointcloud_densification.hpp"
#include "autoware/lidar_centerpoint/ros_utils.hpp"
#include "autoware/lidar_centerpoint/utils.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pcl_ros/transforms.hpp>

#include <memory>
#include <string>
#include <vector>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

namespace autoware::lidar_centerpoint
{
LidarCenterPointNode::LidarCenterPointNode(const rclcpp::NodeOptions & node_options)
: Node("lidar_center_point", node_options), tf_buffer_(this->get_clock())
{
  const std::vector<double> score_thresholds_double =
    this->declare_parameter<std::vector<double>>("post_process_params.score_thresholds");
  const std::vector<float> score_thresholds(
    score_thresholds_double.begin(), score_thresholds_double.end());
  const float circle_nms_dist_threshold = static_cast<float>(
    this->declare_parameter<double>("post_process_params.circle_nms_dist_threshold"));
  const auto yaw_norm_thresholds =
    this->declare_parameter<std::vector<double>>("post_process_params.yaw_norm_thresholds");
  const std::size_t max_num_detections = static_cast<std::size_t>(
    this->declare_parameter<std::int64_t>("post_process_params.max_num_detections", 1000));
  const std::string densification_world_frame_id =
    this->declare_parameter<std::string>("densification_params.world_frame_id");
  const int densification_num_past_frames =
    this->declare_parameter<int>("densification_params.num_past_frames");
  const std::string trt_precision = this->declare_parameter<std::string>("trt_precision");
  const std::size_t cloud_capacity = this->declare_parameter<std::int64_t>("cloud_capacity");
  const std::string encoder_onnx_path = this->declare_parameter<std::string>("encoder_onnx_path");
  const std::string encoder_engine_path =
    this->declare_parameter<std::string>("encoder_engine_path");
  const std::string head_onnx_path = this->declare_parameter<std::string>("head_onnx_path");
  const std::string head_engine_path = this->declare_parameter<std::string>("head_engine_path");
  class_names_ = this->declare_parameter<std::vector<std::string>>("model_params.class_names");
  has_twist_ = this->declare_parameter<bool>("model_params.has_twist");
  const std::size_t point_feature_size = static_cast<std::size_t>(
    this->declare_parameter<std::int64_t>("model_params.point_feature_size"));
  has_variance_ = this->declare_parameter<bool>("model_params.has_variance");
  const std::size_t max_voxel_size =
    static_cast<std::size_t>(this->declare_parameter<std::int64_t>("model_params.max_voxel_size"));
  const auto point_cloud_range =
    this->declare_parameter<std::vector<double>>("model_params.point_cloud_range");
  const auto voxel_size = this->declare_parameter<std::vector<double>>("model_params.voxel_size");
  const std::size_t downsample_factor = static_cast<std::size_t>(
    this->declare_parameter<std::int64_t>("model_params.downsample_factor"));
  const std::size_t encoder_in_feature_size = static_cast<std::size_t>(
    this->declare_parameter<std::int64_t>("model_params.encoder_in_feature_size"));
  const auto allow_remapping_by_area_matrix =
    this->declare_parameter<std::vector<int64_t>>("allow_remapping_by_area_matrix");
  const auto min_area_matrix = this->declare_parameter<std::vector<double>>("min_area_matrix");
  const auto max_area_matrix = this->declare_parameter<std::vector<double>>("max_area_matrix");

  // Set up logger name
  this->logger_name_ = this->declare_parameter<std::string>("logger_name", "lidar_centerpoint");

  detection_class_remapper_.setParameters(
    allow_remapping_by_area_matrix, min_area_matrix, max_area_matrix);

  {
    NMSParams p;
    p.search_distance_2d_ =
      this->declare_parameter<double>("post_process_params.iou_nms_search_distance_2d");
    p.iou_threshold_ = this->declare_parameter<double>("post_process_params.iou_nms_threshold");
    iou_bev_nms_.setParameters(p);
  }

  TrtCommonConfig encoder_param(encoder_onnx_path, trt_precision, encoder_engine_path);
  TrtCommonConfig head_param(head_onnx_path, trt_precision, head_engine_path);
  DensificationParam densification_param(
    densification_world_frame_id, densification_num_past_frames, this->logger_name_);

  if (point_cloud_range.size() != 6) {
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger(this->logger_name_.c_str()),
      "The size of point_cloud_range != 6: use the default parameters.");
  }
  if (voxel_size.size() != 3) {
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger(this->logger_name_.c_str()),
      "The size of voxel_size != 3: use the default parameters.");
  }
  CenterPointConfig config(
    class_names_.size(), point_feature_size, cloud_capacity, max_voxel_size, point_cloud_range,
    voxel_size, downsample_factor, encoder_in_feature_size, score_thresholds,
    circle_nms_dist_threshold, yaw_norm_thresholds, has_variance_, max_num_detections,
    this->logger_name_);
  detector_ptr_ =
    std::make_unique<CenterPointTRT>(encoder_param, head_param, densification_param, config);
  diagnostics_centerpoint_trt_ =
    std::make_unique<autoware_utils::DiagnosticsInterface>(this, "centerpoint_trt");

  // diagnostics parameters
  max_allowed_processing_time_ms_ =
    declare_parameter<double>("diagnostics.max_allowed_processing_time_ms");
  max_acceptable_consecutive_delay_ms_ =
    declare_parameter<double>("diagnostics.max_acceptable_consecutive_delay_ms");

  pointcloud_sub_ =
    std::make_unique<cuda_blackboard::CudaBlackboardSubscriber<cuda_blackboard::CudaPointCloud2>>(
      *this, "~/input/pointcloud",
      std::bind(&LidarCenterPointNode::pointCloudCallback, this, std::placeholders::_1));
  objects_pub_ = this->create_publisher<autoware_perception_msgs::msg::DetectedObjects>(
    "~/output/objects", rclcpp::QoS{1});
  voxel_grid_pub_ =
    this->create_publisher<nav_msgs::msg::OccupancyGrid>("~/debug/voxel_grid", rclcpp::QoS{1});

  // initialize debug tool
  {
    using autoware_utils::DebugPublisher;
    using autoware_utils::StopWatch;
    stop_watch_ptr_ = std::make_unique<StopWatch<std::chrono::milliseconds>>();
    debug_publisher_ptr_ = std::make_unique<DebugPublisher>(this, this->logger_name_.c_str());
    stop_watch_ptr_->tic("cyclic_time");
    stop_watch_ptr_->tic("processing_time");
  }

  if (stop_watch_ptr_) {
    // processing time diagnostics
    const double validation_callback_interval_ms =
      declare_parameter<double>("diagnostics.validation_callback_interval_ms");

    diagnostic_processing_time_updater_.setHardwareID(this->get_name());
    diagnostic_processing_time_updater_.add(
      "processing_time_status", this, &LidarCenterPointNode::diagnoseProcessingTime);
    // msec -> sec
    diagnostic_processing_time_updater_.setPeriod(validation_callback_interval_ms / 1e3);
  }

  if (this->declare_parameter("build_only", false)) {
    RCLCPP_INFO(this->get_logger(), "TensorRT engine is built and shutdown node.");
    rclcpp::shutdown();
  }
  published_time_publisher_ = std::make_unique<autoware_utils::PublishedTimePublisher>(this);
}

void LidarCenterPointNode::pointCloudCallback(
  const std::shared_ptr<const cuda_blackboard::CudaPointCloud2> & input_pointcloud_msg)
{
  const auto objects_sub_count =
    objects_pub_->get_subscription_count() + objects_pub_->get_intra_process_subscription_count();
  if (objects_sub_count < 1) {
    return;
  }

  if (stop_watch_ptr_) {
    stop_watch_ptr_->toc("processing_time", true);
  }
  diagnostics_centerpoint_trt_->clear();

  std::vector<Box3D> det_boxes3d;
  bool is_num_pillars_within_range = true;
  bool is_success = detector_ptr_->detect(
    input_pointcloud_msg, tf_buffer_, det_boxes3d, is_num_pillars_within_range);
  if (!is_success) {
    return;
  }
  diagnostics_centerpoint_trt_->add_key_value(
    "is_num_pillars_within_range", is_num_pillars_within_range);
  if (!is_num_pillars_within_range) {
    std::stringstream message;
    message << "CenterPointTRT::detect: The actual number of pillars exceeds its maximum value, "
            << "which may limit the detection performance.";
    diagnostics_centerpoint_trt_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
  }

  // Publish voxel grid for debug visualization
  if (voxel_grid_pub_->get_subscription_count() > 0) {
    std::vector<int> coordinates;
    std::vector<float> point_counts;
    std::vector<float> voxel_heights;
    std::vector<float> voxel_mean_z;
    unsigned int num_voxels = 0;

    // Visualization mode selector
    enum class VoxelVisualizationMode { OCCUPANCY, HEIGHT, FEAT_MEAN_Z };
    const VoxelVisualizationMode viz_mode = VoxelVisualizationMode::FEAT_MEAN_Z;

    bool data_available = false;
    if (viz_mode == VoxelVisualizationMode::FEAT_MEAN_Z) {
      data_available = detector_ptr_->getVoxelGridData(
        coordinates, point_counts, voxel_heights, voxel_mean_z, num_voxels);
    } else if (viz_mode == VoxelVisualizationMode::HEIGHT) {
      data_available =
        detector_ptr_->getVoxelGridData(coordinates, point_counts, voxel_heights, num_voxels);
    } else {
      data_available = detector_ptr_->getVoxelGridData(coordinates, point_counts, num_voxels);
    }

    if (data_available) {
      // Get voxel config from detector
      const auto & config = detector_ptr_->getConfig();
      const float voxel_size_x = config.voxel_size_x_;
      const float voxel_size_y = config.voxel_size_y_;
      const float range_min_x = config.range_min_x_;
      const float range_min_y = config.range_min_y_;
      const float viz_min_x = -40.0f;
      const float viz_max_x = 40.0f;
      const float viz_min_y = -40.0f;
      const float viz_max_y = 40.0f;
      const float max_point_in_voxel = static_cast<float>(config.max_point_in_voxel_size_);

      // Voxel coordinate system:
      // - Voxel (0,0) left edge is at world position (range_min_x, range_min_y)
      // - Voxel (i,j) left edge is at (range_min_x + i*voxel_size, range_min_y + j*voxel_size)
      // - Voxel (i,j) center is at (range_min_x + (i+0.5)*voxel_size, range_min_y +
      // (j+0.5)*voxel_size)

      // Calculate which voxel index corresponds to the visualization area
      const int grid_offset_x =
        static_cast<int>(std::floor((viz_min_x - range_min_x) / voxel_size_x));
      const int grid_offset_y =
        static_cast<int>(std::floor((viz_min_y - range_min_y) / voxel_size_y));

      // OccupancyGrid origin: In ROS, this is typically at the LOWER-LEFT corner of cell [0,0]
      // which aligns with the LEFT EDGE of the voxel (not center)
      const float grid_origin_x = range_min_x + grid_offset_x * voxel_size_x;
      const float grid_origin_y = range_min_y + grid_offset_y * voxel_size_y;

      // Debug: Calculate world position of first few voxels
      const float voxel_155_center_x = range_min_x + (155 + 0.5f) * voxel_size_x;
      const float voxel_155_center_y = range_min_y + (155 + 0.5f) * voxel_size_y;
      RCLCPP_INFO_ONCE(
        rclcpp::get_logger("lidar_centerpoint"),
        "Voxel 155 center: (%.2f, %.2f), Grid origin: (%.2f, %.2f)", voxel_155_center_x,
        voxel_155_center_y, grid_origin_x, grid_origin_y);

      // Calculate grid dimensions
      const int grid_width = static_cast<int>((viz_max_x - viz_min_x) / voxel_size_x);
      const int grid_height = static_cast<int>((viz_max_y - viz_min_y) / voxel_size_y);

      RCLCPP_INFO_ONCE(
        rclcpp::get_logger("lidar_centerpoint"),
        "Voxel grid debug - offset: (%d, %d), origin: (%.2f, %.2f), size: (%d, %d)", grid_offset_x,
        grid_offset_y, grid_origin_x, grid_origin_y, grid_width, grid_height);

      // Create occupancy grid message
      nav_msgs::msg::OccupancyGrid grid_msg;
      grid_msg.header = input_pointcloud_msg->header;

      // Transform grid to map frame
      const std::string target_frame = "map";
      try {
        // Get transform from ego frame to map frame at the pointcloud timestamp
        geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_.lookupTransform(
          target_frame, input_pointcloud_msg->header.frame_id, input_pointcloud_msg->header.stamp,
          rclcpp::Duration::from_seconds(0.5));

        // Transform the grid origin position from ego frame to map frame
        geometry_msgs::msg::PoseStamped origin_ego;
        origin_ego.header = input_pointcloud_msg->header;
        origin_ego.pose.position.x = grid_origin_x;
        origin_ego.pose.position.y = grid_origin_y;
        origin_ego.pose.position.z = 0.0;
        origin_ego.pose.orientation.w = 1.0;

        geometry_msgs::msg::PoseStamped origin_map;
        tf2::doTransform(origin_ego, origin_map, transform_stamped);

        // Set header to map frame
        grid_msg.header.frame_id = target_frame;

        // Set transformed origin
        grid_msg.info.resolution = voxel_size_x;
        grid_msg.info.width = grid_width;
        grid_msg.info.height = grid_height;
        grid_msg.info.origin.position.x = origin_map.pose.position.x;
        grid_msg.info.origin.position.y = origin_map.pose.position.y;
        grid_msg.info.origin.position.z = origin_map.pose.position.z;
        grid_msg.info.origin.orientation = origin_map.pose.orientation;
      } catch (tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
          rclcpp::get_logger("lidar_centerpoint"), *this->get_clock(), 1000,
          "Could not transform grid to map frame: %s. Publishing in ego frame.", ex.what());

        // Fallback to ego frame
        grid_msg.info.resolution = voxel_size_x;
        grid_msg.info.width = grid_width;
        grid_msg.info.height = grid_height;
        grid_msg.info.origin.position.x = grid_origin_x;
        grid_msg.info.origin.position.y = grid_origin_y;
        grid_msg.info.origin.position.z = 0.0;
        grid_msg.info.origin.orientation.w = 1.0;
      }

      // Initialize grid with unknown (-1)
      grid_msg.data.resize(grid_width * grid_height, -1);

      // Fill grid with voxel data
      int num_filled = 0;
      int min_coord_x = 999999, max_coord_x = -999999;
      int min_coord_y = 999999, max_coord_y = -999999;

      for (unsigned int i = 0; i < num_voxels; ++i) {
        const int coord_x = coordinates[i * 3 + 2];
        const int coord_y = coordinates[i * 3 + 1];

        // Track coordinate range
        min_coord_x = std::min(min_coord_x, coord_x);
        max_coord_x = std::max(max_coord_x, coord_x);
        min_coord_y = std::min(min_coord_y, coord_y);
        max_coord_y = std::max(max_coord_y, coord_y);

        // Convert voxel coordinates directly to grid indices
        const int grid_x = coord_x - grid_offset_x;
        const int grid_y = coord_y - grid_offset_y;

        // Check if within grid bounds
        if (grid_x >= 0 && grid_x < grid_width && grid_y >= 0 && grid_y < grid_height) {
          const int grid_idx = grid_y * grid_width + grid_x;

          float value;
          switch (viz_mode) {
            case VoxelVisualizationMode::FEAT_MEAN_Z:
              // Map mean.z to value: 1m -> 0.0, 0m -> 0.8, empty voxel -> 1.0
              {
                const float mean_z = voxel_mean_z[i];
                const float max_z = 1.0f;
                const float min_value = 0.0f;
                const float max_value = 0.8f;
                // Linear mapping: value = 0.8 - 0.8 * (mean_z / 1.0)
                value = max_value - (mean_z / max_z) * (max_value - min_value);
                value = std::max(min_value, std::min(max_value, value));
              }
              break;

            case VoxelVisualizationMode::HEIGHT:
              // Map height to value: 2m max -> 0.0, 0m -> 0.8
              {
                const float height = voxel_heights[i];
                const float max_height = 2.0f;
                const float min_value = 0.0f;
                const float max_value = 0.8f;
                value = max_value - (height / max_height) * (max_value - min_value);
                value = std::max(min_value, std::min(max_value, value));
              }
              break;

            case VoxelVisualizationMode::OCCUPANCY:
            default:
              // Map point count to occupancy value (inverted scale)
              // 0.0 = fully filled (32 points), 0.8 = lowest filled (1 point), 1.0 = empty
              {
                const float normalized_count = point_counts[i] / max_point_in_voxel;
                value = std::max(0.8f - 0.8f * normalized_count, 0.0f);
              }
              break;
          }

          // Convert to occupancy grid value (0-100)
          grid_msg.data[grid_idx] = static_cast<int8_t>(value * 100);
          num_filled++;
        }
      }

      RCLCPP_INFO_ONCE(
        rclcpp::get_logger("lidar_centerpoint"),
        "Voxels: total=%u, filled_in_grid=%d, coord_x=[%d,%d], coord_y=[%d,%d]", num_voxels,
        num_filled, min_coord_x, max_coord_x, min_coord_y, max_coord_y);

      // Set empty cells (still -1) to 100 (fully empty)
      for (auto & cell : grid_msg.data) {
        if (cell == -1) {
          cell = 100;
        }
      }

      voxel_grid_pub_->publish(grid_msg);
    }
  }

  std::vector<autoware_perception_msgs::msg::DetectedObject> raw_objects;
  raw_objects.reserve(det_boxes3d.size());
  for (const auto & box3d : det_boxes3d) {
    autoware_perception_msgs::msg::DetectedObject obj;
    box3DToDetectedObject(box3d, class_names_, has_twist_, has_variance_, obj);
    raw_objects.emplace_back(obj);
  }

  autoware_perception_msgs::msg::DetectedObjects output_msg;
  output_msg.header = input_pointcloud_msg->header;
  output_msg.objects = iou_bev_nms_.apply(raw_objects);

  detection_class_remapper_.mapClasses(output_msg);

  if (objects_sub_count > 0) {
    objects_pub_->publish(output_msg);
    published_time_publisher_->publish_if_subscribed(objects_pub_, output_msg.header.stamp);
  }
  diagnostics_centerpoint_trt_->publish(input_pointcloud_msg->header.stamp);

  // add processing time for debug
  if (debug_publisher_ptr_ && stop_watch_ptr_) {
    const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);
    const double processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
    const double pipeline_latency_ms =
      std::chrono::duration<double, std::milli>(
        std::chrono::nanoseconds(
          (this->get_clock()->now() - output_msg.header.stamp).nanoseconds()))
        .count();
    debug_publisher_ptr_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/cyclic_time_ms", cyclic_time_ms);
    debug_publisher_ptr_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time_ms", processing_time_ms);
    debug_publisher_ptr_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/pipeline_latency_ms", pipeline_latency_ms);

    last_processing_time_ms_ = processing_time_ms;
  }
}

// Check the processing time and delayed timestamp
// If the node is consistently delayed, publish an error diagnostic message
void LidarCenterPointNode::diagnoseProcessingTime(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  const rclcpp::Time timestamp_now = this->get_clock()->now();
  diagnostic_msgs::msg::DiagnosticStatus::_level_type diag_level =
    diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::stringstream message{"OK"};

  // Check if the node has performed inference
  if (last_processing_time_ms_) {
    // check processing time is acceptable
    if (last_processing_time_ms_ > max_allowed_processing_time_ms_) {
      stat.add("is_processing_time_ms_in_expected_range", false);

      message.clear();
      message << "Processing time exceeds the acceptable limit of "
              << max_allowed_processing_time_ms_ << " ms by "
              << (last_processing_time_ms_.value() - max_allowed_processing_time_ms_) << " ms.";

      // in case the processing starts with a delayed inference
      if (!last_in_time_processing_timestamp_) {
        last_in_time_processing_timestamp_ = timestamp_now;
      }

      diag_level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    } else {
      stat.add("is_processing_time_ms_in_expected_range", true);
      last_in_time_processing_timestamp_ = timestamp_now;
    }
    stat.add("processing_time_ms", last_processing_time_ms_.value());

    const double delayed_state_duration =
      std::chrono::duration<double, std::milli>(
        std::chrono::nanoseconds(
          (timestamp_now - last_in_time_processing_timestamp_.value()).nanoseconds()))
        .count();

    // check consecutive delays
    if (delayed_state_duration > max_acceptable_consecutive_delay_ms_) {
      stat.add("is_consecutive_processing_delay_in_range", false);

      message << " Processing delay has consecutively exceeded the acceptable limit continuously.";

      diag_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    } else {
      stat.add("is_consecutive_processing_delay_in_range", true);
    }
    stat.add("consecutive_processing_delay_ms", delayed_state_duration);
  } else {
    stat.add("is_processing_time_ms_in_expected_range", true);
    stat.add("processing_time_ms", 0.0);
    stat.add("is_consecutive_processing_delay_in_range", true);
    stat.add("consecutive_processing_delay_ms", 0.0);

    message << "Waiting for the node to perform inference.";
  }

  stat.summary(diag_level, message.str());
}

}  // namespace autoware::lidar_centerpoint

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::lidar_centerpoint::LidarCenterPointNode)
