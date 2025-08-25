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

#include "node.hpp"

#include <autoware/tensorrt_common/tensorrt_common.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <rclcpp/rclcpp.hpp>
#include <torch/torch.h>
#include <ATen/ATen.h>
#include <ATen/cuda/CUDAContext.h>
#include <Eigen/Core>

#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <mutex>
#include <thread>

namespace autoware::localized_position_validator
{
using autoware::tensorrt_common::TrtCommonConfig;

void color_input_points(const std::vector<Eigen::Vector4f> & input_points,
  pcl::PointCloud<pcl::PointXYZRGB> & colored_points, const float & ego_x, const float & ego_y,
  const float & ego_z)
{
  for (const auto & pt : input_points) {
    pcl::PointXYZRGB point;
    point.x = pt.x()+ego_x;
    point.y = pt.y()+ego_y;
    point.z = pt.z()+ego_z;

    if (pt.w() < 0.5){
      // map: white
      point.r = 255;
      point.g = 255;
      point.b = 255;
    } else {
      // scan: green
      point.r = 10;
      point.g = 255;
      point.b = 10;
    }

    colored_points.points.push_back(point);
  }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr downsample_voxel_grid(
  const sensor_msgs::msg::PointCloud2 & input_cloud_msg,
  float voxel_size)
{
  // convert from ROS2 message to PCL format
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(input_cloud_msg, *input_cloud);

  // downsample
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setInputCloud(input_cloud);
  voxel_filter.setLeafSize(voxel_size, voxel_size, voxel_size);

  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  voxel_filter.filter(*downsampled_cloud);

  return downsampled_cloud;
}

/**
 * @brief Extract the points within the range, translate the points to the ego-centric position,
 * and add the label information.
 */
std::vector<Eigen::Vector4f> preprocess_input_points(
  const pcl::PointCloud<pcl::PointXYZ> & cloud,
  const float & x_min, const float & x_max,
  const float & y_min, const float & y_max,
  const float & z_min, const float & z_max,
  const Eigen::Vector3f & vehicle_pos,
  const float & label)
{
  std::vector<Eigen::Vector4f> extracted_labeled_points_from_vehicle;
  extracted_labeled_points_from_vehicle.reserve(cloud.points.size());

  for (const auto & pt : cloud.points) {
    const float x = pt.x-vehicle_pos.x();
    const float y = pt.y-vehicle_pos.y();
    const float z = pt.z-vehicle_pos.z();

    // might better to check...
    //if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;

    if (x_min < x && x < x_max && y_min < y && y < y_max && z_min < z && z < z_max) {
      // create points with label
      extracted_labeled_points_from_vehicle.emplace_back(x, y, z, label);
    }
  }

  return extracted_labeled_points_from_vehicle;
}

/**
 * @brief Extract the points within the range, translate the points to the ego-centric position,
 * and add the label information.
 */
std::vector<Eigen::Vector4f> preprocess_input_points(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const float & x_min, const float & x_max,
  const float & y_min, const float & y_max,
  const float & z_min, const float & z_max,
  const Eigen::Vector3f & vehicle_pos,
  const float & label)
{
  std::vector<Eigen::Vector4f> extracted_labeled_points_from_vehicle;

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    const float x = *iter_x-vehicle_pos.x();
    const float y = *iter_y-vehicle_pos.y();
    const float z = *iter_z-vehicle_pos.z();

    // might better to check...
    //if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;

    if (x_min < x && x < x_max && y_min < y && y < y_max && z_min < z && z < z_max) {
      // create points with label
      extracted_labeled_points_from_vehicle.emplace_back(x, y, z, label);
    }
  }

  return extracted_labeled_points_from_vehicle;
}

/**
 * @brief Concatenates two sets of labeled points
 */
std::vector<Eigen::Vector4f> concatenate_points(
  const std::vector<Eigen::Vector4f> & a, const std::vector<Eigen::Vector4f> & b)
{
  std::vector<Eigen::Vector4f> result;
  result.reserve(a.size() + b.size());
  result.insert(result.end(), a.begin(), a.end());
  result.insert(result.end(), b.begin(), b.end());

  return result;
}

LocalizedPositionValidatorNode::LocalizedPositionValidatorNode(
  const rclcpp::NodeOptions & node_options) : Node("localized_position_validator", node_options)
{
  // node parameters
  partial_load_ = declare_parameter<bool>("map_partial_load");
  map_fetch_range_ = declare_parameter<float>("map_fetch_range");
  const float map_refetch_threshold = declare_parameter<float>("map_refetch_threshold");

  float prepro_voxel_size_xy = declare_parameter<float>("preprocess_params.voxel_size_xy");
  float prepro_voxel_size_z = declare_parameter<float>("preprocess_params.voxel_size_z");
  pcd_min_x_range_ = declare_parameter<float>("preprocess_params.pcd_x_range_min");
  pcd_max_x_range_ = declare_parameter<float>("preprocess_params.pcd_x_range_max");
  pcd_min_y_range_ = declare_parameter<float>("preprocess_params.pcd_y_range_min");
  pcd_max_y_range_ = declare_parameter<float>("preprocess_params.pcd_y_range_max");
  pcd_min_z_range_ = declare_parameter<float>("preprocess_params.pcd_z_range_min");
  pcd_max_z_range_ = declare_parameter<float>("preprocess_params.pcd_z_range_max");
  map_downsample_res_ = declare_parameter<float>("preprocess_params.map_downsample_resolution");

  max_time_diff_ = std::chrono::milliseconds(
    declare_parameter<int>("input_topic_delay_tolerance_ms"));
  use_soft_bayesian_update_ = declare_parameter<bool>("use_soft_bayesian_update");

  publish_input_points_ = declare_parameter<bool>("debug.publish_input_points");

  // model parameters
  const std::string model_onnx_path = declare_parameter<std::string>("model_onnx_path");
  const std::string model_engine_path = declare_parameter<std::string>("model_engine_path");
  
  const int max_voxels = declare_parameter<int>("model_params.max_voxels");
  const int max_points_per_voxel = declare_parameter<int>("model_params.max_points_per_voxel");

  // valid: 0, invalid: 1
  //                 prediction
  //               |  valid  invalid
  //       --------+-----------------
  // label   valid |    TP      FN 
  //       invalid |    FP      TN
  //
  // P(z = 0 | s = 0) = TP / (TP + FN)
  // P(z = 1 | s = 0) = FN / (TP + FN)
  // P(z = 0 | s = 1) = FP / (FP + TN)
  // P(z = 1 | s = 1) = TN / (FP + TN)
  //
  p_z_given_s_[0][0] =
    declare_parameter<float>("model_params.prob_observe_valid_given_state_valid");
  p_z_given_s_[1][0] =
    declare_parameter<float>("model_params.prob_observe_invalid_given_state_valid");
  p_z_given_s_[0][1] =
    declare_parameter<float>("model_params.prob_observe_valid_given_state_invalid");
  p_z_given_s_[1][1] =
    declare_parameter<float>("model_params.prob_observe_invalid_given_state_invalid");

  const std::string trt_precision = declare_parameter<std::string>("trt_precision");
  const int profile_points_min = declare_parameter<int>("trt_profile_points_min");
  const int profile_points_opt = declare_parameter<int>("trt_profile_points_opt");
  const int profile_points_max = declare_parameter<int>("trt_profile_points_max");

  // update the map if the vehicle has moved from the fetched point
  // and has reached map_refetch_threshold of the map_fetch_range_
  // if map_fetch_range_ is small, it may cause some issue
  map_update_threshold_dist_ =
    static_cast<double>(
      map_fetch_range_ * map_refetch_threshold) * (map_fetch_range_ * map_refetch_threshold);

  std::vector<float> voxel_sizes = {prepro_voxel_size_xy, prepro_voxel_size_xy,
    prepro_voxel_size_z};
  std::vector<float> pcd_range = {pcd_min_x_range_, pcd_min_y_range_, pcd_min_z_range_,
    pcd_max_x_range_, pcd_max_y_range_, pcd_max_z_range_};

  // subscribers
  if (partial_load_) {
    pcd_map_cli_ = create_client<autoware_map_msgs::srv::GetPartialPointCloudMap>(
      "~/input/partial_pointcloud_map", rmw_qos_profile_default);
  } else {
    pcd_map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "~/input/pointcloud_map", rclcpp::SensorDataQoS{}.keep_last(1),
    std::bind(&LocalizedPositionValidatorNode::callback_map, this, std::placeholders::_1));
  }

  pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "~/input/pointcloud", rclcpp::SensorDataQoS{}.keep_last(1),
    std::bind(&LocalizedPositionValidatorNode::callback_pointcloud, this, std::placeholders::_1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "~/input/odometry", rclcpp::SensorDataQoS{}.keep_last(1),
    std::bind(&LocalizedPositionValidatorNode::callback_odometry, this, std::placeholders::_1));

  // publsiher
  inference_result_pub_ =
    create_publisher<ValidatorPredictionMsg>("~/validation_result", rclcpp::QoS{1});

  // model
  TrtCommonConfig model_param(model_onnx_path, trt_precision, model_engine_path);
  validator_ptr_ = std::make_unique<PositionValidatorTRT>(
    model_param, voxel_sizes, pcd_range, max_voxels, max_points_per_voxel,
    profile_points_min, profile_points_opt, profile_points_max);

  // debug
  stop_watch_ptr_ =
    std::make_unique<autoware_utils_system::StopWatch<std::chrono::milliseconds>>();
  debug_publisher_ptr_ =
    std::make_unique<autoware_utils_debug::DebugPublisher>(this, get_name());

  stop_watch_ptr_->tic("processing_time");

  if (publish_input_points_) {
    debug_input_point_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("~/debug/input", 10);
  }
}

LocalizedPositionValidatorNode::~LocalizedPositionValidatorNode()
{
  // join all worker threads to ensure clean shutdown
  for (auto & t : worker_threads_) {
    if (t.joinable()) t.join();
  }
}

void LocalizedPositionValidatorNode::update_belief_with_observation(const int & z)
{
  const float prior_valid = belief_valid_;
  const float prior_invalid = 1.0 - prior_valid;

  // P(S_t = s | Z_t) = P(Z_t | S_t = s) * P(S_t = s) / P(Z_t)
  const double p_z = p_z_given_s_[z][0] * prior_valid + p_z_given_s_[z][1] * prior_invalid;
  const double posterior_valid = p_z_given_s_[z][0] * prior_valid / p_z;

  // avoid zero-probability trapping
  belief_valid_ = static_cast<float>(std::clamp(posterior_valid, 1e-5, 1.0));
}

// soft Bayesian update assuming confidence is P(Z=valid)
void LocalizedPositionValidatorNode::update_belief_with_confidence(const float & valid_confidence)
{
  const float prior_valid = belief_valid_;
  const float prior_invalid = 1.0 - prior_valid;
  const float p_z_valid = valid_confidence;
  const float p_z_invalid = 1.0 - valid_confidence;

  // a * P(Z | S) + (1 - a) * P(Z | S)
  const double lh_v = p_z_valid * p_z_given_s_[0][0] + p_z_invalid * p_z_given_s_[1][0];
  const double lh_i = p_z_valid * p_z_given_s_[0][1] + p_z_invalid * p_z_given_s_[1][1];

  const double p_z = lh_v * prior_valid + lh_i * prior_invalid;
  const double posterior_valid = (lh_v * prior_valid) / p_z;

  // avoid zero-probability trapping
  belief_valid_ = static_cast<float>(std::clamp(posterior_valid, 1e-5, 1.0));
}

void LocalizedPositionValidatorNode::process_map(
  rclcpp::Client<autoware_map_msgs::srv::GetPartialPointCloudMap>::SharedFuture response)
{
  const auto res = response.get();
  RCLCPP_INFO(get_logger(), "received %lu partial pointcloud maps",
    res->new_pointcloud_with_ids.size());

  // merge map pointclouds
  sensor_msgs::msg::PointCloud2 merged_msg;
  for (const auto & pcd_with_id : res->new_pointcloud_with_ids) {
    if (merged_msg.data.empty()) {
      merged_msg = pcd_with_id.pointcloud;
    } else {
      merged_msg.width += pcd_with_id.pointcloud.width;
      merged_msg.row_step += pcd_with_id.pointcloud.row_step;
      merged_msg.data.insert(
        merged_msg.data.end(),
        pcd_with_id.pointcloud.data.begin(),
        pcd_with_id.pointcloud.data.end());
    }
  }

  // convert to PCL
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud =
    std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromROSMsg(merged_msg, *map_cloud);

  // downsample
  RCLCPP_DEBUG(get_logger(), "map arrived, downsampling...");
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setInputCloud(map_cloud);
  voxel_filter.setLeafSize(map_downsample_res_, map_downsample_res_, map_downsample_res_);
  pcl::PointCloud<pcl::PointXYZ> downsampled_cloud;
  voxel_filter.filter(downsampled_cloud);

  {
    // block map reading
    std::lock_guard<std::mutex> lock(mutex_map_);

    pcd_map_ = downsampled_cloud;
  }

  RCLCPP_DEBUG(get_logger(), "map loaded");

  map_arrived_ = true;
  is_map_request_in_progress_ = false;
}

void LocalizedPositionValidatorNode::on_map_response(
  rclcpp::Client<autoware_map_msgs::srv::GetPartialPointCloudMap>::SharedFuture response)
{
  if (!map_arrived_) {
    // for the first time, we will process in single thread
    process_map(response);
  } else {
    worker_threads_.emplace_back([this, response]() {
      process_map(response);
    });
  }
}

void LocalizedPositionValidatorNode::get_partial_point_cloud_map(
  const float & x, const float & y)
{
  if (!pcd_map_cli_->service_is_ready()) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "partial map loading in pointcloud_map_loader is not ready");
    return;
  }

  // request the map from the map loader node, using the current ego position.
  auto req = std::make_shared<autoware_map_msgs::srv::GetPartialPointCloudMap::Request>();
  req->area.center_x = x;
  req->area.center_y = y;
  req->area.radius = map_fetch_range_;

  // update the position to current requested position
  map_queried_pos_.x = x;
  map_queried_pos_.y = y;

  pcd_map_cli_->async_send_request(req,
    std::bind(&LocalizedPositionValidatorNode::on_map_response, this, std::placeholders::_1));
  is_map_request_in_progress_ = true;
}

void LocalizedPositionValidatorNode::callback_map(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr map_pointcloud_msg_ptr)
{
  RCLCPP_DEBUG(get_logger(), "map arrived, downsampling...");

  pcd_map_ = *downsample_voxel_grid(*map_pointcloud_msg_ptr, 2.0);

  RCLCPP_DEBUG(get_logger(), "map loaded.");

  map_arrived_ = true;
}

void LocalizedPositionValidatorNode::callback_odometry(
  const nav_msgs::msg::Odometry::ConstSharedPtr odometry_msg_ptr)
{
  vcl_odom_ = *odometry_msg_ptr;
  vcl_odom_arrived_ = true;
}

void LocalizedPositionValidatorNode::callback_pointcloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_pointcloud_msg_ptr)
{
  // wait odometry to arrive
  // to fetch map, we need the position of ego vehicle
  if (!vcl_odom_arrived_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 10000, "waiting odometry to arrive...");
    return;
  }

  if (map_arrived_) {
    if (partial_load_ && !is_map_request_in_progress_) {
      // check how far the vehicle has moved
      const float diff_x = vcl_odom_.pose.pose.position.x - map_queried_pos_.x;
      const float diff_y = vcl_odom_.pose.pose.position.y - map_queried_pos_.y;
      if (diff_x * diff_x + diff_y * diff_y >= map_update_threshold_dist_) {
        get_partial_point_cloud_map(static_cast<float>(vcl_odom_.pose.pose.position.x),
          static_cast<float>(vcl_odom_.pose.pose.position.y));

        RCLCPP_INFO(get_logger(), "waiting new map to arrive...");
      }
    }
  } else { // loading map first time
    if (!is_map_request_in_progress_) {
      get_partial_point_cloud_map(
        static_cast<float>(vcl_odom_.pose.pose.position.x),
        static_cast<float>(vcl_odom_.pose.pose.position.y));
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 10000, "waiting map to arrive...");

    return;
  }

  if (input_pointcloud_msg_ptr->width * input_pointcloud_msg_ptr->height == 0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000, "input points length is 0.");
    return;
  }

  rclcpp::Time input_pcd_ts(input_pointcloud_msg_ptr->header.stamp);
  rclcpp::Time vcl_odom_ts(vcl_odom_.header.stamp);
  rclcpp::Duration diff =
    (input_pcd_ts > vcl_odom_ts) ? (input_pcd_ts - vcl_odom_ts) : (vcl_odom_ts - input_pcd_ts);
  if (diff > max_time_diff_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000, "input topic and odometry topic are not synced.");
    return;
  }

  stop_watch_ptr_->toc("processing_time", true);

  Eigen::Vector3f vehicle_position{
    static_cast<float>(vcl_odom_.pose.pose.position.x),
    static_cast<float>(vcl_odom_.pose.pose.position.y),
    static_cast<float>(vcl_odom_.pose.pose.position.z)
  };

  std::vector<Eigen::Vector4f> roi_map_pts;
  {
    // block map overwriting
    std::lock_guard<std::mutex> lock(mutex_map_);

    // filter and transform map
    constexpr float map_label = 0.0; // this must be the same as training
    roi_map_pts = preprocess_input_points(
      pcd_map_,
      pcd_min_x_range_, pcd_max_x_range_,
      pcd_min_y_range_, pcd_max_y_range_,
      pcd_min_z_range_, pcd_max_z_range_,
      vehicle_position,
      map_label);
  }

  // filter and transform scan points
  constexpr float scan_points_label = 1.0; // this must be the same as training
  std::vector<Eigen::Vector4f> roi_scan_pts = preprocess_input_points(
    *input_pointcloud_msg_ptr,
    pcd_min_x_range_, pcd_max_x_range_,
    pcd_min_y_range_, pcd_max_y_range_,
    pcd_min_z_range_, pcd_max_z_range_,
    vehicle_position,
    scan_points_label);

  // concatenate map and scan points data
  std::vector<Eigen::Vector4f> concat_data = concatenate_points(roi_map_pts, roi_scan_pts);

  // run the model
  validator_ptr_->inference(concat_data, predicted_class_, confidence_);

  ValidatorPredictionMsg res_msg;
  res_msg.header = input_pointcloud_msg_ptr->header;
  res_msg.is_position_valid = predicted_class_ == ValidatorPredictionMsg::VALID ? true : false;
  res_msg.confidence = confidence_;

  // update the belief state
  if (use_soft_bayesian_update_) {
    // pass valid confidence
    update_belief_with_confidence(
      (predicted_class_ == ValidatorPredictionMsg::VALID) ? confidence_ : 1.0 - confidence_);
  } else {
    update_belief_with_observation(predicted_class_);
  }

  // set belief state 
  constexpr float belief_valid_prob = 0.9;
  constexpr float belief_invalid_prob = 0.1;
  if (belief_valid_ >= belief_valid_prob) {
    res_msg.prediction_belief_state = static_cast<uint8_t>(ValidatorPredictionMsg::VALID);
  } else if (belief_valid_ <= belief_invalid_prob) {
    res_msg.prediction_belief_state = static_cast<uint8_t>(ValidatorPredictionMsg::INVALID);
  } else {
    res_msg.prediction_belief_state = static_cast<uint8_t>(ValidatorPredictionMsg::UNKNOWN);
  }

  // publish the result
  inference_result_pub_->publish(res_msg);

  // Publish debug info
  debug_publisher_ptr_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "debug/processing_time_ms", stop_watch_ptr_->toc("processing_time", true));

  if (publish_input_points_) {
    // publish input points for debugging
    pcl::PointCloud<pcl::PointXYZRGB> computed_point_rgb;
    color_input_points(concat_data, computed_point_rgb,
      static_cast<float>(vcl_odom_.pose.pose.position.x),
      static_cast<float>(vcl_odom_.pose.pose.position.y),
      static_cast<float>(vcl_odom_.pose.pose.position.z));

    sensor_msgs::msg::PointCloud2 input_points;
    pcl::toROSMsg(computed_point_rgb, input_points);

    input_points.header.stamp = input_pointcloud_msg_ptr->header.stamp;
    input_points.header.frame_id = "map";

    debug_input_point_pub_->publish(input_points);
  }
}

}  // namespace autoware::localized_position_validator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::localized_position_validator::LocalizedPositionValidatorNode)
