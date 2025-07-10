// Copyright 2025 TIER IV
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

#include "autoware/camera_streampetr/node.hpp"
#include "autoware/camera_streampetr/postprocess/non_maximum_supression.hpp"
#include "autoware/camera_streampetr/network/build_trt.hpp"

#include <Eigen/Dense>

#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <algorithm>

namespace autoware::camera_streampetr
{


std::vector<float> cast_to_float(const std::vector<double>& double_vector) {
  std::vector<float> float_vector(double_vector.size());
  std::transform(double_vector.begin(), double_vector.end(), float_vector.begin(),
                [](double value) { return static_cast<float>(value); });
  return float_vector;
}

StreamPetrNode::StreamPetrNode(const rclcpp::NodeOptions & node_options)
: Node("autoware_camera_streampetr", node_options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_),
  rois_number_(static_cast<size_t>(declare_parameter<int>("rois_number", 6))),
  is_compressed_image_(declare_parameter<bool>("is_compressed_image")),
  max_camera_time_diff_(declare_parameter<float>("model_params.max_camera_time_diff", 0.2f)),
  anchor_camera_id_(declare_parameter<int>("anchor_camera_id", 0)),
  cycle_started_(false),
  debug_mode_(declare_parameter<bool>("debug_mode"))
{
  RCLCPP_INFO(get_logger(), "nvinfer: %d.%d.%d\n", NV_TENSORRT_MAJOR, NV_TENSORRT_MINOR, NV_TENSORRT_PATCH);
  cudaSetDevice(0);

  using std::placeholders::_1;

  // Initialize parameters
  const std::string backbone_path = declare_parameter<std::string>("model_params.backbone_path");
  const std::string head_path = declare_parameter<std::string>("model_params.head_path");
  const std::string position_embedding_path = declare_parameter<std::string>("model_params.position_embedding_path");
  const bool fp16_mode = declare_parameter<bool>("model_params.fp16_mode");
  const bool build_only = declare_parameter<bool>("build_only");

  const std::string engine_backbone_path = initEngine(backbone_path, fp16_mode, false, get_logger());
  const std::string engine_head_path = initEngine(head_path, fp16_mode, true, get_logger());
  const std::string engine_position_embedding_path = initEngine(position_embedding_path, fp16_mode, false, get_logger());

  if (build_only) {
    RCLCPP_INFO(get_logger(), "TensorRT engine files built successfully. Shutting Down...");
    rclcpp::shutdown();
  }

  localization_sub_ = this->create_subscription<Odometry>(
      "~/input/kinematic_state", rclcpp::QoS{1},
      [this](const Odometry::ConstSharedPtr msg) {
        this->odometry_callback(msg);
      }
    );

  camera_info_subs_.resize(rois_number_);

  if (is_compressed_image_) 
    compressed_camera_image_subs_.resize(rois_number_);
  else
    camera_image_subs_.resize(rois_number_);

  for (size_t roi_i = 0; roi_i < rois_number_; ++roi_i) {
    camera_info_subs_.at(roi_i) = this->create_subscription<CameraInfo>(
      "~/input/camera" + std::to_string(roi_i) + "/camera_info", rclcpp::SensorDataQoS{}.keep_last(1),
      [this, roi_i](const CameraInfo::ConstSharedPtr msg) {
        this->camera_info_callback(msg, static_cast<int>(roi_i));
      }
    );

    if (is_compressed_image_) {
      compressed_camera_image_subs_.at(roi_i) = this->create_subscription<CompressedImage>(
        "~/input/camera" + std::to_string(roi_i) + "/image", rclcpp::SensorDataQoS{}.keep_last(1),
        [this, roi_i](const CompressedImage::ConstSharedPtr msg) {
          this->camera_image_callback(msg, static_cast<int>(roi_i));
        }
      );
    } else {
      camera_image_subs_.at(roi_i) = this->create_subscription<Image>(
        "~/input/camera" + std::to_string(roi_i) + "/image", rclcpp::SensorDataQoS{}.keep_last(1),
        [this, roi_i](const Image::ConstSharedPtr msg) {
          this->camera_image_callback(msg, static_cast<int>(roi_i));
        }
      );
    }

  }
  
  // Publishers
  pub_objects_ = this->create_publisher<DetectedObjects>("~/output/objects", rclcpp::QoS{1});

  // Data store
  data_store_ = std::make_unique<CameraDataStore>(
    this, rois_number_,
    declare_parameter<int>("model_params.input_image_height"),
    declare_parameter<int>("model_params.input_image_width"),
    anchor_camera_id_,
    is_compressed_image_,
    declare_parameter<int>("decompression_downsample")
  );
  const bool use_temporal = declare_parameter<bool>("model_params.use_temporal"); 
  const double search_distance_2d = declare_parameter<double>("post_process_params.iou_nms_search_distance_2d");
  const double circle_nms_dist_threshold = declare_parameter<double>("post_process_params.circle_nms_dist_threshold");
  const double iou_threshold = declare_parameter<double>("post_process_params.iou_nms_threshold");
  const double confidence_threshold = declare_parameter<double>("post_process_params.confidence_threshold");
  const std::vector<std::string> class_names =  declare_parameter<std::vector<std::string>>("model_params.class_names");
  const int32_t num_proposals = declare_parameter<int32_t>("model_params.num_proposals");
  const std::vector<double> yaw_norm_thresholds = declare_parameter<std::vector<double>>("post_process_params.yaw_norm_thresholds");
  const std::vector<float> detection_range = cast_to_float(declare_parameter<std::vector<double>>("model_params.detection_range"));
  network_ = std::make_unique<StreamPetrNetwork>(engine_backbone_path, 
                                                engine_head_path, 
                                                engine_position_embedding_path, 
                                                use_temporal, search_distance_2d,
                                                circle_nms_dist_threshold, iou_threshold, 
                                                confidence_threshold, class_names, 
                                                num_proposals, yaw_norm_thresholds,
                                                detection_range);
  std::cout << "debug_mode_: " << debug_mode_ << std::endl;
  if(debug_mode_) {
    using autoware_utils::DebugPublisher;
    using autoware_utils::StopWatch;
    stop_watch_ptr_ = std::make_unique<StopWatch<std::chrono::milliseconds>>();
    debug_publisher_ptr_ = std::make_unique<DebugPublisher>(this, this->get_name());
  }
}

void StreamPetrNode::odometry_callback(
  Odometry::ConstSharedPtr input_msg)
{
  if (!initial_kinematic_state_) {
    initial_kinematic_state_ = input_msg;
  }
  latest_kinematic_state_ = input_msg;
  return;
}

void StreamPetrNode::camera_info_callback(
  CameraInfo::ConstSharedPtr input_camera_info_msg,
  const int camera_id)
{
  data_store_->update_camera_info(camera_id, input_camera_info_msg);
}

void StreamPetrNode::camera_image_callback(
  Image::ConstSharedPtr input_camera_image_msg,
  const int camera_id)
{ 
  if (!data_store_->check_if_all_camera_info_received()) {
    RCLCPP_WARN(get_logger(), "Camera info not received for all cameras");
    return;
  }

  if (!cycle_started_) {
    cycle_started_ = true;
    if(stop_watch_ptr_)
      stop_watch_ptr_->tic("latency/cycle_time_ms");
  }

  data_store_->update_camera_image(camera_id, input_camera_image_msg);
  RCLCPP_INFO(get_logger(), "received camera %d", camera_id);

  if (camera_id == anchor_camera_id_)
    step();

}


void StreamPetrNode::camera_image_callback(
  CompressedImage::ConstSharedPtr input_camera_image_msg,
  const int camera_id)
{ 
  if (!data_store_->check_if_all_camera_info_received()) {
    RCLCPP_WARN(get_logger(), "Camera info not received for all cameras");
    return;
  }

  if (!cycle_started_) {
    cycle_started_ = true;
    if(stop_watch_ptr_)
      stop_watch_ptr_->tic("latency/cycle_time_ms");
  }

  data_store_->update_camera_image_compressed(camera_id, input_camera_image_msg);
  RCLCPP_INFO(get_logger(), "received camera %d", camera_id);

  if (camera_id == anchor_camera_id_)
    step();

}

void StreamPetrNode::step() {
  
  double processing_time_ms = -1.0;
  double inference_time_ms = -1.0;

  if (stop_watch_ptr_) {
    stop_watch_ptr_->tic("latency/total");
  }

  const float tdiff = data_store_->check_if_all_images_synced();

  if (tdiff < 0) {
    RCLCPP_WARN(get_logger(), "Not all camera info or image received");
    return;
  }

  if(tdiff>max_camera_time_diff_){
    RCLCPP_WARN(get_logger(), "Cameras are not synced, difference is %.2f seconds", tdiff);
    return;
  }

  std::vector<autoware_perception_msgs::msg::DetectedObject> output_objects;
  RCLCPP_INFO(get_logger(), "All images are synchronized and all camera info received. Predicting after %.2f seconds", static_cast<double>(data_store_->get_timestamp()));
  const auto [ego_pose, ego_pose_inv] = get_ego_pose_vector();

  if(stop_watch_ptr_)
    stop_watch_ptr_->tic("latency/inference");
  std::vector<float> forward_time_ms;
  network_->inference_detector(
    data_store_->get_image_input(),
    ego_pose, ego_pose_inv,
    data_store_->get_image_shape(),
    data_store_->get_camera_info_vector(),
    get_camera_extrinsics_vector(data_store_->get_camera_link_names()),
    data_store_->get_timestamp(),
    output_objects,
    forward_time_ms
  );
  data_store_->step();
  
  if(stop_watch_ptr_)
    inference_time_ms = stop_watch_ptr_->toc("latency/inference", true);
  
  // data_store_->reset_camera_images();
  DetectedObjects output_msg;
  output_msg.objects = output_objects;
  output_msg.header.frame_id = "base_link";
  pub_objects_->publish(output_msg);

  if (stop_watch_ptr_) {
    processing_time_ms = stop_watch_ptr_->toc("latency/total", true);
  }

  if (debug_publisher_ptr_ && stop_watch_ptr_) {
    debug_publisher_ptr_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "latency/preprocess", data_store_->get_preprocess_time_ms());
    debug_publisher_ptr_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "latency/inference", inference_time_ms);
    debug_publisher_ptr_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "latency/total", processing_time_ms);
    debug_publisher_ptr_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "latency/inference/backbone", forward_time_ms[0]);
    debug_publisher_ptr_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "latency/inference/ptshead", forward_time_ms[1]);
    debug_publisher_ptr_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "latency/inference/pos_embed", forward_time_ms[2]);
    debug_publisher_ptr_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "latency/inference/postprocess", forward_time_ms[3]);
    debug_publisher_ptr_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "latency/cycle_time_ms", stop_watch_ptr_->toc("latency/cycle_time_ms", true));
  }
  cycle_started_ = false;
}

std::vector<float> StreamPetrNode::get_camera_extrinsics_vector(
  const std::vector<std::string> & camera_links)
{
  std::vector<float> intrinsics_all = data_store_->get_camera_info_vector();

  std::vector<float> res;
  res.reserve(camera_links.size() * 16);

  for (size_t i = 0; i < camera_links.size(); ++i) {
    Eigen::Matrix4f K_4x4 = Eigen::Matrix4f::Identity();
    {
      size_t offset = i * 16;
      for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
          K_4x4(row, col) = intrinsics_all[offset + row*4 + col];
        }
      }
    }
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      transform_stamped =
        tf_buffer_.lookupTransform(camera_links[i], "base_link", tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      throw std::runtime_error(
        "Could not transform from base_link to " + camera_links[i] + 
        ": " + std::string(ex.what()));
    }

    Eigen::Matrix4f T_lidar2cam = Eigen::Matrix4f::Identity();
    {
      tf2::Quaternion tf2_q(
        transform_stamped.transform.rotation.x,
        transform_stamped.transform.rotation.y,
        transform_stamped.transform.rotation.z,
        transform_stamped.transform.rotation.w
      );
      tf2::Matrix3x3 tf2_R(tf2_q);

      Eigen::Matrix3f R;
      for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
          R(r, c) = static_cast<float>(tf2_R[r][c]);
        }
      }

      Eigen::Vector3f t;
      t << static_cast<float>(transform_stamped.transform.translation.x),
           static_cast<float>(transform_stamped.transform.translation.y),
           static_cast<float>(transform_stamped.transform.translation.z);

      for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
          T_lidar2cam(r, c) = R(r, c);
        }
        T_lidar2cam(r, 3) = t(r);
      }
    }

    Eigen::Matrix4f T_lidar2img = K_4x4 * T_lidar2cam;
    Eigen::Matrix4f T_img2lidar = T_lidar2img.inverse();

  // std::cout << "Final res vector:\n";
    for (int row = 0; row < 4; ++row) {
      for (int col = 0; col < 4; ++col) {
        // std::cout << T_img2lidar(row, col) << " ";
        res.push_back(T_img2lidar(row, col));
      }
      // std::cout <<  "\n";
    }
  }

  return res;
}

std::pair<std::vector<float>, std::vector<float>> StreamPetrNode::get_ego_pose_vector() const
{
  if (!latest_kinematic_state_ || !initial_kinematic_state_) {
    throw std::runtime_error("Kinematic states have not been received.");
  }

  const auto& latest_pose = latest_kinematic_state_->pose.pose;
  const auto& initial_pose = initial_kinematic_state_->pose.pose;
  tf2::Quaternion latest_quat(latest_pose.orientation.x, latest_pose.orientation.y, latest_pose.orientation.z, latest_pose.orientation.w);
  tf2::Matrix3x3 latest_rot;
  latest_rot.setRotation(latest_quat);
  tf2::Matrix3x3 relative_rot = latest_rot;
  tf2::Vector3 latest_translation(latest_pose.position.x-initial_pose.position.x, latest_pose.position.y-initial_pose.position.y, latest_pose.position.z-initial_pose.position.z);
  tf2::Vector3 relative_translation = latest_translation;

  std::vector<float> egopose = {
    static_cast<float>(relative_rot[0][0]), static_cast<float>(relative_rot[0][1]), static_cast<float>(relative_rot[0][2]), static_cast<float>(relative_translation.x()),
    static_cast<float>(relative_rot[1][0]), static_cast<float>(relative_rot[1][1]), static_cast<float>(relative_rot[1][2]), static_cast<float>(relative_translation.y()),
    static_cast<float>(relative_rot[2][0]), static_cast<float>(relative_rot[2][1]), static_cast<float>(relative_rot[2][2]), static_cast<float>(relative_translation.z()),
    0.0f, 0.0f, 0.0f, 1.0f
  };

  tf2::Matrix3x3 inverse_rot = relative_rot.transpose();
  tf2::Vector3 inverse_translation = -(inverse_rot * relative_translation);

  std::vector<float> inverse_egopose = {
    static_cast<float>(inverse_rot[0][0]), static_cast<float>(inverse_rot[0][1]), static_cast<float>(inverse_rot[0][2]), static_cast<float>(inverse_translation.x()),
    static_cast<float>(inverse_rot[1][0]), static_cast<float>(inverse_rot[1][1]), static_cast<float>(inverse_rot[1][2]), static_cast<float>(inverse_translation.y()),
    static_cast<float>(inverse_rot[2][0]), static_cast<float>(inverse_rot[2][1]), static_cast<float>(inverse_rot[2][2]), static_cast<float>(inverse_translation.z()),
    0.0f, 0.0f, 0.0f, 1.0f
  };

  return std::make_pair(egopose, inverse_egopose);
}



}  // namespace autoware::camera_streampetr


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::camera_streampetr::StreamPetrNode)