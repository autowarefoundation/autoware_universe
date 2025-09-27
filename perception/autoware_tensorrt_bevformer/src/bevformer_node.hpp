// Copyright 2025 The Autoware Contributors
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
//
/*
 * Copyright (c) 2025 Multicoreware, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// cspell:ignore BEVFORMER, thre, caminfo, intrin

#ifndef BEVFORMER_NODE_HPP_
#define BEVFORMER_NODE_HPP_

#include "bevformer_data_loader.hpp"
#include "bevformer_data_manager.hpp"
#include "bevformer_inference_engine.hpp"
#include "bevformer_preprocessor.hpp"
#include "marker_util.hpp"
#include "postprocessing/postprocessing.hpp"
#include "ros_utils.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <rclcpp/logging.hpp>

#include "autoware_localization_msgs/msg/kinematic_state.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware
{
namespace tensorrt_bevformer
{
class TRTBEVFormerNode : public rclcpp::Node
{
private:
  size_t img_N_;
  uint32_t src_img_w_;
  uint32_t src_img_h_;
  std::map<std::string, int> model_shape_params_;
  std::map<std::string, std::vector<std::string>> model_output_shapes_;

  std::string engine_file_;
  std::string plugin_path_;

  std::vector<std::string> class_names_;

  std::vector<double> pc_range_;
  std::vector<double> post_center_range_;

  std::string onnx_file_;
  int workspace_size_;
  bool auto_convert_;
  std::string precision_;

  std::map<std::string, std::vector<double>> reshapeTensorOutputs(
    const std::map<std::string, std::vector<double>> & rawOutputs);

  // Static transforms (calculated once)
  Eigen::Quaterniond lidar2ego_rot_static_;
  Eigen::Translation3d lidar2ego_trans_static_;
  bool lidar2ego_transforms_ready_ = false;

  // Inference components
  float score_thre_;         // Score threshold for object detection
  bool has_twist_ = true;    // whether set twist for objects
  bool debug_mode_ = false;  // Flag to enable debug mode for nuscenes marker visualization

  // Publishers and subscribers
  rclcpp::Publisher<autoware_perception_msgs::msg::DetectedObjects>::SharedPtr pub_boxes_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;

  // Camera subscriptions
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_f_caminfo_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_fl_caminfo_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_fr_caminfo_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_b_caminfo_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_bl_caminfo_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_br_caminfo_;

  // Initialization flags
  std::vector<bool>
    caminfo_received_;  // Flag indicating if camera info has been received for each camera
  bool camera_info_received_flag_ = false;

  // can_bus subscription
  message_filters::Subscriber<autoware_localization_msgs::msg::KinematicState> sub_kinematic_state;

  // tf listener
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  // Camera parameters
  std::vector<Eigen::Matrix3d> cams_intrin_;
  std::vector<Eigen::Quaternion<double>> cams2ego_rot_;
  std::vector<Eigen::Translation3d> cams2ego_trans_;

  // Image subscribers with synchronization
  message_filters::Subscriber<sensor_msgs::msg::Image> sub_f_img_;
  message_filters::Subscriber<sensor_msgs::msg::Image> sub_fl_img_;
  message_filters::Subscriber<sensor_msgs::msg::Image> sub_fr_img_;
  message_filters::Subscriber<sensor_msgs::msg::Image> sub_b_img_;
  message_filters::Subscriber<sensor_msgs::msg::Image> sub_bl_img_;
  message_filters::Subscriber<sensor_msgs::msg::Image> sub_br_img_;

  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image,
    sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image,
    autoware_localization_msgs::msg::KinematicState>
    MultiSensorSyncPolicy;

  typedef message_filters::Synchronizer<MultiSensorSyncPolicy> Sync;
  std::shared_ptr<Sync> sync_;

  // Timer for checking initialization
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<Eigen::Matrix4d> viewpad_matrices_;

  // Sensor2Lidar transformation storage
  std::vector<Eigen::Matrix3d> sensor2lidar_rotation_;
  std::vector<Eigen::Vector3d> sensor2lidar_translation_;

  // To add the modular components
  std::unique_ptr<BEVFormerPreprocessor> preprocessor_;
  std::unique_ptr<BEVFormerInferenceEngine> inference_engine_;
  std::unique_ptr<BEVFormerDataManager> data_manager_;

  void calculateSensor2LidarTransformsFromTF(
    const std::vector<sensor_msgs::msg::Image::ConstSharedPtr> & image_msgs,
    const Eigen::Quaterniond & ego2global_rot_ref,
    const Eigen::Translation3d & ego2global_trans_ref);

  std::vector<float> extractCanBusFromKinematicState(
    const autoware_localization_msgs::msg::KinematicState::ConstSharedPtr & kinematic_state_msg);

  void startCameraInfoSubscription();
  void checkInitialization();
  void initModel();
  void startImageSubscription();

  void calculateStaticLidar2EgoTransform();

  /**
   * @brief Clones and resizes the input image message to the specified width and height.
   * @param msg The input image message.
   * @return The cloned and resized OpenCV Mat image.
   */
  cv::Mat cloneAndResize(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

public:
  explicit TRTBEVFormerNode(const rclcpp::NodeOptions & node_options);

  // Callbacks
  void callback(
    const sensor_msgs::msg::Image::ConstSharedPtr & msg_fl_img,
    const sensor_msgs::msg::Image::ConstSharedPtr & msg_f_img,
    const sensor_msgs::msg::Image::ConstSharedPtr & msg_fr_img,
    const sensor_msgs::msg::Image::ConstSharedPtr & msg_bl_img,
    const sensor_msgs::msg::Image::ConstSharedPtr & msg_b_img,
    const sensor_msgs::msg::Image::ConstSharedPtr & msg_br_img,
    const autoware_localization_msgs::msg::KinematicState::ConstSharedPtr & can_bus_msg);

  void cameraInfoCallback(int idx, const sensor_msgs::msg::CameraInfo::SharedPtr msg);
};

}  // namespace tensorrt_bevformer
}  // namespace autoware

#endif  // BEVFORMER_NODE_HPP_
