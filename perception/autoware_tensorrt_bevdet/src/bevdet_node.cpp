// Copyright 2025 AutoCore, Inc.
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

// cspell:ignore BEVDET, thre, TRTBEV, bevdet, caminfo, intrin, Ncams, bevfeat, dlongterm

#include "autoware/tensorrt_bevdet/bevdet_node.hpp"

#include <memory>
#include <string>
#include <vector>

namespace autoware
{
namespace tensorrt_bevdet
{
TRTBEVDetNode::TRTBEVDetNode(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("tensorrt_bevdet_node", node_options)
{
  // Get precision parameter
  precision_ = this->declare_parameter<std::string>("precision");
  RCLCPP_INFO(this->get_logger(), "Using precision mode: %s", precision_.c_str());

  debug_mode_ = this->declare_parameter<bool>("debug_mode");

  // Only start camera info subscription and tf listener at the beginning
  img_n_ = this->declare_parameter<int>("data_params.camera_count");

  caminfo_received_ = std::vector<bool>(img_n_, false);
  cams_intrin_ = std::vector<Eigen::Matrix3f>(img_n_);
  cams2ego_rot_ = std::vector<Eigen::Quaternion<float>>(img_n_);
  cams2ego_trans_ = std::vector<Eigen::Translation3f>(img_n_);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  score_threshold_ =
    static_cast<float>(this->declare_parameter<double>("post_process_params.score_threshold"));

  model_config_ = this->declare_parameter<std::string>("model_config");

  onnx_file_ = this->declare_parameter<std::string>("onnx_path");

  // Generate engine file name based on precision
  std::string engine_file_base =
    this->declare_parameter<std::string>("engine_path", "bevdet_one_lt_d");
  engine_file_ = engine_file_base + (precision_ == "fp16" ? "_fp16.engine" : "_fp32.engine");

  imgs_name_ = this->declare_parameter<std::vector<std::string>>("data_params.cams");
  class_names_ =
    this->declare_parameter<std::vector<std::string>>("post_process_params.class_names");

  // load image width and height from model config YAML
  YAML::Node config = YAML::LoadFile(model_config_);
  auto src_size = config["data_config"]["src_size"];
  img_h_ = src_size[0].as<size_t>();  // height
  img_w_ = src_size[1].as<size_t>();  // width

  startCameraInfoSubscription();

  // Create publishers for detected objects and markers
  pub_boxes_ = this->create_publisher<autoware_perception_msgs::msg::DetectedObjects>(
    "~/output/boxes", rclcpp::QoS{1});
  if (debug_mode_) {
    pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/output_bboxes", rclcpp::QoS{1});
  }

  // Wait for camera info and tf transform initialization
  initialization_check_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&TRTBEVDetNode::checkInitialization, this));
}

void TRTBEVDetNode::initModel()
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Successfully loaded config!");

  inference_input_.param = camParams(cams_intrin_, cams2ego_rot_, cams2ego_trans_);

  RCLCPP_INFO_STREAM(this->get_logger(), "Successfully loaded image params!");

  bevdet_ = std::make_shared<BEVDet>(
    model_config_, img_n_, inference_input_.param.cams_intrin, inference_input_.param.cams2ego_rot,
    inference_input_.param.cams2ego_trans, onnx_file_, engine_file_, precision_);

  RCLCPP_INFO_STREAM(this->get_logger(), "Successfully created bevdet!");

  CHECK_CUDA(cudaMalloc(
    reinterpret_cast<void **>(&imgs_dev_), img_n_ * 3 * img_w_ * img_h_ * sizeof(uchar)));
}

void TRTBEVDetNode::checkInitialization()
{
  if (camera_info_received_flag_) {
    RCLCPP_INFO_STREAM(
      this->get_logger(), "Camera Info and TF Transform Initialization completed!");
    initModel();
    startImageSubscription();
    initialization_check_timer_->cancel();
    initialization_check_timer_.reset();
  } else {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Waiting for Camera Info and TF Transform Initialization...");
  }
}

void TRTBEVDetNode::startImageSubscription()
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  using std::placeholders::_4;
  using std::placeholders::_5;
  using std::placeholders::_6;

  sub_f_img_.subscribe(
    this, "~/input/topic_img_front", rclcpp::SensorDataQoS{}.keep_last(1).get_rmw_qos_profile());
  sub_b_img_.subscribe(
    this, "~/input/topic_img_back", rclcpp::SensorDataQoS{}.keep_last(1).get_rmw_qos_profile());

  sub_fl_img_.subscribe(
    this, "~/input/topic_img_front_left",
    rclcpp::SensorDataQoS{}.keep_last(1).get_rmw_qos_profile());
  sub_fr_img_.subscribe(
    this, "~/input/topic_img_front_right",
    rclcpp::SensorDataQoS{}.keep_last(1).get_rmw_qos_profile());

  sub_bl_img_.subscribe(
    this, "~/input/topic_img_back_left",
    rclcpp::SensorDataQoS{}.keep_last(1).get_rmw_qos_profile());
  sub_br_img_.subscribe(
    this, "~/input/topic_img_back_right",
    rclcpp::SensorDataQoS{}.keep_last(1).get_rmw_qos_profile());

  sync_ = std::make_shared<Sync>(
    MultiCameraApproxSync(10), sub_fl_img_, sub_f_img_, sub_fr_img_, sub_bl_img_, sub_b_img_,
    sub_br_img_);

  sync_->registerCallback(std::bind(&TRTBEVDetNode::callback, this, _1, _2, _3, _4, _5, _6));
}

void TRTBEVDetNode::startCameraInfoSubscription()
{
  // cams: ["CAM_FRONT_LEFT", "CAM_FRONT", "CAM_FRONT_RIGHT", "CAM_BACK_LEFT", "CAM_BACK",
  // "CAM_BACK_RIGHT"]
  sub_fl_caminfo_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "~/input/topic_img_front_left/camera_info", rclcpp::SensorDataQoS{}.keep_last(1),
    [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) { cameraInfoCallback(0, msg); });

  sub_f_caminfo_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "~/input/topic_img_front/camera_info", rclcpp::SensorDataQoS{}.keep_last(1),
    [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) { cameraInfoCallback(1, msg); });

  sub_fr_caminfo_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "~/input/topic_img_front_right/camera_info", rclcpp::SensorDataQoS{}.keep_last(1),
    [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) { cameraInfoCallback(2, msg); });

  sub_bl_caminfo_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "~/input/topic_img_back_left/camera_info", rclcpp::SensorDataQoS{}.keep_last(1),
    [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) { cameraInfoCallback(3, msg); });

  sub_b_caminfo_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "~/input/topic_img_back/camera_info", rclcpp::SensorDataQoS{}.keep_last(1),
    [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) { cameraInfoCallback(4, msg); });

  sub_br_caminfo_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "~/input/topic_img_back_right/camera_info", rclcpp::SensorDataQoS{}.keep_last(1),
    [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) { cameraInfoCallback(5, msg); });
}

void TRTBEVDetNode::callback(
  const sensor_msgs::msg::Image::ConstSharedPtr & msg_fl_img,
  const sensor_msgs::msg::Image::ConstSharedPtr & msg_f_img,
  const sensor_msgs::msg::Image::ConstSharedPtr & msg_fr_img,
  const sensor_msgs::msg::Image::ConstSharedPtr & msg_bl_img,
  const sensor_msgs::msg::Image::ConstSharedPtr & msg_b_img,
  const sensor_msgs::msg::Image::ConstSharedPtr & msg_br_img)
{
  std::vector<cv::Mat> imgs;

  imgs.emplace_back(cloneAndResize(msg_fl_img));
  imgs.emplace_back(cloneAndResize(msg_f_img));
  imgs.emplace_back(cloneAndResize(msg_fr_img));
  imgs.emplace_back(cloneAndResize(msg_bl_img));
  imgs.emplace_back(cloneAndResize(msg_b_img));
  imgs.emplace_back(cloneAndResize(msg_br_img));

  imageTransport(imgs, imgs_dev_, img_w_, img_h_);

  inference_input_.imgs_dev = imgs_dev_;

  std::vector<Box> ego_boxes;
  float inference_duration = 0.f;

  bevdet_->DoInfer(inference_input_, ego_boxes, inference_duration);

  autoware_perception_msgs::msg::DetectedObjects bevdet_objects;
  bevdet_objects.header.frame_id = "base_link";
  bevdet_objects.header.stamp = msg_f_img->header.stamp;

  box3DToDetectedObjects(ego_boxes, bevdet_objects, class_names_, score_threshold_, has_twist_);

  pub_boxes_->publish(bevdet_objects);

  if (debug_mode_) {
    publishDebugMarkers(pub_markers_, bevdet_objects);
  }
}

void TRTBEVDetNode::cameraInfoCallback(int idx, const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  if (caminfo_received_[idx])
    return;  // already received;  not expected to modify because of we init the model only once

  Eigen::Matrix3f intrinsics;
  getCameraIntrinsics(msg, intrinsics);
  cams_intrin_[idx] = intrinsics;

  Eigen::Quaternion<float> rot;
  Eigen::Translation3f translation;
  try {
    getTransform(
      tf_buffer_->lookupTransform("base_link", msg->header.frame_id, rclcpp::Time(0)), rot,
      translation);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "Transform lookup failed: %s", ex.what());
    return;
  }
  cams2ego_rot_[idx] = rot;
  cams2ego_trans_[idx] = translation;

  caminfo_received_[idx] = true;
  camera_info_received_flag_ =
    std::all_of(caminfo_received_.begin(), caminfo_received_.end(), [](bool i) { return i; });
}

cv::Mat TRTBEVDetNode::cloneAndResize(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
  if (img.size() != cv::Size(img_w_, img_h_)) {
    cv::resize(img, img, cv::Size(img_w_, img_h_));  // Resize if needed
  }
  return img;
}

TRTBEVDetNode::~TRTBEVDetNode()
{
  delete imgs_dev_;
}
}  // namespace tensorrt_bevdet
}  // namespace autoware
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::tensorrt_bevdet::TRTBEVDetNode)
