// Copyright 2025 TIER IV.
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

#include "../src/vad_interface.hpp"

#include <opencv2/opencv.hpp>

#include <cmath>
#include <memory>
#include <optional>

namespace autoware::tensorrt_vad
{

namespace
{
const rclcpp::Logger & vad_logger()
{
  static const rclcpp::Logger logger = rclcpp::get_logger("autoware_tensorrt_vad");
  return logger;
}

const rclcpp::Clock::SharedPtr & vad_clock()
{
  static const rclcpp::Clock::SharedPtr clock = rclcpp::Clock::make_shared();
  return clock;
}

std::optional<cv::Mat> to_bgr_image(
  const sensor_msgs::msg::Image::ConstSharedPtr & image_msg, const int32_t camera_idx)
{
  if (image_msg->encoding == "bgr8") {
    return cv::Mat(
      image_msg->height, image_msg->width, CV_8UC3, const_cast<uint8_t *>(image_msg->data.data()),
      image_msg->step);
  }

  if (image_msg->encoding == "bgra8") {
    cv::Mat bgra_img(
      image_msg->height, image_msg->width, CV_8UC4, const_cast<uint8_t *>(image_msg->data.data()),
      image_msg->step);
    cv::Mat bgr_img;
    cv::cvtColor(bgra_img, bgr_img, cv::COLOR_BGRA2BGR);
    return bgr_img;
  }

  RCLCPP_ERROR_THROTTLE(
    vad_logger(), *vad_clock(), 5000, "Unsupported image encoding: %s for camera %d",
    image_msg->encoding.c_str(), camera_idx);
  return std::nullopt;
}

CoreInputFrame to_core_frame(const VadInputTopicData & vad_input_topic_data)
{
  CoreInputFrame frame;
  frame.stamp_sec = vad_input_topic_data.stamp.seconds();

  const int32_t num_cameras = static_cast<int32_t>(vad_input_topic_data.images.size());
  frame.camera_images.resize(num_cameras);
  frame.camera_infos.resize(num_cameras);

  for (int32_t camera_idx = 0; camera_idx < num_cameras; ++camera_idx) {
    const auto & image_msg = vad_input_topic_data.images[camera_idx];
    if (image_msg) {
      const auto bgr_opt = to_bgr_image(image_msg, camera_idx);
      if (bgr_opt.has_value()) {
        frame.camera_images[camera_idx] = bgr_opt.value().clone();
      }
    }

    const auto & camera_info_msg = vad_input_topic_data.camera_infos[camera_idx];
    if (camera_info_msg) {
      CoreCameraInfo info;
      info.width = static_cast<int32_t>(camera_info_msg->width);
      info.height = static_cast<int32_t>(camera_info_msg->height);
      for (size_t i = 0; i < info.k.size(); ++i) {
        info.k[i] = camera_info_msg->k[i];
      }
      info.frame_id = camera_info_msg->header.frame_id;
      frame.camera_infos[camera_idx] = info;
    }
  }

  if (vad_input_topic_data.kinematic_state) {
    CoreKinematicState state;
    const auto & pose = vad_input_topic_data.kinematic_state->pose.pose;
    const auto & twist = vad_input_topic_data.kinematic_state->twist.twist;
    state.position = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
    state.orientation = Eigen::Quaterniond(
      pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    state.linear_velocity = Eigen::Vector3d(
      twist.linear.x, twist.linear.y, twist.linear.z);
    state.angular_velocity = Eigen::Vector3d(
      twist.angular.x, twist.angular.y, twist.angular.z);
    frame.kinematic_state = state;
  }

  if (vad_input_topic_data.acceleration) {
    CoreAcceleration accel;
    const auto & acc = vad_input_topic_data.acceleration->accel.accel.linear;
    accel.linear = Eigen::Vector3d(acc.x, acc.y, acc.z);
    frame.acceleration = accel;
  }

  return frame;
}
}  // namespace

VadInterface::VadInterface(
  const VadInterfaceConfig & config, const std::shared_ptr<tf2_ros::Buffer> tf_buffer)
: config_(config)
{
  // Initialize coordinate transformer
  coordinate_transformer_ = std::make_unique<vad_interface::CoordinateTransformer>(tf_buffer);

  // Initialize core input converter
  input_converter_ = std::make_unique<VadInputConverter>(*coordinate_transformer_, config_);

  // Initialize output converters
  output_trajectory_converter_ =
    std::make_unique<vad_interface::OutputTrajectoryConverter>(*coordinate_transformer_, config_);
  output_map_converter_ =
    std::make_unique<vad_interface::OutputMapConverter>(*coordinate_transformer_, config_);
  output_objects_converter_ =
    std::make_unique<vad_interface::OutputObjectsConverter>(*coordinate_transformer_, config_);
}

VadInputData VadInterface::convert_input(const VadInputTopicData & vad_input_topic_data)
{
  const CoreInputFrame frame = to_core_frame(vad_input_topic_data);
  return input_converter_->convert(frame);
}

VadOutputTopicData VadInterface::convert_output(
  const VadOutputData & vad_output_data, const rclcpp::Time & stamp,
  const double trajectory_timestep, const Eigen::Matrix4d & base2map_transform) const
{
  VadOutputTopicData vad_output_topic_data;

  // Convert candidate trajectories using converter
  vad_output_topic_data.candidate_trajectories =
    output_trajectory_converter_->process_candidate_trajectories(
      vad_output_data.predicted_trajectories, stamp, trajectory_timestep, base2map_transform);

  // Convert trajectory using converter
  vad_output_topic_data.trajectory = output_trajectory_converter_->process_trajectory(
    vad_output_data.predicted_trajectory, stamp, trajectory_timestep, base2map_transform);

  // Convert map_points using converter
  vad_output_topic_data.map_points = output_map_converter_->process_map_points(
    vad_output_data.map_polylines, stamp, base2map_transform);

  // Convert predicted objects using converter
  vad_output_topic_data.objects = output_objects_converter_->process_predicted_objects(
    vad_output_data.predicted_objects, stamp, base2map_transform);

  return vad_output_topic_data;
}

}  // namespace autoware::tensorrt_vad
