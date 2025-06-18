// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/cuda_pointcloud_preprocessor/cuda_pointcloud_preprocessor_node.hpp"

#include "autoware/cuda_pointcloud_preprocessor/memory.hpp"
#include "autoware/cuda_pointcloud_preprocessor/point_types.hpp"
#include "autoware/pointcloud_preprocessor/diagnostics/crop_box_diagnostics.hpp"
#include "autoware/pointcloud_preprocessor/diagnostics/distortion_corrector_diagnostics.hpp"
#include "autoware/pointcloud_preprocessor/diagnostics/latency_diagnostics.hpp"
#include "autoware/pointcloud_preprocessor/diagnostics/pass_rate_diagnostics.hpp"

#include <autoware/point_types/types.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cuda_runtime.h>
#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <chrono>
#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::cuda_pointcloud_preprocessor
{
using sensor_msgs::msg::PointCloud2;

CudaPointcloudPreprocessorNode::CudaPointcloudPreprocessorNode(
  const rclcpp::NodeOptions & node_options)
: Node("cuda_pointcloud_preprocessor", node_options),
  tf2_buffer_(this->get_clock()),
  tf2_listener_(tf2_buffer_)
{
  using std::placeholders::_1;

  // Set CUDA device flags
  // note: Device flags are process-wide
  cudaSetDeviceFlags(cudaDeviceScheduleBlockingSync);

  // Parameters
  base_frame_ = declare_parameter<std::string>("base_frame");
  use_3d_undistortion_ = declare_parameter<bool>("use_3d_distortion_correction");
  use_imu_ = declare_parameter<bool>("use_imu");

  RingOutlierFilterParameters ring_outlier_filter_parameters;
  ring_outlier_filter_parameters.distance_ratio = declare_parameter<float>("distance_ratio");
  ring_outlier_filter_parameters.object_length_threshold =
    declare_parameter<float>("object_length_threshold");

  processing_time_threshold_sec_ = declare_parameter<double>("processing_time_threshold_sec");
  timestamp_mismatch_fraction_threshold_ =
    declare_parameter<double>("timestamp_mismatch_fraction_threshold");

  const auto crop_box_min_x_vector = declare_parameter<std::vector<double>>("crop_box.min_x");
  const auto crop_box_min_y_vector = declare_parameter<std::vector<double>>("crop_box.min_y");
  const auto crop_box_min_z_vector = declare_parameter<std::vector<double>>("crop_box.min_z");

  const auto crop_box_max_x_vector = declare_parameter<std::vector<double>>("crop_box.max_x");
  const auto crop_box_max_y_vector = declare_parameter<std::vector<double>>("crop_box.max_y");
  const auto crop_box_max_z_vector = declare_parameter<std::vector<double>>("crop_box.max_z");

  if (
    crop_box_min_x_vector.size() != crop_box_min_y_vector.size() ||
    crop_box_min_x_vector.size() != crop_box_min_z_vector.size() ||
    crop_box_min_x_vector.size() != crop_box_max_x_vector.size() ||
    crop_box_min_x_vector.size() != crop_box_max_y_vector.size() ||
    crop_box_min_x_vector.size() != crop_box_max_z_vector.size()) {
    throw std::runtime_error("Crop box parameters must have the same size");
  }

  std::vector<CropBoxParameters> crop_box_parameters;

  for (std::size_t i = 0; i < crop_box_min_x_vector.size(); i++) {
    CropBoxParameters parameters;
    parameters.min_x = crop_box_min_x_vector[i];
    parameters.min_y = crop_box_min_y_vector[i];
    parameters.min_z = crop_box_min_z_vector[i];
    parameters.max_x = crop_box_max_x_vector[i];
    parameters.max_y = crop_box_max_y_vector[i];
    parameters.max_z = crop_box_max_z_vector[i];
    crop_box_parameters.push_back(parameters);
  }

  // Publisher
  pub_ =
    std::make_unique<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>(
      *this, "~/output/pointcloud");
  diagnostics_interface_ =
    std::make_unique<autoware_utils::DiagnosticsInterface>(this, this->get_fully_qualified_name());

  // Subscriber
  rclcpp::SubscriptionOptions sub_options;
  sub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;

  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "~/input/pointcloud", rclcpp::SensorDataQoS{}.keep_last(1),
    std::bind(&CudaPointcloudPreprocessorNode::pointcloudCallback, this, std::placeholders::_1),
    sub_options);

  // Twist queue size needs to be larger than 'twist frequency' / 'pointcloud frequency'.
  // To avoid individual tuning, a sufficiently large value is hard-coded.
  // With 100, it can handle twist updates up to 1000Hz if the pointcloud is 10Hz.
  const uint16_t TWIST_QUEUE_SIZE = 100;
  twist_sub_ = autoware_utils::InterProcessPollingSubscriber<
    geometry_msgs::msg::TwistWithCovarianceStamped, autoware_utils::polling_policy::All>::
    create_subscription(this, "~/input/twist", rclcpp::QoS(TWIST_QUEUE_SIZE));

  if (use_imu_) {
    imu_sub_ = autoware_utils::InterProcessPollingSubscriber<
      sensor_msgs::msg::Imu, autoware_utils::polling_policy::All>::
      create_subscription(this, "~/input/imu", rclcpp::QoS(TWIST_QUEUE_SIZE));
  }

  CudaPointcloudPreprocessor::UndistortionType undistortion_type =
    use_3d_undistortion_ ? CudaPointcloudPreprocessor::UndistortionType::Undistortion3D
                         : CudaPointcloudPreprocessor::UndistortionType::Undistortion2D;

  cuda_pointcloud_preprocessor_ = std::make_unique<CudaPointcloudPreprocessor>();
  cuda_pointcloud_preprocessor_->setRingOutlierFilterParameters(ring_outlier_filter_parameters);
  cuda_pointcloud_preprocessor_->setCropBoxParameters(crop_box_parameters);
  cuda_pointcloud_preprocessor_->setUndistortionType(undistortion_type);

  // initialize debug tool
  {
    using autoware_utils::DebugPublisher;
    using autoware_utils::StopWatch;

    stop_watch_ptr_ = std::make_unique<StopWatch<std::chrono::milliseconds>>();
    debug_publisher_ = std::make_unique<DebugPublisher>(this, "cuda_pointcloud_preprocessor");
    stop_watch_ptr_->tic("processing_time");
  }
}

bool CudaPointcloudPreprocessorNode::getTransform(
  const std::string & target_frame, const std::string & source_frame,
  tf2::Transform * tf2_transform_ptr)
{
  if (target_frame == source_frame) {
    tf2_transform_ptr->setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
    tf2_transform_ptr->setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
    return true;
  }

  try {
    const auto transform_msg =
      tf2_buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
    tf2::convert(transform_msg.transform, *tf2_transform_ptr);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "%s", ex.what());
    RCLCPP_ERROR(
      get_logger(), "Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());

    tf2_transform_ptr->setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
    tf2_transform_ptr->setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
    return false;
  }
  return true;
}

void CudaPointcloudPreprocessorNode::twistCallback(
  const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr twist_msg_ptr)
{
  while (!twist_queue_.empty()) {
    // for replay rosbag
    bool backwards_time_jump_detected =
      rclcpp::Time(twist_queue_.front().header.stamp) > rclcpp::Time(twist_msg_ptr->header.stamp);
    bool is_queue_longer_than_1s =
      rclcpp::Time(twist_queue_.front().header.stamp) <
      rclcpp::Time(twist_msg_ptr->header.stamp) - rclcpp::Duration::from_seconds(1.0);

    if (backwards_time_jump_detected) {
      twist_queue_.clear();
    } else if (is_queue_longer_than_1s) {
      twist_queue_.pop_front();
    } else {
      break;
    }
  }

  auto it = std::lower_bound(
    twist_queue_.begin(), twist_queue_.end(), twist_msg_ptr->header.stamp,
    [](const auto & twist, const auto & stamp) {
      return rclcpp::Time(twist.header.stamp) < stamp;
    });
  twist_queue_.insert(it, *twist_msg_ptr);
}

void CudaPointcloudPreprocessorNode::imuCallback(
  const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg)
{
  while (!angular_velocity_queue_.empty()) {
    // for rosbag replay
    bool backwards_time_jump_detected = rclcpp::Time(angular_velocity_queue_.front().header.stamp) >
                                        rclcpp::Time(imu_msg->header.stamp);

    bool is_queue_longer_than_1s =
      rclcpp::Time(angular_velocity_queue_.front().header.stamp) <
      rclcpp::Time(imu_msg->header.stamp) - rclcpp::Duration::from_seconds(1.0);

    if (backwards_time_jump_detected) {
      angular_velocity_queue_.clear();
    } else if (is_queue_longer_than_1s) {
      angular_velocity_queue_.pop_front();
    } else {
      break;
    }
  }

  tf2::Transform imu_to_base_tf2{};
  getTransform(base_frame_, imu_msg->header.frame_id, &imu_to_base_tf2);
  geometry_msgs::msg::TransformStamped imu_to_base_msg;
  imu_to_base_msg.transform.rotation = tf2::toMsg(imu_to_base_tf2.getRotation());

  geometry_msgs::msg::Vector3Stamped angular_velocity;
  angular_velocity.vector = imu_msg->angular_velocity;

  geometry_msgs::msg::Vector3Stamped transformed_angular_velocity;
  tf2::doTransform(angular_velocity, transformed_angular_velocity, imu_to_base_msg);
  transformed_angular_velocity.header = imu_msg->header;

  auto it = std::lower_bound(
    angular_velocity_queue_.begin(), angular_velocity_queue_.end(), imu_msg->header.stamp,
    [](const auto & angular_velocity, const auto & stamp) {
      return rclcpp::Time(angular_velocity.header.stamp) < stamp;
    });
  angular_velocity_queue_.insert(it, transformed_angular_velocity);
}

void CudaPointcloudPreprocessorNode::pointcloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_pointcloud_msg_ptr)
{
  stop_watch_ptr_->toc("processing_time", true);
  const auto & input_pointcloud_msg = *input_pointcloud_msg_ptr;

  validatePointcloudLayout(input_pointcloud_msg);
  const auto [first_point_stamp, first_point_rel_stamp] =
    getFirstPointTimeInfo(input_pointcloud_msg);
  updateTwistQueue(first_point_stamp);
  updateImuQueue(first_point_stamp);

  const auto transform_msg_opt = lookupTransformToBase(input_pointcloud_msg.header.frame_id);
  if (!transform_msg_opt.has_value()) return;

  auto output_pointcloud_ptr =
    processPointcloud(input_pointcloud_msg, *transform_msg_opt, first_point_rel_stamp);
  output_pointcloud_ptr->header.frame_id = base_frame_;

  publishDiagnostics(input_pointcloud_msg, output_pointcloud_ptr);
  pub_->publish(std::move(output_pointcloud_ptr));

  // Preallocate buffer for the next run.
  // This is intentionally done after publish() to avoid adding latency to the current cycle.
  cuda_pointcloud_preprocessor_->preallocateOutput();
}

void CudaPointcloudPreprocessorNode::validatePointcloudLayout(
  const sensor_msgs::msg::PointCloud2 & input_pointcloud_msg)
{
  if (!is_data_layout_compatible_with_point_xyzircaedt(input_pointcloud_msg.fields)) {
    RCLCPP_ERROR(
      get_logger(), "Input pointcloud data layout is not compatible with PointXYZIRCAEDT");
  }
  static_assert(
    sizeof(InputPointType) == sizeof(autoware::point_types::PointXYZIRCAEDT),
    "PointStruct and PointXYZIRCAEDT must have the same size");
}

/**
 * @brief Get the first point's timestamp information from a PointCloud2 message.
 *
 * This function extracts the relative timestamp (`time_stamp` field) of the first point in the
 * point cloud and calculates its absolute timestamp by adding it to the message's header timestamp.
 *
 * @param input_pointcloud_msg The input PointCloud2 message, which must include a "time_stamp"
 * field.
 * @return std::pair<double, std::uint32_t>
 *   - first: The absolute timestamp of the first point (in seconds).
 *   - second: The relative timestamp of the first point (in nanoseconds).
 */
std::pair<double, std::uint32_t> CudaPointcloudPreprocessorNode::getFirstPointTimeInfo(
  const sensor_msgs::msg::PointCloud2 & input_pointcloud_msg)
{
  sensor_msgs::PointCloud2ConstIterator<std::uint32_t> iter_stamp(
    input_pointcloud_msg, "time_stamp");
  auto num_points = input_pointcloud_msg.width * input_pointcloud_msg.height;
  std::uint32_t first_point_rel_stamp = num_points > 0 ? *iter_stamp : 0;
  double first_point_stamp = input_pointcloud_msg.header.stamp.sec +
                             input_pointcloud_msg.header.stamp.nanosec * 1e-9 +
                             first_point_rel_stamp * 1e-9;
  return {first_point_stamp, first_point_rel_stamp};
}

void CudaPointcloudPreprocessorNode::updateTwistQueue(double first_point_stamp)
{
  std::vector<geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr> twist_msgs =
    twist_sub_->take_data();
  for (const auto & msg : twist_msgs) {
    twistCallback(msg);
  }
  while (twist_queue_.size() > 1 &&
         rclcpp::Time(twist_queue_.front().header.stamp).seconds() < first_point_stamp) {
    twist_queue_.pop_front();
  }
}

void CudaPointcloudPreprocessorNode::updateImuQueue(double first_point_stamp)
{
  if (!imu_sub_) return;

  std::vector<sensor_msgs::msg::Imu::ConstSharedPtr> imu_msgs = imu_sub_->take_data();
  for (const auto & msg : imu_msgs) {
    imuCallback(msg);
  }
  while (angular_velocity_queue_.size() > 1 &&
         rclcpp::Time(angular_velocity_queue_.front().header.stamp).seconds() < first_point_stamp) {
    angular_velocity_queue_.pop_front();
  }
}

std::optional<geometry_msgs::msg::TransformStamped>
CudaPointcloudPreprocessorNode::lookupTransformToBase(const std::string & source_frame)
{
  try {
    return tf2_buffer_.lookupTransform(base_frame_, source_frame, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "%s", ex.what());
    return std::nullopt;
  }
}

std::unique_ptr<cuda_blackboard::CudaPointCloud2> CudaPointcloudPreprocessorNode::processPointcloud(
  const sensor_msgs::msg::PointCloud2 & input_pointcloud_msg,
  const geometry_msgs::msg::TransformStamped & transform_msg,
  const std::uint32_t first_point_rel_stamp)
{
  return cuda_pointcloud_preprocessor_->process(
    input_pointcloud_msg, transform_msg, twist_queue_, angular_velocity_queue_,
    first_point_rel_stamp);
}

void CudaPointcloudPreprocessorNode::publishDiagnostics(
  const sensor_msgs::msg::PointCloud2 & input_pointcloud_msg,
  const std::unique_ptr<cuda_blackboard::CudaPointCloud2> & output_pointcloud_ptr)
{
  const auto processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
  const auto pipeline_latency_ms =
    std::chrono::duration<double, std::milli>(
      std::chrono::nanoseconds(
        (this->get_clock()->now() - input_pointcloud_msg.header.stamp).nanoseconds()))
      .count();

  const auto input_point_count =
    static_cast<int>(input_pointcloud_msg.width * input_pointcloud_msg.height);
  const auto output_point_count =
    static_cast<int>(output_pointcloud_ptr->width * output_pointcloud_ptr->height);
  const auto stats = cuda_pointcloud_preprocessor_->getProcessingStats();

  const auto skipped_nan_count = stats.num_nan_points;
  const auto mismatch_count = stats.mismatch_count;
  const auto num_undistorted_points = stats.num_crop_box_passed_points;

  const auto mismatch_fraction =
    num_undistorted_points > 0
      ? static_cast<float>(stats.mismatch_count) / static_cast<float>(num_undistorted_points)
      : 0.0f;

  auto latency_diag = std::make_shared<autoware::pointcloud_preprocessor::LatencyDiagnostics>(
    input_pointcloud_msg.header.stamp, processing_time_ms, pipeline_latency_ms,
    processing_time_threshold_sec_ * 1000.0);
  auto pass_rate_diag = std::make_shared<autoware::pointcloud_preprocessor::PassRateDiagnostics>(
    input_point_count, output_point_count);

  auto distortion_corrector_diag =
    std::make_shared<autoware::pointcloud_preprocessor::DistortionCorrectorDiagnostics>(
      mismatch_count, mismatch_fraction, use_3d_undistortion_, false,
      timestamp_mismatch_fraction_threshold_);

  auto crop_box_diag =
    std::make_shared<autoware::pointcloud_preprocessor::CropBoxDiagnostics>(skipped_nan_count);

  diagnostics_interface_->clear();

  std::vector<std::shared_ptr<const autoware::pointcloud_preprocessor::DiagnosticsBase>>
    diagnostics = {latency_diag, pass_rate_diag, crop_box_diag, distortion_corrector_diag};

  int worst_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string message;

  for (const auto & diag : diagnostics) {
    diag->add_to_interface(*diagnostics_interface_);
    if (const auto status = diag->evaluate_status(); status.has_value()) {
      worst_level = std::max(worst_level, status->first);
      if (!message.empty()) {
        message += " / ";
      }
      message += status->second;
    }
  }
  if (message.empty()) {
    message = "CudaPointcloudPreprocessor operating normally";
  }
  diagnostics_interface_->update_level_and_message(static_cast<int8_t>(worst_level), message);
  diagnostics_interface_->publish(this->get_clock()->now());

  debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "debug/processing_time_ms", processing_time_ms);
  debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "debug/latency_ms", pipeline_latency_ms);
}

}  // namespace autoware::cuda_pointcloud_preprocessor

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::cuda_pointcloud_preprocessor::CudaPointcloudPreprocessorNode)
