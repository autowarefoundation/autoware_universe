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
#include "autoware/cuda_pointcloud_preprocessor/queue_bounds.hpp"
#include "autoware/pointcloud_preprocessor/diagnostics/crop_box_diagnostics.hpp"
#include "autoware/pointcloud_preprocessor/diagnostics/diagnostics_base.hpp"
#include "autoware/pointcloud_preprocessor/diagnostics/distortion_corrector_diagnostics.hpp"
#include "autoware/pointcloud_preprocessor/diagnostics/latency_diagnostics.hpp"
#include "autoware/pointcloud_preprocessor/diagnostics/pass_rate_diagnostics.hpp"

#include <autoware/agnocast_wrapper/autoware_agnocast_wrapper.hpp>
#include <autoware/cuda_utils/cuda_check_error.hpp>
#include <autoware/point_types/types.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cuda_runtime.h>
#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::cuda_pointcloud_preprocessor
{
using sensor_msgs::msg::PointCloud2;
namespace
{

std::size_t queue_size_from_frequency(const double max_frequency_hz)
{
  // TODO(mojomex): derive from a future pointcloud_frequency param instead of assuming 10 Hz.
  constexpr double assumed_pointcloud_frequency_hz = 10.0;
  if (max_frequency_hz <= 0.0) {
    throw std::runtime_error("Queue frequency parameters must be positive");
  }
  return static_cast<std::size_t>(std::ceil(max_frequency_hz / assumed_pointcloud_frequency_hz)) +
         1U;
}

class InputBoundsDiagnostics : public autoware::pointcloud_preprocessor::DiagnosticsBase
{
public:
  InputBoundsDiagnostics(
    const std::size_t input_points, const std::size_t max_input_points,
    const std::size_t truncated_points, const std::size_t twist_queue_size,
    const std::size_t max_twist_queue_size, const std::size_t dropped_twists,
    const std::size_t imu_queue_size, const std::size_t max_imu_queue_size,
    const std::size_t dropped_imus, const bool ring_overflow)
  : input_points_(input_points),
    max_input_points_(max_input_points),
    truncated_points_(truncated_points),
    twist_queue_size_(twist_queue_size),
    max_twist_queue_size_(max_twist_queue_size),
    dropped_twists_(dropped_twists),
    imu_queue_size_(imu_queue_size),
    max_imu_queue_size_(max_imu_queue_size),
    dropped_imus_(dropped_imus),
    ring_overflow_(ring_overflow)
  {
  }

  void add_to_interface(autoware_utils::DiagnosticsInterface & interface) const override
  {
    interface.add_key_value("Input point count", input_points_);
    interface.add_key_value("Maximum input point count", max_input_points_);
    interface.add_key_value("Truncated input point count", truncated_points_);
    interface.add_key_value("Twist queue size", twist_queue_size_);
    interface.add_key_value("Maximum twist queue size", max_twist_queue_size_);
    interface.add_key_value("Dropped twist count", dropped_twists_);
    interface.add_key_value("IMU queue size", imu_queue_size_);
    interface.add_key_value("Maximum IMU queue size", max_imu_queue_size_);
    interface.add_key_value("Dropped IMU count", dropped_imus_);
    interface.add_key_value("Ring organization overflow", ring_overflow_);
  }

  [[nodiscard]] std::optional<std::pair<int, std::string>> evaluate_status() const override
  {
    if (truncated_points_ > 0 || dropped_twists_ > 0 || dropped_imus_ > 0 || ring_overflow_) {
      return std::make_pair(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "[ERROR]: input bounds exceeded; truncated_points=" + std::to_string(truncated_points_) +
          ", dropped_twists=" + std::to_string(dropped_twists_) + ", dropped_imus=" +
          std::to_string(dropped_imus_) + ", ring_overflow=" + (ring_overflow_ ? "true" : "false"));
    }
    return std::nullopt;
  }

private:
  std::size_t input_points_;
  std::size_t max_input_points_;
  std::size_t truncated_points_;
  std::size_t twist_queue_size_;
  std::size_t max_twist_queue_size_;
  std::size_t dropped_twists_;
  std::size_t imu_queue_size_;
  std::size_t max_imu_queue_size_;
  std::size_t dropped_imus_;
  bool ring_overflow_;
};

}  // namespace

CudaPointcloudPreprocessorNode::CudaPointcloudPreprocessorNode(
  const rclcpp::NodeOptions & node_options)
: Node("cuda_pointcloud_preprocessor", node_options),
  tf2_buffer_(this->get_clock()),
  tf2_listener_(tf2_buffer_)
{
  using std::placeholders::_1;

  // Set CUDA device flags
  // note: Device flags are process-wide
  CHECK_CUDA_ERROR(cudaSetDeviceFlags(cudaDeviceScheduleBlockingSync));

  // Parameters
  base_frame_ = declare_parameter<std::string>("base_frame");
  use_3d_undistortion_ = declare_parameter<bool>("use_3d_distortion_correction");
  use_imu_ = declare_parameter<bool>("use_imu");
  bool enable_ring_outlier_filter = declare_parameter<bool>("enable_ring_outlier_filter");
  input_bounds_params_.max_input_point_count =
    static_cast<std::size_t>(declare_parameter<int64_t>("max_input_point_count"));
  input_bounds_params_.max_ring_count =
    static_cast<int>(declare_parameter<int64_t>("max_ring_count"));
  input_bounds_params_.max_points_per_ring =
    static_cast<int>(declare_parameter<int64_t>("max_points_per_ring"));
  const auto max_twist_frequency_hz = declare_parameter<double>("max_twist_frequency_hz");
  const auto max_imu_frequency_hz = declare_parameter<double>("max_imu_frequency_hz");
  input_bounds_params_.max_twist_subscriber_queue_size =
    queue_size_from_frequency(max_twist_frequency_hz);
  input_bounds_params_.max_imu_subscriber_queue_size =
    queue_size_from_frequency(max_imu_frequency_hz);
  // TODO(mojomex): equates to 1s of messages, should be made tighter in the future
  input_bounds_params_.max_twist_queue_size =
    static_cast<std::size_t>(std::ceil(max_twist_frequency_hz));
  input_bounds_params_.max_imu_queue_size =
    static_cast<std::size_t>(std::ceil(max_imu_frequency_hz));

  if (
    input_bounds_params_.max_input_point_count == 0 || input_bounds_params_.max_ring_count <= 0 ||
    input_bounds_params_.max_points_per_ring <= 0) {
    throw std::runtime_error("Pointcloud preprocessor capacity parameters must be positive");
  }

  RingOutlierFilterParameters ring_outlier_filter_parameters;
  ring_outlier_filter_parameters.distance_ratio =
    static_cast<float>(declare_parameter<double>("distance_ratio"));
  ring_outlier_filter_parameters.object_length_threshold =
    static_cast<float>(declare_parameter<double>("object_length_threshold"));

  processing_time_threshold_sec_ = declare_parameter<double>("processing_time_threshold_sec");
  timestamp_mismatch_fraction_threshold_ =
    declare_parameter<double>("timestamp_mismatch_fraction_threshold");

  const auto crop_box_min_x_vector = declare_parameter<std::vector<double>>("crop_box.min_x");
  const auto crop_box_min_y_vector = declare_parameter<std::vector<double>>("crop_box.min_y");
  const auto crop_box_min_z_vector = declare_parameter<std::vector<double>>("crop_box.min_z");

  const auto crop_box_max_x_vector = declare_parameter<std::vector<double>>("crop_box.max_x");
  const auto crop_box_max_y_vector = declare_parameter<std::vector<double>>("crop_box.max_y");
  const auto crop_box_max_z_vector = declare_parameter<std::vector<double>>("crop_box.max_z");
  const auto crop_box_negative_vector = declare_parameter<std::vector<bool>>("crop_box.negative");

  if (
    crop_box_min_x_vector.size() != crop_box_min_y_vector.size() ||
    crop_box_min_x_vector.size() != crop_box_min_z_vector.size() ||
    crop_box_min_x_vector.size() != crop_box_max_x_vector.size() ||
    crop_box_min_x_vector.size() != crop_box_max_y_vector.size() ||
    crop_box_min_x_vector.size() != crop_box_max_z_vector.size() ||
    crop_box_min_x_vector.size() != crop_box_negative_vector.size()) {
    throw std::runtime_error("Crop box parameters must have the same size");
  }

  std::vector<CropBoxParameters> crop_box_parameters;

  for (std::size_t i = 0; i < crop_box_min_x_vector.size(); i++) {
    CropBoxParameters parameters{};
    parameters.min_x = static_cast<float>(crop_box_min_x_vector.at(i));
    parameters.min_y = static_cast<float>(crop_box_min_y_vector.at(i));
    parameters.min_z = static_cast<float>(crop_box_min_z_vector.at(i));
    parameters.max_x = static_cast<float>(crop_box_max_x_vector.at(i));
    parameters.max_y = static_cast<float>(crop_box_max_y_vector.at(i));
    parameters.max_z = static_cast<float>(crop_box_max_z_vector.at(i));
    parameters.negative = static_cast<bool>(crop_box_negative_vector.at(i));
    crop_box_parameters.push_back(parameters);
  }

  // Publisher
  pub_ =
    std::make_unique<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>(
      *this, "~/output/pointcloud");
  diagnostics_interface_ =
    std::make_unique<autoware_utils::DiagnosticsInterface>(this, this->get_fully_qualified_name());

// Subscriber
#ifdef USE_AGNOCAST_ENABLED
  agnocast::SubscriptionOptions sub_options{};
#else
  rclcpp::SubscriptionOptions sub_options;
  sub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
#endif

  // Create mutually exclusive callback group for pointcloud subscription
  // Agnocast requires to be in a mutually exclusive callback group with no native ROS subscriptions
  pointcloud_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  sub_options.callback_group = pointcloud_callback_group_;

  // cppcheck-suppress unknownMacro
  pointcloud_sub_ = AUTOWARE_CREATE_SUBSCRIPTION(
    sensor_msgs::msg::PointCloud2, "~/input/pointcloud", rclcpp::SensorDataQoS{}.keep_last(1),
    std::bind(&CudaPointcloudPreprocessorNode::pointcloudCallback, this, std::placeholders::_1),
    sub_options);

  twist_sub_ = autoware_utils::InterProcessPollingSubscriber<
    geometry_msgs::msg::TwistWithCovarianceStamped, autoware_utils::polling_policy::All>::
    create_subscription(
      this, "~/input/twist", rclcpp::QoS(input_bounds_params_.max_twist_subscriber_queue_size));

  if (use_imu_) {
    imu_sub_ = autoware_utils::InterProcessPollingSubscriber<
      sensor_msgs::msg::Imu, autoware_utils::polling_policy::All>::
      create_subscription(
        this, "~/input/imu", rclcpp::QoS(input_bounds_params_.max_imu_subscriber_queue_size));
  }

  CudaPointcloudPreprocessor::UndistortionType undistortion_type =
    use_3d_undistortion_ ? CudaPointcloudPreprocessor::UndistortionType::Undistortion3D
                         : CudaPointcloudPreprocessor::UndistortionType::Undistortion2D;

  const PreprocessorCapacity preprocessor_capacity{
    input_bounds_params_.max_input_point_count, input_bounds_params_.max_ring_count,
    input_bounds_params_.max_points_per_ring,
    input_bounds_params_.max_twist_queue_size + input_bounds_params_.max_imu_queue_size};
  cuda_pointcloud_preprocessor_ =
    std::make_unique<CudaPointcloudPreprocessor>(preprocessor_capacity);
  cuda_pointcloud_preprocessor_->setRingOutlierFilterParameters(ring_outlier_filter_parameters);
  cuda_pointcloud_preprocessor_->setRingOutlierFilterActive(enable_ring_outlier_filter);
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

void CudaPointcloudPreprocessorNode::insertTwistMessage(
  const geometry_msgs::msg::TwistWithCovarianceStamped & twist_msg)
{
  detail::insert_sorted(twist_queue_, twist_msg);
}

void CudaPointcloudPreprocessorNode::insertImuMessage(const sensor_msgs::msg::Imu & imu_msg)
{
  tf2::Transform imu_to_base_tf2{};
  getTransform(base_frame_, imu_msg.header.frame_id, &imu_to_base_tf2);
  geometry_msgs::msg::TransformStamped imu_to_base_msg;
  imu_to_base_msg.transform.rotation = tf2::toMsg(imu_to_base_tf2.getRotation());

  geometry_msgs::msg::Vector3Stamped angular_velocity;
  angular_velocity.vector = imu_msg.angular_velocity;

  geometry_msgs::msg::Vector3Stamped transformed_angular_velocity;
  tf2::doTransform(angular_velocity, transformed_angular_velocity, imu_to_base_msg);
  transformed_angular_velocity.header = imu_msg.header;

  detail::insert_sorted(angular_velocity_queue_, transformed_angular_velocity);
}

void CudaPointcloudPreprocessorNode::pointcloudCallback(
  // cppcheck-suppress unknownMacro
  AUTOWARE_MESSAGE_UNIQUE_PTR(sensor_msgs::msg::PointCloud2) input_pointcloud_msg_ptr)
{
  const auto & input_pointcloud_msg = *input_pointcloud_msg_ptr;
  latest_input_bounds_status_ = {};
  latest_input_bounds_status_.input_point_count =
    static_cast<std::size_t>(input_pointcloud_msg.width) * input_pointcloud_msg.height;
  latest_input_bounds_status_.truncated_point_count =
    latest_input_bounds_status_.input_point_count > input_bounds_params_.max_input_point_count
      ? latest_input_bounds_status_.input_point_count - input_bounds_params_.max_input_point_count
      : 0U;

  if (!validatePointcloudLayout(input_pointcloud_msg)) {
    return;
  }

  stop_watch_ptr_->toc("processing_time", true);

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

[[nodiscard]] bool CudaPointcloudPreprocessorNode::validatePointcloudLayout(
  const sensor_msgs::msg::PointCloud2 & input_pointcloud_msg) const
{
  static_assert(
    sizeof(InputPointType) == sizeof(autoware::point_types::PointXYZIRCAEDT),
    "PointStruct and PointXYZIRCAEDT must have the same size");

  if (is_data_layout_compatible_with_point_xyzircaedt(input_pointcloud_msg.fields)) {
    return true;
  }

  RCLCPP_ERROR(get_logger(), "Input pointcloud data layout is not compatible with PointXYZIRCAEDT");
  return false;
}

/**
 * @brief Get the first point's timestamp information from a PointCloud2 message.
 *
 * This function extracts the relative timestamp (`time_stamp` field) of the first point in the
 * point cloud and calculates its absolute timestamp by adding it to the message's header timestamp.
 *
 * @param input_pointcloud_msg The input PointCloud2 message, which must include a "time_stamp"
 * field.
 * @return std::pair<std::uint64_t, std::uint32_t>
 *   - first: The absolute timestamp of the first point (in nanoseconds).
 *   - second: The relative timestamp of the first point (in nanoseconds).
 */
std::pair<std::uint64_t, std::uint32_t> CudaPointcloudPreprocessorNode::getFirstPointTimeInfo(
  const sensor_msgs::msg::PointCloud2 & input_pointcloud_msg)
{
  sensor_msgs::PointCloud2ConstIterator<std::uint32_t> iter_stamp(
    input_pointcloud_msg, "time_stamp");
  auto num_points = input_pointcloud_msg.width * input_pointcloud_msg.height;
  std::uint32_t first_point_rel_stamp = num_points > 0 ? *iter_stamp : 0;
  std::uint64_t first_point_stamp = input_pointcloud_msg.header.stamp.sec * 1e9 +
                                    input_pointcloud_msg.header.stamp.nanosec +
                                    first_point_rel_stamp;
  return {first_point_stamp, first_point_rel_stamp};
}

void CudaPointcloudPreprocessorNode::updateTwistQueue(std::uint64_t first_point_stamp)
{
  std::vector<geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr> twist_msgs =
    twist_sub_->take_data();

  latest_input_bounds_status_.dropped_twist_count += detail::prepare_queue_update(
    twist_queue_, twist_msgs, input_bounds_params_.max_twist_queue_size, first_point_stamp);
  for (const auto & msg : twist_msgs) {
    if (detail::is_backward_time_jump(twist_queue_, msg->header.stamp)) {
      twist_queue_.clear();
    }
    insertTwistMessage(*msg);
  }
}

void CudaPointcloudPreprocessorNode::updateImuQueue(std::uint64_t first_point_stamp)
{
  if (!imu_sub_) return;

  std::vector<sensor_msgs::msg::Imu::ConstSharedPtr> imu_msgs = imu_sub_->take_data();

  latest_input_bounds_status_.dropped_imu_count += detail::prepare_queue_update(
    angular_velocity_queue_, imu_msgs, input_bounds_params_.max_imu_queue_size, first_point_stamp);
  for (const auto & msg : imu_msgs) {
    if (detail::is_backward_time_jump(angular_velocity_queue_, msg->header.stamp)) {
      angular_velocity_queue_.clear();
    }
    insertImuMessage(*msg);
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
  auto input_bounds_diag = std::make_shared<InputBoundsDiagnostics>(
    latest_input_bounds_status_.input_point_count, input_bounds_params_.max_input_point_count,
    latest_input_bounds_status_.truncated_point_count, twist_queue_.size(),
    input_bounds_params_.max_twist_queue_size, latest_input_bounds_status_.dropped_twist_count,
    angular_velocity_queue_.size(), input_bounds_params_.max_imu_queue_size,
    latest_input_bounds_status_.dropped_imu_count, stats.ring_overflow);

  diagnostics_interface_->clear();

  std::vector<std::shared_ptr<const autoware::pointcloud_preprocessor::DiagnosticsBase>>
    diagnostics = {
      latency_diag, pass_rate_diag, crop_box_diag, distortion_corrector_diag, input_bounds_diag};

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
