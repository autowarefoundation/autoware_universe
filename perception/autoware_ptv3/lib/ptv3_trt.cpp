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

#include "autoware/ptv3/ptv3_trt.hpp"

#include "autoware/ptv3/preprocess/point_type.hpp"
#include "autoware/ptv3/preprocess/preprocess_kernel.hpp"
#include "autoware/ptv3/ptv3_config.hpp"

#include <autoware/cuda_utils/cuda_utils.hpp>
#include <autoware/point_types/memory.hpp>
#include <autoware/point_types/types.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_field.hpp>

#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::ptv3
{

PTv3TRT::PTv3TRT(const tensorrt_common::TrtCommonConfig & trt_config, const PTv3Config & config)
: config_(config)
{
  stop_watch_ptr_ = std::make_unique<autoware_utils::StopWatch<std::chrono::milliseconds>>();
  stop_watch_ptr_->tic("processing/inner");

  createPointFields();
  initPtr();
  initTrt(trt_config);

  CHECK_CUDA_ERROR(cudaStreamCreate(&stream_));
}

void PTv3TRT::setPublishSegmentedPointcloud(
  std::function<void(std::unique_ptr<const cuda_blackboard::CudaPointCloud2>)> func)
{
  publish_segmented_pointcloud_ = std::move(func);
}

void PTv3TRT::setPublishVisualizationPointcloud(
  std::function<void(std::unique_ptr<const cuda_blackboard::CudaPointCloud2>)> func)
{
  publish_visualization_pointcloud_ = std::move(func);
}

void PTv3TRT::setPublishFilteredPointcloud(
  std::function<void(std::unique_ptr<const cuda_blackboard::CudaPointCloud2>)> func)
{
  publish_filtered_pointcloud_ = std::move(func);
}

void PTv3TRT::allocateMessages()
{
  if (segmented_points_msg_ptr_ == nullptr) {
    segmented_points_msg_ptr_ = std::make_unique<cuda_blackboard::CudaPointCloud2>();
    segmented_points_msg_ptr_->height = 1;
    segmented_points_msg_ptr_->width = config_.max_num_voxels_;
    segmented_points_msg_ptr_->fields = segmented_pointcloud_fields_;
    segmented_points_msg_ptr_->is_bigendian = false;
    segmented_points_msg_ptr_->is_dense = true;
    segmented_points_msg_ptr_->point_step = 17U;
    segmented_points_msg_ptr_->data = cuda_blackboard::make_unique<std::uint8_t[]>(
      config_.max_num_voxels_ * segmented_points_msg_ptr_->point_step);
  }

  if (visualization_points_msg_ptr_ == nullptr) {
    visualization_points_msg_ptr_ = std::make_unique<cuda_blackboard::CudaPointCloud2>();
    visualization_points_msg_ptr_->height = 1;
    visualization_points_msg_ptr_->width = config_.max_num_voxels_;
    visualization_points_msg_ptr_->fields = visualization_pointcloud_fields_;
    visualization_points_msg_ptr_->is_bigendian = false;
    visualization_points_msg_ptr_->is_dense = true;
    visualization_points_msg_ptr_->point_step =
      static_cast<std::uint32_t>(visualization_pointcloud_fields_.size() * sizeof(float));
    visualization_points_msg_ptr_->data = cuda_blackboard::make_unique<std::uint8_t[]>(
      config_.max_num_voxels_ * visualization_points_msg_ptr_->point_step);
  }

  if (filtered_points_msg_ptr_ == nullptr && filtered_output_format_ != CloudFormat::UNKNOWN) {
    filtered_points_msg_ptr_ = std::make_unique<cuda_blackboard::CudaPointCloud2>();
    filtered_points_msg_ptr_->height = 1;
    filtered_points_msg_ptr_->width = config_.max_num_voxels_;
    filtered_points_msg_ptr_->fields = filtered_pointcloud_fields_;
    filtered_points_msg_ptr_->is_bigendian = false;
    filtered_points_msg_ptr_->is_dense = true;
    filtered_points_msg_ptr_->point_step =
      static_cast<std::uint32_t>(get_point_step(filtered_output_format_));
    filtered_points_msg_ptr_->data = cuda_blackboard::make_unique<std::uint8_t[]>(
      config_.max_num_voxels_ * filtered_points_msg_ptr_->point_step);
  }
}

PTv3TRT::~PTv3TRT()
{
  if (stream_) {
    CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));
    CHECK_CUDA_ERROR(cudaStreamDestroy(stream_));
  }
}

void PTv3TRT::initPtr()
{
  grid_coord_d_ = autoware::cuda_utils::make_unique<std::int64_t[]>(config_.max_num_voxels_ * 3);
  feat_d_ = autoware::cuda_utils::make_unique<float[]>(config_.max_num_voxels_ * 4);
  serialized_code_d_ =
    autoware::cuda_utils::make_unique<std::int64_t[]>(config_.max_num_voxels_ * 2);
  pred_labels_d_ = autoware::cuda_utils::make_unique<std::int64_t[]>(config_.max_num_voxels_);
  pred_probs_d_ = autoware::cuda_utils::make_unique<float[]>(
    config_.max_num_voxels_ * config_.class_names_.size());
  compact_points_d_ = autoware::cuda_utils::make_unique<std::uint8_t[]>(
    config_.max_num_voxels_ * sizeof(CloudPointTypeXYZIRCAEDT));

  pre_ptr_ = std::make_unique<PreprocessCuda>(config_, stream_);
  post_ptr_ = std::make_unique<PostprocessCuda>(config_, stream_);

  allocateMessages();
}

void PTv3TRT::createPointFields()
{
  auto make_point_field = [](const std::string & name, int offset, int datatype, int count) {
    sensor_msgs::msg::PointField field;
    field.name = name;
    field.offset = offset;
    field.datatype = datatype;
    field.count = count;
    return field;
  };

  segmented_pointcloud_fields_.push_back(
    make_point_field("x", 0, sensor_msgs::msg::PointField::FLOAT32, 1));
  segmented_pointcloud_fields_.push_back(
    make_point_field("y", 4, sensor_msgs::msg::PointField::FLOAT32, 1));
  segmented_pointcloud_fields_.push_back(
    make_point_field("z", 8, sensor_msgs::msg::PointField::FLOAT32, 1));
  segmented_pointcloud_fields_.push_back(
    make_point_field("class_id", 12, sensor_msgs::msg::PointField::UINT8, 1));
  segmented_pointcloud_fields_.push_back(
    make_point_field("probability", 13, sensor_msgs::msg::PointField::FLOAT32, 1));

  visualization_pointcloud_fields_.push_back(
    make_point_field("x", 0, sensor_msgs::msg::PointField::FLOAT32, 1));
  visualization_pointcloud_fields_.push_back(
    make_point_field("y", 4, sensor_msgs::msg::PointField::FLOAT32, 1));
  visualization_pointcloud_fields_.push_back(
    make_point_field("z", 8, sensor_msgs::msg::PointField::FLOAT32, 1));
  visualization_pointcloud_fields_.push_back(
    make_point_field("rgb", 12, sensor_msgs::msg::PointField::FLOAT32, 1));
}

void PTv3TRT::initTrt(const tensorrt_common::TrtCommonConfig & trt_config)
{
  std::vector<autoware::tensorrt_common::NetworkIO> network_io;

  // Inputs
  network_io.emplace_back("grid_coord", nvinfer1::Dims{2, {-1, 3}});
  network_io.emplace_back("feat", nvinfer1::Dims{2, {-1, 4}});
  network_io.emplace_back("serialized_code", nvinfer1::Dims{2, {2, -1}});

  // Outputs
  network_io.emplace_back("pred_labels", nvinfer1::Dims{1, {-1}});
  network_io.emplace_back(
    "pred_probs", nvinfer1::Dims{2, {-1, static_cast<std::int64_t>(config_.class_names_.size())}});

  std::vector<autoware::tensorrt_common::ProfileDims> profile_dims;

  profile_dims.emplace_back(
    "grid_coord", nvinfer1::Dims{2, {config_.voxels_num_[0], 3}},
    nvinfer1::Dims{2, {config_.voxels_num_[1], 3}}, nvinfer1::Dims{2, {config_.voxels_num_[2], 3}});

  profile_dims.emplace_back(
    "feat", nvinfer1::Dims{2, {config_.voxels_num_[0], 4}},
    nvinfer1::Dims{2, {config_.voxels_num_[1], 4}}, nvinfer1::Dims{2, {config_.voxels_num_[2], 4}});

  profile_dims.emplace_back(
    "serialized_code", nvinfer1::Dims{2, {2, config_.voxels_num_[0]}},
    nvinfer1::Dims{2, {2, config_.voxels_num_[1]}}, nvinfer1::Dims{2, {2, config_.voxels_num_[2]}});

  auto network_io_ptr =
    std::make_unique<std::vector<autoware::tensorrt_common::NetworkIO>>(network_io);
  auto profile_dims_ptr =
    std::make_unique<std::vector<autoware::tensorrt_common::ProfileDims>>(profile_dims);

  network_trt_ptr_ = std::make_unique<autoware::tensorrt_common::TrtCommon>(
    trt_config, std::make_shared<autoware::tensorrt_common::Profiler>(),
    std::vector<std::string>{config_.plugins_path_});

  if (!network_trt_ptr_->setup(std::move(profile_dims_ptr), std::move(network_io_ptr))) {
    throw std::runtime_error("Failed to setup TRT engine." + config_.plugins_path_);
  }

  network_trt_ptr_->setTensorAddress("grid_coord", grid_coord_d_.get());
  network_trt_ptr_->setTensorAddress("feat", feat_d_.get());
  network_trt_ptr_->setTensorAddress("serialized_code", serialized_code_d_.get());
  network_trt_ptr_->setTensorAddress("pred_labels", pred_labels_d_.get());
  network_trt_ptr_->setTensorAddress("pred_probs", pred_probs_d_.get());
}

CloudFormat PTv3TRT::detectCloudFormat(const cuda_blackboard::CudaPointCloud2 & cloud) const
{
  const auto & fields = cloud.fields;
  const auto num_fields = fields.size();

  if (num_fields == 10 && point_types::is_data_layout_compatible_with_point_xyzircaedt(fields)) {
    return CloudFormat::XYZIRCAEDT;
  }
  if (num_fields == 9 && point_types::is_data_layout_compatible_with_point_xyziradrt(fields)) {
    return CloudFormat::XYZIRADRT;
  }
  if (num_fields == 6 && point_types::is_data_layout_compatible_with_point_xyzirc(fields)) {
    return CloudFormat::XYZIRC;
  }
  if (num_fields == 4 && point_types::is_data_layout_compatible_with_point_xyzi(fields)) {
    return CloudFormat::XYZI;
  }

  return CloudFormat::UNKNOWN;
}

bool PTv3TRT::segment(
  const std::shared_ptr<const cuda_blackboard::CudaPointCloud2> & msg_ptr,
  bool should_publish_segmented_pointcloud, bool should_publish_visualization_pointcloud,
  bool should_publish_filtered_pointcloud, std::unordered_map<std::string, double> & proc_timing)
{
  stop_watch_ptr_->toc("processing/inner", true);
  if (!preProcess(msg_ptr)) {
    RCLCPP_ERROR(rclcpp::get_logger("ptv3"), "Pre-process failed. Skipping detection.");
    return false;
  }

  proc_timing.emplace(
    "debug/processing_time/preprocess_ms", stop_watch_ptr_->toc("processing/inner", true));

  if (!inference()) {
    RCLCPP_ERROR(rclcpp::get_logger("ptv3"), "Inference failed. Skipping detection.");
    return false;
  }

  proc_timing.emplace(
    "debug/processing_time/inference_ms", stop_watch_ptr_->toc("processing/inner", true));

  if (!postProcess(
        msg_ptr->header, should_publish_segmented_pointcloud,
        should_publish_visualization_pointcloud, should_publish_filtered_pointcloud)) {
    RCLCPP_ERROR(rclcpp::get_logger("ptv3"), "Post-process failed. Skipping detection");
    return false;
  }
  proc_timing.emplace(
    "debug/processing_time/postprocess_ms", stop_watch_ptr_->toc("processing/inner", true));

  return true;
}

bool PTv3TRT::preProcess(const std::shared_ptr<const cuda_blackboard::CudaPointCloud2> & msg_ptr)
{
  using autoware::cuda_utils::clear_async;

  std::call_once(init_cloud_, [this, &msg_ptr]() {
    input_format_ = detectCloudFormat(*msg_ptr);
    if (input_format_ == CloudFormat::UNKNOWN) {
      throw std::runtime_error(
        "Unsupported point cloud type. Expected one of: XYZIRCAEDT (10 fields), "
        "XYZIRADRT (9 fields), XYZIRC (6 fields), or XYZI (4 fields).");
    }

    const auto requested_output_format = parse_cloud_format_string(config_.filter_output_format_);
    filtered_output_format_ =
      config_.filter_output_format_.empty() ? input_format_ : requested_output_format;
    if (
      filtered_output_format_ == CloudFormat::UNKNOWN ||
      !can_convert_format(input_format_, filtered_output_format_)) {
      throw std::runtime_error(
        "filter.output_format='" + config_.filter_output_format_ +
        "' is not compatible with input format '" + std::string(to_string(input_format_)) + "'.");
    }

    auto append_field = [this](const std::string & name, int offset, int datatype, int count) {
      sensor_msgs::msg::PointField field;
      field.name = name;
      field.offset = offset;
      field.datatype = datatype;
      field.count = count;
      filtered_pointcloud_fields_.push_back(field);
    };
    filtered_pointcloud_fields_.clear();
    switch (filtered_output_format_) {
      case CloudFormat::XYZIRCAEDT:
        append_field("x", 0, sensor_msgs::msg::PointField::FLOAT32, 1);
        append_field("y", 4, sensor_msgs::msg::PointField::FLOAT32, 1);
        append_field("z", 8, sensor_msgs::msg::PointField::FLOAT32, 1);
        append_field("intensity", 12, sensor_msgs::msg::PointField::UINT8, 1);
        append_field("return_type", 13, sensor_msgs::msg::PointField::UINT8, 1);
        append_field("channel", 14, sensor_msgs::msg::PointField::UINT16, 1);
        append_field("azimuth", 16, sensor_msgs::msg::PointField::FLOAT32, 1);
        append_field("elevation", 20, sensor_msgs::msg::PointField::FLOAT32, 1);
        append_field("distance", 24, sensor_msgs::msg::PointField::FLOAT32, 1);
        append_field("time_stamp", 28, sensor_msgs::msg::PointField::UINT32, 1);
        break;
      case CloudFormat::XYZIRADRT:
        append_field("x", 0, sensor_msgs::msg::PointField::FLOAT32, 1);
        append_field("y", 4, sensor_msgs::msg::PointField::FLOAT32, 1);
        append_field("z", 8, sensor_msgs::msg::PointField::FLOAT32, 1);
        append_field("intensity", 12, sensor_msgs::msg::PointField::FLOAT32, 1);
        append_field("ring", 16, sensor_msgs::msg::PointField::UINT16, 1);
        append_field("azimuth", 18, sensor_msgs::msg::PointField::FLOAT32, 1);
        append_field("distance", 22, sensor_msgs::msg::PointField::FLOAT32, 1);
        append_field("return_type", 26, sensor_msgs::msg::PointField::UINT8, 1);
        append_field("time_stamp", 27, sensor_msgs::msg::PointField::FLOAT64, 1);
        break;
      case CloudFormat::XYZIRC:
        append_field("x", 0, sensor_msgs::msg::PointField::FLOAT32, 1);
        append_field("y", 4, sensor_msgs::msg::PointField::FLOAT32, 1);
        append_field("z", 8, sensor_msgs::msg::PointField::FLOAT32, 1);
        append_field("intensity", 12, sensor_msgs::msg::PointField::UINT8, 1);
        append_field("return_type", 13, sensor_msgs::msg::PointField::UINT8, 1);
        append_field("channel", 14, sensor_msgs::msg::PointField::UINT16, 1);
        break;
      case CloudFormat::XYZI:
        append_field("x", 0, sensor_msgs::msg::PointField::FLOAT32, 1);
        append_field("y", 4, sensor_msgs::msg::PointField::FLOAT32, 1);
        append_field("z", 8, sensor_msgs::msg::PointField::FLOAT32, 1);
        append_field("intensity", 12, sensor_msgs::msg::PointField::FLOAT32, 1);
        break;
      default:
        throw std::runtime_error("Unsupported filtered point cloud format.");
    }

    RCLCPP_INFO(
      rclcpp::get_logger("ptv3"),
      "Detected input format with %zu fields and point step %zu bytes; filtered output format '%s'",
      get_num_fields(input_format_), get_point_step(input_format_),
      to_string(filtered_output_format_));
  });
  allocateMessages();

  const auto num_points = msg_ptr->height * msg_ptr->width;

  if (num_points == 0) {
    RCLCPP_ERROR(rclcpp::get_logger("ptv3"), "Empty pointcloud. Skipping segmentation.");
    return false;
  }

  num_voxels_ = pre_ptr_->generateFeatures(
    msg_ptr->data.get(), input_format_, num_points, feat_d_.get(), grid_coord_d_.get(),
    serialized_code_d_.get(), compact_points_d_.get());

  if (num_voxels_ < config_.min_num_voxels_) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("ptv3"), "Too few voxels (" << num_voxels_
                                                     << ") for the actual optimization profile ("
                                                     << config_.min_num_voxels_ << ")");
    return false;
  }
  if (num_voxels_ > config_.max_num_voxels_) {
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("ptv3"), "Actual number of voxels ("
                                    << num_voxels_
                                    << ") is over the limit for the actual optimization profile ("
                                    << config_.max_num_voxels_ << "). Clipping to the limit.");
    num_voxels_ = config_.max_num_voxels_;
  }

  network_trt_ptr_->setInputShape("grid_coord", nvinfer1::Dims{2, {num_voxels_, 3}});
  network_trt_ptr_->setInputShape("feat", nvinfer1::Dims{2, {num_voxels_, 4}});
  network_trt_ptr_->setInputShape("serialized_code", nvinfer1::Dims{2, {2, num_voxels_}});

  return true;
}

bool PTv3TRT::inference()
{
  auto status = network_trt_ptr_->enqueueV3(stream_);
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  if (!status) {
    RCLCPP_ERROR(rclcpp::get_logger("ptv3"), "Fail to enqueue and skip to detect.");
    return false;
  }

  return true;
}

bool PTv3TRT::postProcess(
  const std_msgs::msg::Header & header, bool should_publish_segmented_pointcloud,
  bool should_publish_visualization_pointcloud, bool should_publish_filtered_pointcloud)
{
  // Segmentation pointcloud
  if (should_publish_segmented_pointcloud) {
    post_ptr_->createSegmentationPointcloud(
      feat_d_.get(), pred_labels_d_.get(), pred_probs_d_.get(),
      segmented_points_msg_ptr_->data.get(), config_.class_names_.size(), num_voxels_);
    CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

    segmented_points_msg_ptr_->header = header;
    segmented_points_msg_ptr_->width = num_voxels_;
    publish_segmented_pointcloud_(std::move(segmented_points_msg_ptr_));
    segmented_points_msg_ptr_ = nullptr;
  }

  // Visualization pointcloud
  if (should_publish_visualization_pointcloud) {
    post_ptr_->createVisualizationPointcloud(
      feat_d_.get(), pred_labels_d_.get(),
      reinterpret_cast<float *>(visualization_points_msg_ptr_->data.get()), num_voxels_);
    CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));
    visualization_points_msg_ptr_->header = header;
    visualization_points_msg_ptr_->width = num_voxels_;
    publish_visualization_pointcloud_(std::move(visualization_points_msg_ptr_));
    visualization_points_msg_ptr_ = nullptr;
  }

  if (should_publish_filtered_pointcloud) {
    const auto num_filtered_points = post_ptr_->createFilteredPointcloud(
      compact_points_d_.get(), input_format_, filtered_output_format_, pred_probs_d_.get(),
      filtered_points_msg_ptr_->data.get(), config_.class_names_.size(), num_voxels_);
    CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

    filtered_points_msg_ptr_->header = header;
    filtered_points_msg_ptr_->width = num_filtered_points;
    publish_filtered_pointcloud_(std::move(filtered_points_msg_ptr_));
    filtered_points_msg_ptr_ = nullptr;
  }

  allocateMessages();
  return true;
}

}  //  namespace autoware::ptv3
