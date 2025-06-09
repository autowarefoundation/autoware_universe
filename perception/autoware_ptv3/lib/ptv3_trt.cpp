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

void PTv3TRT::setPublishGroundSegmentedPointcloud(
  std::function<void(std::unique_ptr<const cuda_blackboard::CudaPointCloud2>)> func)
{
  publish_ground_segmented_pointcloud_ = std::move(func);
}

// cSpell:ignore Probs probs
void PTv3TRT::setPublishProbsPointcloud(
  std::function<void(std::unique_ptr<const cuda_blackboard::CudaPointCloud2>)> func)
{
  publish_probs_pointcloud_ = std::move(func);
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
    segmented_points_msg_ptr_->point_step =
      static_cast<std::uint32_t>(segmented_pointcloud_fields_.size() * sizeof(float));
    segmented_points_msg_ptr_->data = cuda_blackboard::make_unique<std::uint8_t[]>(
      config_.max_num_voxels_ * segmented_points_msg_ptr_->point_step);
  }

  if (ground_segmented_points_msg_ptr_ == nullptr) {
    ground_segmented_points_msg_ptr_ = std::make_unique<cuda_blackboard::CudaPointCloud2>();
    ground_segmented_points_msg_ptr_->height = 1;
    ground_segmented_points_msg_ptr_->width = config_.max_num_voxels_;
    ground_segmented_points_msg_ptr_->fields = ground_segmented_pointcloud_fields_;
    ground_segmented_points_msg_ptr_->is_bigendian = false;
    ground_segmented_points_msg_ptr_->is_dense = true;
    ground_segmented_points_msg_ptr_->point_step =
      static_cast<std::uint32_t>(ground_segmented_pointcloud_fields_.size() * sizeof(float));
    ground_segmented_points_msg_ptr_->data = cuda_blackboard::make_unique<std::uint8_t[]>(
      config_.max_num_voxels_ * ground_segmented_points_msg_ptr_->point_step);
  }

  if (probs_points_msg_ptr_ == nullptr) {
    probs_points_msg_ptr_ = std::make_unique<cuda_blackboard::CudaPointCloud2>();
    probs_points_msg_ptr_->height = 1;
    probs_points_msg_ptr_->width = config_.max_num_voxels_;
    probs_points_msg_ptr_->fields = probs_pointcloud_fields_;
    probs_points_msg_ptr_->is_bigendian = false;
    probs_points_msg_ptr_->is_dense = false;
    probs_points_msg_ptr_->point_step =
      static_cast<std::uint32_t>(probs_pointcloud_fields_.size() * sizeof(float));
    probs_points_msg_ptr_->data = cuda_blackboard::make_unique<std::uint8_t[]>(
      config_.max_num_voxels_ * probs_points_msg_ptr_->point_step);
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
    make_point_field("rgb", 12, sensor_msgs::msg::PointField::FLOAT32, 1));

  ground_segmented_pointcloud_fields_.push_back(
    make_point_field("x", 0, sensor_msgs::msg::PointField::FLOAT32, 1));
  ground_segmented_pointcloud_fields_.push_back(
    make_point_field("y", 4, sensor_msgs::msg::PointField::FLOAT32, 1));
  ground_segmented_pointcloud_fields_.push_back(
    make_point_field("z", 8, sensor_msgs::msg::PointField::FLOAT32, 1));
  ground_segmented_pointcloud_fields_.push_back(
    make_point_field("intensity", 12, sensor_msgs::msg::PointField::FLOAT32, 1));

  probs_pointcloud_fields_.push_back(
    make_point_field("x", 0, sensor_msgs::msg::PointField::FLOAT32, 1));
  probs_pointcloud_fields_.push_back(
    make_point_field("y", 4, sensor_msgs::msg::PointField::FLOAT32, 1));
  probs_pointcloud_fields_.push_back(
    make_point_field("z", 8, sensor_msgs::msg::PointField::FLOAT32, 1));

  for (std::size_t i = 0; i < config_.class_names_.size(); ++i) {
    std::string class_name = config_.class_names_[i];
    std::transform(class_name.begin(), class_name.end(), class_name.begin(), [](unsigned char c) {
      return std::tolower(c);
    });

    probs_pointcloud_fields_.push_back(
      make_point_field("prob_" + class_name, 12 + i * 4, sensor_msgs::msg::PointField::FLOAT32, 1));
  }
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

bool PTv3TRT::segment(
  const std::shared_ptr<const cuda_blackboard::CudaPointCloud2> & msg_ptr,
  bool should_publish_segmented_pointcloud, bool should_publish_ground_segmented_pointcloud,
  bool should_publish_probs_pointcloud, std::unordered_map<std::string, double> & proc_timing)
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
        should_publish_ground_segmented_pointcloud, should_publish_probs_pointcloud)) {
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

  const auto num_points = msg_ptr->height * msg_ptr->width;

  if (num_points == 0) {
    RCLCPP_ERROR(rclcpp::get_logger("ptv3"), "Empty pointcloud. Skipping segmentation.");
    return false;
  }

  num_voxels_ = pre_ptr_->generateFeatures(
    reinterpret_cast<InputPointType *>(msg_ptr->data.get()), num_points, feat_d_.get(),
    grid_coord_d_.get(), serialized_code_d_.get());

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
  bool should_publish_ground_segmented_pointcloud, bool should_publish_probs_pointcloud)
{
  // Painted pointcloud
  if (should_publish_segmented_pointcloud) {
    post_ptr_->paintPointcloud(
      feat_d_.get(), pred_labels_d_.get(),
      reinterpret_cast<float *>(segmented_points_msg_ptr_->data.get()), num_voxels_);
    CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

    segmented_points_msg_ptr_->header = header;
    segmented_points_msg_ptr_->width = num_voxels_;
    publish_segmented_pointcloud_(std::move(segmented_points_msg_ptr_));
    segmented_points_msg_ptr_ = nullptr;
  }

  // Ground segmented pointcloud
  if (should_publish_ground_segmented_pointcloud) {
    const auto num_non_ground_points = post_ptr_->createGroundSegmentedPointcloud(
      feat_d_.get(), pred_labels_d_.get(), pred_probs_d_.get(),
      reinterpret_cast<float *>(ground_segmented_points_msg_ptr_->data.get()),
      config_.ground_label_, config_.ground_prob_threshold_, config_.class_names_.size(),
      num_voxels_);
    CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));
    ground_segmented_points_msg_ptr_->header = header;
    ground_segmented_points_msg_ptr_->width = num_non_ground_points;
    publish_ground_segmented_pointcloud_(std::move(ground_segmented_points_msg_ptr_));
    ground_segmented_points_msg_ptr_ = nullptr;
  }

  // Prob pointcloud
  if (should_publish_probs_pointcloud) {
    post_ptr_->createProbsPointcloud(
      feat_d_.get(), pred_probs_d_.get(),
      reinterpret_cast<float *>(probs_points_msg_ptr_->data.get()), config_.class_names_.size(),
      num_voxels_);
    CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

    probs_points_msg_ptr_->header = header;
    probs_points_msg_ptr_->width = num_voxels_;
    publish_probs_pointcloud_(std::move(probs_points_msg_ptr_));
    probs_points_msg_ptr_ = nullptr;
  }

  allocateMessages();
  return true;
}

}  //  namespace autoware::ptv3
