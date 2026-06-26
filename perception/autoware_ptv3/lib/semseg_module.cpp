// Copyright 2026 TIER IV, Inc.
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

#include "autoware/ptv3/semseg_module.hpp"

#include "autoware/ptv3/preprocess/point_type.hpp"

#include <autoware/cuda_utils/cuda_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::ptv3
{

SemsegModule::SemsegModule(
  const tensorrt_common::TrtCommonConfig & trt_config, const PTv3SemsegConfig & config,
  const float * backbone_point_feat, const float * backbone_input_features,
  const void * compact_points)
: config_(config),
  backbone_point_feat_(backbone_point_feat),
  backbone_input_features_(backbone_input_features),
  compact_points_(compact_points)
{
  createPointFields();
  preprocess_ptr_ = std::make_unique<SemsegPreprocessCuda>(config_);

  pred_labels_d_ = autoware::cuda_utils::make_unique<std::int64_t[]>(config_.max_num_voxels_);
  pred_probs_d_ = autoware::cuda_utils::make_unique<float[]>(
    config_.max_num_voxels_ * config_.segmentation_class_names_.size());
  if (config_.source_reconstruction_ == SourceReconstruction::PARTIAL) {
    cropped_source_points_d_ = autoware::cuda_utils::make_unique<std::uint8_t[]>(
      config_.cloud_capacity_ * sizeof(CloudPointTypeXYZIRCAEDT));
  }
  if (config_.source_reconstruction_ != SourceReconstruction::NONE) {
    reconstructed_features_d_ = autoware::cuda_utils::make_unique<float[]>(
      config_.cloud_capacity_ * config_.num_point_feature_size_);
    inverse_map_d_ = autoware::cuda_utils::make_unique<std::int64_t[]>(config_.cloud_capacity_);
    reconstructed_labels_d_ =
      autoware::cuda_utils::make_unique<std::int64_t[]>(config_.cloud_capacity_);
    reconstructed_probs_d_ = autoware::cuda_utils::make_unique<float[]>(
      config_.cloud_capacity_ * config_.segmentation_class_names_.size());
  }

  initTrt(trt_config);
}

bool SemsegModule::shouldRun(
  const bool should_publish_segmented_pointcloud,
  const bool should_publish_visualization_pointcloud,
  const bool should_publish_filtered_pointcloud) const
{
  return should_publish_segmented_pointcloud || should_publish_visualization_pointcloud ||
         should_publish_filtered_pointcloud;
}

void SemsegModule::setPublishSegmentedPointcloud(
  std::function<void(std::unique_ptr<const cuda_blackboard::CudaPointCloud2>)> func)
{
  publish_segmented_pointcloud_ = std::move(func);
}

void SemsegModule::setPublishVisualizationPointcloud(
  std::function<void(std::unique_ptr<const cuda_blackboard::CudaPointCloud2>)> func)
{
  publish_visualization_pointcloud_ = std::move(func);
}

void SemsegModule::setPublishFilteredPointcloud(
  std::function<void(std::unique_ptr<const cuda_blackboard::CudaPointCloud2>)> func)
{
  publish_filtered_pointcloud_ = std::move(func);
}

void SemsegModule::preparePreprocess(
  const cuda_blackboard::CudaPointCloud2 & msg, const PTv3ExecutionContext & context,
  const CloudFormat input_format, const bool should_run)
{
  const auto stream = context.stream();
  prepareFilteredPointcloudFormat(input_format);
  allocateOutputMessages();

  if (!should_run) {
    return;
  }

  using autoware::cuda_utils::clear_async;
  clear_async(pred_labels_d_.get(), static_cast<std::size_t>(config_.max_num_voxels_), stream);
  clear_async(
    pred_probs_d_.get(),
    static_cast<std::size_t>(config_.max_num_voxels_) * config_.segmentation_class_names_.size(),
    stream);
  if (config_.source_reconstruction_ == SourceReconstruction::FULL) {
    num_source_points_ = static_cast<std::size_t>(msg.height) * msg.width;
    current_input_data_ = msg.data.get();
  }
  if (config_.source_reconstruction_ == SourceReconstruction::PARTIAL) {
    clear_async(
      cropped_source_points_d_.get(),
      static_cast<std::size_t>(config_.cloud_capacity_) * sizeof(CloudPointTypeXYZIRCAEDT), stream);
  }
  if (config_.source_reconstruction_ != SourceReconstruction::NONE) {
    clear_async(
      reconstructed_features_d_.get(),
      static_cast<std::size_t>(config_.cloud_capacity_) * config_.num_point_feature_size_, stream);
    clear_async(inverse_map_d_.get(), static_cast<std::size_t>(config_.cloud_capacity_), stream);
    clear_async(
      reconstructed_labels_d_.get(), static_cast<std::size_t>(config_.cloud_capacity_), stream);
    clear_async(
      reconstructed_probs_d_.get(),
      static_cast<std::size_t>(config_.cloud_capacity_) * config_.segmentation_class_names_.size(),
      stream);
  }
}

void SemsegModule::prepareFilteredPointcloudFormat(const CloudFormat input_format)
{
  std::call_once(init_filtered_cloud_, [this, input_format]() {
    input_format_ = input_format;
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
      rclcpp::get_logger("ptv3"), "Filtered point cloud output format '%s'",
      to_string(filtered_output_format_));
  });
}

void SemsegModule::preprocessSourceReconstruction(
  const cuda_blackboard::CudaPointCloud2 & msg, const PTv3ExecutionContext & context,
  const PreprocessCuda & backbone_preprocessor, const std::size_t num_cropped_points,
  const bool should_run)
{
  if (!should_run) {
    return;
  }

  const SemsegSourceReconstructionView view{
    config_.source_reconstruction_ != SourceReconstruction::NONE ? reconstructed_features_d_.get()
                                                                 : nullptr,
    config_.source_reconstruction_ == SourceReconstruction::PARTIAL ? cropped_source_points_d_.get()
                                                                    : nullptr,
    config_.source_reconstruction_ != SourceReconstruction::NONE ? inverse_map_d_.get() : nullptr,
    &num_cropped_points_};
  preprocess_ptr_->reconstructSource(
    backbone_preprocessor, msg.data.get(), input_format_,
    static_cast<std::size_t>(msg.height) * msg.width, num_cropped_points, view, context.stream());
}

void SemsegModule::setInputShape(const std::int64_t num_voxels)
{
  trt_ptr_->setInputShape(
    "point_feat", nvinfer1::Dims{2, {num_voxels, config_.backbone_feat_dim_}});
}

bool SemsegModule::enqueue(const PTv3ExecutionContext & context)
{
  if (!trt_ptr_->enqueueV3(context.stream())) {
    RCLCPP_ERROR(rclcpp::get_logger("ptv3"), "Fail to enqueue seg3d head.");
    return false;
  }
  return true;
}

bool SemsegModule::postProcess(
  const std_msgs::msg::Header & header, const PTv3ExecutionContext & context,
  const PreprocessCuda & preprocessor, const CloudFormat input_format,
  const bool should_publish_segmented_pointcloud,
  const bool should_publish_visualization_pointcloud, const bool should_publish_filtered_pointcloud,
  const std::int64_t num_voxels)
{
  const auto stream = context.stream();
  if (post_ptr_ == nullptr) {
    post_ptr_ = std::make_unique<PostprocessCuda>(config_);
  }

  if (config_.source_reconstruction_ == SourceReconstruction::PARTIAL) {
    post_ptr_->reconstructPartial(
      inverse_map_d_.get(), pred_labels_d_.get(), pred_probs_d_.get(),
      reconstructed_labels_d_.get(), reconstructed_probs_d_.get(),
      config_.segmentation_class_names_.size(), num_cropped_points_, num_voxels, stream);
  }
  if (config_.source_reconstruction_ == SourceReconstruction::FULL) {
    post_ptr_->reconstructFull(
      preprocessor.cropMask(), preprocessor.cropIndices(), inverse_map_d_.get(),
      pred_labels_d_.get(), pred_probs_d_.get(), reconstructed_labels_d_.get(),
      reconstructed_probs_d_.get(), config_.segmentation_class_names_.size(), num_source_points_,
      num_voxels, stream);
  }

  const auto source_features = config_.source_reconstruction_ != SourceReconstruction::NONE
                                 ? reconstructed_features_d_.get()
                                 : backbone_input_features_;
  const auto source_labels = config_.source_reconstruction_ != SourceReconstruction::NONE
                               ? reconstructed_labels_d_.get()
                               : pred_labels_d_.get();
  const auto source_probs = config_.source_reconstruction_ != SourceReconstruction::NONE
                              ? reconstructed_probs_d_.get()
                              : pred_probs_d_.get();
  const auto source_points = config_.source_reconstruction_ == SourceReconstruction::FULL
                               ? current_input_data_
                             : config_.source_reconstruction_ == SourceReconstruction::PARTIAL
                               ? cropped_source_points_d_.get()
                               : compact_points_;
  const auto num_source_output_points =
    config_.source_reconstruction_ == SourceReconstruction::FULL      ? num_source_points_
    : config_.source_reconstruction_ == SourceReconstruction::PARTIAL ? num_cropped_points_
                                                                      : num_voxels;

  if (should_publish_segmented_pointcloud) {
    post_ptr_->createSegmentationPointcloud(
      source_features, source_labels, source_probs, segmented_points_msg_ptr_->data.get(),
      config_.segmentation_class_names_.size(), num_source_output_points, stream);
    CHECK_CUDA_ERROR(cudaStreamSynchronize(stream));

    segmented_points_msg_ptr_->header = header;
    segmented_points_msg_ptr_->width = static_cast<std::uint32_t>(num_source_output_points);
    publish_segmented_pointcloud_(std::move(segmented_points_msg_ptr_));
    segmented_points_msg_ptr_ = nullptr;
  }

  if (should_publish_visualization_pointcloud) {
    post_ptr_->createVisualizationPointcloud(
      source_features, source_labels,
      reinterpret_cast<float *>(visualization_points_msg_ptr_->data.get()),
      config_.segmentation_class_names_.size(), num_source_output_points, stream);
    CHECK_CUDA_ERROR(cudaStreamSynchronize(stream));
    visualization_points_msg_ptr_->header = header;
    visualization_points_msg_ptr_->width = static_cast<std::uint32_t>(num_source_output_points);
    publish_visualization_pointcloud_(std::move(visualization_points_msg_ptr_));
    visualization_points_msg_ptr_ = nullptr;
  }

  if (should_publish_filtered_pointcloud) {
    const auto num_filtered_points = post_ptr_->createFilteredPointcloud(
      source_points, input_format, filtered_output_format_, source_probs,
      filtered_points_msg_ptr_->data.get(), config_.segmentation_class_names_.size(),
      num_source_output_points, stream);
    CHECK_CUDA_ERROR(cudaStreamSynchronize(stream));

    filtered_points_msg_ptr_->header = header;
    filtered_points_msg_ptr_->width = num_filtered_points;
    publish_filtered_pointcloud_(std::move(filtered_points_msg_ptr_));
    filtered_points_msg_ptr_ = nullptr;
  }

  allocateOutputMessages();
  return true;
}

void SemsegModule::initTrt(const tensorrt_common::TrtCommonConfig & trt_config)
{
  std::vector<autoware::tensorrt_common::NetworkIO> network_io;

  network_io.emplace_back(
    "point_feat", nvinfer1::Dims{2, {-1, config_.backbone_feat_dim_}}, nvinfer1::DataType::kFLOAT);
  network_io.emplace_back("pred_labels", nvinfer1::Dims{1, {-1}}, nvinfer1::DataType::kINT64);
  network_io.emplace_back(
    "pred_probs",
    nvinfer1::Dims{2, {-1, static_cast<std::int64_t>(config_.segmentation_class_names_.size())}},
    nvinfer1::DataType::kFLOAT);

  std::vector<autoware::tensorrt_common::ProfileDims> profile_dims;
  profile_dims.emplace_back(
    "point_feat", nvinfer1::Dims{2, {config_.voxels_num_[0], config_.backbone_feat_dim_}},
    nvinfer1::Dims{2, {config_.voxels_num_[1], config_.backbone_feat_dim_}},
    nvinfer1::Dims{2, {config_.voxels_num_[2], config_.backbone_feat_dim_}});

  trt_ptr_ = std::make_unique<autoware::tensorrt_common::TrtCommon>(
    trt_config, std::make_shared<autoware::tensorrt_common::Profiler>(),
    std::vector<std::string>{config_.plugins_path_});

  if (!trt_ptr_->setup(
        std::make_unique<std::vector<autoware::tensorrt_common::ProfileDims>>(profile_dims),
        std::make_unique<std::vector<autoware::tensorrt_common::NetworkIO>>(network_io))) {
    throw std::runtime_error("Failed to setup seg3d_head TRT engine.");
  }

  trt_ptr_->setTensorAddress("point_feat", const_cast<float *>(backbone_point_feat_));
  trt_ptr_->setTensorAddress("pred_labels", pred_labels_d_.get());
  trt_ptr_->setTensorAddress("pred_probs", pred_probs_d_.get());
}

void SemsegModule::createPointFields()
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
  segmented_pointcloud_fields_.push_back(
    make_point_field("entropy", 17, sensor_msgs::msg::PointField::FLOAT32, 1));

  visualization_pointcloud_fields_.push_back(
    make_point_field("x", 0, sensor_msgs::msg::PointField::FLOAT32, 1));
  visualization_pointcloud_fields_.push_back(
    make_point_field("y", 4, sensor_msgs::msg::PointField::FLOAT32, 1));
  visualization_pointcloud_fields_.push_back(
    make_point_field("z", 8, sensor_msgs::msg::PointField::FLOAT32, 1));
  visualization_pointcloud_fields_.push_back(
    make_point_field("rgb", 12, sensor_msgs::msg::PointField::FLOAT32, 1));
}

std::int64_t SemsegModule::outputCapacity() const
{
  return config_.source_reconstruction_ != SourceReconstruction::NONE ? config_.cloud_capacity_
                                                                      : config_.max_num_voxels_;
}

void SemsegModule::allocateOutputMessages()
{
  const auto output_capacity = outputCapacity();
  if (segmented_points_msg_ptr_ == nullptr) {
    segmented_points_msg_ptr_ = std::make_unique<cuda_blackboard::CudaPointCloud2>();
    segmented_points_msg_ptr_->height = 1;
    segmented_points_msg_ptr_->width = output_capacity;
    segmented_points_msg_ptr_->fields = segmented_pointcloud_fields_;
    segmented_points_msg_ptr_->is_bigendian = false;
    segmented_points_msg_ptr_->is_dense = true;
    segmented_points_msg_ptr_->point_step = 21U;
    segmented_points_msg_ptr_->data = cuda_blackboard::make_unique<std::uint8_t[]>(
      output_capacity * segmented_points_msg_ptr_->point_step);
  }

  if (visualization_points_msg_ptr_ == nullptr) {
    visualization_points_msg_ptr_ = std::make_unique<cuda_blackboard::CudaPointCloud2>();
    visualization_points_msg_ptr_->height = 1;
    visualization_points_msg_ptr_->width = output_capacity;
    visualization_points_msg_ptr_->fields = visualization_pointcloud_fields_;
    visualization_points_msg_ptr_->is_bigendian = false;
    visualization_points_msg_ptr_->is_dense = true;
    visualization_points_msg_ptr_->point_step =
      static_cast<std::uint32_t>(visualization_pointcloud_fields_.size() * sizeof(float));
    visualization_points_msg_ptr_->data = cuda_blackboard::make_unique<std::uint8_t[]>(
      output_capacity * visualization_points_msg_ptr_->point_step);
  }

  if (filtered_points_msg_ptr_ == nullptr && filtered_output_format_ != CloudFormat::UNKNOWN) {
    filtered_points_msg_ptr_ = std::make_unique<cuda_blackboard::CudaPointCloud2>();
    filtered_points_msg_ptr_->height = 1;
    filtered_points_msg_ptr_->width = output_capacity;
    filtered_points_msg_ptr_->fields = filtered_pointcloud_fields_;
    filtered_points_msg_ptr_->is_bigendian = false;
    filtered_points_msg_ptr_->is_dense = true;
    filtered_points_msg_ptr_->point_step =
      static_cast<std::uint32_t>(get_point_step(filtered_output_format_));
    filtered_points_msg_ptr_->data = cuda_blackboard::make_unique<std::uint8_t[]>(
      output_capacity * filtered_points_msg_ptr_->point_step);
  }
}

}  // namespace autoware::ptv3
