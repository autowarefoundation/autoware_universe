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

#include "autoware/ptv3/backbone_engine.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::ptv3
{

BackboneEngine::BackboneEngine(
  const tensorrt_common::TrtCommonConfig & trt_config, const PTv3BackboneConfig & config,
  std::int32_t * grid_coord, float * feat, std::int64_t * serialized_code)
: config_(config)
{
  point_feat_d_ = autoware::cuda_utils::make_unique<float[]>(
    config_.max_num_voxels_ * config_.backbone_feat_dim_);
  point_grid_coord_d_ =
    autoware::cuda_utils::make_unique<std::int32_t[]>(config_.max_num_voxels_ * 3);
  point_offset_d_ = autoware::cuda_utils::make_unique<std::int64_t[]>(1);

  std::vector<autoware::tensorrt_common::NetworkIO> network_io;

  network_io.emplace_back("grid_coord", nvinfer1::Dims{2, {-1, 3}}, nvinfer1::DataType::kINT32);
  network_io.emplace_back("feat", nvinfer1::Dims{2, {-1, 4}}, nvinfer1::DataType::kFLOAT);
  network_io.emplace_back(
    "serialized_code", nvinfer1::Dims{2, {2, -1}}, nvinfer1::DataType::kINT64);
  network_io.emplace_back(
    "point_feat", nvinfer1::Dims{2, {-1, config_.backbone_feat_dim_}}, nvinfer1::DataType::kFLOAT);
  network_io.emplace_back(
    "point_grid_coord", nvinfer1::Dims{2, {-1, 3}}, nvinfer1::DataType::kINT32);
  network_io.emplace_back("point_offset", nvinfer1::Dims{1, {1}}, nvinfer1::DataType::kINT64);

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

  const auto add_pooling_io = [&network_io, &profile_dims](
                                const std::string & name, const nvinfer1::Dims & io_dims,
                                const nvinfer1::Dims & min_dims, const nvinfer1::Dims & opt_dims,
                                const nvinfer1::Dims & max_dims,
                                const std::optional<nvinfer1::DataType> data_type = std::nullopt) {
    network_io.emplace_back(name, io_dims, data_type);
    profile_dims.emplace_back(name, min_dims, opt_dims, max_dims);
  };

  const std::int64_t min_voxels = config_.voxels_num_[0];
  const std::int64_t opt_voxels = config_.voxels_num_[1];
  const std::int64_t max_voxels = config_.voxels_num_[2];
  const std::int64_t num_orders = static_cast<std::int64_t>(config_.serialization_orders_.size());

  for (std::size_t stage = 0; stage < config_.pooling_strides_.size(); ++stage) {
    const auto prefix = "serialized_pooling_" + std::to_string(stage) + "_";
    const std::int64_t in_min = stage == 0 ? min_voxels : 1;
    add_pooling_io(
      prefix + "indices", nvinfer1::Dims{1, {-1}}, nvinfer1::Dims{1, {in_min}},
      nvinfer1::Dims{1, {opt_voxels}}, nvinfer1::Dims{1, {max_voxels}});
    add_pooling_io(
      prefix + "cluster", nvinfer1::Dims{1, {-1}}, nvinfer1::Dims{1, {in_min}},
      nvinfer1::Dims{1, {opt_voxels}}, nvinfer1::Dims{1, {max_voxels}});
    add_pooling_io(
      prefix + "indptr", nvinfer1::Dims{1, {-1}}, nvinfer1::Dims{1, {2}},
      nvinfer1::Dims{1, {opt_voxels + 1}}, nvinfer1::Dims{1, {max_voxels + 1}});
    add_pooling_io(
      prefix + "head_indices", nvinfer1::Dims{1, {-1}}, nvinfer1::Dims{1, {1}},
      nvinfer1::Dims{1, {opt_voxels}}, nvinfer1::Dims{1, {max_voxels}});
    add_pooling_io(
      prefix + "grid_coord", nvinfer1::Dims{2, {-1, 3}}, nvinfer1::Dims{2, {1, 3}},
      nvinfer1::Dims{2, {opt_voxels, 3}}, nvinfer1::Dims{2, {max_voxels, 3}},
      nvinfer1::DataType::kINT32);
    add_pooling_io(
      prefix + "serialized_order", nvinfer1::Dims{2, {num_orders, -1}},
      nvinfer1::Dims{2, {num_orders, 1}}, nvinfer1::Dims{2, {num_orders, opt_voxels}},
      nvinfer1::Dims{2, {num_orders, max_voxels}});
    add_pooling_io(
      prefix + "serialized_inverse", nvinfer1::Dims{2, {num_orders, -1}},
      nvinfer1::Dims{2, {num_orders, 1}}, nvinfer1::Dims{2, {num_orders, opt_voxels}},
      nvinfer1::Dims{2, {num_orders, max_voxels}});
  }

  trt_ptr_ = std::make_unique<autoware::tensorrt_common::TrtCommon>(
    trt_config, std::make_shared<autoware::tensorrt_common::Profiler>(),
    std::vector<std::string>{config_.plugins_path_});

  if (!trt_ptr_->setup(
        std::make_unique<std::vector<autoware::tensorrt_common::ProfileDims>>(profile_dims),
        std::make_unique<std::vector<autoware::tensorrt_common::NetworkIO>>(network_io))) {
    throw std::runtime_error("Failed to setup backbone TRT engine.");
  }

  trt_ptr_->setTensorAddress("grid_coord", grid_coord);
  trt_ptr_->setTensorAddress("feat", feat);
  trt_ptr_->setTensorAddress("serialized_code", serialized_code);
  trt_ptr_->setTensorAddress("point_feat", point_feat_d_.get());
  trt_ptr_->setTensorAddress("point_grid_coord", point_grid_coord_d_.get());
  trt_ptr_->setTensorAddress("point_offset", point_offset_d_.get());
}

BackboneOutputView BackboneEngine::output() const
{
  return BackboneOutputView{point_feat_d_.get(), point_grid_coord_d_.get(), point_offset_d_.get()};
}

void BackboneEngine::bindSerializedPoolingAddresses(
  const std::vector<SerializedPoolingDeviceStageView> & stages)
{
  for (std::size_t stage = 0; stage < stages.size(); ++stage) {
    const auto prefix = "serialized_pooling_" + std::to_string(stage) + "_";
    const auto & buffers = stages[stage];
    trt_ptr_->setTensorAddress((prefix + "indices").c_str(), buffers.indices);
    trt_ptr_->setTensorAddress((prefix + "indptr").c_str(), buffers.indptr);
    trt_ptr_->setTensorAddress((prefix + "head_indices").c_str(), buffers.head_indices);
    trt_ptr_->setTensorAddress((prefix + "cluster").c_str(), buffers.cluster);
    trt_ptr_->setTensorAddress((prefix + "grid_coord").c_str(), buffers.grid_coord);
    trt_ptr_->setTensorAddress((prefix + "serialized_order").c_str(), buffers.serialized_order);
    trt_ptr_->setTensorAddress((prefix + "serialized_inverse").c_str(), buffers.serialized_inverse);
  }
}

bool BackboneEngine::setInputShapes(
  const std::int64_t num_voxels, const std::vector<std::int64_t> & serialized_pooling_num_voxels)
{
  trt_ptr_->setInputShape("grid_coord", nvinfer1::Dims{2, {num_voxels, 3}});
  trt_ptr_->setInputShape("feat", nvinfer1::Dims{2, {num_voxels, 4}});
  trt_ptr_->setInputShape("serialized_code", nvinfer1::Dims{2, {2, num_voxels}});

  bool success = true;
  const auto num_orders = static_cast<std::int64_t>(config_.serialization_orders_.size());
  for (std::size_t stage = 0; stage < config_.pooling_strides_.size(); ++stage) {
    const auto prefix = "serialized_pooling_" + std::to_string(stage) + "_";
    const auto in_count = serialized_pooling_num_voxels[stage];
    const auto out_count = serialized_pooling_num_voxels[stage + 1];
    success &= trt_ptr_->setInputShape((prefix + "indices").c_str(), nvinfer1::Dims{1, {in_count}});
    success &= trt_ptr_->setInputShape((prefix + "cluster").c_str(), nvinfer1::Dims{1, {in_count}});
    success &=
      trt_ptr_->setInputShape((prefix + "indptr").c_str(), nvinfer1::Dims{1, {out_count + 1}});
    success &=
      trt_ptr_->setInputShape((prefix + "head_indices").c_str(), nvinfer1::Dims{1, {out_count}});
    success &=
      trt_ptr_->setInputShape((prefix + "grid_coord").c_str(), nvinfer1::Dims{2, {out_count, 3}});
    success &= trt_ptr_->setInputShape(
      (prefix + "serialized_order").c_str(), nvinfer1::Dims{2, {num_orders, out_count}});
    success &= trt_ptr_->setInputShape(
      (prefix + "serialized_inverse").c_str(), nvinfer1::Dims{2, {num_orders, out_count}});
  }

  return success;
}

bool BackboneEngine::enqueue(const PTv3ExecutionContext & context)
{
  if (!trt_ptr_->enqueueV3(context.stream())) {
    RCLCPP_ERROR(rclcpp::get_logger("ptv3"), "Fail to enqueue backbone.");
    return false;
  }
  return true;
}

}  // namespace autoware::ptv3
