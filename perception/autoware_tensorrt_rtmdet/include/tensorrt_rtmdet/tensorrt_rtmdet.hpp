// Copyright 2024 TIER IV, Inc.
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

#ifndef TENSORRT_RTMDET__TENSORRT_RTMDET_HPP_
#define TENSORRT_RTMDET__TENSORRT_RTMDET_HPP_

#include "tensorrt_rtmdet/calibrator.hpp"
#include "tensorrt_rtmdet/preprocess.hpp"

#include <cuda_utils/cuda_unique_ptr.hpp>
#include <cuda_utils/stream_unique_ptr.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tensorrt_common/tensorrt_common.hpp>

#include <cv_bridge/cv_bridge.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace autoware::tensorrt_rtmdet
{
using cuda_utils::CudaUniquePtr;
using cuda_utils::CudaUniquePtrHost;
using cuda_utils::makeCudaStream;
using cuda_utils::StreamUniquePtr;

struct Object
{
  int32_t x1;
  int32_t y1;
  int32_t x2;
  int32_t y2;
  int32_t class_id;
  int32_t mask_index;
  float score;
};

using ObjectArray = std::vector<Object>;
using ObjectArrays = std::vector<ObjectArray>;

using LabelColor = struct LabelColor
{
  std::string class_name;
  cv::Vec3b color;
  uint8_t label_id;
};

using ColorMap = std::map<int, LabelColor>;

class TrtRTMDet
{
public:
  TrtRTMDet(
    const std::string & model_path, const std::string & precision, const ColorMap & color_map,
    const float score_threshold = 0.3, const float nms_threshold = 0.7,
    const float mask_threshold = 200.0,
    const tensorrt_common::BuildConfig & build_config = tensorrt_common::BuildConfig(),
    const bool use_gpu_preprocess = false,
    const std::string & calibration_image_list_file = std::string(), const double norm_factor = 1.0,
    const std::vector<float> & mean = {103.53, 116.28, 123.675},
    const std::vector<float> & std = {57.375, 57.12, 58.395},
    [[maybe_unused]] const std::string & cache_dir = "",
    const tensorrt_common::BatchConfig & batch_config = {1, 1, 1},
    const size_t max_workspace_size = (1u << 30u),
    const std::vector<std::string> & plugin_paths = {});

  ~TrtRTMDet();

  TrtRTMDet(const TrtRTMDet &) = delete;
  TrtRTMDet & operator=(const TrtRTMDet &) = delete;
  TrtRTMDet(TrtRTMDet &&) = default;
  TrtRTMDet & operator=(TrtRTMDet &&) = delete;

  bool do_inference(
    const std::vector<cv::Mat> & images, ObjectArrays & objects, cv::Mat & mask,
    std::vector<uint8_t> & class_ids);

  void print_profiling();

private:
  void preprocess(const std::vector<cv::Mat> & images);

  void preprocess_gpu(const std::vector<cv::Mat> & images);

  bool feedforward(
    const std::vector<cv::Mat> & images, ObjectArrays & objects, cv::Mat & mask,
    std::vector<uint8_t> & class_ids);

  [[nodiscard]] float intersection_over_union(const Object & a, const Object & b) const;

  void nms_sorted_bboxes(const ObjectArray & input_objects, ObjectArray & output_objects) const;

  std::unique_ptr<tensorrt_common::TrtCommon> trt_common_;

  std::vector<float> input_h_;
  CudaUniquePtr<float[]> input_d_;
  CudaUniquePtr<float[]> out_dets_d_;
  CudaUniquePtr<int32_t[]> out_labels_d_;
  CudaUniquePtr<float[]> out_masks_d_;

  StreamUniquePtr stream_{makeCudaStream()};

  uint32_t max_detections_;
  float scale_width_;
  float scale_height_;

  // size of input image for model
  uint32_t model_input_width_;
  uint32_t model_input_height_;

  const float score_threshold_;
  const float nms_threshold_;
  const float mask_threshold_;
  const int batch_size_;

  // flag whether preprocess are performed on GPU
  const bool use_gpu_preprocess_;
  // host buffer for preprocessing on GPU
  CudaUniquePtrHost<unsigned char[]> image_buf_h_;
  // device buffer for preprocessing on GPU
  CudaUniquePtr<unsigned char[]> image_buf_d_;
  // normalization factor used for preprocessing
  const double norm_factor_;

  int32_t src_width_;
  int32_t src_height_;

  const std::vector<float> mean_;
  const std::vector<float> std_;

  // Host pointer for outputs
  std::unique_ptr<float[]> out_dets_h_;
  std::unique_ptr<int32_t[]> out_labels_h_;
  std::unique_ptr<float[]> out_masks_h_;

  // Segmentation
  const ColorMap color_map_;
};
}  // namespace autoware::tensorrt_rtmdet

#endif  // TENSORRT_RTMDET__TENSORRT_RTMDET_HPP_
