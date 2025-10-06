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

// cspell:ignore BEVFORMER

#ifndef ROS_UTILS_HPP_
#define ROS_UTILS_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <cuda_runtime.h>
#include <tf2/LinearMath/Quaternion.h>

// TensorRT headers
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <NvInfer.h>
#pragma GCC diagnostic pop

#include <cstdio>
#include <string>
#include <vector>

// Macro for error checking CUDA calls
#define CHECK_CUDA(call)                                                                       \
  do {                                                                                         \
    cudaError_t status = call;                                                                 \
    if (status != cudaSuccess) {                                                               \
      fprintf(                                                                                 \
        stderr, "CUDA Error at %s:%d - %s\n", __FILE__, __LINE__, cudaGetErrorString(status)); \
      exit(1);                                                                                 \
    }                                                                                          \
  } while (0)

namespace autoware
{
namespace tensorrt_bevformer
{

// Reusable structs and functions
struct Box3D
{
  double x;
  double y;
  double z;
  double w;
  double l;
  double h;
  double r;
  float score;
  double vx;
  double vy;
  int label;
};

// Function to convert Box3D vector to ROS DetectedObjects message
void box3DToDetectedObjects(
  const std::vector<Box3D> & boxes, autoware_perception_msgs::msg::DetectedObjects & objects_msg,
  const std::vector<std::string> & class_names, float score_threshold, bool add_twist);

// Extract camera intrinsics from CameraInfo message
void getCameraIntrinsics(
  const sensor_msgs::msg::CameraInfo::SharedPtr msg, Eigen::Matrix3d & intrinsics);

void getTransform(
  const geometry_msgs::msg::TransformStamped & transform_stamped,
  Eigen::Quaternion<double> & rotation, Eigen::Translation3d & translation);

}  // namespace tensorrt_bevformer
}  // namespace autoware

#endif  // ROS_UTILS_HPP_
