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

#include "autoware/cuda_pointcloud_preprocessor/point_types.hpp"
#include "autoware/cuda_pointcloud_preprocessor/types.hpp"
#include "autoware/cuda_pointcloud_preprocessor/undistort_kernels.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <autoware/cuda_utils/cuda_check_error.hpp>

namespace autoware::cuda_pointcloud_preprocessor
{

__device__ __constant__ double time_diff_threshold = 0.1;

__host__ __device__ Eigen::Matrix3f skewSymmetric(const Eigen::Vector3f & v)
{
  Eigen::Matrix3f m;
  m << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
  return m;
}

__host__ __device__ Eigen::Matrix3f leftJacobianSO3(const Eigen::Vector3f & omega)
{
  double theta = omega.norm();
  if (std::abs(theta) < 1e-6) {
    return Eigen::Matrix3f::Identity();
  }

  Eigen::Matrix3f Omega = skewSymmetric(omega);

  Eigen::Matrix3f Omega2 = Omega * Omega;
  double theta2 = theta * theta;
  double theta3 = theta2 * theta;

  // Rodrigues' formula for Jacobian
  return Eigen::Matrix3f::Identity() + ((1 - cos(theta)) / theta2) * Omega +
         ((theta - sin(theta)) / theta3) * Omega2;
}

__host__ __device__ Eigen::Matrix4f transformationMatrixFromVelocity(
  const Eigen::Vector3f & linear_velocity, const Eigen::Vector3f & angular_velocity, double dt)
{
  Eigen::Matrix3f R = Eigen::AngleAxisf(angular_velocity.norm() * dt, angular_velocity.normalized())
                        .toRotationMatrix();
  Eigen::Matrix3f J = leftJacobianSO3(angular_velocity * dt);

  Eigen::Vector3f translation = J * (linear_velocity * dt);

  Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
  transformation.block<3, 3>(0, 0) = R;
  transformation.block<3, 1>(0, 3) = translation;

  return transformation;
}

__global__ void undistort2DKernel(
  InputPointType * input_points, int num_points, TwistStruct2D * twist_structs, int num_twists,
  std::uint8_t * __restrict__ output_mismatch_mask)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx < num_points) {
    InputPointType & point = input_points[idx];

    // The twist must always be newer than the point ! (or it was the last twist)
    int twist_index = 0;
    while (twist_index < num_twists && twist_structs[twist_index].stamp_nsec < point.time_stamp) {
      twist_index++;
    }

    twist_index = min(twist_index, num_twists - 1);

    TwistStruct2D twist = twist_structs[twist_index];
    float x = twist.cum_x;
    float y = twist.cum_y;
    float theta = twist.cum_theta;

    double dt_nsec =
      point.time_stamp > twist.last_stamp_nsec ? point.time_stamp - twist.last_stamp_nsec : 0;
    double dt = 1e-9 * (dt_nsec);

    output_mismatch_mask[idx] = static_cast<std::uint8_t>(dt > time_diff_threshold);

    theta += twist.v_theta * dt;
    float d = twist.v_x * dt;
    x += d * cos(theta);
    y += d * sin(theta);

    float distorted_x = point.x;
    float distorted_y = point.y;

    point.x = distorted_x * cos(theta) - distorted_y * sin(theta) + x;
    point.y = distorted_x * sin(theta) + distorted_y * cos(theta) + y;
  }
}

__global__ void undistort3DKernel(
  InputPointType * input_points, int num_points, TwistStruct3D * twist_structs, int num_twists,
  std::uint8_t * __restrict__ output_mismatch_mask)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx < num_points) {
    InputPointType & point = input_points[idx];

    // The twist must always be newer than the point ! (or it was the last twist)
    int twist_index = 0;
    while (twist_index < num_twists && twist_structs[twist_index].stamp_nsec < point.time_stamp) {
      twist_index++;
    }

    twist_index = min(twist_index, num_twists - 1);

    TwistStruct3D twist = twist_structs[twist_index];
    Eigen::Map<Eigen::Matrix4f> cum_transform_buffer_map(twist.cum_transform_buffer);
    Eigen::Map<Eigen::Vector3f> v_map(twist.v);
    Eigen::Map<Eigen::Vector3f> w_map(twist.w);

    double dt_nsec =
      point.time_stamp > twist.last_stamp_nsec ? point.time_stamp - twist.last_stamp_nsec : 0;
    double dt = 1e-9 * (dt_nsec);

    output_mismatch_mask[idx] = static_cast<std::uint8_t>(dt > time_diff_threshold);

    Eigen::Matrix4f transform =
      cum_transform_buffer_map * transformationMatrixFromVelocity(v_map, w_map, dt);
    Eigen::Vector3f p(point.x, point.y, point.z);
    p = transform.block<3, 3>(0, 0) * p + transform.block<3, 1>(0, 3);

    point.x = p.x();
    point.y = p.y();
    point.z = p.z();
  }
}

void undistort2DLaunch(
  InputPointType * input_points, int num_points, TwistStruct2D * twist_structs, int num_twists,
  std::uint8_t * output_mismatch_mask, int threads_per_block, int blocks_per_grid,
  cudaStream_t & stream)
{
  undistort2DKernel<<<blocks_per_grid, threads_per_block, 0, stream>>>(
    input_points, num_points, twist_structs, num_twists, output_mismatch_mask);
  CHECK_CUDA_ERROR(cudaGetLastError());
}

void undistort3DLaunch(
  InputPointType * input_points, int num_points, TwistStruct3D * twist_structs, int num_twists,
  std::uint8_t * output_mismatch_mask, int threads_per_block, int blocks_per_grid,
  cudaStream_t & stream)
{
  undistort3DKernel<<<blocks_per_grid, threads_per_block, 0, stream>>>(
    input_points, num_points, twist_structs, num_twists, output_mismatch_mask);
  CHECK_CUDA_ERROR(cudaGetLastError());
}

void setupTwist2DStructs(
  const std::deque<geometry_msgs::msg::TwistWithCovarianceStamped> & twist_queue,
  const std::deque<geometry_msgs::msg::Vector3Stamped> & angular_velocity_queue,
  const std::uint64_t pointcloud_stamp_nsec, const std::uint32_t first_point_rel_stamp_nsec,
  thrust::device_vector<TwistStruct2D> & device_twist_2d_structs, cudaStream_t & stream)
{
  std::vector<TwistStruct2D> host_twist_2d_structs;
  host_twist_2d_structs.reserve(twist_queue.size() + angular_velocity_queue.size());

  // Twist preprocessing
  float cum_x = 0;
  float cum_y = 0;
  float cum_theta = 0;
  // All time stamps from now on are in nsec from the "beginning of the pointcloud"
  std::uint32_t last_stamp_nsec = first_point_rel_stamp_nsec;

  std::size_t twist_index = 0;
  std::size_t angular_velocity_index = 0;
  float v_x{0.f}, v_theta{0.f};

  for (; twist_index < twist_queue.size() ||
         angular_velocity_index < angular_velocity_queue.size();) {
    std::uint64_t twist_stamp, input_twist_global_stamp_nsec, angular_velocity_global_stamp_nsec;

    if (twist_index < twist_queue.size()) {
      input_twist_global_stamp_nsec =
        1'000'000'000 * static_cast<std::uint64_t>(twist_queue[twist_index].header.stamp.sec) +
        static_cast<std::uint64_t>(twist_queue[twist_index].header.stamp.nanosec);
      v_x = twist_queue[twist_index].twist.twist.linear.x;
    } else {
      input_twist_global_stamp_nsec = std::numeric_limits<std::uint64_t>::max();
    }

    if (angular_velocity_index < angular_velocity_queue.size()) {
      angular_velocity_global_stamp_nsec =
        1'000'000'000 * static_cast<std::uint64_t>(
                          angular_velocity_queue[angular_velocity_index].header.stamp.sec) +
        static_cast<std::uint64_t>(
          angular_velocity_queue[angular_velocity_index].header.stamp.nanosec);
      v_theta = angular_velocity_queue[angular_velocity_index].vector.z;
    } else {
      angular_velocity_global_stamp_nsec = std::numeric_limits<std::uint64_t>::max();
    }

    if (input_twist_global_stamp_nsec < angular_velocity_global_stamp_nsec) {
      twist_stamp = input_twist_global_stamp_nsec;
      twist_index++;
    } else if (input_twist_global_stamp_nsec > angular_velocity_global_stamp_nsec) {
      twist_stamp = angular_velocity_global_stamp_nsec;
      angular_velocity_index++;
    } else {
      twist_stamp = input_twist_global_stamp_nsec;
      twist_index++;
      angular_velocity_index++;
    }

    TwistStruct2D twist;
    twist.cum_x = cum_x;
    twist.cum_y = cum_y;
    twist.cum_theta = cum_theta;

    std::uint64_t twist_global_stamp_nsec = twist_stamp;
    assert(twist_global_stamp_nsec > pointcloud_stamp_nsec);  // by construction
    std::uint32_t twist_from_pointcloud_start_nsec =
      twist_global_stamp_nsec - pointcloud_stamp_nsec;

    twist.stamp_nsec = twist_from_pointcloud_start_nsec;
    twist.v_x = v_x;
    twist.v_theta = v_theta;
    twist.last_stamp_nsec = last_stamp_nsec;
    host_twist_2d_structs.push_back(twist);

    double dt_seconds = 1e-9 * (twist.stamp_nsec - last_stamp_nsec);
    last_stamp_nsec = twist.stamp_nsec;
    cum_theta += v_theta * dt_seconds;
    float d = twist.v_x * dt_seconds;
    cum_x += d * cos(cum_theta);
    cum_y += d * sin(cum_theta);
  }

  // Copy to device
  device_twist_2d_structs.resize(host_twist_2d_structs.size());
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    thrust::raw_pointer_cast(device_twist_2d_structs.data()), host_twist_2d_structs.data(),
    host_twist_2d_structs.size() * sizeof(TwistStruct2D), cudaMemcpyHostToDevice, stream));
}

void setupTwist3DStructs(
  const std::deque<geometry_msgs::msg::TwistWithCovarianceStamped> & twist_queue,
  const std::deque<geometry_msgs::msg::Vector3Stamped> & angular_velocity_queue,
  const std::uint64_t pointcloud_stamp_nsec, const std::uint32_t first_point_rel_stamp_nsec,
  thrust::device_vector<TwistStruct3D> & device_twist_3d_structs, cudaStream_t & stream)
{
  std::vector<TwistStruct3D> host_twist_3d_structs;
  host_twist_3d_structs.reserve(twist_queue.size() + angular_velocity_queue.size());

  Eigen::Matrix4f cum_transform = Eigen::Matrix4f::Identity();
  Eigen::Vector3f v = Eigen::Vector3f::Zero();
  Eigen::Vector3f w = Eigen::Vector3f::Zero();

  // All time stamps from now on are in nsec from the "beginning of the pointcloud"
  std::uint32_t last_stamp_nsec = first_point_rel_stamp_nsec;

  std::size_t twist_index = 0;
  std::size_t angular_velocity_index = 0;

  for (; twist_index < twist_queue.size() ||
         angular_velocity_index < angular_velocity_queue.size();) {
    std::uint64_t twist_stamp, input_twist_global_stamp_nsec, angular_velocity_global_stamp_nsec;

    if (twist_index < twist_queue.size()) {
      input_twist_global_stamp_nsec =
        1'000'000'000 * static_cast<std::uint64_t>(twist_queue[twist_index].header.stamp.sec) +
        static_cast<std::uint64_t>(twist_queue[twist_index].header.stamp.nanosec);
      v.x() = twist_queue[twist_index].twist.twist.linear.x;
      v.y() = twist_queue[twist_index].twist.twist.linear.y;
      v.z() = twist_queue[twist_index].twist.twist.linear.z;
    } else {
      input_twist_global_stamp_nsec = std::numeric_limits<uint64_t>::max();
      v = Eigen::Vector3f::Zero();
    }

    if (angular_velocity_index < angular_velocity_queue.size()) {
      angular_velocity_global_stamp_nsec =
        1'000'000'000 * static_cast<std::uint64_t>(
                          angular_velocity_queue[angular_velocity_index].header.stamp.sec) +
        static_cast<std::uint64_t>(
          angular_velocity_queue[angular_velocity_index].header.stamp.nanosec);
      w.x() = angular_velocity_queue[angular_velocity_index].vector.x;
      w.y() = angular_velocity_queue[angular_velocity_index].vector.y;
      w.z() = angular_velocity_queue[angular_velocity_index].vector.z;
    } else {
      angular_velocity_global_stamp_nsec = std::numeric_limits<std::uint64_t>::max();
      w = Eigen::Vector3f::Zero();
    }

    if (input_twist_global_stamp_nsec < angular_velocity_global_stamp_nsec) {
      twist_stamp = input_twist_global_stamp_nsec;
      twist_index++;
    } else if (input_twist_global_stamp_nsec > angular_velocity_global_stamp_nsec) {
      twist_stamp = angular_velocity_global_stamp_nsec;
      angular_velocity_index++;
    } else {
      twist_index++;
      angular_velocity_index++;
    }

    TwistStruct3D twist{};

    Eigen::Map<Eigen::Matrix4f> cum_transform_buffer_map(twist.cum_transform_buffer);
    Eigen::Map<Eigen::Vector3f> v_map(twist.v);
    Eigen::Map<Eigen::Vector3f> w_map(twist.w);
    cum_transform_buffer_map = cum_transform;

    std::uint64_t twist_global_stamp_nsec = twist_stamp;
    assert(twist_global_stamp_nsec > pointcloud_stamp_nsec);  // by construction
    std::uint32_t twist_from_pointcloud_start_nsec =
      twist_global_stamp_nsec - pointcloud_stamp_nsec;

    twist.stamp_nsec = twist_from_pointcloud_start_nsec;
    v_map = v;
    w_map = w;
    twist.last_stamp_nsec = last_stamp_nsec;
    host_twist_3d_structs.push_back(twist);

    double dt_seconds = 1e-9 * (twist.stamp_nsec - last_stamp_nsec);
    last_stamp_nsec = twist.stamp_nsec;

    auto delta_transform = transformationMatrixFromVelocity(v, w, dt_seconds);
    cum_transform = cum_transform * delta_transform;
  }

  // Copy to device
  device_twist_3d_structs.resize(host_twist_3d_structs.size());
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    thrust::raw_pointer_cast(device_twist_3d_structs.data()), host_twist_3d_structs.data(),
    host_twist_3d_structs.size() * sizeof(TwistStruct3D), cudaMemcpyHostToDevice, stream));
}

}  // namespace autoware::cuda_pointcloud_preprocessor
