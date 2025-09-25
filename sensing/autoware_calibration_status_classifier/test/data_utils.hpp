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

#ifndef DATA_UTILS_HPP_
#define DATA_UTILS_HPP_

#include "autoware/calibration_status_classifier/data_type.hpp"
#include "autoware/calibration_status_classifier/data_type_eigen.hpp"

#include <Eigen/Geometry>
#include <autoware/point_types/types.hpp>
#include <opencv2/opencv.hpp>
#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <gtest/gtest.h>
#include <zlib.h>

#include <algorithm>
#include <execution>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace data_utils
{
constexpr size_t width = 2880;
constexpr size_t height = 1860;
constexpr size_t channels = 5;

using autoware::calibration_status_classifier::CameraLidarInfo;
using autoware::point_types::PointXYZIRC;
using autoware::point_types::PointXYZIRCGenerator;
using point_cloud_msg_wrapper::PointCloud2Modifier;
using sensor_msgs::msg::Image;
using sensor_msgs::msg::PointCloud2;

struct TestSample
{
  PointCloud2::SharedPtr pointcloud;
  Image::SharedPtr image_rgb;
  Image::SharedPtr image_bgr;
  Image::SharedPtr image_rgb_undistorted;
  Image::SharedPtr image_bgr_undistorted;
  CameraLidarInfo camera_lidar_info_calibrated;
  CameraLidarInfo camera_lidar_info_miscalibrated;
  std::vector<float> input_data_calibrated;
  std::vector<float> input_data_miscalibrated;
  std::string sample_name;
};

const size_t chunk_size = 16384;  // 16 KB

template <typename T>
std::vector<T> load_binary(const std::string & filename)
{
  // Read the entire compressed file into a buffer
  std::ifstream file(filename, std::ios::binary | std::ios::ate);
  if (!file) {
    throw std::runtime_error("Failed to open file: " + filename);
  }
  std::streamsize compressed_size = file.tellg();
  file.seekg(0, std::ios::beg);
  std::vector<unsigned char> compressed_buffer(compressed_size);
  if (!file.read(reinterpret_cast<char *>(compressed_buffer.data()), compressed_size)) {
    throw std::runtime_error("Failed to read compressed file: " + filename);
  }

  // Decompress the buffer in chunks
  std::vector<unsigned char> decompressed_buffer;

  z_stream strm = {};
  strm.zalloc = Z_NULL;
  strm.zfree = Z_NULL;
  strm.opaque = Z_NULL;
  strm.avail_in = compressed_buffer.size();
  strm.next_in = compressed_buffer.data();

  // Use +16 for gzip decoding
  if (inflateInit2(&strm, 16 + MAX_WBITS) != Z_OK) {
    throw std::runtime_error("inflateInit failed");
  }

  unsigned char out_chunk[chunk_size];
  int ret;
  do {
    strm.avail_out = chunk_size;
    strm.next_out = out_chunk;
    ret = inflate(&strm, Z_NO_FLUSH);

    if (ret != Z_OK && ret != Z_STREAM_END) {
      inflateEnd(&strm);
      throw std::runtime_error("Zlib inflate failed with error code: " + std::to_string(ret));
    }

    size_t have = chunk_size - strm.avail_out;
    decompressed_buffer.insert(decompressed_buffer.end(), out_chunk, out_chunk + have);
  } while (strm.avail_out == 0);

  inflateEnd(&strm);  // Clean up

  // Verify the final data and copy it to the result vector
  if (decompressed_buffer.size() % sizeof(T) != 0) {
    throw std::runtime_error("Decompressed size is not a multiple of element size.");
  }

  size_t num_elements = decompressed_buffer.size() / sizeof(T);
  std::vector<T> data(num_elements);
  std::memcpy(data.data(), decompressed_buffer.data(), decompressed_buffer.size());

  return data;
}

PointCloud2::SharedPtr get_pointcloud(
  const std::filesystem::path & data_dir, const std::string & sample_name)
{
  const auto sample_dir = data_dir / sample_name;
  auto cloud_data = load_binary<uint8_t>(sample_dir / "pointcloud.dat.gz");
  const size_t num_points = cloud_data.size() / sizeof(PointXYZIRC);
  PointCloud2::SharedPtr pointcloud = std::make_shared<PointCloud2>();
  PointCloud2Modifier<PointXYZIRC, PointXYZIRCGenerator> modifier(*pointcloud, "base_link");
  modifier.resize(num_points);
  pointcloud->data = std::move(cloud_data);
  return pointcloud;
}

Image::SharedPtr get_image(
  const std::filesystem::path & data_dir, const std::string & sample_name,
  const bool is_undistorted, const std::string & encoding)
{
  const auto sample_dir = data_dir / sample_name;
  const std::string suffix = is_undistorted ? "_undistorted" : "";
  auto image_data = load_binary<uint8_t>(sample_dir / ("image" + suffix + ".dat.gz"));

  // Swap R and B channels
  if (encoding == "bgr8") {
    std::vector<size_t> indices(image_data.size() / 3);
    std::iota(indices.begin(), indices.end(), 0);
    std::for_each(std::execution::seq, indices.begin(), indices.end(), [&image_data](size_t i) {
      std::swap(image_data[i * 3], image_data[i * 3 + 2]);
    });
  }

  Image::SharedPtr image = std::make_shared<Image>();
  image->height = height;
  image->width = width;
  image->encoding = encoding;
  image->step = image->width * 3;
  image->header.frame_id = "optical_camera_link";
  image->data = std::move(image_data);
  return image;
}

CameraLidarInfo get_camera_lidar_info(
  const std::filesystem::path & data_dir, const std::string & sample_name, bool is_miscalibrated)
{
  const auto sample_dir = data_dir / sample_name;
  const std::string suffix = is_miscalibrated ? "miscalibrated" : "calibrated";
  auto camera_lidar_info = CameraLidarInfo();

  camera_lidar_info.width = width;
  camera_lidar_info.height = height;
  camera_lidar_info.d.resize(autoware::calibration_status_classifier::dist_coeffs_size);

  auto dist_coeffs =
    load_binary<double>(sample_dir / ("distortion_coefficients_" + suffix + ".dat.gz"));
  if (dist_coeffs.size() != 8) {
    throw std::runtime_error("Invalid distortion coefficients size, expected 8 elements.");
  }
  std::move(dist_coeffs.begin(), dist_coeffs.end(), camera_lidar_info.d.begin());
  GTEST_LOG_(INFO) << "[" << suffix << "] Distortion coefficients:\n[" << dist_coeffs.at(0) << ", "
                   << dist_coeffs.at(1) << ", " << dist_coeffs.at(2) << ", " << dist_coeffs.at(3)
                   << ", " << dist_coeffs.at(4) << ", " << dist_coeffs.at(5) << ", "
                   << dist_coeffs.at(6) << ", " << dist_coeffs.at(7) << "]";

  auto camera_matrix = load_binary<double>(sample_dir / ("camera_matrix_" + suffix + ".dat.gz"));
  if (camera_matrix.size() != 9) {
    throw std::runtime_error("Invalid camera matrix size, expected 9 elements.");
  }
  camera_lidar_info.k =
    Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(camera_matrix.data());
  GTEST_LOG_(INFO) << "[" << suffix << "] Camera matrix:\n[" << camera_matrix.at(0) << ", "
                   << camera_matrix.at(1) << ", " << camera_matrix.at(2) << "]\n["
                   << camera_matrix.at(3) << ", " << camera_matrix.at(4) << ", "
                   << camera_matrix.at(5) << "]\n[" << camera_matrix.at(6) << ", "
                   << camera_matrix.at(7) << ", " << camera_matrix.at(8) << "]";

  auto projection_matrix =
    load_binary<double>(sample_dir / ("projection_matrix_" + suffix + ".dat.gz"));
  if (projection_matrix.size() != 12) {
    throw std::runtime_error("Invalid projection matrix size, expected 12 elements.");
  }
  camera_lidar_info.p =
    Eigen::Map<const Eigen::Matrix<double, 3, 4, Eigen::RowMajor>>(projection_matrix.data());
  GTEST_LOG_(INFO) << "[" << suffix << "] Projection matrix:\n[" << projection_matrix.at(0) << ", "
                   << projection_matrix.at(1) << ", " << projection_matrix.at(2) << ", "
                   << projection_matrix.at(3) << "]\n[" << projection_matrix.at(4) << ", "
                   << projection_matrix.at(5) << ", " << projection_matrix.at(6) << ", "
                   << projection_matrix.at(7) << "]\n[" << projection_matrix.at(8) << ", "
                   << projection_matrix.at(9) << ", " << projection_matrix.at(10) << ", "
                   << projection_matrix.at(11) << "]";

  auto transform_data =
    load_binary<double>(sample_dir / ("lidar_to_camera_tf_" + suffix + ".dat.gz"));
  if (transform_data.size() != 16) {
    throw std::runtime_error("Invalid transform data size, expected 16 elements.");
  }
  camera_lidar_info.tf_camera_to_lidar =
    Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(transform_data.data());

  GTEST_LOG_(INFO) << "[" << suffix << "] Lidar to camera transform:\n"
                   << camera_lidar_info.tf_camera_to_lidar.matrix();

  camera_lidar_info.to_undistort = true;

  return camera_lidar_info;
}

std::vector<float> get_input_data(
  const std::filesystem::path & data_dir, const std::string & sample_name, bool is_miscalibrated)
{
  const auto sample_dir = data_dir / sample_name;
  const std::string suffix = is_miscalibrated ? "miscalibrated" : "calibrated";
  auto input_data = load_binary<float>(sample_dir / ("input_data_" + suffix + ".dat.gz"));
  if (input_data.size() != width * height * channels) {
    throw std::runtime_error(
      "Invalid input data size, expected " + std::to_string(width * height * channels) +
      " elements.");
  }
  return input_data;
}

void save_img(
  std::vector<uint8_t> data, const int width, const int height,
  const std::filesystem::path & data_dir, const std::string & filename, const int type,
  const std::string & encoding)
{
  std::filesystem::path output_path = data_dir / filename;
  cv::Mat img_mat(height, width, type, data.data());
  if (encoding == "bgr8") {
    cv::cvtColor(img_mat, img_mat, cv::COLOR_BGR2RGB);
  }
  cv::imwrite(output_path.string(), img_mat);
}

TestSample load_test_sample(const std::filesystem::path & data_dir, const std::string & sample_name)
{
  TestSample sample;
  sample.pointcloud = get_pointcloud(data_dir, sample_name);
  sample.image_rgb = get_image(data_dir, sample_name, false, "rgb8");
  sample.image_bgr = get_image(data_dir, sample_name, false, "bgr8");
  sample.image_rgb_undistorted = get_image(data_dir, sample_name, true, "rgb8");
  sample.image_bgr_undistorted = get_image(data_dir, sample_name, true, "bgr8");
  sample.camera_lidar_info_calibrated = get_camera_lidar_info(data_dir, sample_name, false);
  sample.camera_lidar_info_miscalibrated = get_camera_lidar_info(data_dir, sample_name, true);
  sample.input_data_calibrated = get_input_data(data_dir, sample_name, false);
  sample.input_data_miscalibrated = get_input_data(data_dir, sample_name, true);
  sample.sample_name = sample_name;
  return sample;
}

}  // namespace data_utils

#endif  // DATA_UTILS_HPP_
