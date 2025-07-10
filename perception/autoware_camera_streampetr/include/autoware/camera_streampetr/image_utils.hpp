#ifndef AUTOWARE_CAMERA_STREAMPETR_IMAGE_UTILS_HPP
#define AUTOWARE_CAMERA_STREAMPETR_IMAGE_UTILS_HPP

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <cstdint>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/camera_info.hpp>

namespace autoware::camera_streampetr {

// void undistort_image_cv(
//   const std::uint8_t * input_image_data,
//   const int height,
//   const int width,
//   const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg,
//   std::vector<std::uint8_t> & output_image_data);

void decompress_image(
  const sensor_msgs::msg::CompressedImage::ConstSharedPtr & input_compressed_image_msg, 
  cv_bridge::CvImagePtr & cv_ptr, 
  const int decompression_downsample);

}

#endif