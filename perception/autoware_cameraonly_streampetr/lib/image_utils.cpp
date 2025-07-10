#include "autoware/camera_streampetr/image_utils.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cstring>

namespace autoware::camera_streampetr {

// void undistort_image_cv(
//   const std::uint8_t * input_image_data,
//   const int height,
//   const int width,
//   const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg,
//   std::vector<std::uint8_t> & output_image_data)
// {
//   if (!camera_info_msg) {
//     throw std::runtime_error("CameraInfo is null");
//   }

//   // Wrap input image
//   cv::Mat input_image(height, width, CV_8UC3, const_cast<std::uint8_t*>(input_image_data));

//   // Extract camera matrix K
//   cv::Mat K = (cv::Mat_<double>(3, 3) <<
//     camera_info_msg->k[0], camera_info_msg->k[1], camera_info_msg->k[2],
//     camera_info_msg->k[3], camera_info_msg->k[4], camera_info_msg->k[5],
//     camera_info_msg->k[6], camera_info_msg->k[7], camera_info_msg->k[8]);

//   // Extract distortion coefficients D
//   cv::Mat D = (cv::Mat_<double>(1, 5) <<
//     camera_info_msg->d[0], camera_info_msg->d[1], camera_info_msg->d[2],
//     camera_info_msg->d[3], camera_info_msg->d[4]);

//   // Undistort the image
//   cv::Mat undistorted_image;
//   cv::undistort(input_image, undistorted_image, K, D);

//   // Resize output buffer to match undistorted image
//   output_image_data.resize(undistorted_image.total() * undistorted_image.elemSize());
//   std::memcpy(output_image_data.data(), undistorted_image.data, output_image_data.size());
// }

void decompress_image(
  const sensor_msgs::msg::CompressedImage::ConstSharedPtr & input_compressed_image_msg, 
  cv_bridge::CvImagePtr & cv_ptr, 
  const int decompression_downsample)
{
  cv_ptr->header = input_compressed_image_msg->header;

  int mode = cv::IMREAD_COLOR;
  switch (decompression_downsample) {
    case 1:
      mode = cv::IMREAD_COLOR;
      break;
    case 2:
      mode = cv::IMREAD_COLOR | cv::IMREAD_REDUCED_COLOR_2;
      break;
    case 4:
      mode = cv::IMREAD_COLOR | cv::IMREAD_REDUCED_COLOR_4;
      break;
    case 8:
      mode = cv::IMREAD_COLOR | cv::IMREAD_REDUCED_COLOR_8;
      break;
    default:
      std::cout << "Invalid decompression downsample: " << decompression_downsample << " . Should be 1, 2, 4, or 8." << std::endl;
      break;
  }

  cv_ptr->image = cv::imdecode(cv::Mat(input_compressed_image_msg->data), mode);

  // Assign image encoding string
  const size_t split_pos = input_compressed_image_msg->format.find(';');
  if (split_pos == std::string::npos) {
    // Older version of compressed_image_transport does not signal image format
    switch (cv_ptr->image.channels()) {
      case 1:
        cv_ptr->encoding = sensor_msgs::image_encodings::MONO8;
        break;
      case 3:
        cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;
        break;
      default:
        std::cout << "Unsupported number of channels: " << cv_ptr->image.channels() << std::endl;
        break;
    }
  } else {
    std::string image_encoding = input_compressed_image_msg->format.substr(0, split_pos);
    cv_ptr->encoding = image_encoding;

    // Convert compressed image to OpenCV format
    if (sensor_msgs::image_encodings::isColor(image_encoding)) {
      std::string compressed_encoding = input_compressed_image_msg->format.substr(split_pos);
      bool compressed_bgr_image =
        (compressed_encoding.find("compressed bgr") != std::string::npos);

      // Revert color transformation
      if (compressed_bgr_image) {
        // if necessary convert colors from bgr to rgb
        if (
          (image_encoding == sensor_msgs::image_encodings::RGB8) ||
          (image_encoding == sensor_msgs::image_encodings::RGB16)) {
          cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_BGR2RGB);
        }

        if (
          (image_encoding == sensor_msgs::image_encodings::RGBA8) ||
          (image_encoding == sensor_msgs::image_encodings::RGBA16)) {
          cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_BGR2RGBA);
        }

        if (
          (image_encoding == sensor_msgs::image_encodings::BGRA8) ||
          (image_encoding == sensor_msgs::image_encodings::BGRA16)) {
          cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_BGR2BGRA);
        }
      } else {
        // if necessary convert colors from rgb to bgr
        if (
          (image_encoding == sensor_msgs::image_encodings::BGR8) ||
          (image_encoding == sensor_msgs::image_encodings::BGR16)) {
          cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_RGB2BGR);
        }

        if (
          (image_encoding == sensor_msgs::image_encodings::BGRA8) ||
          (image_encoding == sensor_msgs::image_encodings::BGRA16)) {
          cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_RGB2BGRA);
        }

        if (
          (image_encoding == sensor_msgs::image_encodings::RGBA8) ||
          (image_encoding == sensor_msgs::image_encodings::RGBA16)) {
          cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_RGB2RGBA);
        }
      }
    }
  }
}

} 