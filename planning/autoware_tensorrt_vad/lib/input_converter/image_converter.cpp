#include "autoware/tensorrt_vad/input_converter/image_converter.hpp"
#include <opencv2/opencv.hpp>
#include <stdexcept>

namespace autoware::tensorrt_vad::vad_interface {

InputImageConverter::InputImageConverter(const CoordinateTransformer& coordinate_transformer, const VadInterfaceConfig& config)
  : Converter(coordinate_transformer, config)
{
}

CameraImagesData InputImageConverter::process_image(
  const std::vector<sensor_msgs::msg::Image::ConstSharedPtr>& images) const
{
  std::vector<cv::Mat> processed_images;
  const int32_t num_cameras = static_cast<int32_t>(config_.autoware_to_vad_camera_mapping.size());
  processed_images.resize(num_cameras); // Initialize in VAD camera order

  // Process each camera image
  for (int32_t autoware_idx = 0; autoware_idx < num_cameras; ++autoware_idx) {
    const auto& image_msg = images[autoware_idx];

    // Create cv::Mat from sensor_msgs::msg::Image
    cv::Mat bgr_img;
    if (image_msg->encoding == "bgr8") {
      // For BGR8, use data directly
      bgr_img = cv::Mat(image_msg->height, image_msg->width, CV_8UC3, 
                        const_cast<uint8_t*>(image_msg->data.data()), image_msg->step);
    } else {
      RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("autoware_tensorrt_vad"), *rclcpp::Clock::make_shared(), 5000, "Unsupported image encoding: %s", image_msg->encoding.c_str());
      continue;
    }

    if (bgr_img.empty()) {
      RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("autoware_tensorrt_vad"), *rclcpp::Clock::make_shared(), 5000, "Failed to decode image data: %d", autoware_idx);
      continue;
    }

    // Clone the data to ensure memory continuity
    cv::Mat processed_img = bgr_img.clone();

    // Store in VAD camera order
    int32_t vad_idx = config_.autoware_to_vad_camera_mapping.at(autoware_idx);
    processed_images[vad_idx] = processed_img;
  }

  return processed_images;
}

} // namespace autoware::tensorrt_vad::vad_interface
