#ifndef AUTOWARE_TENSORRT_VAD_INPUT_CONVERTER_IMAGE_CONVERTER_HPP_
#define AUTOWARE_TENSORRT_VAD_INPUT_CONVERTER_IMAGE_CONVERTER_HPP_

#include "autoware/tensorrt_vad/converter.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/core/mat.hpp>
#include <vector>
#include <memory>

namespace autoware::tensorrt_vad::vad_interface {

using CameraImagesData = std::vector<cv::Mat>;

/**
 * @brief InputImageConverter handles camera image data processing and normalization
 * 
 * This class converts ROS Image messages to normalized float arrays suitable for VAD model input:
 * - Image format conversion (BGR to RGB)
 * - Image resizing to target dimensions
 * - Pixel value normalization using mean and std parameters
 * - Multiple camera image concatenation in VAD camera order
 */
class InputImageConverter : public Converter {
public:
  /**
   * @brief Constructor
   * @param coordinate_transformer Reference to coordinate transformer
   * @param config Reference to configuration containing image parameters
   */
  InputImageConverter(const CoordinateTransformer& coordinate_transformer, const VadInterfaceConfig& config);

  /**
   * @brief Process multiple camera images for VAD model input
   * @param images Vector of ROS Image messages from multiple cameras
   * @return CameraImagesData Vector of cv::Mat images processed for VAD model
   */
  CameraImagesData process_image(
    const std::vector<sensor_msgs::msg::Image::ConstSharedPtr>& images) const;

};

} // namespace autoware::tensorrt_vad::vad_interface

#endif  // AUTOWARE_TENSORRT_VAD_INPUT_CONVERTER_IMAGE_CONVERTER_HPP_
