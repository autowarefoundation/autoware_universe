#include "autoware/tensorrt_vad/input_converter/transform_matrix_converter.hpp"
#include <rclcpp/rclcpp.hpp>

namespace autoware::tensorrt_vad::vad_interface {

InputTransformMatrixConverter::InputTransformMatrixConverter(
  const CoordinateTransformer& coordinate_transformer, const VadInterfaceConfig& config)
  : Converter(coordinate_transformer, config)
{
}

Eigen::Matrix4f InputTransformMatrixConverter::create_cam2img(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info) const
{
  // Create cam2img matrix directly from camera_info->k
  Eigen::Matrix4f cam2img;
  cam2img << camera_info->k[0], camera_info->k[1], camera_info->k[2], 0.0f,
             camera_info->k[3], camera_info->k[4], camera_info->k[5], 0.0f,
             camera_info->k[6], camera_info->k[7], camera_info->k[8], 0.0f,
             0.0f,              0.0f,              0.0f,              1.0f;

  return cam2img;
}


std::pair<float, float> InputTransformMatrixConverter::calculate_scale(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info) const
{
  const float scale_width = config_.target_image_width / static_cast<float>(camera_info->width);
  const float scale_height = config_.target_image_height / static_cast<float>(camera_info->height);
  return std::make_pair(scale_width, scale_height);
}

Eigen::Matrix4f InputTransformMatrixConverter::apply_scaling(
  const Eigen::Matrix4f& vad_base2img, const float scale_width, const float scale_height) const
{
  Eigen::Matrix4f scale_matrix = Eigen::Matrix4f::Identity();
  scale_matrix(0, 0) = scale_width;
  scale_matrix(1, 1) = scale_height;
  return scale_matrix * vad_base2img;
}

std::vector<float> InputTransformMatrixConverter::matrix_to_flat(const Eigen::Matrix4f& matrix) const
{
  std::vector<float> flat(16);
  int32_t k = 0;
  for (int32_t i = 0; i < 4; ++i) {
    for (int32_t j = 0; j < 4; ++j) {
      flat[k++] = matrix(i, j);
    }
  }
  return flat;
}

VadBase2ImgData InputTransformMatrixConverter::process_vad_base2img(
  const std::vector<sensor_msgs::msg::CameraInfo::ConstSharedPtr>& camera_infos) const
{
  const int32_t num_cameras = static_cast<int32_t>(config_.autoware_to_vad_camera_mapping.size());
  std::vector<float> frame_vad_base2img(16 * num_cameras, 0.0f); // Reserve space for cameras

  // Process each camera
  for (int32_t autoware_camera_id = 0; autoware_camera_id < num_cameras; ++autoware_camera_id) {
    if (!camera_infos[autoware_camera_id]) {
      continue;
    }

    // Use transformer to lookup base2cam transformation
    auto base2cam_opt = coordinate_transformer_.lookup_base2cam(camera_infos[autoware_camera_id]->header.frame_id);
    if (!base2cam_opt) {
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("autoware_tensorrt_vad"), *rclcpp::Clock::make_shared(), 5000, 
                  "Failed to lookup transformation for camera %d", autoware_camera_id);
      continue;
    }
    Eigen::Matrix4f base2cam = *base2cam_opt;

    Eigen::Matrix4f cam2img = create_cam2img(camera_infos[autoware_camera_id]);
    
    // Get vad2base transformation from config through coordinate transformer
    // The coordinate transformer uses vad2base internally, but we need to access it for vad2cam calculation
    // Calculate vad2cam transformation: vad2img = cam2img * base2cam * vad2base
    Eigen::Matrix4f vad2cam_rt = base2cam * config_.vad2base;
    Eigen::Matrix4f vad_base2img = cam2img * vad2cam_rt;

    // Calculate scaling factors from camera_info
    auto [scale_width, scale_height] = calculate_scale(camera_infos[autoware_camera_id]);

    // Apply scaling
    Eigen::Matrix4f vad_base2img_scaled = apply_scaling(vad_base2img, scale_width, scale_height);

    std::vector<float> vad_base2img_flat = matrix_to_flat(vad_base2img_scaled);

    // Store result at VAD camera ID position after vad_base2img calculation
    int32_t vad_camera_id = config_.autoware_to_vad_camera_mapping.at(autoware_camera_id);
    if (vad_camera_id >= 0 && vad_camera_id < num_cameras) {
      std::copy(vad_base2img_flat.begin(), vad_base2img_flat.end(),
                frame_vad_base2img.begin() + vad_camera_id * 16);
    }
  }

  return frame_vad_base2img;
}

} // namespace autoware::tensorrt_vad::vad_interface
