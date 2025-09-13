#include "autoware/tensorrt_vad/vad_interface.hpp"
#include <opencv2/opencv.hpp>

namespace autoware::tensorrt_vad
{

VadInterface::VadInterface(const VadInterfaceConfig& config, const std::shared_ptr<tf2_ros::Buffer> tf_buffer)
  : config_(config),
    prev_can_bus_(),
    vad_base2img_transform_(std::nullopt)
{
  // Initialize coordinate transformer
  coordinate_transformer_ = std::make_unique<vad_interface::CoordinateTransformer>(
    config.vad2base, config.base2vad, tf_buffer);
  
  // Initialize input converters
  input_image_converter_ = std::make_unique<vad_interface::InputImageConverter>(*coordinate_transformer_, config_);
  input_transform_matrix_converter_ = std::make_unique<vad_interface::InputTransformMatrixConverter>(*coordinate_transformer_, config_);
  input_can_bus_converter_ = std::make_unique<vad_interface::InputCanBusConverter>(*coordinate_transformer_, config_);
  input_bev_shift_converter_ = std::make_unique<vad_interface::InputBEVShiftConverter>(*coordinate_transformer_, config_);
  
  // Initialize output converters  
  output_trajectory_converter_ = std::make_unique<vad_interface::OutputTrajectoryConverter>(*coordinate_transformer_, config_);
  output_map_converter_ = std::make_unique<vad_interface::OutputMapConverter>(*coordinate_transformer_, config_);
  output_objects_converter_ = std::make_unique<vad_interface::OutputObjectsConverter>(*coordinate_transformer_, config_);
}

VadInputData VadInterface::convert_input(const VadInputTopicData & vad_input_topic_data)
{
  VadInputData vad_input_data;

  // Process vad_base2img transformation using converter, with caching
  if (!vad_base2img_transform_.has_value()) {
    vad_base2img_transform_ = input_transform_matrix_converter_->process_vad_base2img(
      vad_input_topic_data.camera_infos
    );
  }
  vad_input_data.vad_base2img = vad_base2img_transform_.value();
  
  // Process can_bus using converter
  vad_input_data.can_bus = input_can_bus_converter_->process_can_bus(
    vad_input_topic_data.kinematic_state,
    vad_input_topic_data.acceleration,
    prev_can_bus_
  );
  
  // Process shift using converter
  vad_input_data.shift = input_bev_shift_converter_->process_shift(vad_input_data.can_bus, prev_can_bus_);
  
  // Process image data using converter
  vad_input_data.camera_images = input_image_converter_->process_image(vad_input_topic_data.images);
  
  // Set default command
  vad_input_data.command = config_.default_command;
  
  // Update prev_can_bus_ for next iteration
  prev_can_bus_ = vad_input_data.can_bus;
  
  return vad_input_data;
}

VadOutputTopicData VadInterface::convert_output(
  const VadOutputData & vad_output_data,
  const rclcpp::Time & stamp,
  const double trajectory_timestep,
  const Eigen::Matrix4d & base2map_transform) const
{
  VadOutputTopicData vad_output_topic_data;

  // Convert candidate trajectories using converter
  vad_output_topic_data.candidate_trajectories = output_trajectory_converter_->process_candidate_trajectories(
    vad_output_data.predicted_trajectories, stamp, trajectory_timestep, base2map_transform);
  
  // Convert trajectory using converter
  vad_output_topic_data.trajectory = output_trajectory_converter_->process_trajectory(
    vad_output_data.predicted_trajectory, stamp, trajectory_timestep, base2map_transform);

  // Convert map_points using converter
  vad_output_topic_data.map_points = output_map_converter_->process_map_points(vad_output_data.map_polylines, stamp, base2map_transform);

  // Convert predicted objects using converter
  vad_output_topic_data.objects = output_objects_converter_->process_predicted_objects(vad_output_data.predicted_objects, stamp, base2map_transform);

  return vad_output_topic_data;
}

} // namespace autoware::tensorrt_vad
