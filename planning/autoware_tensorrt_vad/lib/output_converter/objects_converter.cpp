// Copyright 2025 TIER IV.
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

#include "autoware/tensorrt_vad/output_converter/objects_converter.hpp"
#include <autoware_utils_uuid/uuid_helper.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <cmath>

namespace autoware::tensorrt_vad::vad_interface {

OutputObjectsConverter::OutputObjectsConverter(const CoordinateTransformer& coordinate_transformer, const VadInterfaceConfig& config)
  : Converter(coordinate_transformer, config)
{
  z_offset_ = +1.2f;
}

autoware_perception_msgs::msg::ObjectClassification OutputObjectsConverter::convert_classification(
  const int32_t object_class,
  const float confidence) const
{
  autoware_perception_msgs::msg::ObjectClassification classification;
  
  if (object_class >= 0 && object_class < static_cast<int32_t>(config_.class_mapping.size())) {
    // Get Autoware class name from class mapping array using VAD class index
    const std::string& autoware_class_name = config_.class_mapping[object_class];
    
    // Convert string to Autoware ObjectClassification enum
    if (autoware_class_name == "CAR") {
      classification.label = autoware_perception_msgs::msg::ObjectClassification::CAR;
    } else if (autoware_class_name == "TRUCK") {
      classification.label = autoware_perception_msgs::msg::ObjectClassification::TRUCK;
    } else if (autoware_class_name == "BUS") {
      classification.label = autoware_perception_msgs::msg::ObjectClassification::BUS;
    } else if (autoware_class_name == "BICYCLE") {
      classification.label = autoware_perception_msgs::msg::ObjectClassification::BICYCLE;
    } else if (autoware_class_name == "MOTORCYCLE") {
      classification.label = autoware_perception_msgs::msg::ObjectClassification::MOTORCYCLE;
    } else if (autoware_class_name == "PEDESTRIAN") {
      classification.label = autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN;
    } else {
      classification.label = autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;
    }
  } else {
    classification.label = autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;
  }
  
  classification.probability = confidence;
  return classification;
}

std::optional<float> OutputObjectsConverter::calculate_predicted_path_yaw(
  const BBox& bbox,
  const Eigen::Matrix4d& base2map_transform) const
{
  float max_confidence = 0.0f;
  std::optional<float> predicted_path_yaw = std::nullopt;
  float vad_z = bbox.bbox[4] + bbox.bbox[5] * 0.5f; // object center

  for (const auto& pred_traj : bbox.trajectories) {
    if (pred_traj.confidence > max_confidence) {
      // Calculate direction from first 2 points
      float traj_vad_x1 = pred_traj.trajectory[0][0] + bbox.bbox[0];
      float traj_vad_y1 = pred_traj.trajectory[0][1] + bbox.bbox[1];
      float traj_vad_x2 = pred_traj.trajectory[1][0] + bbox.bbox[0];
      float traj_vad_y2 = pred_traj.trajectory[1][1] + bbox.bbox[1];

      auto [traj_aw_x1, traj_aw_y1, traj_aw_z1] = coordinate_transformer_.vad2aw_xyz(traj_vad_x1, traj_vad_y1, vad_z);
      auto [traj_aw_x2, traj_aw_y2, traj_aw_z2] = coordinate_transformer_.vad2aw_xyz(traj_vad_x2, traj_vad_y2, vad_z);

      Eigen::Vector4d pos1_base(static_cast<double>(traj_aw_x1), static_cast<double>(traj_aw_y1), static_cast<double>(traj_aw_z1), 1.0);
      Eigen::Vector4d pos2_base(static_cast<double>(traj_aw_x2), static_cast<double>(traj_aw_y2), static_cast<double>(traj_aw_z2), 1.0);
      Eigen::Vector4d pos1_map = base2map_transform * pos1_base;
      Eigen::Vector4d pos2_map = base2map_transform * pos2_base;

      float dx = pos2_map.x() - pos1_map.x();
      float dy = pos2_map.y() - pos1_map.y();
      if (std::sqrt(dx*dx + dy*dy) > 0.01) {
        predicted_path_yaw = std::atan2(dy, dx);
        max_confidence = pred_traj.confidence;
      }
    }
  }

  return predicted_path_yaw;
}

geometry_msgs::msg::Point OutputObjectsConverter::convert_position(
  const BBox& bbox,
  const Eigen::Matrix4d& base2map_transform) const
{
    geometry_msgs::msg::Point position;
    // BBox format: [c_x, c_y, w, l, c_z, h, sin(theta), cos(theta), v_x, v_y]
    // [c_x, c_y, c_z] in VAD is bottom center
    float vad_x = bbox.bbox[0];
    float vad_y = bbox.bbox[1];
    float vad_z = bbox.bbox[4] + bbox.bbox[5] * 0.5f; // z + h / 2. object center
    vad_z += z_offset_;
    auto [aw_x, aw_y, aw_z] = coordinate_transformer_.vad2aw_xyz(vad_x, vad_y, vad_z);
    Eigen::Vector4d position_base(static_cast<double>(aw_x), static_cast<double>(aw_y), static_cast<double>(aw_z), 1.0);
    Eigen::Vector4d position_map = base2map_transform * position_base;
    position.x = position_map.x();
    position.y = position_map.y();
    position.z = position_map.z();
    return position;
}

float OutputObjectsConverter::calculate_object_orientation(
  const BBox& bbox,
  const Eigen::Matrix4d& base2map_transform) const
{
  float sin_theta = bbox.bbox[6];
  float cos_theta = bbox.bbox[7];
  float vad_yaw = std::atan2(sin_theta, cos_theta);

  // Get vehicle rotation from base2map_transform
  Eigen::Matrix3d rotation_matrix = base2map_transform.block<3, 3>(0, 0);
  float transform_yaw = std::atan2(rotation_matrix(1, 0), rotation_matrix(0, 0));
  float map_yaw = vad_yaw + transform_yaw;

  return calculate_predicted_path_yaw(bbox, base2map_transform).value_or(map_yaw);
}

geometry_msgs::msg::Twist OutputObjectsConverter::convert_velocity(
  const BBox& bbox) const
{
    geometry_msgs::msg::Twist twist;
    float v_x = bbox.bbox[8];
    float v_y = bbox.bbox[9];
    auto [aw_vx, aw_vy, aw_vz] = coordinate_transformer_.vad2aw_xyz(v_x, v_y, 0.0f);

    twist.linear.x = aw_vx;
    twist.linear.y = aw_vy;
    twist.linear.z = 0.0f;
    return twist;
}

autoware_perception_msgs::msg::Shape OutputObjectsConverter::convert_shape(
  const BBox& bbox)
{
    autoware_perception_msgs::msg::Shape shape;
    shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
    shape.dimensions.x = bbox.bbox[3];  // length
    shape.dimensions.y = bbox.bbox[2];  // width
    shape.dimensions.z = bbox.bbox[5];  // height
    return shape;
}

std::vector<autoware_perception_msgs::msg::PredictedPath> OutputObjectsConverter::convert_predicted_paths(
  const BBox& bbox,
  const Eigen::Matrix4d& base2map_transform,
  const float yaw) const
{
  std::vector<autoware_perception_msgs::msg::PredictedPath> predicted_paths;

  // Set predicted trajectories
  for (const auto& pred_traj : bbox.trajectories) {
    autoware_perception_msgs::msg::PredictedPath predicted_path;
    predicted_path.confidence = pred_traj.confidence;
    
    // Set time step (assuming 0.1 seconds per step for future trajectory)
    predicted_path.time_step.sec = 0;
    predicted_path.time_step.nanosec = 100000000;  // 0.1 seconds in nanoseconds
    
    // Transform each point of the trajectory
    for (int32_t ts = 0; ts < 6; ++ts) {
      geometry_msgs::msg::Pose pose;
      
      // Predicted trajectory is in relative coordinates (ego coordinate system), so transform to agent center
      float traj_vad_x = pred_traj.trajectory[ts][0] + bbox.bbox[0];  // Relative coordinates from agent center
      float traj_vad_y = pred_traj.trajectory[ts][1] + bbox.bbox[1];  // Relative coordinates from agent center
      float traj_vad_z = bbox.bbox[4] + bbox.bbox[5] * 0.5f; // z + h / 2. object center
      traj_vad_z += z_offset_;
      auto [traj_aw_x, traj_aw_y, traj_aw_z] = coordinate_transformer_.vad2aw_xyz(traj_vad_x, traj_vad_y, traj_vad_z);

      // Transform to map coordinate system
      Eigen::Vector4d traj_position_base(static_cast<double>(traj_aw_x), static_cast<double>(traj_aw_y), static_cast<double>(traj_aw_z), 1.0);
      Eigen::Vector4d traj_position_map = base2map_transform * traj_position_base;
      
      pose.position.x = traj_position_map.x();
      pose.position.y = traj_position_map.y();
      pose.position.z = traj_position_map.z();
      
      // Calculate trajectory direction (direction to next point)
      // Default is object direction (using corrected yaw)
      float traj_yaw = yaw;
      
      if (ts < 5) {  // If next point exists
        float next_vad_x = pred_traj.trajectory[ts + 1][0] + bbox.bbox[0];
        float next_vad_y = pred_traj.trajectory[ts + 1][1] + bbox.bbox[1];
        auto [next_aw_x, next_aw_y, next_aw_z] = coordinate_transformer_.vad2aw_xyz(next_vad_x, next_vad_y, traj_vad_z);
        
        // Transform next point to map coordinate system
        Eigen::Vector4d next_position_base(static_cast<double>(next_aw_x), static_cast<double>(next_aw_y), static_cast<double>(next_aw_z), 1.0);
        Eigen::Vector4d next_position_map = base2map_transform * next_position_base;
        
        // Calculate direction vector in map coordinate system
        float dx = next_position_map.x() - traj_position_map.x();
        float dy = next_position_map.y() - traj_position_map.y();
        traj_yaw = std::atan2(dy, dx);
      }
      
      pose.orientation = autoware_utils::create_quaternion_from_yaw(traj_yaw);
      
      predicted_path.path.push_back(pose);
    }
    
    // Add trajectory only if it contains points
    if (!predicted_path.path.empty()) {
      predicted_paths.push_back(predicted_path);
    }
  }

  return predicted_paths;
}

autoware_perception_msgs::msg::PredictedObjects OutputObjectsConverter::process_predicted_objects(
  const std::vector<BBox>& bboxes,
  const rclcpp::Time& stamp,
  const Eigen::Matrix4d& base2map_transform) const
{
  autoware_perception_msgs::msg::PredictedObjects predicted_objects;
  // Set header
  predicted_objects.header.stamp = stamp;
  predicted_objects.header.frame_id = "map";
  
  // Object class name mapping
  for (const auto& bbox : bboxes) {
    autoware_perception_msgs::msg::PredictedObject predicted_object;
    
    // Set object ID
    predicted_object.object_id = autoware_utils_uuid::generate_uuid();
    
    // Set existence probability
    predicted_object.existence_probability = bbox.confidence;
    
    // Set classification
    predicted_object.classification.push_back(convert_classification(bbox.object_class, bbox.confidence));

    // Set position
    predicted_object.kinematics.initial_pose_with_covariance.pose.position = convert_position(bbox, base2map_transform);

    // Set orientation
    float yaw = calculate_object_orientation(bbox, base2map_transform);
    predicted_object.kinematics.initial_pose_with_covariance.pose.orientation = autoware_utils::create_quaternion_from_yaw(yaw);

    // Set shape
    predicted_object.shape = convert_shape(bbox);

    // Set velocity
    predicted_object.kinematics.initial_twist_with_covariance.twist = convert_velocity(bbox);

    // Process predicted trajectories
    predicted_object.kinematics.predicted_paths = convert_predicted_paths(bbox, base2map_transform, yaw);

    predicted_objects.objects.push_back(predicted_object);
  }

  return predicted_objects;
}

} // namespace autoware::tensorrt_vad::vad_interface
