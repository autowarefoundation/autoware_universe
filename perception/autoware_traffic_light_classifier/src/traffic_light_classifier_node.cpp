// Copyright 2023 TIER IV, Inc.
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
#include "traffic_light_classifier_node.hpp"

#include "classifier_params.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::traffic_light
{
namespace
{
// Set `value` from the parameter named `name` if present; return whether it was found.
bool update_param(
  const std::vector<rclcpp::Parameter> & parameters, const std::string & name, int & value)
{
  for (const auto & parameter : parameters) {
    if (parameter.get_name() == name) {
      value = parameter.as_int();
      return true;
    }
  }
  return false;
}
}  // namespace

TrafficLightClassifierNode::TrafficLightClassifierNode(const rclcpp::NodeOptions & options)
: Node("traffic_light_classifier_node", options)
{
  const auto classify_traffic_light_type =
    static_cast<uint8_t>(this->declare_parameter<int>("traffic_light_type"));

  using std::placeholders::_1;
  using std::placeholders::_2;
  is_approximate_sync_ = this->declare_parameter<bool>("approximate_sync");
  const auto over_exposure_threshold = this->declare_parameter<double>("over_exposure_threshold");
  const auto under_exposure_threshold = this->declare_parameter<double>("under_exposure_threshold");

  if (is_approximate_sync_) {
    approximate_sync_.reset(new ApproximateSync(ApproximateSyncPolicy(10), image_sub_, roi_sub_));
    approximate_sync_->registerCallback(
      std::bind(&TrafficLightClassifierNode::image_roi_callback, this, _1, _2));
  } else {
    sync_.reset(new Sync(SyncPolicy(10), image_sub_, roi_sub_));
    sync_->registerCallback(
      std::bind(&TrafficLightClassifierNode::image_roi_callback, this, _1, _2));
  }

  traffic_signal_array_pub_ = this->create_publisher<tier4_perception_msgs::msg::TrafficLightArray>(
    "~/output/traffic_signals", rclcpp::QoS{1});

  // Subscribe unconditionally; message_filters delivers synchronized image + ROI pairs to
  // image_roi_callback. (Previously a 100 ms timer-driven connectCb subscribed lazily, only while
  // the output had subscribers; always-on is simpler, and the callback no-ops until classifier_
  // is set.)
  image_sub_.subscribe(this, "~/input/image", "raw", rmw_qos_profile_sensor_data);
  roi_sub_.subscribe(this, "~/input/rois", rclcpp::QoS{1}.get_rmw_qos_profile());

  int classifier_type = this->declare_parameter<int>("classifier_type");
  std::shared_ptr<ClassifierInterface> classifier_ptr;
  if (classifier_type == TrafficLightClassifierNode::ClassifierType::HSVFilter) {
    // Keep a typed handle so the node can drive the color backend's dynamic reconfigure.
    color_classifier_ = std::make_shared<ColorClassifier>(declare_hsv_config(this));
    set_param_res_ = this->add_on_set_parameters_callback(
      std::bind(&TrafficLightClassifierNode::on_set_parameters_callback, this, _1));
    classifier_ptr = color_classifier_;
  } else if (classifier_type == TrafficLightClassifierNode::ClassifierType::CNN) {
#if ENABLE_GPU
    classifier_ptr = std::make_shared<CNNClassifier>(declare_cnn_config(this));
#else
    RCLCPP_ERROR(this->get_logger(), "please install CUDA, and TensorRT to use cnn classifier");
#endif
  } else if (classifier_type == TrafficLightClassifierNode::ClassifierType::LampRecognizer) {
#if ENABLE_GPU
    auto lamp_config = declare_lamp_config(this);
    // The recognizer needs its traffic-light type at construction (pedestrian lamps force CIRCLE);
    // it is declared once as its own parameter, so pass it in rather than re-declaring.
    lamp_config.traffic_light_type = classify_traffic_light_type;
    classifier_ptr = std::make_shared<CnnLampRecognizer>(lamp_config);
#else
    RCLCPP_ERROR(
      this->get_logger(), "please install CUDA, CUDNN and TensorRT to use LampRecognizer");
#endif
  }

  if (classifier_ptr) {
    classifier_ = std::make_unique<TrafficLightClassifier>(
      classifier_ptr, classify_traffic_light_type, over_exposure_threshold,
      under_exposure_threshold);
    debug_image_pub_ = image_transport::create_publisher(
      this, "~/output/debug/image", rclcpp::QoS{1}.get_rmw_qos_profile());
  }

  diagnostics_interface_ptr_ =
    std::make_unique<autoware_utils::DiagnosticsInterface>(this, "traffic_light_classifier");

  if (this->declare_parameter<bool>("build_only")) {
    RCLCPP_INFO(get_logger(), "TensorRT engine is built and shutdown node.");
    rclcpp::shutdown();
  }
}

void TrafficLightClassifierNode::image_roi_callback(
  const sensor_msgs::msg::Image::ConstSharedPtr & input_image_msg,
  const tier4_perception_msgs::msg::TrafficLightRoiArray::ConstSharedPtr & input_rois_msg)
{
  if (!classifier_) {
    return;
  }
  auto result = classifier_->classify(*input_image_msg, *input_rois_msg);
  if (!result) {
    RCLCPP_ERROR(this->get_logger(), "failed classify image, abort callback");
    return;
  }

  traffic_signal_array_pub_->publish(result->signals);

  // No rois means classify took its empty-input short-circuit: nothing was classified,
  // so there is nothing to report diagnostics or a debug view for.
  if (input_rois_msg->rois.empty()) {
    return;
  }

  // publish diagnostics
  diagnostics_interface_ptr_->clear();
  diagnostics_interface_ptr_->add_key_value(
    "detect_traffic_light_over_exposure", result->detected_over_exposure);
  diagnostics_interface_ptr_->add_key_value(
    "detect_traffic_light_under_exposure", result->detected_under_exposure);

  if (result->detected_over_exposure || result->detected_under_exposure) {
    diagnostics_interface_ptr_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      "Detected out-of-range exposure in ROI. Corresponding ROI was overwritten with UNKNOWN.");
  }
  diagnostics_interface_ptr_->publish(result->signals.header.stamp);

  // Publish the debug view last, and only when a consumer is attached (building it is a cold path),
  // so a debug-rendering failure cannot skip the primary signal output or diagnostics above.
  if (debug_image_pub_.getNumSubscribers() > 0) {
    const auto debug_image_msg = classifier_->make_debug_image(*result);
    if (debug_image_msg) {
      debug_image_pub_.publish(debug_image_msg);
    }
  }
}

rcl_interfaces::msg::SetParametersResult TrafficLightClassifierNode::on_set_parameters_callback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  HSVConfig config = color_classifier_->get_config();
  update_param(parameters, "green_min_h", config.green_min_h);
  update_param(parameters, "green_min_s", config.green_min_s);
  update_param(parameters, "green_min_v", config.green_min_v);
  update_param(parameters, "green_max_h", config.green_max_h);
  update_param(parameters, "green_max_s", config.green_max_s);
  update_param(parameters, "green_max_v", config.green_max_v);
  update_param(parameters, "yellow_min_h", config.yellow_min_h);
  update_param(parameters, "yellow_min_s", config.yellow_min_s);
  update_param(parameters, "yellow_min_v", config.yellow_min_v);
  update_param(parameters, "yellow_max_h", config.yellow_max_h);
  update_param(parameters, "yellow_max_s", config.yellow_max_s);
  update_param(parameters, "yellow_max_v", config.yellow_max_v);
  update_param(parameters, "red_min_h", config.red_min_h);
  update_param(parameters, "red_min_s", config.red_min_s);
  update_param(parameters, "red_min_v", config.red_min_v);
  update_param(parameters, "red_max_h", config.red_max_h);
  update_param(parameters, "red_max_s", config.red_max_s);
  update_param(parameters, "red_max_v", config.red_max_v);

  color_classifier_->set_config(config);

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

}  // namespace autoware::traffic_light

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::traffic_light::TrafficLightClassifierNode)
