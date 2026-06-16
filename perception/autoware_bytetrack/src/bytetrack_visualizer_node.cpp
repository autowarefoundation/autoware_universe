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

#include "autoware/bytetrack/bytetrack_visualizer_node.hpp"

#include <autoware_perception_msgs/msg/object_classification.hpp>

#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <algorithm>
#include <memory>
#include <vector>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <chrono>
#include <string>

namespace autoware::bytetrack
{
namespace
{
cv::Scalar getColorByClassId(const int class_id)
{
  using Classification = autoware_perception_msgs::msg::ObjectClassification;

  switch (class_id) {
    case Classification::UNKNOWN:
      return cv::Scalar(160, 160, 160);
    case Classification::CAR:
      return cv::Scalar(0, 200, 0);
    case Classification::TRUCK:
      return cv::Scalar(255, 128, 0);
    case Classification::BUS:
      return cv::Scalar(0, 255, 255);
    case Classification::TRAILER:
      return cv::Scalar(128, 0, 255);
    case Classification::MOTORCYCLE:
      return cv::Scalar(255, 0, 255);
    case Classification::BICYCLE:
      return cv::Scalar(255, 255, 0);
    case Classification::PEDESTRIAN:
      return cv::Scalar(0, 0, 255);
    case Classification::ANIMAL:
      return cv::Scalar(0, 165, 255);
    case Classification::HAZARD:
      return cv::Scalar(0, 255, 128);
    case Classification::OVER_DRIVABLE:
      return cv::Scalar(255, 0, 0);
    case Classification::UNDER_DRIVABLE:
      return cv::Scalar(180, 80, 80);
    default:
      return cv::Scalar(255, 255, 255);
  }
}
}  // namespace

ByteTrackVisualizerNode::ByteTrackVisualizerNode(const rclcpp::NodeOptions & node_options)
: Node("bytetrack_visualizer", node_options)
{
  using std::chrono_literals::operator""ms;

  use_raw_ = declare_parameter<bool>("use_raw");

  // Create timer to find proper settings for subscribed topics
  timer_ = rclcpp::create_timer(
    this, get_clock(), 100ms, std::bind(&ByteTrackVisualizerNode::on_timer, this));

  image_pub_ = image_transport::create_publisher(this, "~/out/image");
}

ByteTrackVisualizerNode::~ByteTrackVisualizerNode()
{
  cv::destroyAllWindows();
}

bool ByteTrackVisualizerNode::get_topic_qos(const std::string & query_topic, rclcpp::QoS & qos)
{
  auto publisher_info = this->get_publishers_info_by_topic(query_topic);
  if (publisher_info.size() < 1) {
    return false;
  } else if (publisher_info.size() > 1) {
    RCLCPP_ERROR_STREAM(
      get_logger(),
      query_topic << " seems to have more than 1 publisher." << this->get_name()
                  << "will be terminated since it could not determine the topic source.");
    rclcpp::shutdown();
  }

  qos = publisher_info[0].qos_profile();
  return true;
}

void ByteTrackVisualizerNode::on_timer()
{
  auto ResolveTopicName = [this](const std::string & query) {
    return this->get_node_topics_interface()->resolve_topic_name(query, false);
  };

  std::string image_topic = ResolveTopicName("~/in/image");
  std::string image_topic_for_qos_query = image_topic;
  if (!use_raw_) {
    // Use the other instance because image_transport adds '/compressed' post-fix automatically
    image_topic_for_qos_query += "/compressed";
  }
  rclcpp::QoS image_qos(0);
  bool is_image_query_succeeded = get_topic_qos(image_topic_for_qos_query, image_qos);

  std::string rect_topic = ResolveTopicName("~/in/rect");
  rclcpp::QoS rect_qos(0);
  bool is_rect_query_succeeded = get_topic_qos(rect_topic, rect_qos);

  std::string uuid_topic = ResolveTopicName("~/in/uuid");
  rclcpp::QoS uuid_qos(0);
  bool is_uuid_query_succeeded = get_topic_qos(uuid_topic, uuid_qos);

  if (is_image_query_succeeded && is_rect_query_succeeded && is_uuid_query_succeeded) {
    // All queries are succeeded. Stop the timer and start subscribing
    std::string conversion_type = use_raw_ ? "raw" : "compressed";
    image_sub_.subscribe(this, image_topic, conversion_type, image_qos.get_rmw_qos_profile());
    rect_sub_.subscribe(this, rect_topic, rect_qos.get_rmw_qos_profile());
    uuid_sub_.subscribe(this, uuid_topic, uuid_qos.get_rmw_qos_profile());
    // Since rect and uuid topics inherit the header information from the source image,
    // ExactTime sync policy suits the purpose
    sync_ptr_ =
      std::make_shared<ExactTimeSync>(ExactTimeSyncPolicy(10), image_sub_, rect_sub_, uuid_sub_);
    sync_ptr_->registerCallback(&ByteTrackVisualizerNode::callback, this);

    timer_->cancel();
  } else {
    RCLCPP_INFO_STREAM(
      get_logger(),
      "Subscribed topics seem not to be ready:"
        << std::endl
        << image_topic << ": " << (is_image_query_succeeded ? "ready" : "NOT ready") << std::endl
        << rect_topic << ": " << (is_rect_query_succeeded ? "ready" : "NOT ready") << std::endl
        << uuid_topic << ": " << (is_uuid_query_succeeded ? "ready" : "NOT ready") << std::endl);
  }
}

void ByteTrackVisualizerNode::callback(
  const sensor_msgs::msg::Image::SharedPtr & image_msg,
  const tier4_perception_msgs::msg::DetectedObjectsWithFeature::SharedPtr & rect_msg,
  const tier4_perception_msgs::msg::DynamicObjectArray::SharedPtr & uuid_msg)
{
  // Extract data from received messages
  cv_bridge::CvImagePtr in_image_ptr;
  try {
    auto desired_encoding = sensor_msgs::image_encodings::BGR8;
    in_image_ptr = cv_bridge::toCvCopy(image_msg, desired_encoding);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  std::vector<cv::Rect> bboxes;
  std::vector<int> class_ids;
  for (const auto & feat_obj : rect_msg->feature_objects) {
    auto roi_msg = feat_obj.feature.roi;
    cv::Rect rect(roi_msg.x_offset, roi_msg.y_offset, roi_msg.width, roi_msg.height);
    bboxes.push_back(rect);
    if (feat_obj.object.classification.empty()) {
      class_ids.push_back(autoware_perception_msgs::msg::ObjectClassification::UNKNOWN);
    } else {
      class_ids.push_back(feat_obj.object.classification.front().label);
    }
  }

  std::vector<boost::uuids::uuid> uuids;
  for (const auto & dynamic_obj : uuid_msg->objects) {
    auto uuid_msg = dynamic_obj.id.uuid;
    boost::uuids::uuid uuid_raw;
    std::copy(uuid_msg.begin(), uuid_msg.end(), uuid_raw.begin());
    uuids.push_back(uuid_raw);
  }

  // Draw results and publish it
  cv::Mat image = in_image_ptr->image;
  draw(image, bboxes, class_ids, uuids);

  cv_bridge::CvImage pub_image_msg;
  pub_image_msg.header = image_msg->header;
  pub_image_msg.image = image;
  pub_image_msg.encoding = sensor_msgs::image_encodings::BGR8;
  image_pub_.publish(pub_image_msg.toImageMsg());
}

void ByteTrackVisualizerNode::draw(
  cv::Mat & image, const std::vector<cv::Rect> & bboxes, const std::vector<int> & class_ids,
  const std::vector<boost::uuids::uuid> & uuids)
{
  for (size_t idx = 0; idx < bboxes.size(); idx++) {
    auto bbox = bboxes[idx];
    auto class_id = class_ids[idx];
    auto uuid = uuids[idx];

    auto uuid_str = boost::lexical_cast<std::string>(uuid);
    // Take sub string because full UUID is too long to display
    uuid_str = uuid_str.substr(0, 5);
    auto color = getColorByClassId(class_id);

    const auto left = std::max(0, static_cast<int>(bbox.x));
    const auto top = std::max(0, static_cast<int>(bbox.y));
    const auto right = std::min(static_cast<int>(bbox.x + bbox.width), image.size().width);
    const auto bottom = std::min(static_cast<int>(bbox.y + bbox.height), image.size().height);

    constexpr uint font_thickness = 1;
    constexpr uint bbox_thickness = 2;
    cv::rectangle(image, cv::Point(left, top), cv::Point(right, bottom), color, bbox_thickness);
    cv::putText(
      image, cv::format("ID: %s", uuid_str.c_str()), cv::Point(left, top - 5),
      cv::FONT_HERSHEY_SIMPLEX,
      1,  // font scale
      color,
      font_thickness,  // thickness
      cv::LINE_AA);
  }
}

}  // namespace autoware::bytetrack

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::bytetrack::ByteTrackVisualizerNode)
