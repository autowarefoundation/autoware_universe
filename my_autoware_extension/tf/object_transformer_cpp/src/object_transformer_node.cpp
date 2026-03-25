#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "autoware_perception_msgs/msg/detected_objects.hpp"

using std::placeholders::_1;

class ObjectTransformerNode : public rclcpp::Node
{
public:
  using DetectedObjects = autoware_perception_msgs::msg::DetectedObjects;

  ObjectTransformerNode()
  : Node("object_transformer_node")
  {
    // パラメータ: ターゲットフレーム (デフォルト: map)
    target_frame_id_ = this->declare_parameter<std::string>("target_frame_id", "map");

    // QoS設定
    rclcpp::QoS qos_profile(1);
    qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    qos_profile.history(rclcpp::HistoryPolicy::KeepLast);

    // TF設定
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ★ここが変更点: 固定の名前で作成します (あとでLaunchでリマップするため)
    pub_ = this->create_publisher<DetectedObjects>("output/objects", qos_profile);
    
    sub_ = this->create_subscription<DetectedObjects>(
      "input/objects", 
      qos_profile,
      std::bind(&ObjectTransformerNode::callback, this, _1)
    );

    RCLCPP_INFO(this->get_logger(), "Transformer initialized. Target: %s", target_frame_id_.c_str());
  }

private:
  void callback(const DetectedObjects::SharedPtr msg)
  {
    std::string source_frame_id = msg->header.frame_id;

    if (source_frame_id.empty()) return;

    if (msg->objects.empty()) {
      return; 
    }

    // 変換不要ならそのままスルー
    if (source_frame_id == target_frame_id_) {
      pub_->publish(*msg);
      return;
    }

    DetectedObjects transformed_msg;
    transformed_msg.header = msg->header;
    transformed_msg.header.frame_id = target_frame_id_;

    try {
      auto transform = tf_buffer_->lookupTransform(
        target_frame_id_, source_frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.1));

      for (const auto & obj : msg->objects) {
        geometry_msgs::msg::PoseStamped pose_in, pose_out;
        pose_in.header = msg->header;
        pose_in.pose = obj.kinematics.pose_with_covariance.pose;
        
        tf2::doTransform(pose_in, pose_out, transform);

        auto new_obj = obj;
        new_obj.kinematics.pose_with_covariance.pose = pose_out.pose;
        transformed_msg.objects.push_back(new_obj);
      }
      
      pub_->publish(transformed_msg);

    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "TF Error: %s", ex.what());
    }
  }

  std::string target_frame_id_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<DetectedObjects>::SharedPtr pub_;
  rclcpp::Subscription<DetectedObjects>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectTransformerNode>());
  rclcpp::shutdown();
  return 0;
}