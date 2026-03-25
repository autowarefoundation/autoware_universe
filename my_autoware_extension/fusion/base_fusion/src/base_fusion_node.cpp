#include <rclcpp/rclcpp.hpp>
#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <cmath>
#include <vector>
#include <memory>

using DetectedObjects = autoware_perception_msgs::msg::DetectedObjects;
using DetectedObject = autoware_perception_msgs::msg::DetectedObject;

class BaseObjectFusion : public rclcpp::Node {
public:
    BaseObjectFusion() : Node("base_fusion_node") {
        fusion_distance_threshold_ = this->declare_parameter("fusion_distance_threshold", 2.0);

        rclcpp::QoS sensor_qos = rclcpp::QoS(1).best_effort().durability_volatile();

        sub_vehicle_ = this->create_subscription<DetectedObjects>(
            "/transformed_objects_vehicle", sensor_qos,
            [this](const DetectedObjects::SharedPtr msg) { last_msg_v_ = msg; processFusion(); });
        
        sub_rsu1_ = this->create_subscription<DetectedObjects>(
            "/selected_rsu1_objects", sensor_qos,
            [this](const DetectedObjects::SharedPtr msg) { last_msg_r1_ = msg; });

        sub_rsu2_ = this->create_subscription<DetectedObjects>(
            "/selected_rsu2_objects", sensor_qos,
            [this](const DetectedObjects::SharedPtr msg) { last_msg_r2_ = msg; });

        pub_fused_objects_ = this->create_publisher<DetectedObjects>("/fused_objects", 1);
    }

private:
    double getDistance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2) {
        return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
    }

    void processFusion() {
        if (!last_msg_v_) return;

        DetectedObjects fused_msg;
        fused_msg.header = last_msg_v_->header;

        // 1. 自車の物体をコピーしてベースを作る
        for (const auto& obj : last_msg_v_->objects) {
            fused_msg.objects.push_back(obj);
        }

        bool merged = false;
        if (last_msg_r1_) {
            mergeAndAverage(fused_msg, *last_msg_r1_);
            merged = true;
        }
        if (last_msg_r2_) {
            mergeAndAverage(fused_msg, *last_msg_r2_);
            merged = true;
        }

        if (merged) {
            pub_fused_objects_->publish(fused_msg);
        }
    }

    void mergeAndAverage(DetectedObjects& base_msg, const DetectedObjects& new_msg) {
        double time_diff = std::abs((rclcpp::Time(base_msg.header.stamp) - rclcpp::Time(new_msg.header.stamp)).seconds());
        if (time_diff > 0.3) return;
        for (const auto& new_obj : new_msg.objects) {
            bool is_duplicate = false;
            auto& new_pos = new_obj.kinematics.pose_with_covariance.pose.position;

            for (auto& base_obj : base_msg.objects) {
                auto& base_pos = base_obj.kinematics.pose_with_covariance.pose.position;
                
                if (getDistance(new_pos, base_pos) < fusion_distance_threshold_) {
                    // ★ 座標を足して2で割る (平均化)
                    base_pos.x = (base_pos.x + new_pos.x) / 2.0;
                    base_pos.y = (base_pos.y + new_pos.y) / 2.0;
                    base_pos.z = (base_pos.z + new_pos.z) / 2.0;
                    
                    is_duplicate = true;
                    break;
                }
            }

            if (!is_duplicate) {
                base_msg.objects.push_back(new_obj);
            }
        }
    }

    double fusion_distance_threshold_;
    DetectedObjects::SharedPtr last_msg_v_, last_msg_r1_, last_msg_r2_;
    rclcpp::Subscription<DetectedObjects>::SharedPtr sub_vehicle_, sub_rsu1_, sub_rsu2_;
    rclcpp::Publisher<DetectedObjects>::SharedPtr pub_fused_objects_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BaseObjectFusion>());
    rclcpp::shutdown();
    return 0;
}