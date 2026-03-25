#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>

#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <optional>
#include <vector>

// ROS 2のメッセージ型エイリアス
using DetectedObjects = autoware_perception_msgs::msg::DetectedObjects;
using DetectedObject = autoware_perception_msgs::msg::DetectedObject;
using Point = geometry_msgs::msg::Point;

// RSUの固定位置と検知範囲を格納する構造体
struct RsuConfig {
    double x, y, z;
    std::vector<Point> polygon_points;
    bool is_ready = false;
};

// ====================================================================
// ノードクラス定義
// ====================================================================
class DistanceSelectiveRsu : public rclcpp::Node {
public:
    DistanceSelectiveRsu() : Node("distance_selective_rsu_node") {
        
        // --- 1. パラメータの宣言と取得 ---
        
        rsu1_config_.x = this->declare_parameter("rsu1_pos_x", 0.0);
        rsu1_config_.y = this->declare_parameter("rsu1_pos_y", 0.0);
        rsu1_config_.z = this->declare_parameter("rsu1_pos_z", 0.0);
        
        rsu2_config_.x = this->declare_parameter("rsu2_pos_x", 0.0);
        rsu2_config_.y = this->declare_parameter("rsu2_pos_y", 0.0);
        rsu2_config_.z = this->declare_parameter("rsu2_pos_z", 0.0);
        
        association_threshold_ = this->declare_parameter("association_threshold", 3);
        buffer_size_ = this->declare_parameter("buffer_size", 20); // 保持する履歴数
        max_age_sec_ = this->declare_parameter("max_age_sec", 0.3); // 許容する最大時間差
        rsu1_frame_id_ = this->declare_parameter("rsu1_frame_id", std::string("rsu1"));
        rsu2_frame_id_ = this->declare_parameter("rsu2_frame_id", std::string("rsu2"));

        // --- 2. 購読の設定 ---
        
        rclcpp::QoS sensor_qos = rclcpp::QoS(1).best_effort().durability_volatile();

        sub_vehicle_ = this->create_subscription<DetectedObjects>(
            "/transformed_objects_vehicle", sensor_qos,
            std::bind(&DistanceSelectiveRsu::vehicleCallback, this, std::placeholders::_1));
        sub_rsu1_ = this->create_subscription<DetectedObjects>(
            "/transformed_objects_rsu1", sensor_qos,
            [this](const DetectedObjects::SharedPtr msg) { 
                rsu1_buffer_.push_back(msg);
                if (rsu1_buffer_.size() > buffer_size_) rsu1_buffer_.pop_front();
            });
        sub_rsu2_ = this->create_subscription<DetectedObjects>(
            "/transformed_objects_rsu2", sensor_qos,
            [this](const DetectedObjects::SharedPtr msg) {
                rsu2_buffer_.push_back(msg);
                if (rsu2_buffer_.size() > buffer_size_) rsu2_buffer_.pop_front();
            });
        
        auto poly_qos = rclcpp::QoS(10).transient_local();

        sub_all_polygons_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
            "/rsu/all_polygons", poly_qos, 
            [this](const geometry_msgs::msg::PolygonStamped::SharedPtr msg) {
                // frame_id を見て、rsu1かrsu2かを判定して更新する
                if (msg->header.frame_id == rsu1_frame_id_) {
                    updatePolygon(rsu1_config_, msg, "RSU1");
                } else if (msg->header.frame_id == rsu2_frame_id_) {
                    updatePolygon(rsu2_config_, msg, "RSU2");
                }
            });

        // --- 3. 出版の設定 ---
        pub_selected_rsu1_ = this->create_publisher<DetectedObjects>("/selected_rsu1_objects", 1);
        pub_selected_rsu2_ = this->create_publisher<DetectedObjects>("/selected_rsu2_objects", 1);
        pub_vehicle_relay_ = this->create_publisher<DetectedObjects>("/transformed_objects_vehicle_out", 1);
    }

private:
    // 購読/出版
    rclcpp::Subscription<DetectedObjects>::SharedPtr sub_vehicle_, sub_rsu1_, sub_rsu2_;
    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr sub_all_polygons_;
    rclcpp::Publisher<DetectedObjects>::SharedPtr pub_selected_rsu1_, pub_selected_rsu2_, pub_vehicle_relay_;
    // 履歴保持用のバッファ
    std::deque<DetectedObjects::SharedPtr> rsu1_buffer_, rsu2_buffer_;
    size_t buffer_size_;
    double max_age_sec_;

    RsuConfig rsu1_config_, rsu2_config_;
    std::vector<Point> rsu1_temp_points_, rsu2_temp_points_;
    double association_threshold_;
    std::string rsu1_frame_id_, rsu2_frame_id_;
    const int POLYGON_POINTS_COUNT = 4;

    void updatePolygon(RsuConfig& config, const geometry_msgs::msg::PolygonStamped::SharedPtr msg, const std::string& name) {
        config.polygon_points.clear();
        for (const auto& p32 : msg->polygon.points) {
            Point p;
            p.x = p32.x; p.y = p32.y; p.z = p32.z;
            config.polygon_points.push_back(p);
        }
        config.is_ready = true;
        RCLCPP_INFO(this->get_logger(), "%s Polygon updated from Manager. Points: %zu", name.c_str(), config.polygon_points.size());
    }

    // --- ユーティリティ関数 ---

    // RSU位置までのユークリッド距離計算 (RSU Configを使用)
    double calculateDistance(const Point& p_obj, const RsuConfig& rsu_config) {
        return std::sqrt(
            std::pow(p_obj.x - rsu_config.x, 2) + 
            std::pow(p_obj.y - rsu_config.y, 2) + 
            std::pow(p_obj.z - rsu_config.z, 2)
        );
    }
    
    // 物体間の距離計算 (関連付け用)
    double calculateDistance(const Point& p1, const Point& p2) {
        return std::sqrt(
            std::pow(p1.x - p2.x, 2) + 
            std::pow(p1.y - p2.y, 2) + 
            std::pow(p1.z - p2.z, 2)
        );
    }

    bool isPointInPolygon(const Point& target_point, const std::vector<Point>& polygon) {
        if (polygon.size() < 3) return false;
        
        int intersect_count = 0;
        const double target_y = target_point.y;
        const double target_x = target_point.x;
        
        for (size_t i = 0; i < polygon.size(); ++i) {
            const Point& p1 = polygon[i];
            const Point& p2 = polygon[(i + 1) % polygon.size()];

            // 1. 線分 p1-p2 が目標点の Y 座標をまたいでいるかを確認 (交差の可能性)
            bool y_range_ok = ((p1.y <= target_y && p2.y > target_y) || 
                               (p2.y <= target_y && p1.y > target_y));
            
            if (y_range_ok) {
                // 2. 目標の Y における線分上の X 座標 (x_intersect) を計算
                // X = X1 + (X2 - X1) * (Target_Y - Y1) / (Y2 - Y1)
                double x_intersect = p1.x + (p2.x - p1.x) * (target_y - p1.y) / (p2.y - p1.y);

                // 3. 交点 x_intersect が目標点の X より右側にあるかを確認 (奇数回交差判定)
                if (x_intersect > target_x) {
                    intersect_count++;
                }
            }
        }
        
        // 交差カウントが奇数であれば、点がポリゴン内部にある
        return (intersect_count % 2 == 1);
    }

    // RSUリストから関連付けられる物体を検索（IoUの代わりに距離を使用）
    std::optional<DetectedObject> findBestMatchFromBuffer(const DetectedObject& target_obj, const rclcpp::Time& v_time, const std::deque<DetectedObjects::SharedPtr>& buffer) {
        const auto& t_pos = target_obj.kinematics.pose_with_covariance.pose.position;

        // 時間的に最も近いメッセージを1つ選ぶ
        const DetectedObjects* closest_msg = nullptr;
        double min_dt = std::numeric_limits<double>::max();
        for (const auto& r_msg : buffer) {
            double dt = std::abs((v_time - rclcpp::Time(r_msg->header.stamp)).seconds());
            if (dt <= max_age_sec_ && dt < min_dt) {
                min_dt = dt;
                closest_msg = r_msg.get();
            }
        }
        if (!closest_msg) return std::nullopt;

        // そのメッセージ内で最も近い物体を探す
        double min_dist = std::numeric_limits<double>::max();
        std::optional<DetectedObject> best_match = std::nullopt;
        for (const auto& r_obj : closest_msg->objects) {
            double dist = calculateDistance(t_pos, r_obj.kinematics.pose_with_covariance.pose.position);
            if (dist < min_dist) {
                min_dist = dist;
                best_match = r_obj;
            }
        }

        if (min_dist <= association_threshold_) {
            return best_match;
        }
        return std::nullopt;
    }

    // --- メイン処理: 車両データが届いた時に実行 ---
    void vehicleCallback(const DetectedObjects::SharedPtr msg_v) {
        pub_vehicle_relay_->publish(*msg_v);
        if (!rsu1_config_.is_ready && !rsu2_config_.is_ready) return;

        rclcpp::Time v_time(msg_v->header.stamp);
        DetectedObjects output1, output2;
        output1.header = msg_v->header;
        output2.header = msg_v->header;

        for (const auto& obj_v : msg_v->objects) {
            const auto& pos = obj_v.kinematics.pose_with_covariance.pose.position;
            bool in_r1 = rsu1_config_.is_ready && isPointInPolygon(pos, rsu1_config_.polygon_points);
            bool in_r2 = rsu2_config_.is_ready && isPointInPolygon(pos, rsu2_config_.polygon_points);

            if (in_r1 && in_r2) {
                // 1. まず両方のRSUバッファから、この車両(obj_v)に対応するオブジェクトを探す
                auto match1 = findBestMatchFromBuffer(obj_v, v_time, rsu1_buffer_);
                auto match2 = findBestMatchFromBuffer(obj_v, v_time, rsu2_buffer_);
                // 2. 両方で見つかった場合 (m かつ n)
                if (match1 && match2) {
                    RCLCPP_INFO(this->get_logger(), 
                        "Overlap Match (RSU1 & RSU2) detected.\n"
                        "  Time: %.3f\n"
                        "  Location: x=%.2f, y=%.2f, z=%.2f",
                        v_time.seconds(), pos.x, pos.y, pos.z);
                    // 判定基準: 車両位置からRSUまでの物理的距離を比較
                    // (センサーに近い方が一般的に精度が高いため、そちらを採用する)
                    double d1 = calculateDistance(pos, rsu1_config_);
                    double d2 = calculateDistance(pos, rsu2_config_);

                    if (d1 < d2) {
                        output1.objects.push_back(*match1); // RSU1を採用
                        // RCLCPP_INFO(this->get_logger(), "Overlap: Both found. Selected RSU1 (Closer).");
                    } else {
                        output2.objects.push_back(*match2); // RSU2を採用
                        // RCLCPP_INFO(this->get_logger(), "Overlap: Both found. Selected RSU2 (Closer).");
                    }
                }
                // 3. RSU1だけで見つかった場合
                else if (match1) {
                    output1.objects.push_back(*match1);
                }
                // 4. RSU2だけで見つかった場合
                else if (match2) {
                    output2.objects.push_back(*match2);
                }
                // (どちらも見つからなければ何もしない)

            } else if (in_r1) {
                auto m = findBestMatchFromBuffer(obj_v, v_time, rsu1_buffer_);
                if (m) output1.objects.push_back(*m);
            } else if (in_r2) {
                auto m = findBestMatchFromBuffer(obj_v, v_time, rsu2_buffer_);
                if (m) output2.objects.push_back(*m);
            }
        }
        if (!output1.objects.empty()) pub_selected_rsu1_->publish(output1);
        if (!output2.objects.empty()) pub_selected_rsu2_->publish(output2);
    }
};

// ====================================================================
// メイン関数
// ====================================================================
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DistanceSelectiveRsu>());
    rclcpp::shutdown();
    return 0;
}