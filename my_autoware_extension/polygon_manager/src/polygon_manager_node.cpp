#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <vector>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <string>
#include <map>
#include <set>
#include <algorithm>

namespace fs = std::filesystem;

class PolygonManager : public rclcpp::Node {
public:
    PolygonManager() : Node("polygon_manager_node") {
        auto poly_qos = rclcpp::QoS(10).transient_local();
        pub_all_polygons_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>("/rsu/all_polygons", poly_qos);
        pub_marker_ = this->create_publisher<visualization_msgs::msg::Marker>("/rsu/polygon_markers", 10);

        fs::path home_dir = getenv("HOME");
        save_path_ = (home_dir / "autoware_data" / "polygon_data.yaml").string();

        loadPolygons();

        // 1秒ごとに新しいトピックを自動検知
        discovery_timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&PolygonManager::discoverRsuTopics, this));

        // 5秒ごとにマーカーを再パブリッシュ（RVizが後から接続しても表示されるように）
        marker_republish_timer_ = this->create_wall_timer(
            std::chrono::seconds(5), std::bind(&PolygonManager::republishMarkers, this));

        RCLCPP_INFO(this->get_logger(), "Polygon Manager Started. IDs will be sanitized (no '/') for frame_id.");
    }

private:
    // スラッシュを取り除く/置換するユーティリティ
    std::string sanitizeId(std::string id) {
        if (id.empty()) return "unknown_rsu";
        // 先頭のスラッシュを削除
        if (id[0] == '/') id.erase(0, 1);
        // 残りのスラッシュをアンダースコアに置換
        std::replace(id.begin(), id.end(), '/', '_');
        return id;
    }

    void discoverRsuTopics() {
        auto topic_names_and_types = this->get_topic_names_and_types();
        for (const auto& [name, types] : topic_names_and_types) {
            // "/rsu" で始まるトピックを対象にする
            if (name.find("/rsu") == 0 && name != "/rsu/all_polygons" && name != "/rsu/polygon_markers") {
                if (registered_topics_.find(name) == registered_topics_.end()) {
                    setupRsuSubscription(name);
                }
            }
        }
    }

    void setupRsuSubscription(const std::string& topic_name) {
        registered_topics_.insert(topic_name);
        rsu_buffers_[topic_name] = std::vector<geometry_msgs::msg::Point>();
        
        auto sub = this->create_subscription<geometry_msgs::msg::PointStamped>(
            topic_name, 10,
            [this, topic_name](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
                handleCallback(msg, topic_name);
            });
        
        subs_.push_back(sub);
        RCLCPP_INFO(this->get_logger(), "Auto-subscribed to: %s", topic_name.c_str());
    }

    void handleCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg, const std::string& original_topic) {
        auto& pts = rsu_buffers_[original_topic];
        pts.push_back(msg->point);
        
        if (pts.size() == 4) {
            // IDを安全な形式に変換 (例: "/rsu/area1" -> "rsu_area1")
            std::string safe_id = sanitizeId(original_topic);
            
            size_t hash = std::hash<std::string>{}(safe_id);
            float r = static_cast<float>((hash & 0xFF0000) >> 16) / 255.0f;
            float g = static_cast<float>((hash & 0x00FF00) >> 8) / 255.0f;
            float b = static_cast<float>(hash & 0x0000FF) / 255.0f;

            savePolygon(safe_id, original_topic, msg->header.frame_id, pts);
            publishPolygon(safe_id, msg->header.frame_id, pts, r, g, b);
            pts.clear(); 
            RCLCPP_INFO(this->get_logger(), "Polygon saved with safe frame_id: [%s]", safe_id.c_str());
        }
    }

    void publishPolygon(const std::string& safe_id, const std::string& header_frame, const std::vector<geometry_msgs::msg::Point>& pts, float r, float g, float b) {
        geometry_msgs::msg::PolygonStamped poly;
        poly.header.frame_id = safe_id; // スラッシュなしのID
        poly.header.stamp = this->now();
        for (const auto& p : pts) {
            geometry_msgs::msg::Point32 p32;
            p32.x = p.x; p32.y = p.y; p32.z = 0.0;
            poly.polygon.points.push_back(p32);
        }
        pub_all_polygons_->publish(poly);

        visualization_msgs::msg::Marker m;
        m.header.frame_id = header_frame; 
        m.header.stamp = this->now();
        m.ns = safe_id;
        m.id = 0;
        m.type = visualization_msgs::msg::Marker::LINE_STRIP;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.scale.x = 0.2; m.color.a = 1.0; m.color.r = r; m.color.g = g; m.color.b = b;
        for (const auto& p : pts) { m.points.push_back(p); }
        m.points.push_back(m.points.front());
        pub_marker_->publish(m);
    }

    void savePolygon(const std::string& safe_id, const std::string& original_topic, const std::string& header_frame, const std::vector<geometry_msgs::msg::Point>& pts) {
        YAML::Node config;
        try { if (fs::exists(save_path_)) config = YAML::LoadFile(save_path_); } catch (...) {}
        config[safe_id]["original_topic"] = original_topic;
        config[safe_id]["map_frame"] = header_frame;
        config[safe_id]["points"].reset();
        for (const auto& pt : pts) {
            YAML::Node p_node; p_node["x"] = pt.x; p_node["y"] = pt.y;
            config[safe_id]["points"].push_back(p_node);
        }
        fs::create_directories(fs::path(save_path_).parent_path());
        std::ofstream fout(save_path_); fout << config;
    }

    void loadPolygons() {
        if (!fs::exists(save_path_)) {
            RCLCPP_INFO(this->get_logger(), "No saved YAML found at %s", save_path_.c_str());
            return;
        }

        try {
            YAML::Node config = YAML::LoadFile(save_path_);
            RCLCPP_INFO(this->get_logger(), "Loading polygons from YAML...");

            for (auto it = config.begin(); it != config.end(); ++it) {
                std::string safe_id = it->first.as<std::string>();
                std::string map_frame = it->second["map_frame"].as<std::string>();
                
                std::vector<geometry_msgs::msg::Point> pts;
                for (const auto& p : it->second["points"]) {
                    geometry_msgs::msg::Point pt;
                    pt.x = p["x"].as<double>();
                    pt.y = p["y"].as<double>();
                    pt.z = 0.0;
                    pts.push_back(pt);
                }

                if (pts.empty()) continue;

                // 色の計算（ハッシュ値から生成）
                size_t hash = std::hash<std::string>{}(safe_id);
                float r = static_cast<float>((hash & 0xFF0000) >> 16) / 255.0f;
                float g = static_cast<float>((hash & 0x00FF00) >> 8) / 255.0f;
                float b = static_cast<float>(hash & 0x0000FF) / 255.0f;

                // ★ 確実に配信を実行
                publishPolygon(safe_id, map_frame, pts, r, g, b);
                loaded_polygons_.push_back({safe_id, map_frame, pts, r, g, b});
                
                // 次回の自動検知（discovery）で重複しないように元トピック名で登録
                std::string original_topic = safe_id;
                if (it->second["original_topic"]) {
                    original_topic = it->second["original_topic"].as<std::string>();
                }
                registered_topics_.insert(original_topic);
                
                RCLCPP_INFO(this->get_logger(), "-> Published from YAML: [%s]", safe_id.c_str());
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load YAML: %s", e.what());
        }
    }

    struct PolygonEntry {
        std::string safe_id, frame_id;
        std::vector<geometry_msgs::msg::Point> pts;
        float r, g, b;
    };

    void republishMarkers() {
        for (const auto& e : loaded_polygons_) {
            publishPolygon(e.safe_id, e.frame_id, e.pts, e.r, e.g, e.b);
        }
    }

    std::string save_path_;
    std::set<std::string> registered_topics_;
    std::map<std::string, std::vector<geometry_msgs::msg::Point>> rsu_buffers_;
    std::vector<rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr> subs_;
    std::vector<PolygonEntry> loaded_polygons_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pub_all_polygons_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_;
    rclcpp::TimerBase::SharedPtr discovery_timer_;
    rclcpp::TimerBase::SharedPtr marker_republish_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PolygonManager>());
    rclcpp::shutdown();
    return 0;
}