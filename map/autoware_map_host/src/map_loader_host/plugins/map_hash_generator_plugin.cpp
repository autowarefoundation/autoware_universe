// Copyright 2026 TIER IV, Inc.
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

#include "autoware/map_loader/map_loader_plugin_base.hpp"

#include <autoware_utils/ros/parameter.hpp>
#include <pluginlib/class_list_macros.hpp>

#include "tier4_external_api_msgs/msg/map_hash.hpp"
#include "tier4_external_api_msgs/msg/response_status.hpp"
#include "tier4_external_api_msgs/srv/get_text_file.hpp"

#include <openssl/evp.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace autoware::map_loader::plugin
{
namespace fs = std::filesystem;

class MapHashGeneratorPlugin : public MapLoaderPluginBase
{
public:
  void initialize(
    const std::string & /*name*/, rclcpp::Node * node, MapLoaderData & /*data*/) override
  {
    node_ = node;
    using autoware_utils_rclcpp::get_or_declare_parameter;
    lanelet2_map_path_ = get_or_declare_parameter<std::string>(*node, "lanelet2_map_path");
    pointcloud_map_path_ = get_or_declare_parameter<std::string>(*node, "pointcloud_map_path");

    lanelet_text_ = load_lanelet_text(lanelet2_map_path_);
    lanelet_hash_ = generate_lanelet_file_hash(lanelet_text_);
    pcd_hash_ = generate_pcd_file_hash(pointcloud_map_path_);

    const auto qos = rclcpp::QoS(1).transient_local();
    hash_pub_ = node->create_publisher<tier4_external_api_msgs::msg::MapHash>(
      "/api/autoware/get/map/info/hash", qos);

    lanelet_xml_srv_ = node->create_service<tier4_external_api_msgs::srv::GetTextFile>(
      "/api/autoware/get/map/lanelet/xml", std::bind(
                                             &MapHashGeneratorPlugin::on_get_lanelet_xml, this,
                                             std::placeholders::_1, std::placeholders::_2));
  }

  void on_startup(MapLoaderData & /*data*/) override
  {
    tier4_external_api_msgs::msg::MapHash msg;
    msg.lanelet = lanelet_hash_;
    msg.pcd = pcd_hash_;
    hash_pub_->publish(msg);
    RCLCPP_INFO(node_->get_logger(), "Published map hash.");
  }

private:
  rclcpp::Node * node_{nullptr};
  std::string lanelet2_map_path_;
  std::string pointcloud_map_path_;
  std::string lanelet_text_;
  std::string lanelet_hash_;
  std::string pcd_hash_;
  rclcpp::Publisher<tier4_external_api_msgs::msg::MapHash>::SharedPtr hash_pub_;
  rclcpp::Service<tier4_external_api_msgs::srv::GetTextFile>::SharedPtr lanelet_xml_srv_;

  static std::string load_lanelet_text(const std::string & path)
  {
    const fs::path file_path(path);
    if (!fs::is_regular_file(file_path)) {
      return "";
    }
    std::ifstream ifs(file_path);
    std::stringstream buffer;
    buffer << ifs.rdbuf();
    return buffer.str();
  }

  static std::string sha256_hex(const std::string & data)
  {
    unsigned char hash[EVP_MAX_MD_SIZE];
    unsigned int hash_len = 0;
    EVP_MD_CTX * context = EVP_MD_CTX_new();
    EVP_DigestInit_ex(context, EVP_sha256(), nullptr);
    EVP_DigestUpdate(context, data.data(), data.size());
    EVP_DigestFinal_ex(context, hash, &hash_len);
    EVP_MD_CTX_free(context);

    std::ostringstream oss;
    for (unsigned int i = 0; i < hash_len; ++i) {
      oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(hash[i]);
    }
    return oss.str();
  }

  static std::string sha256_file_hex(const std::string & path)
  {
    std::ifstream file(path, std::ios::binary);
    if (!file) {
      return "";
    }
    std::string content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    return sha256_hex(content);
  }

  static std::string generate_lanelet_file_hash(const std::string & data)
  {
    return data.empty() ? "" : sha256_hex(data);
  }

  std::string generate_pcd_file_hash(const std::string & path) const
  {
    const fs::path map_path(path);
    if (fs::is_regular_file(map_path)) {
      if (map_path.extension() != ".pcd" && map_path.extension() != ".PCD") {
        RCLCPP_ERROR(node_->get_logger(), "[%s] is not pcd file", path.c_str());
        return "";
      }
      return sha256_file_hex(path);
    }

    if (fs::is_directory(map_path)) {
      EVP_MD_CTX * context = EVP_MD_CTX_new();
      EVP_DigestInit_ex(context, EVP_sha256(), nullptr);
      bool has_pcd = false;
      std::vector<fs::path> pcd_files;
      for (const auto & entry : fs::directory_iterator(map_path)) {
        if (entry.path().extension() == ".pcd" || entry.path().extension() == ".PCD") {
          pcd_files.push_back(entry.path());
        }
      }
      std::sort(pcd_files.begin(), pcd_files.end());
      for (const auto & pcd_file : pcd_files) {
        std::ifstream file(pcd_file, std::ios::binary);
        if (!file) {
          RCLCPP_ERROR(node_->get_logger(), "Failed to open %s", pcd_file.c_str());
          EVP_MD_CTX_free(context);
          return "";
        }
        std::string content(
          (std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
        EVP_DigestUpdate(context, content.data(), content.size());
        has_pcd = true;
      }
      if (!has_pcd) {
        RCLCPP_ERROR(node_->get_logger(), "there are no pcd files in [%s]", path.c_str());
        EVP_MD_CTX_free(context);
        return "";
      }
      unsigned char hash[EVP_MAX_MD_SIZE];
      unsigned int hash_len = 0;
      EVP_DigestFinal_ex(context, hash, &hash_len);
      EVP_MD_CTX_free(context);
      std::ostringstream oss;
      for (unsigned int i = 0; i < hash_len; ++i) {
        oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(hash[i]);
      }
      return oss.str();
    }

    RCLCPP_ERROR(node_->get_logger(), "[%s] is neither file nor directory", path.c_str());
    return "";
  }

  void on_get_lanelet_xml(
    const tier4_external_api_msgs::srv::GetTextFile::Request::SharedPtr /*request*/,
    const tier4_external_api_msgs::srv::GetTextFile::Response::SharedPtr response) const
  {
    response->status.code = tier4_external_api_msgs::msg::ResponseStatus::SUCCESS;
    response->file.text = lanelet_text_;
  }
};

}  // namespace autoware::map_loader::plugin

PLUGINLIB_EXPORT_CLASS(
  autoware::map_loader::plugin::MapHashGeneratorPlugin,
  autoware::map_loader::plugin::MapLoaderPluginBase)
