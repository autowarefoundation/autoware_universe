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

#include "autoware/map_loader/lanelet2_map_loader_node.hpp"
#include "autoware/map_loader/map_loader_plugin_base.hpp"

#include <autoware/component_interface_specs/map.hpp>
#include <autoware_lanelet2_extension/io/autoware_osm_parser.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_lanelet2_extension/version.hpp>
#include <autoware_utils/ros/parameter.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <stdexcept>
#include <string>

namespace autoware::map_loader::plugin
{

class Lanelet2MapLoaderPlugin : public MapLoaderPluginBase
{
public:
  void initialize(
    const std::string & /*name*/, rclcpp::Node * node, MapLoaderData & /*data*/) override
  {
    node_ = node;
    using autoware_utils_rclcpp::get_or_declare_parameter;
    allow_unsupported_version_ = get_or_declare_parameter<bool>(*node, "allow_unsupported_version");
    lanelet2_map_path_ = get_or_declare_parameter<std::string>(*node, "lanelet2_map_path");
    center_line_resolution_ = get_or_declare_parameter<double>(*node, "center_line_resolution");
    use_waypoints_ = get_or_declare_parameter<bool>(*node, "use_waypoints");

    using VectorMap = autoware::component_interface_specs::map::VectorMap;
    pub_map_bin_ =
      node->create_publisher<VectorMap::Message>(VectorMap::name, rclcpp::QoS{1}.transient_local());
  }

  void on_startup(MapLoaderData & data) override
  {
    if (!data.projector_info.has_value()) {
      throw std::runtime_error("Map projector info is not available for lanelet2 map loader.");
    }

    const auto & projector_info = data.projector_info.value();
    const auto map = Lanelet2MapLoaderNode::load_map(lanelet2_map_path_, projector_info);
    if (!map) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to load lanelet2_map. Not published.");
      return;
    }

    std::string format_version{"null"};
    std::string map_version{""};
    lanelet::io_handlers::AutowareOsmParser::parseVersions(
      lanelet2_map_path_, &format_version, &map_version);
    if (format_version == "null" || format_version.empty() || !isdigit(format_version[0])) {
      RCLCPP_WARN(
        node_->get_logger(),
        "%s has no format_version(null) or non semver-style format_version(%s) information",
        lanelet2_map_path_.c_str(), format_version.c_str());
      if (!allow_unsupported_version_) {
        throw std::invalid_argument(
          "allow_unsupported_version is false, so stop loading lanelet map");
      }
    } else if (const auto map_major_ver_opt =
                 lanelet::io_handlers::parseMajorVersion(format_version);
               map_major_ver_opt.has_value()) {
      const auto map_major_ver = map_major_ver_opt.value();
      if (map_major_ver > static_cast<uint64_t>(lanelet::autoware::version)) {
        RCLCPP_WARN(
          node_->get_logger(),
          "format_version(%ld) of the provided map(%s) is larger than the supported version(%ld)",
          map_major_ver, lanelet2_map_path_.c_str(),
          static_cast<uint64_t>(lanelet::autoware::version));
        if (!allow_unsupported_version_) {
          throw std::invalid_argument(
            "allow_unsupported_version is false, so stop loading lanelet map");
        }
      }
    }
    RCLCPP_INFO(node_->get_logger(), "Loaded map format_version: %s", format_version.c_str());

    if (use_waypoints_) {
      lanelet::utils::overwriteLaneletsCenterlineWithWaypoints(map, center_line_resolution_, false);
    } else {
      lanelet::utils::overwriteLaneletsCenterline(map, center_line_resolution_, false);
    }

    const auto map_bin_msg =
      Lanelet2MapLoaderNode::create_map_bin_msg(map, lanelet2_map_path_, node_->now());
    pub_map_bin_->publish(map_bin_msg);

    data.lanelet_map_ptr = map;
    data.vector_map_msg = map_bin_msg;

    RCLCPP_INFO(node_->get_logger(), "Succeeded to load lanelet2_map. Map is published.");
  }

private:
  rclcpp::Node * node_{nullptr};
  bool allow_unsupported_version_{false};
  std::string lanelet2_map_path_;
  double center_line_resolution_{5.0};
  bool use_waypoints_{true};
  rclcpp::Publisher<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr pub_map_bin_;
};

}  // namespace autoware::map_loader::plugin

PLUGINLIB_EXPORT_CLASS(
  autoware::map_loader::plugin::Lanelet2MapLoaderPlugin,
  autoware::map_loader::plugin::MapLoaderPluginBase)
