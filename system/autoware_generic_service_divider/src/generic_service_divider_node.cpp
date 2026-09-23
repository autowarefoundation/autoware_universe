// Copyright 2025 TIER IV, Inc.
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

#include "generic_service_divider/service_divider_plugin_base.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcpputils/join.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace generic_service_divider
{

class GenericServiceDividerNode : public rclcpp::Node
{
public:
  explicit GenericServiceDividerNode(const rclcpp::NodeOptions & options)
  : Node("generic_service_divider", options),
    diag_updater_(this),
    plugin_loader_(
      "autoware_generic_service_divider", "generic_service_divider::ServiceDividerPluginBase")
  {
    diag_updater_.setHardwareID("generic_service_divider");
    diag_updater_.add(
      "service_startup_readiness", this,
      &GenericServiceDividerNode::check_service_startup_readiness);

    const auto plugin_names =
      declare_parameter<std::vector<std::string>>("plugins", std::vector<std::string>{});

    if (plugin_names.empty()) {
      RCLCPP_WARN(get_logger(), "No plugins configured. Node will do nothing.");
      diag_timer_ =
        create_wall_timer(std::chrono::seconds(1), [this]() { diag_updater_.force_update(); });
      return;
    }

    auto this_shared = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});

    for (const auto & name : plugin_names) {
      RCLCPP_INFO(get_logger(), "Loading plugin: %s", name.c_str());
      try {
        auto plugin = plugin_loader_.createSharedInstance(name);
        plugin->initialize(this_shared);
        plugin->setup_service_division();
        plugins_.push_back(plugin);
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "Failed to load plugin '%s': %s", name.c_str(), e.what());
      }
    }

    RCLCPP_INFO(get_logger(), "Loaded %zu service divider plugin(s)", plugins_.size());

    diag_timer_ =
      create_wall_timer(std::chrono::seconds(1), [this]() { diag_updater_.force_update(); });
    diag_updater_.force_update();
  }

private:
  struct ReadinessSummary
  {
    bool all_ready{true};
    std::size_t total_ready{0};
    std::size_t total_outputs{0};
    std::vector<std::string> waiting_records;
  };

  void check_service_startup_readiness(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    if (plugins_.empty()) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "No plugins configured");
      stat.add("plugin_count", 0);
      return;
    }

    const auto summary = collect_readiness_summary(stat);

    stat.add("plugin_count", static_cast<int>(plugins_.size()));
    stat.addf("output_services_ready", "%zu/%zu", summary.total_ready, summary.total_outputs);
    stat.add(
      "waiting_output_services", summary.waiting_records.empty()
                                   ? std::string("none")
                                   : rcpputils::join(summary.waiting_records, " | "));

    if (summary.all_ready) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "All service checks completed");
    } else {
      stat.summary(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "Waiting for output services before input service advertisement");
    }
  }

  /// Aggregate every plugin's startup status, adding the per-plugin entries to `stat`.
  ReadinessSummary collect_readiness_summary(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    ReadinessSummary summary;

    for (const auto & plugin : plugins_) {
      const auto info = plugin->get_startup_diagnostic_info();
      summary.total_ready += info.ready_output_service_count;
      summary.total_outputs += info.total_output_service_count;
      summary.all_ready = summary.all_ready && info.input_service_started;

      if (!info.waiting_output_services.empty()) {
        summary.waiting_records.push_back(
          info.input_service_name + " -> [" + rcpputils::join(info.waiting_output_services, ", ") +
          "]");
      }

      stat.add(
        "input_service." + info.input_service_name,
        info.input_service_started ? "ready" : "waiting");
    }

    return summary;
  }

  diagnostic_updater::Updater diag_updater_;
  rclcpp::TimerBase::SharedPtr diag_timer_;
  pluginlib::ClassLoader<ServiceDividerPluginBase> plugin_loader_;
  std::vector<std::shared_ptr<ServiceDividerPluginBase>> plugins_;
};

}  // namespace generic_service_divider

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(generic_service_divider::GenericServiceDividerNode)
