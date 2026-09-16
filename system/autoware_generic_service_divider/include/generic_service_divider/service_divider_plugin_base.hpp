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

#ifndef GENERIC_SERVICE_DIVIDER__SERVICE_DIVIDER_PLUGIN_BASE_HPP_
#define GENERIC_SERVICE_DIVIDER__SERVICE_DIVIDER_PLUGIN_BASE_HPP_

#include "generic_service_divider/generic_client.hpp"
#include "generic_service_divider/generic_service.hpp"
#include "generic_service_divider/output_service_config.hpp"
#include "rclcpp/rclcpp.hpp"

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace generic_service_divider
{

class ServiceDividerPluginBase
{
public:
  struct StartupDiagnosticInfo
  {
    bool input_service_started{false};
    std::string input_service_name;
    std::size_t ready_output_service_count{0};
    std::size_t total_output_service_count{0};
    std::vector<std::string> waiting_output_services;
  };

  virtual ~ServiceDividerPluginBase() = default;

  // --- Plugin must override these ---

  /// Read parameters from the node to configure the plugin.
  virtual void initialize(rclcpp::Node::SharedPtr node) = 0;

  /// Service type string (e.g. "autoware_system_msgs/srv/ChangeOperationMode").
  virtual std::string service_type() const = 0;

  /// Input service name to advertise.
  virtual std::string input_service_name() const = 0;

  /// Output service configurations.
  virtual std::vector<OutputServiceConfig> output_services() const = 0;

  /// Check whether a raw response is considered successful (type-specific).
  virtual bool is_response_success(const void * response) const = 0;

  /// Build an error response with the given message (type-specific).
  virtual std::shared_ptr<void> create_error_response(const std::string & message) const = 0;

  /// Optional request formatter for detailed logs.
  virtual std::string format_request(const void *) const { return ""; }

  /// Optional response formatter for detailed logs.
  virtual std::string format_response(const void *) const { return ""; }

  // --- Provided by base class ---

  /// Create GenericService / GenericClients from the plugin's configuration.
  void setup_service_division();

  /// Startup status for diagnostics.
  StartupDiagnosticInfo get_startup_diagnostic_info();

protected:
  rclcpp::Node::SharedPtr node_;

private:
  void try_start_input_service();

  /// Names of the output services whose servers are not available yet.
  /// Caller must hold `service_start_mutex_`.
  std::vector<std::string> collect_not_ready_output_services() const;

  /// Create the input service and stop the retry timer.
  /// Caller must hold `service_start_mutex_`.
  void advertise_input_service();

  struct OutputClientEntry
  {
    std::shared_ptr<GenericClient> client;
    OutputServiceConfig config;
    rclcpp::CallbackGroup::SharedPtr callback_group;
  };

  struct PendingDivision
  {
    std::shared_ptr<rmw_request_id_t> request_header;
    std::shared_ptr<GenericService> service;
    std::map<std::string, std::shared_ptr<void>> responses;
    std::map<std::string, bool> completed;
    std::map<std::string, bool> timed_out;
    int awaiting_count{0};
    std::mutex mutex;
    std::vector<rclcpp::TimerBase::SharedPtr> timeout_timers;
  };

  struct DivisionOutcome
  {
    bool all_success{true};
    std::string primary_name;
    std::shared_ptr<void> primary_response;
  };

  void handle_request(
    std::shared_ptr<GenericService> service, std::shared_ptr<rmw_request_id_t> request_header,
    std::shared_ptr<void> request);

  /// Register a new pending division and return the id used in logs.
  int64_t register_pending_division(const std::shared_ptr<PendingDivision> & pending);

  /// Send the request to a single output service and arm its timeout timer.
  void forward_request(
    OutputClientEntry & entry, const std::shared_ptr<PendingDivision> & pending, int64_t pending_id,
    const std::shared_ptr<void> & request);

  /// Mark an output service as completed. Returns false if it was already completed,
  /// which means the caller lost the race against the timeout (or the response).
  bool mark_output_completed(
    const std::shared_ptr<PendingDivision> & pending, const std::string & name, bool timed_out,
    std::shared_ptr<void> response);

  void try_finalize_response(std::shared_ptr<PendingDivision> pending);

  /// Aggregate the collected responses. Caller must hold `pending->mutex`.
  DivisionOutcome evaluate_outputs(const std::shared_ptr<PendingDivision> & pending) const;

  /// Reply to the input service caller according to `outcome`.
  /// Caller must hold `pending->mutex`.
  void send_final_response(
    const std::shared_ptr<PendingDivision> & pending, const DivisionOutcome & outcome);

  void erase_pending_division(const std::shared_ptr<PendingDivision> & pending);

  std::shared_ptr<GenericService> input_service_;
  std::vector<OutputClientEntry> output_clients_;
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  rclcpp::TimerBase::SharedPtr server_wait_timer_;
  std::mutex service_start_mutex_;
  bool input_service_started_{false};

  std::mutex pending_map_mutex_;
  std::map<int64_t, std::shared_ptr<PendingDivision>> pending_divisions_;
  int64_t next_pending_id_{0};
};

}  // namespace generic_service_divider

#endif  // GENERIC_SERVICE_DIVIDER__SERVICE_DIVIDER_PLUGIN_BASE_HPP_
