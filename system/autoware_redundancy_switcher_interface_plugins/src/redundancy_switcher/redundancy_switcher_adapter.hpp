//  Copyright 2025 The Autoware Contributors
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
#ifndef REDUNDANCY_SWITCHER_ADAPTER_HPP_
#define REDUNDANCY_SWITCHER_ADAPTER_HPP_

#include "uds_receiver.hpp"
#include "uds_sender.hpp"
#include "uds_types.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <redundancy_switcher_interface/plugin/event_gateway.hpp>
#include <redundancy_switcher_interface/plugin/i_adapter_plugin.hpp>

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <atomic>
#include <unordered_set>

namespace autoware::redundancy_switcher
{

class RedundancySwitcherAdapter : public IAdapterPlugin
{
public:
  RedundancySwitcherAdapter() = default;
  ~RedundancySwitcherAdapter() override;

  void initialize(rclcpp::Node * node, std::shared_ptr<EventGateway> gateway) override;
  void execute(const OutputCommand & command) override;

private:
  void on_switcher_status(const ElectionStatus & status);
  void send_election_request(const ElectionRequest & request);
  void check_election_status_timeout();
  void check_switcher_connection();
  void uds_receive_loop();
  bool no_data(
    const std::optional<ElectionStatus> & status,
    diagnostic_updater::DiagnosticStatusWrapper & stat) const;

public:
  // Returns true for INITIALIZING(0) or IN_ELECTION(3): connection flags are unstable
  // in these states and fault_mutex_ may still hold data from a prior state.
  static bool is_transitional_state(uint8_t node_state);
  static SwitcherSignals to_switcher_signals(uint8_t node_state);
  static ActiveControlUnit to_active_control_unit(uint8_t path_info);

private:
  static std::string node_state_to_string(uint8_t node_state);
  static std::string path_info_to_string(uint8_t path_info);

  void update_main_ecu_fault_diag(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void update_sub_ecu_fault_diag(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void update_main_vcu_fault_diag(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void update_sub_vcu_fault_diag(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void update_main_ecu_to_sub_ecu_link_fault_diag(
    diagnostic_updater::DiagnosticStatusWrapper & stat);
  void update_main_ecu_to_main_vcu_link_fault_diag(
    diagnostic_updater::DiagnosticStatusWrapper & stat);
  void update_main_ecu_to_sub_vcu_link_fault_diag(
    diagnostic_updater::DiagnosticStatusWrapper & stat);
  void update_sub_ecu_to_main_vcu_link_fault_diag(
    diagnostic_updater::DiagnosticStatusWrapper & stat);
  void update_sub_ecu_to_sub_vcu_link_fault_diag(
    diagnostic_updater::DiagnosticStatusWrapper & stat);
  void update_main_vcu_to_sub_vcu_link_fault_diag(
    diagnostic_updater::DiagnosticStatusWrapper & stat);

  rclcpp::Node * node_{nullptr};
  std::shared_ptr<EventGateway> gateway_;

  // UDS: Interface → Switcher (command send)
  std::unique_ptr<UdsSender<ElectionRequest>> uds_sender_;
  // UDS: Switcher → Interface (status receive)
  std::unique_ptr<UdsReceiver<ElectionStatus>> uds_receiver_;

  // Parameters
  double election_status_timeout_milli_{1000.0};
  bool is_main_ecu_{true};

  // State
  std::optional<ElectionStatus> last_election_status_;
  std::optional<rclcpp::Time> stamp_election_status_;
  std::optional<AutowareReady> autoware_ready_;  // cached via UpdateAutowareReadyCommand
  bool another_ecu_availability_timeout_{false};  // cached via UpdateAnotherEcuAvailabilityTimeoutCommand
  std::unordered_set<std::string> node_fault_points_;
  std::unordered_set<std::string> link_fault_points_;
  bool uds_connection_established_{false};  // true once the UDS connection is established

  // Thread management
  std::thread uds_receiver_thread_;
  std::atomic<bool> is_uds_receiver_running_{false};
  mutable std::mutex status_mutex_;  // last_election_status_, stamp, uds connection state
  mutable std::mutex policy_mutex_;  // cached state updated from execute()
  mutable std::mutex fault_mutex_;   // cached diagnostic computation results

  // Timers
  rclcpp::TimerBase::SharedPtr timer_;

  // Diagnostics
  std::unique_ptr<diagnostic_updater::Updater> updater_;
};

}  // namespace autoware::redundancy_switcher
#endif  // REDUNDANCY_SWITCHER_ADAPTER_HPP_
