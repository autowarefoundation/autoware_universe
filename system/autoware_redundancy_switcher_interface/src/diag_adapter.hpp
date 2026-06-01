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
#ifndef DIAG_ADAPTER_HPP_
#define DIAG_ADAPTER_HPP_

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <redundancy_switcher_interface/ir/domain_types.hpp>
#include <redundancy_switcher_interface/plugin/i_adapter_plugin.hpp>

#include <memory>
#include <mutex>
#include <optional>

namespace autoware::redundancy_switcher
{

/**
 * @brief Built-in adapter that publishes aggregated status diagnostics.
 *
 * Responsibilities:
 *   - Receives UpdateStatusDiagCommand and triggers the diagnostic_updater.
 *   - Aggregates each field of DomainSnapshot (SwitcherSignals, AutowareReady,
 *     VelocityStatus, ControlMode) and publishes them as redundancy_switcher_interface/status.
 *   - Emits ERROR if the transitional state (is_stable, is_self_interrupted, and is_faulted
 *     all false) persists longer than diag.transitional_timeout_milli.
 *   - Architecture-agnostic; does not depend on a specific Switcher implementation or ECU architecture.
 *   - Does not submit any events to the EventGateway (output-only).
 */
class DiagAdapter : public IAdapterPlugin
{
public:
  DiagAdapter() = default;
  ~DiagAdapter() override = default;

  void initialize(rclcpp::Node * node, std::shared_ptr<EventGateway> gateway) override;
  void execute(const OutputCommand & command) override;

private:
  void update_status(diagnostic_updater::DiagnosticStatusWrapper & stat);

  rclcpp::Node * node_{nullptr};
  std::shared_ptr<EventGateway> gateway_;
  std::unique_ptr<diagnostic_updater::Updater> updater_;
  double transitional_timeout_milli_;
  std::optional<double> transitional_start_ms_;  // monotonic ms; nullopt = not in transitional
  mutable std::mutex updater_mutex_;
  mutable std::mutex transition_mutex_;
};

}  // namespace autoware::redundancy_switcher
#endif  // DIAG_ADAPTER_HPP_
